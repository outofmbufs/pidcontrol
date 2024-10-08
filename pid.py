# MIT License
#
# Copyright (c) 2024 Neil Webber
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import itertools
import contextlib
import copy
import re
from collections import deque, ChainMap, Counter


class PID:
    """Simple PID control."""

    def __init__(self, /, *, Kp=0, Ki=0, Kd=0, dt=None):
        """Create a PID controller with the given parameters.

        Kp, Ki, Kd        -- weights for P/I/D control signals
        dt                -- [OPTIONAL] default interval
        """

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

        # initial conditions always start pv=0, setpoint=0 but applications
        # can (and SHOULD) call initial_conditions themselves as needed.
        self.initial_conditions(pv=0, setpoint=0)

    def initial_conditions(self, /, *, pv=None, setpoint=None):
        """Establish initial conditions -- resets state accordingly.

        Use this rather than bashing .pv and .setpoint directly, because
        this will also reset relevant state variables and will avoid
        any "kick" from instantaneous pv/setpoint changes.
        """

        if setpoint is not None:
            self.setpoint = setpoint

        if pv is not None:
            self.integration = 0        # For I: the current integral
            self.previous_pv = pv       # For D: previous process variable
            self.pv = pv                # observed process variable value

        self.last_pid = (0, 0, 0)

    def pid(self, pv, dt=None):
        """Return the new commanded control value for the most recent pv/dt.

        If dt is omitted the previous dt will be re-used.

        Startup sequence should be something like:
             z = PID(Kp=foo, Ki=bar, Kd=baz)
             z.initial_conditions(pv=xxx, setpoint=sss)
             (optionally) wait dt seconds    # ok to not wait first time
             u = z.pid(newpv, dt)
        """
        self.pv = pv
        if dt is not None:
            self.dt = dt
        return self._calculate()

    def _calculate(self):
        """Return control value ('u') for current state."""
        # NOTE: individually broken out into functions for reuse in PIDPlus
        e = self._error()
        p = self._proportional(e)
        i = self._integral(e)
        d = self._derivative(e)
        self.last_pid = (p, i, d)
        return self._u(p, i, d)

    def _error(self):
        """Return the (unweighted) error calculation."""
        return self.setpoint - self.pv

    def _proportional(self, e):
        """Return the (unweighted) proportional error term."""
        return e

    def _integral(self, e):
        """Accumulate and return the (unweighted) integral error term."""
        self.integration += (e * self.dt)
        return self.integration

    def _derivative(self, e):
        """Return the (unweighted) derivative term, using PV."""

        # As noted in wikipedia (bleh):
        #   In most commercial control systems, derivative action is based
        #   on process variable rather than error. That is, a change in the
        #   setpoint does not affect the derivative action
        #
        # The same is noted in
        #       https://www.ni.com/en/shop/labview/pid-theory-explained.html
        # without discussion, where they merely assert:
        #   The derivative component causes the output to decrease if the
        #   process variable is increasing rapidly. The derivative response
        #   is proportional to the rate of change of the process variable.
        #
        # Thus this uses delta-pv for the derivative term.
        # See D_DeltaE (a PIDModifier) to force use of delta-e instead.
        #

        if self.dt == 0:
            raise ValueError(f"Cannot compute D term with zero dt")

        dpv = (self.previous_pv - self.pv) / self.dt
        self.previous_pv = self.pv
        return dpv

    def _u(self, p, i, d):
        return (p * self.Kp) + (i * self.Ki) + (d * self.Kd)

    def __repr__(self):
        s = self.__class__.__name__ + "("
        pre = ""
        # NOTE: If __repr__ requested in PIDHookAttached processing, it
        #       is before Kp/etc exist. That's why 'dflt' is in getattr()
        for a, dflt in (('Kp', 0), ('Ki', 0), ('Kd', 0)):
            v = getattr(self, a, dflt)
            if v != dflt:
                s += f"{pre}{a}={v!r}"
                pre = ", "
        return s + ")"


#
# A PIDPlus is a PID that supports "modifiers" (PIDModifier subclasses).
# PIDModifiers can enhance/alter the base PID calculations and outputs.
# They can provide setpoint change ramping; integration windup limits, etc.
#
# See PIDHook and PIDModifier class descriptions for details on how
# the behavior "modifiers" work.
#
class PIDPlus(PID):
    """PID with the ability to have modifiers like setpoint ramps, etc"""

    def __init__(self, *args, modifiers=None, **kwargs):
        """Takes one additional argument beyond the standard PID arguments:
        modifiers         -- a single PIDModifier or a sequence of them.
                             Optional algorithm modifications/enhancements.
                             These are features like integration windup,
                             setpoint ramping, etc.
        """

        if modifiers is None:
            self.modifiers = tuple()    # degenerate case; essentially PID()
        else:
            # force modifiers to be a sequence (tuple), possibly zero length.
            # Note: modifiers can be a naked PIDModifier which is auto
            #       converted into a tuple of length 1.
            try:
                self.modifiers = tuple(modifiers)
            except TypeError:
                self.modifiers = (modifiers,)

        # Tracks nested notification level (see notify()/_notify())
        self.nn_level = 0

        # let each modifier know it has been attached to this pid
        self.notify(PIDHookAttached())

        # Must be after establishing modifiers, because of PIDHookInitial..
        # event that will be triggered in initial_conditions()
        super().__init__(*args, **kwargs)

    def initial_conditions(self, *args, **kwargs):

        # suppress the setpoint changes that could be generated by this
        saved_mods = self.modifiers
        try:
            self.modifiers = []
            super().initial_conditions(*args, **kwargs)
        finally:
            self.modifiers = saved_mods

        # NOTE: It is important that modifiers with state have a handler
        #       for this event. For example, setpoint ramping could have
        #       been triggered (when the base class changes the setpoint)
        #       and that should be reset by this event.
        self.notify(PIDHookInitialConditions(*args, **kwargs))

    # .setpoint becomes a property so PIDModifiers can be notified of changes
    @property
    def setpoint(self):
        return self._setpoint

    @setpoint.setter
    def setpoint(self, v):
        # this dance avoids generating spurious notification during __init__
        try:
            sp_from = self._setpoint
        except AttributeError:        # happens during __init__ only
            sp_from = self._setpoint = v

        if v == sp_from:
            return                    # no notifications on no-change 'changes'

        # the notification protocol
        event = self.notify(PIDHookSetpointChange(sp_from=sp_from, sp_to=v))
        self._setpoint = v if event.sp is None else event.sp

    def _calculate(self):
        """Run PIDModifier protocol and return control value ('u')."""

        # There are three notification points
        #    BaseTerms:
        #        An opportunity to totally override (prevent) the built-in
        #        computation of e/p/i/d/u values.
        #
        #    ModifyTerms:
        #        At this point 'e' has been determined (and is not relevant
        #        to any further built-in calculations) and p/i/d/u values
        #        have been set either by a BaseTerms event handler or
        #        the built-in logic. This is an opportunity to further
        #        modify those now-calculated p/i/d/u values.
        #
        #     CalculateU:
        #        Last opportunity to bash 'u' (only).
        #

        # FIRST PIDHookEvent: BaseTerms
        # Run all the BaseTerms events which may (or may not) establish
        # some of the process control values (e/p/i/d/u).
        cx = self.notify(PIDHookBaseTerms(dt=self.dt))

        # Anything not supplied by a BaseTerms modifier is calculated
        # the standard way here:
        if cx.e is None:
            cx.e = self._error()
        for attr, f in (('p', self._proportional),
                        ('i', self._integral),
                        ('d', self._derivative)):
            if getattr(cx, attr) is None:
                setattr(cx, attr, f(cx.e))

        # NOTE: 'e' is not used further after this point, unless a modifier
        #       looks at it specifically.

        # SECOND PIDHookEvent: ModifyTerms
        # This is where most modifiers do their work, given the "raw"
        # p/i/d/ values (though they potentially were supplied by
        # a modifier, not by the normal calculation).
        cx = self.notify(PIDHookModifyTerms(**cx.attrs()))

        # Generally 'u' calculation should be overridden in a CalculateU.
        # See, for example, BangBang. But allow for it in ModifyTerms by
        # testing for it. By far the common case is cx.u is still None here.
        if cx.u is None:
            cx.u = self._u(cx.p, cx.i, cx.d)

        # At this point p/i/d are fixed (note: read-only in CalculateU).
        # Record last_pid accordingly. Caution: Because modifiers can
        # perform arbitrary calculations to provide a 'u' value, the
        # relevance of last_pid is less clear in a PIDPlus. Nevertheless:
        self.last_pid = (cx.p, cx.i, cx.d)

        # THIRD (final) PIDHookEvent: CalculateU
        # This is where a modifier such as BangBang can do violence to 'u'
        # Note: attrs from the ModifyTerms are cloned to the CalculateU
        cx = self.notify(PIDHookCalculateU(**cx.attrs()))

        # Whatever 'u' came through all that ... that's the result!
        return cx.u

    def notify(self, event, /, *, modifiers=None):
        """Invoke the notification handler for (by default) all modifiers.
        An explicit (usually tail-end subset) modifiers list can be given.

        For notational convenience, returns event, for 1-liners like:
            event = self.notify(PIDHookSomething(foo))
        when the returned event is needed even after the notify()
        """

        # This dance allows EventPrint to indent nested event notifications
        self.nn_level += 1
        try:
            rv = self._notify(event, modifiers=modifiers)
        finally:
            self.nn_level -= 1
        return rv

    def _notify(self, event, /, *, modifiers=None):
        """The guts of notify(). (notify() itself just maintains nn_level)"""
        if modifiers is None:
            modifiers = self.modifiers

        # automate the setting of 'pid' as a convenience
        # NOTE: if this was copied (e.g., ModifyTerms from BaseTerms)
        #       then pid can already be there and cannot be set again.
        if not hasattr(event, 'pid'):
            event.pid = self

        for nth, m in enumerate(modifiers):
            h = getattr(m, event.handlername(), getattr(m, 'PH_default'))
            try:
                h(event)        # ... this calls m.PH_foo(event)
            except HookStop:
                # stop propagating; notify the REST of modifiers of THAT
                self.notify(
                    PIDHookHookStopped(
                        event=event,     # the event that was HookStop'd
                        stopper=m,       # whodunit
                        nth=nth,         # in case more than one 'm'
                        modifiers=modifiers
                    ), modifiers=modifiers[nth+1:])
                break
            except Exception as exc:
                # some modifier bailed out. It is unlikely to be helpful
                # to let other modifiers know, but... do it anyway
                self.notify(PIDHookFailure(
                        event=event,     # the event that caused exc
                        exc=exc,         # the exception
                        stopper=m,       # whodunit
                        nth=nth,         # in case more than one 'm'
                        modifiers=modifiers
                    ), modifiers=modifiers[nth+1:])
                raise
        return event            # notational convenience

    def __repr__(self):
        s = super().__repr__()
        if self.modifiers:
            # turn 'PIDPlus()' into 'PIDPlus('
            # turn 'PIDPlus(anything)' into 'PIDPlus(anything, )'
            s = s[:-1]
            if s[-1] != '(':
                s += ', '
            # maybe too cute but use the simplified form for 1 modifier
            if len(self.modifiers) == 1:
                s += f"modifiers={self.modifiers[0]}"
            elif self.modifiers:
                premod = "modifiers=("
                for m in self.modifiers:
                    s += f"{premod}{m}"
                    premod = ", "
                s += ")"
            s += ")"
        return s


# PIDModifier/PIDHook system.
#
# There are two class hierarchies:
#
#   PIDModifier subclasses
#
#       These contain the behavior implementations for specific features.
#       Each feature will be implemented in its own subclass of PIDModifier.
#       For example, class I_Windup(PIDModifier) contains the implementation
#       of the integral "windup" limit behavior.
#
#       Behavior-specific attributes (e.g., the limit value for 'windup') are
#       stored in these objects, and handlers for PIDHookEvents that need to
#       be received are defined in these (sub)classes (see "CONNECTIONS")
#
#   PIDHookEvent subclasses
#
#      These are "events" generated at known places within the control
#      calculations. They encapsulate event-specific variables and get
#      sent to interested PIDModifier subclasses for processing.
#
# CONNECTIONS
#
# Each specific PIDHookEvent defines a name of a method that a PIDModifier is
# expected to supply if that PIDModifier wants to receive that type of event.
#
# For example, setpoint modifications generate a PIDHookSetpointChange event.
# The NOTIFYHANDLER for PIDHookSetpointChange is 'PH_setpoint_change'.
# A PIDModifier that wishes to receive setpoint change events must provide
# a method with that name and it will automatically receive these events.
# To reduce the possibility of clashes with future modifier/event
# variables, by convention these handler attribute use a 'PH_' prefix.
#
# If a given PIDModifier does not need to see a particular event, it
# can simply not define a handler method for it and by default the event
# will be ignored. A special 'PH_default' method can also be provided;
# if that is present it will receive all events (unless there is also
# an event-specific handler, which then takes precedence). This is how,
# for example, the PIDHistory modifier simply receives all events and
# creates a record of them for analysis/debug.
#


# ----------------------------
# PIDHookEvent class hierarchy.
# ----------------------------

class _PIDHookEvent:
    """Base class for events that get generated within PIDPlus."""

    # subclasses can/should override these as appropriate
    DEFAULTED = {}
    READONLY = {'*'}

    @classmethod
    def handlername(cls):
        """Return deduced handler name for the event class."""
        try:
            # A subclass can explicitly set a name other than the
            # automatic name, if it wants to. Return that if it exists.
            return cls.NOTIFYHANDLER
        except AttributeError:
            # Nothing explicitly set; the auto naming works like this:
            #   PIDHookFooBar becomes PH_foo_bar
            # Essentially ^PIDHook becomes PH and any upper case
            # letter becomes an underbar and corresponding lower case.
            # Subclass names that don't start with PIDHook are not translated
            # at all; the class should have set NOTIFYHANDLER instead.
            hname = cls.__name__
            if not hname.startswith('PIDHook'):
                raise AttributeError(f"Cannot autoname from {hname}")
            cls.NOTIFYHANDLER = 'PH'
            for c in hname[7:]:
                if c.isupper():
                    cls.NOTIFYHANDLER += '_'
                cls.NOTIFYHANDLER += c.lower()
        return cls.NOTIFYHANDLER

    # Property/descriptor implementing read-only-ness
    class _ReadOnlyDescr:
        _PVTPREFIX = "_PVT_ReadOnly__"
        _WRITEONCE = object()

        # Factory for annointing PIDHookEvent classes with descriptors
        # for any attributes that are enforced as read-only.
        #    cls      -- will be (of course) this class
        #    attrname -- name of a (to be set up) read-only attribute
        #    value    -- its initial value
        #    obj      -- the underlying PIDHookEvent OBJECT (not class)
        #
        # The obj.__class__ (something like, e.g., PIDHookSetpointChange)
        # needs a data descriptor property created for this attrname, but
        # that property should only be created ONCE (per attrname, per class).
        # In contrast, individual object instances each need, of course,
        # their own instance variable (object attribute) for the underlying
        # readonly value; that attribute name is built using _PVTPREFIX to
        # distinguish it from ordinary (read/write) attrs.

        @classmethod
        def establish_property(cls, attrname, value=None, /, *, obj=None):
            """Factory for creating the property descriptors (dynamically)."""

            pidhook_class = obj.__class__   # e.g., PIDHookSetpointChange, etc

            # See if there is already a data descriptor (i.e., if this is
            # not the first time establish_property() has been called for
            # this attrname on this cls.
            try:
                descriptor = getattr(pidhook_class, attrname)
            except AttributeError:
                # this code hasn't run yet, so there is no descriptor for
                # this attrname in this pidhook_class. Make it.

                descriptor = cls()         # i.e., a _ReadOnlyDescr()

                # If "foo" is an attrname, something 'like' foo needs
                # to be stored in the object (not the class) as the
                # underlying read-only value for that instance.
                # The name is arbitrarily constructed this way and saved
                # ONCE in the descriptor for this attribute.
                descriptor._name = cls._PVTPREFIX + attrname
                descriptor._publicname = attrname
                # and this establishes the data descriptor on the class
                setattr(pidhook_class, attrname, descriptor)

            # now can just set the underlying attribute directly.
            setattr(obj, descriptor._name, value)

        # readonly attributes won't be in (generic) vars(), and their
        # underlying ("privatized") names will. So vars() is not
        # (very) helpful. Instead, _Readonly class provides an attrs()
        # method, which returns all the appropriate attribute names and
        # their values, including the readonly ones, and not any
        # of the privatized ("underlying") names.
        @classmethod
        def attrs(cls, obj):
            """Return a dictionary of attributes, finessing readonly stuff."""
            vd = {}
            for a, v in vars(obj).items():
                if a.startswith(cls._PVTPREFIX):
                    a = a[len(cls._PVTPREFIX):]
                vd[a] = v
            return vd

        def __get__(self, obj, objtype=None):
            if obj is None:
                return self
            v = getattr(obj, self._name)
            if v is self._WRITEONCE:
                raise AttributeError(
                    f"{self._publicname} hasn't been set yet")
            return v

        def __set__(self, obj, value):
            if getattr(obj, self._name) is self._WRITEONCE:
                setattr(obj, self._name, value)
            else:
                raise TypeError(
                    f"write to read-only attribute "
                    f"'{self._publicname}' not allowed")

        def __delete__(self, obj):
            raise TypeError(
                f"deletion of read-only attribute "
                f"'{self._publicname}' not allowed")

    def __init__(self, /, **kwargs):

        # Also, 'pid' is hardwired read-only
        try:
            readonly = self.READONLY | {'pid'}
        except TypeError:
            # raise something more helpful if READONLY is not a set
            raise ValueError(
                f"READONLY must be a set, not '{self.READONLY}'") from None

        # NOTE: READONLY={'*'} only affects attributes established
        #       by __init__() either as explicit kwargs or via DEFAULTED.
        #       This is by design.
        for k, v in ChainMap(kwargs, self.DEFAULTED).items():
            if {k, '*'} & readonly:
                self._ReadOnlyDescr.establish_property(k, v, obj=self)
            else:
                setattr(self, k, v)

        # attrs in READONLY by name but not yet supplied become write-once
        for k in readonly:
            if k != '*' and not hasattr(self, k):
                self._ReadOnlyDescr.establish_property(
                    k, self._ReadOnlyDescr._WRITEONCE, obj=self)

    def attrs(self):
        """Built-in vars returns gibberish for readonly's; this doesn't."""
        return {a: v for a, v in self._ReadOnlyDescr.attrs(self).items()}

    def __repr__(self):
        s = self.__class__.__name__ + "("
        pre = ""
        for a, v in self.attrs().items():
            s += f"{pre}{a}={v!r}"
            pre = ", "
        return s + ")"

    # the str form is meant to be slightly more human-friendly:
    #    * The .pid object is omitted entirely
    #    * Anything whose __str__ matches r'<[a-z_]*\.[^ ]* object at'
    #      becomes <.__class__.__name__>
    def __str__(self):
        s = self.__class__.__name__ + "("
        pre = ""
        post = ")"
        for a, v in self.attrs().items():
            if a == 'pid':
                continue
            vstr = str(v)
            if re.match(r'\<[a-z_]*\.[^ ]* object at', vstr):
                vstr = f"<{v.__class__.__name__}>"
            s += f"{pre}{a}={vstr}"
            pre = ", "
        return s + post


# PIDHookEvent .__init__() provides a framework so that typically the
# specific event types are just subclasses with a few class variables:
#
#    DEFAULTED -- dictionary of default attribute names/values that
#                 will be used (if no explicit values given in init)
#    READONLY  -- set() ... names of attributes that are read-only.
#                 The default value is {'*'} which means (almost) all
#                 attributes (see discussion below) will be read-only.
#
# A few notes on READONLY:
#  - If an attribute name is in READONLY but not supplied at __init__
#    time either as a kwarg or via DEFAULTED, then it becomes a
#    "write-once" attribute. The application can write a value to this
#    attribute later (once), but once written it cannot be changed.
#    Attempting to read an attribute that is in WRITEONCE state (i.e.,
#    hasn't been set yet) will raise AttributeError like any other
#    uninitialized attribute.
#
#  - Attribute 'pid' is hardwired read-only regardless of whether it is
#    in READONLY (explicitly as 'pid' or implicitly via '*').
#
# Notes on DEFAULTED:
#
#  - "DEFAULTED" is an alternate for a boilerplate __init__() that would
#    provide defaults via the argument signature. Subclasses can still do
#    it that way if desired; but the DEFAULTED mechanism makes trivial
#    cases trivial and vs boilerplate:
#         def __init__(self, *args, foo=17, bar='bozo', **kwargs):
#             super().__init__(*args, **kwargs)
#             self.foo = foo
#             self.bar = bar
#
# The name of the corresponding notify handler will be automatically
# inferred from the subclass name, IF it starts with 'PIDHook'.
# The name will be PH_foo where "foo" is everything that comes after
# PIDHook but capital letters will be turned into lower case and have
# an underbar preceding them. Thus: the automatic notification handler
# name for PIDHookFooBar is: PH_foo_bar
#
# To override the automatic name, explicitly set a NOTIFYHANDLER class attr.
#

class PIDHookAttached(_PIDHookEvent):
    pass


class PIDHookHookStopped(_PIDHookEvent):
    pass


class PIDHookFailure(_PIDHookEvent):
    pass


class PIDHookInitialConditions(_PIDHookEvent):
    DEFAULTED = dict(setpoint=None)


class PIDHookSetpointChange(_PIDHookEvent):
    DEFAULTED = dict(sp=None)
    READONLY = {'sp_from', 'sp_to'}     # sp will be read/write


# Base for BaseTerms, ModifyTerms, CalculateU. Establish defaults.
class _PIDHook_Calc(_PIDHookEvent):
    DEFAULTED = dict(e=None, p=None, i=None, d=None, u=None)


class PIDHookBaseTerms(_PIDHook_Calc):
    READONLY = {'dt'}


class PIDHookModifyTerms(_PIDHook_Calc):
    READONLY = {'e', 'dt'}


class PIDHookCalculateU(_PIDHook_Calc):
    READONLY = {'e', 'p', 'i', 'd', 'dt'}


# Any PIDModifier that wants to stop hook processing raises this:
class HookStop(Exception):
    pass


# ----------------------------
# PIDModifier class hierarchy.
# ----------------------------

class PIDModifier:
    """Base class for the 'modifiers' used in a PIDPlus."""

    # -------------------------- IMPORTANT -----------------------------
    # This base class MUST NOT have any explicit PH_foo handlers (other
    # than PH_default). If it does, they prevent a subclass PH_default
    # from seeing those events (which it would expect to see if the
    # subclass itself does not handle those events).
    # --------------------------------------------------------------------

    #
    # A subclass can set:
    #        PH_attached = PIDModifier.attached_once_check
    # to prevent object re-use across multiple pids. Or, of course, it
    # can simply invoke this from its own PH_attached if it wants.
    #
    def attached_once_check(self, event):
        """Disallow more than one PH_attached in any circumstance."""
        try:
            if self.__attached:      # effectively just "if hasattr"
                raise TypeError(
                    f"multiple attachment attempted: " +
                    f"'{self}', {self.__attached}")
        except AttributeError:
            self.__attached = event

    def PH_default(self, event):
        """Default (no-op) handler for unhandled event types"""
        pass


class PIDHistory(PIDModifier):
    """Look-back record, event counts."""

    DEFAULTSIZE = 1000

    def __init__(self, n=DEFAULTSIZE, *args, detail=False, **kwargs):
        """Counts events and records the last 'n' of them.

        If 'n' is None, the deque will be unbounded and store all events.
        CAUTION: The deque could grow enormously large. CAVEAT CODER.

        If 'detail' is True (default: False), the underlying pid attributes
        are logged as attribute 'pidinfo' added to each event.
        """

        super().__init__(*args, **kwargs)
        self.detail = detail
        self.eventcounts = Counter()
        self.resize(n, preserve=None)

    # this _default method gets all events and counts/logs them
    def PH_default(self, event):
        # Make a copy of the event for two reasons:
        #   1) If 'detail' is true, vars(pid) is added to it for logging
        #      (general principles: prefer to not modify 'event' that way)
        #
        #   2) If this PIDHistory is not the last in the modifier list then
        #      subsequent handlers might modify event, which will change this
        #      entry. Presumably
        #      of putting this earlier in the modifiers list was to capture
        #      the state NOW ... so that's another need to copy it.
        e2 = copy.copy(event)
        if self.detail:
            try:
                # copy vars() result for the usual "want static data" reason
                e2.pidinfo = dict(**vars(event.pid))
            except AttributeError:
                # probably never happens but allow for no .pid attr
                e2.pidinfo = None
        self.history.append(e2)
        self.eventcounts[e2.handlername()] += 1

    def resize(self, n, /, *, preserve='*'):
        """Choose a new maximum size ('n') for history.

        With just one argument, preserves tail of existing entries w.r.t. 'n'.
        If 'n' is None the new size is infinite. (BE CAREFUL WITH THIS)

        Keyword-only argument 'preserve' controls how much old history
        is preserved (but w.r.t. 'n'):
           preserve=None  -- No prior history is preserved
           preserve=x     -- 'x' must be an iterable and its items will
                             be entered into the new history (i.e., preserved
                             in accordance with 'n')

        See also: .since() which may be useful/appropriate for 'preserve'
        """

        if preserve == '*':
            preserve = self.history
        elif preserve is None:
            preserve = []
        self.history = deque(preserve, n)

    # Generate a list of events ignoring ones up to and including the given.
    def since(self, event=None):
        """Generate events from the history.

        Given an 'event' previously obtained from this method or from
        .history directly, generate subsequent events.

        If 'event' is not found in .history generate all .history events.
        With no arguments, or with event=None, generates all.
        """

        # implementation note: event=None is handled implicitly because
        # it is known that None will never be *in* .history
        gen = itertools.dropwhile(lambda x: x != event, self.history)
        try:
            _ = next(gen)                  # skip the event itself
        except StopIteration:
            gen = self.history             # event wasn't in there at all
        yield from gen

    def __repr__(self):
        s = f"{self.__class__.__name__}("
        pre = ""
        if self.history.maxlen != self.DEFAULTSIZE:
            s += f"{self.history.maxlen}"
            pre = ", "
        if self.detail:
            s += pre + "detail=True"
        return s + ")"


class EventPrint(PIDModifier):
    """Event printer, helpful for debug and learning."""
    def __init__(self, *args, prefix="", **kwargs):
        super().__init__(*args, **kwargs)
        self.prefix = prefix

    # override this to log, or do something else, if desired
    def printevent(self, event):
        indent = "  " * (event.pid.nn_level - 1)
        print(f"{self.prefix}{indent}{event}")

    def PH_default(self, event):
        # Although subclasses *could* just override PH_default, giving
        # them this other method (printevent) to override decouples them
        # from "knowing" PH_default is the only handler. Indeed, in the
        # past there was also a PH_attached handler Because Reasons even
        # though now it's gone.
        self.printevent(event)


class I_Windup(PIDModifier):
    """Enhances I control with "windup" limit."""

    def __init__(self, w, *args, **kwargs):
        """The integration value will be held to [-w, w] range.

        OPTIONALLY: if w is a tuple then the integration value will be
                    held to [a, b] range where:
                       a, b = sorted(w)
        """

        # see if w is a tuple (of length 2)
        try:
            a, b = sorted(w)
        except TypeError:
            a, b = -abs(w), abs(w)    # prevent negative shenanigans

        # make sure a and b are both numeric
        try:
            foo = a + 1
            bar = b + 2
        except TypeError:
            raise ValueError(f"invalid limit values: {w}") from None

        super().__init__(*args, **kwargs)
        self.w0 = a
        self.w1 = b

    def PH_modify_terms(self, event):
        clamped = max(self.w0, min(event.pid.integration, self.w1))
        event.i = event.pid.integration = clamped

    def __repr__(self):
        s = f"{self.__class__.__name__}("
        if abs(self.w0) == abs(self.w1):
            s += f"{abs(self.w0)}"
        else:
            s += f"({self.w0}, {self.w1})"
        return s + ")"


# This modifier allows freezing the integration term for a period of time
# based on external logic. As described, for example, in the wikipedia PID
# article section on algorithm modifications:
#
#     For example, a PID loop is used to control the temperature of an
#     electric resistance furnace where the system has stabilized. Now when
#     the door is opened and something cold is put into the furnace the
#     temperature drops below the setpoint. The integral function of the
#     controller tends to compensate for error by introducing another error
#     in the positive direction. This overshoot can be avoided by freezing
#     of the integral function after the opening of the door for the time
#     the control loop typically needs to reheat the furnace.
#
# See methods freeze() and unfreeze()
#
class I_Freeze(PIDModifier):
    """Adds freeze/unfreeze capability to integration term"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.unfreeze()

    # This is a stateful modifier and cannot be shared among PIDs
    PH_attached = PIDModifier.attached_once_check

    # The state method returns True if integration is frozen, False if not,
    # and is responsible for advancing any internal timers or other internal
    # freeze duration state as appropriate given the (PHBaseTerms) event.
    #
    # In this default version, state() implements the timed duration and also
    # of course responds to freeze/unfreeze requests.
    #
    # Subclasses can override this to implement more-elaborate rules for
    # when the integration should be frozen/unfrozen.
    def state(self, event):
        if self._duration == 0:
            return False
        elif self._duration is None:
            return self.frozen
        else:
            self._duration = max(0, self._duration - event.dt)
            self.frozen = self._duration > 0
            return True

    def freeze(self, /, *, duration=None):
        """Freeze the current integration term.

        By default freezes it until unfreeze() invoked.
        If given a duration, freezes it only for that long (in seconds).
        """
        self.frozen = True
        self._duration = duration

    def unfreeze(self):
        """Unfreeze integration; resume it at pre-freeze value."""
        self.frozen = False
        self._duration = 0

    def PH_base_terms(self, event):
        if self.state(event):
            event.i = event.pid.integration   # just keep the current term


class I_SetpointReset(PIDModifier):
    """Reset integration, with optional pause, when setpoint changes."""

    # This is a stateful modifier and cannot be shared among PIDs
    PH_attached = PIDModifier.attached_once_check

    def __init__(self, secs, *args, **kwargs):
        """Resets integration on setpoint change, and pause it 'secs'.

        NOTE: 'secs' can be zero, in which case a setpoint change resets the
              integration, but restarts it immediately (i.e, 0 second pause).
        """
        super().__init__(*args, **kwargs)
        self.integration_pause = secs
        self.pause_remaining = 0

    def PH_initial_conditions(self, event):
        self.pause_remaining = 0

    def PH_setpoint_change(self, event):
        event.pid.integration = 0
        self.pause_remaining = self.integration_pause

    def PH_base_terms(self, event):
        # When triggered (by a setpoint change):
        #   - Reset the integration. It is as if the controller is
        #     starting over afresh (for integration)
        #   - integration is paused while the controller settles into the
        #     new regime. This is a variation on "windup protection"
        #     which has a similar goal.
        if self.pause_remaining > 0:
            # max() for good housekeeping re: floating fuzz; force neg to 0
            self.pause_remaining = max(0, self.pause_remaining - event.dt)

            # Supplying .i here supplants _integral() (i.e., prevents it
            # from being invoked in _calculate). Therefore, this also
            # (correctly) prevents accumulation in pid.integration
            event.i = 0


class SetpointRamp(PIDModifier):
    """Add setpoint ramping (smoothing out setpoint changes) to a PID"""

    # This is a stateful modifier and cannot be shared among PIDs
    def PH_attached(self, event):
        self.attached_once_check(event)       # from the base class
        self.pid = event.pid

    def __init__(self, secs, *args, hiddenramp=False, threshold=0, **kwargs):
        """Smooth (i.e., ramp) setpoint changes over 'secs' seconds.

        If 'hiddenramp' is False (the default), the pid .setpoint attribute
        will change on each .pid() call during the ramp period. This means
        the ramping will be externally visible. This ensures that any other
        modifiers can "see" the ramping and will be notified of each change.

        If hiddenramp is True, the pid .setpoint attribute will appear to
        immediately be set to the target value, but the internal calculation
        of the error term (which feeds p/i/d terms) will be based on the
        (hidden) ramping setpoint value.
        """

        if secs < 0:                # NOTE: secs is allowed to be zero
            raise ValueError(f"ramp time (={secs}) must not be negative")
        if threshold < 0:
            raise ValueError(f"threshold (={threshold}) must not be negative")

        super().__init__(*args, **kwargs)
        self.pid = None            # won't know pid until PH_attached
        self._hiddenramp = hiddenramp
        self._ramptime = secs
        self._threshold = threshold
        self._noramp(setpoint=0)

    def _noramp(self, /, *, setpoint):
        """Returns state to 'no ramp in progress'."""
        self._target_sp = setpoint           # ending setpoint
        self._countdown = 0                  # time remaining in ramp
        self._set_real_setpoint(setpoint)    # observed setpoint

    def _ramped(self):
        """Return the current (ramped) setpoint based on _countdown"""
        pcttime = (self._ramptime - self._countdown) / self._ramptime
        totaldelta = (self._target_sp - self._start_sp)
        return self._clamper(self._start_sp + (totaldelta * pcttime))

    # property because if it changes the ramping may have to be adjusted
    @property
    def secs(self):
        return self._ramptime

    @secs.setter
    def secs(self, v):
        if self._countdown > 0:     # i.e., mid-ramp
            # This arbitrarily defines the semantic of a mid-ramp secs change
            # to mean: continue whatever ramping remains, with that remaining
            # ramp spread out over the new secs

            self._start_sp = self._ramped()   # i.e., ramp starts HERE ...

            # but if it's been set to zero, just go there RIGHT NOW
            # NOTE: being able to set secs to zero midramp requires knowing
            #       the .pid here (to... set the setpoint). Obviously, there
            #       is no "event" to get the pid out of here, so that's why
            #       PH_attached records self.pid (and, in sympathy, why all
            #       other places the pid is needed it comes from self.pid
            #       even when it is available more naturally as event.pid)
            if v == 0:
                self._set_real_setpoint(self._target_sp)

        self._ramptime = self._countdown = v

    # If the setpoint gets changed via an initial_conditions method
    # then it happens immediately (no ramping).
    def PH_initial_conditions(self, event):
        # If the setpoint is unchanged (carried forward) it is None...
        sp = self.pid.setpoint if event.setpoint is None else event.setpoint
        self._noramp(setpoint=sp)

    def PH_setpoint_change(self, event):
        # NO-OP conditions:
        #   ramp parameter zero,
        #   no change in setpoint
        if self._ramptime == 0 or event.sp_to == self._target_sp:
            return

        # if the change is within threshold, set immediate, cancel ramping
        if abs(event.sp_to - self.pid.setpoint) < self._threshold:
            self._noramp(setpoint=event.sp_to)
            return

        self._start_sp = event.sp_from
        self._target_sp = event.sp_to
        self._countdown = self._ramptime
        clampf = min if self._start_sp < self._target_sp else max
        self._clamper = lambda x: clampf(x, self._target_sp)
        if not self._hiddenramp:
            event.sp = self._ramped()     # make it ramp

    def _set_real_setpoint(self, v):
        """Set the real setpoint in the underlying pid, bypasing ramping."""

        # There are several ways can end up here before attachment, so ...
        if self.pid is None:
            return

        # if the ramp is being hidden there's nothing to do here; allowing
        # _set_real_setpoint() to be called anyway simplifies the rest of
        # code from having to test _hiddenramp
        if self._hiddenramp:
            return

        # This try/finally block is necessary because there could be OTHER
        # modifiers with PH_setpoint_change handlers, so anything in terms
        # of exceptions is possible. Of course, things are probably woefully
        # awry if that happens, but, try to maintain sanity here anyway.
        try:
            saved_ramp, self._ramptime = self._ramptime, 0
            self.pid.setpoint = v
        finally:
            self._ramptime = saved_ramp

    def PH_base_terms(self, event):
        # optimize the common case of no ramping in progress
        if self._countdown == 0:
            return

        # Test if the dt is the same or larger than the remaining ramp.
        # DO NOT test for _countdown reaching exactly zero because of
        # floating point fuzziness; this test works correctly if the countdown
        # is slightly "ahead" (i.e., slightly too small). If the fuzziness is
        # the other way (countdown slightly larger than correct) then this
        # tick gets the setpoint to 99.x% of the final value and there will
        # be one extra tick for the rest. No one cares; this is good enough.
        if self._countdown <= event.dt:
            # last tick; slam to _target_sp in case of any floating fuzz.
            self._set_real_setpoint(self._target_sp)
            self._countdown = 0
        else:
            # Still ramping ... This is the normal ramping case but could
            # potentially also be the "one extra" tick mentioned above
            # in which case pcttime will be very very close to 1.00 and
            # the next tick after this will trigger the above branch
            self._countdown -= event.dt

            # when the ramping setpoint is being hidden, this needs to
            # supply an alternate 'e' making use of the ramping setpoint
            # rather than the public .setpoint
            if self._hiddenramp:
                event.e = self._ramped() - self.pid.pv
            else:
                # otherwise: here is where the ramping becomes effective
                self._set_real_setpoint(self._ramped())

    def __repr__(self):
        s = f"{self.__class__.__name__}({self._ramptime}"
        if self._hiddenramp:
            s += ", hiddenramp=True"
        if self._threshold > 0:
            s += f", threshold={self._threshold}"
        return s + ")"


class DeadBand(PIDModifier):

    def __init__(self, biggerthan, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.biggerthan = biggerthan
        self.__deadbanded = False     # was the last 'u' in 'deadband'

    def deadbanded(self):
        """Return True if most recent 'u' was snapped to its prior 'u'."""
        return self.__deadbanded

    def PH_calculate_u(self, event):
        try:
            delta = abs(event.u - self.previous_u)
        except AttributeError:
            self.previous_u = event.u
            self.__deadbanded = False
        else:
            self.__deadbanded = delta <= self.biggerthan

        if self.__deadbanded:
            # make a copy of the proposed 'u' really only so that
            # it would get logged if followed by, e.g., PIDHistory
            event.deadbanded_u = event.u
            event.u = self.previous_u
        else:
            self.previous_u = event.u

    def PH_initial_conditions(self, event):
        if hasattr(self, 'previous_u'):
            del self.previous_u


#
# Probably not spectacularly useful, but as an example:
# Allow dt=0 so long as Kd=0 (i.e., there would be no derivative term)
#
class DTZero(PIDModifier):
    """Allow dt==0 if Kd==0 (so D term becomes zero)."""
    def PH_base_terms(self, event):
        if event.dt == 0 and event.pid.Kd == 0:
            event.d = 0


#
# Convert a PID into a "bangbang" control.
#
class BangBang(PIDModifier):
    """Implement bang-bang control."""
    def __init__(self, *args,
                 on_threshold=0, off_threshold=0,
                 on_value=1, off_value=0,
                 dead_value=None,
                 **kwargs):
        """Threshold semantics are:
        If the OFF threshold is None and ON not None:
              ON: >= on_threshold
             OFF:  < on_threshold

        If the OFF threshold is not None and ON is None:
             ON:  > off_threshold
            OFF: <= off_threshold

        If both on/off thresholds specified, then:
             ON: >= on_threshold
            OFF: <= off_threshold
           DEAD: > on and < off
        """
        super().__init__(*args, **kwargs)
        self.on_threshold = on_threshold
        self.off_threshold = off_threshold
        self.on_value = on_value
        self.off_value = off_value
        self.dead_value = dead_value

    def PH_calculate_u(self, event):
        if self.off_threshold is None:    # on_thr.. must not be None
            if event.u >= self.on_threshold:
                val = self.on_value
            else:
                val = self.off_value
        elif self.on_threshold is None:  # off_thr.. must not be None
            if event.u > self.off_threshold:
                val = self.on_value
            else:
                val = self.off_value
        else:
            if event.u >= self.on_threshold:
                val = self.on_value
            elif event.u <= self.off_threshold:
                val = self.off_value
            else:
                val = self.dead_value     # which may be None

        if val is not None:
            event.u = val


#
# It's not clear why this would ever be a good idea, but if delta-E
# is preferred for the D calculation over delta-PV, this modifier does that.
#

class D_DeltaE(PIDModifier):
    """Make D term use delta-e rather than delta-pv."""

    # This is a stateful modifier and cannot be shared among PIDs
    PH_attached = PIDModifier.attached_once_check

    def __init__(self, /, *, kickfilter=False):
        """Modifier: Make D term use delta-e rather than delta-pv.

           kickfilter (True/False, default False)

              Controls 'derivative kick' filtering. Derivative kick is a
              one-interval spike in D output caused by any setpoint change.
              If kickfilter is enabled, D calculation will simply repeat the
              previous D value (the 'e spike' will be gone after one tick)
        """
        self.kickfilter = kickfilter
        self._initialize_state()

    def PH_initial_conditions(self, event):
        self._initialize_state()

    def _initialize_state(self):
        self.kickticks = 0
        self.previous_e = None        # no matter what, the FIRST time no D

    def PH_setpoint_change(self, event):
        if self.kickfilter:
            self.kickticks = 1

    def PH_modify_terms(self, event):
        if self.previous_e is None:     # special case for very first call
            event.d = 0
        elif self.kickticks == 0:
            event.d = (event.e - self.previous_e) / event.dt
        else:
            self.kickticks -= 1
            event.d = self.previous_d
        self.previous_e = event.e
        self.previous_d = event.d


if __name__ == "__main__":
    import unittest
    import math

    class TestMethods(unittest.TestCase):
        def test_simple(self):
            def _onetest(p, pv=0, sp=0, ex0=0, exincr=0):
                p.initial_conditions(pv=pv, setpoint=sp)
                expected = ex0
                for i in range(10):
                    u = p.pid(pv, dt=1)
                    self.assertEqual(u, expected)
                    expected += exincr

            testvecs = (
                #  (pidargs, onetestargs)
                # anything omitted is zero
                (dict(Kp=10), dict(pv=3, ex0=-30)),                 # P
                (dict(Kp=10, Ki=2), dict(sp=1, ex0=12, exincr=2)),  # PI
                                                                    # PID
                (dict(Kp=10, Ki=2, Kd=5), dict(sp=1, ex0=12, exincr=2)),
            )

            for t in testvecs:
                for klass in (PID, PIDPlus):
                    with self.subTest(klass=klass, t=t):
                        pidargs, testargs = t
                        p = klass(**pidargs)
                        _onetest(p, **testargs)

        def test_KTT(self):
            # Test the suggestions in the README file on how to create a PID
            # that takes Ti/Td argument forms. This interface adapter class
            # (KTTPID) allows either form; it is pasted from the README

            class KTTPID(PID):
                def __init__(self, *args, Ti=None, Td=None, **kwargs):
                    try:
                        Kp = kwargs['Kp']
                    except KeyError:
                        if (Ti is not None or Td is not None):
                            raise TypeError("Ti or Td requires Kp") from None
                    if Ti is not None:
                        if 'Ki' in kwargs:
                            raise TypeError("Can't have both Ti and Ki")
                        kwargs['Ki'] = Kp / Ti if Ti != 0 else 0
                    if Td is not None:
                        if 'Kd' in kwargs:
                            raise TypeError("Can't have both Td and Kd")
                        kwargs['Kd'] = Kp * Td
                    super().__init__(*args, **kwargs)

            # great example of the magic of multiple inheritance and the mro
            class KTTPlus(KTTPID, PIDPlus):
                pass

            for args in (dict(Kp=100, Ti=2, Td=0.1),):
                with self.subTest(args=args):
                    z = KTTPID(**args)
                    Kp = args.get('Kp')
                    Ti = args.get('Ti')
                    Td = args.get('Td')
                    if Kp is not None:
                        self.assertEqual(z.Kp, Kp)
                    if Ti is not None:
                        self.assertEqual(z.Ki, Kp / Ti)
                    if Td is not None:
                        self.assertEqual(z.Kd, Kp * Td)

        def test_movingpv_D(self):
            # basically the same as test_simple/_onetest but this
            # has the process variable moving and tests the D term,
            # whereas since test_simple never moves the pv it doesn't
            # really test D term.
            Kp = 1
            Ki = 0         # simplifies this test
            Kd = 5
            p = PID(Kp=Kp, Ki=Ki, Kd=Kd)
            pv = 0
            setpoint = 0

            testvecs = ((0, -1, -3, -7),)
            for errs in testvecs:
                p.initial_conditions(pv=pv, setpoint=setpoint)
                for e in errs:
                    u = p.pid(pv - e, dt=1)
                # at this point the control should be Kp times
                # the last error, and Kd times the diff between the last 2
                expected = (errs[-1] * Kp) + ((errs[-1] - errs[-2]) * Kd)
                self.assertEqual(u, expected)

        def test_Dkick(self):
            pv0, u0 = 5, 0              # startup case
            pv1, u1 = 6, -1             # u1 = -1 bcs pv incr'd 1
            pv2, u2 = 7, -1             # u2 = -1 bcs pv incr'd 1
            pv3, u3 = 7, 0              # identical, so no u because flat

            # using initial_conditions() there should be no D kick
            p = PID(Kd=1)
            p.initial_conditions(pv=pv0)
            self.assertEqual(p.pid(pv0, dt=1), u0)
            self.assertEqual(p.pid(pv1, dt=1), u1)
            self.assertEqual(p.pid(pv2, dt=1), u2)
            self.assertEqual(p.pid(pv3, dt=1), u3)

            # same test, changing the setpoint has no effect because
            # the D term measures delta pv, not delta e
            p = PID(Kd=1)
            p.initial_conditions(pv=pv0)
            p.setpoint = -42
            self.assertEqual(p.pid(pv0, dt=1), u0)
            p.setpoint += 1
            self.assertEqual(p.pid(pv1, dt=1), u1)
            p.setpoint += 1
            self.assertEqual(p.pid(pv2, dt=1), u2)
            p.setpoint += 1
            self.assertEqual(p.pid(pv3, dt=1), u3)

            # but if pv changes (not via initial_conditions()) that
            # will induce kick
            p = PID(Kd=1)
            p.pv = pv0
            self.assertEqual(p.pid(pv0, dt=1), -pv0)   # this is the kick
            self.assertEqual(p.pid(pv1, dt=1), u1)
            self.assertEqual(p.pid(pv2, dt=1), u2)
            self.assertEqual(p.pid(pv3, dt=1), u3)

        def test_DdeltaE_kick(self):
            # same test but with D_DeltaE so the particulars of
            # how the kick shows up are different.
            pv0, u0 = 5, 0              # startup case
            pv1, u1 = 6, -1             # u1 = -1 bcs pv incr'd 1
            pv2, u2 = 7, -1             # u2 = -1 bcs pv incr'd 1
            pv3, u3 = 7, 0              # identical, so no u because flat

            # using initial_conditions() there should be no D kick
            p = PIDPlus(Kd=1, modifiers=D_DeltaE())
            p.initial_conditions(pv=pv0)
            self.assertEqual(p.pid(pv0, dt=1), u0)
            self.assertEqual(p.pid(pv1, dt=1), u1)
            self.assertEqual(p.pid(pv2, dt=1), u2)
            self.assertEqual(p.pid(pv3, dt=1), u3)

            # same test, changing the setpoint has no effect because
            # the D term measures delta pv, not delta e
            p = PIDPlus(Kd=1, modifiers=D_DeltaE())
            p.initial_conditions(pv=pv0)
            p.setpoint = -42
            self.assertEqual(p.pid(pv0, dt=1), u0)
            p.setpoint += 17
            self.assertEqual(p.pid(pv1, dt=1), u1+17)  # kick
            p.setpoint += 42
            self.assertEqual(p.pid(pv2, dt=1), u2+42)  # kick again
            p.setpoint += 37
            self.assertEqual(p.pid(pv3, dt=1), u3+37)  # kick again

            # but if pv changes (not via initial_conditions()) that
            # does not induce kick because it takes an interval to
            # have a delta e and by then the kick is gone
            p = PIDPlus(Kd=1, modifiers=D_DeltaE())
            p.pv = pv0
            self.assertEqual(p.pid(pv0, dt=1), u0)  # contrast to Dkick
            self.assertEqual(p.pid(pv1, dt=1), u1)
            self.assertEqual(p.pid(pv2, dt=1), u2)
            self.assertEqual(p.pid(pv3, dt=1), u3)

        # I_Freeze tests
        def test_i_freeze_basic(self):
            frz = I_Freeze()
            z = PIDPlus(Ki=1, modifiers=frz)
            setpoint = 5
            setpoint_divisor = 100
            dt = setpoint / setpoint_divisor
            pv = 0
            z.initial_conditions(pv=pv, setpoint=setpoint)

            # test this range, with an interruption in the middle for freeze
            n = 2 * setpoint_divisor
            freeze_at = n // 2

            for i in range(n):
                u = z.pid(pv, dt=dt)
                self.assertTrue(
                    math.isclose(u, (setpoint - pv) * dt * (i + 1)))

                if i == freeze_at:
                    # freeze it and do a while bunch of intervals
                    frz.freeze()
                    for _ in range(100):
                        self.assertTrue(math.isclose(u, z.pid(pv, dt=dt)))

                    # unfreeze it and see if it resumes correctly
                    frz.unfreeze()

        def test_i_freeze_secs(self):
            # similar to basic test but only freeze for a period of time
            frz = I_Freeze()
            z = PIDPlus(Ki=1, modifiers=frz)
            setpoint = 5
            setpoint_divisor = 100
            dt = setpoint / setpoint_divisor
            pv = 0
            z.initial_conditions(pv=pv, setpoint=setpoint)

            # test this range, with an interruption in the middle for freeze
            n = 2 * setpoint_divisor
            freeze_at = n // 2

            # need to do an explicit iter because playing games in the middle
            g = iter(range(n))
            for i in g:
                u = z.pid(pv, dt=dt)
                self.assertTrue(
                    math.isclose(u, (setpoint - pv) * dt * (i + 1)))

                if i == freeze_at:
                    # freeze it and do a while bunch of intervals
                    ndts = 10
                    frz.freeze(duration=dt*ndts)
                    for t in itertools.count():
                        v = z.pid(pv, dt=dt)
                        if not math.isclose(u, v):
                            # presumably freeze ended; check time
                            self.assertTrue(
                                math.isclose(t*dt, dt*(ndts+1)))
                            # at this point one 'dt' got used up so ...
                            next(g)
                            break

        # Test I_SetpointReset
        def test_i_reset1(self):
            # changing the setpoint should reset integration and
            # stop accumulation of integration for N seconds
            setpoint = 0.5
            for secs in (0, 5):
                z = PIDPlus(Ki=1, modifiers=I_SetpointReset(secs))
                z.initial_conditions(pv=0, setpoint=setpoint)
                # two pids should accumulate two setpoint's error
                # and the initial_conditions should not have triggered reset
                u1 = z.pid(0, dt=1)
                self.assertEqual(u1, setpoint)
                u2 = z.pid(0, dt=1)
                self.assertEqual(u2, u1 + setpoint)
                # but now changing the setpoint should cause a reset
                z.setpoint = 2*setpoint
                self.assertEqual(z.integration, 0)
                for tick in range(secs):
                    with self.subTest(tick=tick, secs=secs):
                        self.assertEqual(z.pid(0, dt=1), 0)

                # and this last one should be > 0 as now past the secs
                self.assertTrue(z.pid(0, dt=1) > 0)

        def test_setpointramp_trivia(self):
            # a few very directed simple tests at some prior bugs

            for hidden in (False, True):
                p = PIDPlus(modifiers=SetpointRamp(1, hiddenramp=hidden))

                initial_setpoint = p.setpoint    # this will be zero

                p.setpoint = 10
                if hidden:
                    # it should always just show as the target
                    expecting = 10
                else:
                    # it should read back as zero because the ramp should be
                    # set up but no ramping has happened yet.
                    expecting = 0
            self.assertEqual(p.setpoint, expecting)

        def test_spramp1(self):
            ramptime = 17       # just to be ornery with division/fuzz
            setpoint = 5
            dt = 0.100

            for hidden in (False, True):
                r = SetpointRamp(ramptime, hiddenramp=hidden)
                p = PIDPlus(Kp=1, modifiers=r)
                p.initial_conditions(pv=0, setpoint=setpoint)

                # verify that initial_conditions didn't cause ramping
                for pv, u in ((1, setpoint-1),
                              (1, setpoint-1),
                              (2, setpoint-2),
                              (0, setpoint),
                              (setpoint, 0)):
                    with self.subTest(pv=pv, u=u):
                        self.assertEqual(p.pid(pv, dt=dt), u)

                # now verify that it ramps ... go up to 2*setpoint
                p.setpoint = 2 * setpoint

                ramped = setpoint
                for i in range(int((ramptime / dt) + 0.5)):
                    with self.subTest(i=i):
                        u = p.pid(0, dt=dt)
                        ramped += (setpoint / ramptime) * dt
                        self.assertTrue(math.isclose(u, ramped))

                # at this point the setpoint is not usually exact because
                # of floating point fuzz. But if so, one more iteration should
                # peg it to the _target_sp and be exact. Check that.
                u = p.pid(0, dt=dt)
                self.assertEqual(p.setpoint, setpoint * 2)

        def test_spramp2(self):
            # test that fussing with the secs parameter midramp works

            for hidden in (False, True):
                ramper = SetpointRamp(4, hiddenramp=hidden)
                p = PIDPlus(Kp=1, modifiers=ramper)
                p.initial_conditions(pv=0, setpoint=0)
                p.setpoint = 100

                # in two 1-second intervals it should get halfway there
                p.pid(0, dt=1)
                u = p.pid(0, dt=1)
                self.assertTrue(math.isclose(u, 50))
                if not hidden:
                    self.assertTrue(math.isclose(p.setpoint, 50))

                # now if change secs to 5, should go by 10 more each time...
                ramper.secs = 5
                for i in range(5):
                    u = p.pid(0, dt=1)
                    expected = 50 + ((i + 1) * 10)
                    with self.subTest(sp=p.setpoint, u=u, i=i, hdn=hidden):
                        if not hidden:
                            self.assertTrue(math.isclose(p.setpoint, expected))
                        self.assertTrue(math.isclose(u, expected))

        def test_spramp3(self):
            # test that fussing with the secs parameter midramp works
            # but in this case testing "aborting the ramp" by setting secs=0
            for hidden in (False, True):
                ramper = SetpointRamp(4, hiddenramp=hidden)
                p = PIDPlus(Kp=1, modifiers=ramper)
                p.initial_conditions(pv=0, setpoint=0)
                p.setpoint = 100

                # in two 1-second intervals it should get halfway there
                p.pid(0, dt=1)
                u = p.pid(0, dt=1)
                if not hidden:
                    self.assertTrue(math.isclose(p.setpoint, 50))
                self.assertTrue(math.isclose(u, 50))

                # now if change secs to 0, should ramp immediately to target
                ramper.secs = 0
                u = p.pid(0, dt=1)
                with self.subTest(sp=p.setpoint, hidden=hidden):
                    self.assertTrue(math.isclose(u, 100))

        def test_sprampmonogamous(self):
            ramper = SetpointRamp(17)
            z1 = PIDPlus(Kp=1, modifiers=ramper)
            with self.assertRaises(TypeError):
                _ = PIDPlus(Kp=2, modifiers=ramper)

        def test_sprampthreshold(self):
            # test threshold concept
            threshold = 0.5
            setpoint = 10 * threshold
            ramptime = 5
            dt = 1

            for hidden in (False, True):
                for smallchange in (-threshold / 2,
                                    threshold / 2,
                                    # these values really just test the test
                                    -threshold / 123456,
                                    threshold / 123456):
                    with self.subTest(smallchange=smallchange, hdn=hidden):
                        r = SetpointRamp(ramptime,
                                         threshold=threshold,
                                         hiddenramp=hidden)
                        z = PIDPlus(Kp=1, modifiers=r)
                        z.setpoint = setpoint
                        u = z.pid(0, dt=dt)
                        ramped = (dt / ramptime) * setpoint
                        self.assertTrue(math.isclose(u, ramped))
                        if not hidden:
                            # because Kp was 1 these should be equal
                            self.assertEqual(z.setpoint, u)

                        # now make a 'smallchange' which should also have
                        # the side-effect of cancelling the in-progress ramp
                        newsetpoint = z.setpoint + smallchange
                        z.setpoint = newsetpoint
                        u = z.pid(0, dt=dt)
                        self.assertEqual(u, newsetpoint)
                        self.assertEqual(z.setpoint, newsetpoint)

                        # and the next z.pid call shouldn't be ramping anything
                        self.assertEqual(z.pid(0, dt=dt), u)

        def test_initial_conditions(self):
            h = PIDHistory()
            z = PIDPlus(modifiers=h)
            self.assertEqual(h.eventcounts['PH_attached'], 1)
            self.assertEqual(h.eventcounts['PH_initial_conditions'], 1)
            self.assertEqual(sum(h.eventcounts.values()), 2)

            z.initial_conditions(pv=17, setpoint=42)
            self.assertEqual(z.pv, 17)
            self.assertEqual(z.setpoint, 42)
            self.assertEqual(sum(h.eventcounts.values()), 3)
            self.assertEqual(h.eventcounts['PH_initial_conditions'], 2)

        def test_windup(self):
            windup_limit = 2.25
            dt = 0.1
            p = PIDPlus(Ki=1, modifiers=I_Windup(windup_limit))
            p.initial_conditions(pv=0, setpoint=1)
            for i in range(int(windup_limit / dt) + 1):
                p.pid(0, dt=dt)
            self.assertEqual(p.integration, windup_limit)
            # whatever the u value is now, it should stay here as there
            # can be no more integration.
            u = p.pid(0, dt=dt)
            for i in range(10):
                self.assertEqual(p.pid(0, dt=dt), u)

            # it should start decreasing immediately if pv goes high
            for i in range(int(windup_limit / dt) + 1):
                u2 = p.pid(2, dt=dt)
                self.assertTrue(u2 < u)
                if u2 < 0:
                    break
                u = u2
            else:
                raise ValueError(f"{i=}, {u2=}")

        def test_windup2(self):
            # like test_windup but asymmetric limits
            wlo = 1.0     # test only works if this is > 0
            whi = 2.25
            windup_limits = [whi, wlo]   # deliberately "backwards"
            dt = 0.1
            p = PIDPlus(Ki=1, modifiers=I_Windup(windup_limits))
            p.initial_conditions(pv=0, setpoint=1)

            # because wlo > 0, even a tiny tiny amount of integration
            # should jump up to wlo immediately; test that
            p.pid(0, dt=dt/1000)
            self.assertEqual(p.integration, wlo)

            # now do enough to exceed whi for certain, and test that
            # the integration value is within limits the whole way
            for i in range(int(whi / dt) + 1):
                p.pid(0, dt=dt)
                # note: it's known to be > (not >=) because there's already
                # one pid() call above, so this first one should add more
                self.assertTrue(p.integration > wlo)
                self.assertTrue(p.integration <= whi)

            self.assertEqual(p.integration, whi)
            # whatever the u value is now, it should stay here as there
            # can be no more integration.
            u = p.pid(0, dt=dt)
            for i in range(10):
                self.assertEqual(p.pid(0, dt=dt), u)

            # it should start decreasing immediately if pv goes high
            # but it should not get below wlo
            for i in range(int(whi / dt) + 1):
                u2 = p.pid(2, dt=dt)
                self.assertTrue(u2 < u)
                if u2 < wlo:
                    raise ValueError(f"{i=}, {u2=}")
                elif u2 == wlo:
                    break
            else:
                raise ValueError(f"never got down to {wlo}")

        def test_readonly(self):
            class Foo(PIDModifier):
                def PH_modify_terms(self, event):
                    event.d = 17

                def PH_calculate_u(self, event):
                    if event.d != 17:
                        raise ValueError(f"Got {event.d}")
                    try:
                        event.d += 1              # should fail
                    except TypeError:
                        event.u = event.d + 1     # this is expected to run

            z = PIDPlus(Ki=1, modifiers=Foo())
            u = z.pid(0, dt=1)
            self.assertEqual(z.last_pid, (0, 0, 17))
            self.assertEqual(u, 18)

        # a test that READONLY applies to an attribute even if the attribute
        # is not established at __init__ time -- it is "write once"
        def test_writeonce(self):
            class FooEvent(_PIDHookEvent):
                READONLY = {'foo'}

            f = FooEvent()
            with self.assertRaises(AttributeError):
                x = f.foo       # hasn't been set yet, should fail

            f.foo = 'fooby'
            self.assertEqual(f.foo, 'fooby')

            with self.assertRaises(TypeError):
                f.foo = 'this fails because now read-only'

            self.assertEqual(f.foo, 'fooby')

        def test_multi_readonly(self):
            # tests that the property magic for read-only attrs
            # works properly in the face of multiple hook objects
            # (this was once a bug)
            e1 = PIDHookCalculateU(e=1)
            e1.pid = 'bozo'
            e2 = PIDHookCalculateU(e=2)
            e2.pid = 'bonzo'
            self.assertEqual(e1.pid, 'bozo')
            self.assertEqual(e1.e, 1)
            self.assertEqual(e2.pid, 'bonzo')
            self.assertEqual(e2.e, 2)

        def test_pidreadonly(self):
            class NothingReadOnly(_PIDHookEvent):
                READONLY = set()

            class PidReadOnly(_PIDHookEvent):
                READONLY = {'pid'}       # unnecessary but make sure ok

            class AllRO(_PIDHookEvent):
                READONLY = {'*'}

            class P(_PIDHookEvent):
                pass

            # .pid is always readonly after set once
            pidval = 'bozo'
            aval = 1
            testargs = (
                dict(),
                dict(a=aval),
                dict(pid=pidval),
                dict(a=aval, pid=pidval)
                )

            for klass in (NothingReadOnly, PidReadOnly, AllRO, P):
                for args in testargs:
                    with self.subTest(klass=klass, args=args):
                        ebozo = klass(**args)
                        if 'pid' not in args:
                            ebozo.pid = pidval
                        if 'a' in args:
                            self.assertEqual(ebozo.a, aval)
                        with self.assertRaises(TypeError):
                            ebozo.pid = object()    # i.e., something else
                        self.assertEqual(ebozo.pid, pidval)

        def test_attrpropagation(self):
            # Modifiers can add more attributes during the pre/mid/post
            # calculation events (that might be a bug?) ... test this.
            class FooMod(PIDModifier):
                counter = 0

                def PH_base_terms(self, event):
                    event.foo = self.counter
                    self.counter += 1

                def PH_calculate_u(self, event):
                    if event.foo != self.counter - 1:
                        raise ValueError("XXX")

            z = PIDPlus(modifiers=FooMod())
            z.pid(1, dt=0.01)         # that this doesn't bomb is the test

        def test_handler_exceptions_1(self):
            # a small test demonstrating PIDHookFailure handling but
            # also demonstrating what happens if a PIDHookFailure handler
            # itself causes another exception (which, by design, bubbles out)

            class TestException(Exception):
                def __init__(self, *args, marker=None, **kwargs):
                    super().__init__(*args, **kwargs)
                    self.test_exception_marker = marker

            ff_marker = object()
            bf_marker = object()

            class FailFail(PIDModifier):
                def PH_failure(self, event):
                    raise TestException(marker=ff_marker)

            class BaseFail(PIDModifier):
                def PH_base_terms(self, event):
                    raise TestException(marker=bf_marker)

            h0 = PIDHistory()
            h1 = PIDHistory()
            h2 = PIDHistory()

            z = PIDPlus(modifiers=(h0, BaseFail(), h1, FailFail(), h2))
            try:
                z.pid(0, dt=0.25)
            except TestException as e:
                self.assertEqual(e.test_exception_marker, ff_marker)
            else:
                raise ValueError("Did not get testException")

            # all three histories shoudl have the same length, because the
            # exceptions turn into PIDHookFailure events and so everything
            # should track 1:1 in terms of length and related events in
            # this case (wouldn't be true if nested events were triggered)
            self.assertEqual(len(h0.history), len(h1.history))
            self.assertEqual(len(h0.history), len(h2.history))

            # convenience
            last0 = h0.history[-1]
            last1 = h1.history[-1]
            last2 = h2.history[-1]

            # the last entry in h0 should be the BaseTerms event
            self.assertTrue(isinstance(last0, PIDHookBaseTerms))

            # the last entry in h1 should be a PIDHookFailure event
            # with the exception having the bf_marker in it
            self.assertTrue(isinstance(last1, PIDHookFailure))
            self.assertEqual(last1.exc.test_exception_marker, bf_marker)

            # the last entry in h2 should be a PIDHookFailure event
            # with the exception having the ff marker in it and check the
            # event stack in the exceptions
            self.assertTrue(isinstance(last2, PIDHookFailure))
            self.assertEqual(last2.exc.test_exception_marker, ff_marker)
            self.assertEqual(last2.event.event, last1.event)

        def test_handler_exceptions_2(self):
            # if handlers cause exceptions (other than HookStop) in
            # all likelihood the error is unrecoverable; however, the
            # notify system tries to at least let the other handlers
            # know when this happens. This tests that.

            class TestException(Exception):
                pass

            class BadAttach(PIDModifier):
                def PH_attached(self, event):
                    raise TestException("LIONS AND TIGERS")

            class BadModify(PIDModifier):
                def PH_modify_terms(self, event):
                    raise TestException("AND BEARS, OH MY!")

            class NoPrint(EventPrint):
                def __init__(self, *args, **kwargs):
                    super().__init__(*args, **kwargs)
                    self._nestings = []

                def printevent(self, event):
                    self._nestings.append((event, event.pid.nn_level))

            with self.assertRaises(TestException):
                h = PIDHistory()
                z = PIDPlus(Kp=1, modifiers=(BadAttach(), h))

            self.assertEqual(h.eventcounts[PIDHookFailure.handlername()], 1)

            # now make sure nesting still starts at 1 after an exc
            h = PIDHistory()
            npr = NoPrint()
            mods = (h, BadModify(), SetpointRamp(2), npr)
            z = PIDPlus(Kp=1, modifiers=mods)
            z.setpoint = 10
            for _ in range(4):
                with self.assertRaises(TestException):
                    z.pid(0, dt=0.25)

            # after all that ... make sure the PIDHookBaseTerms all started
            # at level 1 (others should start there too but this is what is
            # checked here). To test this test, an incorrect implementation
            # of PIDHookFailure logic in notify() was tried just to see that
            # this would indeed catch that sort of problem.
            #
            for e, lvl in npr._nestings:
                if e.__class__ is PIDHookBaseTerms:
                    self.assertEqual(lvl, 1)

        def test_readme_1(self):
            # just testing an example given in the README.md
            class SetpointPercent(PIDModifier):
                def PH_setpoint_change(self, event):
                    event.sp = event.sp_to / 100

            z = PIDPlus(Kp=1, modifiers=SetpointPercent())
            z.setpoint = 50
            self.assertEqual(z.setpoint, 50/100)

        def test_readme_2(self):
            # just testing another example fromthe README.md
            class UBash(PIDModifier):
                def PH_base_terms(self, event):
                    event.u = .666

            z = PIDPlus(Kp=1, modifiers=UBash())
            self.assertEqual(z.pid(0, dt=0.01), 0.666)

        def test_readme_3(self):
            # yet another README.md example test
            class SetpointPercent(PIDModifier):
                def PH_base_terms(self, event):
                    event.e = (event.pid.setpoint / 100) - event.pid.pv

            z = PIDPlus(Kp=1, modifiers=SetpointPercent())
            z.setpoint = 50
            u = z.pid(0, dt=0.01)
            self.assertEqual(z.setpoint, 50)
            self.assertEqual(u, 50 / 100)

        def test_deadband(self):
            # test the DeadBand modifier

            # in this first test all should be within the deadband
            db = 25
            m = DeadBand(db)
            z = PIDPlus(Kp=1, modifiers=m)
            setpoint = 50
            pv0 = 60
            z.initial_conditions(pv=pv0, setpoint=setpoint)
            cv0 = z.pid(pv0, dt=1)
            for deltapv in range(2*db):
                pv = pv0 - db + deltapv
                cv = z.pid(pv, dt=1)
                with self.subTest(deltapv=deltapv, cv=cv, pv=pv):
                    self.assertTrue(cv == cv0)
                    self.assertTrue(m.deadbanded())

            # varying tests, again in both forms
            pv0 = 60
            pvs = [pv0, pv0 - db, pv0 + db, pv0 - (db - 1), pv0 + (db - 1),
                   pv0 + (db + 1), 17, 22, 1, 76, 56, 35, 89, 22, 96, 3, 79,
                   69, 83, 86, 92, 67, 45, 36, 44, 92, 36, 1, 25, 1, 97, 18,
                   72, 65, 4, 16, 2, 92, 14, 40, 72, 73, 1, 34, 38, 48, 30,
                   10, 54, 72, 21, 38, 32, 26, 73, 63, 37, 61, 46, 9, 83, 26,
                   79, 83, 36, 53, 93, 94, 78, 51, 92, 85, 15, 93, 13, 18,
                   86, 28, 36, 1, 68, 30, 51, 41, 85, 6, 74, 34, 96, 15, 50,
                   96, 14, 54, 30, 24, 55, 4, 55, 84, 92, 69, 70, 58, 11]
            m = DeadBand(db)
            z = PIDPlus(Kp=1, modifiers=m)
            z.initial_conditions(pv=pv0, setpoint=setpoint)
            cv0 = z.pid(pv0, dt=1)
            for i, pv in enumerate(pvs):
                cv = z.pid(pv, dt=1)
                with self.subTest(i=i, pv=pv, cv=cv, cv0=cv0):
                    if abs(pv - pv0) > db:
                        self.assertNotEqual(cv0, cv)
                        self.assertTrue(not m.deadbanded())
                        cv0 = cv
                        pv0 = pv
                    else:
                        self.assertTrue(cv == cv0)
                        self.assertTrue(m.deadbanded())

        def test_history_sizing(self):
            # test the various 'n'/resize/since features of PIDHistory

            # simple test of 'n'
            h = PIDHistory(1)
            z = PIDPlus(modifiers=h)
            z.setpoint = 5             # cause a SetpointChange
            self.assertEqual(h.history[0], h.history[-1])
            self.assertTrue(isinstance(h.history[0], PIDHookSetpointChange))

            # simple test of resize; start infinite, do 100 setpoints,
            # resize to 10, see if it's the last 10 setpoints saved.
            h = PIDHistory(None)
            z = PIDPlus(modifiers=h)

            # set the setpoint to something that won't be used below
            z.setpoint = 867.5309

            # get the current last item (the SetpointChange just done)
            last = h.history[-1]

            # now do a bunch of setpoints...
            maxsetpoints = 100
            saveonly = 10
            for i in range(maxsetpoints):
                z.setpoint = i

            # and see if all 100 are there (this is also a test of "since")
            for i, e in enumerate(h.since(last)):
                self.assertEqual(e.sp_to, i)

            # resize to the new size...
            h.resize(saveonly)

            # and now it should only be the last 10 in there
            for i, e in enumerate(h.history):
                self.assertEqual(e.sp_to, i + maxsetpoints - saveonly)

            # really shouldn't do this kind of hack, supplying bogus events
            # as the 'preserve' events, but test it anyway...
            hackevents = 'abcde'       # each char will be a new 'event'
            h.resize(saveonly, preserve=hackevents)
            for i, e in enumerate(h.history):
                self.assertEqual(hackevents[i], e)

        def test_history_copy(self):
            # this in particular tests that the history captures the
            # state of the events AT THE TIME in THAT POSITION in the
            # modifications list. Basically it's a test that copy works.
            class Clown(PIDModifier):
                def __init__(self, name, *args, **kwargs):
                    super().__init__(*args, **kwargs)
                    self.clown = name

                def PH_setpoint_change(self, event):
                    event.sp = self.clown

            h1 = PIDHistory(10)
            h2 = PIDHistory(10)
            h3 = PIDHistory(10)
            bozo = Clown('bozo')
            bonzo = Clown('bonzo')

            z = PIDPlus(Kp=1, modifiers=[h1, bozo, h2, bonzo, h3])
            z.setpoint = 7

            e1 = h1.history[-1]
            e2 = h2.history[-1]
            e3 = h3.history[-1]

            self.assertTrue(id(e1) != id(e2))
            self.assertTrue(id(e1) != id(e3))
            self.assertTrue(id(e2) != id(e3))
            self.assertEqual(e1.sp, None)
            self.assertEqual(e2.sp, 'bozo')
            self.assertEqual(e3.sp, 'bonzo')
            with self.assertRaises(TypeError):
                e1.pid = None

        def test_hookstop(self):
            # test HookStop - a modifier asking to abort further notifications
            class Stopper(PIDModifier):
                def PH_default(self, event):
                    raise HookStop

            class Count(PIDModifier):
                count = 0
                hookstops = []

                # just to filter this one out of the PH_default count
                def PH_attached(self, event):
                    self.pid = event.pid     # implicitly testing this attach

                def PH_hook_stopped(self, event):
                    self.hookstops.append(event)

                def PH_default(self, event):
                    self.count += 1

            c1 = Count()
            c2 = Count()
            stopper = Stopper()

            z = PIDPlus(Kp=1, modifiers=[c1, stopper, c2])

            # just the initialization will generate an event, which c1
            # should have seen and c2 should not have seen.
            self.assertEqual(c1.count, 1)
            self.assertEqual(c2.count, 0)

            # implicitly test that the attach worked for c1 but
            # was stopped for c2 (hah)
            self.assertEqual(c1.pid, z)
            self.assertFalse(hasattr(c2, 'pid'))

            # and test that c2 got notified of that hookstop and also
            # the initial_conditions hookstop
            self.assertEqual(len(c2.hookstops), 2)

        def test_oneattach(self):
            class AttachToJustOne(PIDModifier):
                def PH_attached(self, event):
                    try:
                        if self.__attached_to != event.pid:
                            raise TypeError("multiple attachment attempted")
                    except AttributeError:
                        self.__attached_to = event.pid

            j1 = AttachToJustOne()
            z1 = PIDPlus(Kp=1, modifiers=j1)
            with self.assertRaises(TypeError):
                z2 = PIDPlus(Kp=1, modifiers=j1)

            # but (at least in this case) it is allowed to be attached
            # to the same one multiple times
            jx = AttachToJustOne()
            _ = PIDPlus(Kp=1, modifiers=[jx, jx])

        def test_notifications(self):
            class XCounter(PIDHistory):
                def __init__(self, n=0, *args, **kwargs):
                    super().__init__(n, *args, **kwargs)
                    self.hookstopped = Counter()
                    self.failures = 0

                def _gotevent(self, event):
                    super().PH_default(event)    # let PIDHistory count it

                # Hide hook_stopped from the underlying PIDHistory
                # Count them separately
                def PH_hook_stopped(self, event):
                    stopped = event.event
                    self.hookstopped[stopped.handlername()] += 1

                # same deal for Failure (which never happens)
                def PH_failure(self, event):
                    self.failures += 1

                def PH_default(self, event):
                    raise TypeError("DEFAULT HANDLER INVOKED")

                @property
                def totalcount(self):
                    return (sum(self.eventcounts.values()) +
                            sum(self.hookstopped.values()))

            class Stopper(PIDModifier):
                def PH_default(self, event):
                    raise HookStop

            # dynamically discover the handlernames
            def _all_notification_method_names(cls):
                for subcls in cls.__subclasses__():
                    yield from _all_notification_method_names(subcls)
                    try:
                        yield subcls.handlername()
                    except AttributeError:
                        pass     # e.g., _PIDHookCalc framework class

            METHODNAMES = set(_all_notification_method_names(_PIDHookEvent))
            for nm in METHODNAMES:
                # only set to _gotevent if not already in NotificationChecker
                if not hasattr(XCounter, nm):
                    setattr(XCounter, nm, XCounter._gotevent)

            # Test Outline: make a PID with several XCounter modifiers,
            # with a HookStop emitter ('stopper') somewhere in the middle.
            # Perform operations known/expected to cause every METHODNAMES
            # event exactly once. Test the count results - 1 per event
            # in every XCounter before the stopper, and 1 hookstopped per
            # event for every XCounter after.

            stopper = Stopper()
            precount = 3           # should be at least 2
            postcount = 3          # should be at least 2
            mods = [*[XCounter() for i in range(precount)],
                    stopper,
                    *[XCounter() for i in range(postcount)]]

            z = PIDPlus(Kp=1, modifiers=mods)
            z.setpoint = 17
            z.pid(0, dt=.01)

            # only events that had _gotevent as their handler are counted
            COUNTED = (nm for nm in METHODNAMES
                       if getattr(XCounter, nm) == XCounter._gotevent)

            beforestopper = True
            for i, mod in enumerate(mods):
                if mod is stopper:
                    beforestopper = False
                    continue

                # none of them should have seen a PH_failure
                self.assertEqual(mod.failures, 0)

                if beforestopper:
                    # In the ones preceding the stopper, each event
                    # should have been seen once and no hook stops.
                    seen = mod.eventcounts
                    zeros = mod.hookstopped
                else:
                    # In the 'last' it's the reverse
                    seen = mod.hookstopped
                    zeros = mod.eventcounts

                for mname in COUNTED:
                    with self.subTest(mname=mname, i=i):
                        self.assertEqual(seen[mname], 1)
                        self.assertEqual(zeros[mname], 0)
                for v in ChainMap(seen.keys(), zeros.keys()):
                    with self.subTest(v=v):
                        self.assertTrue(v in METHODNAMES)

                # whether real events or hookstopped, they should all
                # have seen the same number in total
                self.assertEqual(mods[0].totalcount, mod.totalcount)

        def test_hh(self):
            # a test of HookStop being raised within HookStop handlers
            class Stopper(PIDModifier):
                def __init__(self, nth, *args, ut, **kwargs):
                    super().__init__(*args, **kwargs)
                    self.ut = ut             # unittest object
                    self.nth = nth
                    self.got_ic = False
                    self.got_atd = False

                def PH_hook_stopped(self, event):
                    # first off this should not occur at nth 0
                    # because it is always a result of a prior modifier
                    # raising HookStop within a handler
                    self.ut.assertTrue(self.nth > 0)

                    # compute the implied depth from the HookStopped chain
                    foo = event
                    hh_depth = 0
                    while isinstance(foo, PIDHookHookStopped):
                        hh_depth += 1
                        foo = foo.event

                    # and compare to nestinglevel (which starts at 1)
                    self.ut.assertEqual(hh_depth, self.nth)
                    raise HookStop

                def PH_attached(self, event):
                    self.ut.assertEqual(self.nth, 0)
                    self.ut.assertFalse(self.got_atd)
                    self.got_atd = True
                    raise HookStop

                def PH_initial_conditions(self, event):
                    self.ut.assertEqual(self.nth, 0)
                    self.ut.assertTrue(self.got_atd)
                    self.ut.assertFalse(self.got_ic)
                    self.got_ic = True
                    raise HookStop

                def PH_default(self, event):
                    raise TypeError("DEFAULT")

            z = PIDPlus(modifiers=[Stopper(i, ut=self) for i in range(10)])
            # NOTE: the test runs in the handlers, implicitly, and just
            #       creating the PIDPlus triggers attached and init_conds.

    unittest.main()

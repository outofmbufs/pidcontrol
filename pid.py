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

import copy
from collections import deque, ChainMap


class PID:
    """Simple PID control."""

    def __init__(self, /, *, Kp=0, Ki=0, Kd=0):
        """Create a PID controller with the given parameters.

        Kp, Ki, Kd        -- weights for P/I/D control signals
        """

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # if different initial conditions are desired, callers should
        # call initial_conditions themselves after this initialization.
        self.initial_conditions(pv=0, setpoint=0)

    def initial_conditions(self, /, *, pv=None, setpoint=None):
        """Establish initial conditions -- resets state accordingly.

        Use this rather than bashing .pv and .setpoint directly, because
        this will also reset relevant state variables and will avoid
        any "kick" from instantaneous pv/setpoint changes.
        """

        self.dt = 0          # corresponding dt (between pid() calls)
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
        NOTE: Calling this with a zero dt will fail w/ZeroDivisionError.
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
        return self._u(p, i, d)

    def _error(self):
        return self.setpoint - self.pv

    def _proportional(self, e):
        """Return the (unweighted) proportional error term."""
        return e

    def _integral(self, e):
        """Return the (unweighted) integral error term."""
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
        dpv = (self.previous_pv - self.pv) / self.dt
        self.previous_pv = self.pv
        return dpv

    def _u(self, p, i, d):
        self.last_pid = (p, i, d)
        return (p * self.Kp) + (i * self.Ki) + (d * self.Kd)

    def __repr__(self):
        s = self.__class__.__name__
        pre = "("
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

        # let each modifier know it has been attached to this pid
        PIDHookAttached(pid=self).notify(self.modifiers)

        # Must be after establishing modifiers, because of PIDHookInitial..
        # event that will be triggered in initial_conditions()
        super().__init__(*args, **kwargs)

    def initial_conditions(self, *args, **kwargs):
        super().initial_conditions(*args, **kwargs)

        # NOTE: It is important that modifiers with state have a handler
        #       for this event. For example, setpoint ramping could have
        #       been triggered (when the base class changes the setpoint)
        #       and that should be reset by this event.
        PIDHookInitialConditions(
            *args, pid=self, **kwargs).notify(self.modifiers)

    # .setpoint becomes a property so PIDModifiers can be notified of changes
    @property
    def setpoint(self):
        return self._setpoint

    @setpoint.setter
    def setpoint(self, v):
        # this dance avoids generating spurious notification during __init__
        try:
            sp_prev = self._setpoint
        except AttributeError:        # happens during __init__ only
            sp_prev = self._setpoint = v

        if v == sp_prev:
            return                    # no notifications on no-change 'changes'

        # the notification protocol
        event = PIDHookSetpoint(
            pid=self, sp_prev=sp_prev, sp_set=v).notify(self.modifiers)
        self._setpoint = v if event.sp_now is None else event.sp_now

    def _calculate(self):
        """Run PIDModifier protocol and return control value ('u')."""

        # There are three notification points: Pre/Mid/Post which allow
        # the various PIDModifiers to do their thing.

        # -- PRE --
        # Run all the precalc's which may (or may not) establish some
        # of the process control values (e/p/i/d/u).
        cx = PIDHookPreCalc(pid=self).notify(self.modifiers)

        # -- MID --
        # Calculate e/p/i/d except if already supplied by a PRE
        if cx.e is None:
            cx.e = self._error()
        for attr, f in (('p', self._proportional),
                        ('i', self._integral),
                        ('d', self._derivative)):
            if getattr(cx, attr) is None:
                setattr(cx, attr, f(cx.e))

        # note the vars from the PreCalc are cloned to the MidCalc
        cx = PIDHookMidCalc(**cx.vars()).notify(self.modifiers)

        # -- POST --
        # Generally 'u' calculation should be overridden in a PostCalc.
        # See, for example, BangBang. But allow for it in Pre/Mid by
        # testing for it. By far the common case is cx.u is still None here.
        if cx.u is None:
            cx.u = self._u(cx.p, cx.i, cx.d)
        # again note the vars from the MidCalc are cloned to the PostCalc
        cx = PIDHookPostCalc(**cx.vars()).notify(self.modifiers)

        # Whatever 'u' came through all that ... that's the result!
        return cx.u

    def __repr__(self):
        s = super().__repr__()
        if self.modifiers:
            # turn 'PIDPlus()' into 'PIDPlus('
            # turn 'PIDPlus(anything)' into 'PIDPlus(anything, )'
            s = s[:-1]
            if s[-1] != '(':
                s += ', '
            premod = "modifiers="
            for m in self.modifiers:
                s += f"{premod}{m}"
                premod = ", "
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
# Each specific PIDHookEvent defines a name (via NOTIFYHANDLER attribute)
# of a method that a PIDModifier is expected to supply if that PIDModifier
# wants to receive that type of event.
#
# For example, setpoint modifications generate a PIDHookSetpoint event.
# The NOTIFYHANDLER attribute in PIDHookSetpoint is 'PH_setpoint_change'.
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

    # Subclasses can redefine this to supply default attribute values
    DEFAULTED_VARS = {}

    # Subclasses MUST put attribute names in here for attributes
    # that are writable (all other attributes will be read-only)
    # NOTE: '*' means all variables RW, but see STAR_READONLY too.
    RW_VARS = set()

    # these attributes are read-only even if '*' used in RW_VARS
    STAR_READONLY = {'pid'}

    # Property/descriptor implementing read-only-ness
    class _ReadOnlyDescr:
        _PVTPREFIX = "__PVT_ReadOnly_"

        # Factory for annointing PIDHookEvent classes with descriptors
        # for any attributes that are enforced as read-only.
        #    cls      -- will be (of course) this class
        #    attrname -- name of a (to be set up) read-only attribute
        #    value    -- its initial value
        #    obj      -- the underlying PIDHookEvent OBJECT (not class)
        #
        # The obj.__class__ (something like, e.g., PIDHookSetpoint) needs
        # a data descriptor property created for this attrname, but that
        # property should only be created ONCE (per attrname, per class).
        # In contrast, individual object instances each need, of course,
        # their own instance variable (object attribute) for the underlying
        # readonly value; that attribute name is built using _PVTPREFIX to
        # distinguish it from ordinary (read/write) attrs.
        @classmethod
        def establish_property(cls, attrname, value, /, *, obj=None):
            """Factory for creating the property descriptors (dynamically)."""

            pidhook_class = obj.__class__    # e.g., PIDHookSetpoint, etc.

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

                # and this establishes the data descriptor on the class
                setattr(pidhook_class, attrname, descriptor)

            # now can just set the underlying attribute directly.
            setattr(obj, descriptor._name, value)

        # readonly attributes won't be in (generic) vars(), and their
        # underlying ("privatized") names will. So vars() is not
        # (very) helpful. Instead, _Readonly class provides its own
        # vars() implementation. This will return all the appropriate
        # attribute names, including the readonly ones, and not any
        # of the privatized ("underlying") names.
        @classmethod
        def vars(cls, obj):
            """like vars() but finesse readonly name shenanigans."""
            vd = {}
            for a, v in vars(obj).items():
                if a.startswith(cls._PVTPREFIX):
                    a = a[len(cls._PVTPREFIX):]
                vd[a] = v
            return vd

        def __get__(self, obj, objtype=None):
            if obj is None:
                return self
            return getattr(obj, self._name)

        def __set__(self, obj, value):
            raise TypeError(
                f"write to read-only attr '{self._name}' not allowed")

        def __delete__(self, obj):
            raise TypeError(
                f"attempted deletion of read-only attr '{self._name}'")

    def __init__(self, /, **kwargs):
        for k, v in ChainMap(kwargs, self.DEFAULTED_VARS).items():
            readonly = (k in self.STAR_READONLY if '*' in self.RW_VARS
                        else k not in self.RW_VARS)
            if readonly:
                self._ReadOnlyDescr.establish_property(k, v, obj=self)
            else:
                setattr(self, k, v)

    def notify(self, modifiers):
        """Invoke the notification handler for all modifiers.

        For notational convenience, returns self, for 1-liners like:
            event = PIDHookSomething(foo).notify(modlist)
        when the returned event is needed even after the notify()
        """

        for nth, m in enumerate(modifiers):
            h = getattr(m, self.NOTIFYHANDLER, getattr(m, 'PH_default'))
            try:
                h(self)        # ... this calls m.PH_foo(event)
            except HookStop:
                # stop propagating; notify the REST of the modifiers of THAT
                PIDHookHookStopped(
                    event=self,        # the event that was HookStop'd
                    stopper=m,         # whodunit
                    nth=nth,           # in case more than one 'm'
                    modifiers=modifiers
                ).notify(modifiers[nth+1:])
                break
        return self            # notational convenience

    def vars(self):
        """Built-in vars returns gibberish for readonly's; this doesn't."""
        return self._ReadOnlyDescr.vars(self)

    def __repr__(self):
        s = self.__class__.__name__
        pre = "("
        for a, v in self.vars().items():
            s += f"{pre}{a}={v!r}"
            pre = ", "
        return s + ")"


# with the above __init__ and notify as a framework, typically each
# specific event is simply a subclass with some class variables:
#    NOTIFYHANDLER -- the name of the corresponding handler to invoke
#    DEFAULTED_VARS -- dictionary of default attribute names/values that
#                      will be used (if no explicit values given in init)
#    RW_VARS        -- names of attributes that are allowed to be read-write.
#                      The default for any attributes not in RW_VARS is
#                      to enforce read-only access of them. As a special
#                      case convenience, RW_VARS can be '*' (the string)
#                      which will mean make all the attributes read/write
#
# Conceptually the NOTIFYHANDLER could have been programmatically
# determined from the subclass name, but "explicit is better" won out here.
#
# The "DEFAULTED_VARS" is an alternate way to implement what would otherwise
# be an __init()__ in each subclass with a custom signature and super() call
# for *args/**kwargs. Subclasses can still do it that way if desired; but the
# DEFAULTED_VARS mechanism makes trivial cases trivial and avoids boilerplate
# like: "super().__init__(*args, arg1=arg1, arg2=arg2, ... **kwargs)"
#

class PIDHookAttached(_PIDHookEvent):
    NOTIFYHANDLER = 'PH_attached'


class PIDHookHookStopped(_PIDHookEvent):
    NOTIFYHANDLER = 'PH_hookstopped'


class PIDHookInitialConditions(_PIDHookEvent):
    NOTIFYHANDLER = 'PH_initial_conditions'
    DEFAULTED_VARS = dict(setpoint=None)


class PIDHookSetpoint(_PIDHookEvent):
    NOTIFYHANDLER = 'PH_setpoint_change'
    DEFAULTED_VARS = dict(sp_now=None)
    RW_VARS = {'sp_now'}


# generic base for PreCalc/Mid/Post. Establishes the event default vars.
class _PIDHook_Calc(_PIDHookEvent):
    DEFAULTED_VARS = dict(e=None, p=None, i=None, d=None, u=None)
    RW_VARS = '*'


class PIDHookPreCalc(_PIDHook_Calc):
    NOTIFYHANDLER = 'PH_precalc'


class PIDHookMidCalc(_PIDHook_Calc):
    NOTIFYHANDLER = 'PH_midcalc'


class PIDHookPostCalc(_PIDHook_Calc):
    NOTIFYHANDLER = 'PH_postcalc'
    RW_VARS = 'u'         # setting others is a no-op, so prevent that error


# Any PIDModifier that wants to stop hook processing raises this:
class HookStop(Exception):
    pass


# ----------------------------
# PIDModifier class hierarchy.
# ----------------------------

class PIDModifier:
    """Base, no-op, class for the 'modifiers' used in a PIDPlus."""

    def PH_default(self, event):
        """Default (no-op) handler for unhandled event types"""
        pass

    # Subclasses that want to limit themselves to being attached to
    # exactly one PIDPlus controller can put this line in their class body:
    #      PH_attached = PIDModifier._pid_limit_1
    # or, alternatively, invoke this within their own PH_attached handler.
    #
    # NOTE: Using this establishes a .pid attribute, which is available
    #       for use by the subclass (including, meh, deleting it to reset!)
    def _pid_limit_1(self, event):
        try:
            if self.pid != event.pid:
                raise TypeError(f"multiple attachment attempted: '{self}'")
        except AttributeError:
            self.pid = event.pid


class PIDHistory(PIDModifier):
    """Adds a look-back record of control computations to a PID."""

    def __init__(self, n, *args, **kwargs):
        """Adds a record of the past 'n' control computations to a PID."""
        super().__init__(*args, **kwargs)
        self.history = deque([], n)

    # this _default method gets all events and logs them
    def PH_default(self, event):
        # if PIDHistory is the last in the modifier list then copying
        # the event is superfluous. But if there are modifiers after,
        # they get the same event object (which is part of how the whole
        # protocol works, after all) but presumably if this PIDHistory was
        # placed earlier in the stack it was done so to capture the event
        # state at that point. Therefore ... need to copy the event.
        self.history.append(copy.copy(event))


class I_Windup(PIDModifier):
    """Enhances I control with "windup" limit."""

    def __init__(self, w, *args, **kwargs):
        super().__init__(*args, **kwargs)
        """The integration value will be held to [-w, w] range.

        OPTIONALLY: if w is a tuple, then the integration value
                    will be held to [a, b] range where:
                       a, b = sorted(w)
        """
        try:
            a, b = sorted(w)
        except TypeError:
            a, b = -abs(w), abs(w)    # prevent negative shenanigans
        self.w0 = a
        self.w1 = b

    def PH_midcalc(self, event):
        clamped = max(self.w0, min(event.pid.integration, self.w1))
        event.i = event.pid.integration = clamped


class I_SetpointReset(PIDModifier):
    """Reset integration, with optional pause, when setpoint changes."""

    # This is a stateful modifier and cannot be shared among PIDs
    PH_attached = PIDModifier._pid_limit_1

    def __init__(self, secs, *args, **kwargs):
        """Resets integration on setpoint change, and pause it 'secs'.

        NOTE: 'secs' can be zero, in which case a setpoint change resets the
              integration, but restarts it immediately (i.e, 0 second pause).
        """
        super().__init__(*args, **kwargs)
        self.integration_pause = secs
        self._initialize_state()

    def PH_initial_conditions(self, event):
        self._initialize_state()

    def _initialize_state(self):
        self.pause_remaining = 0
        self._trigger = False

    def PH_setpoint_change(self, event):
        self._trigger = True

    def PH_precalc(self, event):
        # When triggered (by a setpoint change):
        #   - Reset the integration. It is as if the controller is
        #     starting over afresh (for integration)
        #   - integration is paused while the controller settles into the
        #     new regime. This is a variation on "windup protection"
        #     which has a similar goal.

        if self._trigger:
            event.pid.integration = 0
            self.pause_remaining = self.integration_pause
            self._trigger = False
            event.i = 0

        if self.pause_remaining > 0:
            self.pause_remaining -= event.pid.dt
            event.i = 0


class SetpointRamp(PIDModifier):
    """Add setpoint ramping (smoothing out setpoint changes) to a PID"""

    def __init__(self, secs, *args, **kwargs):
        """Smooth (i.e., ramp) setpoint changes over 'secs' seconds."""
        if secs < 0:                # NOTE: secs is allowed to be zero
            raise ValueError(f"ramp time (={secs}) must not be negative")

        super().__init__(*args, **kwargs)
        self._ramptime = secs
        self._noramp(setpoint=0)

    def _noramp(self, /, *, setpoint):
        """Returns state to 'no ramp in progress'."""
        self._start_sp = setpoint    # starting setpoint
        self._target_sp = setpoint   # ending setpoint
        self._countdown = 0          # time remaining in ramp

    # Enforce one PIDPlus per SetpointRamp (because stateful)
    # NOTE: This also establishes the .pid attribute (at attach time)
    PH_attached = PIDModifier._pid_limit_1

    # property because if it changes the ramping may have to be adjusted
    @property
    def secs(self):
        return self._ramptime

    @secs.setter
    def secs(self, v):
        self._ramptime = v
        try:
            midramp = self.pid.setpoint != self._target_sp
        except AttributeError:         # not attached yet
            midramp = False

        if midramp:
            # This arbitrarily defines the semantic of a mid-ramp secs change
            # to mean: continue whatever ramping remains, with that remaining
            # ramp spread out over the new secs
            self._start_sp = self.pid.setpoint   # i.e., ramp starts HERE ...
            self._countdown = v                  # ... NOW

            # but if it's been set to zero, just go there RIGHT NOW
            if v == 0:
                self.__set_real_setpoint(self._target_sp)

    def __repr__(self):
        return f"{self.__class__.__name__}({self.secs})"

    # If the setpoint gets changed via an initial_conditions method
    # then it happens immediately (no ramping).
    def PH_initial_conditions(self, event):
        # If the setpoint is unchanged (carried forward) it is None...
        sp = event.pid.setpoint if event.setpoint is None else event.setpoint
        self._noramp(setpoint=sp)

    def PH_setpoint_change(self, event):
        # NO-OP conditions: ramp parameter zero, or no change in sp
        if self._ramptime == 0 or event.sp_set == self._target_sp:
            return

        self._start_sp = event.sp_prev
        self._target_sp = event.sp_set
        self._countdown = self._ramptime
        clampf = min if self._start_sp < self._target_sp else max
        self._clamper = lambda x: clampf(x, self._target_sp)

    def __set_real_setpoint(self, v):
        """Set the real setpoint in the underlying pid, bypasing ramping."""

        # This try/finally block is necessary because there could be OTHER
        # modifiers with PH_setpoint_change handlers, so anything in terms
        # of exceptions is possible. Of course, things are probably woefully
        # awry if that happens, but, try to maintain sanity here anyway.
        try:
            saved_ramp, self._ramptime = self._ramptime, 0
            self.pid.setpoint = v
        finally:
            self._ramptime = saved_ramp

    def PH_precalc(self, event):
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
        if self._countdown <= event.pid.dt:
            # last tick; slam to _target_sp in case of any floating fuzz.
            self.__set_real_setpoint(self._target_sp)
            self._countdown = 0
        else:
            # Still ramping ... This is the normal ramping case but could
            # potentially also be the "one extra" tick mentioned above
            # in which case pcttime will be very very close to 1.00 and
            # the next tick after this will trigger the above branch
            self._countdown -= event.pid.dt
            pcttime = (self._ramptime - self._countdown) / self._ramptime
            totaldelta = (self._target_sp - self._start_sp)
            ramped = self._clamper(self._start_sp + (totaldelta * pcttime))
            self.__set_real_setpoint(ramped)


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

    def PH_postcalc(self, event):
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

    # can't re-use on multiple because track previous_e here
    PH_attached = PIDModifier._pid_limit_1

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

    # done as midcalc so it doesn't have to (re)compute e itself
    def PH_midcalc(self, event):
        if self.previous_e is None:     # special case for very first call
            event.d = 0
        elif self.kickticks == 0:
            event.d = (event.e - self.previous_e) / event.pid.dt
        else:
            self.kickticks -= 1
            event.d = self.previous_d
        self.previous_e = event.e
        self.previous_d = event.d


if __name__ == "__main__":
    import unittest
    import math
    from collections import Counter

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

        def test_setpointramp(self):
            ramptime = 17       # just to be ornery with division/fuzz
            setpoint = 5
            dt = 0.100

            p = PIDPlus(Kp=1, modifiers=SetpointRamp(ramptime))
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
            ramper = SetpointRamp(4)
            p = PIDPlus(Kp=1, modifiers=ramper)
            p.initial_conditions(pv=0, setpoint=0)
            p.setpoint = 100

            # in two 1-second intervals it should get halfway there
            p.pid(0, dt=1)
            p.pid(0, dt=1)
            self.assertTrue(math.isclose(p.setpoint, 50))

            # now if change secs to 5, should go by 10 more each time...
            ramper.secs = 5
            for i in range(5):
                p.pid(0, dt=1)
                expected = 50 + ((i + 1) * 10)
                with self.subTest(sp=p.setpoint, i=i):
                    self.assertTrue(math.isclose(p.setpoint, expected))

        def test_spramp3(self):
            # test that fussing with the secs parameter midramp works
            # but in this case testing "aborting the ramp" by setting secs=0
            ramper = SetpointRamp(4)
            p = PIDPlus(Kp=1, modifiers=ramper)
            p.initial_conditions(pv=0, setpoint=0)
            p.setpoint = 100

            # in two 1-second intervals it should get halfway there
            p.pid(0, dt=1)
            p.pid(0, dt=1)
            self.assertTrue(math.isclose(p.setpoint, 50))

            # now if change secs to 0, should ramp immediately to target
            ramper.secs = 0
            p.pid(0, dt=1)
            with self.subTest(sp=p.setpoint):
                self.assertTrue(math.isclose(p.setpoint, 100))

        def test_sprampmonogamous(self):
            # this is really a test of _pid_limit_1 but hey, why not.
            ramper = SetpointRamp(17)
            z1 = PIDPlus(Kp=1, modifiers=ramper)
            with self.assertRaises(TypeError):
                _ = PIDPlus(Kp=2, modifiers=ramper)

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
            wlo = 1.0
            whi = 2.25
            windup_limits = [whi, wlo]   # deliberately "backwards"
            dt = 0.1
            p = PIDPlus(Ki=1, modifiers=I_Windup(windup_limits))
            p.initial_conditions(pv=0, setpoint=1)
            for i in range(int(whi / dt) + 1):
                p.pid(0, dt=dt)
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

        def test_readonlyvars(self):
            class Foo(PIDModifier):
                def PH_midcalc(self, event):
                    event.d = 17

                def PH_postcalc(self, event):
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

        def test_multi_readonly(self):
            # tests that the property magic for read-only attrs
            # works properly in the face of multiple hook objects
            # (this was once a bug)
            e1 = PIDHookPostCalc(pid='bozo', e=1)
            e2 = PIDHookPostCalc(pid='bonzo', e=2)
            self.assertEqual(e1.pid, 'bozo')
            self.assertEqual(e1.e, 1)
            self.assertEqual(e2.pid, 'bonzo')
            self.assertEqual(e2.e, 2)

        def test_pidreadonly(self):
            # .pid should be readonly even if RW_VARS was '*'
            ebozo = PIDHookPreCalc(pid='bozo', e=1)
            with self.assertRaises(TypeError):
                ebozo.pid = 'bonzo'
            self.assertEqual(ebozo.pid, 'bozo')

        def test_attrpropagation(self):
            # Modifiers can add more attributes during the pre/mid/post
            # calculation events (that might be a bug?) ... test this.
            class FooMod(PIDModifier):
                counter = 0

                def PH_precalc(self, event):
                    event.foo = self.counter
                    self.counter += 1

                def PH_postcalc(self, event):
                    if event.foo != self.counter - 1:
                        raise ValueError("XXX")

            z = PIDPlus(modifiers=FooMod())
            z.pid(1, dt=0.01)         # that this doesn't bomb is the test

        def test_readme_1(self):
            # just testing an example given in the README.md
            class SetpointPercent(PIDModifier):
                def PH_setpoint_change(self, event):
                    event.sp_now = event.sp_set / 100

            z = PIDPlus(Kp=1, modifiers=SetpointPercent())
            z.setpoint = 50
            self.assertEqual(z.setpoint, 50/100)

        def test_readme_2(self):
            # just testing another example fromthe README.md
            class UBash(PIDModifier):
                def PH_precalc(self, event):
                    event.u = .666

            z = PIDPlus(Kp=1, modifiers=UBash())
            self.assertEqual(z.pid(0, dt=0.01), 0.666)

        def test_readme_3(self):
            # yet another README.md example test
            class SetpointPercent(PIDModifier):
                def PH_precalc(self, event):
                    event.e = (event.pid.setpoint / 100) - event.pid.pv

            z = PIDPlus(Kp=1, modifiers=SetpointPercent())
            z.setpoint = 50
            u = z.pid(0, dt=0.01)
            self.assertEqual(z.setpoint, 50)
            self.assertEqual(u, 50 / 100)

        def test_history_copy(self):
            # this in particular tests that the history captures the
            # state of the events AT THE TIME in THAT POSITION in the
            # modifications list. Basically it's a test that copy works.
            class Clown(PIDModifier):
                def __init__(self, name, *args, **kwargs):
                    super().__init__(*args, **kwargs)
                    self.clown = name

                def PH_setpoint_change(self, event):
                    event.sp_now = self.clown

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
            self.assertEqual(e1.sp_now, None)
            self.assertEqual(e2.sp_now, 'bozo')
            self.assertEqual(e3.sp_now, 'bonzo')
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

                def PH_hookstopped(self, event):
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
            class NotificationChecker(PIDModifier):
                def __init__(self, *args, **kwargs):
                    super().__init__(*args, **kwargs)
                    self.notifications = Counter()

                def _gotevent(self, event):
                    self.notifications[event.NOTIFYHANDLER] += 1

                # explicitly ignore these
                def PH_hookstopped(self, event):
                    pass

                def PH_default(self, event):
                    raise TypeError("DEFAULT HANDLER INVOKED")

            class Stopper(PIDModifier):
                def PH_default(self, event):
                    raise HookStop

            # arguably working way too hard at DRY; make all the counters
            # (handlers) by dynamically discovering NOTIFYHANDLER names
            def _all_notification_method_names(cls):
                for subcls in cls.__subclasses__():
                    yield from _all_notification_method_names(subcls)
                    try:
                        if subcls.NOTIFYHANDLER != 'PIDHookHookStopped':
                            yield subcls.NOTIFYHANDLER
                    except AttributeError:     # base class; possibly others
                        pass

            METHODNAMES = set(_all_notification_method_names(_PIDHookEvent))
            for nm in list(METHODNAMES):    # NOTE: modified within loop
                if hasattr(NotificationChecker, nm):
                    METHODNAMES.remove(nm)
                else:
                    setattr(
                        NotificationChecker, nm, NotificationChecker._gotevent)

            # so the idea of this test is to make a PID controller with
            # several copies of the above notification count modifier, with
            # the last one preceded by a HookStop emitter. Then put this
            # mess through operations known/expected to result in
            # every METHODNAMES entry being called exactly once. Of course
            # the last nchk will have zero counts; the rest should have 1
            # for every possible NOTIFYHANDLER name.

            nchks = [NotificationChecker() for i in range(3)]
            stopper = Stopper()
            last = NotificationChecker()
            nchks += [stopper, last]

            z = PIDPlus(Kp=1, modifiers=nchks)
            z.setpoint = 17
            z.pid(0, dt=.01)
            expected = 1
            for i, nchk in enumerate(nchks):
                if nchk is stopper:
                    with self.subTest(m=m, note="LAST"):
                        self.assertEqual(last.notifications[m], 0)
                    expected = 0
                else:
                    for m in METHODNAMES:
                        with self.subTest(m=m, i=i):
                            self.assertEqual(nchk.notifications[m], expected)
                    for v in nchk.notifications:
                        with self.subTest(v=v):
                            self.assertTrue(v in METHODNAMES)

        def test_hh(self):
            # a test of HookStop from naughty HookStop handlers
            class Stopper(PIDModifier):
                def PH_default(self, event):
                    foo = event
                    hh_depth = 0
                    while isinstance(foo, PIDHookHookStopped):
                        hh_depth += 1
                        foo = foo.event

                    try:
                        self.maxrecursion = max(self.maxrecursion, hh_depth)
                    except AttributeError:
                        self.maxrecursion = hh_depth
                    raise HookStop

            mods = [Stopper() for _ in range(10)]
            z = PIDPlus(modifiers=mods)
            for i, m in enumerate(mods):
                self.assertEqual(i, m.maxrecursion)

    unittest.main()

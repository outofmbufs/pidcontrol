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

from collections import deque, ChainMap
from itertools import filterfalse
from contextlib import contextmanager


class PID:
    """Simple PID control."""

    def __init__(self, /, *, Kp=0, Ki=0, Kd=0):
        """Create a PID controller with the given parameters.

        Kp, Ki, Kd        -- weights for P/I/D control signals
        """

        # P/I/D weighting parameters. Note: they can't all be zero.
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        if (self.Kp, self.Ki, self.Kd) == (0, 0, 0):
            raise ValueError("Kp/Ki/Kd must not all be zero")

        # if different initial conditions are desired, callers should
        # call initial_conditions themselves after this initialization.
        self.initial_conditions(pv=0, setpoint=0)

    # the advantage of calling this, rather than bashing pv and setpoint
    # directly, is that this will also reset various state variables
    # and avoid any "kick" from the instantaneous pv/setpoint changes.
    # This becomes even more important in PIDPlus where other features
    # will also need resetting (e.g., setpoint ramping).
    #
    def initial_conditions(self, /, *, pv=None, setpoint=None):
        """Establish initial conditions -- resets state accordingly."""

        self.dt = 0          # corresponding dt (between pid() calls)
        if setpoint is not None:
            self.setpoint = setpoint

        if pv is not None:
            self.integration = 0        # For I: the current integral
            self.previous_pv = pv       # For D: previous process variable

            self.pv = pv          # observed process variable value

    def pid(self, pv, /, *, dt=None):
        """Return the new commanded control value for the most recent pv/dt.

        If dt is omitted the previous dt will be re-used.
        NOTE: Calling this with a zero dt will fail w/ZeroDivisionError.
              Startup sequence should be something like:
                  z = PID(Kp=foo, Ki=bar, Kd=baz)
                  z.pv = current process value    # this line is optional
                  wait some amount of time (=dt)
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


#
# A PIDPlus is a PID that supports "modifiers" (PIDModification subclasses)
# which can enhance/alter the base PID calculations and outputs. Modifiers
# can provide setpoint change ramping; integration windup protection, etc.
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

        # force the mods to be a sequence, possibly zero length
        # Note: the arg can be a naked PIDModifier if only one is being used
        #       (force that into a tuple of length 1)
        if modifiers is None:
            self.modifiers = tuple()
        elif isinstance(modifiers, PIDModifier):
            self.modifiers = (modifiers,)
        else:
            self.modifiers = modifiers

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
        """Return Info with the new control variable value."""

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
        cx = PIDHookMidCalc(clone=cx).notify(self.modifiers)

        # -- POST --
        # Generally 'u' calculation should be overridden in a PostCalc.
        # See, for example, BangBang. But allow for it in Pre/Mid by
        # testing for it. By far the common case is cx.u is still None here.
        if cx.u is None:
            cx.u = self._u(cx.p, cx.i, cx.d)
        # again note the vars from the MidCalc are cloned to the PostCalc
        cx = PIDHookPostCalc(clone=cx).notify(self.modifiers)

        # Whatever 'u' came through all that ... that's the result!
        return cx.u

    def findmod(self, modclass):
        for m in self.modifiers:
            if isinstance(m, modclass):
                return m


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

    def __init__(self, /, *, clone=None, **kwargs):
        # clone (if supplied) vars override kwargs, which in turn
        # override DEFAULTED_VARS (usually supplied via subclassing).
        # ChainMap that all up and set the attributes.
        if clone is not None:
            cv = {a: getattr(clone, a) for a in vars(clone)}
        else:
            cv = {}
        for k, v in ChainMap(cv, kwargs, self.DEFAULTED_VARS).items():
            setattr(self, k, v)

    def notify(self, modifiers):
        """Invoke the notification handler for all modifiers.

        For notational convenience, returns self, for 1-liners like:
            event = PIDHookSomething(foo).notify(modlist)
        when the returned event is needed even after the notify()
        """

        for m in modifiers:
            try:
                # invoke the NOTIFYHANDLER or the PH_default.
                # One or the other needs to be present in every modifier.
                # NOTE: The base PIDModifier defines a (no-op) PH_default.
                getattr(m, self.NOTIFYHANDLER, getattr(m, 'PH_default'))(self)

            # A modifier can raise HookStop if it wants to abort
            # sending this to further modifiers coming after it.
            except HookStop:
                break

        return self            # notational convenience

    def __repr__(self):
        s = self.__class__.__name__ + f"(pid={self.pid!r}"
        for a in filterfalse(lambda x: x == 'pid', vars(self)):
            s += f", {a}={getattr(self, a)!r}"
        return s + ")"


# with the above __init__ and notify as a framework, typically each
# specific event is simply a subclass with a class attribute to
# define its NOTIFYHANDLER method name and (if needed) the DEFAULTED_VARS.
# Conceptually the NOTIFYHANDLER could have been programmatically
# determined from the subclass name, but "explicit is better" won out here.

class PIDHookInitialConditions(_PIDHookEvent):
    NOTIFYHANDLER = 'PH_initial_conditions'
    DEFAULTED_VARS = dict(setpoint=None)


class PIDHookSetpoint(_PIDHookEvent):
    NOTIFYHANDLER = 'PH_setpoint_change'
    DEFAULTED_VARS = dict(sp_now=None)


# generic base for PreCalc/Mid/Post. Establishes the event default vars.
class _PIDHook_Calc(_PIDHookEvent):
    DEFAULTED_VARS = dict(e=None, p=None, i=None, d=None, u=None)


class PIDHookPreCalc(_PIDHook_Calc):
    NOTIFYHANDLER = 'PH_precalc'


class PIDHookMidCalc(_PIDHook_Calc):
    NOTIFYHANDLER = 'PH_midcalc'


class PIDHookPostCalc(_PIDHook_Calc):
    NOTIFYHANDLER = 'PH_postcalc'


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


class PIDHistory(PIDModifier):
    """Adds a look-back record of control computations to a PID."""

    def __init__(self, n, /):
        """Adds a record of the past 'n' control computations to a PID."""
        self.history = deque([], n)

    # this _default method gets all events and logs them
    def PH_default(self, event):
        self.history.append(event)


class I_Windup(PIDModifier):
    """Enhances I control with "windup" limit."""

    def __init__(self, w, /):
        """The integration value will be held to [-w, w] range."""
        if w < 0:
            raise ValueError(f"windup ({w=}) must not be negative.")
        self.w = w

    def PH_midcalc(self, event):
        # windup limiting to range [-self.w , self.w]
        if self.w:
            clamped = max(-self.w, min(event.pid.integration, self.w))
            event.i = event.pid.integration = clamped


class I_SetpointReset(PIDModifier):
    """Reset integration, with optional pause, when setpoint changes."""

    def __init__(self, secs, /):
        """Resets integration on setpoint change, and pause it 'secs'.

        NOTE: 'secs' can be zero, in which case a setpoint change resets the
              integration, but restarts it immediately (i.e, 0 second pause).
        """
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

    def __init__(self, secs, /):
        """Smooth (i.e., ramp) setpoint changes over 'secs' seconds."""
        if secs < 0:
            raise ValueError(f"ramp time (={secs}) must not be negative")

        # allow zero secs to make life simpler for callers (if ramp time
        # is tweakable, avoid making zero a special case).
        self.ramptime = secs
        self._initialize_state(0)

    def PH_initial_conditions(self, event):
        # If the setpoint is unchanged (carried forward) it is None...
        sp = event.pid.setpoint if event.setpoint is None else event.setpoint
        self._initialize_state(sp)

    def _initialize_state(self, setpoint):
        self._start_sp = setpoint    # starting setpoint
        self._target_sp = setpoint   # ending setpoint
        self._countdown = 0          # time remaining in ramp

    def PH_setpoint_change(self, event):
        # NO-OP conditions: ramp parameter zero, or no change in sp
        if self.ramptime == 0 or event.sp_set == self._target_sp:
            return

        self._start_sp = event.sp_prev
        self._target_sp = event.sp_set
        self._countdown = self.ramptime
        clampf = min if self._start_sp < self._target_sp else max
        self._clamper = lambda x: clampf(x, self._target_sp)

    # this is a hack so that when the ramp logic needs to set the setpoint,
    # doing so doesn't trigger ANOTHER ramping and infinite regress
    @contextmanager
    def _bypass_ramping(self):
        saved_sphandler = self.PH_setpoint_change
        self.PH_setpoint_change = lambda x: None    # i.e., a no-op
        try:
            yield
        finally:
            self.PH_setpoint_change = saved_sphandler

    def PH_precalc(self, event):
        if self._countdown == 0:       # not currently ramping
            return

        self._countdown -= event.pid.dt

        if self._countdown < 0:
            # Done ramping; slam to _target_sp (in case of any floating fuzz)
            self._countdown = 0
            with self._bypass_ramping():
                event.pid.setpoint = self._target_sp
        else:
            # Still ramping ...
            pcttime = (self.ramptime - self._countdown) / self.ramptime
            totaldelta = (self._target_sp - self._start_sp)
            ramped = self._clamper(self._start_sp + (totaldelta * pcttime))
            with self._bypass_ramping():
                event.pid.setpoint = ramped


#
# Convert a PID into a "bangbang" control.
#
class BangBang(PIDModifier):
    """Implement bang-bang control."""
    def __init__(self, /, *,
                 on_threshold=0, off_threshold=0,
                 on_value=1, off_value=0,
                 dead_value=None):
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

    unittest.main()

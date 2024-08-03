# pidcontrol
A simple PID controller, in python.

See [Wikipedia PID](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) for a complete explanation of PID controllers.

## Creating a controller
To create a simple P/I/D (proportional/integral/derivative) controller:

    from pid import PID
    z = PID(Kp=foo, Ki=bar, Kd=baz)

where foo/bar/baz (Kp=/Ki=/Kd=) are the gain constants for the respective control calculations.

## Terminology

Refer to this drawing:

![](wikipid.png)

from the Wikipedia PID article.

Attribution: [Arturo Urquizo](https://commons.wikimedia.org/wiki/File:PID.svg), [CC BY-SA 3.0](https://creativecommons.org/licenses/by-sa/3.0/), via Wikimedia Commons


- **controlled-device**: The overall mechanism being controlled. "Plant/Process" in the above picture. It has one input, `u(t)` or the "control variable" and one output, `y(t)` or the "process variable". For example, the control variable might be the position of a valve, and the output variable might be a measured resulting temperature. 
- **process-variable** or `y(t)`: The measured value from the controlled device.
- **setpoint** or `r(t)`: The target ("reference") value for the process-variable.
- **control-variable** or `u(t)`: The value computed by the PID controller and sent to the controlled device.

The units of `u(t)` and `y(t)` are often different; the gain constants Kp/Ki/Kd are therefore application-specific in their general order of magnitude. For example, in an automobile cruise-control application the process variable is the speed in mph. The setpoint, which always has the same units as the process variable, would also be in mph. However, the control variable will be something such as "percentage implied application of the accelerator pedal."


## Picking the gain constants ("tuning")
Choosing reasonable values for Kp/Ki/Kd is critical for proper operation; however, it is highly application-specific and hard to give general advice. Refer to the Wikipedia article for several references to tuning methods.


## Running a control loop
The method `pid` takes two arguments, a current process variable value and a time interval (the delta-t between this pv measurement and prior). In a typical use the interval is constant, so in pseudo-code a common coding idiom is:

    z = PID(Kp=foo, Ki=bar, Kd=baz)     # constants need to be determined
    interval = 0.1                      # 100msec

    loop: execute once every 'interval' seconds:
        pv = ... read process value from the controlled device ...
        cv = z.pid(pv, dt=interval)
        ... send cv to the controlled device ...

## Ti, Td vs Ki, Kd

Support for the `Ti` ("integration time") and `Td` ("derivative time") form of control constants is not provided. If using Ti/Td is preferred, perform this conversion before constructing the PID:

    Ki = Kp / Ti
    Kd = Kp * Td


## Derivative Term Calculation
The PID() class calculates the derivative term based on the change of the process variable, not the change of the computed error (difference between setpoint and process variable). In applications where the setpoint never changes, the two computation methods are identical. In applications where the setpoint can change from time to time, using the computed error results in a one-interval spike (so-called "derivative-kick") in the D calculation; whereas using the process variable instead avoids this kick.

See `PIDPlus` and the `D_DeltaE` PIDModifier class if, for some reason,
it is desirable to use the delta-e calculation instead of delta-pv despite the derivative-kick problem.


## Public Attributes and Methods

- *object*.**initial_conditions(pv=None, setpoint=None)**: Call this to establish initial conditions for the process variable and/or the setpoint. Using this function to establish those initial values will prevent any false "kick" in the control variable signal from an implicitly-spurious instantaneous pv change and/or setpoint change.

- *object*.**setpoint**: Public attribute. The setpoint. Can be set directly any time the desired reference value changes; however, see the `initial_conditions` method for a discussion on how to best set this at startup time.

- *object*.**pid(pv, dt)**: Perform a PID calculation given a current process-variable value and a delta-t (in seconds, floating point) for the interval between calls. Returns the new control-variable (`u(t)`) value.

- *object*.**last_pid**: Public attribute. A tuple containing the last three, unweighted, values for the Proportional ("error"), Integral, and Derivative control variables.


## Simple PID() Example

See pidexample.py for an example. Try running it like this:

   python pidexample.py --Kp=70 --Ti=1.25 --Td=0.5 -n=500


## PID Algorithm Addons: PIDPlus class

The simple PID algorithm works well enough in most use cases. For others, the class `PIDPlus` provides several customizations (called "modifiers") and a framework for adding more.

The following features are available as `PIDPlus` modifiers:

- **Setpoint Ramping**: This converts an abrupt setpoint change into a series of smaller setpoint changes interpolated (i.e., "ramped") over a configurable time period. Can be helpful in some types of controlled systems.

- **Integration windup protection**: A large setpoint change (or other dynamic conditions) that creates a large error term can cause excessive accumulation in the integration term. The excess will persist (and distort control output) until there has been sufficient cumulative time spent with an opposite error. Among other problems, this can cause overshoot and a slower return to the commanded setpoint. This excessive accumulation is (sometimes) called *integral windup*. Windup protection allows an absolute limit to be set on the integration term value, thus limiting the amount of windup possible. Setting this value correctly requires application-specific knowledge; in particular note that low values will limit the control authority range of the integration term (this is both the point, and the peril, of windup limiting).

- **Integration reset and pause**: When the setpoint is changed, it may be desirable to reset the integration term back to zero, and optionally cause the integration accumulation to pause for a little while for the other controls to settle into a steadier state. This is another approach to mitigating the same type of problem that windup protection attempts to solve. This solution is close to emulating a new cold-start of the controller with a new setpoint. Note that in some systems the integration term is, in effect, a dynamically-discovered "bias" value (minimum control value needed for equilibrium). In such systems using this modifier can make things worse, not better; obviously this is application-specific.

- **History Recording**: This modifier doesn't affect any algorithm operation but provides a lookback of controller computations. Can be useful during tuning and debugging.

- **Bang Bang**: This is probably rarely a useful modification; it was implemented primarily as a test of the PIDModifier system to see how far customization features could be pushed. This alters the behavior of the PID controller such that the control variable is always returned as a fully "on" value or a fully "off" value.

- **Use Delta E for derivative**: This changes the algorithm so that the derivative term is computed based on the slope of the error term rather than the slope of the process variable. There is likely no real-world use for this plug-in but it is there so the two approaches can be compared.

## Using PIDPlus with Modifiers

To create a PIDPlus control object, begin by creating a sequence (usually list or tuple) of PIDModifier objects desired. Each PIDModifier object represents a specific behavior modification with specific parameters. These objects are then passed in to PIDPlus along with the standard PID parameters (Kp/Kd/Ki).

Details shown below, but first an example:

    from pid import PIDPlus, PIDHistory
    
    # initialize a PIDHistory 'modifier' with 100-entry lookback
    h = PIDHistory(100)
    
    # now make a modifier list (in this case there is only one modifier)
    mods = [ h ]

    # make the PIDPlus
    z = PIDPlus(Kp=foo, Ki=bar, Kd=baz, modifiers=mods)

    ... do the normal things with z

    # The history collected by PIDHistory is available as an
    # attribute of the PIDHistory object:
    print(h.history)              # for example, to see the record

As a convenience, if there is only one modifier it can be supplied by itself instead of being in a list:

    h = PIDHistory(100)
    z = PIDPlus(Kp=foo, Ki=bar, Kd=baz, modifiers=h)


The specific PIDModifier class names and __init__ signatures are:

### EventPrint

This is a trivially-simple modifier that will print out events as they occur. It can be helpful in debugging and also for learning about how the event system works when writing a custom PIDModifier.

Example:

    from pid import PIDPlus, EventPrint
    z = PIDPlus(modifiers=EventPrint())

will immediately print:

    PIDHookAttached()
    PIDHookInitialConditions(setpoint=0, pv=0)

### SetpointRamp

Example usage - to cause all modifications of z.setpoint to be ramped in over a period of 1.5 seconds:

    ramper = SetpointRamp(1.5)

and `ramper` must (of course) be given to PIDPlus in the modifiers list.

By default, during the ramp the value of z.setpoint will be seen to change as the setpoint is being ramped in. For example:

    z = PIDPlus(Kp=1, modifiers=SetpointRamp(5))
    # the initial setpoint = 0, so this starts ramping from there
    z.setpoint = 4.0
    for i in range(5):
        cv = z.pid(pv=0, dt=1)
        print(f"{cv=}, {z.setpoint=}")

This will print:

    cv=0.8, z.setpoint=0.8
    cv=1.6, z.setpoint=1.6
    cv=2.4, z.setpoint=2.4
    cv=3.2, z.setpoint=3.2
    cv=4.0, z.setpoint=4.0

which shows the setpoint ramping 1/5th of the way on each call (dt=1) and the returned control variable clearly being calculated off of that ramping setpoint (Kp=1).

For most use-cases this is the best way to handle setpoint ramping as it makes the ramping explicitly visible. However, it is possible to hide the ramping with `hidddenramp=True`:

    z = PIDPlus(Kp=1, modifiers=SetpointRamp(5, hiddenramp=True))
    z.setpoint = 4.0             # starts ramping from zero
    for i in range(5):
        cv = z.pid(pv=0, dt=1)
        print(f"{cv=}, {z.setpoint=}")

This will print:

    cv=0.8, z.setpoint=4.0
    cv=1.6, z.setpoint=4.0
    cv=2.4, z.setpoint=4.0
    cv=3.2, z.setpoint=4.0
    cv=4.0, z.setpoint=4.0

showing that the control variable is still being computed from a ramping setpoint, but hides that ramping and always shows `z.setpoint` at the final target value. Whether this is a good idea or not is application-specific, especially regarding how other modifiers (if any) may behave with respect to hidden or not hidden ramping.

It is possible to change the ramp time in an active controller by changing the `secs` attribute:

    ramper.secs = 2.5

would change the ramp time to 2.5 seconds. If a ramping is already in progress when this is done, a new ramp is calculated from the then-current (partly-ramped) setpoint to the already-established target, at a rate dictated by the new ramp time. If the new ramp time is zero, the setpoint is immediately changed to that target.

NOTE that the ramp time is defined in terms of `pid()` calls and the `dt` argument. In other words, "1.5 seconds" doesn't mean, necessarily, 1.5 seconds of real time. It means *over a series of `pid()` calls until the sum of the `dt` values provided in those calls reaches or exceeds 1.5*.

It may be desirable to skip ramping for small setpoint changes. The keyword-only argument `threshold` allows for this. It defines an amount of setpoint change (in either positive or negative direction) that will be allowed immediately rather than ramped. The default value of `threshold` is zero, meaning all changes are ramped. Note that `threshold` does not affect the setpoint changes made by SetpointRamp itself (which operates "behind the scene" so that its own changes to the setpoint are, of course, never recursively ramped).

### Windup Protection

To limit the (unweighted!) integration term to +/- x:

    w = I_Windup(x)

To limit the (unweighted!) integration term to a closed asymmetric range [lo, hi]:

    w = I_Windup(lo, hi)

It is also allowed to specify one argument, a tuple:

    w = I_Windup((lo, hi))

Note that the order of the two values (`lo` and `hi`) does not matter; they will be sorted so that the smaller value will be the low limit and the larger value will be the high limit.

In normal use cases the two limits should have opposite signs; however, this is not enforced. If, for example:

    w = I_Windup(6, 9)

then at startup time unless the first pid() call already gets the integration value all the way up to 6, the integration value will jump up to 6. It works analogously if both limits are negative.


### Integration reset and delay on setpoint change

To cause the integration term to be reset to zero on any setpoint change:

    m = I_SetpointReset(0)

This causes the term to be reset, but sets a zero second delay for resumption of integration. To implement a delay of x seconds:

    m = I_SetpointReset(x)

NOTE: There is no way to ONLY implement a delay, without a reset. Though, of course, users can write their own additional PIDModifier subclasses.

### History

The PIDHistory modifier keeps a `deque` (i.e, a FIFO list) of PIDHookEvents.

    m = PIDHistory()
    z = PIDPlus(modifiers=m)
    for e in m.history:
        print(e)

will print:

    PIDHookAttached()
    PIDHookInitialConditions(setpoint=0, pv=0)

Note that str() representations of events are simplified; try this same example using `print(f"{e!r}")` for more event object details.

The maximum events stored in the FIFO `deque` defaults to 1000 but can be given as an argument:

    m = PIDHistory(100)

will limit the deque to 100 entries.

Entries are available via the `.history` attribute, which is a sequence. The zeroth element is the oldest.

It is possible, but likely a bad idea, to keep infinite history via None:

    m = PIDHistory(None)

in which case the .history sequence will grow without bound.

By default, `PIDHistory` just records the event notifications, but it can optionally include all of the `pid` object state in those records as well. To enable this use `detail=True`:

    m = PIDHistory(100, detail=True)

will keep 100 entries, and each entry will have an extra attribute, `pidinfo`, which will contain a dictionary copy of `vars(pid)`.

A `PIDHistory` also counts event occurrences, by handlername(). The counts are in the `.eventcounts` attribute:

    m = PIDHistory()
    z = PIDPlus(modifiers=m)
    print(m.eventcounts)

will print:

    Counter({'PH_attached': 1, 'PH_initial_conditions': 1})

showing the count of events (1 `PH_attached` and 1 `PH_initial_conditions`) that occur when creating the PIDPlus object. Note that these counts are for events _generated_ regardless of whether any modifier had a handler for them.


### Bang Bang

This one is complicated; quoting from the __init__ signature and doc string:

    def __init__(self, /, *,
                 on_threshold=0, off_threshold=0,
                 on_value=1, off_value=0,
                 dead_value=None):

    Threshold semantics are:
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


It implements a bang-bang controller with an optional middle "dead" zone (which is really a third potential control variable value). It is not likely to be useful but does demonstrate the capabilities of the PIDModifier system.

### D_DeltaE

There are two forms. The simplest is:

    m = D_DeltaE()

There is an option for a 1-interval kick filter which ameliorates derivative-kick on setpoint changes. To request that feature be turned on:

    m = D_DeltaE(kickfilter=True)

### Putting It All Together - PIDPlus with multiple modifiers

Here is a code example for a PIDPlus that uses setpoint ramping, windup limit, and the PIDHistory feature:

    ramp = SetpointRamp(1.5)      # ramp over 1.5 seconds
    windup = I_Windup(4.2)        # limit integration to [-4.2, 4.2]     
    h = PIDHistory(1000)          # keep 1000 history records
    
    z = PIDPlus(Kp=foo, Ki=bar, Kd=baz, modifiers=[ramp, windup, h])

Note that the order of the modifiers in that list is significant: they will be activated in that order each time z.pid() calculates values. This is irrelevant for most of the modifiers; however, it may be important to think about where to put the PIDHistory modifier depending on what it is you are trying to see. The PIDHistory modifier is actually one that can be meaningfully instantiated multiple times, thus:

    ramp = SetpointRamp(1.5)      # ramp over 1.5 seconds
    windup = I_Windup(4.2)        # limit integration to [-4.2, 4.2]     
    h1 = PIDHistory(1000)         # keep 1000 history records
    h2 = PIDHistory(1000)         # also keep 1000 history records
    
    z = PIDPlus(Kp=foo, Ki=bar, Kd=baz, modifiers=[h1, ramp, windup, h2])

In this example, h1.history would have the records from BEFORE the ramp and windup handlers ran; h2.history would have the records from AFTER. Instantiating a PIDPlus with multiple objects of other modifiers (e.g., two SetpointRamp modifiers) is not generally useful (and the exact behavior may be implementation-dependent).

It also, of course, possible to encapsulate all of this into a PIDPlus subclass. For example:

    class RampWindupHistory(PIDPlus):
        def __init__(self, *args, ramptime=1.5, windup=4.2, **kwargs):
            # Make the modifiers easily available as attributes
            self.ramp = SetpointRamp(ramptime)
            self.windup = I_Windup(windup)
            self.h = PIDHistory()

            mods = (self.ramp, self.windup, self.h)
            super().__init__(*args, modifiers=mods, **kwargs)

which simplifies usage (with the default parameters) to:

    z = RampWindupHistory()


## General Performance Notes
A `PIDPlus` can be used with no modifiers:

    z = PIDPlus(Kp=foo, Ki=bar, Kd=baz)

This is fine but runs slower than the equivalent `PID` would. To compare performance of `PID` vs a no-modifier `PIDPlus`, the following tests were run:

    % python3 -mtimeit -s'from pid import PID; z = PID()' 'z.pid(0, dt=0.01)' 
    1000000 loops, best of 5: 285 nsec per loop

This is about 0.3 microseconds per `pid()` call.

    % python3 -mtimeit \
      -s'from pid import PIDPlus; z = PIDPlus()' 'z.pid(0, dt=0.01)'
    20000 loops, best of 5: 13.2 usec per loop

Although the no-modifier `PIDPlus` could be said to be roughly 45 times slower than the corresponding `PID`, the total overhead of roughly 13 microseconds per iteration seems unlikely to be significant overhead in the context of any system that can be pragmatically controlled via a python program.

Adding a PIDHistory, which is one of the more expensive modifiers (as it responds to ALL events):


    % python3 -mtimeit \
       -s'from pid import PIDPlus, PIDHistory' \
       -s'h = PIDHistory(); z=PIDPlus(modifiers=h)' 'z.pid(0, dt=0.01)' 
    20000 loops, best of 5: 18.1 usec per loop

indicates roughly 5usec additional overhead per modifier added.

## Writing a PIDModifier

A custom PIDModifier interacts with one or more PIDHookEvent event notifications by providing a handler method with a specific name (as shown below) for that notification.

The event classes and their corresponding handler names are:

| Event Class              | Handler Method Name   |
| :----------------------- | :-------------------- |
| PIDHookAttached          | PH_attached           |
| PIDHookInitialConditions | PH_initial_conditions |
| PIDHookSetpointChange    | PH_setpoint_change    |
| PIDHookBaseTerms         | PH_base_terms         |
| PIDHookModifyTerms       | PH_modify_terms       |
| PIDHookCalculateU        | PH_calculate_u        |
| PIDHookHookStopped       | PH_hook_stopped       |
| * (see discussion) *     | PH_default            |


In general a modifier should be a subclass of PIDModifier and provide methods `PH_something` for the names corresponding to events it wants to handle. The method signatures are always:

    def PH_foo(self, event):
        code handling the foo event (a PIDHookFoo object) goes here

For example:

    class ExampleModifier(PIDModifier):
        def PH_setpoint_change(self, event):
            print(event)

    z = PIDPlus(modifiers=ExampleModifier())
    z.setpoint = 7

will print:

    PIDHookSetpointChange(sp_now=None, sp_prev=0, sp_set=7)

A modification can declare its own `__init__` method if it needs additional parameters. Best practice includes using *args/**kwargs and super() to continue the `__init__` calls up the subclass chain. So, as a trivial example, to add a 'foo' parameter to the ExampleModifier shown above:

    class ExampleModifierFoo(PIDModifier):
        def __init__(self, foo, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.foo = foo

        def PH_setpoint_change(self, event):
            print(f"'{self.foo}' : {event}")

    z = PIDPlus(modifiers=ExampleModifierFoo('bozo'))
    z.setpoint = 7

and this will print:

    'bozo' : PIDHookSetpointChange(sp_now=None, sp_prev=0, sp_set=7)

**PH_default**

A PIDModifier subclass can also choose to handle _all_ events by providing a `PH_default` method. NOTE: The `PH_default` method will only see events for events the subclass does not have an explicit handler for.

Thus, for example:

    class ExampleTwo(PIDModifier):
        def PH_setpoint_change(self, event):
            print(f"In PH_setpoint_change, got: {event.__class__.__name__}")

        def PH_default(self, event):
            print(f"In PH_default, got: {event.__class__.__name__}")
    z = PIDPlus(modifiers=ExampleTwo())
    z.setpoint = 7

will print:

    In PH_default, got: PIDHookAttached
    In PH_default, got: PIDHookInitialConditions
    In PH_setpoint_change, got: PIDHookSetpointChange

The `ExampleTwo` shows that two events, `PIDHookAttached` and `PIDHookInitialConditions`, are generated just by instantating a `PIDPlus`.

**Exception: Hookstop**

A handler can raise an exception, `HookStop`, to prevent handlers that come after it from seeing an event. See the `PIDHookHookStopped` description for more details on how this works.

**Event Attributes**

In these next sections each individual event and the semantics of what a handler can do with them are shown. In these descriptions, event attributes that are noted as READ-ONLY cannot be altered (attempting to modify them will raise an exception). Attributes that are READ-WRITE indicate ways that an event handler can alter the system semantics as described in each event.

Modifiers are also allowed to set entirely new attributes in an event. These will, of course, be ignored by the framework itself so is often pointless. However there are two ways it might be useful:
- During a `pid()` calculation there are three events generated: `PIDHookBaseTerms`, `PIDHookModifyTerms`, and `PIDHookCalculateU`. Any extra attributes that got added into the `PIDHookBaseTerms` will be copied to the `PIDHookModifyTerms` event, and similarly they (and any additional new attributes) will be copied to the `PIDHookCalculateU` event.

- If for some reason it is desirable to pass information to handlers that are later in the modifiers list. This might be helpful, for example, to allow internal attributes to propagate to the later PIDHistory modifier for logging/debugging.

As an example of setting attributes for logging purposes:


    class ExampleModifierFoo2(PIDModifier):
        def __init__(self, foo, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.foo = foo

        def PH_setpoint_change(self, event):
            # instead of: print(f"'{self.foo}' : {event}")
            # this does this:
            event.xyz = self.foo

    h = PIDHistory()
    f = ExampleModifierFoo2('bozo')
    z = PIDPlus(modifiers=[f, h])
    z.setpoint = 7
    print(h.history[-1])   # print the last (newest) entry

this will print

    PIDHookSetpointChange(sp_now=None, sp_prev=0, sp_set=7, xyz=bozo)

**Suggestion for learning/debugging**

An easy way to get a better feel for what the event stream does is to use the `EventPrint` modifier and just interactively try some examples. Here is a sample session:

    Python 3.12.3 (v3.12.3:f6650f9ad7, Apr  9 2024, 08:18:47
    Type "help", "copyright", "credits" or "license" for more information.

    >>> from pid import PIDPlus, EventPrint
    >>> z = PIDPlus(Kp=2, Ki=0.1, Kd=0.5, modifiers=EventPrint())
    PIDHookAttached()
    PIDHookInitialConditions(setpoint=0, pv=0)

    >>> z.initial_conditions(setpoint=5, pv=2)
    PIDHookInitialConditions(setpoint=5, pv=2)

    >>> cv = z.pid(2, dt=1)
    PIDHookBaseTerms(e=None, p=None, i=None, d=None, u=None)
    PIDHookModifyTerms(e=3, p=3, i=3, d=0.0, u=None)
    PIDHookCalculateU(e=3, p=3, i=3, d=0.0, u=6.3)

    >>> cv
    6.3

    >>>

See also PIDHistory which stores events for later analysis, rather than printing them.

**Details of each PIDHookEvent subclass and handlers**

Each event object and its semantics are described below.

### PIDHookAttached (handler method: PH_attached)

Generated **_DURING_** PIDPlus.__init__ for each modifier attached to that PIDPlus.

READ-ONLY ATTRIBUTES:
- `pid` -- the PIDPlus object. CAUTION: NOT FULLY INITIALIZED AT THIS POINT.

This notification happens before the PIDPlus object is fully set up; handlers should not attempt to look into the `event.pid` object. They can, of course, look **at** `event.pid` itself which is the reason for this notification.

The base PIDModifier class provides a method, `attached_once_check` that is useful for modifiers that need to limit themselves to one attachment (usually because they have pid-specific state); it will raise a TypeError if called more than once per modifier object.

Thus a modifier can limit itself to one pid like this:

    class OnlyOneAttach(PIDModifier):
        def PH_attached(self, event):
            # raises TypeError if called more than once
            self.attached_once_check(event)

    m = OnlyOneAttach()
    z1 = PIDPlus(modifiers=m)
    z2 = PIDPlus(modifiers=m)

will result in an exception:

    TypeError: multiple attachment attempted: '<__main__.OnlyOne>'

If the modifier itself doesn't need to do anything other than limit attachment it can simply set `PH_attached` to the attached_once_check method directly:

    class OnlyOne(PIDModifier):
        PH_attached = PIDModifier.attached_once_check

this will produce the same result as the explicit `PH_attached` handler shown above.

### PIDHookInitialConditions (handler method: PH_initial_conditions)

Generated when the `initial_conditions` method is invoked on a PIDPlus object.

READ-ONLY ATTRIBUTES:
- `pid` -- the PIDPlus object
- `setpoint` -- the `setpoint` argument specified in the corresponding `initial_conditions()` method invocation. NOTE: Can be None (meaning: no setpoint change).
- `pv` -- the `pv` argument; can be None.

As noted above, this event is generated **after** the initial_conditions method has modified the `pid` object. If it makes sense to allow a modifier to be "reset" midstream, the initializations necessary to start afresh should go in a `PH_initial_conditions` handler.

### PIDHookSetpointChange: (handler method: PH_setpoint_change)

Generated when the setpoint attribute is set on a PIDPlus object. a

NOTE: This event is NOT generated if the setpoint change happens as part of an initial_conditions() call.

This event is generated BEFORE any modifications in the underlying PIDPlus object occur. This gives the handler the opportunity to affect the setpoint change as described with the attributes below.

There is one read/write attribute and the rest are read-only.

READ-WRITE ATTRIBUTES:
- `sp_now` -- If the handler sets this attribute to something other than None (its default), then this value is used to set the setpoint. 

READ-ONLY ATTRIBUTES:
- `pid` -- the PIDPlus object
- `sp_prev` -- the setpoint value prior to any modification.
- `sp_set` -- the setpoint value that was requested to be set by the application (see discussion below). 

As noted above, this event is generate **before** the assignment to the setpoint attribute. This allows handlers to change the value that gets assigned, by setting `sp_now` to something other than `sp_set` (`sp_set` itself is read-only)

As a trivial example:

    class SetpointPercent(PIDModifier):
        def PH_setpoint_change(self, event):
            event.sp_now = event.sp_set / 100

This would allow users to use "percentages" for the setpoint, e.g.:

    z = PIDPlus(Kp=1, modifiers=SetpointPercent())
    z.setpoint = 50
    print(z.setpoint)

This will print 0.50 as the resulting setpoint (whether it is a good idea to implement something like this is up for debate, but it's a simple example). Another example later will show another (better) way to accomplish this without the mismatch between written and read .setpoint values.

### PIDHookBaseTerms (handler name: PH_base_terms)
Generated at the start of the calculations related to a `pid()` call.

READ-WRITE ATTRIBUTES:
- `e` -- the error value to use. Default: None.
- `p` -- the (unweighted) proportional term value to use. Default: None.
- `i` -- the (unweighted) integral term value to use. Default: None.
- `d` -- the (unweighted) derivative term value to use. Default: None
- `u` -- the control value to ultimately return. Default: None

READ-ONLY ATTRIBUTES:
- `pid` -- the PIDPlus object

The idea of this event is that the framework is about to calculate `e`, `p`, `i`, and `d` the usual way; these are the so-called _base_ terms. However, a modifier can substitute an alternate value for any of those terms; it does so by setting the corresponding attribute in this event to something other than None. Doing that for `e` will override the internal _error() calculation and this `e` value will be passed to the other p/i/d internal methods. Setting any of `p`, `i`, or `d` to not-None will similarly bypass the corresponding internal method for that term and substitute the value given. Setting `u` to anything not-None will short-circuit the whole shebang and cause that value to be the final control value returned by `.pid()`. 

Consider this trivial example:

    class UBash(PIDModifier):
        def PH_base_terms(self, event):
            event.u = .666

    z = PIDPlus(Kp=1, modifiers=UBash())
    print(z.pid(0, dt=0.01)

This will print 0.666 even though the control variable would otherwise normally be calculated to be "0" in this situation as the pv is equal to the (default) setpoint and only Kp is non-zero.

Bashing `u` this early in the sequence is unlikely to be useful, but later in the sequence (i.e, in a `PH_compute_u` handler) `u` could be usefully modified.

Given this hook point, here is a better way to implement "setpoint as a percent"that was shown with `PIDHookSetpointChange`. This modifier sets the `e` value, which is normally computed as:

    e = setpoint - pv

but this overrides it as shown, to allow the setpoint to be scaled up by 100 (i.e., expressed as a percent) in the attribute, but be treated as a value between 0 and 1 in the error term calculation:

    class SetpointPercent(PIDModifier):
        def PH_base_terms(self, event):
            event.e = (event.pid.setpoint / 100) - event.pid.pv

The underlying PIDPlus code only uses the setpoint in the `e` calculation, so by doing this calculation here the setpoint can be treated as scaled by 100.

### PIDHookModifyTerms (handler name: PH_modify_terms)

This event occurs immediately after all the base terms are established, whether they were obtained by the built-in calculations or from values provided by `PH_base_terms` handlers.

READ-WRITE ATTRIBUTES:
- `p` -- the current candidate (unweighted) proportional term value. NEVER None.
- `i` -- the current (unweighted) candidate integral term value. NEVER None.
- `d` -- the current (unweighted) candidate derivative term value. NEVER None.
- `u` -- the control value to ultimately return. Default: None

READ-ONLY ATTRIBUTES:
- `e` -- the current candidate error value. Will NEVER be None.
- `pid` -- the PIDPlus object

Essentially this gives handlers of this event a second swing at the parameters, with a full view of all values whether supplied by the internal calculations or by `PH_base_terms` handlers. The `p`, `i`, and `d` attributes will never be None here (as the framework will fill them in if they were not provided in a `PH_base_term`). The `u` parameter will normally be None, as it is a rare use-case for a `PH_base_terms` to want to supply a meaningful `u` at that stage; but, theoretically, it could be non-None here. A `PH_modify_terms` handler can also modify `u` here, though it may be more typical to do it in the PIDHookCalculateU stage which is explicitly set up for that.

The `e` term can not be modified here as it has already been used by this point.

### PIDHookCalculateU (handler name: PH_calculate_u)
The last of the three events during `.pid()` calculation. At this stage a candidate `u` value has been computed from the p/i/d terms. Only the `u` value can be overridden at this stage; the remaining attributes are read-only.

READ-WRITE ATTRIBUTES:
- `u` -- the candidate control value to ultimately return. Will NEVER be None.

READ-ONLY ATTRIBUTES:
- `pid` -- the PIDPlus object
- `e` -- for reference: the error value. NEVER None.
- `p` -- for reference: the (unweighted) proportional term value. NEVER None.
- `i` -- for reference: the (unweighted) integral term value. NEVER None.
- `d` -- for reference: the (unweighted) derivative term value. NEVER None.

### PIDHookHookStopped (handler name: PH_hook_stopped)

If a PIDModifier raises HookStop then the event in question (the _stopped_ event) will not be propagated to the remaining modifiers further down the list. Instead, a PIDHookHookStopped event will be sent to those modifiers.

READ-ONLY ATTRIBUTES:
- `event`-- The _stopped_ event.
- `stopper` -- The PIDModifier object that raised HookStop.
- `nth` -- The position of `stopper` in the modifiers list, which may be helpful in disambiguating where the HookStop happened if the modifier itself appears in multiple places within the list (none of the built-in modifiers raises HookStop, and only PIDHistory really makes sense to appear as a single object in multiple locations within the modifiers list, but this disambiguation is provided regardless)
- `modifiers` -- the modifiers list against which `nth` can be interpreted.

Notes:
- It is legal, but almost certainly questionable, for a PH_hook_stopped handler to raise HookStop. This does not cause infinite recursion because this new PIDHookHookStopped event will be sent only to the modifiers that come after the one (recursively) raising HookStop. Each layer of such recursion shortens the remaining modifiers list by 1; therefore there is no infinite recursion.

- Similarly, it is legal, but almost certainly questionable, for a PH_hook_stopped handler to try to circumvent the HookStop by peering into the `event` attribute; note that because recursion is allowed such code needs to be able to follow the entire event.event.event... chain until it gets down to the original (i.e., non-recursive) event.


# pidcontrol
A simple PID controller, in python.

See [Wikipedia PID](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) for a complete explanation of PID controllers.

## Creating a controller
To create a simple P/I/D (proportional/integral/derivative) controller:

    from pid import PID
    p = PID(Kp=foo, Ki=bar, Kd=baz)

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

### SetpointRamp

Example usage - to cause all modifications of z.setpoint to be ramped in over a period of 1.5 seconds:

    m = SetpointRamp(1.5)

and m must (of course) be given to PIDPlus in the modifiers list.

By default, during the ramp the value of z.setpoint will be seen to change as the setpoint is being ramped in. For example:

    z = PIDPlus(Kp=1, modifiers=SetpointRamp(5))
    z.setpoint = 4.0             # starts ramping from zero
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

    m.secs = 2.5

would change the ramp time to 2.5 seconds. If a ramping is already in progress when this is done, a new ramp is calculated from the then-current (partly-ramped) setpoint to the already-established target, at a rate dictated by the new ramp time. If the new ramp time is zero, the setpoint is immediately changed to that target.

NOTE that the ramp time is defined in terms of `pid()` calls and the `dt` argument. In other words, "1.5 seconds" doesn't mean, necessarily, 1.5 seconds of real time. It means *over a series of `pid()` calls until the sum of the `dt` values provided in those calls reaches or exceeds 1.5*.

It may be desirable to skip ramping for small setpoint changes. The keyword-only argument `threshold` allows for this. It defines an amount of setpoint change (in either positive or negative direction) that will be allowed immediately rather than ramped. The default value of `threshold` is zero, meaning all changes are ramped. Note that `threshold` does not affect the setpoint changes made by SetpointRamp itself (which operates "behind the scene" so that its own changes to the setpoint are, of course, never recursively ramped).

### Windup Protection

To limit the (unweighted!) integration term to +/- x:

    m = I_Windup(x)

To limit the (unweighted!) integration term to a closed asymmetric range [lo, hi]:

    m = I_Windup(lo, hi)

It is also allowed to specify one argument, a tuple:

    m = I_Windup((lo, hi))

Note that the order of the two values (`lo` and `hi`) does not matter; they will be sorted so that the smaller value will be the low limit and the larger value will be the high limit.

In normal use cases the two limits should have opposite signs; however, this is not enforced. If, for example:

    m = I_Windup(6, 9)

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

Note that str() rebpresentations of events are simplified; try this same example using `print(f"{e!r}")` for more event object details.

The maximum events stored in the FIFO `deque` defaults to 1000 but can be given as an argument:

    m = PIDHistory(100)

will limit the deque to 100 entries.

Entries are available via the `.history` attribute, which is a sequence. The zeroth element is the oldest.

It is possible, but likely a bad idea, to keep infinite history via None:

    m = PIDHistory(None)

in which case the .history sequence will grow without bound.

A `PIDHistory` also counts event occurrences, by handlername(). The counts are in the `.eventcounts` attribute:

    m = PIDHistory()
    z = PIDPlus(modifiers=m)
    print(m.eventcounts)

will print:

    Counter({'PH_attached': 1, 'PH_initial_conditions': 1})

showing the count of events (1 `PH_attached` and 1 `PH_initial_conditions`) that occur simply from creating the PIDPlus object.


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
    
    p = PIDPlus(Kp=foo, Ki=bar, Kd=baz, modifiers=[h1, ramp, windup, h2])

In this example, h1.history would have the records from BEFORE the ramp and windup handlers ran; h2.history would have the records from AFTER. Instantiating a PIDPlus with multiple objects of other modifiers (e.g., two SetpointRamp modifiers) is not generally useful (and the exact behavior may be implementation-dependent).

## General Performance Notes
A `PIDPlus` can be used with no modifiers:

    p = PIDPlus(Kp=foo, Ki=bar, Kd=baz)

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

A custom PIDModifier interacts with one or more PIDHookEvent notifications by providing a handler method for that notification. There are 7 PIDHookEvent types, each with a specific notification method name as noted here:

- PIDHookAttached: generated at the **end** of PIDPlus.__init__ for each modifier attached to that PIDPlus. PIDModifier method name: `PH_attached` . The significance of this being "at the end of" PIDPlus.__init__ is that the PIDPlus will be fully-initialized when this notification happens.

- PIDHookInitialConditions: generated when the 'initial_conditions()' method is invoked on a PIDPlus object. PIDModifier method name: `PH_initial_conditions` .

- PIDHookSetpointChange: generated when the setpoint attribute is set on a PIDPlus object. PIDModifier method name: `PH_setpoint_change` .

- PIDHookBaseTerms, PIDHookModifyTerms, PIDHookCalculateU: generated at three different stages of the PID calculation. These three spots offer different ways of modifying the PID control calculation. PIDModifier method names: `PH_base_terms`, `PH_modify_terms`, `PH_calculate_u`.

- PIDHookHookStopped: generated when any PIDModifier raises HookStop to prevent further propagation of an individual PIDHook event. This at least lets modifiers later in the list know that they were cut off from an event. PIDModifier method name: `PH_hook_stopped`.

In addition to the specific method names mentioned above,  each notification subclass defines its own 'event' object which will be passed to those handler methods. The specific event objects will be described later.

To create a new modification type, subclass PIDModifier and supply a handler for every event of interest. For example, to create a PIDModifier that will interact with PIDHookSetpointChange events and PIDHookModifyTerms events:

    class ExampleModification(PIDModifier):
        def PH_setpoint_change(self, event):
            print(f"setpoint_change event: {event}")

        def PH_modify_terms(self, event):
            print(f"modify_terms event: {event}")

A modification can declare its own __init__ method if it needs additional parameters. Best practice includes using *args/**kwargs and super() to continue the __init__ calls up the subclass chain. So, as a trivial example, to add a 'foo' parameter to the ExampleModification:

    class ExampleModification(PIDModifier):
        def __init__(self, foo, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.foo = foo

        def PH_setpoint_change(self, event):
            print(f"setpoint_change event: {event}, foo: {foo}")

        def PH_modify_terms(self, event):
            print(f"modify_terms event: {event}, foo: {foo}")

which is the same as the first example but now includes a 'foo' parameter that presumably would be used for something in a real example.

A PIDModifier can also define a catch-all handler, called `PH_default` . It receives notifications this PIDModifier doesn't otherwise handle explicitly. As an example, here is a simplified PIDHistory modifier:

    class PIDHistory(PIDModifier):
        def __init__(self, n, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.history = deque([], n)

        # this _default method gets all events and stores them
        def PH_default(self, event):
            self.history.append(copy.copy(event))

This works by creating a deque (from collections) in the PIDHistory object, and simply entering (a copy of) every event (caught by `PH_default`) into it. The reason for copying the event is because modifiers further down the chain can alter `event` values. For the purpose of the history modifier the desired semantic is to record the values at the time they were seen by the history module; hence the event is copied.

As will be described next, each event contains its own set of attributes specific to that event. Some of the attributes are read-only, emphasizing that changing them will have no effect on the PIDPlus operation. Others are read-write and changing them will have event-specific semantics as discussed with each event below. Modifiers _can_ set additional attributes in an event, though this might be a questionable practice. Such additional attributes will be read/write but (of course) will be ignored by the built-in machinery.

The specific events, and associated semantics, for each PIDHook event are:

**PIDHookAttached**:
Event is generated at the very end of PIDPlus initialization, so the underlying PIDPlus object is "ready to go" when this notification is received. See "Shared Modifiers" discussion for one way to use this event.

HANDLER SIGNATURE: `PH_attached(self, event)`

READ-ONLY ATTRIBUTES:
- `pid` -- the PIDPlus object

**PIDHookInitialConditions**:
Event is generated AFTER the `initial_conditions()` method has completed modifying the PIDPlus object. The intent of this event is to allow PIDModifier implementations a hook for (re)initializing their own state if the application uses the `initial_conditions` method to change pv and/or the setpoint.

HANDLER SIGNATURE: `PH_initial_conditions(self, event)`

READ-ONLY ATTRIBUTES:
- `pid` -- the PIDPlus object
- `setpoint` -- the `setpoint` argument specified in the corresponding `initial_conditions()` method invocation. NOTE: Can be None (meaning: no setpoint change).
- `pv` -- the `pv`argument; can be None.

As noted above, this event is generated **after** the initial_conditions method has modified the `pid` object. 

**PIDHookSetpointChange**:
Event is generated BEFORE any modifications in the underlying PIDPlus object occur. This gives the handler the opportunity to affect the setpoint change.

There is one read/write attribute and the rest are read-only.

HANDLER SIGNATURE: `PH_setpoint_change(self, event)`

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

**PIDHookBaseTerms**:
This event is generated at the very beginning of the calculations implied by invoking the `.pid()` method. It contains 1 read-only attribute and 5 read-write attributes:

HANDLER SIGNATURE: `PH_base_terms(self, event)`

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

Given this hook point, here is a better way to implement "setpoint as a percent" ... this modifier sets the `e` value, which is normally computed as:

    e = setpoint - pv

but this overrides it as shown, to allow the setpoint to be scaled up by 100 (i.e., expressed as a percent) in the attribute, but be treated as a value between 0 and 1 in the error term calculation:

    class SetpointPercent(PIDModifier):
        def PH_base_terms(self, event):
            event.e = (event.pid.setpoint / 100) - event.pid.pv

The underlying PIDPlus code only uses the setpoint in the `e` calculation, so by doing this calculation here the setpoint can be treated as scaled by 100.

**PIDHookModifyTerms**:
This event occurs immediately after all the base terms are established, whether they were obtained by the built-in calculations or from values provided by `PH_base_terms` handlers.

HANDLER SIGNATURE: `PH_modify_terms(self, event)`

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

**PIDHookCalculateU**:
The last of the three events during `.pid()` calculation. At this stage a candidate `u` value has been computed from the p/i/d terms. Only the `u` value can be overridden at this stage; the remaining attributes are read-only.

HANDLER SIGNATURE: `PH_calculate_u(self, event)`

READ-WRITE ATTRIBUTES:
- `u` -- the candidate control value to ultimately return. Will NEVER be None.

READ-ONLY ATTRIBUTES:
- `pid` -- the PIDPlus object
- `e` -- for reference: the error value. NEVER None.
- `p` -- for reference: the (unweighted) proportional term value. NEVER None.
- `i` -- for reference: the (unweighted) integral term value. NEVER None.
- `d` -- for reference: the (unweighted) derivative term value. NEVER None.

**PIDHookHookStopped**:
If a PIDModifier raises HookStop then the event in question (the _stopped_ event) will not be propagated to the remaining modifiers further down the list. Instead, a PIDHookHookStopped event will be sent to those modifiers.

HANDLER SIGNATURE: `PH_hook_stopped(self, event)`

READ-ONLY ATTRIBUTES:
- `event`-- The _stopped_ event.
- `stopper` -- The PIDModifier object that raised HookStop.
- `nth` -- The position of `stopper` in the modifiers list, which may be helpful in disambiguating where the HookStop happened if the modifier itself appears in multiple places within the list (none of the built-in modifiers raises HookStop, and only PIDHistory really makes sense to appear as a single object in multiple locations within the modifiers list, but this disambiguation is provided regardless)
- `modifiers` -- the modifiers list against which `nth` can be interpreted.

Notes:
- It is legal, but almost certainly questionable, for a PH_hook_stopped handler to raise HookStop. This does not cause infinite recursion because this new PIDHookHookStopped event will be sent only to the modifiers that come after the one (recursively) raising HookStop. Each layer of such recursion shortens the remaining modifiers list by 1; therefore there is no infinite recursion.

- Similarly, it is legal, but almost certainly questionable, for a PH_hook_stopped handler to try to circumvent the HookStop by peering into the `event` attribute; note that because recursion is allowed such code needs to be able to follow the entire event.event.event... chain until it gets down to the original (i.e., non-recursive) event.

## Shared Modifiers
Some modifiers can be meaningfully shared among multiple PIDPlus controllers, because they have no implicit/internal state. A good example is `I_Windup` which simply performs a (stateless) calculation on the integration value it is supplied via a notification event.

Some modifiers have state but still can be meaningfully shared; a good example is the PIDHistory modifier. If a single PIDHistory is attached to multiple PIDPlus control objects, the history will contain an interwoven list of events (each which will have the .pid attribute to disambiguate which controller it came from).

Some modifiers cannot be meaningfully shared because they have controller-specific internal state. The SetpointRamp modifier is a good example of this. Such modifiers should generally be coded to raise an exception if sharing is attempted. The `PH_attached` notification can be used for this; here is a trivial example that is a no-op but does not allow itself to be attached more than once:

    class AttachToJustOne(PIDModifier):
        def PH_attached(self, event):
            try:
                if self.__attached_to != event.pid:
                    raise TypeError("multiple attachment attempted")
            except AttributeError:
                self.__attached_to = event.pid


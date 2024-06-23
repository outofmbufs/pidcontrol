# pidcontrol
A simple PID controller, in python.

See, for example, [Wikipedia PID](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller) for more explanation.

## Creating a controller
To create a simple P/I/D (proportional/integral/derivative) controller:

    from pid import PID
    p = PID(Kp=foo, Ki=bar, Kd=baz)

where foo/bar/baz (Kp=/Ki=/Kd=) are the gain constants for the respective control calculations.

## Terminology

- **controlled-device**: The overall mechanism being controlled. It has an input which is the `control-variable`, a desired result which is the `setpoint`, and a current measured condition which is the `process-variable`.
- **process-variable**: The measured value from the controlled device. Wikipedia (referenced above) calls this `y(t)`, a function of time as it varies thoughout operation of the device.
- **setpoint**: The target value for the process-variable. Wikipedia calls this `r(t)`.
- **control-variable**: The value that is computed by the PID controller and gets sent to the controlled device. Wikipedia calls this `u(t)`.

Note that sometimes the units of the control variable are different from the units of the process variable. For example, in an automobile cruise-control application, the process variable is the speed in mph. The setpoint, which always has the same units as the process variable, would also be in mph. However, the control variable is "percentage implied application of the accelerator pedal" or something along those lines.

The Kp/Ki/Kd gain values will be specific to each application and could vary widely in order-of-magnitude from one application to another depending on the relationship of the process and control variables.


## Running a control loop
The method `pid` takes two arguments, a current process variable value and a time interval (the delta-t between this pv measurement and prior). In a typical use the interval is constant, so in pseudo-code a common coding idiom is:

    p = PID(Kp=foo, Ki=bar, Kd=baz)     # constants need to be determined
    interval = 0.1                      # 100msec

    loop: execute once every 'interval' seconds:
        pv = ... read process value from the controlled device ...
        cv = p.pid(pv, dt=interval)
        ... send cv to the controlled device ...

## Ti, Td vs Ki, Kd

No support for the "integration time" and "derivative time" form of control constants is provided, even though in general those are the much more convenient form for tuning and understanding a PID controller. Calling code can easily provide the conversion itself:

    Ki = Kp / Ti
    Kd = Kp * Td

if if wants to allow users to specify those constants in their time form. Note that in this form there is no explicit way to set Ki to zero (which is one reason the PID class only supports the Kx forms; the other reason being stone-cold simplicity and laziness).

## Derivative Term Calculation
The PID() class calculates the derivative term based on the change of the process variable, not the change of the computed error (difference between setpoint and process variable). If the setpoint never changes, the two computation methods are identical. If the setpoint changes, there will be a one-interval spike (so-called "derivative-kick") in the D calculation; using the process variable instead avoids this kick.

See `PIDPlus` and the `D_DeltaE` PIDModifier class if, for some reason,
it is desirable to use the delta-e calculation instead of delta-pv.


## Public Attributes and Methods
In these examples, p is a PID() object.

- **p.initial_conditions(pv=None, setpoint=None)**: Call this to establish initial conditions for the process variable and/or the setpoint. Using this function to establish those initial values will prevent any false "kick" in the control variable signal from an implicitly-spurious instantaneous pv change and/or setpoint change.

- **p.setpoint**: Public attribute. The setpoint. See `initial_conditions` for establishing this at startup time as an initialization step, but if the setpoint in the controlled device can change over time, that is done by simply assigning to the `setpoint` attribute.


## Simple PID() Example

See pidexample.py for an example. Try running it like this:

   python pidexample.py --Kp=70 --Ti=1.25 --Td=0.5 -n=500


## PID Algorithm Addons: PIDPlus class

The `PID` class implements zero options for algorithm enhancements. There are, however, several modifications to the simple PID algorithm that are sometimes helpful or even necessary. To support those, use the `PIDPlus` class and supply one or more of the `PIDModifiers` as described here.

The following features are available as "modifiers":

- **Setpoint Ramping**: Any time p.setpoint is changed, instead of the change taking effect immediately it will be ramped-in over a configurable time period. In applications where the setpoint does indeed change midstream, this may be useful in reducing or eliminating abrupt control change behavior (especially overshoot).

- **Integration windup protection**: A large setpoint change (or other dynamic conditions) can cause a large integration term to accumulate; it may take a long time (and distort the controller behavior) for that to be worked off with a corresponding cumulative error in the oppsite direction. This is sometimes called "windup". Windup protection allows an absolute limit to be set on the integration term value.

- **Integration reset and pause**: When the setpoint is changed, it may be desirable to reset the integration term back to zero, and optionally cause the integration accumulation to pause for a little while for the other controls to settle into a steadier state. This is another approach to mitigating the same type of problem that windup protection attempts to solve.

- **History Recording**: This modifier doesn't affect any algorithm operation but provides a lookback of controller computations. Can be useful during tuning and debugging.

- **Bang Bang**: This is probably rarely a useful modification; it was implemented primarily as a test of the PIDModifier system to see how far customization features could be pushed. This alters he behavior of the PID controller such that the control variable is always returned as a fully "on" value or a fully "off" value.

- **Use Delta E for derivative**: This changes the algorithm so that the derivative term is computed based on the slope of the error term rather than the slope of the process variable. There is likely no real-world use for this plug-in but it is there so the two approaches can be compared.

## Using PIDPlus with Modifiers

Each PIDModifier is a class. To use a modifier, an object is instantiated and this is where modifier-specific arguments are provided. One or more of these objects is then passed to PIDPlus via the `modifiers` argument; the result is a PIDPlus controller object that will behave according to the underlying PID algorithm as modified by the list of provided modifiers.

The __init__ arguments for each PIDModifier are specific below, but as a way of introductory example consider this code:

    from pid import PIDPlus, PIDHistory
    
    # initialize a PIDHistory 'modifier' with 1000-entry lookback
    h = PIDHistory(1000)
    
    # now make the PIDPlus with this modifier
    # Note in this case there is only one modifier and it can
    # be supplied directly; if there were multiple they can
    # instead be supplied as 'modifiers=[modifier_1, modifier_2, etc]'
    p = PIDPlus(Kp=foo, Ki=bar, Kd=baz, modifiers=h)

    ... do the normal things with p

    # The history collected by PIDHistory is available as an
    # attribute of the object:
    print(h.history)              # for example, to see the record

The specific modifier class names and __init__ signatures are:

### SetpointRamp

Example usage - to cause all modifications of p.setpoint to be ramped in over a period of 1.5 seconds:

    m = SetpointRamp(1.5)

and m must (of course) be given to PIDPlus in the modifiers list.

### Windup Protection

To limit the (unweighted!) integration term to +/- x:

    m = I_Windup(x)

### Integration reset and delay on setpoint change

To cause the integration term to be reset to zero on any setpoint change:

    m = I_SetpointReset(0)

This causes the term to be reset, but sets a zero second delay for resumption of integration. To implement a delay of x seconds:

    m = I_SetpointReset(x)

NOTE: There is no way to ONLY implement a delay, without a reset. Though, of course, users can write their own additional PIDModifier subclasses.

### History

This was already shown in the introductory example. The PIDHistory modifier keeps a `deque` (i.e, a FIFO list) of the PIDHookEvent stream which encapsulates all the internal computations and state changes of the PID system. The objects in this record have repr implementations that will print all the relevant variables so these can be sent directly to a logger, for example, if so desired.

To instantiate a history modifier to track the most recent 1000 entries:

    m = PIDHistory(1000)

and the entries will be found in m.history as a `deque` (which could potentially have fewer than N entries if it hasn't been fully populated yet).

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
    
    p = PIDPlus(Kp=foo, Ki=bar, Kd=baz, modifiers=[ramp, windup, h])

Note that the order of the modifiers in that list is significant: they will be activated in that order each time p.pid() calculates values. This is irrelevant for most of the modifiers; however, it may be important to think about where to put the PIDHistory modifier depending on what it is you are trying to see. The PIDHistory modifier is actually one that can be meaningfully instantiated multiple times, thus:

    ramp = SetpointRamp(1.5)      # ramp over 1.5 seconds
    windup = I_Windup(4.2)        # limit integration to [-4.2, 4.2]     
    h1 = PIDHistory(1000)         # keep 1000 history records
    h2 = PIDHistory(1000)         # also keep 1000 history records
    
    p = PIDPlus(Kp=foo, Ki=bar, Kd=baz, modifiers=[h1, ramp, windup, h2])

In this example, h1.history would have the records from BEFORE the ramp and windup handlers ran; h2.history would have the records from AFTER. Instantiating a PIDPlus with multiple objects of other modifiers (e.g., SetpointRamp) is not generally useful (and the exact behavior may be implementation-dependent).

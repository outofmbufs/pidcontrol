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

## Public Attributes and Methods
In these examples, p is a PID() object.

- **p.initial_conditions(pv=None, setpoint=None)**: Call this to establish initial conditions for the process variable and/or the setpoint. Using this function to establish those initial values will prevent any false "kick" in the control variable signal from an implicitly-spurious instantaneous pv change and/or setpoint change.

- **p.setpoint**: Public attribute. The setpoint. See `initial_conditions` for establishing this at startup time as an initialization step, but if the setpoint in the controlled device can change over time, that is done by simply assigning to the `setpoint` attribute.


## PID Algorithm Addons: PIDPlus class

The `PID` class implements zero options for algorithm enhancements. There are, however, several modifications to the simple PID algorithm that are sometimes helpful or even necessary. To support those, use the `PIDPlus` class and supply one or more of the `PIDModifiers` as described here.

XXX DOCUMENTATION IS TODO XXX

## EXAMPLE

See pidexample.py for an example. Try running it like this:

   python pidexample.py --Kp=70 --Ti=1.25 --Td==0.5 -n=500







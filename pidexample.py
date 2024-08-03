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

from pid import PID


# This is a simple simulation of a thrust device (e.g., a fan) constrained
# to go only straight up and down (e.g., on a pole-mounted slider).
# Something like this:
#
#              |
#              |
#             [=]
#              ^
#              |
#              |
#
# where '^' is the fan and '[=]' is the slider mounted to the pole
# (in front of the pole in this crude picture).
#
# Parameters:
#     Height of pole: H = 1m
#     slider mass: M = 1kg
#     fan max thrust: Smax = 12N (kg-m/s^2)
#     fan rpm: 0 .. 100%; thrust generated linearly
#
# Friction is assumed zero.
# The fan is assumed to respond instantly to rpm commands.
# Gravity (of course) is g = 9.8 m/s^2
#
# The goal of this example is to control the fan rpm to
# keep the slider halfway up the pole.
#

class FanSlider:
    g = 9.8           # m/s^2

    def __init__(self, /, *,
                 H=1.0,                        # m
                 Smax=12.0,                    # kg m/s^2
                 M=1.0,                        # kg
                 ):
        self.H = H
        self.M = M
        self.Smax = Smax

        self.percentrpm = 0        # commanded %RPM 0 .. 100
        self.position = 0.0
        self.velocity = 0.0

    def simulate(self, tick):
        # sanity check
        if self.percentrpm < 0 or self.percentrpm > 100:
            raise ValueError(f"percentrpm ({self.percentrpm}) <0 or >100")

        self.position_update(tick)

        # limit tests
        if self.position > self.H:
            raise ValueError("flew off the top!")

        sitting_on_bottom = self.position == 0 and self.velocity >= 0
        if self.position <= 0 and not sitting_on_bottom:
            raise ValueError("crashed into the bottom")

        self.velocity_update(tick)
        # if sitting on the bottom don't let gravity make velocity negative
        if sitting_on_bottom and self.velocity < 0:
            self.velocity = 0

    def position_update(self, tick):
        self.position += (self.velocity * tick)

    def velocity_update(self, tick):
        fan_thrust = (self.percentrpm / 100.0) * self.Smax
        fan_a = fan_thrust / self.M        # F = ma; a = F/m
        net_a = fan_a - self.g
        self.velocity += (net_a * tick)


if __name__ == "__main__":
    import time
    import argparse
    import tkinter as T
    from types import SimpleNamespace

    def main():
        args = processargs()

        f = FanSlider(Smax=args.fanthrust)
        if all(x is None
               for x in (args.Kp, args.Ki, args.Kd, args.Ti, args.Td)):
            print("Try --Kp=70 --Ti=1.25 --Td=0.5 -n=500")
            return
        z = make_pid_control(args)
        z.setpoint = args.setpoint
        runsim(z, f, args.n, args.interval)

    def processargs():
        def requires(args, a_name, b_name):
            """Argument 'a' if not None requires argument 'b' be not None."""
            a = getattr(args, a_name)
            b = getattr(args, b_name)
            if a is not None and b is None:
                raise ValueError(f"Argument '{a_name}' requires '{b_name}'")

        parser = argparse.ArgumentParser()
        parser.add_argument('--Kp', type=float, metavar='P_GAIN')

        g = parser.add_mutually_exclusive_group()
        g.add_argument('--Ki', type=float, metavar='I_GAIN')
        g.add_argument(
            '--Ti', type=float, metavar='I_TIMECONST',
            help="Alternative to Ki; integration time constant")

        g = parser.add_mutually_exclusive_group()
        g.add_argument('--Kd', type=float, default=None, metavar='D_GAIN')
        g.add_argument(
            '--Td', type=float, default=None,
            metavar='D_TIMECONST',
            help="Alternative to Kd; derivative time constant")

        parser.add_argument('--setpoint', type=float, default=0.5)
        parser.add_argument('-n', type=int, default=10000,
                            help="Number of intervals to run")
        parser.add_argument('--interval', type=float, default=0.05,
                            help="Interval length (seconds)")
        parser.add_argument('--fanthrust', type=float, default=12.0)
        args = parser.parse_args()
        requires(args, 'Ti', 'Kp')
        requires(args, 'Td', 'Kp')
        return args

    def runsim(z, f, n, interval):
        guivars = guisetup(z, f)

        for i in range(n):
            t0 = time.time()
            u = z.pid(f.position, dt=interval)
            f.percentrpm = min(max(u, 0), 100)
            f.simulate(interval)
            draw(guivars, f)
            while (t1 := time.time()) - t0 < interval:
                time.sleep(interval // 10)

    def make_pid_control(args):
        # if Ti was specified, compute Ki.
        if args.Ti is not None:
            args.Ki = args.Kp / args.Ti

        # similar dance with Td but calc is different
        if args.Td is not None:
            args.Kd = args.Kp * args.Td

        # note: turning None into 0 here
        return PID(Kp=args.Kp or 0, Ki=args.Ki or 0, Kd=args.Kd or 0)

    def guisetup(z, f):
        # this is quick and dirty and not at all generalized
        gv = SimpleNamespace()
        gv.mg = T.Tk()
        gv.width = 300
        gv.height = 800
        gv.w = T.Canvas(gv.mg, width=gv.width, height=gv.height)
        gv.w.pack()
        gv.pole_L = 500

        ctr_x = gv.width // 2
        gv.pole_bottom_y = gv.height - 200
        gv.pole_top_y = gv.pole_bottom_y - gv.pole_L

        # the pole
        gv.w.create_polygon(ctr_x-1, gv.pole_bottom_y,
                            ctr_x+1, gv.pole_bottom_y,
                            ctr_x+1, gv.pole_top_y,
                            ctr_x-1, gv.pole_top_y,
                            fill='#303010', tag='pole')

        # lines marking the setpoint
        sp_y = gv.pole_bottom_y - (gv.pole_L * (z.setpoint / f.H))
        sp_color = '#0000c0'
        gv.w.create_line(ctr_x-30, sp_y, ctr_x-10, sp_y,
                         fill=sp_color, tag='pole')
        gv.w.create_line(ctr_x+30, sp_y, ctr_x+10, sp_y,
                         fill=sp_color, tag='pole')

        gv.mg.update_idletasks()
        gv.mg.update()
        return gv

    def draw(gv, f):
        gv.w.delete('device')

        # the slider is drawn 13x9 (w x h) pixels, centered on the pole
        yoff = int((f.position / f.H) * gv.pole_L)
        ctr_x = gv.width // 2
        slider_bottom = gv.pole_bottom_y - (yoff - 4)

        gv.w.create_polygon(ctr_x-6, slider_bottom,
                            ctr_x+6, slider_bottom,
                            ctr_x+6, slider_bottom - 9,
                            ctr_x-6, slider_bottom - 9,
                            fill="#a0c000", tag='device')

        # the thrust triangle, 100 pixels high = 100%
        tytop = slider_bottom      # the top is the bottom of the slider
        tybottom = slider_bottom + (100 * (f.percentrpm / 100))
        gv.w.create_polygon(ctr_x, tytop,
                            ctr_x-4, tybottom,
                            ctr_x+4, tybottom,
                            fill="#ff0000", tag='device')
        gv.mg.update_idletasks()
        gv.mg.update()

    main()

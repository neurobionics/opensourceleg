"""
A basic motion script that moves the osl joints through their range of motion.
This script can be helpful when getting started to make sure the OSL is functional.

Kevin Best
Neurobionics Lab
Robotics Department
University of Michigan
October 26, 2023
"""

import numpy as np

from opensourceleg.osl import OpenSourceLeg
from opensourceleg.tools import units

osl = OpenSourceLeg(frequency=200)
osl.add_joint("knee", gear_ratio=9 * 83 / 18)
osl.add_joint("ankle", gear_ratio=9 * 83 / 18)


def make_periodic_traj_func(period, minimum, maximum):
    amplitude = (maximum - minimum) / 2
    mean = amplitude + minimum
    return lambda t: amplitude * np.cos(t * 2 * np.pi / period) + mean


ankle_traj = make_periodic_traj_func(10, -20, 20)
knee_traj = make_periodic_traj_func(10, 10, 90)

with osl:
    osl.home()
    input("Homing complete: Press enter to continue")
    osl.knee.set_mode(osl.knee.control_modes.position)
    osl.ankle.set_mode(osl.ankle.control_modes.position)
    osl.knee.set_position_gains(kp=5)
    osl.ankle.set_position_gains(kp=5)

    for t in osl.clock:
        osl.update()
        knee_setpoint = units.convert_to_default(knee_traj(t), units.position.deg)
        ankle_setpoint = units.convert_to_default(ankle_traj(t), units.position.deg)
        osl.knee.set_output_position(knee_setpoint)
        osl.ankle.set_output_position(ankle_setpoint)
        print(
            "Ankle Desired {:+.2f} rad, Ankle Actual {:+.2f} rad, Knee Desired {:+.2f} rad, Ankle Desired {:+.2f} rad".format(
                ankle_setpoint,
                osl.ankle.output_position,
                knee_setpoint,
                osl.knee.output_position,
            ),
            end="\r",
        )

print("\n")

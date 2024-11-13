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

from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.time import SoftRealtimeLoop
from opensourceleg.units import units

FREQUENCY = 200

knee = DephyActuator(
    tag="knee",
    firmware_version="7.2.0",
    port="/dev/ttyACM0",
    gear_ratio=9 * 83 / 18,
    frequency=FREQUENCY,
)

ankle = DephyActuator(
    tag="ankle",
    firmware_version="7.2.0",
    port="/dev/ttyACM1",
    gear_ratio=9 * 83 / 18,
    frequency=FREQUENCY,
)

acutators = [knee, ankle]

clock = SoftRealtimeLoop(dt=1 / FREQUENCY)


def make_periodic_trajectory(period, minimum, maximum):
    amplitude = (maximum - minimum) / 2
    mean = amplitude + minimum
    return lambda t: amplitude * np.cos(t * 2 * np.pi / period) + mean


ankle_traj = make_periodic_trajectory(10, -20, 20)
knee_traj = make_periodic_trajectory(10, 10, 90)

with knee, ankle:
    for actuator in acutators:
        actuator.home()

    input("Homing complete: Press enter to continue")

    knee.set_control_mode(knee.CONTROL_MODES.POSITION)
    ankle.set_control_mode(ankle.CONTROL_MODES.POSITION)
    knee.set_position_gains(kp=5)
    ankle.set_position_gains(kp=5)

    for t in clock:
        knee.update()
        ankle.update()

        knee_setpoint = units.convert_to_default(knee_traj(t), units.position.deg)
        ankle_setpoint = units.convert_to_default(ankle_traj(t), units.position.deg)

        knee.set_output_position(knee_setpoint)
        ankle.set_output_position(ankle_setpoint)

        print(
            f"Ankle Desired {ankle_setpoint:+.2f} rad, Ankle Actual {ankle.output_position:+.2f} rad, Knee Desired {knee_setpoint:+.2f} rad, Ankle Desired {knee.output_position:+.2f} rad",
            end="\r",
        )

print("\n")

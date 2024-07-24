import numpy as np

import opensourceleg.units.units as units
from opensourceleg.robots import OpenSourceLeg

osl = OpenSourceLeg(frequency=200)  # 200 Hz
osl.add_joint(gear_ratio=9.0)

stiffness = 20  # Nm/rad
damping = 20  # Nm/rad/s
equilibrium_angle = units.convert_to_default(45, units.position.deg)  # rad

with osl:

    osl.knee.set_mode(osl.knee.control_modes.impedance)
    osl.knee.set_joint_impedance(K=stiffness, B=damping)
    osl.knee.set_motor_position(osl.knee.motor_position + equilibrium_angle)

    for t in osl.clock:
        osl.log.info(osl.knee.motor_position)
        osl.update()

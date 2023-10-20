import opensourceleg.tools.units as units
from opensourceleg.osl import OpenSourceLeg

osl = OpenSourceLeg(frequency=200)  # 200 Hz
osl.add_joint(gear_ratio=9.0)

set_point = units.convert_to_default(45, units.position.deg)

with osl:
    osl.knee.set_mode(osl.knee.control_modes.position)
    osl.knee.set_motor_position(osl.knee.motor_position + set_point)

    for t in osl.clock:
        osl.log.info(osl.knee.motor_position)
        osl.update()

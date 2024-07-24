import opensourceleg.units.units as units
from opensourceleg.robots import OpenSourceLeg

osl = OpenSourceLeg(frequency=200)
osl.add_joint(gear_ratio=9.0)

with osl:
    osl.knee.set_mode(osl.knee.control_modes.position)

    for t in osl.clock:
        osl.log.info(
            units.convert_from_default(osl.knee.motor_position, units.position.deg)
        )
        osl.update()

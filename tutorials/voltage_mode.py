import opensourceleg.units as units
from opensourceleg.osl import OpenSourceLeg

osl = OpenSourceLeg(frequency=200)  # 200 Hz
osl.add_joint(gear_ratio=9.0)

with osl:
    osl.knee.set_mode(osl.knee.control_modes.voltage)

    for t in osl.clock:
        osl.knee.set_voltage(1000)  # mV
        osl.log.info(
            units.convert_from_default(osl.knee.motor_position, units.position.deg)
        )
        osl.update()

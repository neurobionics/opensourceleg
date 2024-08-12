from opensourceleg.robots import OpenSourceLeg
from opensourceleg.time import SoftRealtimeLoop

osl = OpenSourceLeg(frequency=200)  # 200 Hz
osl.add_joint(gear_ratio=9.0)

loop = SoftRealtimeLoop(dt = 1/200)

with osl:
    osl.knee.set_mode(osl.knee.control_modes.current)

    for t in loop:

        osl.knee.set_current(400)  # 400 mA
        osl.log.info(osl.knee.motor_position)
        osl.update()

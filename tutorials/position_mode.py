from opensourceleg.osl import OpenSourceLeg

osl = OpenSourceLeg(frequency=200)  # 200 Hz
osl.add_joint(gear_ratio=9.0)

osl.units["position"] = "deg"
osl.log.info(osl.units)

set_point = 50  # motor ticks

with osl:
    osl.knee.set_mode("position")

    osl.knee.set_motor_position(osl.knee.motor_position + set_point)

    for t in osl.clock:
        osl.log.info(osl.knee.motor_position)
        osl.update()

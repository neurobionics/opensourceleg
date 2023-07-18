from opensourceleg.osl import OpenSourceLeg

osl = OpenSourceLeg(frequency=200)
osl.add_joint(gear_ratio=9.0)

osl.units["position"] = "deg"
osl.log.info(osl.units)

set_point = 50

with osl:
    osl.knee.set_mode("position")

    for t in osl.clock:
        osl.knee.set_motor_position(osl.knee.motor_position + set_point)
        osl.log.info(osl.knee.motor_position)
        osl.update()

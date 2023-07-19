from opensourceleg.osl import OpenSourceLeg

osl = OpenSourceLeg(frequency=200)
osl.add_joint(gear_ratio=9.0)

osl.units["position"] = "deg"
osl.log.info(osl.units)

with osl:
    osl.knee.set_mode("voltage")

    for t in osl.clock:
        osl.knee.set_voltage(1000)
        osl.log.info(osl.knee.motor_position)
        osl.update()

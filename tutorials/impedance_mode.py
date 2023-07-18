from opensourceleg.osl import OpenSourceLeg

osl = OpenSourceLeg(frequency=200)
osl.add_joint(gear_ratio=9.0)

osl.units["position"] = "deg"
osl.log.info(osl.units)

stiffness = 0.05
damping = 0.005

set_point = 50


with osl:
    osl.knee.set_mode("impedance")
    osl.knee.set_motor_impedance(K=stiffness, B=damping)

    for t in osl.clock:
        osl.knee.set_motor_position(osl.knee.motor_position + set_point)
        osl.log.info(osl.knee.motor_position)
        osl.update()

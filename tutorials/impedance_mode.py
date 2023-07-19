from opensourceleg.osl import OpenSourceLeg

osl = OpenSourceLeg(frequency=200)
osl.add_joint(gear_ratio=9.0)

osl.units["position"] = "deg"
osl.log.info(osl.units)

test_stiffness_value = 20
test_damping_value = 20

set_point = 70

with osl:

    osl.knee.set_mode("impedance")
    osl.knee.set_joint_impedance(K=test_stiffness_value, B=test_damping_value)

    for t in osl.clock:

        osl.knee.set_motor_position(osl.knee.motor_position + set_point)
        osl.log.info(osl.knee.motor_position)
        osl.update()

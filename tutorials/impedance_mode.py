from opensourceleg.osl import OpenSourceLeg

osl = OpenSourceLeg(frequency=200)  # 200 Hz
osl.add_joint(gear_ratio=9.0)

osl.units["position"] = "deg"
osl.log.info(osl.units)

test_stiffness_value = 20  # Nm/rad
test_damping_value = 20  # Nm/rad/s

set_point = 50  # deg

with osl:

    osl.knee.set_mode("impedance")
    osl.knee.set_joint_impedance(K=test_stiffness_value, B=test_damping_value)
    osl.knee.set_motor_position(osl.knee.motor_position + set_point)

    for t in osl.clock:
        osl.log.info(osl.knee.motor_position)
        osl.update()

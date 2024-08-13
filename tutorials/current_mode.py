from opensourceleg.robots import OpenSourceLeg

osl = OpenSourceLeg(frequency=200)  # 200 Hz
osl.add_joint(gear_ratio=9.0)

with osl:
    osl.knee.set_mode(osl.knee.control_modes.current)

    for t in osl.clock:
        osl.knee.set_current(400)  # 400 mA
        osl.log.info(osl.knee.motor_position)
        osl.update()

from opensourceleg.osl import OpenSourceLeg
import numpy as np

if __name__ == "__main__":

    osl = OpenSourceLeg(frequency=200)
    osl.add_joint(name="knee", gear_ratio=41.4999)

    STIFFNESS = 50
    DAMPING = 400

    with osl:

        osl.home()
        osl.knee.set_mode(mode = osl.knee.control_modes.impedance)
        osl.update()

        current_position = osl.knee.output_position

        for t in osl.clock:
            osl.update()
            current_position = osl.knee.output_position

            STIFFNESS += 100
            osl.knee.set_impedance_gains(
                kp=40, 
                ki=400, 
                K=STIFFNESS, 
                B=DAMPING, 
                ff=128,
            )
            osl.knee.set_output_position(
                position = current_position + np.pi/12
            )

            osl.log.info(f"STIFFNESS: {STIFFNESS}, Motor Position: {osl.knee.output_position}, Motor Current: {osl.knee.motor_current}")
            if input("Would you like to continue? (y/n) ").lower() == "n":
                break


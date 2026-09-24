from opensourceleg.actuators.brushed import MaxonActuator
from opensourceleg.logging.logger import Logger
from opensourceleg.sensors.encoderCounter import LS7366R

TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY


def home_motor():
    homing_logger = Logger(log_path="./logs", file_name="home_motor")

    # initialize actuator and encodercounter
    actuator = MaxonActuator(frequency=FREQUENCY, offline=False)
    encoder_counter = LS7366R()

    # set up motor and start homing
    with actuator:
        actuator.set_motor_encoder(encoder_counter)
        actuator.position_control_config()

        if actuator._is_homed:
            homing_logger.info("Actuator already homed to 0% stiffness.")
        else:
            homing_logger.info("Actuator homing to 0% stiffness.")
            actuator.home(home_zero=True)
            encoder_counter.clear_counter()
            actuator._is_homed = True

            actuator.update()
            actuator.check_coupler_drift()
            homing_logger.info(
                f"Homing complete. position = "
                f"{actuator.cts_to_perc(actuator.motor_position_cts):.2f}% "
                f"({actuator.motor_position_mm:.3f} mm)"
            )


if __name__ == "__main__":
    home_motor()

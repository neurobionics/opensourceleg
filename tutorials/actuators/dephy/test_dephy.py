import time

import numpy as np

import opensourceleg.actuators.dephy as Dephy
from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.logging.logger import LOGGER
from opensourceleg.time import SoftRealtimeLoop

TIME_TO_STEP = 1.0
DT = 0.001
clock = SoftRealtimeLoop(dt=DT)


def main():
    actpack = Dephy.DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=9.0,
    )

    control_types = {
        "Current": test_current_control,
        "Position": test_position_control,
        "Voltage": test_voltage_control,
        "Torque": test_voltage_control,
    }

    with actpack:
        try:
            actpack.start()
        except OSError:
            LOGGER.exception(msg="OSError for actpack.start()")

    control_type = input("Please select a type of control:\n 1. Current\n 2. Position\n 3. Voltage\n 4. Torque\n")

    if control_type in control_types:
        control_types[control_type](actpack)
    else:
        print("Please enter a valid control type. Valid control types are 'Current', 'Position', 'Voltage', 'Torque'")

    LOGGER.close()
    actpack.stop()
    exit()


def test_current_control(actpack: Dephy.DephyActuator):
    t = clock.time()
    with actpack:
        try:
            # Set control mode to current
            actpack.set_control_mode(mode=CONTROL_MODES.CURRENT)
            # Set current gains
            actpack.set_current_gains()
            # Set motor current

            actpack.update()
            _prompt_to_continue(actpack, "current")

            current_t = clock.time()
            while (t - current_t) < 1:
                command_mcurrent = 2000
                actpack.set_motor_current(value=command_mcurrent)

                actpack.update()
                _log_current_state(actpack, "Current", command_mcurrent)

                time.sleep(DT)
                t = clock.time()

            command_mcurrent = 0
            actpack.set_motor_current(value=command_mcurrent)

        except KeyboardInterrupt:
            LOGGER.exception("KeyboardInterrupt while setting motor current")


def test_position_control(actpack: Dephy.DephyActuator):
    t = clock.time()
    with actpack:
        try:
            # Set control mode to position
            actpack.set_control_mode(mode=CONTROL_MODES.POSITION)
            # Set position gains
            actpack.set_position_gains()
            # Set motor position
            actpack.update()
            motor_position = actpack.motor_position
            _prompt_to_continue(actpack, "position")
            current_t = clock.time()
            while (t - current_t) < 5:
                command_mposition = motor_position + np.pi
                actpack.set_motor_position(value=command_mposition)
                actpack.update()
                _log_current_state(actpack, "Position", command_mposition)
                time.sleep(DT)
                t = clock.time()

            command_mposition = 0
            actpack.set_motor_position(value=command_mposition)
        except KeyboardInterrupt:
            LOGGER.exception("KeyboardInterrupt while setting motor position")


def test_voltage_control(actpack: Dephy.DephyActuator):
    t = clock.time()
    with actpack:
        try:
            # Set motor voltage
            actpack.update()
            _prompt_to_continue(actpack, "voltage")
            current_t = clock.time()
            while (t - current_t) < 2:
                command_voltage = 2500
                actpack.set_motor_voltage(value=command_voltage)
                actpack.update()
                _log_current_state(actpack, "Voltage", command_voltage)

                time.sleep(DT)
                t = clock.time()

            command_voltage = 0
            actpack.set_motor_voltage(value=command_voltage)

        except KeyboardInterrupt:
            LOGGER.exception("KeyboardInterrupt while setting motor voltage")


def test_torque_control(actpack: Dephy.DephyActuator):
    t = clock.time()
    with actpack:
        try:
            # Set control mode to torque
            actpack.set_control_mode(mode=CONTROL_MODES.TORQUE)
            # Set motor current
            actpack.update()
            _prompt_to_continue(actpack, "torque")

            current_t = clock.time()
            while (t - current_t) < 1:
                command_torque = 0.005
                actpack.set_motor_torque(value=command_torque)

                actpack.update()
                _log_current_state(actpack, "Torque", command_torque)

                time.sleep(DT)
                t = clock.time()

            command_torque = 0
            actpack.set_motor_torque(value=command_torque)

        except KeyboardInterrupt:
            LOGGER.exception("KeyboardInterrupt while setting motor torque")


def _prompt_to_continue(actpack: Dephy.DephyActuator, mode: str) -> None:
    """Prompt to continue into next control mode"""

    LOGGER.info(
        "".join(
            f"Time:{clock.time()}\t"
            + f"Motor Position: {actpack.motor_position}\t"
            + f"Motor Voltage: {actpack.motor_voltage}\t"
            + f"Motor Current: {actpack.motor_current}\t"
        )
    )
    input(f"Press enter to continue into {mode} control mode")
    return


def _log_current_state(actpack: Dephy.DephyActuator, mode: str, command: int):
    """Helper function to log the current state"""
    LOGGER.info(
        "".join(
            f"Time:{clock.time()}\t"
            + f"Command {mode} : {command}\t"
            + f"Motor Position: {actpack.motor_position}\t"
            + f"Motor Voltage: {actpack.motor_voltage}\t"
            + f"Motor Current: {actpack.motor_current}\t"
        )
    )


if __name__ == "__main__":
    main()

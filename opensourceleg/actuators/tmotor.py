from opensourceleg.actuators.base import (
    CONTROL_MODE_CONFIGS,
    CONTROL_MODES,
    MOTOR_CONSTANTS,
    ActuatorBase,
    ControlGains,
    ControlModeConfig,
)
from opensourceleg.actuators.decorators import (
    check_actuator_connection,
    check_actuator_open,
    check_actuator_stream,
)

from opensourceleg.time import SoftRealtimeLoop

import time

TMOTOR_ACTUATOR_CONSTANTS = MOTOR_CONSTANTS(
    MOTOR_COUNT_PER_REV=16384,
    NM_PER_AMP=0.1133,
    NM_PER_RAD_TO_K=0.0,  # TODO: Find value
    NM_S_PER_RAD_TO_B=0.0,  # TODO: Find value
    MAX_CASE_TEMPERATURE=80,
    MAX_WINDING_TEMPERATURE=110,
)

from TMotorCANControl.mit_can import TMotorManager_mit_can, LOG_VARIABLES

def _tmotor_impedance_mode_exit(tmotor_actuator: "TMotorActuator") -> None:
    tmotor_actuator.stop_motor()


def _tmotor_current_mode_exit(tmotor_actuator: "TMotorActuator") -> None:
    tmotor_actuator.stop_motor()

def _tmotor_velocity_mode_exit(tmotor_actuator: "TMotorActuator") -> None:
    tmotor_actuator.stop_motor()


TMOTOR_CONTROL_MODE_CONFIGS = CONTROL_MODE_CONFIGS(
    IMPEDANCE=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=_tmotor_impedance_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
    CURRENT=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=_tmotor_current_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
    VELOCITY=ControlModeConfig(
        entry_callback=lambda _: None,
        exit_callback=_tmotor_velocity_mode_exit,
        has_gains=False,
        max_gains=None,
    ),
)

class TMotorMITCANActuator(ActuatorBase, TMotorManager_mit_can):
    def __init__(
        self,
        tag: str = "TMotorActuator",
        port: str = "/dev/ttyACM0",
        motor_type = "AK80-9",
        motor_ID = 41,
        gear_ratio: float = 1.0,
        baud_rate: int = 230400,
        frequency: int = 500,
        offline: bool = False,
    ) -> None:
        ActuatorBase.__init__(
            self,
            tag=tag,
            gear_ratio=gear_ratio,
            motor_constants=TMOTOR_ACTUATOR_CONSTANTS,
            frequency=frequency,
            offline=offline,
        )
        TMotorManager_mit_can.__init__(
            self,
            motor_type=motor_type,
            motor_ID=motor_ID,
        )

    @property
    def _CONTROL_MODE_CONFIGS(self) -> CONTROL_MODE_CONFIGS:
        return TMOTOR_CONTROL_MODE_CONFIGS


    def home(self):
        # TODO: implement homing
        # LOGGER.info(msg=f"[{self.__repr__()}] Homing not implemented.")
        pass




    @property
    def case_temperature(self):
        return self.get_temperature_celsius()

    @property
    def motor_current(self):
        return self.get_current_qaxis_amps()

    @property
    def motor_position(self):
        return self.get_motor_angle_radians()

    @property
    def motor_torque(self):
        return self.get_motor_torque_newton_meters()

    @property
    def motor_velocity(self):
        return self.get_motor_velocity_radians_per_second()

    @property
    def motor_voltage(self):
        # Not implemented
        pass

    @property
    def winding_temperature(self):
        #Not implemented
        pass

    def set_current_gains(self):
        TMotorManager_mit_can.set_current_gains(self)

    def set_position_gains(self) -> None:
        # Not implemented
        pass

    def set_impedance_gains(self, K, B):
        self.set_impedance_gains_real_unit(K=K, B=B)

    def set_joint_torque(self, value):
        self.torque = value

    def set_motor_current(self, value):
        self.current = value

    def set_motor_position(self, value):
        self.position_motorside = value

    def set_motor_torque(self, value):
        self.torque_motorside = value

    def set_motor_voltage(self, value):
        #Not implemented
        pass

    def start(self):
        TMotorManager_mit_can.__enter__(self)

    def stop(self):
        TMotorManager_mit_can.__exit__(self, None, None, None)

    def update(self):
        TMotorManager_mit_can.update(self)









if __name__ == "__main__":
    with TMotorMITCANActuator() as dev:
        dev.set_zero_position() # has a delay!
        time.sleep(1.5)
        dev.set_impedance_gains(K=10,B=0.5)

        print("Starting position step demo. Press ctrl+C to quit.")

        loop = SoftRealtimeLoop(dt = 0.01, report=True, fade=0)
        for t in loop:
            dev.update()
            if t < 1.0:
                dev.set_motor_position(0.0)
            else:
                dev.set_motor_position(1.0)

            print("Actual: ", round(dev.get_output_angle_radians(), 3), "Desired: ", dev._command.position)

        del loop
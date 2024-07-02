from typing import Any, Callable, Union, overload

import asyncio
import ctypes
import enum
import os
import time
from ctypes import c_int
from dataclasses import dataclass

import moteus
# import moteus.moteus_tool
import moteus_pi3hat as pihat
import numpy as np

import opensourceleg.hardware.actuators.base as base
from opensourceleg.hardware.thermal import ThermalModel
from opensourceleg.tools.logger import LOGGER



DEFAULT_POSITION_GAINS = base.ControlGains(kp=50, ki=0, kd=0, K=0, B=0, ff=0)

DEFAULT_CURRENT_GAINS = base.ControlGains(kp=40, ki=400, kd=0, K=0, B=0, ff=128)

DEFAULT_IMPEDANCE_GAINS = base.ControlGains(kp=40, ki=400, kd=0, K=200, B=400, ff=128)


class VoltageMode(base.ActuatorMode):
    
    def __init__(self, device: "MoteusObject") -> None:
        super().__init__(mode_pass=moteus.Mode.VOLTAGE, device=device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit
        self._device: MoteusObject = device
        self._control_mode = moteus.Mode.VOLTAGE

    def _entry(self) -> None:
        # TODO: check log instance
        LOGGER.debug(msg=f"[Actpack] Entering Voltage mode.")

    def _exit(self) -> None:
        # TODO: check log instance
        LOGGER.debug(msg=f"[Actpack] Exiting Voltage mode.")
        # self.set_voltage(voltage_value=0)
        time.sleep(0.1)

    def set_voltage(self, voltage_value: int) -> None:
        # TODO: Check instances here
        # TODO: Check servo mappings
        self._device._commands.append(
            self._device._servos[11].make_vfoc(
                theta=0, voltage=voltage_value, query=True
            )
        )


class CurrentMode(base.ActuatorMode):
    def __init__(self, device: "MoteusObject") -> None:
        super().__init__(mode_pass=moteus.Mode.CURRENT, device=device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit
        self._device: MoteusObject = device
        self._gains = DEFAULT_CURRENT_GAINS

        # self._control_mode = moteus.Mode.CURRENT

    def _entry(self) -> None:

        # TODO: check log instance
        LOGGER.debug(msg=f"[Actpack] Entering Current mode.")

        if not self.has_gains:
            self.set_gains(DEFAULT_CURRENT_GAINS)

        self.set_current(current_value=0)

    def _exit(self) -> None:
        # TODO: check log instance
        LOGGER.debug(msg=f"[Actpack] Exiting Current mode.")
        # self._device.send_motor_command(ctrl_mode=fxe.FX_VOLTAGE, value=0)

    def set_current(
        self,
        current_value: int,
    ) -> None:
        """Sets the Q-axis current of the motor

        Args:
            current_value (int): _description_
        """
        super().set_current(current_value=current_value)

        # assert 0 <= Gains.kp <= 80, "kp must be between 0 and 80"
        # assert 0 <= Gains.ki <= 800, "ki must be between 0 and 800"
        # assert 0 <= Gains.ff <= 128, "ff must be between 0 and 128"

        # self._device.set_gains(kp=Gains.kp, ki=Gains.ki, kd=0, k=0, b=0, ff=Gains.ff)
        # TODO: Check instances here
        self._device._commands.append(
            self._device._servos[11].make_current(
                # TODO: Check servo mappings
                q_A=current_value,
                query=True,
            )
        )

    def set_gains(
        self,
        gains: base.ControlGains = DEFAULT_CURRENT_GAINS,
    ) -> None:

        assert 0 <= gains.kp <= 80, "kp must be between 0 and 80"
        assert 0 <= gains.ki <= 800, "ki must be between 0 and 800"
        assert 0 <= gains.ff <= 128, "ff must be between 0 and 128"
        self._has_gains = True
        self._gains = gains
        # TODO: Check Gain Instances
        # self._device.set_gains(kp=gains.kp, ki=gains.ki, kd=0, k=0, b=0, ff=gains.ff)


class PositionMode(base.ActuatorMode):
    def __init__(self, device: "MoteusObject") -> None:
        super().__init__(mode_pass=moteus.Mode.POSITION, device=device)
        self._entry_callback = self._entry
        self._exit_callback = self._exit
        self._device: MoteusObject = device
        self._gains = DEFAULT_POSITION_GAINS
        # self._control_mode = moteus.Mode.POSITION
        pass

    def _entry(self) -> None:
        LOGGER.debug(msg=f"[Actpack] Entering Position mode.")

        if not self.has_gains:
            self.set_gains(self._gains)

        self.set_position(encoder_count=0)

    def _exit(self) -> None:
        LOGGER.debug(msg=f"[Actpack] Exiting Position mode.")
        # self._device.send_motor_command(ctrl_mode=fxe.FX_VOLTAGE, value=0)
        time.sleep(0.1)

    def set_position(
        self,
        encoder_count: int,
    ) -> None:
        super().set_position(encoder_count=encoder_count)
        self._device._commands.append(
            self._device._servos[11].make_position(
                # TODO: Check servo mappings
                position=encoder_count,
                velocity=0.1,
                query=True,
            )
        )

    def set_gains(
        self,
        gains: base.ControlGains = DEFAULT_POSITION_GAINS,
    ) -> None:

        assert 0 <= gains.kp <= 1000, "kp must be between 0 and 1000"
        assert 0 <= gains.ki <= 1000, "ki must be between 0 and 1000"
        assert 0 <= gains.kd <= 1000, "kd must be between 0 and 1000"
        self._has_gains = True
        self._gains = gains
        # TODO: Check Gain Instances
        # self._device.set_gains(
        #     kp=gains.kp, ki=gains.ki, kd=gains.kd, k=0, b=0, ff=gains.ff
        # )


# class ImpedanceMode(base.ImpedanceMode):
#     def __init__(self, device: "MoteusObject") -> None:
#         super().__init__(mode_pass=fxe.FX_IMPEDANCE, device=device)
#         self._entry_callback = self._entry
#         self._exit_callback = self._exit
#         self._device: MoteusObject = device
#         self._gains = DEFAULT_IMPEDANCE_GAINS
#         self._control_mode: c_int = fxe.FX_IMPEDANCE
#         pass

#     def _entry(self) -> None:
#         LOGGER.debug(msg=f"[Actpack] Entering Impedance mode.")
#         if not self.has_gains:
#             self.set_gains(gains=self._gains)

#         self._device.set_motor_position(self._device.motor_position)

#     def _exit(self) -> None:
#         LOGGER.debug(msg=f"[Actpack] Exiting Impedance mode.")
#         self._device.send_motor_command(ctrl_mode=fxe.FX_VOLTAGE, value=0)
#         time.sleep(1 / self._device.frequency)

#     def set_gains(self, gains: base.ControlGains = DEFAULT_IMPEDANCE_GAINS):
#         assert 0 <= gains.kp <= 80, "kp must be between 0 and 80"
#         assert 0 <= gains.ki <= 800, "ki must be between 0 and 800"
#         assert 0 <= gains.ff <= 128, "ff must be between 0 and 128"
#         assert 0 <= gains.K, "K must be greater than 0"
#         assert 0 <= gains.B, "B must be greater than 0"

#         self._has_gains = True
#         self._gains = gains
#         self._device.set_gains(
#             kp=gains.kp,
#             ki=gains.ki,
#             kd=0,
#             k=gains.K,
#             b=gains.B,
#             ff=gains.ff,
#         )

#     def set_position(
#         self,
#         encoder_count: int,
#     ) -> None:
#         super().set_position(encoder_count=encoder_count)
#         self._device.send_motor_command(ctrl_mode=fxe.FX_IMPEDANCE, value=encoder_count)
#         pass


@dataclass(init=False)
class MoteusControlModes:
    """
    Actpack modes

    Args:
        voltage (VoltageMode): Voltage mode
        current (CurrentMode): Current mode
        position (PositionMode): Position mode
        impedance (ImpedanceMode): Impedance mode
    """

    voltage: VoltageMode
    current: CurrentMode
    position: PositionMode
    # impedance: ImpedanceMode

    def __init__(self, device: "MoteusObject") -> None:
        self.voltage = VoltageMode(device=device)
        self.current = CurrentMode(device=device)
        self.position = PositionMode(device=device)
        # self.impedance = ImpedanceMode(device=device)

    def __repr__(self) -> str:
        return f"ActpackControlModes"


class MoteusObject(base.Actuator):
    def __init__(
        self,
        name: str = "MoteusObject",
        addr_map={
            1: [11],
        },
        logger: Logger = Logger(),
        frequency: int = 500,
        debug_level: int = 0,
        # dephy_log: bool = False,
        *args,
        **kwargs,
    ) -> None:
        # TODO: check for better ways!!!
        # os.system("sudo bash")
        base.Actuator.__init__(
            self,
            Gains=base.ControlGains(),
            MecheSpecs=base.MechanicalConstants(),
        )
        self._map: dict[int, list[int]] = addr_map
        self._transport = pihat.Pi3HatRouter(servo_bus_map=addr_map)
        self._hat_bus = []
        self._controller_id = []
        for index, (hat_bus, controller_id) in enumerate(self._map.items()):
            self._hat_bus.append(hat_bus)
            self._controller_id.append(controller_id)
        
        
        self._commands: list[moteus.Command] = []

        self._servos = {
            item: moteus.Controller(
                id = item, 
                transport=self._transport,          
            )
            for row in self._controller_id for item in row
        }
        self._name: str = name

        # self._frequency: int = frequency
        # self._debug_level: int = debug_level
        # # self._dephy_log: bool = dephy_log
        # self._name: str = name
        self._log: Logger = logger
        self._encoder_map = None

        self._CANbus = pihat.CanConfiguration()
        self._CANbus.fdcan_frame = False
        self._CANbus.bitrate_switch = False

        self._motor_zero_position = 0.0

        self._motor_offset = 0.0

        self.control_modes: MoteusControlModes = MoteusControlModes(device=self)

        self._mode: base.ActuatorMode = None
        self._data: Any = None

        self._result: Any = None

    def __repr__(self) -> str:
        return f"MoteusObject[{self._name}]"

    async def start(self) -> None:
        super().start()
        
        self._result = await self._transport.cycle(
            [x.make_stop() for x in self._servos.values()]
        )
        await asyncio.sleep(0.02)
        # self._mode.enter()

    async def stop(self) -> None:
        try:
            self._transport.cycle([self.make_stop()])
        except OSError as e:
            print("\n")
            LOGGER.error(
                msg=f"[{self.__repr__()}] Need admin previleges to open the port '{self.port}'. \n\nPlease run the script with 'sudo' command or add the user to the dialout group.\n"
            )
            os._exit(status=1)

        time.sleep(0.1)
        self._data = self.read()
        self._mode.enter()

    def stop(self) -> None:
        super().stop()
        await self._transport.cycle([x.make_stop()] for x in self._servos.values())

        await asyncio.sleep(0.02)
        # self.close()

    async def update(self) -> None:
        # TODO: check instance here
        self._result = await self._transport.cycle(commands=self._commands)
        self._commands = []
        # self._log.warning(
        #     msg=f"[{self.__repr__()}] Please open() the device before streaming data."
        # )
    def update(self) -> None:
        super().update()
        if self.is_streaming:
            self._data = self.read()
            self._thermal_model.T_c = self.case_temperature
            self._thermal_scale = self._thermal_model.update_and_get_scale(
                dt=(1 / self._frequency),
                motor_current=self.motor_current,
            )

            # Check for thermal fault, bit 2 of the execute status byte
            if self._data.status_ex & 0b00000010 == 0b00000010:
                raise RuntimeError("Actpack Thermal Limit Tripped")

        else:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Please open() the device before streaming data."
            )

    def set_mode(self, mode: base.ActuatorMode) -> None:
        if type(mode) in [VoltageMode, CurrentMode, PositionMode]:
            self._mode = mode
            self._mode.transition(to_state=mode)
            
        else:
            self._log.warning(msg=f"[{self.__repr__()}] Mode {mode} not found")
            LOGGER.warning(msg=f"[{self.__repr__()}] Mode {mode} not found")
            pass

    def set_voltage(self, voltage_value: float):
        if self._mode != self.control_modes.voltage:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set voltage in mode {self._mode}"
            )
            return

        self._mode.set_voltage(
            int(voltage_value),
        )

    def set_current(
        self,
        current_value: float,
    ):
        if self._mode != self.control_modes.current:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set current in mode {self._mode}"
            )
            return

        self._mode.set_current(
            int(current_value),
        )

    def set_motor_torque(self, torque: float) -> None:
        """
        Sets the motor torque in Nm.

        Args:
            torque (float): The torque to set in Nm.
        """
        if self._mode != self.control_modes.current:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set motor_torque in mode {self._mode}"
            )
            return

        self._mode.set_current(
            int(torque / self._MecheConsts.NM_PER_MILLIAMP),
        )

    def set_motor_position(self, position: float) -> None:
        """
        Sets the motor position in radians.
        If in impedance mode, this sets the equilibrium angle in radians.

        Args:
            position (float): The position to set
        """
        if self._mode not in [
            self.control_modes.position,
            # self.control_modes.impedance,
        ]:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set motor position in mode {self._mode}"
            )
            return

        self._mode.set_position(
            int(
                (position + self._motor_zero_position + self._motor_offset)
                / self._MecheConsts.RAD_PER_COUNT
            ),
        )

    def set_position_gains(
        self,
        kp: int = DEFAULT_POSITION_GAINS.kp,
        ki: int = DEFAULT_POSITION_GAINS.ki,
        kd: int = DEFAULT_POSITION_GAINS.kd,
        ff: int = DEFAULT_POSITION_GAINS.ff,
    ) -> None:
        """
        Sets the position gains in arbitrary Dephy units.

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            kd (int): The derivative gain
            ff (int): The feedforward gain
        """
        if self._mode != self.control_modes.position:
            LOGGER.warning(
                msg=f"[{self.__repr__()}] Cannot set position gains in mode {self._mode}"
            )
            return

        self._mode.set_gains(base.ControlGains(kp=kp, ki=ki, kd=kd, K=0, B=0, ff=ff))  # type: ignore

    def set_current_gains(
        self,
        kp: int = DEFAULT_CURRENT_GAINS.kp,
        ki: int = DEFAULT_CURRENT_GAINS.ki,
        ff: int = DEFAULT_CURRENT_GAINS.ff,
    ) -> None:
        """
        Sets the current gains in arbitrary Dephy units.

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            ff (int): The feedforward gain
        """
        if self._mode != self.control_modes.current:
            LOGGER.warning(
                f"[{self.__repr__()}] Cannot set current gains in mode {self._mode}"
            )
            return

        self._mode.set_gains(base.ControlGains(kp=kp, ki=ki, kd=0, K=0, B=0, ff=ff))  # type: ignore

    # def set_impedance_gains(
    #     self,
    #     kp: int = DEFAULT_IMPEDANCE_GAINS.kp,
    #     ki: int = DEFAULT_IMPEDANCE_GAINS.ki,
    #     K: int = DEFAULT_IMPEDANCE_GAINS.K,
    #     B: int = DEFAULT_IMPEDANCE_GAINS.B,
    #     ff: int = DEFAULT_IMPEDANCE_GAINS.ff,
    # ) -> None:
    #     """
    #     Sets the impedance gains in arbitrary actpack units.
    #     See Dephy's webpage for conversions or use other library methods that handle conversion for you.

    #     Args:
    #         kp (int): The proportional gain
    #         ki (int): The integral gain
    #         K (int): The spring constant
    #         B (int): The damping constant
    #         ff (int): The feedforward gain
    #     """
    #     if self._mode != self.control_modes.impedance:
    #         LOGGER.warning(
    #             msg=f"[{self.__repr__()}] Cannot set impedance gains in mode {self._mode}"
    #         )
    #         return

    #     self._mode.set_gains(base.ControlGains(kp=kp, ki=ki, kd=0, K=K, B=B, ff=ff))  # type: ignore

    def set_motor_zero_position(self, position: float) -> None:
        """Sets motor zero position in radians"""
        self._motor_zero_position = position

    def set_motor_offset(self, position: float) -> None:
        """Sets joint offset position in radians"""
        self._motor_offset = position

    def set_encoder_map(self, encoder_map) -> None:
        """Sets the joint encoder map"""
        self._encoder_map = encoder_map

    def auto_map(self) -> None:
        # TODO: auto change the servo controller mapping based on the configuration
        pass

    def fault_code(self, id: int):
        if self._result != []:
            for num in range(len(self._result)):
                if id == self._result[num].id:
                    return self._result[id].values[moteus.Register.FAULT]

    def motor_position(self, id: int):
        if self._result != []:
            for num in range(len(self._result)):
                if id == self._result[num].id:
                    return self._result[id].values[moteus.Register.POSITION]

    def motor_velocity(self, id: int):
        if self._result != []:
            for num in range(len(self._result)):
                if id == self._result[num].id:
                    return self._result[id].values[moteus.Register.VELOCITY]

    def temperature(self, id: int):
        if self._result != []:
            for num in range(len(self._result)):
                if id == self._result[num].id:
                    return self._result[id].values[
                        int(str(moteus.Register.TEMPERATURE))
                    ]

    def bus_voltage(self, id: int):
        if self._result != []:
            for num in range(len(self._result)):
                if id == self._result[num].id:
                    return self._result[id].values[moteus.Register.VOLTAGE]

    # @property
    # def bus_voltage(self):
    #     if self._result != []:
    #         return self._result.values[13]
    @property
    def frequency(self) -> int:
        return self._frequency

    @property
    def encoder_map(self):
        """Polynomial coefficients defining the joint encoder map from counts to radians."""
        return self._encoder_map

    @property
    def motor_zero_position(self) -> float:
        """Motor encoder zero position in radians."""
        return self._motor_zero_position

    @property
    def joint_zero_position(self) -> float:
        """Joint encoder zero position in radians."""
        return self._joint_zero_position

    @property
    def joint_offset(self) -> float:
        """Joint encoder offset in radians."""
        return self._joint_offset

    @property
    def motor_offset(self) -> float:
        """Motor encoder offset in radians."""
        return self._motor_offset

    @property
    def joint_direction(self) -> float:
        """Joint direction: 1 or -1"""
        return self._joint_direction

    @property
    def battery_voltage(self) -> float:
        """Battery voltage in mV."""
        if self._data is not None:
            return float(self._data.batt_volt)
        else:
            return 0.0

    @property
    def battery_current(self) -> float:
        """Battery current in mA."""
        if self._data is not None:
            return float(self._data.batt_curr)
        else:
            return 0.0

    @property
    def motor_voltage(self) -> float:
        """Q-axis motor voltage in mV."""
        if self._data is not None:
            return float(self._data.mot_volt)
        else:
            return 0.0

    @property
    def motor_current(self) -> float:
        """Q-axis motor current in mA."""
        if self._data is not None:
            return float(self._data.mot_cur)
        else:
            return 0.0

    @property
    def motor_torque(self) -> float:
        """
        Torque at motor output in Nm.
        This is calculated using the motor current and torque constant.
        """
        if self._data is not None:
            return float(self._data.mot_cur * self._MecheConsts.NM_PER_MILLIAMP)
        else:
            return 0.0

    @property
    def motor_position(self) -> float:
        """Angle of the motor in radians."""
        if self._data is not None:
            return (
                float(self._data.mot_ang * self._MecheConsts.RAD_PER_COUNT)
                - self._motor_zero_position
                - self.motor_offset
            )
        else:
            return 0.0

    @property
    def motor_encoder_counts(self) -> int:
        """Raw reading from motor encoder in counts."""
        return int(self._data.mot_ang)

    @property
    def joint_encoder_counts(self) -> int:
        """Raw reading from joint encoder in counts."""
        return int(self._data.ank_ang)

    @property
    def motor_velocity(self) -> float:
        """Motor velocity in rad/s."""
        if self._data is not None:
            return int(self._data.mot_vel) * self._MecheConsts.RAD_PER_DEG
        else:
            return 0.0

    @property
    def motor_acceleration(self) -> float:
        """Motor acceleration in rad/s^2."""
        if self._data is not None:
            return float(self._data.mot_acc)
        else:
            return 0.0

    @property
    def joint_position(self) -> float:
        """Measured angle from the joint encoder in radians."""
        if self._data is not None:
            if self.encoder_map is not None:
                return float(self.encoder_map(self._data.ank_ang))
            else:
                return (
                    float(self._data.ank_ang * self._MecheConsts.RAD_PER_COUNT)
                    - self.joint_zero_position
                    - self.joint_offset
                ) * self.joint_direction
        else:
            return 0.0

    @property
    def joint_velocity(self) -> float:
        """Measured velocity from the joint encoder in rad/s."""
        if self._data is not None:
            return float(self._data.ank_vel * self._MecheConsts.RAD_PER_COUNT)
        else:
            return 0.0

    @property
    def case_temperature(self) -> float:
        """Case temperature in celsius."""
        if self._data is not None:
            return float(self._data.temperature)
        else:
            return 0.0

    @property
    def winding_temperature(self) -> float:
        """
        ESTIMATED temperature of the windings in celsius.
        This is calculated based on the thermal model using motor current.
        """
        if self._data is not None:
            return float(self._thermal_model.T_w)
        else:
            return 0.0

    @property
    def thermal_scaling_factor(self) -> float:
        """
        Scale factor to use in torque control, in [0,1].
        If you scale the torque command by this factor, the motor temperature will never exceed max allowable temperature.
        For a proof, see paper referenced in thermal model.
        """
        return float(self._thermal_scale)

    @property
    def genvars(self):
        """Dephy's 'genvars' object."""
        if self._data is not None:
            return np.array(
                object=[
                    self._data.genvar_0,
                    self._data.genvar_1,
                    self._data.genvar_2,
                    self._data.genvar_3,
                    self._data.genvar_4,
                    self._data.genvar_5,
                ]
            )
        else:
            return np.zeros(shape=6)

    @property
    def accelx(self) -> float:
        """
        Acceleration in x direction in m/s^2.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.accelx * self._MecheConsts.M_PER_SEC_SQUARED_ACCLSB)
        else:
            return 0.0

    @property
    def accely(self) -> float:
        """
        Acceleration in y direction in m/s^2.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.accely * self._MecheConsts.M_PER_SEC_SQUARED_ACCLSB)
        else:
            return 0.0

    @property
    def accelz(self) -> float:
        """
        Acceleration in z direction in m/s^2.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.accelz * self._MecheConsts.M_PER_SEC_SQUARED_ACCLSB)
        else:
            return 0.0

    @property
    def gyrox(self) -> float:
        """
        Angular velocity in x direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.gyrox * self._MecheConsts.RAD_PER_SEC_GYROLSB)
        else:
            return 0.0

    @property
    def gyroy(self) -> float:
        """
        Angular velocity in y direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.gyroy * self._MecheConsts.RAD_PER_SEC_GYROLSB)
        else:
            return 0.0

    @property
    def gyroz(self) -> float:
        """
        Angular velocity in z direction in rad/s.
        Measured using actpack's onboard IMU.
        """
        if self._data is not None:
            return float(self._data.gyroz * self._MecheConsts.RAD_PER_SEC_GYROLSB)
        else:
            return 0.0


class MockData:
    def __init__(
        self,
        batt_volt=0,
        batt_curr=0,
        mot_volt=0,
        mot_cur=0,
        mot_ang=0,
        ank_ang=0,
        mot_vel=0,
        mot_acc=0,
        ank_vel=0,
        temperature=0,
        genvar_0=0,
        genvar_1=0,
        genvar_2=0,
        genvar_3=0,
        genvar_4=0,
        genvar_5=0,
        accelx=0,
        accely=0,
        accelz=0,
        gyrox=0,
        gyroy=0,
        gyroz=0,
    ):
        self.batt_volt = batt_volt
        self.batt_curr = batt_curr
        self.mot_volt = mot_volt
        self.mot_cur = mot_cur
        self.mot_ang = mot_ang
        self.ank_ang = ank_ang
        self.mot_vel = mot_vel
        self.mot_acc = mot_acc
        self.ank_vel = ank_vel
        self.temperature = temperature
        self.genvar_0 = genvar_0
        self.genvar_1 = genvar_1
        self.genvar_2 = genvar_2
        self.genvar_3 = genvar_3
        self.genvar_4 = genvar_4
        self.genvar_5 = genvar_5
        self.accelx = accelx
        self.accely = accely
        self.accelz = accelz
        self.gyrox = gyrox
        self.gyroy = gyroy
        self.gyroz = gyroz
        self.status_ex = 0b00000000

    def __repr__(self):
        return f"MockData"


class MockMoteusObject(MoteusObject):
    """
    MockMoteusObject class definition for testing.\n
    This class inherits everything from the MoteusObject class but
    deletes the super().__init__() call in the constructor so the
    constructor does not try to connect to a device. It also overrides
    some of the methods to allow for testing without a device, and adds
    attributes used to determine if the methods were called properly.
    """

    def __init__(
        self,
        name: str = "MockMoteusObject",
        port: str = "/dev/ttyACM0",
        baud_rate: int = 230400,
        frequency: int = 500,
        debug_level: int = 0,
        dephy_log: bool = False,
    ) -> None:
        """
        Initializes the MockMoteusObject class

        Args:
            name (str): _description_. Defaults to "MockMoteusObject".
            port (str): _description_
            baud_rate (int): _description_. Defaults to 230400.
            frequency (int): _description_. Defaults to 500.
            logger (Logger): _description_
            debug_level (int): _description_. Defaults to 0.
            dephy_log (bool): _description_. Defaults to False.
        """
        MoteusObject.__init__(self)
        # self._debug_level: int = debug_level
        # self._dephy_log: bool = dephy_log
        # self._frequency: int = frequency
        # # self._data: MockData = MockData()
        # self._name: str = name

        # self.log: Logger = logger
        # self._state = None

        # # New attributes to be used for testing

        # # This is used in the open() method to display the port the device should be connected to
        # self.port: str = port

        # This is used in the send_motor_command() method to display the motor command that was sent
        self._motor_command: str = "None"

        # This is used in the set_gains() method to display the gains that were set
        self._gains: dict[str, float] = {
            "kp": 0,
            "ki": 0,
            "kd": 0,
            "k": 0,
            "b": 0,
            "ff": 0,
        }

        self._data: MockData = MockData()

    def open(self, freq, log_level, log_enabled):
        if freq == 100 and log_level == 5 and log_enabled:
            raise OSError
        else:
            LOGGER.debug(msg=f"Opening Device at {self.port}")

    def send_motor_command(self, ctrl_mode, value):
        self._motor_command = f"Control Mode: {ctrl_mode}, Value: {value}"

    # Overrides the set_gains method to set the gains in the new _gains attribute
    def set_gains(self, kp, ki, kd, k, b, ff):
        self._gains["kp"] = kp
        self._gains["ki"] = ki
        self._gains["kd"] = kd
        self._gains["k"] = k
        self._gains["b"] = b
        self._gains["ff"] = ff

    # Overrides the read method to modify the data incrementally instead of through a device data stream
    def read(self):
        small_noise = 0

        self._data.batt_volt += 15
        self._data.batt_curr += 15
        self._data.mot_volt += 15
        self._data.mot_cur += 15
        self._data.mot_ang += 15
        self._data.ank_ang += 15
        self._data.mot_vel += 15
        self._data.mot_acc += 15
        self._data.ank_vel += 15
        self._data.temperature += 15
        self._data.genvar_0 += 15
        self._data.genvar_1 += 15
        self._data.genvar_2 += 15
        self._data.genvar_3 += 15
        self._data.genvar_4 += 15
        self._data.genvar_5 += 15
        self._data.accelx += 15
        self._data.accely += 15
        self._data.accelz += 15
        self._data.gyrox += 15
        self._data.gyroy += 15
        self._data.gyroz += 15
        self._data.status_ex = 0b00000000
        return self._data

    # Overrides the close method to do nothing
    def close(self):
        pass


if __name__ == "__main__":
    pass

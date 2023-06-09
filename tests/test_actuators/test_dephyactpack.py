# import pytest

# from opensourceleg.actuators import DephyActpack, VoltageMode
# from opensourceleg.units import UnitsDefinition, DEFAULT_UNITS
# from opensourceleg.logger import Logger
# from opensourceleg.thermal import ThermalModel


# class MockDephyActpack(DephyActpack):
#     def __init__(
#         self,
#         port: str = "/dev/ttyACM0",
#         baud_rate: int = 230400,
#         frequency: int = 500,
#         logger: Logger = Logger(),
#         units: UnitsDefinition = DEFAULT_UNITS,
#         debug_level: int = 0,
#         dephy_log: bool = False,
#     ) -> None:
#         self._debug_level = debug_level
#         self._dephy_log = dephy_log
#         self._frequency = frequency
#         self._log = logger
#         self._state = None
#         self._units = units
#         self._motor_zero_position = 0.0
#         self._joint_zero_position = 0.0
#         self._thermal_model = ThermalModel()
#         self._modes = {"voltage": VoltageMode(device=self)}
#         self._mode = self._modes["voltage"]

# test_dap = MockDephyActpack()

# def test_init():
#     test_dap = MockDephyActpack()
#     assert test_dap._debug_level == 0
#     assert test_dap._dephy_log == False
#     assert test_dap._frequency == 500
#     assert test_dap._log != None
#     assert test_dap._state == None
#     assert test_dap._units != None
#     assert test_dap._motor_zero_position == 0.0
#     assert test_dap._joint_zero_position == 0.0
#     assert test_dap._thermal_model != None
#     assert test_dap._modes != None
#     assert test_dap._mode != None

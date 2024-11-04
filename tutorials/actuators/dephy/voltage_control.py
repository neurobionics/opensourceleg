# import time
# import sys
# from opensourceleg.actuators.base import CONTROL_MODES
# import opensourceleg.actuators.dephy as Dephy
# from opensourceleg.logging.logger import LOGGER

# actpack = Dephy.DephyActuator(
#     port="/dev/ttyACM0",
#     gear_ratio=1.0,
# )

# with actpack:

#     actpack.set_control_mode(mode=CONTROL_MODES.VOLTAGE)

#     while True:
#         actpack.set_motor_voltage(value=3000)  # in mV
#         actpack.update()
#         LOGGER.info(
#             "".join(
#                 f"Motor Position: {actpack.motor_position}\t"
#                 + f"Case Temperature: {actpack.case_temperature}"
#                 + f"Winding Temperature: {actpack.winding_temperature}"
#             )
#         )
#         time.sleep(0.005)


import time

import pandas as pd

import opensourceleg.actuators.dephy as Dephy
from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.logging.logger import LOGGER
from opensourceleg.time import SoftRealtimeLoop

TIME_TO_STEP = 1.0
FREQUENCY = 200
DT = 1 / FREQUENCY


def main():
    actpack = Dephy.DephyActuator(
        port="/dev/ttyACM0",
        gear_ratio=1.0,
    )
    voltage_data = pd.DataFrame(
        {
            "Time": [],
            "Output_Voltage": [],
            "Command_Voltage": [],
        }
    )
    clock = SoftRealtimeLoop(dt=DT)
    with actpack:

        try:
            actpack.set_control_mode(mode=CONTROL_MODES.VOLTAGE)

            for t in clock:

                if t > TIME_TO_STEP:
                    command_voltage = 3000
                    actpack.set_motor_voltage(value=command_voltage)  # in mV

                else:
                    command_voltage = 0

                actpack.update()

                LOGGER.info(
                    "".join(
                        f"Motor Position: {actpack.motor_position}\t"
                        + f"Motor Voltage: {actpack.motor_voltage}\t"
                        + f"Motor Current: {actpack.motor_current}\t"
                    )
                )
                voltage_data = pd.concat(
                    [
                        voltage_data,
                        pd.DataFrame(
                            {
                                "Time": [t],
                                "Output_Voltage": [actpack.motor_voltage],
                                "Command_Voltage": [command_voltage],
                            }
                        ),
                    ],
                    ignore_index=True,
                )

                time.sleep(DT)

        finally:
            voltage_data.to_csv("voltage_data_dephy.csv", index=False)
            exit()


if __name__ == "__main__":
    main()

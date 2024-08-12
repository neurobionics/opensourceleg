import time

import numpy as np

import opensourceleg.actuators.dephy as Dephy
from opensourceleg.logging.logger import LOGGER
import pandas as pd

actpack = Dephy.DephyActpack(
    port="/dev/ttyACM0",
    gear_ratio=9.0,
)
with actpack:
    position_data = pd.DataFrame(
        {
            "Time": [],
            "Output_Position": [],
            "Command_Position": [],
        }
    )
    iternum = 0
    time_period = 0.002
    try:
        actpack.set_control_mode(mode=actpack.CONTROL_MODES.POSITION)
        
        actpack.set_position_gains(
                # if no input, then default gains are applied
        )

        for iternum in range(100):
            actpack.update()
            # iternum += 1
            position_data = pd.concat(
                [
                    position_data,
                    pd.DataFrame(
                        {
                            "Time": [iternum * time_period],
                            "Output_Position": [actpack.output_position],
                            "Command_Position": [actpack.output_position],
                        }
                    ),
                ],
                ignore_index=True,
            )
            # position_data.to_csv("position_data_dephy.csv", index=False)
            time.sleep(time_period)
        
        # actpack.update()
        current_position = actpack.output_position + np.pi / 2
        while True:
            iternum += 1
            actpack.update()
            
            actpack.set_output_position(value= current_position)

            LOGGER.info(
                "".join(
                    f"Motor Position: {actpack.output_position}\t"
                    + f"Motor Voltage: {actpack.motor_voltage}\t"
                    + f"Motor Current: {actpack.motor_current}\t"
                )
            )

            position_data = pd.concat(
                [
                    position_data,
                    pd.DataFrame(
                        {
                            "Time": [iternum * time_period],
                            "Output_Position": [actpack.output_position],
                            "Command_Position": [current_position],
                        }
                    ),
                ],
                ignore_index=True,
            )

            # position_data.to_csv("position_data_dephy.csv", index=False)
            time.sleep(time_period)

    except KeyboardInterrupt:
        position_data.to_csv("position_data_dephy.csv", index=False)
        actpack.stop()

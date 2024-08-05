import asyncio
import numpy as np
import math
from opensourceleg.actuators.moteus import MoteusController
from moteus import Register
from opensourceleg.logging.logger import LOGGER
import pandas as pd
async def main():
    mc1 = MoteusController(
        servo_id=42,
        bus_id=3, 
        gear_ratio = 9.0, 
    )
    try: 
        mc1.start()
        await mc1.update() 
        mc1.set_control_mode(mode = mc1.CONTROL_MODES.TORQUE)
        current_data = pd.DataFrame({
            "Time": [], 
            "Output_Torque": [],
            "Command_Torque": [], 
        })
        iter = 0
        time_period = 0.001
        while True: 
            iter += 1
            mc1.set_motor_torque(value = 0.0027)
            await mc1.update()
            print(f"######")
            LOGGER.info("".join(
                f"Output Torque: {mc1._data[0].values[Register.TORQUE]}\t"
                )
            )
            print(f"------")
            current_data = pd.concat(
                [current_data, pd.DataFrame({
                    "Time": [iter * time_period],
                    "Output_Torque": [mc1._data[0].values[Register.TORQUE]],
                    "Command_Torque": [mc1._data[0].values[Register.COMMAND_FEEDFORWARD_TORQUE]],
                })],
                ignore_index=True,
            )
            current_data.to_csv("torque_data.csv", index = False)
            
            await asyncio.sleep(time_period)

    finally:
        await mc1.stop()

if __name__ == '__main__':
    asyncio.run(main())

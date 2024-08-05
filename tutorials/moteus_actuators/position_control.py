import asyncio
import numpy as np

from opensourceleg.actuators.moteus import MoteusController
from opensourceleg.logging.logger import LOGGER
from moteus import Register
import pandas as pd

async def main():
    mc1 = MoteusController(
        servo_id=42,
        bus_id=3, 
        gear_ratio=1.0
    )

    position_data = pd.DataFrame({
        "Time": [], 
        "Output_Position": [],
        "Command_Position": [],
    })
    try:
        mc1.start()
        await mc1.update()
        mc1.set_control_mode(mode = mc1.CONTROL_MODES.POSITION) 
        await mc1.set_position_gains(
            kp = 4.0, # 2
            ki = 1.0, # 5
            kd = 0.05, # 10
        )
        pos = mc1.motor_position
        iter = 0
        time_period = 0.01
        while True: 
            
            iter += 1
            mc1.set_motor_position(
                value = pos + np.pi / 2, 
            )
            await mc1.update()
            print(f"######")
            LOGGER.info("".join(
                f"Motor Position: {mc1.motor_position}\t"
                + f"Command_Position: {mc1._data[0].values[Register.COMMAND_POSITION] * 2* np.pi / mc1.gear_ratio}\t"
                )
            )
            position_data = pd.concat(
                [position_data, pd.DataFrame({
                    "Time": [iter * time_period], 
                    "Output_Position": [mc1.motor_position],
                    "Command_Position": [mc1._data[0].values[Register.COMMAND_POSITION] * 2 * np.pi],
                })],
                ignore_index=True,
            )
            position_data.to_csv("position_data.csv", index = False)
            print(f"------")
            await asyncio.sleep(time_period)

    finally:
        await mc1.stop()
    

if __name__ == '__main__':
    asyncio.run(main())

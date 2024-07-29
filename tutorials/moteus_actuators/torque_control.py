import asyncio
import numpy as np
import math
from opensourceleg.actuators.moteus import MoteusController
from moteus import Register
from opensourceleg.logging.logger import LOGGER
import pandas as pd
async def main():
    mc1 = MoteusController(
        servo_id=2,
        bus_id=2, 
    )
    try: 
        mc1.start()
        await mc1.update() 
        mc1.set_control_mode(mode = mc1.CONTROL_MODES.TORQUE)
        torque_data = pd.DataFrame({
            "Time": [], 
            "Output_Current": [],
            "Command_Current": [], 
        })
        iter = 0
        time_period = 0.0001
        while True: 
            iter += 1
            mc1.set_motor_torque(value = 0.00)
            await mc1.update()
            print(f"######")
            LOGGER.info("".join(
                f"Output Current: {mc1._data[0].values[Register.Q_CURRENT]}\t"
                )
            )
            print(f"------")
            torque_data = pd.concat(
                [torque_data, pd.DataFrame({
                    "Time": [iter * time_period],
                    "Output_Current": [mc1._data[0].values[Register.Q_CURRENT]],
                    "Command_Current": [mc1._data[0].values[Register.COMMAND_Q_CURRENT]]
                })],
                ignore_index=True,
            )
            torque_data.to_csv("current_data.csv", index = False)
            
            await asyncio.sleep(time_period)

    finally:
        await mc1.stop()

if __name__ == '__main__':
    asyncio.run(main())

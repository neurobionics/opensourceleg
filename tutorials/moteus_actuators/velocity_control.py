import asyncio
import numpy as np

from opensourceleg.actuators.moteus import MoteusController
from moteus import Register
from opensourceleg.logging.logger import LOGGER
import pandas as pd

async def main():
    mc1 = MoteusController(
        servo_id=42,
        bus_id=3, 
        gear_ratio=1.0, 
    )
    velocity_data = pd.DataFrame({
        "Time": [], 
        "Output_Velocity": [],
        "Command_Velocity": [],
    })
    
    try:
        mc1.start()
        mc1.set_control_mode(mode = mc1.CONTROL_MODES.VELOCITY)
        iter = 0
        time_period = 0.001
        await mc1.set_velocity_gains(
                kp = 4.0, # 2
                ki = 1.0, # 5
                kd = 0.05, # 10
            )
        while True: 
            iter += 1
            
            mc1.set_motor_velocity(
                value = np.pi, 
            )
            await mc1.update()
            print(f"######")
            LOGGER.info("".join(
                f"Motor Velocity: {mc1.motor_velocity}\t"
                + f"Motor Velocity Command: {mc1._data[0].values[Register.COMMAND_VELOCITY] * 2 * np.pi / mc1.gear_ratio}\t"
                )
            )
            velocity_data = pd.concat(
                [velocity_data, pd.DataFrame({
                    "Time": [iter * time_period], 
                    "Output_Velocity": [mc1.motor_velocity],
                    "Command_Velocity": [mc1._data[0].values[Register.COMMAND_VELOCITY] * 2 * np.pi / mc1.gear_ratio],
                })],
                ignore_index=True,
            )
            velocity_data.to_csv("velocity_data.csv", index = False)
            print(f"------")
            await asyncio.sleep(time_period)

    finally:
        await mc1.stop()
    

if __name__ == '__main__':
    asyncio.run(main())

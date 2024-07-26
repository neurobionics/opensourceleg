import asyncio
import numpy as np

from opensourceleg.actuators.moteus import MoteusController
from moteus import Register
from opensourceleg.logging.logger import LOGGER
async def main():
    mc1 = MoteusController(
        servo_id=2,
        bus_id=2, 
    )
    try:
        mc1.start()
        mc1.set_control_mode(mode = mc1.CONTROL_MODES.VELOCITY) 
        while True: 
            mc1.set_velocity_gains(
                kp = 2, # 2
                kd = 5, # 5
                ki = 10, # 10
            )
            mc1.set_motor_velocity(
                value = np.pi * 2, 
            )
            await mc1.update()
            print(f"######")
            LOGGER.info("".join(
                f"Motor Velocity: {mc1.motor_velocity}\t"
                + f"Motor Velocity Command: {mc1._data[0].values[Register.COMMAND_VELOCITY] * 2 * np.pi}\t"
                )
            )
            print(f"------")
            await asyncio.sleep(0.02)

    finally:
        await mc1.stop()
    

if __name__ == '__main__':
    asyncio.run(main())

import asyncio
import numpy as np

from opensourceleg.actuators.moteus import MoteusController
from opensourceleg.logging.logger import LOGGER

async def main():
    mc1 = MoteusController(
        servo_id=2,
        bus_id=4, 
    )
    try:

        mc1.start()
        await mc1.update()
        mc1.set_control_mode(mode = mc1.CONTROL_MODES.POSITION) 
        mc1.set_position_gains(
            kp = 1.0, 
            kd = 10.0, 
            ki = 0.1,
        )
        pos = mc1.motor_position
        while True: 
            mc1.set_motor_position(
                value = pos + np.pi / 2, 
            )
            await mc1.update()
            print(f"######")
            LOGGER.info("".join(
                f"Motor Position: {mc1.motor_position}\t"
                )
            )
            print(f"------")
            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        print("KeyboardInterrupt...")

    finally:
        await mc1.stop()
    

if __name__ == '__main__':
    asyncio.run(main())

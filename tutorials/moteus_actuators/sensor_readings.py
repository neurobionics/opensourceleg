import asyncio
import numpy as np
import math
from opensourceleg.actuators.moteus import MoteusController
from opensourceleg.logging.logger import Logger
from moteus import Register
import pandas as pd
async def main():
    mc1 = MoteusController(
        servo_id=2,
        bus_id=2, 
    )
    
    try: 
        mc1.start()
        
        while True: 
            await mc1.update()
            print(f"######")
            
            # LOGGER.info("".join(
            #     f"Motor Position: {mc1.motor_position}\t"
            #     + f"Motor Voltage: {mc1.motor_voltage}\t"
            #     + f"Motor Current: {mc1.motor_current}\t"
            #     )
            # )
            print(f"------")
            
            await asyncio.sleep(0.02)

    finally:
        await mc1.stop()

    

if __name__ == '__main__':
    asyncio.run(main())

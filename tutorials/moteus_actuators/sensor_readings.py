import asyncio
import numpy as np
import math
from opensourceleg.actuators.moteus import MoteusController
from opensourceleg.logging.logger import LOGGER
from moteus import Register
import pandas as pd
async def main():
    mc1 = MoteusController(
        servo_id=11,
        bus_id=2, 
    )
    
    try: 
        mc1.start()
        
        while True: 
            await mc1.update()
            print(f"######")
            print(f"{mc1._data[0]}")
            print(f"------")
            
            await asyncio.sleep(0.02)

    finally:
        await mc1.stop()

    

if __name__ == '__main__':
    asyncio.run(main())

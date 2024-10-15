from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging import LOGGER
from opensourceleg.time import SoftRealtimeLoop
from opensourceleg.actuators.base import CONTROL_MODES
from flexsea.device import Device

# from skip_testbed import futek
from opensourceleg.sensors.torque_sensor import Futek100Nm

import numpy as np
import time
import csv

FREQUENCY = 200








if __name__ == "__main__":
    driving_motor = DephyActuator(port="/dev/ttyACM0", gear_ratio=9.0, frequency=FREQUENCY)
    clock = SoftRealtimeLoop(dt=1/FREQUENCY, report=True)

    # futek_reader = futek.Big100NmFutek()
    futek_reader = Futek100Nm()

    csv_header = ['time', 'current', 'position', 'torque']

    f_log = open('torque_reading.csv', 'w')


    with driving_motor:
        driving_motor.set_control_mode(CONTROL_MODES.CURRENT)
        driving_motor.set_current_gains()
        driving_motor.set_motor_current(0)

        csv_writer = csv.writer(f_log)
        csv_writer.writerow(csv_header)

        for t in clock:
            driving_motor.update()
            # print(t)
            if t < 2:
                # print("t < 200")
                driving_motor.set_motor_current(0)
            elif t < 4:
                # print("t < 400")
                driving_motor.set_motor_current(1000)
            else:
                # print("t > 400")
                driving_motor.set_motor_current(0)
            current = driving_motor.motor_current
            output_position = driving_motor.output_position
            futek_reader.get_torque()
            print("Current: ", current, "Futek Torque: ", futek_reader.torque)


            csv_writer.writerow([t, current, output_position, futek_reader.torque])









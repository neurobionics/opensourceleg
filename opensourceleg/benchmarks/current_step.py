from opensourceleg.actuators.dephy import DephyActuator
from opensourceleg.logging import LOGGER
from opensourceleg.time import SoftRealtimeLoop
from opensourceleg.actuators.base import CONTROL_MODES
from flexsea.device import Device

from opensourceleg.sensors.torque_sensor import Futek
from opensourceleg.benchmarks.benchmark_manager import SimpleTimer, LimitVelocity
import numpy as np
import csv

FREQUENCY = 200



def current_step():
    t_cond = 2.0 # [sec], time held on target
    current_velocity = 500 # [mA/s], ramp current velocity
    max_current = 4000 # mA
    num_steps = 21

    # current_steps = np.linspace(0, max_current, num=num_steps)
    current_steps = [(i / (num_steps - 1)) * max_current for i in range(num_steps)]

    futek = Futek()
    futek.calibrate_loadcell()

    on_target = 0
    curr_command = 0.0
    stopping = False

    driving_motor = DephyActuator(port="/dev/ttyACM0", gear_ratio=9.0, frequency=FREQUENCY)
    clock = SoftRealtimeLoop(dt=1/FREQUENCY, report=True)


    timer = SimpleTimer()
    ramp = LimitVelocity(current_velocity, FREQUENCY)
    csv_header = ['time','boolean_on_target', 'target_current', 'current', 'position', 'torque']
    f_log = open('torque_reading.csv', 'w')

    # Within loop
    with driving_motor:
        driving_motor.set_control_mode(CONTROL_MODES.CURRENT)
        driving_motor.set_current_gains()
        driving_motor.set_motor_current(0)

        csv_writer = csv.writer(f_log)
        csv_writer.writerow(csv_header)

        for t in clock:

            driving_motor.update()

            if timer.is_done:
                if timer.just_done:
                    if current_steps is not None:
                        curr_command = current_steps.pop(0)
                        print("Setting q-current to %2f mA."% curr_command)
                    else:
                        curr_command = 0.0
                        stopping = True
                elif on_target:
                    timer.start(t_cond)

            curr_command_lim = ramp.update(curr_command)

            if stopping and ramp.is_stopped:
                break

            on_target = ramp.on_target

            driving_motor.set_motor_current(curr_command_lim)

            current = driving_motor.motor_current
            output_position = driving_motor.output_position
            futek.get_torque()

            print("Desired Current: ", curr_command_lim, "Current: ", current, "Futek Torque: ", futek.torque)

            csv_writer.writerow([t, on_target, curr_command_lim, current, output_position, futek.torque])

if __name__ == "__main__":
    current_step()

from opensourceleg.osl import OpenSourceLeg
from opensourceleg.control.virtual_hard_stop import VirtualHardStop, Direction
from opensourceleg.tools.units import convert_to_default
import numpy as np

LOOP_FREQUENCY = 200 # Hz
osl = OpenSourceLeg(frequency=LOOP_FREQUENCY, file_name='./' + 'hardStopTest')
osl.clock.report = True

osl.add_joint("knee", gear_ratio=9 * 83 / 18, offline_mode=False)
osl.add_joint("ankle", gear_ratio=9 * 83 / 18, offline_mode=False)

ankle_hard_stop_pos = VirtualHardStop(osl.ankle, Direction.POSITIVE, np.deg2rad(20), 100, 9)
ankle_hard_stop_neg = VirtualHardStop(osl.ankle, Direction.NEGATIVE, np.deg2rad(-20), 100, 9)

with osl:
    osl.home()
    osl.update()

    # Testing Impedance Mode
    ankle_stiffness = 10
    osl.knee.set_mode(osl.knee.control_modes.impedance)
    osl.knee.set_joint_impedance(K = 10, B = 5)
    osl.knee.set_output_position(osl.knee.output_position)
    osl.ankle.set_mode(osl.ankle.control_modes.impedance)
    osl.ankle.set_joint_impedance(K = ankle_stiffness, B = 1)
    osl.ankle.set_output_position(osl.ankle.output_position)


    for time in osl.clock:
        osl.update()
        hard_stop_eq_angle_bias = ankle_hard_stop_pos.calculate_eq_angle_bias(ankle_stiffness) + ankle_hard_stop_neg.calculate_eq_angle_bias(ankle_stiffness)
        osl.ankle.set_output_position(hard_stop_eq_angle_bias)
        print(hard_stop_eq_angle_bias, ankle_hard_stop_neg.is_active, ankle_hard_stop_pos.is_active)

    # Testing Torque Mode
    # osl.knee.set_mode(osl.knee.control_modes.current)
    # osl.knee.set_output_torque(0)
    # osl.ankle.set_mode(osl.ankle.control_modes.current)
    # osl.ankle.set_output_torque(0)

    # for time in osl.clock:
    #     osl.update()
    #     hard_stop_torque = ankle_hard_stop_pos.calculate_hard_stop_torque() + ankle_hard_stop_neg.calculate_hard_stop_torque()
    #     print(hard_stop_torque)
    #     osl.ankle.set_output_torque(hard_stop_torque)
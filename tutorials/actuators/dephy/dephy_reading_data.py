import time
from collections import deque

import numpy as np
from flexsea.device import Device

from opensourceleg.math import ThermalModel

FREQUENCY = 1000  # Hz
MAX_WINDING_TEMPERATURE = 150
MAX_CASE_TEMPERATURE = 100

MA_WINDOW_SIZE = 3

if __name__ == "__main__":
    local_data = []

    current_window_01 = deque(maxlen=MA_WINDOW_SIZE)
    current_window_02 = deque(maxlen=MA_WINDOW_SIZE)

    case_temperature_01 = deque(maxlen=MA_WINDOW_SIZE)
    case_temperature_02 = deque(maxlen=MA_WINDOW_SIZE)

    for _ in range(MA_WINDOW_SIZE):
        current_window_01.append(0)
        current_window_02.append(0)
        case_temperature_01.append(0)
        case_temperature_02.append(0)

    thermalModel_1 = ThermalModel(
        temp_limit_windings=MAX_WINDING_TEMPERATURE,
        soft_border_C_windings=10,
        temp_limit_case=MAX_CASE_TEMPERATURE,
        soft_border_C_case=10,
    )

    thermalModel_2 = ThermalModel(
        temp_limit_windings=MAX_WINDING_TEMPERATURE,
        soft_border_C_windings=10,
        temp_limit_case=MAX_CASE_TEMPERATURE,
        soft_border_C_case=10,
    )

    device_1 = Device(
        port="/dev/ttyACM0",
        firmwareVersion="7.2.0",
        debug=True,
        logLevel=0,
    )
    device_2 = Device(
        port="/dev/ttyACM1",
        firmwareVersion="7.2.0",
        debug=True,
        logLevel=0,
    )

    device_1.open()
    device_1.start_streaming(FREQUENCY)

    device_2.open()
    device_2.start_streaming(FREQUENCY)

    t = 0

    while True:
        try:
            data_1 = device_1.read()
            data_2 = device_2.read()

            case_temperature_01.append(data_1["temperature"])
            case_temperature_02.append(data_2["temperature"])

            if np.abs(data_1["mot_cur"]) > 100 * 1000:
                current_window_01.append(0)
            else:
                current_window_01.append(data_1["mot_cur"])

            if np.abs(data_2["mot_cur"]) > 100 * 1000:
                current_window_02.append(0)
            else:
                current_window_02.append(data_2["mot_cur"])

            filtered_current_01 = sum(current_window_01) / MA_WINDOW_SIZE
            filtered_current_02 = sum(current_window_02) / MA_WINDOW_SIZE

            filtered_case_temperature_01 = sum(case_temperature_01) / MA_WINDOW_SIZE
            filtered_case_temperature_02 = sum(case_temperature_02) / MA_WINDOW_SIZE

            local_data.append([t, data_1["mot_cur"], data_2["mot_cur"], data_1["temperature"], data_2["temperature"]])

            print(
                f"Time: {t:0.2f}, \
                    Motor 01 current: {data_1['mot_cur']:0.2f}, \
                    Motor 02 current: {data_2['mot_cur']:0.2f}",
                end="\r",
            )

            _ = thermalModel_1.update_and_get_scale(
                dt=1 / FREQUENCY,
                motor_current=filtered_current_01,
            )

            if filtered_case_temperature_01 >= MAX_CASE_TEMPERATURE:
                print(
                    f"ACTPACK 01: Case thermal limit {MAX_CASE_TEMPERATURE} reached. "
                    f"Current Case Temperature: {data_1['temperature']} C. Exiting."
                )
                break

            if thermalModel_1.T_w >= MAX_WINDING_TEMPERATURE:
                print(
                    f"THERMAL MODEL 01: Winding thermal limit {MAX_WINDING_TEMPERATURE} reached."
                    f"Current Window 01: {current_window_01}."
                    f"Motor current 01: {data_1['mot_cur']} mA."
                    f"Current Winding Temperature: {thermalModel_1.T_w} C. Exiting."
                )
                break

            # # Check for thermal fault, bit 2 of the execute status byte
            # if data_1["status_ex"] & 0b00000010 == 0b00000010:
            #     # "Maximum Average Current" limit exceeded for "time at current limit,
            #     # review physical setup to ensure excessive torque is not normally applied
            #     # If issue persists, review "Maximum Average Current", "Current Limit", and
            #     # "Time at current limit" settings for the Dephy ActPack Firmware using the Plan GUI software
            #     print(f"Condition: {data_1['status_ex'] & 0b00000010}")
            #     print(f"ACTPACK 01: I2t limit exceeded. STATUS EX: {data_1['status_ex']};\
            #        STATUS RE: {data_1['status_re']}; Current: {data_1['mot_cur']} mA. ")
            #     break

            _ = thermalModel_2.update_and_get_scale(
                dt=1 / FREQUENCY,
                motor_current=filtered_current_02,
            )

            if filtered_case_temperature_02 >= MAX_CASE_TEMPERATURE:
                print(
                    f"ACTPACK 02: Case thermal limit {MAX_CASE_TEMPERATURE} reached. "
                    f"Current Case Temperature: {data_2['temperature']} C. Exiting."
                )
                break

            if thermalModel_2.T_w >= MAX_WINDING_TEMPERATURE:
                print(
                    f"THERMAL MODEL 02: Winding thermal limit {MAX_WINDING_TEMPERATURE} reached."
                    f"Current Window 02: {current_window_02}."
                    f"Motor current 02: {data_2['mot_cur']} mA."
                    f"Current Winding Temperature: {thermalModel_2.T_w} C. Exiting."
                )
                break

            # # Check for thermal fault, bit 2 of the execute status byte
            # if data_2["status_ex"] & 0b00000010 == 0b00000010:
            #     # "Maximum Average Current" limit exceeded for "time at current limit,
            #     # review physical setup to ensure excessive torque is not normally applied
            #     # If issue persists, review "Maximum Average Current", "Current Limit", and
            #     # "Time at current limit" settings for the Dephy ActPack Firmware using the Plan GUI software
            #     print(f"Condition: {data_2['status_ex'] & 0b00000010}")
            #     print(f"ACTPACK 02: I2t limit exceeded.  STATUS EX: {data_2['status_ex']}; \
            #       STATUS RE: {data_2['status_re']}; Current: {data_2['mot_cur']} mA. ")
            #     break

            t += 1 / FREQUENCY
            time.sleep(1 / FREQUENCY)

        except KeyboardInterrupt:
            break

    local_np_data = np.array(local_data)
    np.savetxt(
        "dephy_reading_data.csv",
        local_np_data,
        delimiter=",",
        header="Time,Motor_01_Current,Motor_02_Current,Case_01_Temperature,Case_02_Temperature",
        comments="",
        fmt="%.3f",
    )

    device_1.stop_streaming()
    device_2.stop_streaming()

    device_1.close()
    device_2.close()

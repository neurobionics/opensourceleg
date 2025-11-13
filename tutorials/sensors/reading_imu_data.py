from opensourceleg.rust import Logger
from opensourceleg.sensors.imu import LordMicrostrainIMU
from opensourceleg.utilities import SoftRealtimeLoop

FREQUENCY = 200
DT = 1 / FREQUENCY

if __name__ == "__main__":
    Logger.update_log_file_configuration(
        log_directory="./logs",
        log_name="reading_imu_data.log",
    )
    clock = SoftRealtimeLoop(dt=DT)
    imu = LordMicrostrainIMU(
        tag="LordMicrostrainIMU",
        port=r"/dev/ttyS0",
        baud_rate=921600,
        frequency=FREQUENCY,
        update_timeout=500,
        max_packets=1,
        return_packets=False,
        offline=False,
    )
    Logger.track_functions({"Roll": lambda: imu.roll, "Pitch": lambda: imu.pitch, "Yaw": lambda: imu.yaw})

    with imu:
        for t in clock:
            imu.update()
            Logger.info(f"Time: {t}; Roll: {imu.roll}; Pitch: {imu.pitch}; Yaw: {imu.yaw};")
            Logger.record()

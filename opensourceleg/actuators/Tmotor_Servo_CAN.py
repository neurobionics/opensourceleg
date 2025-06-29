import can
import time
import csv
import traceback
from enum import Enum
import numpy as np
import warnings
import os

# Control mode contain {0,1,2,3,4,5,6,7} Seven eigenvalues correspond to seven control modes
# respectively
# Duty cycle mode: 0
# Current loop mode: 1
# Current brake mode: 2
# Velocity mode: 3
# Position mode: 4
# Set origin mode:5
# Position velocity loop mode :6
# Parameter dictionary for each specific motor that can be controlled with this library
# Thresholds are in the datasheet for the motor on cubemars.com
# Verified Error codes for servo motor

Servo_Params = {
        'ERROR_CODES':{
            0 : 'No Error',
            1 : 'Over voltage fault',           # FAULT_CODE_OVER_VOLTAGE
            2 : 'Under voltage fault',          # FAULT_CODE_UNDER_VOLTAGE  
            3 : 'DRV fault',                    # FAULT_CODE_DRV
            4 : 'Absolute over current fault',  # FAULT_CODE_ABS_OVER_CURRENT
            5 : 'Over temp FET fault',          # FAULT_CODE_OVER_TEMP_FET
            6 : 'Over temp motor fault',        # FAULT_CODE_OVER_TEMP_MOTOR
            7 : 'Gate driver over voltage fault', # FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE
            8 : 'Gate driver under voltage fault', # FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE
            9 : 'MCU under voltage fault',      # FAULT_CODE_MCU_UNDER_VOLTAGE
            10: 'Booting from watchdog reset fault', # FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET
            11: 'Encoder SPI fault',            # FAULT_CODE_ENCODER_SPI
            12: 'Encoder sincos below min amplitude fault', # FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE
            13: 'Encoder sincos above max amplitude fault', # FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE
            14: 'Flash corruption fault',       # FAULT_CODE_FLASH_CORRUPTION
            15: 'High offset current sensor 1 fault', # FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1
            16: 'High offset current sensor 2 fault', # FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2
            17: 'High offset current sensor 3 fault', # FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3
            18: 'Unbalanced currents fault'     # FAULT_CODE_UNBALANCED_CURRENTS
        },
        'AK10-9':{
            'P_min' : -360000000,  # -36000 deg (based on protocol doc)
            'P_max' : 360000000,   # 36000 deg (based on protocol doc)
            'V_min' : -100000,     # -100000 ERPM electrical speed
            'V_max' : 100000,      # 100000 ERPM electrical speed
            'Curr_min': -60000,    # -60A (protocol doc: -60000-60000 represents -60A-60A)
            'Curr_max': 60000,     # 60A (protocol doc: -60000-60000 represents -60A-60A)
            'T_min' : -15,         # NM
            'T_max' : 15,          # NM
            'Temp_min': -20,       # -20°C (based on protocol doc)
            'Temp_max': 127,       # 127°C (based on protocol doc)
            'Kt_TMotor' : 0.16,    # from TMotor website (actually 1/Kvll)
            'Current_Factor' : 0.59, # UNTESTED CONSTANT!
            'Kt_actual': 0.206,    # UNTESTED CONSTANT!
            'GEAR_RATIO': 9.0, 
            'NUM_POLE_PAIRS': 21,
            'Use_derived_torque_constants': False, # true if you have a better model
        },
        'AK80-9':{
            'P_min' : -360000000,  # -36000 deg (based on protocol doc)
            'P_max' : 360000000,   # 36000 deg (based on protocol doc)
            'V_min' : -100000,     # -100000 ERPM electrical speed (same as AK10-9)
            'V_max' : 100000,      # 100000 ERPM electrical speed (same as AK10-9)
            'Curr_min': -60000,    # -60A (protocol doc: -60000-60000 represents -60A-60A)
            'Curr_max': 60000,     # 60A (protocol doc: -60000-60000 represents -60A-60A)
            'T_min' : -30,         # NM
            'T_max' : 30,          # NM
            'Temp_min': -20,       # -20°C (based on protocol doc)
            'Temp_max': 127,       # 127°C (based on protocol doc)
            'Kt_TMotor' : 0.091,   # from TMotor website (actually 1/Kvll)
            'Current_Factor' : 0.59,
            'Kt_actual': 0.115,
            'GEAR_RATIO': 9.0, 
            'NUM_POLE_PAIRS': 21,
            'Use_derived_torque_constants': False, # true if you have a better model
        },
        'CAN_PACKET_ID':{
            'CAN_PACKET_SET_DUTY':0,        # Motor runs in duty cycle mode
            'CAN_PACKET_SET_CURRENT':1,     # Motor runs in current loop mode
            'CAN_PACKET_SET_CURRENT_BRAKE':2, # Motor current brake mode operation
            'CAN_PACKET_SET_RPM':3,         # Motor runs in velocity mode
            'CAN_PACKET_SET_POS':4,         # Motor runs in position loop mode
            'CAN_PACKET_SET_ORIGIN_HERE':5, # Set origin mode
            'CAN_PACKET_SET_POS_SPD':6,     # Position velocity loop mode
        },
}
"""
A dictionary with the parameters needed to control the motor
"""

class servo_motor_state:
    """Data structure to store and update motor states"""
    def __init__(self,position, velocity, current, temperature, error, acceleration):
        """
        Sets the motor state to the input.

        Args:
            position: Position in rad
            velocity: Velocity in rad/s
            current: current in amps
            temperature: temperature in degrees C
            error: error code, 0 means no error
            acceleration: acceleration in rad/s
        """
        self.set_state(position, velocity, current, temperature, error, acceleration)

    def set_state(self, position, velocity, current, temperature, error, acceleration):
        """
        Sets the motor state to the input.

        Args:
            position: Position in rad
            velocity: Velocity in rad/s
            current: current in amps
            temperature: temperature in degrees C
            error: error code, 0 means no error
            acceleration: acceleration in rad/s
        """
        self.position = position
        self.velocity = velocity
        self.current = current
        self.temperature = temperature
        self.error = error
        self.acceleration = acceleration

    def set_state_obj(self, other_motor_state):
        """
        Sets this motor state object's values to those of another motor state object.

        Args:
            other_motor_state: The other motor state object with values to set this motor state object's values to.
        """
        self.position = other_motor_state.position
        self.velocity = other_motor_state.velocity
        self.current = other_motor_state.current
        self.temperature = other_motor_state.temperature
        self.error = other_motor_state.error
        self.acceleration = other_motor_state.acceleration

    def __str__(self):
        return 'Position: {} | Velocity: {} | Current: {} | Temperature: {} | Error: {}'.format(self.position, self.velocity, self.current, self.temperature, self.error)

class servo_command:
    """Data structure to store Servo command that will be sent upon update"""
    def __init__(self, position, velocity, current, duty, acceleration):
        """
        Sets the motor state to the input.

        Args:
            position: Position in deg
            velocity: Velocity in ERPM
            current: Current in amps
            duty: Duty cycle in percentage ratio (-1 to 1)
            acceleration: acceleration in ERPMs
        """
        self.position = position
        self.velocity = velocity
        self.current = current
        self.duty = duty
        self.acceleration = acceleration

# # motor state from the controller, uneditable named tuple
# servo_motor_state = namedtuple('motor_state', 'position velocity current temperature error')
# """
# Motor state from the controller, uneditable named tuple
# """

# python-can listener object, with handler to be called upon reception of a message on the CAN bus
class motorListener(can.Listener):
    """Python-can listener object, with handler to be called upon reception of a message on the CAN bus"""
    def __init__(self, canman, motor):
        """
        Sets stores can manager and motor object references
        
        Args:
            canman: The CanManager object to get messages from
            motor: The TMotorCANManager object to update
        """
        self.canman = canman
        self.bus = canman.bus
        self.motor = motor

    def on_message_received(self, msg):
        """
        Updates this listener's motor with the info contained in msg, if that message was for this motor.

        args:
            msg: A python-can CAN message
        """
        data = bytes(msg.data)
        ID = msg.arbitration_id & 0x00000FF
        if ID == self.motor.ID:
            self.motor._update_state_async(self.canman.parse_servo_message(data))

# A class to manage the low level CAN communication protocols
class CAN_Manager_servo(object):
    """A class to manage the low level CAN communication protocols"""
    debug = False
    """
    Set to true to display every message sent and received for debugging.
    """
    _instance = None
    """
    Used to keep track of one instantiation of the class to make a singleton object
    """
    
    def __new__(cls):
        """
        Makes a singleton object to manage socketcan CAN bus for Raspberry Pi.
        """
        if not cls._instance:
            cls._instance = super(CAN_Manager_servo, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        """
        Initialize CAN bus connection for Raspberry Pi
        """
        if self._initialized:
            return
            
        print("Initializing CAN Manager for Raspberry Pi")
        
        try:
            # Configure CAN interface
            os.system('sudo /sbin/ip link set can0 down')
            os.system('sudo /sbin/ip link set can0 up type can bitrate 1000000')
            os.system('sudo ifconfig can0 txqueuelen 1000')
            
            # Create CAN bus object using socketcan (standard for Linux/RPi)
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            # Create notifier for message handling
            self.notifier = can.Notifier(bus=self.bus, listeners=[])
            
            print("Connected on: " + str(self.bus))
            self._initialized = True
            
        except Exception as e:
            print(f"CAN bus initialization failed: {e}")
            print("Make sure:")
            print("1. CAN interface is available (sudo ip link show can0)")
            print("2. User has sudo privileges")
            print("3. can-utils is installed (sudo apt install can-utils)")
            raise RuntimeError("CAN bus initialization failed")

    def __del__(self):
        """
        # shut down the CAN bus when the object is deleted
        # This may not ever get called, so keep a reference and explicitly delete if this is important.
        """
        os.system( 'sudo /sbin/ip link set can0 down' ) 

    # subscribe a motor object to the CAN bus to be updated upon message reception
    def add_motor(self, motor):
        """
        Subscribe a motor object to the CAN bus to be updated upon message reception

        Args:
            motor: The TMotorManager object to be subscribed to the notifier
        """
        self.notifier.add_listener(motorListener(self, motor))

#* Buffer information for servo mode data manipulation

#******************START****************************#
    # Simplified buffer functions for servo mode data manipulation
    @staticmethod
    def pack_int16(number):
        """Pack 16-bit integer into byte list"""
        return [(number >> 8) & 0xFF, number & 0xFF]
    
    @staticmethod
    def pack_uint16(number):
        """Pack unsigned 16-bit integer into byte list"""
        return [(number >> 8) & 0xFF, number & 0xFF]
       
    @staticmethod
    def pack_int32(number):
        """Pack 32-bit integer into byte list"""
        return [(number >> 24) & 0xFF, (number >> 16) & 0xFF, 
                (number >> 8) & 0xFF, number & 0xFF]

    @staticmethod
    def pack_uint32(number):
        """Pack unsigned 32-bit integer into byte list"""
        return [(number >> 24) & 0xFF, (number >> 16) & 0xFF, 
                (number >> 8) & 0xFF, number & 0xFF]

    # Legacy methods for backward compatibility
    @staticmethod
    def buffer_append_int16(buffer, number, index=None):
        """Legacy method - use pack_int16() instead"""
        buffer.extend(CAN_Manager_servo.pack_int16(number))
    
    @staticmethod
    def buffer_append_uint16(buffer, number, index=None):
        """Legacy method - use pack_uint16() instead"""
        buffer.extend(CAN_Manager_servo.pack_uint16(number))
       
    @staticmethod
    def buffer_append_int32(buffer, number, index=None):
        """Legacy method - use pack_int32() instead"""
        buffer.extend(CAN_Manager_servo.pack_int32(number))

    @staticmethod
    def buffer_append_uint32(buffer, number, index=None):
        """Legacy method - use pack_uint32() instead"""
        buffer.extend(CAN_Manager_servo.pack_uint32(number))

    # Buffer allocation for 64 bit
    @staticmethod
    def buffer_append_int64(buffer, number, index=None):
        """
        buffer size for int 64

        Args:
            Buffer: memory allocated to store data.
            number: value.
            index: optional index pointer (for compatibility)
        """
        buffer.append((number >> 56) & 0xFF)
        buffer.append((number >> 48) & 0xFF)
        buffer.append((number >> 40) & 0xFF)
        buffer.append((number >> 32) & 0xFF)
        buffer.append((number >> 24) & 0xFF)
        buffer.append((number >> 16) & 0xFF)
        buffer.append((number >> 8) & 0xFF)
        buffer.append(number & 0xFF)

    # Buffer allocation for Unsigned 64 bit
    @staticmethod
    def buffer_append_uint64(buffer, number, index=None):
        """
        buffer size for uint 64

        Args:
            Buffer: memory allocated to store data.
            number: value.
            index: optional index pointer (for compatibility)
        """
        buffer.append((number >> 56) & 0xFF)
        buffer.append((number >> 48) & 0xFF)
        buffer.append((number >> 40) & 0xFF)
        buffer.append((number >> 32) & 0xFF)
        buffer.append((number >> 24) & 0xFF)
        buffer.append((number >> 16) & 0xFF)
        buffer.append((number >> 8) & 0xFF)
        buffer.append(number & 0xFF)


#******************END****************************#

#* Sends data via CAN
    # sends a message to the motor (when the motor is in Servo mode)
    def send_servo_message(self, motor_id, data,data_len):
        """
        Sends a Servo Mode message to the motor, with a header of motor_id and data array of data

        Args:
            motor_id: The CAN ID of the motor to send to.
            data: An array of integers or bytes of data to send.
        """
        DLC = data_len
        assert (DLC <= 8), ('Data too long in message for motor ' + str(motor_id))
        
        if self.debug:
            print('ID: ' + str(hex(motor_id)) + '   Data: ' + '[{}]'.format(', '.join(hex(d) for d in data)) )
        
        message = can.Message(arbitration_id=motor_id, data=data, is_extended_id=True)

        try:
            self.bus.send(message)
            if self.debug:
                print("    Message sent on " + str(self.bus.channel_info) )
        except can.CanError as e:
            if self.debug:
                print("    Message NOT sent: " + e.message)

    # send the power on code
    def power_on(self, motor_id):
        """
        Sends the power on code to motor_id.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
            Data: This is obtained from the datasheet.
        """

        self.send_servo_message(motor_id, [ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFC], 8)
        
    # send the power off code
    def power_off(self, motor_id):
        """
        Sends the power off code to motor_id.

        Args:
            motor_id: The CAN ID of the motor to send the message to.
        """
        self.send_servo_message(motor_id, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD], 8)


#* Code for the working of different modes in servo mode. 
   
    #* ********************START*******************************************#
    #TODO: Controller id vs motorID
    # Send Servo control message for duty cycle mode
    #*Duty cycle mode: duty cycle voltage is specified for a given motor, similar to squarewave drive mode
    def comm_can_set_duty(self, controller_id, duty):
        """
        Send a servo control message for duty cycle mode

        Args:
            controller_id: CAN ID of the motor to send the message to
            duty: duty cycle (-1 to 1) to use
        """
        buffer = []
        self.buffer_append_int32(buffer, np.int32(duty * 100000.0))
        self.send_servo_message(controller_id|(Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_DUTY'] << 8), buffer, len(buffer))

    # Send Servo control message for current loop mode
    #*Current loop mode: given the Iq current specified by the motor, the motor output torque = Iq *KT, so it can be used as a torque loop
    def comm_can_set_current(self, controller_id, current):
        """
        Send a servo control message for current loop mode

        Args:
            controller_id: CAN ID of the motor to send the message to
            current: current in Amps to use (-60 to 60)
        """
        buffer = []
        self.buffer_append_int32(buffer, np.int32(current * 1000.0))
        self.send_servo_message(controller_id|(Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_CURRENT'] << 8), buffer, len(buffer))

    # Send Servo control message for current brake mode
    #*Current brake mode: the motor is fixed at the current position by the specified brake current given by the motor (pay attention to the motor temperature when using)
    def comm_can_set_cb(self, controller_id, current):
        """
        Send a servo control message for current brake mode

        Args:
            controller_id: CAN ID of the motor to send the message to
            current: current in Amps to use (0 to 60)
        """
        buffer = []
        self.buffer_append_int32(buffer, np.int32(current * 1000.0))
        self.send_servo_message(controller_id|(Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_CURRENT_BRAKE'] << 8), buffer, len(buffer))
        
    # Send Servo control message for Velocity mode
    #*Velocity mode: the speed specified by the given motor
    def comm_can_set_rpm(self, controller_id, rpm):
        """
        Send a servo control message for velocity control mode

        Args:
            controller_id: CAN ID of the motor to send the message to
            rpm: velocity in ERPM (-100000 to 100000)
        """
        buffer = []
        self.buffer_append_int32(buffer, np.int32(rpm))
        self.send_servo_message(controller_id| (Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_RPM'] << 8), buffer, len(buffer))
    
    # Send Servo control message for Position Loop mode
    #*Position mode: Given the specified position of the motor, the motor will run to the specified position, (default speed 12000erpm acceleration 40000erpm)
    def comm_can_set_pos(self, controller_id, pos):
        """
        Send a servo control message for position control mode

        Args:
            controller_id: CAN ID of the motor to send the message to
            pos: desired position in degrees
        """
        buffer = []
        # Use 10000.0 to match Arduino implementation, not 1000000.0 from protocol doc
        self.buffer_append_int32(buffer, np.int32(pos * 10000.0))
        self.send_servo_message(controller_id|(Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_POS'] << 8), buffer, len(buffer))
    
    #Set origin mode
    #*0 means setting the temporary origin (power failure elimination), 1 means setting the permanent zero point (automatic parameter saving), 2 means restoring the default zero point (automatic parameter saving)
    def comm_can_set_origin(self, controller_id, set_origin_mode):
        """
        set the origin

        Args:
            controller_id: CAN ID of the motor to send the message to
            set_origin_mode: 0 means setting the temporary origin (power failure elimination), 1 means setting the permanent zero point (automatic parameter saving), 2 means restoring the default zero point (automatic parameter saving)
        """
        buffer = [set_origin_mode]
        self.send_servo_message(controller_id |(Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_ORIGIN_HERE'] << 8), buffer, len(buffer))

    #Position and Velocity Loop Mode
    #* Check documentation
    def comm_can_set_pos_spd(self, controller_id, pos, spd, RPA):
        """
        Send a servo control message for position control mode, with specified velocity and acceleration
        This will be a trapezoidal speed profile.

        Args:
            controller_id: CAN ID of the motor to send the message to
            pos: desired position in degrees
            spd: desired max speed in ERPM
            RPA: desired acceleration in ERPM/s
        """
        buffer = []
        # Match Arduino implementation: pos * 10000.0, spd/10, RPA/10
        self.buffer_append_int32(buffer, np.int32(pos * 10000.0))
        self.buffer_append_int16(buffer, np.int16(spd / 10.0))
        self.buffer_append_int16(buffer, np.int16(RPA / 10.0))
        self.send_servo_message(controller_id |(Servo_Params['CAN_PACKET_ID']['CAN_PACKET_SET_POS_SPD'] << 8), buffer, len(buffer))

    #* **************************END************************************************#
 


#*****************Parsing message data********************************#
    def parse_servo_message(self, data):
        """
        Unpack the servo message into a servo_motor_state object

        Args:
            data: bytes of the message to be processed

        Returns:
            A servo_motor_state object representing the state based on the data received.
        """
        # using numpy to convert signed/unsigned integers
        pos_int = np.int16(data[0] << 8 | data[1])
        spd_int = np.int16(data[2] << 8 | data[3])
        cur_int = np.int16(data[4] << 8 | data[5])
        motor_pos= float( pos_int * 0.1) # motor position
        motor_spd= float( spd_int * 10.0) # motor speed
        motor_cur= float( cur_int * 0.01) # motor current
        motor_temp= np.int16(data[6])  # motor temperature
        motor_error= data[7] # motor error mode
        if self.debug:
            print(data)
            print('  Position: ' + str(motor_pos))
            print('  Velocity: ' + str(motor_spd))
            print('  Current: ' + str(motor_cur))
            print('  Temp: ' + str(motor_temp))
            print('  Error: ' + str(motor_error))
            
        return servo_motor_state(motor_pos, motor_spd,motor_cur,motor_temp, motor_error, 0)


# default variables to be logged
LOG_VARIABLES = [
        "motor_position" , 
        "motor_speed" , 
        "motor_current", 
        "motor_temperature" 
]
"""
default variables to be logged
"""

# possible states for the controller
class _TMotorManState_Servo(Enum):
    """
    An Enum to keep track of different control states
    """
    DUTY_CYCLE = 0
    CURRENT_LOOP = 1
    CURRENT_BRAKE = 2
    VELOCITY = 3
    POSITION = 4
    SET_ORIGIN=5
    POSITION_VELOCITY=6
    IDLE = 7

# the user-facing class that manages the motor.
class TMotorManager_servo_can():
    """
    The user-facing class that manages the motor. This class should be
    used in the context of a with as block, in order to safely enter/exit
    control of the motor.
    """
    def __init__(self, motor_type='AK80-9', motor_ID=1, max_mosfett_temp = 50, CSV_file=None, log_vars = LOG_VARIABLES):
        """
        Sets up the motor manager. Note the device will not be powered on by this method! You must
        call __enter__, mostly commonly by using a with block, before attempting to control the motor.

        Args:
            motor_type: The type of motor being controlled, ie AK80-9.
            motor_ID: The CAN ID of the motor.
            max_mosfett_temp: temperature of the mosfett above which to throw an error, in Celsius
            CSV_file: A CSV file to output log info to. If None, no log will be recorded.
            log_vars: The variables to log as a python list. The full list of possibilities is
                - "output_angle"
                - "output_velocity"
                - "output_acceleration"
                - "current"
                - "output_torque"
                - "motor_angle"
                - "motor_velocity"
                - "motor_acceleration"
                - "motor_torque"
        """
        self.type = motor_type
        self.ID = motor_ID
        self.csv_file_name = CSV_file
        self.max_temp = max_mosfett_temp # max temp in deg C, can update later
        print("Initializing device: " + self.device_info_string())

        self._motor_state = servo_motor_state(0.0,0.0,0.0,0.0,0.0,0.0)
        self._motor_state_async = servo_motor_state(0.0,0.0,0.0,0.0,0.0,0.0)
        self._command = servo_command(0.0,0.0,0.0,0.0,0.0)
        self._control_state = _TMotorManState_Servo.IDLE

        self.radps_per_ERPM = 5.82E-04
        self.rad_per_Eang = np.pi/Servo_Params[self.type]['NUM_POLE_PAIRS'] # 2*(np.pi/180)/(Servo_Params[self.type]['NUM_POLE_PAIRS'])

        self._entered = False
        self._start_time = time.time()
        self._last_update_time = self._start_time
        self._last_command_time = None
        self._updated = False
        self._command_sent = False
        
        self.log_vars = log_vars
        self.LOG_FUNCTIONS = {
            "motor_position" : self.get_motor_angle_radians, 
            "motor_speed" : self.get_motor_velocity_radians_per_second, 
            "motor_current" : self.get_current_qaxis_amps, 
            "motor_temperature" : self.get_temperature_celsius,
        }
        
        self._canman = CAN_Manager_servo()
        self._canman.add_motor(self)
               
    def __enter__(self):
        """
        Used to safely power the motor on and begin the log file.
        """
        print('Turning on control for device: ' + self.device_info_string())
        if self.csv_file_name is not None:
            with open(self.csv_file_name,'w') as fd:
                writer = csv.writer(fd)
                writer.writerow(["pi_time"]+self.log_vars)
            self.csv_file = open(self.csv_file_name,'a').__enter__()
            self.csv_writer = csv.writer(self.csv_file)
        self.power_on() #TODO: How to control this?
        self._send_command()
        self._entered = True
        if not self.check_can_connection():
            raise RuntimeError("Device not connected: " + str(self.device_info_string()))
        return self

    def __exit__(self, etype, value, tb):
        """
        Used to safely power the motor off and close the log file.
        """
        print('Turning off control for device: ' + self.device_info_string())
        self.power_off()#TODO: How to control this

        if self.csv_file_name is not None:
            self.csv_file.__exit__(etype, value, tb)

        if not (etype is None):
            traceback.print_exception(etype, value, tb)

    def qaxis_current_to_TMotor_current(self, iq):
        return iq*(Servo_Params[self.type]['GEAR_RATIO']*Servo_Params[self.type]['Kt_TMotor'])/Servo_Params[self.type]['Current_Factor']

    # this method is called by the handler every time a message is received on the bus
    # from this motor, to store the most recent state information for later
    def _update_state_async(self, servo_state):
        """
        This method is called by the handler every time a message is received on the bus
        from this motor, to store the most recent state information for later
        
        Args:
            servo_state: the servo_state object with the updated motor state

        Raises:
            RuntimeError when device sends back an error code that is not 0 (0 meaning no error)
        """
        if servo_state.error != 0:
            raise RuntimeError('Driver board error for device: ' + self.device_info_string() + ": " + Servo_Params['ERROR_CODES'][servo_state.error])

        now = time.time()
        dt = now - self._last_update_time
        self._last_update_time = now
        self._motor_state_async.acceleration = (servo_state.velocity - self._motor_state_async.velocity)/dt
        self._motor_state_async.set_state_obj(servo_state)
        self._updated = True

    
    # this method is called by the user to synchronize the current state used by the controller
    # with the most recent message received
    def update(self):
        """
        This method is called by the user to synchronize the current state used by the controller/logger
        with the most recent message received, as well as to send the current command.
        """
        # check that the motor is safely turned on
        if not self._entered:
            raise RuntimeError("Tried to update motor state before safely powering on for device: " + self.device_info_string())

        if self.get_temperature_celsius() > self.max_temp:
            raise RuntimeError("Temperature greater than {}C for device: {}".format(self.max_temp, self.device_info_string()))
        # check that the motor data is recent
        now = time.time()
        if self._last_command_time is not None and (now - self._last_command_time) < 0.25 and ( (now - self._last_update_time) > 0.1):
            warnings.warn("State update requested but no data from motor. Delay longer after zeroing, decrease frequency, or check connection. " + self.device_info_string(), RuntimeWarning)
        else:
            self._command_sent = False

        self._motor_state.set_state_obj(self._motor_state_async)
        self._motor_state.position = self._motor_state.position/Servo_Params[self.type]["GEAR_RATIO"]
        
        # send current motor command
        self._send_command()

        # writing to log file
        if self.csv_file_name is not None:
            self.csv_writer.writerow([self._last_update_time - self._start_time] + [self.LOG_FUNCTIONS[var]() for var in self.log_vars])

        self._updated = False
        
    # sends a command to the motor depending on whats controlm mode the motor is in
    def _send_command(self):
        """
        Sends a command to the motor depending on whats controlm mode the motor is in. This method
        is called by update(), and should only be called on its own if you don't want to update the motor state info.
        """
        if self._control_state == _TMotorManState_Servo.DUTY_CYCLE:
            self._canman.comm_can_set_duty(self.ID,self._command.duty)
        elif self._control_state == _TMotorManState_Servo.CURRENT_LOOP:
            self._canman.comm_can_set_current(self.ID,self._command.current)
        elif self._control_state == _TMotorManState_Servo.CURRENT_BRAKE:
            self._canman.comm_can_set_cb(self.ID,self._command.current)
        elif self._control_state == _TMotorManState_Servo.VELOCITY:
            self._canman.comm_can_set_rpm(self.ID, self._command.velocity)
        elif self._control_state == _TMotorManState_Servo.POSITION:
            self._canman.comm_can_set_pos(self.ID, self._command.position)
        elif self._control_state == _TMotorManState_Servo.POSITION_VELOCITY:
            self._canman.comm_can_set_pos_spd(self.ID, self._command.position, self._command.velocity, self._command.acceleration)
        elif self._control_state == _TMotorManState_Servo.IDLE:
            self._canman.comm_can_set_duty(self.ID, 0.0)

        #TODO:Add other modes
        else:
            raise RuntimeError("UNDEFINED STATE for device " + self.device_info_string())

        self._last_command_time = time.time()

    # Basic Motor Utility Commands
    def power_on(self):
        """Powers on the motor."""
        self._canman.power_on(self.ID)
        self._updated = True

    def power_off(self):
        """Powers off the motor."""
        self._canman.power_off(self.ID)

    # zeros the position
    def set_zero_position(self):
        """Zeros the position"""
        self._canman.comm_can_set_origin(self.ID,1)
        self._last_command_time = time.time()

    # getters for motor state
    def get_temperature_celsius(self):
        """
        Returns:
            The most recently updated motor temperature in degrees C.
        """
        return self._motor_state.temperature
    
    def get_motor_error_code(self):
        """
        Returns:
            The most recently updated motor error code.
            Note the program should throw a runtime error before you get a chance to read
            this value if it is ever anything besides 0.

        Codes:
            0 : 'No Error',
            1 : 'Over voltage fault',           # FAULT_CODE_OVER_VOLTAGE
            2 : 'Under voltage fault',          # FAULT_CODE_UNDER_VOLTAGE  
            3 : 'DRV fault',                    # FAULT_CODE_DRV
            4 : 'Absolute over current fault',  # FAULT_CODE_ABS_OVER_CURRENT
            5 : 'Over temp FET fault',          # FAULT_CODE_OVER_TEMP_FET
            6 : 'Over temp motor fault',        # FAULT_CODE_OVER_TEMP_MOTOR
            7 : 'Gate driver over voltage fault', # FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE
            8 : 'Gate driver under voltage fault', # FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE
            9 : 'MCU under voltage fault',      # FAULT_CODE_MCU_UNDER_VOLTAGE
            10: 'Booting from watchdog reset fault', # FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET
            11: 'Encoder SPI fault',            # FAULT_CODE_ENCODER_SPI
            12: 'Encoder sincos below min amplitude fault', # FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE
            13: 'Encoder sincos above max amplitude fault', # FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE
            14: 'Flash corruption fault',       # FAULT_CODE_FLASH_CORRUPTION
            15: 'High offset current sensor 1 fault', # FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1
            16: 'High offset current sensor 2 fault', # FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2
            17: 'High offset current sensor 3 fault', # FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3
            18: 'Unbalanced currents fault'     # FAULT_CODE_UNBALANCED_CURRENTS
        """
        return self._motor_state.error

    def get_current_qaxis_amps(self):
        """
        Returns:
            The most recently updated qaxis current in amps
        """
        return self._motor_state.current

    def get_output_angle_radians(self):
        """
        Returns:
            The most recently updated output angle in radians
        """
        return self._motor_state.position*self.rad_per_Eang

    def get_output_velocity_radians_per_second(self):
        """
        Returns:
            The most recently updated output velocity in radians per second
        """
        return self._motor_state.velocity*self.radps_per_ERPM

    def get_output_acceleration_radians_per_second_squared(self):
        """
        Returns:
            The most recently updated output acceleration in radians per second per second
        """
        return self._motor_state.acceleration

    def get_output_torque_newton_meters(self):
        """
        Returns:
            the most recently updated output torque in Nm
        """
        return self.get_current_qaxis_amps()*Servo_Params[self.type]["Kt_actual"]*Servo_Params[self.type]["GEAR_RATIO"]

    def enter_duty_cycle_control(self):
        """
        Must call this to enable sending duty cycle commands.
        """
        self._control_state = _TMotorManState_Servo.DUTY_CYCLE

    def enter_current_control(self):
        """
        Must call this to enable sending current commands.
        """
        self._control_state = _TMotorManState_Servo.CURRENT_LOOP

    def enter_current_brake_control(self):
        """
        Must call this to enable sending current brake commands.
        """
        self._control_state = _TMotorManState_Servo.CURRENT_BRAKE

    def enter_velocity_control(self):
        """
        Must call this to enable sending velocity commands.
        """
        self._control_state = _TMotorManState_Servo.VELOCITY

    def enter_position_control(self):
        """
        Must call this to enable position commands.
        """
        self._control_state = _TMotorManState_Servo.POSITION

    def enter_position_velocity_control(self):
        """
        Must call this to enable sending position commands with specified velocity and accleration limits.
        """
        self._control_state = _TMotorManState_Servo.POSITION_VELOCITY

    def enter_idle_mode(self):
        """
        Enter the idle state, where duty cycle is set to 0. (This is the default state.)
        """
        self._control_state = _TMotorManState_Servo.IDLE

    # used for either impedance or MIT mode to set output angle
    def set_output_angle_radians(self, pos, vel, acc):
        """
        Update the current command to the desired position, when in position or position-velocity mode.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            pos: The desired output angle in rad
            vel: The desired speed to get there in rad/s (when in POSITION_VELOCITY mode)
            acc: The desired acceleration to get there in rad/s/s, ish (when in POSITION_VELOCITY mode)
        """
        # Convert to degrees for range checking (protocol doc specifies range in degrees)
        pos_degrees = pos * 180.0 / np.pi
        if np.abs(pos_degrees) >= 36000:  # Based on protocol doc: -36000° to 36000°
            raise RuntimeError("Cannot control position with magnitude greater than " + str(36000) + " degrees (630 radians)!")
        
        pos = (pos / self.rad_per_Eang)
        vel = (vel / self.radps_per_ERPM)
        acc = (acc / self.radps_per_ERPM)
        if self._control_state == _TMotorManState_Servo.POSITION_VELOCITY:
            self._command.position = pos
            self._command.velocity = vel
            self._command.acceleration = acc
        elif self._control_state == _TMotorManState_Servo.POSITION:
            self._command.position = pos
        else:
            raise RuntimeError("Attempted to send position command without entering position control " + self.device_info_string())

    def set_duty_cycle_percent(self, duty):
        """
        Used for duty cycle mode, to set desired duty cycle.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            duty: The desired duty cycle, (-1 to 1)
        """
        if self._control_state not in [_TMotorManState_Servo.DUTY_CYCLE]:
            raise RuntimeError("Attempted to send duty cycle command without gains for device " + self.device_info_string()) 
        else:
            if np.abs(duty) > 1:
                raise RuntimeError("Cannot control using duty cycle mode for duty cycles greater than 100%!")
            self._command.duty = duty

    def set_output_velocity_radians_per_second(self, vel):
        """
        Used for velocity mode to set output velocity command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.

        Args:
            vel: The desired output speed in rad/s
        """
        if np.abs(vel) >= Servo_Params[self.type]["V_max"]:
            raise RuntimeError("Cannot control using speed mode for velocities with magnitude greater than " + str(Servo_Params[self.type]["V_max"]) + "rad/s!")

        if self._control_state not in [_TMotorManState_Servo.VELOCITY]:
            raise RuntimeError("Attempted to send speed command without gains for device " + self.device_info_string()) 
        self._command.velocity = vel/self.radps_per_ERPM

    # used for either current MIT mode to set current
    def set_motor_current_qaxis_amps(self, current):
        """
        Used for current mode to set current command.
        Note, this does not send a command, it updates the TMotorManager's saved command,
        which will be sent when update() is called.
        
        Args:
            current: the desired current in amps.
        """
        if self._control_state not in [_TMotorManState_Servo.CURRENT_LOOP, _TMotorManState_Servo.CURRENT_BRAKE]:
            raise RuntimeError("Attempted to send current command before entering current mode for device " + self.device_info_string()) 
        
        # Check current limits based on control mode
        if self._control_state == _TMotorManState_Servo.CURRENT_BRAKE:
            # Current brake mode: 0-60A only (based on protocol doc)
            if current < 0 or current > 60:
                raise RuntimeError(f"Current brake mode requires current between 0-60A, got {current}A")
        else:
            # Current loop mode: -60A to 60A
            if current < -60 or current > 60:
                raise RuntimeError(f"Current loop mode requires current between -60A to 60A, got {current}A")
        
        self._command.current = current

    # used for either current or MIT Mode to set current, based on desired torque
    def set_output_torque_newton_meters(self, torque):
        """
        Used for current mode to set current, based on desired torque.
        If a more complicated torque model is available for the motor, that will be used.
        Otherwise it will just use the motor's torque constant.
        
        Args:
            torque: The desired output torque in Nm.
        """
        self.set_motor_current_qaxis_amps((torque/Servo_Params[self.type]["Kt_actual"]/Servo_Params[self.type]["GEAR_RATIO"]) )

    # motor-side functions to account for the gear ratio
    def set_motor_torque_newton_meters(self, torque):
        """
        Wrapper of set_output_torque that accounts for gear ratio to control motor-side torque
        
        Args:
            torque: The desired motor-side torque in Nm.
        """
        self.set_output_torque_newton_meters(torque*Servo_Params[self.type]["GEAR_RATIO"])

    def set_motor_angle_radians(self, pos, vel=0.0, acc=0.0):
        """
        Wrapper for set_output_angle that accounts for gear ratio to control motor-side angle
        
        Args:
            pos: The desired motor-side position in rad.
            vel: The desired velocity in rad/s (default 0.0 for position-only mode)
            acc: The desired acceleration in rad/s/s (default 0.0 for position-only mode)
        """
        self.set_output_angle_radians(pos/(Servo_Params[self.type]["GEAR_RATIO"]), vel, acc)

    def set_motor_velocity_radians_per_second(self, vel):
        """
        Wrapper for set_output_velocity that accounts for gear ratio to control motor-side velocity
        
        Args:
            vel: The desired motor-side velocity in rad/s.
        """
        self.set_output_velocity_radians_per_second(vel/(Servo_Params[self.type]["GEAR_RATIO"]) )

    def get_motor_angle_radians(self):
        """
        Wrapper for get_output_angle that accounts for gear ratio to get motor-side angle
        
        Returns:
            The most recently updated motor-side angle in rad.
        """
        return self._motor_state.position*self.rad_per_Eang*Servo_Params[self.type]["GEAR_RATIO"]

    def get_motor_velocity_radians_per_second(self):
        """
        Wrapper for get_output_velocity that accounts for gear ratio to get motor-side velocity
        
        Returns:
            The most recently updated motor-side velocity in rad/s.
        """
        return self._motor_state.velocity*self.radps_per_ERPM*Servo_Params[self.type]["GEAR_RATIO"]

    def get_motor_acceleration_radians_per_second_squared(self):
        """
        Wrapper for get_output_acceleration that accounts for gear ratio to get motor-side acceleration
        
        Returns:
            The most recently updated motor-side acceleration in rad/s/s.
        """
        return self._motor_state.acceleration*Servo_Params[self.type]["GEAR_RATIO"]

    def get_motor_torque_newton_meters(self):
        """
        Wrapper for get_output_torque that accounts for gear ratio to get motor-side torque
        
        Returns:
            The most recently updated motor-side torque in Nm.
        """
        return self.get_output_torque_newton_meters()/Servo_Params[self.type]["GEAR_RATIO"]

    # Simple setter methods for property compatibility
    def set_output_angle_simple(self, pos):
        """Simple setter for position property (only position, vel=0, acc=0)"""
        self.set_output_angle_radians(pos, 0.0, 0.0)

    def set_motor_angle_simple(self, pos):
        """Simple setter for motor angle property"""
        self.set_motor_angle_radians(pos)

    # Pretty stuff
    def __str__(self):
        """Prints the motor's device info and current"""
        return self.device_info_string() + " | Position: " + '{:.3f}'.format(self.position) + " rad | Velocity: " + '{:.3f}'.format(self.velocity) + " rad/s | current: " + '{:.3f}'.format(self.current_qaxis) + " A | temp: " + '{:.0f}'.format(self.temperature) + " C"

    def device_info_string(self):
        """Prints the motor's ID and device type."""
        return str(self.type) + "  ID: " + str(self.ID)

    # Checks the motor connection by sending a 10 commands and making sure the motor responds.
    def check_can_connection(self):
        """
        Checks the motor's connection by attempting to send 10 startup messages.
        If it gets 10 replies, then the connection is confirmed.

        Returns:
            True if a connection is established and False otherwise.
        """
        if not self._entered:
            raise RuntimeError("Tried to check_can_connection before entering motor control! Enter control using the __enter__ method, or instantiating the TMotorManager in a with block.")
        Listener = can.BufferedReader()
        self._canman.notifier.add_listener(Listener)
        for i in range(10):
            self.power_on()
            time.sleep(0.001)
        success = True
        time.sleep(0.1)  # Allow time for responses
        for i in range(10):
            if Listener.get_message(timeout=0.1) is None:  # Check for replies
                success = False
        self._canman.notifier.remove_listener(Listener)  # Remove listener to prevent memory leaks
        return success

    # Properties
    temperature = property(get_temperature_celsius, doc="temperature_degrees_C")
    """Temperature in Degrees Celsius"""

    error = property(get_motor_error_code, doc="motor_error_code")
    """Motor error code. 0 means no error.
    
    Codes:
        0 : 'No Error',
        1 : 'Over voltage fault',           # FAULT_CODE_OVER_VOLTAGE
        2 : 'Under voltage fault',          # FAULT_CODE_UNDER_VOLTAGE  
        3 : 'DRV fault',                    # FAULT_CODE_DRV
        4 : 'Absolute over current fault',  # FAULT_CODE_ABS_OVER_CURRENT
        5 : 'Over temp FET fault',          # FAULT_CODE_OVER_TEMP_FET
        6 : 'Over temp motor fault',        # FAULT_CODE_OVER_TEMP_MOTOR
        7 : 'Gate driver over voltage fault', # FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE
        8 : 'Gate driver under voltage fault', # FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE
        9 : 'MCU under voltage fault',      # FAULT_CODE_MCU_UNDER_VOLTAGE
        10: 'Booting from watchdog reset fault', # FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET
        11: 'Encoder SPI fault',            # FAULT_CODE_ENCODER_SPI
        12: 'Encoder sincos below min amplitude fault', # FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE
        13: 'Encoder sincos above max amplitude fault', # FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE
        14: 'Flash corruption fault',       # FAULT_CODE_FLASH_CORRUPTION
        15: 'High offset current sensor 1 fault', # FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1
        16: 'High offset current sensor 2 fault', # FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2
        17: 'High offset current sensor 3 fault', # FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3
        18: 'Unbalanced currents fault'     # FAULT_CODE_UNBALANCED_CURRENTS
    """

    # electrical variables
    current_qaxis = property(get_current_qaxis_amps, set_motor_current_qaxis_amps, doc="current_qaxis_amps_current_only")
    """Q-axis current in amps"""

    # output-side variables
    position = property(get_output_angle_radians, set_output_angle_simple, doc="output_angle_radians_impedance_only")
    """Output angle in rad"""

    velocity = property (get_output_velocity_radians_per_second, set_output_velocity_radians_per_second, doc="output_velocity_radians_per_second")
    """Output velocity in rad/s"""

    acceleration = property(get_output_acceleration_radians_per_second_squared, doc="output_acceleration_radians_per_second_squared")
    """Output acceleration in rad/s/s"""

    torque = property(get_output_torque_newton_meters, set_output_torque_newton_meters, doc="output_torque_newton_meters")
    """Output torque in Nm"""

    # motor-side variables       
    angle_motorside = property(get_motor_angle_radians, set_motor_angle_simple, doc="motor_angle_radians_impedance_only")
    """Motor-side angle in rad"""
    
    velocity_motorside = property (get_motor_velocity_radians_per_second, set_motor_velocity_radians_per_second, doc="motor_velocity_radians_per_second")
    """Motor-side velocity in rad/s"""

    acceleration_motorside = property(get_motor_acceleration_radians_per_second_squared, doc="motor_acceleration_radians_per_second_squared")
    """Motor-side acceleration in rad/s/s"""

    torque_motorside = property(get_motor_torque_newton_meters, set_motor_torque_newton_meters, doc="motor_torque_newton_meters")
    """Motor-side torque in Nm"""






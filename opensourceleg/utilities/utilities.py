import ctypes
import glob
import sys

import serial


def get_active_ports():
    """
    Lists active serial ports.
    """
    if sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        ports = glob.glob("/dev/tty[A-Za-z]C*")
    elif sys.platform.startswith("darwin"):
        ports = glob.glob("/dev/tty.*")
    elif sys.platform.startswith("win"):
        ports = ["COM%s" % (i + 1) for i in range(256)]
    else:
        raise OSError("Unsupported platform.")

    serial_ports = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            serial_ports.append(port)
        except (OSError, serial.SerialException):
            pass

    return serial_ports


def get_ctype_args(input_header: str):
    """
    Converts a header file from C string into a list of ctypes arguments.

    Keyword Arguments:
        inputHeader: string from header file, such as "const struct0_T *thighIMU, double Knee_joint_position,
          double Ankle_joint_position"
    returns:
        ctypes list of the appropriate types for the inputs, such as (ctypes.c_void_p, ctypes.c_double, ctypes.c_double)

    Author: Kevin Best,
    https://github.com/tkevinbest
    """
    input_header_split = input_header.split(",")

    arg_list = [get_ctype(token) for token in input_header_split]
    return arg_list


def get_ctype(token):
    """
    Converts a single token from a header file into a ctypes argument.

    Author: Kevin Best, 8/7/2023
    https://github.com/tkevinbest
    """
    if "*" in token:
        out = ctypes.c_void_p
    elif "double" in token:
        out = ctypes.c_double
    elif "boolean_T" in token or "bool" in token:
        out = ctypes.c_bool
    else:
        raise Exception("Unknown type: " + token)

    return out

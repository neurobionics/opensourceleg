import ctypes


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

class Counter:
    """
    A simple counter class that increments a counter each time the increment_counter argument is set true. 
    To reset the counter, call update with increment_counter set to false. 

    Author: Kevin Best, 9/25/2024
    https://github.com/tkevinbest
    """
    def __init__(self):
        self._count: int = 0

    def update(self, increment_counter:bool):
        if increment_counter:
            self._count += 1
        else:
            self._count = 0

    @property
    def current_count(self):
        return self._count

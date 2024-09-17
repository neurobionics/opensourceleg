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


def to_twos_compliment(value: int, bit_length: int) -> int:
    """Converts a signed integer to 2's compliment for of a defined number of bits
    as an unsigned integer

    Args:
        value (int): Signed integer to convert
        bits (int): Number of bits of 2's compliment representation

    Returns:
        int: Unsigned integer 2's compliment

    Author: Axel Sjögren Holtz (axel.sjogren.holtz@vgregion.se)
    """
    assert value >= -(
        2 ** (bit_length - 1)
    ), f"Value {value} is too small for {bit_length} bits"
    assert value < 2 ** (
        bit_length - 1
    ), f"Value {value} is too large for {bit_length} bits"
    if value >= 0:
        return value
    return value + 2**bit_length


def from_twos_compliment(value: int, bit_length: int) -> int:
    """Converts a 2's compliment integer to a signed integer

    Args:
        value (int): 2's compliment integer
        bit_length (int): Number of bits of 2's compliment representation

    Returns:
        int: Signed integer

    Author: Axel Sjögren Holtz (axel.sjogren.holtz@vgregion.se)
    """
    assert type(value) == int
    assert value >= 0
    assert type(bit_length) == int
    assert bit_length >= 0
    assert value.bit_length() <= bit_length
    if value >= 2 ** (bit_length - 1):
        return int(value - (2**bit_length))
    else:
        return int(value)
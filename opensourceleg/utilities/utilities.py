def to_twos_complement(value: int, bit_length: int) -> int:
    """Converts a signed integer to 2's complement for a defined number of bits
    as an unsigned integer

    Args:
        value (int): Signed integer to convert
        bits (int): Number of bits of 2's compliment representation

    Returns:
        int: Unsigned integer 2's compliment

    Author: Axel Sjögren Holtz (axel.sjogren.holtz@vgregion.se)

    Raises:
        ValueError: If value is too small or too large for the given bit length
    """
    min_value = -(2 ** (bit_length - 1))
    max_value = 2 ** (bit_length - 1) - 1

    if value < min_value:
        raise ValueError(f"Value {value} is too small for {bit_length} bits")
    if value > max_value:
        raise ValueError(f"Value {value} is too large for {bit_length} bits")

    if value >= 0:
        return value

    return int(value + 2**bit_length)


def from_twos_compliment(value: int, bit_length: int) -> int:
    """Converts a 2's compliment integer to a signed integer

    Args:
        value (int): 2's compliment integer
        bit_length (int): Number of bits of 2's compliment representation

    Returns:
        int: Signed integer

    Author: Axel Sjögren Holtz (axel.sjogren.holtz@vgregion.se)

    Raises:
        TypeError: If value or bit_length is not an integer
        ValueError: If value is negative, bit_length is negative, or value exceeds bit_length
    """
    if not isinstance(value, int):
        raise TypeError("value must be an integer")
    if value < 0:
        raise ValueError("value must be non-negative")
    if not isinstance(bit_length, int):
        raise TypeError("bit_length must be an integer")
    if bit_length < 0:
        raise ValueError("bit_length must be non-negative")
    if value.bit_length() > bit_length:
        raise ValueError(f"value ({value}) exceeds the specified bit_length ({bit_length})")

    if value >= 2 ** (bit_length - 1):
        return int(value - (2**bit_length))
    else:
        return int(value)

import numpy as np
import pytest


from opensourceleg.utilities.utilities import *


def test_get_ctype_args():
    test = get_ctype_args("const struct *IMU, double position, bool joint")
    assert type(test) == list
    assert test == [ctypes.c_void_p, ctypes.c_double, ctypes.c_bool]
    assert len(test) >= 3

    with pytest.raises(TypeError):
        get_ctype_args()


def test_get_ctype():
    assert get_ctype("const *test") == ctypes.c_void_p

    assert get_ctype("double test") == ctypes.c_double

    assert get_ctype("boolean_T test") == ctypes.c_bool

    assert get_ctype("bool test") == ctypes.c_bool

    #what happens when pointer to double?
    assert get_ctype("double *ptr") == ctypes.c_void_p

    #what happens when name has "double" in it
    assert get_ctype("bool double_arm") == ctypes.c_double

    with pytest.raises(Exception):
       get_ctype(" ")

    with pytest.raises(TypeError):
        get_ctype()   
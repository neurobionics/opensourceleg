import pytest

from opensourceleg import control

zero_gains = control.Gains()
nonzero_gains = control.Gains()
nonzero_gains.kp = 1
nonzero_gains.ki = 2
nonzero_gains.kd = 3
nonzero_gains.K = 4
nonzero_gains.B = 5
negative_gains = control.Gains()
negative_gains.kp = -1
negative_gains.ki = -2
negative_gains.kd = -3
negative_gains.K = -4
negative_gains.B = -5


def test_gains():
    assert repr(zero_gains) == "kp=0, ki=0, kd=0, K=0, B=0, ff=0"
    assert repr(nonzero_gains) == "kp=1, ki=2, kd=3, K=4, B=5, ff=0"
    assert repr(negative_gains) == "kp=-1, ki=-2, kd=-3, K=-4, B=-5, ff=0"

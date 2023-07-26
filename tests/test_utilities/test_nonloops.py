import pytest

from opensourceleg.utilities import EdgeDetector, SaturatingRamp


def test_edge_detector_init():
    
    edi = EdgeDetector(bool_in=False)
    assert edi.cur_state == False    
    assert edi.rising_edge == False
    assert edi.falling_edge == False


def test_edge_detector_update():
    
    edu = EdgeDetector(bool_in=False)
    edu.update(bool_in=True)
    assert edu.rising_edge == True
    assert edu.falling_edge == False
    assert edu.cur_state == True
    edu2 = EdgeDetector(bool_in=True)
    edu2.update(bool_in=False)
    assert edu2.rising_edge == False
    assert edu2.falling_edge == True
    assert edu2.cur_state == False


def test_saturating_ramp_init():
    
    sri = SaturatingRamp(loop_frequency=100, ramp_time=1.0)
    assert sri.delta_per_update == 1.0 / 100
    assert sri.value == 0.0


def test_saturating_ramp_update():
    
    sru = SaturatingRamp(loop_frequency=100, ramp_time=1.0)
    sru.update(enable_ramp=True)
    assert sru.value == 0.01
    sru.update(enable_ramp=False)
    assert sru.value == 0.0




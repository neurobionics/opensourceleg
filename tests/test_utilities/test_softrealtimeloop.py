import time

import pytest

from opensourceleg.utilities.softrealtimeloop import LoopKiller, SoftRealtimeLoop


def test_loopkiller_init():
    """
    Tests the LoopKiller constructor\n
    Initializes a default LoopKiller object and asserts the attribute values
    are set properly. Then initializes a LoopKiller object with a fade_time
    and asserts the attribute values are set properly.
    """

    lk = LoopKiller()
    assert lk._fade_time == 0.0
    assert lk._soft_kill_time == 0.0
    lk1 = LoopKiller(fade_time=1.0)
    assert lk1._fade_time == 1.0
    assert lk1._soft_kill_time == 0.0
    assert lk1._kill_now is False
    assert lk1._kill_soon is False


def test_loopkiller_handle_signal():
    """
    Tests the LoopKiller handle_signal method\n
    Initializes a default LoopKiller object and asserts the kill_now attribute
    is set to false. Then calls the handle_signal method with arguments set to
    none and asserts the kill_now attribute is set to true.
    """

    lkhs = LoopKiller()
    assert lkhs.kill_now is False
    lkhs.handle_signal(signum=None, frame=None)
    assert lkhs.kill_now is True


@pytest.fixture
def patch_time_time2(monkeypatch):
    """
    Fixture to patch the time.monotonic method\n
    Patches the time.time method to return a list of values one at a time.
    """

    values = [0, 1, 2, 3, 4, 5]
    monkeypatch.setattr(time, "monotonic", lambda: values.pop(0))


def test_loopkiller_get_fade(patch_time_time2):
    """
    Tests the LoopKiller get_fade method\n
    Initializes a LoopKiller object with a fade_time and asserts the get_fade
    method returns the correct value. Then sets the soft_kill_time attribute
    and asserts the get_fade method returns the correct value. Then calls the
    get_fade method again and asserts the get_fade method returns 0.0.
    """

    lkgf = LoopKiller(fade_time=1.0)
    assert lkgf.get_fade() == 1.0
    lkgf._kill_soon = True
    lkgf._soft_kill_time = 0.0
    assert lkgf.get_fade() == 1.0
    assert lkgf.get_fade() == 0.0


def test_loopkiller_kill_now_prop(patch_time_time2):
    lkknp = LoopKiller(fade_time=1.0)
    lkknp._kill_soon = True
    lkknp._soft_kill_time = 0.0
    assert lkknp.kill_now is False
    lkknp._soft_kill_time = 0.5
    assert lkknp.kill_now is False
    assert lkknp.kill_now is True
    assert lkknp.kill_now is True


def test_loopkiller_kill_now_setter(patch_time_time2):
    lkkns = LoopKiller(fade_time=1.0)
    lkkns._kill_now = True
    lkkns._kill_soon = True
    lkkns._soft_kill_time = 0.0
    lkkns.kill_now = False
    assert lkkns._kill_now is False
    assert lkkns._kill_soon is False
    assert lkkns._soft_kill_time == 0.0
    lkkns.kill_now = True
    assert lkkns._kill_soon is True
    assert lkkns._soft_kill_time == 0.0
    assert lkkns._kill_now is False
    lkkns.kill_now = True
    assert lkkns._kill_now is True
    lkkns.kill_now = False
    lkkns._fade_time = 0.0
    lkkns.kill_now = True
    assert lkkns._kill_now is True


def test_softrealtimeloop_init(patch_time_time2):
    srtl = SoftRealtimeLoop()
    assert srtl.loop_deadline == srtl.dt
    assert srtl.loop_start_time == 0
    assert isinstance(srtl.killer, LoopKiller)
    assert srtl.sum_err == 0.0
    assert srtl.sum_var == 0.0
    assert srtl.sleep_t_agg == 0.0
    assert srtl.n == 0
    assert srtl.report is True
    assert srtl._maintain_original_phase is False


@pytest.fixture
def patch_time_time3(monkeypatch):
    """
    Fixture to patch the time.time method\n
    Patches the time.time method to return a list of values one at a time.
    """

    values = [0, 0, 1, 2, 3, 4, 5]
    monkeypatch.setattr(time, "monotonic", lambda: values.pop(0))


def test_softrealtimeloop_del():
    srtld = SoftRealtimeLoop(report=True)
    srtld.n = 2
    del srtld


def test_softrealtimeloop_iter(patch_time_time2):
    srtli = SoftRealtimeLoop()
    iter_srtli = iter(srtli)
    assert iter_srtli.loop_start_time == 1.00
    assert iter_srtli.loop_deadline == 1.001


def test_softrealtimeloop_fade_prop(patch_time_time3):
    srtlf = SoftRealtimeLoop(fade=1.0)
    assert srtlf.fade == 1.0
    srtlf.killer._kill_soon = True
    srtlf.killer._soft_kill_time = 0.0
    assert srtlf.fade == 1.0
    assert srtlf.fade == 0.0


def test_softrealtimeloop_stop(patch_time_time2):
    srtls = SoftRealtimeLoop()
    srtls.killer._kill_soon = True
    assert srtls.killer._kill_now is False
    srtls.stop()
    assert srtls.killer.kill_now is True


def test_softrealtimeloop_current_time(patch_time_time2):
    srtlt = SoftRealtimeLoop()
    assert srtlt.loop_start_time == 0.0
    assert srtlt.current_time == 1.0


def test_softrealtimeloop_time_since_start(patch_time_time2):
    srtlt = SoftRealtimeLoop()
    assert srtlt.loop_start_time == 0.0
    assert srtlt.time_since_start == 1.0


def test_softrealtimeloop_reset(patch_time_time2):
    srtl = SoftRealtimeLoop()
    assert srtl.loop_start_time == 0.0
    srtl.reset()
    assert srtl.loop_start_time == 1.0


class DemoClass:
    def __init__(self) -> None:
        self.x = 5

    def update(self) -> int:
        self.x -= 1
        return self.x


def test_softrealtimeloop_with_custom_dt():
    """
    Tests the SoftRealtimeLoop with a custom time step (dt).
    Ensures the loop respects the custom dt value.
    """
    custom_dt = 0.005
    srtl = SoftRealtimeLoop(dt=custom_dt, maintain_original_phase=True)
    assert srtl.dt == custom_dt
    assert srtl.loop_deadline == pytest.approx(srtl.loop_start_time + custom_dt, rel=1e-5)
    demo = DemoClass()
    srtl.run(demo.update)
    assert srtl.loop_deadline == pytest.approx(srtl.loop_start_time + custom_dt * srtl.n, rel=1e-5)


def test_softrealtimeloop_n():
    srtl = SoftRealtimeLoop()
    demo = DemoClass()
    expected_n = demo.x
    srtl.run(demo.update)
    assert expected_n == srtl.n

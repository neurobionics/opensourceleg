import time

from opensourceleg.utilities.profile import Profiler


def test_tic_toc():
    profiler = Profiler("tic_toc_test")
    profiler.tic()
    time.sleep(0.5)
    t = profiler.toc()
    assert t > 0.4 and t < 1.0  # More lenient timing for CI environments
    profiler.tic()
    time.sleep(0.5)
    t = profiler.toc()
    assert t > 0.4 and t < 1.0  # More lenient timing for CI environments
    assert profiler.N == 2
    assert profiler.agg > 0.8 and profiler.agg < 2.0  # More lenient aggregate timing
    assert profiler.aggvar > 0.4 and profiler.aggvar < 1.0  # More lenient variance


def test_no_runs():
    profiler = Profiler("test_none")
    assert profiler.N == 0
    assert profiler.agg == 0.0
    assert profiler.aggvar == 0.0


def test_all_tocs():
    profiler = Profiler("test_all_tocs")
    for _ in range(5):
        profiler.toc()
    assert profiler.N == 0
    assert profiler.agg == 0.0
    assert profiler.aggvar == 0.0


def test_decorator():
    profiler = Profiler("decorator")

    @profiler.decorate
    def my_decorated_function():
        time.sleep(0.0001)

    for _ in range(1000):
        my_decorated_function()
    assert profiler.N == 1000
    assert profiler.agg > 0.1


def test_lambda():
    profiler = Profiler("lambda")
    for _ in range(1000):
        profiler.profile(lambda: time.sleep(0.0001))
    assert profiler.N == 1000
    assert profiler.agg > 0.1


def test_toc_without_tic():
    profiler = Profiler("toc_without_tic")
    t = profiler.toc()
    assert t == 0.0
    assert profiler.N == 0
    assert profiler.agg == 0.0
    assert profiler.aggvar == 0.0


def test_multiple_tic_toc():
    profiler = Profiler("multiple_tic_toc")
    for _ in range(10):
        profiler.tic()
        time.sleep(0.1)
        t = profiler.toc()
        assert t > 0.05 and t < 0.5  # More lenient timing for CI environments
    assert profiler.N == 10
    assert profiler.agg > 0.5 and profiler.agg < 5.0  # More lenient aggregate timing
    assert profiler.aggvar > 0.05 and profiler.aggvar < 0.5  # More lenient variance


def test_context():
    p = Profiler("context")
    with p:
        time.sleep(0.1)

    assert p.N == 1
    assert p.agg > 0.05 and p.agg < 0.5  # More lenient timing for CI environments

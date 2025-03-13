import time

import pytest

from opensourceleg.timing.profile import Profiler


# def test_no_runs():
#     Profiler("test_none")

# def test_all_tocs():
#     Profiler("test_all_tocs").toc()
#     Profiler("test_all_tocs").toc()
#     Profiler("test_all_tocs").toc()
#     Profiler("test_all_tocs").toc()
#     Profiler("test_all_tocs").toc()

# def test_decorator():
#     @Profiler("decorator").decorate
#     def my_decorated_function():
#         time.sleep(0.0001)
#     for i in range(1000):
#         my_decorated_function()

# def test_lambda():
#     for i in range(1000):
#         Profiler("lambda").profile(lambda : time.sleep(0.0001))

def test_tic_toc():
    profiler = Profiler("tic_toc_test")
    profiler.tic()
    time.sleep(0.5)
    t = profiler.toc()
    assert t > 0.45 and t < 0.55
    profiler.tic()
    time.sleep(0.5)
    t = profiler.toc()
    assert t > 0.45 and t < 0.55
    assert profiler.N == 2
    assert profiler.agg > 0.9 and profiler.agg < 1.1
    assert profiler.aggvar > 0.45 and profiler.aggvar < 0.55


# test_tic_toc()

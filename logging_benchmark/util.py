import os
import shutil
import time

KB = 1000
MB = 1000000


def time_taken(func, *args, **kwargs):
    start = time.time()
    func(*args, **kwargs)
    delta = time.time() - start
    print(f"{func.__name__} took {delta:.6} seconds to run")
    clean_log_files()


def clean_log_files():
    for fn in os.listdir("."):
        if fn.endswith(".log"):
            try:
                os.remove(fn)
            except Exception:
                print("Couldn't delete a log file")

    if os.path.exists("logs") and os.path.isdir("logs"):
        try:
            shutil.rmtree("logs")
        except Exception:
            print("Couldn't delete logs dir")

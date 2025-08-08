from observable import PyProfiler

prof = PyProfiler()
def func1():
    for i in range(0, 1000):
        pass

def func2():
    for i in range(0, 2000):
        pass

def func3():
    for i in range(0, 4000):
        pass

def combine():
    for i in range(0, 1000):
        func1()
        func2()
        func3()

@PyProfiler
def tCombine(var):
    combine()
    
prof.tic()
tCombine(5)
prof.toc()

f = lambda : tCombine(5)
print(prof.profile(f))

with prof as p:
    tCombine(5)
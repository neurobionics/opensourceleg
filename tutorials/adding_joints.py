from opensourceleg.robots.robots import OpenSourceLeg

if __name__ == "__main__":
    osl = OpenSourceLeg(frequency=200, file_name="getting_started.log")
    osl.add_joint(name="knee", gear_ratio=41.99, has_loadcell=False)
    osl.add_joint(name="ankle", gear_ratio=41.99, has_loadcell=False)

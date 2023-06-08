import opensourceleg.constants as constants
from opensourceleg.osl import OpenSourceLeg

osl = OpenSourceLeg(frequency=200, file_name="getting_started.log")
osl.add_joint(name="knee", gear_ratio=41.99, has_loadcell=False)
osl.add_joint(name="ankle", gear_ratio=41.99, has_loadcell=False)

osl.add_loadcell(dephy_mode=False, loadcell_matrix=constants.LOADCELL_MATRIX)

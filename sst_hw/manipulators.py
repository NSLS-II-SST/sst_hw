from sst_base.manipulator import Manipulator1AxBase
from sst_base.motors import PrettyMotorFMBO
from ophyd import Component as Cpt


class MultiMesh(Manipulator1AxBase):
    x = Cpt(PrettyMotorFMBO, "i0upmmesh}Mtr", name="Multimesh")


multimesh = MultiMesh(None, "XF:07ID1-BI{UCAL-Ax:", name="i0upmultimesh")

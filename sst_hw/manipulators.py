from sst_base.manipulator import Manipulator1AxBase
from sst_base.motors import PrettyMotorFMBO
from ophyd import Component as Cpt


class MultiMesh(Manipulator1AxBase):
    x = Cpt(PrettyMotorFMBO, "Mmesh}Mtr", name="Multimesh")


multimesh = MultiMesh(None, "XF:07ID1-BI{I0Up-Ax:", name="i0upmultimesh")

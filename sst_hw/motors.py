from sst_base.motors import PrettyMotorFMBO, PrettyMotor, PrettyMotorFMBODeadbandFlyer


Exit_Slit = PrettyMotorFMBO(
    "XF:07ID2-BI{Slt:11-Ax:YGap}Mtr",
    name="Exit Slit of Mono Vertical Gap",
    kind="hinted",
)
grating = PrettyMotorFMBODeadbandFlyer(
    "XF:07ID1-OP{Mono:PGM1-Ax:GrtP}Mtr", name="Mono Grating", kind="hinted"
)
mirror2 = PrettyMotorFMBODeadbandFlyer(
    "XF:07ID1-OP{Mono:PGM1-Ax:MirP}Mtr", name="Mono Mirror", kind="hinted"
)
gratingx = PrettyMotorFMBODeadbandFlyer(
    "XF:07ID1-OP{Mono:PGM1-Ax:GrtX}Mtr", name="Mono Grating", kind="hinted"
)
mirror2x = PrettyMotorFMBODeadbandFlyer(
    "XF:07ID1-OP{Mono:PGM1-Ax:MirX}Mtr", name="Mono Mirror", kind="hinted"
)

i0upAu = PrettyMotor("XF:07ID1-BI{I0Up-Ax:Upper}Mtr", name="i0upAu", kind="hinted")

from ophyd import EpicsSignalRO, EpicsSignal

ring_current = EpicsSignalRO("SR:OPS-BI{DCCT:1}I:Real-I", name="NSLS-II Ring Current", kind="normal")
sst_control = EpicsSignalRO("XF:07ID1-CT{Bl-Ctrl}Endstn-Sel", name="SST endstation in Control", kind="normal",string=True)

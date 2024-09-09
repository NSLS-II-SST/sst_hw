from ophyd import EpicsSignal, PVPositionerPC, Signal, Component
from ophyd.status import SubscriptionStatus
from nbs_bl.printing import run_report


run_report(__file__)


Shutter_enable = EpicsSignal(
    "XF:07IDB-CT{DIODE-MTO:1}OutMaskBit:2-Sel",
    name="RSoXS Shutter Toggle Enable",
    kind="normal",
)
Shutter_SAXS_count = EpicsSignal(
    "XF:07IDB-CT{DIODE-Local:2}InCounter01:Count-I",
    name="RSoXS SAXS Shutter Counter",
    kind="normal",
)
Shutter_WAXS_count = EpicsSignal(
    "XF:07IDB-CT{DIODE-Local:2}InCounter00:Count-I",
    name="RSoXS WAXS Shutter Counter",
    kind="normal",
)
Shutter_enable1 = EpicsSignal(
    "XF:07IDB-CT{DIODE-MTO:1}InMaskBit:1-Sel",
    name="RSoXS Shutter Toggle Enable In",
    kind="normal",
)
Shutter_enable2 = EpicsSignal(
    "XF:07IDB-CT{DIODE-MTO:1}InMaskBit:2-Sel",
    name="RSoXS Shutter Toggle Enable In2",
    kind="normal",
    put_complete=False,
    auto_monitor=False,
)
Shutter_enable3 = EpicsSignal(
    "XF:07IDB-CT{DIODE-MTO:1}InMaskBit:3-Sel",
    name="RSoXS Shutter Toggle Enable In3",
    kind="normal",
)
Shutter_delay = EpicsSignal(
    "XF:07IDB-CT{DIODE-MTO:1}OutDelaySet:2-SP",
    name="RSoXS Shutter Delay (ms)",
    kind="normal",
)
Shutter_open_time = EpicsSignal(
    "XF:07IDB-CT{DIODE-MTO:1}OutWidthSet:2-SP",
    name="RSoXS Shutter Opening Time (ms)",
    kind="normal",
)
Shutter_trigger = EpicsSignal(
    "XF:07IDB-CT{DIODE-MTO:1}Trigger:PV-Cmd",
    name="RSoXS Shutter Trigger",
    kind="normal",
)
Light_control = EpicsSignal(
    "XF:07IDB-CT{DIODE-Local:1}OutPt05:Data-Sel",
    name="RSoXS Light Toggle",
    kind="normal",
)
MC21_disable = EpicsSignal(
    "XF:07IDB-CT{DIODE-Local:1}OutPt06:Data-Sel",
    name="MC21_disable",
    kind="normal",
)
MC20_disable = EpicsSignal(
    "XF:07IDB-CT{DIODE-Local:1}OutPt07:Data-Sel",
    name="MC20_disable",
    kind="normal",
)
MC19_disable = EpicsSignal(
    "XF:07IDB-CT{DIODE-Local:1}OutPt08:Data-Sel",
    name="MC19_disable",
    kind="normal",
)

class ShutterSet(PVPositionerPC):
    readback = Component(EpicsSignal,'-RB')
    setpoint = Component(EpicsSignal,'-SP')

    def set(self, value,*args,**kwargs):
        if value is None:
            saw_rise = False

            def watcher(*,old_value,value,**kwargs):
                nonlocal saw_rise
                if value == 1:
                    saw_rise = True
                    return False
                if value == 0 and saw_rise:
                    return True
            return SubscriptionStatus(self.readback, watcher)
        else:
            return super().set(value, *args, **kwargs)


class ShutterWait(EpicsSignal):
    def set(self, value, *, just_wait=False, **kwargs):
            """
            Set the value of the Signal, or just wait for the object to change to a value, either way returning a Status object

            Parameters
            ----------
            value : either a set value of a value to wait for
            just_wait : boolean whether to not set anything but just wait for the value to change to this value
            
            Returns
            -------
            Status

            """
            if(just_wait):
                wait_value = value
                def watcher(*,old_value,value,**kwargs):
                    if value == wait_value:
                        return True
                    else:
                        return False
                return SubscriptionStatus(self, watcher)
            else:
                return super().set(value, **kwargs)

Shutter_control = ShutterWait(
    "XF:07IDB-CT{DIODE-Local:1}OutPt01:Data-Sel",
    name="RSoXS Shutter Toggle",
    kind="normal",
)


shutter_open_set = ShutterSet('XF:07IDB-CT{DIODE-MTO:1}Output:2',name = "Shutter Open with Watcher")

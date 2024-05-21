from ophyd import (
    PVPositioner,
    EpicsSignalRO,
    PseudoPositioner,
    EpicsMotor,
    EpicsSignal,
    PVPositionerPC,
    SoftPositioner,
    Signal,
)
from ophyd import Component as Cpt, Device
import bluesky.plan_stubs as bps
from ophyd.pseudopos import pseudo_position_argument, real_position_argument
import pathlib
import numpy as np
import xarray as xr
from sst_funcs.gGrEqns import energy as calc_energy
from sst_funcs.printing import boxed_text, colored
from sst_base.motors import PrettyMotorFMBO, FlyerMixin
from sst_base.positioners import DeadbandEpicsMotor, DeadbandMixin, PseudoSingle
from sst_base.mirrors import FMBHexapodMirrorAxisStandAlonePitch
from sst_hw.shutters import psh4
from sst_hw.motors import grating, mirror2
from sst_hw.mirrors import mir3

import time
from ophyd.status import DeviceStatus, SubscriptionStatus, UnknownStatusFailure
import threading

from queue import Queue, Empty

##############################################################################################


class UndulatorMotor(FlyerMixin,DeadbandEpicsMotor):
    user_setpoint = Cpt(EpicsSignal, "-SP", limits=True)
    # done = Cpt(EpicsSignalRO, ".MOVN")
    # done_value = 0


class EpuMode(PVPositionerPC):
    setpoint = Cpt(EpicsSignal, "-SP", kind="config")
    readback = Cpt(EpicsSignal, "-RB", kind="normal")


# epu_mode = EpicsSignal(
#    "SR:C07-ID:G1A{SST1:1-Ax:Phase}Phs:Mode-SP", name="EPU 60 Mode", kind="normal"
# )
class FMB_Mono_Grating_Type(PVPositioner):
    setpoint = Cpt(EpicsSignal, "_TYPE_SP", string=True, kind="config")
    readback = Cpt(EpicsSignal, "_TYPE_MON", string=True, kind="config")
    actuate = Cpt(EpicsSignal, "_DCPL_CALC.PROC", kind="config")
    enable = Cpt(EpicsSignal, "_ENA_CMD.PROC", kind="config")
    kill = Cpt(EpicsSignal, "_KILL_CMD.PROC", kind="config")
    home = Cpt(EpicsSignal, "_HOME_CMD.PROC", kind="config")
    clear_encoder_loss = Cpt(EpicsSignal, "_ENC_LSS_CLR_CMD.PROC", kind="config")
    done = Cpt(EpicsSignal, "_AXIS_STS", kind="config")


class Monochromator(FlyerMixin,DeadbandMixin, PVPositioner):
    setpoint = Cpt(EpicsSignal, ":ENERGY_SP", kind="config")
    readback = Cpt(EpicsSignalRO, ":ENERGY_MON", kind="config")
    en_mon = Cpt(EpicsSignalRO, ":READBACK2.A", name="Energy", kind="hinted")

    grating = Cpt(PrettyMotorFMBO, "GrtP}Mtr", name="Mono Grating", kind="config")
    mirror2 = Cpt(PrettyMotorFMBO, "MirP}Mtr", name="Mono Mirror", kind="config")
    cff = Cpt(EpicsSignal, ":CFF_SP", name="Mono CFF", kind="config", auto_monitor=True)
    vls = Cpt(
        EpicsSignal, ":VLS_B2.A", name="Mono VLS", kind="config", auto_monitor=True
    )
    gratingx = Cpt(FMB_Mono_Grating_Type, "GrtX}Mtr", kind="config")
    mirror2x = Cpt(FMB_Mono_Grating_Type, "MirX}Mtr", kind="config")

    

    scanlock = Cpt(Signal, value=0, name="lock flag for during scans", kind="config")
    done = Cpt(EpicsSignalRO, ":ERDY_STS", kind="config")
    done_value = 1
    stop_signal = Cpt(EpicsSignal, ":ENERGY_ST_CMD", kind="config")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    
    def min_energy(self):
        #TODO: return the minimum energy for the current grating
        if '250' in self.gratingx.setpoint.get():
            return 70
        elif '1200' in self.gratingx.setpoint.get():
            return 100
        elif 'rsoxs' in self.gratingx.setpoint.get().lower():
            return 70
        return nan

    def max_energy(self):
         #TODO: return the maximum energy for the current grating
        if '250' in self.gratingx.setpoint.get():
            return 1200
        elif '1200' in self.gratingx.setpoint.get():
            return 2200
        elif 'rsoxs' in self.gratingx.setpoint.get().lower():
            return 1200
        return nan

    # def _setup_move(self, position):
    #     """Move and do not wait until motion is complete (asynchronous)"""
    #     self.log.debug("%s.setpoint = %s", self.name, position)
    #     # copy from pv_positioner, with wait changed to false
    #     # possible problem with IOC not returning from a set
    #     self.setpoint.put(position, wait=False)
    #     if self.actuate is not None:
    #         self.log.debug("%s.actuate = %s", self.name, self.actuate_value)
    #         self.actuate.put(self.actuate_value, wait=False)


# mono_en= Monochromator('XF:07ID1-OP{Mono:PGM1-Ax:', name='Monochromator Energy',kind='normal')

class FlyControl(Device):
    undulator_dance_enable = Cpt(EpicsSignal,'MACROControl-RB',write_pv='MACROControl-SP',name='Enable Undulator Dance')
    flymove_stop_ev = Cpt(EpicsSignal, "FlyMove-Mtr-SP", name="Energy scan stop energy", kind="config")
    flymove_speed_ev = Cpt(EpicsSignal, "FlyMove-Speed-SP", name="Energy scan speed", kind="config")
    flymove_start = Cpt(EpicsSignal, "FlyMove-Mtr-Go.PROC", name="Energy scan start command", kind="config")
    flymove_stop = Cpt(EpicsSignal,"FlyMove-Mtr.STOP", name="Energy scan stop command", kind="config")
    flymove_moving = Cpt(EpicsSignal, "FlyMove-Mtr.MOVN", name="en_flymove_moving", kind="config")
    scan_start_ev = Cpt(EpicsSignal, "EScanFirst-SP", name="en_scan_start", kind="config")
    scan_stop_ev = Cpt(EpicsSignal, "EScanLast-SP", name="en_scan_stop", kind="config")
    scan_speed_ev = Cpt(EpicsSignal, "EScan-Speed-SP", name="en_scan_speed", kind="config")
    scan_trigger_width = Cpt(EpicsSignal, "EScanTriggerWidth-RB", write_pv="EScanTriggerWidth-SP", name="trigger_width", kind="config")
    scan_trigger_n = Cpt(EpicsSignal, "EScanNTriggers-RB", write_pv="EScanNTriggers-SP", name="num_triggers", kind="config")
    scan_start_go = Cpt(EpicsSignal, "FlyScan-Mtr-Go.PROC", name="scan_start", kind="config")
    scanning = Cpt(EpicsSignal,"FlyScan-Mtr.MOVN", name="scan_moving", kind="config")

    LUT_en = Cpt(EpicsSignal, 'FlyLUT-Energy-RB',write_pv= "FlyLUT-Energy-SP", name="Flying undulator lookup table energy", kind="config")
    LUT_gap = Cpt(EpicsSignal, 'FlyLUT-Gap-RB',write_pv= "FlyLUT-Gap-SP", name="Flying undulator lookup table gap", kind="config")
    LUT_calc = Cpt(EpicsSignal, "CalculateSpline.PROC", name="Flying Undulator Calculate Spline", kind="config")
    LUT_ok = Cpt(EpicsSignal, "FlySplineOK-RB", name="Flying Undulator Spline OK", kind="config")
    flyer_pol = 0 # default polarization is 0


    def enable_undulator_sync(self):
        # Read status
        status = self.undulator_dance_enable.get()
        def check_value(*, old_value, value, **kwargs):
            if int(value) & 4:
                return True
            else:
                return False
        

        print('turning on undulator dance mode')
        st = SubscriptionStatus(self.undulator_dance_enable, check_value, run=True)
        self.undulator_dance_enable.set(1).wait()
        return st
    
    def disable_undulator_sync(self):
        # Read status
        status = self.undulator_dance_enable.get()
        def check_value(*, old_value, value, **kwargs):
            if int(value) & 10:
                return True
            else:
                return False
        

        print('turning off undulator dance mode')
        st = SubscriptionStatus(self.undulator_dance_enable, check_value, run=True)
        self.undulator_dance_enable.set(0).wait()
        return st
    
 #   def flymove(self, start, speed=5):
 #       self.enable_undulator_sync().wait()
 #       self.flymove_stop_ev.set(start).wait()
 #       self.flymove_speed_ev.set(speed).wait()
 #       def check_value(* old_value, value, **kwargs):
 #           return (old_value != 0 and value == 0)

 #       self.flymove_start.set(1).wait()
 #       move_st = SubscriptionStatus(self.flymove_moving, check_value, run=False)
 #       return move_st
            
    def scan_setup(self, start, stop, speed):
        self.scan_start_ev.set(start).wait()
        self.scan_stop_ev.set(stop).wait()
        self.scan_speed_ev.set(speed).wait()
        scan_range = stop - start
        self.scan_trigger_width.set(0.1).wait()
        trig_width = self.scan_trigger_width.get()
        # Not relevant yet, but required for scan
        ntrig = np.abs(scan_range//(2*trig_width))
        print(f'number of triggers : {ntrig}')
        self.scan_trigger_n.set(ntrig).wait()

    def scan_start(self):
        
        self.scan_start_go.set(1).wait()
        def check_value(*, old_value, value, **kwargs):
            if old_value != 0 and value == 0:
                return True
        fly_move_st = SubscriptionStatus(self.scanning, check_value, run=False)
        return fly_move_st
    
class EnPos(PseudoPositioner):
    """Energy pseudopositioner class.
    Parameters:
    -----------
    """

    # synthetic axis
    energy = Cpt(PseudoSingle, kind="hinted", limits=(71, 2250), name="Beamline Energy")
    polarization = Cpt(
        PseudoSingle, kind="normal", limits=(-1, 180), name="X-ray Polarization"
    )
    sample_polarization = Cpt(
        PseudoSingle, kind="config", name="Sample X-ray polarization"
    )
    # real motors

    monoen = Cpt(
        Monochromator, "XF:07ID1-OP{Mono:PGM1-Ax:", kind="config", name="Mono Energy"
    )
    epugap = Cpt(
        UndulatorMotor,
        "SR:C07-ID:G1A{SST1:1-Ax:Gap}-Mtr",
        kind="config",
        name="EPU Gap",
    )
    epuphase = Cpt(
        UndulatorMotor,
        "SR:C07-ID:G1A{SST1:1-Ax:Phase}-Mtr",
        kind="config",
        name="EPU Phase",
    )
    epumode = Cpt(
        EpuMode,
        "SR:C07-ID:G1A{SST1:1-Ax:Phase}Phs:Mode",
        name="EPU Mode",
        kind="config",
    )

    sim_epu_mode = Cpt(
        Signal, value=0, name="dont interact with the real EPU", kind="config"
    )
    scanlock = Cpt(
        Signal, value=0, name="Lock Harmonic, Pitch, Grating for scan", kind="config"
    )
    flycontrol = Cpt(FlyControl, "SR:C07-ID:G1A{SST1:1}", name="FlyscanControl", kind="config")
    harmonic = Cpt(Signal, value=1, name="EPU Harmonic", kind="config")
    offset_gap = Cpt(Signal, value=0, name="EPU Gap offset", kind="config")
    rotation_motor = None



    @pseudo_position_argument
    def forward(self, pseudo_pos):
        """Run a forward (pseudo -> real) calculation"""
        ret = self.RealPosition(
            epugap=self.gap(
                pseudo_pos.energy,
                pseudo_pos.polarization,
                self.scanlock.get(),
                self.sim_epu_mode.get(),
            ),
            monoen=pseudo_pos.energy,
            epuphase=abs(
                self.phase(
                    pseudo_pos.energy, pseudo_pos.polarization, self.sim_epu_mode.get()
                )
            ),
            epumode=self.mode(pseudo_pos.polarization, self.sim_epu_mode.get()),
        )
        return ret

    @real_position_argument
    def inverse(self, real_pos):
        """Run an inverse (real -> pseudo) calculation"""
        # print('in Inverse')
        ret = self.PseudoPosition(
            energy=real_pos.monoen,
            polarization=self.pol(real_pos.epuphase, real_pos.epumode),
            sample_polarization=self.sample_pol(
                self.pol(real_pos.epuphase, real_pos.epumode)
            ),
        )
        # print('Finished inverse')
        return ret

    def where_sp(self):
        return (
            "Beamline Energy Setpoint : {}"
            "\nMonochromator Readback : {}"
            "\nEPU Gap Setpoint : {}"
            "\nEPU Gap Readback : {}"
            "\nEPU Phase Setpoint : {}"
            "\nEPU Phase Readback : {}"
            "\nEPU Mode Setpoint : {}"
            "\nEPU Mode Readback : {}"
            "\nGrating Setpoint : {}"
            "\nGrating Readback : {}"
            "\nGratingx Setpoint : {}"
            "\nGratingx Readback : {}"
            "\nMirror2 Setpoint : {}"
            "\nMirror2 Readback : {}"
            "\nMirror2x Setpoint : {}"
            "\nMirror2x Readback : {}"
            "\nCFF : {}"
            "\nVLS : {}"
        ).format(
            colored("{:.2f}".format(self.monoen.setpoint.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.monoen.readback.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.epugap.user_setpoint.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.epugap.user_readback.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.epuphase.user_setpoint.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.epuphase.user_readback.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.epumode.setpoint.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.epumode.readback.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.monoen.grating.user_setpoint.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.monoen.grating.user_readback.get()).rstrip("0").rstrip("."), "yellow"),
            colored(self.monoen.gratingx.setpoint.get(), "yellow"),
            colored(self.monoen.gratingx.readback.get(), "yellow"),
            colored("{:.2f}".format(self.monoen.mirror2.user_setpoint.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.monoen.mirror2.user_readback.get()).rstrip("0").rstrip("."), "yellow"),
            colored(self.monoen.mirror2x.setpoint.get(), "yellow"),
            colored(self.monoen.mirror2x.readback.get(), "yellow"),
            colored("{:.2f}".format(self.monoen.cff.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.monoen.vls.get()).rstrip("0").rstrip("."), "yellow"),
        )

    def where(self):
        return (
            "Beamline Energy : {}\nPolarization : {}\nSample Polarization : {}"
        ).format(
            colored("{:.2f}".format(self.monoen.readback.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.polarization.readback.get()).rstrip("0").rstrip("."), "yellow"),
            colored("{:.2f}".format(self.sample_polarization.readback.get()).rstrip("0").rstrip("."), "yellow"),
        )

    def wh(self):
        boxed_text(self.name + " location", self.where_sp(), "green", shrink=True)


    def preflight(self, start, stop, speed, *args, locked=True, time_resolution=None):
        print('starting preflight')
        if len(args) > 0:
            if len(args) % 3 != 0:
                raise ValueError("args must be start2, stop2, speed2[, start3, stop3, speed3, ...] and must be a multiple of 3")
            else:
                self.flight_segments = ((args[3*n], args[3*n + 1], args[3*n + 2]) for n in range(len(args)//3))
        else:
            self.flight_segments = iter(())
        print('setting up scan parameters')
        

        if time_resolution is not None:
            self._time_resolution = time_resolution
        elif self._time_resolution is None:
            self._time_resolution = self._default_time_resolution

        #if locked:
        #    self.scanlock.set(True).wait()
        
        # self.energy.set(start).wait()
        
        print('setting up spline for undulator scanning')
        # update spline to current polarization setting
        ens,gaps = self.get_gap_en_spline()
        try:
            self.flycontrol.LUT_en.set(ens,timeout=1).wait()
        except UnknownStatusFailure:
            pass
        try:
            self.flycontrol.LUT_gap.set(gaps,timeout=1).wait()
        except UnknownStatusFailure:
            pass
        try:
            self.flycontrol.LUT_calc.set(1,timeout=1).wait()
        except UnknownStatusFailure:
            pass
        if not self.flycontrol.LUT_ok.get():
            raise ValueError('spline calculation not good')
        
        print('setting up scan parameters')
        # input the scan parameters
        self.flycontrol.scan_setup(start, stop, speed)
        # enable undulator control
        print('enabling scan control')
        self.flycontrol.enable_undulator_sync().wait()
        self._last_mono_value = start
        self._mono_stop = stop
        self._ready_to_fly = True

    def fly(self):
        """
        Should be called after all detectors start flying, so that we don't lose data
        """
        if not self._ready_to_fly:
            self._fly_move_st = DeviceStatus(device=self)
            self._fly_move_st.set_exception(RuntimeError)
        else:
            def check_value(*, old_value, value, **kwargs):
                if (old_value != 0 and value == 0): # was moving, but not moving anymore
                # this is triggering too early, before we are near the end energy... 
                # i think we need a test for if the energy is at the end
                    try:
                        print('got to stopping point')
                        start, stop, speed = next(self.flight_segments)
                        self.flycontrol.scan_setup(start, stop, speed).wait()
                        print(f'starting next step to {stop}eV at {speed}eV/sec')
                        self.flycontrol.scan_start()
                        return False
                    except StopIteration:
                        return True
                else:
                    return False

                
            print('beginning the undulator dance')
            # Need our own check_value that will keep flying until there are no more flight segments left
            self._fly_move_st = SubscriptionStatus(self.flycontrol.scanning, check_value, run=False)
            self.flycontrol.scan_start()
            self._flying = True
            self._ready_to_fly = False
        return self._fly_move_st

    def land(self):
        if self._fly_move_st.done:
            self._flying = False
            self._time_resolution = None
            self.scanlock.set(False).wait()
            self.flycontrol.disable_undulator_sync().wait()

    def kickoff(self):
        kickoff_st = DeviceStatus(device=self)
        if self._time_resolution is None:
            self._time_resolution = self._default_time_resolution

        self._flyer_queue = Queue()
        self._measuring = True
        self._flyer_buffer = []
        threading.Thread(target=self._aggregate, daemon=True).start()
        kickoff_st.set_finished()
        return kickoff_st

    def _aggregate(self):
        name = 'energy_readback'
        while self._measuring:
            rb = self.monoen.readback.read()
            t = time.time()
            value = rb[self.monoen.readback.name]['value']
            ts = rb[self.monoen.readback.name]['timestamp']
            self._flyer_buffer.append(value)
            event = dict()
            event['time'] = t
            event['data'] = dict()
            event['timestamps'] = dict()
            event['data'][name] = value
            event['timestamps'][name] = ts
            self._flyer_queue.put(event)
            #if abs(self._last_mono_value - value) > self._flyer_lag_ev:
            #    self._last_mono_value = value
            #    self.epugap.set(self.gap(value + self._flyer_gap_lead, self._flyer_pol, False))
            time.sleep(self._time_resolution)
        return

    def collect(self):
        events = []
        while True:
            try:
                e = self._flyer_queue.get_nowait()
                events.append(e)
            except Empty:
                break
        yield from events

    def complete(self):
        if self._measuring:
            self._measuring = False
        completion_status = DeviceStatus(self)
        completion_status.set_finished()
        self._time_resolution = None
        return completion_status

    def describe_collect(self):
        dd = dict({"energy_readback": {'source': self.monoen.readback.pvname, 'dtype': 'number', 'shape': []}})
        return {"energy_readback_monitor": dd}


    # end class methods, begin internal methods

    # begin LUT Functions

    def __init__(
        self,
        a,
        rotation_motor=None,
        configpath=pathlib.Path(__file__).parent.absolute() / "config",
        **kwargs,
    ):
        self.gap_fitnew = np.array([[-2.02817540e+03,  3.02264723e+02, -1.78252111e+00,
                                        7.43668353e-03, -1.91232012e-05,  2.51973358e-08,
                                        4.79962799e-12, -8.29186995e-14,  1.57617047e-16,
                                        -1.59186547e-19,  9.43016130e-23, -3.09532281e-26,
                                        4.36145287e-30],
                                    [ 4.03257973e-01, -1.16153798e-02,  1.42259540e-04,
                                        -9.21569724e-07,  3.64833617e-09, -9.41596905e-12,
                                        1.63464324e-14, -1.92640661e-17,  1.52209377e-20,
                                        -7.72874330e-24,  2.28187017e-27, -2.98088495e-31,
                                        0.00000000e+00],
                                    [ 4.56475603e-05, -4.07999403e-07,  1.18075497e-09,
                                        -2.87363757e-12,  3.75535610e-15,  3.29862492e-18,
                                        -1.94014184e-20,  2.74619195e-23, -1.83395988e-26,
                                        5.77828602e-30, -6.21442519e-34,  0.00000000e+00,
                                        0.00000000e+00],
                                    [-5.25493975e-08,  9.13848518e-11, -8.89125498e-14,
                                        -7.70071244e-17,  1.56845096e-19,  2.27044971e-22,
                                        -3.84069721e-25,  1.07113860e-28,  6.45500669e-32,
                                        -3.56486225e-35,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00],
                                    [ 1.69412943e-11, -1.72741103e-14,  2.32736978e-17,
                                        -1.27270356e-20, -2.28179895e-23,  1.64992858e-26,
                                        5.41608428e-30, -6.86000848e-33,  2.31195976e-36,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00],
                                    [-3.28740709e-15,  1.72993353e-18, -1.95111611e-21,
                                        2.04503884e-24,  1.86619961e-28, -8.41281283e-31,
                                        1.99741076e-34, -6.65135708e-38,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00],
                                    [ 4.12832071e-19, -1.08634915e-22,  9.38953584e-26,
                                        -9.25160150e-29,  2.47681044e-32,  1.24161680e-35,
                                        1.18873213e-39,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00],
                                    [-3.46595227e-23,  3.42794252e-27, -3.81112396e-30,
                                        1.81952044e-33, -8.72888305e-37, -1.72881705e-40,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00],
                                    [ 1.97641466e-27,  2.15621764e-32,  1.26835147e-34,
                                        -8.69314807e-39,  1.16321066e-41,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00],
                                    [-7.55639549e-32, -5.20717157e-36, -2.61944925e-39,
                                        -1.53939901e-43,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00],
                                    [ 1.84709041e-36,  1.46245833e-40,  2.35251768e-44,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00],
                                    [-2.59793922e-41, -1.36029553e-45,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00],
                                    [ 1.59420902e-46,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                                        0.00000000e+00]])
        

        # values for the minimum energy as a function of angle polynomial 10th deg
        # 80.934 ± 0.0698
        # -0.91614 ± 0.0446
        # 0.39635 ± 0.00925
        # -0.020478 ± 0.000881
        # 0.00069047 ± 4.54e-05
        # -1.5413e-05 ± 1.37e-06
        # 2.1448e-07 ± 2.49e-08
        # -1.788e-09 ± 2.68e-10
        # 8.162e-12 ± 1.57e-12
        # -1.5545e-14 ± 3.88e-15

        self.polphase = xr.load_dataarray(configpath / "polphase.nc")
        self.phasepol = xr.DataArray(
            data=self.polphase.pol,
            coords={"phase": self.polphase.values},
            dims={"phase"},
        )
        self.rotation_motor = rotation_motor
        super().__init__(a, **kwargs)
        self.epugap.tolerance.set(3).wait()
        self.epuphase.tolerance.set(10).wait()
        # self.mir3Pitch.tolerance.set(0.01)
        self.monoen.tolerance.set(0.01).wait()
        self._ready_to_fly = False
        self._fly_move_st = None
        self._default_time_resolution = 0.05
        self._flyer_lag_ev = 0.1
        self._flyer_gap_lead = 0.0
        self._time_resolution = self._default_time_resolution
        self._flying = False
    """
    def stage(self):
        if self.scanlock.get():
            self.epuphase.tolerance.set(10)
        super().stage()

    def unstage(self):
        self.epuphase.tolerance.set(0)
        super().unstage()
    """

    def gap(self, energy, pol, locked, sim=0):
        if sim:
            return (
                self.epugap.get()
            )  # never move the gap if we are in simulated gap mode
            # this might cause problems if someone else is moving the gap, we might move it back
            # but I think this is not a common reason for this mode

        self.harmonic.set(self.choose_harmonic(energy, pol, locked)).wait()
        energy = energy / self.harmonic.get()

        if (pol == -1) or (pol == -0.5):
            encalc = energy
            gap = 6202.6
            gap += 74.094 * encalc ** 1
            gap += 0.14654 * encalc ** 2
            gap += -0.001609 * encalc ** 3
            gap += 5.443e-06 * encalc ** 4
            gap += -1.0023e-08 * encalc ** 5
            gap += 1.1005e-11 * encalc ** 6
            gap += -7.1779e-15 * encalc ** 7
            gap += 2.5652e-18 * encalc ** 8
            gap += -3.86e-22 * encalc ** 9

            return max(14000.0, min(100000.0, gap)) + self.offset_gap.get()
        elif 0 <= pol <= 90:
            return (
                max(14000.0, min(100000.0, self.epu_gap(energy, pol)))
                + self.offset_gap.get()
            )
        elif 90 < pol <= 180:
            return (
                max(14000.0, min(100000.0, self.epu_gap(energy, 180.0 - pol)))
                + self.offset_gap.get()
            )
        else:
            return np.nan
    
    def epu_gap(self, en, pol ):
        """
        calculate the epu gap from the energy and polarization, using a 2D polynomial fit
        @param en: energy (valid between ~70 and 1300
        @param pol: polarization (valid between 0 and 90)
        @return: gap in microns
        """
        y = float(en)
        x = float(self.phase(en,pol))
        z = 0.0
        for i in np.arange(self.gap_fitnew.shape[0]):
            for j in np.arange(self.gap_fitnew.shape[1]):
                z += self.gap_fitnew[j, i] * (x ** j) * (y ** i)
        return z

    def phase(self, en, pol, sim=0):
        if sim:
            return (
                self.epuphase.get()
            )  # never move the gap if we are in simulated gap mode
            # this might cause problems if someone else is moving the gap, we might move it back
            # but I think this is not a common reason for this mode
        if pol == -1:
            return 15000
        elif pol == -0.5:
            return 15000
        elif 90 < pol <= 180:
            return -min(
                29500.0,
                max(0.0, float(self.polphase.interp(pol=180 - pol, method="cubic"))),
            )
        else:
            return min(
                29500.0, max(0.0, float(self.polphase.interp(pol=pol, method="cubic")))
            )

    def pol(self, phase, mode):
        if mode == 0:
            return -1
        elif mode == 1:
            return -0.5
        elif mode == 2:
            return float(self.phasepol.interp(phase=np.abs(phase), method="cubic"))
        elif mode == 3:
            return 180 - float(
                self.phasepol.interp(phase=np.abs(phase), method="cubic")
            )

    def mode(self, pol, sim=0):
        """
        @param pol:
        @return:
        """
        if sim:
            return (
                self.epumode.get()
            )  # never move the gap if we are in simulated gap mode
            # this might cause problems if someone else is moving the gap, we might move it back
            # but I think this is not a common reason for this mode
        if pol == -1:
            return 0
        elif pol == -0.5:
            return 1
        elif 90 < pol <= 180:
            return 3
        else:
            return 2

    def sample_pol(self, pol):
        th = self.rotation_motor.user_setpoint.get()
        return (
            np.arccos(np.cos(pol * np.pi / 180) * np.sin(th * np.pi / 180))
            * 180
            / np.pi
        )

    def choose_harmonic(self, energy, pol, locked):
        if locked:
            return self.harmonic.get()
        elif energy < 1200:
            return 1
        else:
            return 3
    def get_gap_en_spline(self):
        
        #TODO: add spline calculation for energy range (relevant for the grating)
        minen = self.monoen.min_energy()
        maxen = self.monoen.max_energy()
        ens = np.linspace(minen, maxen, 20)
        gaps = [self.gap(energy,self.polarization.setpoint.get(),True,False) for energy in ens]
        return ens, gaps

def base_set_polarization(pol, en):
    yield from bps.mv(en.polarization, pol)
    return 0


def base_grating_to_250(mono_en, en):
    type = mono_en.gratingx.readback.get()
    if "250l/mm" in type:
        print("the grating is already at 250 l/mm")
        return 0  # the grating is already here
    print("Moving the grating to 250 l/mm.  This will take a minute...")
    yield from psh4.close()
    yield from bps.abs_set(mono_en.gratingx, 2, wait=True)
    # yield from bps.sleep(60)
    # yield from bps.mv(mirror2.user_offset, 0.04) #0.0315)
    # yield from bps.mv(grating.user_offset, -0.0874)#-0.0959)
    # yield from bps.mv(en.m3offset, 7.90)
    yield from bps.mv(mono_en.cff, 1.385)
    yield from bps.mv(en, 270)
    yield from psh4.open()
    print("the grating is now at 250 l/mm signifigant higher order")
    return 1


def base_grating_to_1200(mono_en, en):
    type = mono_en.gratingx.readback.get()
    if "1200" in type:
        print("the grating is already at 1200 l/mm")
        return 0  # the grating is already here
    print("Moving the grating to 1200 l/mm.  This will take a minute...")
    yield from psh4.close()
    yield from bps.abs_set(mono_en.gratingx, 9, wait=True)
    # yield from bps.sleep(60)
    # yield from bps.mv(mirror2.user_offset, 0.2044) #0.1962) #0.2052) # 0.1745)  # 8.1264)
    # yield from bps.mv(grating.user_offset, 0.0769) #0.0687) # 0.0777) # 0.047)  # 7.2964)  # 7.2948)#7.2956
    yield from bps.mv(mono_en.cff, 1.7)
    # yield from bps.mv(en.m3offset, 7.791)
    yield from bps.mv(en, 270)
    yield from psh4.open()
    print("the grating is now at 1200 l/mm")
    return 1


def base_grating_to_rsoxs(mono_en, en):
    type = mono_en.gratingx.readback.get()
    if "RSoXS" in type:
        print("the grating is already at RSoXS")
        return 0  # the grating is already here
    print("Moving the grating to RSoXS 250 l/mm.  This will take a minute...")
    yield from psh4.close()
    yield from bps.abs_set(mono_en.gratingx, 10, wait=True)
    # yield from bps.sleep(60)
    # yield from bps.mv(mirror2.user_offset, 0.2044) #0.1962) #0.2052) # 0.1745)  # 8.1264)
    # yield from bps.mv(grating.user_offset, 0.0769) #0.0687) # 0.0777) # 0.047)  # 7.2964)  # 7.2948)#7.2956
    # yield from bps.mv(mono_en.cff, 1.7)
    # yield from bps.mv(en.m3offset, 7.87)
    yield from bps.mv(en, 270)
    yield from psh4.open()
    print("the grating is now at RSoXS 250 l/mm with low higher order")
    return 1


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
from ophyd import Component as Cpt
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
from ophyd.status import DeviceStatus, SubscriptionStatus
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

    Scan_Start_ev = Cpt(
        EpicsSignal, ":EVSTART_SP", name="MONO scan start energy", kind="config"
    )
    Scan_Stop_ev = Cpt(
        EpicsSignal, ":EVSTOP_SP", name="MONO scan stop energy", kind="config"
    )
    Scan_Speed_ev = Cpt(
        EpicsSignal, ":EVVELO_SP", name="MONO scan speed", kind="config"
    )
    Scan_Start = Cpt(
        EpicsSignal, ":START_CMD.PROC", name="MONO scan start command", kind="config"
    )
    Scan_Stop = Cpt(
        EpicsSignal,
        ":ENERGY_ST_CMD.PROC",
        name="MONO scan start command",
        kind="config",
    )

    scanlock = Cpt(Signal, value=0, name="lock flag for during scans", kind="config")
    done = Cpt(EpicsSignalRO, ":ERDY_STS", kind="config")
    done_value = 1
    stop_signal = Cpt(EpicsSignal, ":ENERGY_ST_CMD", kind="config")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def _setup_move(self, position):
        """Move and do not wait until motion is complete (asynchronous)"""
        self.log.debug("%s.setpoint = %s", self.name, position)
        # copy from pv_positioner, with wait changed to false
        # possible problem with IOC not returning from a set
        self.setpoint.put(position, wait=False)
        if self.actuate is not None:
            self.log.debug("%s.actuate = %s", self.name, self.actuate_value)
            self.actuate.put(self.actuate_value, wait=False)


# mono_en= Monochromator('XF:07ID1-OP{Mono:PGM1-Ax:', name='Monochromator Energy',kind='normal')


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
            colored(
                "{:.2f}".format(self.monoen.setpoint.get()).rstrip("0").rstrip("."),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.monoen.readback.get()).rstrip("0").rstrip("."),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.epugap.user_setpoint.get())
                .rstrip("0")
                .rstrip("."),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.epugap.user_readback.get())
                .rstrip("0")
                .rstrip("."),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.epuphase.user_setpoint.get())
                .rstrip("0")
                .rstrip("."),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.epuphase.user_readback.get())
                .rstrip("0")
                .rstrip("."),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.epumode.setpoint.get()).rstrip("0").rstrip("."),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.epumode.readback.get()).rstrip("0").rstrip("."),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.monoen.grating.user_setpoint.get())
                .rstrip("0")
                .rstrip("."),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.monoen.grating.user_readback.get())
                .rstrip("0")
                .rstrip("."),
                "yellow",
            ),
            colored(
                self.monoen.gratingx.setpoint.get(),
                "yellow",
            ),
            colored(
                self.monoen.gratingx.readback.get(),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.monoen.mirror2.user_setpoint.get())
                .rstrip("0")
                .rstrip("."),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.monoen.mirror2.user_readback.get())
                .rstrip("0")
                .rstrip("."),
                "yellow",
            ),
            colored(
                self.monoen.mirror2x.setpoint.get(),
                "yellow",
            ),
            colored(
                self.monoen.mirror2x.readback.get(),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.monoen.cff.get()).rstrip("0").rstrip("."), "yellow"
            ),
            colored(
                "{:.2f}".format(self.monoen.vls.get()).rstrip("0").rstrip("."), "yellow"
            ),
        )

    def where(self):
        return (
            "Beamline Energy : {}\nPolarization : {}\nSample Polarization : {}"
        ).format(
            colored(
                "{:.2f}".format(self.monoen.readback.get()).rstrip("0").rstrip("."),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.polarization.readback.get())
                .rstrip("0")
                .rstrip("."),
                "yellow",
            ),
            colored(
                "{:.2f}".format(self.sample_polarization.readback.get())
                .rstrip("0")
                .rstrip("."),
                "yellow",
            ),
        )

    def wh(self):
        boxed_text(self.name + " location", self.where_sp(), "green", shrink=True)


    def preflight(self, start, stop, speed, *args, locked=True, time_resolution=None):
        self.monoen.Scan_Start_ev.set(start)
        self.monoen.Scan_Stop_ev.set(stop)
        self.monoen.Scan_Speed_ev.set(speed)
        if len(args) > 0:
            if len(args) % 3 != 0:
                raise ValueError("args must be start2, stop2, speed2[, start3, stop3, speed3, ...] and must be a multiple of 3")
            else:
                self.flight_segments = ((args[3*n], args[3*n + 1], args[3*n + 2]) for n in range(len(args)//3))
        else:
            self.flight_segments = iter(())

        self._flyer_pol = self.polarization.setpoint.get()

        if time_resolution is not None:
            self._time_resolution = time_resolution
        elif self._time_resolution is None:
            self._time_resolution = self._default_time_resolution

        self.energy.set(start + 10).wait()
        if locked:
            self.scanlock.set(True).wait()
        self.energy.set(start).wait()
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
                if (old_value != 1 and value == 1):
                    try:
                        start, stop, speed = next(self.flight_segments)
                        self.monoen.Scan_Start_ev.set(start)
                        self.monoen.Scan_Stop_ev.set(stop)
                        self.monoen.Scan_Speed_ev.set(speed)
                        self.monoen.Scan_Start.set(1)
                        return False
                    except StopIteration:
                        return True
                else:
                    return False

            self._fly_move_st = SubscriptionStatus(self.monoen.done, check_value, run=False)
            self.monoen.Scan_Start.set(1)
            self._flying = True
            self._ready_to_fly = False
        return self._fly_move_st

    def land(self):
        if self._fly_move_st.done:
            self._flying = False
            self._time_resolution = None
            self.scanlock.set(False).wait()

    def kickoff(self):
        kickoff_st = DeviceStatus(device=self)
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
            if abs(self._last_mono_value - value) > self._flyer_lag_ev:
                self._last_mono_value = value
                self.epugap.set(self.gap(value + self._flyer_gap_lead, self._flyer_pol, False))
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
        self.gap_fit = np.array([
            [
                -6.70046527e03,
                -3.36183042e02,
                -4.41933673e01,
                -5.19219800e-01,
                1.71180769e-01,
                -8.28021054e-03,
                1.79256702e-04,
                -2.49895358e-07,
                -9.74541279e-08,
                2.66233798e-09,
                -2.87278811e-11,
                -3.04454699e-14,
                4.58365092e-15,
                -5.58084693e-17,
                3.08135286e-19,
                -6.82291347e-22,
            ],
            [
                4.80199793e02,
                1.10195258e01,
                4.66148873e-01,
                -1.13385642e-02,
                -2.43198107e-04,
                4.16472599e-05,
                -2.48801673e-06,
                8.97086113e-08,
                -1.83991797e-09,
                1.85767115e-11,
                -2.41274374e-15,
                -2.27183960e-15,
                2.74641088e-17,
                -1.46460912e-19,
                3.09241528e-22,
                0.00000000e00,
            ],
            [
                -4.55523642e00,
                -1.37162157e-01,
                -2.97592927e-03,
                8.76245460e-05,
                -2.49061355e-06,
                1.08030038e-07,
                -4.84688258e-09,
                1.05059261e-10,
                -1.12031164e-12,
                6.19812442e-15,
                -5.57132550e-18,
                -2.36878495e-19,
                1.76084014e-21,
                -3.41965754e-24,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                3.10151548e-02,
                8.80612685e-04,
                9.24963607e-06,
                -1.62116353e-07,
                5.98586736e-10,
                1.54757849e-10,
                -1.85887969e-12,
                -5.87043722e-15,
                -2.86698480e-16,
                8.04672740e-18,
                -6.77723042e-20,
                3.24136659e-22,
                -9.40850289e-25,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                -1.41190950e-04,
                -3.30852856e-06,
                -1.90177839e-08,
                1.69418361e-10,
                -7.17048807e-12,
                -1.33784835e-13,
                3.88750265e-15,
                7.31878454e-18,
                -1.50678642e-19,
                -8.75409579e-22,
                -4.72660966e-25,
                5.98919910e-26,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                4.29632028e-07,
                7.68576434e-09,
                2.98083215e-11,
                2.17961063e-13,
                1.06223588e-14,
                -1.44538057e-16,
                -1.62125840e-18,
                -1.67039183e-20,
                1.68574272e-22,
                6.86859844e-25,
                -8.03883330e-27,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                -8.63084774e-10,
                -1.09255846e-11,
                -4.14279152e-14,
                -6.09141408e-16,
                2.46418963e-18,
                1.62913643e-19,
                1.74418887e-22,
                2.07949104e-23,
                -1.38838552e-25,
                6.09913030e-28,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                1.08299019e-12,
                8.29159117e-15,
                4.21109303e-17,
                2.15003600e-19,
                -9.09803438e-21,
                1.94874857e-23,
                -1.29653759e-24,
                -7.24862901e-27,
                -1.35800936e-29,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                -6.83085605e-16,
                -7.12619059e-19,
                -1.61270699e-20,
                2.78077809e-22,
                -1.11719681e-24,
                -1.62100704e-26,
                1.21610413e-27,
                2.82639188e-30,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                -9.51594859e-20,
                -4.13379686e-21,
                -1.37699859e-23,
                -4.14241045e-26,
                5.61220485e-27,
                -3.92522578e-29,
                -3.87257024e-31,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                4.58825904e-22,
                2.25153178e-24,
                1.58384875e-26,
                -2.46101605e-28,
                -1.63603522e-30,
                1.98873341e-32,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                -1.58849385e-25,
                1.54061965e-27,
                -3.34742348e-30,
                1.46866501e-31,
                -1.67977111e-34,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                -2.21751502e-28,
                -2.30806958e-30,
                -1.37491092e-33,
                -2.16350259e-35,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                2.51776696e-31,
                1.03324923e-33,
                4.79185498e-37,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                -1.02722988e-34,
                -1.68057032e-37,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
            [
                1.57365083e-38,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
                0.00000000e00,
            ],
        ])

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
        #self.mir3Pitch.tolerance.set(0.01)
        self.monoen.tolerance.set(0.01).wait()
        self._ready_to_fly = False
        self._fly_move_st = None
        self._default_time_resolution = 0.05
        self._flyer_lag_ev = 0.1
        self._flyer_gap_lead = 0.0
        self._time_resolution = None
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
            encalc = energy - 110
            gap = 14400.0
            gap += 81.839 * encalc ** 1
            gap += -0.27442 * encalc ** 2
            gap += 0.0010336 * encalc ** 3
            gap += -2.8625e-06 * encalc ** 4
            gap += 5.3711e-09 * encalc ** 5
            gap += -6.5229e-12 * encalc ** 6
            gap += 4.8906e-15 * encalc ** 7
            gap += -2.0525e-18 * encalc ** 8
            gap += 3.6942e-22 * encalc ** 9
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

    def epu_gap(self, en, pol):
        """
        calculate the epu gap from the energy and polarization, using a 2D polynomial fit
        @param en: energy (valid between ~70 and 1300
        @param pol: polarization (valid between 0 and 90)
        @return: gap in microns
        """
        x = float(en)
        y = float(pol)
        z = 0.0
        for i in np.arange(self.gap_fit.shape[0]):
            for j in np.arange(self.gap_fit.shape[1]):
                z += self.gap_fit[j, i] * (x ** j) * (y ** i)
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


from ophyd import (
    PVPositioner,
    EpicsSignalRO,
    PseudoPositioner,
    EpicsMotor,
    EpicsSignal,
    PVPositionerPC,
    SoftPositioner,
    Signal)
from ophyd import Component as Cpt
import bluesky.plan_stubs as bps
from ophyd.pseudopos import pseudo_position_argument, real_position_argument
from ophyd.status import DeviceStatus, SubscriptionStatus
import threading
from queue import Queue, Empty
import pathlib
import numpy as np
import xarray as xr
import time
from sst_funcs.printing import boxed_text, colored
from sst_base.motors import PrettyMotorFMBO
from sst_base.positioners import DeadbandEpicsMotor, DeadbandMixin, PseudoSingle
from sst_base.mirrors import FMBHexapodMirrorAxisStandAlonePitch
from sst_hw.shutters import psh4
from sst_hw.motors import grating, mirror2
from sst_hw.mirrors import mir3

##############################################################################################


class UndulatorMotor(DeadbandEpicsMotor):
    user_setpoint = Cpt(EpicsSignal, "-SP", limits=True)
    #done = Cpt(EpicsSignalRO, ".MOVN")
    #done_value = 0


class EpuMode(PVPositionerPC):
    setpoint = Cpt(EpicsSignal, "-SP", kind="normal")
    readback = Cpt(EpicsSignal, "-RB", kind="normal")


#epu_mode = EpicsSignal(
#    "SR:C07-ID:G1A{SST1:1-Ax:Phase}Phs:Mode-SP", name="EPU 60 Mode", kind="normal"
#)
class FMB_Mono_Grating_Type(PVPositioner):
    setpoint = Cpt(EpicsSignal, '_TYPE_SP', string=True)
    readback = Cpt(EpicsSignal, '_TYPE_MON', string=True)
    actuate = Cpt(EpicsSignal, '_DCPL_CALC.PROC')
    enable = Cpt(EpicsSignal, '_ENA_CMD.PROC')
    kill = Cpt(EpicsSignal, '_KILL_CMD.PROC')
    home = Cpt(EpicsSignal, '_HOME_CMD.PROC')
    clear_encoder_loss = Cpt(EpicsSignal, '_ENC_LSS_CLR_CMD.PROC')
    done = Cpt(EpicsSignal, '_AXIS_STS')

class Monochromator(DeadbandMixin, PVPositioner):
    setpoint = Cpt(EpicsSignal, ":ENERGY_SP", kind="normal")
    readback = Cpt(EpicsSignalRO, ":ENERGY_MON", kind="hinted")

    grating = Cpt(PrettyMotorFMBO, "GrtP}Mtr", name="Mono Grating", kind="config")
    mirror2 = Cpt(PrettyMotorFMBO, "MirP}Mtr", name="Mono Mirror", kind="config")
    cff = Cpt(EpicsSignal, ":CFF_SP", name="Mono CFF", kind="config", auto_monitor=True)
    vls = Cpt(EpicsSignal, ":VLS_B2.A", name="Mono VLS", kind="config", auto_monitor=True)
    gratingx = Cpt(FMB_Mono_Grating_Type, "GrtX}Mtr", kind="config")
    mirror2x = Cpt(FMB_Mono_Grating_Type, "MirX}Mtr", kind="config")

    Scan_Start_ev = Cpt(EpicsSignal, ":EVSTART_SP", name="MONO scan start energy", kind="config")
    Scan_Stop_ev = Cpt(EpicsSignal, ":EVSTOP_SP", name="MONO scan stop energy", kind="config")
    Scan_Speed_ev = Cpt(EpicsSignal, ":EVVELO_SP", name="MONO scan speed", kind="config")
    Scan_Start = Cpt(EpicsSignal, ":START_CMD.PROC", name="MONO scan start command", kind="config")
    Scan_Stop = Cpt(EpicsSignal, ":ENERGY_ST_CMD.PROC", name="MONO scan stop command", kind="config")

    scanlock = Cpt(Signal, value=0, name='lock flag for during scans')
    done = Cpt(EpicsSignalRO, ":ERDY_STS")
    done_value = 1
    stop_signal = Cpt(EpicsSignal, ":ENERGY_ST_CMD")

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


class NewEnPos(PseudoPositioner):
    """Energy pseudopositioner class.
    Parameters:
    -----------
    """

    # synthetic axis
    energy = Cpt(PseudoSingle, kind="hinted", limits=(71, 2250), name="Beamline Energy")
    polarization = Cpt(PseudoSingle, kind="config", limits=(-1, 180), name="X-ray Polarization")
    sample_polarization = Cpt(PseudoSingle, kind="config", name="Sample X-ray polarization")
    # real motors

    monoen = Cpt(Monochromator, "XF:07ID1-OP{Mono:PGM1-Ax:",
                 kind="hinted", name="Mono Energy")
    epugap = Cpt(UndulatorMotor, "SR:C07-ID:G1A{SST1:1-Ax:Gap}-Mtr",
                 kind="config", name="EPU Gap")
    epuphase = Cpt(UndulatorMotor, "SR:C07-ID:G1A{SST1:1-Ax:Phase}-Mtr",
                   kind="config", name="EPU Phase")
    #mir3Pitch = Cpt(FMBHexapodMirrorAxisStandAlonePitch,
    #                "XF:07ID1-OP{Mir:M3ABC", kind="normal",
    #                name="M3Pitch")
    epumode = Cpt(EpuMode,'SR:C07-ID:G1A{SST1:1-Ax:Phase}Phs:Mode',
                  name='EPU Mode', kind="config")


    sim_epu_mode = Cpt(Signal, value=0, name='dont interact with the real EPU',kind='config')
    scanlock = Cpt(Signal, value=0, name="Lock Harmonic, Pitch, Grating for scan",kind='config')
    harmonic = Cpt(Signal, value=1, name="EPU Harmonic", kind='config')
    m3offset = Cpt(Signal, value=7.91, name="EPU Harmonic", kind='config')
    rotation_motor = None

    def __init__(self, a, rotation_motor=None,
                 configpath=pathlib.Path(__file__).parent.absolute() / "config",
                 **kwargs):
        self.gap_fit = np.zeros((10, 10))
        self.gap_fit[0][:] = [889.981, 222.966, -0.945368, 0.00290731,
                              -5.87973e-06, 7.80556e-09, -6.69661e-12,
                              3.56679e-15, -1.07195e-18, 1.39775e-22]
        self.gap_fit[1][:] = [-51.6545, -1.60757, 0.00914746, -2.65003e-05,
                              4.46303e-08, -4.8934e-11, 3.51531e-14,
                              -1.4802e-17, 2.70647e-21, 0]
        self.gap_fit[2][:] = [9.74128, 0.0528884, -0.000270428, 6.71135e-07,
                              -6.68204e-10, 2.71974e-13, -2.82766e-17,
                              -3.77566e-21, 0, 0]
        self.gap_fit[3][:] = [-2.94165, -0.00110173, 3.13309e-06, -1.21787e-08,
                              1.21638e-11, -4.27216e-15, 3.59552e-19,
                              0, 0, 0]
        self.gap_fit[4][:] = [0.19242, 2.19545e-05, 6.11159e-08, 4.21707e-11,
                              -6.84942e-14, 1.84302e-17, 0, 0, 0, 0]
        self.gap_fit[5][:] = [-0.00615458, -9.55015e-07, -1.28929e-09,
                              4.28363e-13, 3.26302e-17, 0, 0, 0, 0, 0]
        self.gap_fit[6][:] = [0.000113341, 1.90112e-08, 6.92088e-12,
                              -1.87659e-15, 0, 0, 0, 0, 0, 0]
        self.gap_fit[7][:] = [-1.22095e-06, -1.5686e-10, -1.09857e-14,
                              0, 0, 0, 0, 0, 0, 0]
        self.gap_fit[8][:] = [7.13593e-09, 4.69949e-13, 0, 0, 0, 0, 0, 0, 0, 0]
        self.gap_fit[9][:] = [-1.74622e-11, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.polphase = xr.load_dataarray(configpath / "polphase.nc")
        self.phasepol = xr.DataArray(
            data=self.polphase.pol,
            coords={"phase": self.polphase.values},
            dims={"phase"},
        )
        self.rotation_motor = rotation_motor
        super().__init__(a, **kwargs)
        self.epugap.tolerance.set(3)
        self.epuphase.tolerance.set(10)
        #self.mir3Pitch.tolerance.set(.01)
        self.monoen.tolerance.set(0.01)
        self._ready_to_fly = False
        self._fly_move_st = None
        self._default_time_resolution = 0.05
        self._flyer_lag_ev = 0.1
        self._time_resolution = None
        self._flying = False

    @pseudo_position_argument
    def forward(self, pseudo_pos):
        """Run a forward (pseudo -> real) calculation"""
        # print('In forward')
        epu_mode = self.sim_epu_mode.get()
        ret = self.RealPosition(
            epugap=self.gap(pseudo_pos.energy, pseudo_pos.polarization, self.scanlock.get(), epu_mode),
            monoen=pseudo_pos.energy,
            epuphase=abs(self.phase(pseudo_pos.energy, pseudo_pos.polarization, epu_mode)),
            #mir3Pitch=self.m3pitchcalc(pseudo_pos.energy, self.scanlock.get()),
            epumode=self.mode(pseudo_pos.polarization, epu_mode),
            #harmonic=self.choose_harmonic(pseudo_pos.energy,pseudo_pos.polarization,self.scanlock.get())
        )
        # print('finished forward')
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

    def _sequential_move(self, real_pos, timeout=None, **kwargs):
        raise Exception("nope")

    def preflight(self, start, stop, speed, *args, time_resolution=None):
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

        self.energy.set(start).wait()
        time.sleep(0.25)
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

    def kickoff(self):
        kickoff_st = DeviceStatus(device=self)
        self._flyer_queue = Queue()
        self._measuring = True
        self._flyer_buffer = []
        threading.Thread(target=self._aggregate, daemon=True).start()
        kickoff_st.set_finished()
        return kickoff_st

    def _aggregate(self):
        name = self.monoen.readback.name
        while self._measuring:
            rb = self.monoen.readback.read()
            t = time.time()
            value = rb[name]['value']
            ts = rb[name]['timestamp']
            self._flyer_buffer.append(value)
            event = dict()
            event['time'] = t
            event['data'] = dict()
            event['timestamps'] = dict()
            event['data'][name + '_raw'] = value
            event['timestamps'][name + '_raw'] = ts
            self._flyer_queue.put(event)
            if abs(self._last_mono_value - value) > self._flyer_lag_ev:
                self._last_mono_value = value
                self.epugap.set(self.gap(value, self._flyer_pol, False))
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
        dd = dict({self.monoen.readback.name + '_raw': {'source': self.monoen.readback.pvname, 'dtype': 'number', 'shape': []}})
        return {self.name: dd}

        
    # end class methods, begin internal methods

    # begin LUT Functions

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
            return self.epugap.get()  # never move the gap if we are in simulated gap mode
            # this might cause problems if someone else is moving the gap, we might move it back
            # but I think this is not a common reason for this mode

        #self.harmonic.set(self.choose_harmonic(energy, pol, locked))
        energy = energy / self.harmonic.get()

        if (pol == -1):
            encalc = energy - 105.002
            gap = 13979.0
            gap +=   82.857      * encalc ** 1
            gap +=   -0.26294    * encalc ** 2
            gap +=    0.00090199 * encalc ** 3
            gap +=   -2.3176e-06 * encalc ** 4
            gap +=    4.205e-09  * encalc ** 5
            gap +=   -5.139e-12  * encalc ** 6
            gap +=    4.0034e-15 * encalc ** 7
            gap +=   -1.7862e-18 * encalc ** 8
            gap +=    3.4687e-22 * encalc ** 9
            return max(14000.0,min(100000.0, gap))
        elif (pol == -0.5):
            encalc = energy - 104.996
            gap = 14013.0
            gap +=   82.76       * encalc ** 1
            gap +=   -0.26128    * encalc ** 2
            gap +=    0.00088353 * encalc ** 3
            gap +=   -2.2149e-06 * encalc ** 4
            gap +=    3.8919e-09 * encalc ** 5
            gap +=   -4.5887e-12 * encalc ** 6
            gap +=    3.4467e-15 * encalc ** 7
            gap +=   -1.4851e-18 * encalc ** 8
            gap +=    2.795e-22  * encalc ** 9
            return max(14000.0,min(100000.0, gap))
        elif 0 <= pol <= 90:
            return max(14000.0,min(100000.0, self.epu_gap(energy,pol)))
        elif 90 < pol <= 180:
            return max(14000.0,min(100000.0, self.epu_gap(energy,180.0-pol)))
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
                z += self.gap_fit[j, i] * (x ** i) * (y ** j)
        return z

    def phase(self, en, pol, sim=0):
        if sim:
            return self.epuphase.get() # never move the gap if we are in simulated gap mode
            # this might cause problems if someone else is moving the gap, we might move it back
            # but I think this is not a common reason for this mode
        if pol == -1:
            return 15000
        elif pol == -0.5:
            return 15000
        elif 90 < pol <= 180:
            return -min(29500.0, max(0.0,float(self.polphase.interp(pol=180 - pol, method="cubic"))))
        else:
            return min(29500.0, max(0.0, float(self.polphase.interp(pol=pol, method="cubic"))))

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
            return self.epumode.get() # never move the gap if we are in simulated gap mode
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

    def m3pitchcalc(self, energy, locked):
        pitch = self.mir3Pitch.setpoint.get()
        if locked:
            return pitch
        elif "1200" in self.monoen.gratingx.readback.get():
            pitch =  self.m3offset.get()+0.038807*np.exp(-(energy-100)/91.942)+0.050123*np.exp(-(energy-100)/1188.9)
        elif "250l/mm" in self.monoen.gratingx.readback.get():
            pitch =  self.m3offset.get()+0.022665*np.exp(-(energy-90)/37.746)+0.024897*np.exp(-(energy-90)/450.9)
        elif "RSoXS" in self.monoen.gratingx.readback.get():
            pitch =  self.m3offset.get()-0.017669*np.exp(-(energy-100)/41.742)-0.068631*np.exp(-(energy-100)/302.75)

        return round(100*pitch)/100

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



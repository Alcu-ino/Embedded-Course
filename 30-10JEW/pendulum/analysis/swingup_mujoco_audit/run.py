#!/usr/bin/env python3
"""MuJoCo + original SIM swingup.c, deterministic 1 ms loop, no board.

This isolates model/control effects. It does not reproduce USB latency, TIM5
count errors, stepper physics or LQR capture. No files in SIMULAZIONE are edited.
"""
import argparse
import csv
import ctypes
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import tempfile

import mujoco
import numpy as np

sys.dont_write_bytecode = True  # The inspected simulation directory is read-only.

HERE = Path(__file__).resolve().parent
DEFAULT_SOURCE = Path('/home/lorenzo/Downloads/27-10/SIMULAZIONE/Embedded-Course-SIMULAZIONE')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--out', type=Path, default=HERE/'results')
    parser.add_argument('--cases', nargs='+', default=['baseline', 'no_damping', 'half_damping',
                        'small_eps', 'moderate_gain', 'no_contacts'])
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    simdir=args.source/'Simulazione'
    core=args.source/'pendulum/test/Core'
    speed_match = re.search(r'(?m)^\s*#define\s+FMAXmotor\s+([0-9.]+)[fF]?\s',
                            (core/'Src/main.c').read_text())
    if speed_match is None:
        raise ValueError('FMAXmotor must be a numeric literal; inspect main.c before running.')
    fmax_steps = float(speed_match.group(1))
    source_files = [core/'Src/main.c', core/'Src/swingup.c', core/'Inc/swingup.h',
                    core/'Inc/lqr.h', core/'Src/motor.c', core/'Src/hil.c',
                    simdir/'hil_mujoco.py', simdir/'cart_pendolo.xml']
    source_hashes = {str(p.relative_to(args.source)): hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in source_files}
    spec=importlib.util.spec_from_file_location('actual_hil_mujoco',simdir/'hil_mujoco.py')
    hil=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(hil)

    with tempfile.TemporaryDirectory(prefix='pendulum-swingup-') as tmp:
        libpath=Path(tmp)/'controller.so'
        subprocess.run(['gcc','-std=gnu11','-O2','-shared','-fPIC','-Wall','-Wextra',
                        f'-DAUDIT_FMAX_STEPS={fmax_steps}f',
                        '-I',str(HERE.parent/'offline_audit'),'-I',str(core/'Inc'),
                        '-I',str(core/'Src'),str(HERE/'controller_bridge.c'),
                        '-lm','-o',str(libpath)],check=True)
        controller=ctypes.CDLL(str(libpath))
        controller.audit_config.argtypes=[ctypes.c_float]*9
        controller.audit_reset.argtypes=[ctypes.c_ushort]
        controller.audit_tick.argtypes=[ctypes.c_ushort,ctypes.POINTER(ctypes.c_float)]
        controller.audit_tick.restype=ctypes.c_int
        controller.audit_defaults.argtypes=[ctypes.POINTER(ctypes.c_float)]
        defaults=(ctypes.c_float*9)()
        controller.audit_defaults(defaults)
        default_cfg=dict(zip(['eps','k','ref','amax','vmax','kx','kv','J','mgl'],defaults))
        output=(ctypes.c_float*7)()
        summaries=[]
        for name in args.cases:
            plant=hil.Pianta(str(simdir/'cart_pendolo.xml'),.001,2,1.0,1.0,None)
            eq=plant.parametri_equivalenti()
            cfg=default_cfg.copy()
            if name=='no_damping': plant.model.dof_damping[plant.dq]=0.
            elif name=='half_damping': plant.model.dof_damping[plant.dq]=.005
            elif name=='small_eps': cfg['eps']=.05
            elif name=='moderate_gain': cfg.update(k=4.,ref=2.2,eps=.05,kv=2.)
            elif name=='no_contacts':
                plant.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
            elif name=='more_speed': cfg['vmax']=.8
            elif name=='less_speed': cfg['vmax']=32000*.0314159*.005/32
            elif name=='lower_gain_only': cfg['k']=4.
            elif name=='matched_energy_same_eps':
                cfg.update(J=eq['J'],mgl=eq['mgl'],ref=2.2,k=4.,kv=2.)
            elif name=='physical_energy': cfg.update(J=eq['J'],mgl=eq['mgl'],ref=2.,k=4.,eps=.05,kv=2.)
            elif name!='baseline': raise ValueError(name)
            controller.audit_config(*cfg.values())
            controller.audit_reset(plant.conteggi())
            peak=0.; maximum_x=0.; clamped=0; samples=0; contact_ticks=0
            loss=0.; net_work=0.; last_v=plant.xd
            reason='duration'; peaks=[]; previous_height=0.; rising=False
            with (args.out/f'{name}.csv').open('w',newline='') as file:
                writer=csv.writer(file)
                writer.writerow(['t','height_from_down_deg','theta_fw_deg','x','xd','v_cmd',
                                 'a_after_acc_limit','a_command_effective','a_cart','p_command',
                                 'w_filter','w_true','E_est','E_true','pump_sign',
                                 'E_dot_drive','E_dot_damping','contacts'])
                for tick in range(round(args.duration/.001)):
                    captured=controller.audit_tick(plant.conteggi(),output)
                    if captured:
                        reason='swingup_capture_return_20deg';break
                    theta_before=plant.theta_fw()
                    omega_before=float(plant.data.qvel[plant.dq])
                    plant.passo(output[0])
                    acc_cart=(plant.xd-last_v)/.001;last_v=plant.xd
                    theta=plant.theta_fw()
                    height=math.degrees(math.acos(max(-1.,min(1.,math.cos(theta)))))
                    if height<previous_height and rising:
                        peaks.append([plant.t,previous_height])
                    rising=height>previous_height
                    previous_height=height
                    w=float(plant.data.qvel[plant.dq])
                    energy=.5*eq['J']*w*w+eq['mgl']*(1-math.cos(theta))
                    power_drive=eq['m']*eq['lc']*acc_cart*omega_before*math.cos(theta_before)
                    power_loss=float(plant.model.dof_damping[plant.dq])*omega_before**2
                    net_work+=power_drive*.001;loss+=power_loss*.001
                    peak=max(peak,height);maximum_x=max(maximum_x,abs(plant.x))
                    clamped+=abs(output[0])>=cfg['vmax']*.9999
                    samples+=1;contact_ticks+=plant.data.ncon>0
                    if tick%5==0:
                        writer.writerow([plant.t,height,math.degrees(theta),plant.x,plant.xd,
                                         output[0],output[1],output[6],acc_cart,output[2],output[3],w,
                                         output[4],energy,output[5],power_drive,power_loss,plant.data.ncon])
                    if plant.finecorsa():reason='rail_limit';break
            summary=dict(case=name,seconds=plant.t,peak_height_deg=peak,max_abs_x=maximum_x,
                         stop_reason=reason,speed_saturated_fraction=clamped/max(1,samples),
                         contact_ticks=contact_ticks,hinge_damping=float(plant.model.dof_damping[plant.dq]),
                         last_peaks=peaks[-6:],approx_drive_work_J=net_work,approx_damping_loss_J=loss,
                         model=eq,config=cfg)
            print(json.dumps(summary),flush=True)
            summaries.append(summary)
        (args.out/'summary.json').write_text(json.dumps(dict(mujoco_version=mujoco.__version__,
                         duration=args.duration,source=str(args.source),source_sha256=source_hashes,
                         cases=summaries),indent=2)+'\n')


if __name__=='__main__':main()

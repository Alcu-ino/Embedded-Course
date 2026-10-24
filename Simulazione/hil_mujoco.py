#!/usr/bin/env python3
"""
hil_mujoco.py - Hardware-in-the-loop per il pendolo su carrello.

La Nucleo F446RE esegue swing-up + LQR (firmware con HIL_ATTIVO = 1);
questo script simula carrello e pendolo con MuJoCo (cart_pendolo.xml) e parla
con la scheda sulla Virtual COM Port dell'ST-LINK (USART2).

Sincronizzazione: la scheda e' il clock. Per OGNI trama di stato ricevuta
(un tick di TIM1) la simulazione avanza di un passo di controllo dt con la
velocita' del carrello comandata dalla scheda, poi rimanda l'angolo in
conteggi encoder. Il tempo simulato coincide con i tick della scheda: i
ritardi del PC/USB si vedono solo come angolo "vecchio di qualche tick"
(statistica 'ritardo' a schermo), non come deriva del tempo.

Uso:
    pip install mujoco pyserial numpy
    python hil_mujoco.py --port COM5            (Windows)
    python hil_mujoco.py --port /dev/ttyACM0    (Linux)
    python hil_mujoco.py --port /dev/ttyACM0 --log prova.csv --durata 30
    python hil_mujoco.py --port ... --theta0 175     (parte vicino all'alto: prova del solo LQR)

Su macOS: mjpython hil_mujoco.py ...  (richiesto dal viewer passivo di MuJoCo)

Convenzioni (le stesse del firmware):
    angolo encoder dal basso: 0 = giu', 180 gradi = su, 2400 conteggi/giro
    MuJoCo: giunto 'rotazione' a 0 = braccio lungo in alto, pi = in basso
    theta_fw = pi + verso * q_mujoco      (verso = --verso-angolo, default +1)
    x_mujoco = verso_carrello * position  (verso = --verso-carrello, default +1)
Con i default, inclinare il pendolo verso +x fa salire l'angolo sopra 180:
e' la prova descritta per LQR_VERSO_ANGOLO = +1 in lqr.h.
"""
import argparse
import csv
import math
import struct
import sys
import time

import mujoco
import numpy as np
import serial

# --------------------------------------------------------------------------
# Protocollo (deve coincidere con hil.h)
# --------------------------------------------------------------------------
SYNC = b"\xAA\x55"
LEN_ANGOLO = 8
LEN_STATO = 20

FLAG_RESET = 0x01
FLAG_FINECORSA = 0x02

ST_LQR = 0x01
ST_GUASTO = 0x02
ST_ATTESA = 0x04
ST_RESET = 0x08

CONTEGGI_GIRO = 600 * 4          # CPR * RES di encoder.h

# Parametri del firmware (lqr.h / swingup.h), solo per il confronto a schermo
FW_M, FW_L, FW_G = 0.10, 0.7056, 9.81


def crc8(data: bytes) -> int:
    c = 0
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def trama_angolo(seq: int, cnt: int, flags: int) -> bytes:
    corpo = struct.pack("<HHB", seq & 0xFFFF, cnt & 0xFFFF, flags & 0xFF)
    return SYNC + corpo + bytes([crc8(corpo)])


class ParserStato:
    """Estrae le trame di stato (20 byte) dal flusso seriale."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_err = 0

    def feed(self, dati: bytes):
        self.buf += dati
        trame = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                del self.buf[:-1]          # tiene un eventuale 0xAA finale
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < LEN_STATO:
                break
            t = bytes(self.buf[:LEN_STATO])
            if crc8(t[2:LEN_STATO - 1]) == t[LEN_STATO - 1]:
                seq, ack, acc, v, p, st = struct.unpack("<HHfffB", t[2:LEN_STATO - 1])
                trame.append((seq, ack, acc, v, p, st))
                del self.buf[:LEN_STATO]
            else:
                self.crc_err += 1
                del self.buf[:1]           # risincronizza
        return trame


# --------------------------------------------------------------------------
# Impianto MuJoCo
# --------------------------------------------------------------------------
class Pianta:
    def __init__(self, xml, dt, substeps, verso_angolo, verso_carrello, theta0_deg):
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.model.opt.timestep = dt / substeps
        self.data = mujoco.MjData(self.model)
        self.dt = dt
        self.substeps = substeps
        self.va = verso_angolo
        self.vc = verso_carrello
        self.theta0 = theta0_deg

        m = self.model
        j_cart = m.joint("scorrimento").id
        j_pend = m.joint("rotazione").id
        self.qx = m.jnt_qposadr[j_cart]
        self.qq = m.jnt_qposadr[j_pend]
        self.dx = m.jnt_dofadr[j_cart]
        self.dq = m.jnt_dofadr[j_pend]
        self.lim = float(m.jnt_range[j_cart][1])
        self.att = m.actuator("velocita_carrello").id
        self.j_pend = j_pend
        self.reset()

    def reset(self):
        m, d = self.model, self.data
        mujoco.mj_resetDataKeyframe(m, d, m.key("inizio_dal_basso").id)
        if self.theta0 is not None:
            # theta_fw = pi + va*q  ->  q = va*(theta_fw - pi)
            d.qpos[self.qq] = self.va * (math.radians(self.theta0) - math.pi)
        d.ctrl[:] = 0.0
        mujoco.mj_forward(m, d)

    def passo(self, v_cmd):
        """Avanza di un passo di controllo con velocita' carrello comandata [m/s]."""
        self.data.ctrl[self.att] = self.vc * v_cmd
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)

    @property
    def t(self):
        return self.data.time

    @property
    def x(self):
        return float(self.data.qpos[self.qx])

    @property
    def xd(self):
        return float(self.data.qvel[self.dx])

    def theta_fw(self):
        """Angolo nella convenzione del firmware [rad, 0..2pi)."""
        return (math.pi + self.va * float(self.data.qpos[self.qq])) % (2.0 * math.pi)

    def conteggi(self):
        """Angolo quantizzato come TIM3->CNT (0..2399)."""
        return int(self.theta_fw() / (2.0 * math.pi) * CONTEGGI_GIRO) % CONTEGGI_GIRO

    def finecorsa(self):
        return abs(self.x) >= self.lim - 5e-4

    def parametri_equivalenti(self):
        """Pendolo equivalente calcolato dal modello (massa, baricentro, inerzia)."""
        m = self.model
        b = m.jnt_bodyid[self.j_pend]
        mass = float(m.body_mass[b])
        c = np.array(m.body_ipos[b]) - np.array(m.jnt_pos[self.j_pend])
        a = np.array(m.jnt_axis[self.j_pend])
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, m.body_iquat[b])
        R = R.reshape(3, 3)
        I_com = R @ np.diag(m.body_inertia[b]) @ R.T
        r_perp2 = float(c @ c - (a @ c) ** 2)
        J = float(a @ I_com @ a) + mass * r_perp2
        lc = math.sqrt(r_perp2)
        g = -float(m.opt.gravity[2])
        return dict(m=mass, lc=lc, J=J, A21=mass * g * lc / J, B2=mass * lc / J,
                    L_eq=J / (mass * lc), mgl=mass * g * lc)


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="HIL pendolo su carrello: MuJoCo <-> Nucleo F446RE")
    ap.add_argument("--port", required=True, help="porta seriale ST-LINK VCP (COMx, /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=921600, help="deve coincidere con HIL_BAUD")
    ap.add_argument("--xml", default="cart_pendolo.xml")
    ap.add_argument("--dt", type=float, default=0.001, help="periodo di controllo della scheda [s] (1/Fs)")
    ap.add_argument("--substeps", type=int, default=2, help="passi MuJoCo per passo di controllo")
    ap.add_argument("--verso-angolo", type=float, default=1.0, choices=[1.0, -1.0])
    ap.add_argument("--verso-carrello", type=float, default=1.0, choices=[1.0, -1.0])
    ap.add_argument("--theta0", type=float, default=None,
                    help="angolo iniziale in gradi, convenzione firmware (0 = giu', 180 = su). "
                         "Default: keyframe 'inizio_dal_basso'")
    ap.add_argument("--durata", type=float, default=None, help="tempo simulato massimo [s]")
    ap.add_argument("--log", default=None, help="file CSV con un passo per riga")
    ap.add_argument("--no-viewer", action="store_true")
    args = ap.parse_args()

    pianta = Pianta(args.xml, args.dt, args.substeps, args.verso_angolo,
                    args.verso_carrello, args.theta0)

    eq = pianta.parametri_equivalenti()
    print("Pendolo nel modello MuJoCo: m = %.3f kg, lc = %.3f m, J = %.4f kg m^2"
          % (eq["m"], eq["lc"], eq["J"]))
    print("  -> L equivalente = %.3f m, A21 = %.2f 1/s^2, B2 = %.3f 1/m, m*g*lc = %.3f J"
          % (eq["L_eq"], eq["A21"], eq["B2"], eq["mgl"]))
    print("Firmware (lqr.h/swingup.h): L = %.4f m, A21 = %.2f 1/s^2, B2 = %.3f 1/m, m*g*l = %.3f J"
          % (FW_L, FW_G / FW_L, 1.0 / FW_L, FW_M * FW_G * FW_L))

    ser = serial.Serial(args.port, args.baud, timeout=0, write_timeout=0.1)
    ser.reset_input_buffer()
    parser = ParserStato()

    viewer = None
    if not args.no_viewer:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(pianta.model, pianta.data)

    log = None
    if args.log:
        fh = open(args.log, "w", newline="")
        log = csv.writer(fh)
        log.writerow(["t", "x", "xd", "theta_deg", "cnt", "acc_cmd", "v_cmd", "p_scheda",
                      "lqr", "ritardo_tick"])

    # ------------------------------------------------------------------
    # 1) Aggancio: RESET finche' la scheda non lo conferma
    # ------------------------------------------------------------------
    print("Attendo la scheda su %s @ %d baud..." % (args.port, args.baud))
    seq = 0
    t_reset = 0.0
    in_corsa = False
    inviato_da = {}               # seq angolo -> tick scheda che l'ha generato
    ultimo_rx = time.monotonic()

    # statistiche
    n_tick, t_stat = 0, time.monotonic()
    rit_somma, rit_max, rit_n = 0, 0, 0
    tick_prec = None
    tick_persi = 0
    t_viewer = 0.0

    try:
        while True:
            ora = time.monotonic()
            if viewer is not None and not viewer.is_running():
                break

            if not in_corsa and ora - t_reset > 0.05:
                pianta.reset()
                ser.write(trama_angolo(seq, pianta.conteggi(), FLAG_RESET))
                t_reset = ora
                seq = (seq + 1) & 0xFFFF

            dati = ser.read(ser.in_waiting or 1)
            if not dati:
                if in_corsa and ora - ultimo_rx > 1.0:
                    print("\nNessuna trama dalla scheda da 1 s: scheda resettata o scollegata.")
                    break
                time.sleep(0.0002)
                continue
            ultimo_rx = ora

            for (tick, ack, acc, v, p, st) in parser.feed(dati):
                if st & ST_RESET:
                    # la scheda ha azzerato il controllo con l'angolo 'ack'
                    pianta.reset()
                    seq = (ack + 1) & 0xFFFF
                    inviato_da = {ack: tick}
                    tick_prec = tick
                    if not in_corsa:
                        print("Agganciato: simulazione avviata.")
                    in_corsa = True
                    continue
                if not in_corsa:
                    continue
                if st & ST_GUASTO:
                    print("\nGUASTO dalla scheda (finecorsa) a t = %.3f s, x = %.3f m"
                          % (pianta.t, pianta.x))
                    raise KeyboardInterrupt
                if st & ST_ATTESA:
                    print("\nLa scheda ha perso il collegamento (timeout): nuovo aggancio.")
                    in_corsa = False
                    t_reset = 0.0
                    break

                # tick saltati dalla scheda (trama di stato non inviata)
                if tick_prec is not None:
                    salto = (tick - tick_prec) & 0xFFFF
                    tick_persi += max(0, salto - 1)
                tick_prec = tick

                # ritardo: tick tra la trama che ha generato l'angolo e quella che lo usa
                rit = None
                if ack in inviato_da:
                    rit = (tick - inviato_da[ack]) & 0xFFFF
                    rit_somma += rit
                    rit_n += 1
                    rit_max = max(rit_max, rit)

                # --- un passo di controllo ---
                pianta.passo(v)
                fc = pianta.finecorsa()
                cnt = pianta.conteggi()
                ser.write(trama_angolo(seq, cnt, FLAG_FINECORSA if fc else 0))
                inviato_da[seq] = tick
                if len(inviato_da) > 4096:
                    for k in list(inviato_da)[:2048]:
                        del inviato_da[k]
                seq = (seq + 1) & 0xFFFF
                n_tick += 1

                if log is not None:
                    log.writerow(["%.4f" % pianta.t, "%.5f" % pianta.x, "%.5f" % pianta.xd,
                                  "%.3f" % math.degrees(pianta.theta_fw()), cnt,
                                  "%.4f" % acc, "%.5f" % v, "%.5f" % p,
                                  1 if st & ST_LQR else 0,
                                  "" if rit is None else rit])

                if args.durata is not None and pianta.t >= args.durata:
                    raise KeyboardInterrupt

            if viewer is not None and ora - t_viewer > 1.0 / 60.0:
                viewer.sync()
                t_viewer = ora

            if ora - t_stat >= 1.0:
                f_tick = n_tick / (ora - t_stat)
                rit_med = rit_somma / rit_n if rit_n else float("nan")
                print("t=%7.2f s  tick %6.0f Hz  theta=%6.1f deg  x=%+.3f m  ritardo %.1f/%d tick  "
                      "persi %d  crc %d   " % (pianta.t, f_tick, math.degrees(pianta.theta_fw()),
                                             pianta.x, rit_med, rit_max, tick_persi, parser.crc_err),
                      end="\r", flush=True)
                if in_corsa and n_tick > 0 and abs(f_tick * args.dt - 1.0) > 0.1:
                    print("\nATTENZIONE: la scheda manda %.0f trame/s ma --dt = %g s: "
                          "il tempo simulato non scorre come quello reale." % (f_tick, args.dt))
                n_tick, t_stat = 0, ora
                rit_somma, rit_max, rit_n = 0, 0, 0
    except KeyboardInterrupt:
        pass
    finally:
        print()
        ser.close()
        if log is not None:
            fh.close()
            print("Log salvato in", args.log)
        if viewer is not None:
            viewer.close()


if __name__ == "__main__":
    main()

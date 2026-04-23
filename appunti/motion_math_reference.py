#!/usr/bin/env python3
"""Riferimento leggibile della matematica di movimento di indi-gapers.

Questa è una trascrizione Python delle funzioni matematiche pure implementate
in src/gapers_math.cpp (namespace GapersMath). Serve come documentazione
human-readable della logica di calcolo, senza dipendenze da INDI o hardware.

I test unitari veri e propri si trovano in tests/test_gapers_math.cpp (Google Test).

Costanti driver:
  SPINDLE_STEPS       = 12800   passi per giro mandrino
  ROTATION_THRESHOLD  = 80 * 12800 = 1_024_000   soglia spin-drive
  ENCODER_MIN         = -8_388_608
  ENCODER_MAX         =  8_388_607
  ENCODER_RANGE       = 16_777_216

Costanti assi:
  RA  vp=220000, rs=500000, spd=220088.2, vs=919.456 (correzione siderale)
  DEC vp=220000, rs=500000, spd=192000   (nessuna correzione siderale)
  DOME dome_speed=94.33 s/giro (configurabile via INDI)
"""

from dataclasses import dataclass


SPINDLE_STEPS          = 12800
ROTATION_THRESHOLD     = 80 * SPINDLE_STEPS   # 1_024_000 passi

ENCODER_MIN            = -8_388_608
ENCODER_MAX            =  8_388_607
ENCODER_RANGE          = ENCODER_MAX - ENCODER_MIN  # 16_777_216


@dataclass
class AxisMovementData:
    """Corrisponde a GapersMath::AxisMovementData in include/gapers_math.h"""
    angle:      float   # distanza angolare richiesta (gradi)
    steps:      int     # passi totali (signed)
    startQuote: int     # quota encoder iniziale (spin-drive)
    endQuote:   int     # quota encoder finale   (spin-drive)
    rotations:  int     # numero di giri mandrino (spin-drive)
    time:       float   # tempo di movimento stimato (secondi)


# ---------------------------------------------------------------------------
# GapersMath::rangeDistance
# Normalizza un angolo nell'intervallo [-180, +180].
# Usato per calcolare il percorso più breve tra due posizioni.
# ---------------------------------------------------------------------------
def range_distance(angle: float) -> float:
    r = angle
    while r < -180.0:
        r += 360.0
    while r > 180.0:
        r -= 360.0
    return r


# ---------------------------------------------------------------------------
# GapersMath::normalizeAz
# Normalizza un azimuth nell'intervallo [0, 360).
# Usato per la cupola.
# ---------------------------------------------------------------------------
def normalize_az(az: float) -> float:
    r = az
    while r < 0.0:
        r += 360.0
    while r >= 360.0:
        r -= 360.0
    return r


# ---------------------------------------------------------------------------
# GapersMath::calcMoveTime
# Calcola il tempo di movimento in secondi con rampa lineare accel/decel.
#
# Parametri:
#   steps  passi da percorrere (valore assoluto)
#   vp     velocità di picco (passi/s)
#   rs     passi di rampa (distanza per raggiungere vp da 200 passi/s)
#
# Se steps <= rs  → movimento nella sola zona di rampa (triangolare)
# Se steps >  rs  → tratto a velocità costante + rampe
# ---------------------------------------------------------------------------
def calc_move_time(steps: float, vp: float, rs: float) -> float:
    tr = rs / ((vp - 200.0) / 2.0)   # tempo totale delle due rampe
    if steps > rs:
        return ((steps - rs) / vp) + tr
    return (steps * tr) / rs


# ---------------------------------------------------------------------------
# GapersMath::rotationsCalc
# Calcola quote encoder e numero di giri per movimenti in spin-drive.
# Restituisce False se |steps| < ROTATION_THRESHOLD (non serve spin-drive).
#
# La logica gestisce l'overflow dell'encoder a 24 bit signed:
#   - partenza da ENCODER_MIN (moto positivo) o ENCODER_MAX (moto negativo)
#   - margine di sicurezza per non finire a ridosso dei limiti
#   - endQuote != 0 per evitare che il PLC interpreti zero come "ferma"
# ---------------------------------------------------------------------------
def rotations_calc(steps: int):
    if abs(steps) < ROTATION_THRESHOLD:
        return False, 0, 0, 0

    if steps > 0:
        m_sq = ENCODER_MIN
        m_eq = (steps % ENCODER_RANGE) + m_sq
        m_giri = ((steps - ROTATION_THRESHOLD) // SPINDLE_STEPS) + 1
        if m_eq < (m_sq + ROTATION_THRESHOLD):
            m_sq += ROTATION_THRESHOLD
            m_eq += ROTATION_THRESHOLD
        if m_eq == 0:
            m_sq += 100
            m_eq = 100
        return True, m_sq, m_eq, m_giri

    m_sq = ENCODER_MAX
    m_eq = (steps % ENCODER_RANGE) + m_sq
    m_giri = ((steps + ROTATION_THRESHOLD) // SPINDLE_STEPS) - 1
    if m_eq > (m_sq - ROTATION_THRESHOLD):
        m_sq -= ROTATION_THRESHOLD
        m_eq -= ROTATION_THRESHOLD
    if m_eq == 0:
        m_sq -= 100
        m_eq = -100
    return True, m_sq, m_eq, m_giri


# ---------------------------------------------------------------------------
# GapersMath::setMoveDataRA
# Calcola i parametri di movimento per l'asse RA.
#
# Costanti:
#   vs  = 919.456   passi/s  velocità siderale (correzione durante il goto)
#   vp  = 220000    passi/s  velocità di picco
#   spd = 220088.2  passi/grado
#   rs  = 500000    passi di rampa
#
# La correzione siderale aggiunge vs*tm passi nella direzione del moto
# per compensare la rotazione terrestre durante il tempo di goto.
# ---------------------------------------------------------------------------
def set_move_data_ra(distance: float) -> AxisMovementData:
    vs   = 919.456
    vp   = 220000.0
    spd  = 220088.2
    rs   = 500000.0

    direction = 1 if distance > 0 else -1
    steps = abs(distance) * spd
    tm = calc_move_time(steps, vp, rs)
    correction = tm * vs
    raw = ((steps + 0.5) * direction) + correction
    final_steps = int(raw)   # C++: static_cast<long> tronca verso zero

    ok, sq, eq, giri = rotations_calc(final_steps)
    return AxisMovementData(
        angle=distance, steps=final_steps,
        startQuote=sq, endQuote=eq, rotations=giri,
        time=calc_move_time(abs(final_steps), vp, rs)
    )


# ---------------------------------------------------------------------------
# GapersMath::setMoveDataDEC
# Calcola i parametri di movimento per l'asse DEC.
#
# Costanti:
#   vp  = 220000    passi/s  velocità di picco
#   spd = 192000    passi/grado
#   rs  = 500000    passi di rampa
#
# Nessuna correzione siderale (DEC non ruota con la Terra).
# ---------------------------------------------------------------------------
def set_move_data_dec(distance: float) -> AxisMovementData:
    vp   = 220000.0
    spd  = 192000.0
    rs   = 500000.0

    direction = 1 if distance > 0 else -1
    steps = abs(distance) * spd
    raw = (steps + 0.5) * direction
    final_steps = int(raw)

    ok, sq, eq, giri = rotations_calc(final_steps)
    return AxisMovementData(
        angle=distance, steps=final_steps,
        startQuote=sq, endQuote=eq, rotations=giri,
        time=calc_move_time(abs(final_steps), vp, rs)
    )


# ---------------------------------------------------------------------------
# Calcolo tempo movimento cupola (non estratto in gapers_math.cpp perché
# usa un parametro INDI configurabile a runtime, ma la formula è semplice).
#
# Formula C++ (GapersScope::DomeGoto):
#   long movTime = static_cast<long>(((domeSpeedN[0].value / 360.0) * azDist * 1000.0) + 0.5)
# ---------------------------------------------------------------------------
def dome_move_time_ms(az_current: float, az_target: float, dome_speed_sec: float = 94.33) -> tuple:
    """Restituisce (az_dist_gradi, move_time_ms)."""
    az_current = normalize_az(az_current)
    az_target  = normalize_az(az_target)

    az_dist = az_target - az_current
    if az_dist >  180.0: az_dist -= 360.0
    if az_dist < -180.0: az_dist += 360.0

    if az_dist == 0.0:
        return 0.0, 0

    move_time_ms = int((dome_speed_sec / 360.0) * abs(az_dist) * 1000.0 + 0.5)
    return az_dist, move_time_ms


# ---------------------------------------------------------------------------
# Esempio interattivo (eseguire con: python3 appunti/motion_math_reference.py)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("=== Esempi di calcolo movimento ===\n")

    print("RA: goto da 0h a 1h (15°)")
    ra = set_move_data_ra(range_distance((0.0 - 1.0) * 15.0))
    print(f"  steps={ra.steps:+d}, startQ={ra.startQuote}, endQ={ra.endQuote}, giri={ra.rotations}, t={ra.time:.3f}s\n")

    print("DEC: goto da 90° a 87°")
    dec = set_move_data_dec(range_distance(90.0 - 87.0))
    print(f"  steps={dec.steps:+d}, startQ={dec.startQuote}, endQ={dec.endQuote}, giri={dec.rotations}, t={dec.time:.3f}s\n")

    print("Cupola: da 350° a 10° (percorso breve = +20°)")
    dist, ms = dome_move_time_ms(350.0, 10.0)
    print(f"  az_dist={dist:+.1f}°, move_time={ms}ms\n")

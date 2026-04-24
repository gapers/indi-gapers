/*
GAPers Telescope driver — pure math utilities (implementation)

Copyright (C) 2026 Massimiliano Masserelli
Copyright (C) 2026 Gruppo Astrofili Persicetani
*/

#include "gapers_math.h"

#include <cmath>

namespace GapersMath {

// Riporta l'angolo nel range ±180°, così da usare sempre il percorso angolare più breve.
double rangeDistance(double angle)
{
    double r = angle;
    while (r < -180.0) r += 360.0;
    while (r >  180.0) r -= 360.0;
    return r;
}

double normalizeAz(double az)
{
    while (az >= 360.0) az -= 360.0;
    while (az <    0.0) az += 360.0;
    return az;
}

double calcMoveTime(double steps, double vp, double rs)
{
    // Tempo della rampa lineare: vp-200 è lo swing di velocità (200 = velocità minima di
    // partenza del driver), la velocità media vale (vp-200)/2. La rampa è simmetrica.
    const double tr = rs / ((vp - 200.0) / 2.0);
    // Movimento lungo: il motore raggiunge la velocità di regime per una tratta.
    if (steps > rs)
        return ((steps - rs) / vp) + tr;
    // Movimento breve: il motore non raggiunge mai vp — proporzione su tr.
    return (steps * tr) / rs;
}

// Calcola i parametri per il movimento «per giri» del PLC.
//
// L'encoder stepper ha un range signed 24-bit (-8388608..+8388607), che limita il
// movimento basato su differenza di quota a ≈38°. Per spostamenti maggiori si usa
// questa procedura: si eseguono giri completi sfruttando l'overflow del contatore,
// poi ci si posiziona alla quota esatta con il normale movimento per passi.
//
// ROTATION_THRESHOLD (80 giri = 1 024 000 passi ≈ 4,65°) è un margine di sicurezza:
// il movimento per giri si ferma prima della quota target, lasciando la correzione
// fine al movimento per passi. Senza questo margine si rischierebbe di sorpassare
// la quota e dover invertire il senso di marcia (problematico in RA per la
// correzione siderale). Vedere: appunti/calcoli_movimento_motori.md
bool rotationsCalc(long steps, long &m_sq, long &m_eq, long &m_giri)
{
    m_sq = 0;
    m_eq = 0;
    m_giri = 0;

    // Sanity check: la procedura è progettata per spostamenti ≳38°; sotto
    // ROTATION_THRESHOLD (≈4,65°) non c'è margine sufficiente per i giri.
    if (std::abs(steps) < ROTATION_THRESHOLD)
        return false;

    if (steps > 0) {
        // Movimento orario: encoder parte dal minimo e conta verso l'alto.
        m_sq   = ENCODER_MIN;
        m_eq   = (steps % ENCODER_RANGE) + m_sq;
        m_giri = ((steps - ROTATION_THRESHOLD) / SPINDLE_STEPS) + 1;
        // Evitiamo di trovarci a cavallo del boundary di overflow a fine movimento per giri.
        if (m_eq < (m_sq + ROTATION_THRESHOLD)) {
            m_sq += ROTATION_THRESHOLD;
            m_eq += ROTATION_THRESHOLD;
        }
        // Il PLC non accetta quota == 0 nella richiesta di movimento per giri.
        if (m_eq == 0) {
            m_sq += 100;
            m_eq  = 100;
        }
    } else {
        // Movimento antiorario: encoder parte dal massimo e conta verso il basso.
        m_sq   = ENCODER_MAX;
        m_eq   = (steps % ENCODER_RANGE) + m_sq;
        m_giri = ((steps + ROTATION_THRESHOLD) / SPINDLE_STEPS) - 1;
        // Evitiamo di trovarci a cavallo del boundary di overflow a fine movimento per giri.
        if (m_eq > (m_sq - ROTATION_THRESHOLD)) {
            m_sq -= ROTATION_THRESHOLD;
            m_eq -= ROTATION_THRESHOLD;
        }
        // Il PLC non accetta quota == 0 nella richiesta di movimento per giri.
        if (m_eq == 0) {
            m_sq -= 100;
            m_eq  = -100;
        }
    }
    return true;
}

bool setMoveDataRA(double distance, AxisMovementData &out)
{
    // Costanti asse RA
    const double vs  = 919.456;   // velocità moto siderale apparente (passi/s)
    const double vp  = 220000.0;  // velocità di regime motore (passi/s)
    const double spd = 220088.2;  // passi motore per grado RA
    const double rs  = 500000.0;  // passi totali rampe accel+decel

    // La correzione siderale è sempre positiva (il cielo si muove est→ovest), quindi
    // si lavora sul valore assoluto dei passi e si aggiunge la correzione in fondo.
    const int direction = (distance > 0) ? 1 : -1;
    const double steps  = std::fabs(distance) * spd;
    const double tm     = calcMoveTime(steps, vp, rs);
    // NB: l'algoritmo è volutamente approssimato: non itera per tener conto
    // dell'effetto che i passi di correzione aggiungono a tm. L'errore è trascurabile.
    const double correction = tm * vs;

    out.angle      = distance;
    out.steps      = static_cast<long>((steps + 0.5) * direction + correction);
    out.startQuote = 0;
    out.endQuote   = 0;
    out.rotations  = 0;
    out.time       = tm;

    if (std::abs(out.steps) > ROTATION_THRESHOLD)
        return rotationsCalc(out.steps, out.startQuote, out.endQuote, out.rotations);

    return true;
}

bool setMoveDataDEC(double distance, AxisMovementData &out)
{
    // Costanti asse DEC — nessuna correzione siderale (il moto apparente è solo in RA)
    const double vp  = 220000.0;  // velocità di regime motore (passi/s)
    const double spd = 192000.0;  // passi motore per grado DEC
    const double rs  = 500000.0;  // passi totali rampe accel+decel

    const int direction = (distance > 0) ? 1 : -1;
    const double steps  = std::fabs(distance) * spd;
    const double tm     = calcMoveTime(steps, vp, rs);

    out.angle      = distance;
    out.steps      = static_cast<long>((steps + 0.5) * direction);
    out.startQuote = 0;
    out.endQuote   = 0;
    out.rotations  = 0;
    out.time       = tm;

    if (std::abs(out.steps) > ROTATION_THRESHOLD)
        return rotationsCalc(out.steps, out.startQuote, out.endQuote, out.rotations);

    return true;
}

} // namespace GapersMath

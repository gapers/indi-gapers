/*
GAPers Telescope driver — pure math utilities (implementation)

Copyright (C) 2026 Massimiliano Masserelli
Copyright (C) 2026 Gruppo Astrofili Persicetani
*/

#include "gapers_math.h"

#include <cmath>

namespace GapersMath {

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
    // Ramp time: time to traverse the combined accel+decel ramp at mean
    // velocity (vp-200)/2.
    const double tr = rs / ((vp - 200.0) / 2.0);
    if (steps > rs)
        return ((steps - rs) / vp) + tr;
    return (steps * tr) / rs;
}

bool rotationsCalc(long steps, long &m_sq, long &m_eq, long &m_giri)
{
    m_sq = 0;
    m_eq = 0;
    m_giri = 0;

    if (std::abs(steps) < ROTATION_THRESHOLD)
        return false;

    if (steps > 0) {
        m_sq   = ENCODER_MIN;
        m_eq   = (steps % ENCODER_RANGE) + m_sq;
        m_giri = ((steps - ROTATION_THRESHOLD) / SPINDLE_STEPS) + 1;
        // Keep end quote safely away from the overflow boundary.
        if (m_eq < (m_sq + ROTATION_THRESHOLD)) {
            m_sq += ROTATION_THRESHOLD;
            m_eq += ROTATION_THRESHOLD;
        }
        // The PLC does not accept a quote of exactly 0.
        if (m_eq == 0) {
            m_sq += 100;
            m_eq  = 100;
        }
    } else {
        m_sq   = ENCODER_MAX;
        m_eq   = (steps % ENCODER_RANGE) + m_sq;
        m_giri = ((steps + ROTATION_THRESHOLD) / SPINDLE_STEPS) - 1;
        if (m_eq > (m_sq - ROTATION_THRESHOLD)) {
            m_sq -= ROTATION_THRESHOLD;
            m_eq -= ROTATION_THRESHOLD;
        }
        if (m_eq == 0) {
            m_sq -= 100;
            m_eq  = -100;
        }
    }
    return true;
}

bool setMoveDataRA(double distance, AxisMovementData &out)
{
    // RA constants
    const double vs  = 919.456;   // Sidereal drift speed (steps/sec)
    const double vp  = 220000.0;  // Peak motor velocity (steps/sec)
    const double spd = 220088.2;  // Motor steps per degree of RA movement
    const double rs  = 500000.0;  // Ramp steps (accel + decel)

    const int direction = (distance > 0) ? 1 : -1;
    const double steps  = std::fabs(distance) * spd;
    const double tm     = calcMoveTime(steps, vp, rs);
    // Compensate for sky drift during the slew.
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
    // DEC constants (no sidereal correction needed)
    const double vp  = 220000.0;  // Peak motor velocity (steps/sec)
    const double spd = 192000.0;  // Motor steps per degree of DEC movement
    const double rs  = 500000.0;  // Ramp steps (accel + decel)

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

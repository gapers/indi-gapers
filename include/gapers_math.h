/*
GAPers Telescope driver — pure math utilities

Pure functions with no INDI dependency, extracted from the driver to allow
unit testing without linking against libindi.

Copyright (C) 2026 Massimiliano Masserelli
Copyright (C) 2026 Gruppo Astrofili Persicetani
*/

#pragma once

#include <cstdint>

namespace GapersMath {

// ---------------------------------------------------------------------------
// Hardware constants
// ---------------------------------------------------------------------------

/// Steps per motor revolution (stepper spindle).
constexpr long SPINDLE_STEPS = 12800;

/// Threshold above which spin-based (rotation) driving is used instead of
/// direct step commands.  Equals 80 motor revolutions (~4.65 degrees for RA).
constexpr long ROTATION_THRESHOLD = 80 * SPINDLE_STEPS; // 1 024 000 steps

/// Encoder integer range boundaries (signed 24-bit).
constexpr long ENCODER_MIN   = -8388608;
constexpr long ENCODER_MAX   =  8388607;
constexpr long ENCODER_RANGE = ENCODER_MAX - ENCODER_MIN + 1; // 16 777 216

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

/// Result of an axis movement calculation.
struct AxisMovementData {
    double angle;       ///< Requested angular distance (degrees)
    long   steps;       ///< Motor steps (signed; positive = one direction)
    long   startQuote;  ///< Encoder start quote for spin-based moves (0 otherwise)
    long   endQuote;    ///< Encoder end   quote for spin-based moves (0 otherwise)
    long   rotations;   ///< Number of motor revolutions (0 for step-based moves)
    double time;        ///< Estimated movement time (seconds)
};

// ---------------------------------------------------------------------------
// Pure math functions
// ---------------------------------------------------------------------------

/// Normalize an angle to the [-180, +180) degree range (shortest path).
double rangeDistance(double angle);

/// Normalize an azimuth to the [0, 360) degree range.
double normalizeAz(double az);

/// Calculate movement time (seconds) given step count, peak velocity, and
/// ramp steps.  Assumes symmetric linear acceleration/deceleration ramps.
///
/// @param steps  Absolute number of motor steps (must be >= 0).
/// @param vp     Peak velocity (steps/sec).
/// @param rs     Total ramp steps (accel + decel combined).
double calcMoveTime(double steps, double vp, double rs);

/// Calculate encoder quotes and revolution count for spin-based (long) moves.
///
/// @param steps  Signed step count (abs must be >= ROTATION_THRESHOLD).
/// @param m_sq   Output: encoder start quote.
/// @param m_eq   Output: encoder end   quote.
/// @param m_giri Output: revolution count (signed).
/// @return true on success, false if steps is below the rotation threshold.
bool rotationsCalc(long steps, long &m_sq, long &m_eq, long &m_giri);

/// Calculate RA axis movement parameters including sidereal correction.
///
/// @param distance  Angular distance in degrees (signed).
/// @param out       Output movement data.
/// @return true on success.
bool setMoveDataRA(double distance, AxisMovementData &out);

/// Calculate DEC axis movement parameters (no sidereal correction).
///
/// @param distance  Angular distance in degrees (signed).
/// @param out       Output movement data.
/// @return true on success.
bool setMoveDataDEC(double distance, AxisMovementData &out);

} // namespace GapersMath

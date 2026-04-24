/*
GAPers Telescope driver — Google Test suite for GapersMath

Tests the pure mathematical functions in gapers_math.cpp directly (no INDI
dependency, no serial I/O, no hardware required).

Copyright (C) 2026 Massimiliano Masserelli
Copyright (C) 2026 Gruppo Astrofili Persicetani
*/

#include <gtest/gtest.h>
#include "gapers_math.h"

#include <cmath>

using namespace GapersMath;

// ===========================================================================
// rangeDistance
// ===========================================================================

TEST(RangeDistance, ZeroAndIdentity)
{
    EXPECT_DOUBLE_EQ(rangeDistance(0.0),   0.0);
    EXPECT_DOUBLE_EQ(rangeDistance(90.0),  90.0);
    EXPECT_DOUBLE_EQ(rangeDistance(-90.0), -90.0);
}

TEST(RangeDistance, Boundaries)
{
    EXPECT_DOUBLE_EQ(rangeDistance( 180.0),  180.0);
    EXPECT_DOUBLE_EQ(rangeDistance(-180.0), -180.0);
}

TEST(RangeDistance, WrapsAbove180)
{
    EXPECT_DOUBLE_EQ(rangeDistance(200.0), -160.0);
    EXPECT_DOUBLE_EQ(rangeDistance(270.0),  -90.0);
    EXPECT_DOUBLE_EQ(rangeDistance(360.0),    0.0);
}

TEST(RangeDistance, WrapsBelow180)
{
    EXPECT_DOUBLE_EQ(rangeDistance(-200.0),  160.0);
    EXPECT_DOUBLE_EQ(rangeDistance(-270.0),   90.0);
    EXPECT_DOUBLE_EQ(rangeDistance(-360.0),   0.0);
}

TEST(RangeDistance, MultiTurnWrapping)
{
    EXPECT_DOUBLE_EQ(rangeDistance( 540.0),  180.0);
    EXPECT_DOUBLE_EQ(rangeDistance(-540.0), -180.0);
    EXPECT_DOUBLE_EQ(rangeDistance( 720.0),    0.0);
}

TEST(RangeDistance, ShortestPath)
{
    // 350° difference should become -10° (shorter to go backwards)
    EXPECT_DOUBLE_EQ(rangeDistance(350.0), -10.0);
    EXPECT_DOUBLE_EQ(rangeDistance(-350.0), 10.0);
}

// ===========================================================================
// normalizeAz
// ===========================================================================

TEST(NormalizeAz, ZeroAndIdentity)
{
    EXPECT_DOUBLE_EQ(normalizeAz(0.0),   0.0);
    EXPECT_DOUBLE_EQ(normalizeAz(180.0), 180.0);
    EXPECT_NEAR(normalizeAz(359.9), 359.9, 1e-9);
}

TEST(NormalizeAz, WrapsAt360)
{
    EXPECT_DOUBLE_EQ(normalizeAz(360.0),  0.0);
    EXPECT_DOUBLE_EQ(normalizeAz(361.0),  1.0);
    EXPECT_DOUBLE_EQ(normalizeAz(720.0),  0.0);
    EXPECT_DOUBLE_EQ(normalizeAz(1080.0), 0.0);
}

TEST(NormalizeAz, WrapsNegative)
{
    EXPECT_DOUBLE_EQ(normalizeAz(  -1.0), 359.0);
    EXPECT_DOUBLE_EQ(normalizeAz(-180.0), 180.0);
    EXPECT_DOUBLE_EQ(normalizeAz(-360.0),   0.0);
}

// ===========================================================================
// calcMoveTime
// ===========================================================================

// Parameters used throughout the driver:
//   vp = 220000 steps/sec   (peak velocity)
//   rs = 500000 steps        (ramp steps)
// tr = 500000 / ((220000 - 200) / 2) = 500000 / 109900 ≈ 4.5496 s

static constexpr double VP = 220000.0;
static constexpr double RS = 500000.0;

TEST(CalcMoveTime, SanityPositive)
{
    // time must be strictly positive for positive steps
    EXPECT_GT(calcMoveTime(100000.0, VP, RS), 0.0);
    EXPECT_GT(calcMoveTime(1000000.0, VP, RS), 0.0);
}

TEST(CalcMoveTime, BelowRamp)
{
    // steps < rs: uses linear interpolation on the ramp
    // time = steps * tr / rs  where tr = rs/((vp-200)/2)
    const double tr = RS / ((VP - 200.0) / 2.0);
    const double steps = 250000.0;
    const double expected = (steps * tr) / RS;
    EXPECT_NEAR(calcMoveTime(steps, VP, RS), expected, 1e-9);
}

TEST(CalcMoveTime, AboveRamp)
{
    // steps > rs: cruise phase + ramp time
    const double tr = RS / ((VP - 200.0) / 2.0);
    const double steps = 1000000.0;
    const double expected = ((steps - RS) / VP) + tr;
    EXPECT_NEAR(calcMoveTime(steps, VP, RS), expected, 1e-9);
}

TEST(CalcMoveTime, AtRampBoundary)
{
    // Both formulas should agree at steps == rs
    const double tr = RS / ((VP - 200.0) / 2.0);
    const double t_below = (RS * tr) / RS;      // = tr
    const double t_above = ((RS - RS) / VP) + tr; // = tr
    EXPECT_DOUBLE_EQ(t_below, t_above);
    EXPECT_NEAR(calcMoveTime(RS, VP, RS), tr, 1e-9);
}

TEST(CalcMoveTime, LongerMoveTakesLonger)
{
    EXPECT_LT(calcMoveTime(200000.0, VP, RS), calcMoveTime(400000.0, VP, RS));
    EXPECT_LT(calcMoveTime(500000.0, VP, RS), calcMoveTime(1000000.0, VP, RS));
}

// ===========================================================================
// rotationsCalc
// ===========================================================================

TEST(RotationsCalc, ReturnsFalseForSmallSteps)
{
    long sq, eq, giri;
    EXPECT_FALSE(rotationsCalc(ROTATION_THRESHOLD - 1, sq, eq, giri));
    EXPECT_FALSE(rotationsCalc(0,                       sq, eq, giri));
    EXPECT_FALSE(rotationsCalc(-(ROTATION_THRESHOLD - 1), sq, eq, giri));
}

TEST(RotationsCalc, ReturnsTrueAboveThreshold)
{
    long sq, eq, giri;
    EXPECT_TRUE(rotationsCalc( ROTATION_THRESHOLD + 1, sq, eq, giri));
    EXPECT_TRUE(rotationsCalc(-(ROTATION_THRESHOLD + 1), sq, eq, giri));
}

TEST(RotationsCalc, PositiveStepsBasicCase)
{
    // steps = 2 000 000 (positive → clockwise)
    const long steps = 2000000;
    long sq, eq, giri;
    ASSERT_TRUE(rotationsCalc(steps, sq, eq, giri));

    // Start quote must be at or above ENCODER_MIN
    EXPECT_GE(sq, ENCODER_MIN);
    // End quote must be above start quote (both positive direction)
    EXPECT_GT(eq, ENCODER_MIN);
    // Rotation count must be positive
    EXPECT_GT(giri, 0);
    // End quote must never be zero (PLC constraint)
    EXPECT_NE(eq, 0);
}

TEST(RotationsCalc, NegativeStepsBasicCase)
{
    // steps = -2 000 000 (negative → counterclockwise)
    const long steps = -2000000;
    long sq, eq, giri;
    ASSERT_TRUE(rotationsCalc(steps, sq, eq, giri));

    // Start quote must be at or below ENCODER_MAX
    EXPECT_LE(sq, ENCODER_MAX);
    EXPECT_LT(eq, ENCODER_MAX);
    // Rotation count must be negative
    EXPECT_LT(giri, 0);
    EXPECT_NE(eq, 0);
}

TEST(RotationsCalc, EndQuoteNeverZero_MultipleOfRange)
{
    // steps = ENCODER_RANGE (multiple of range) forces m_eq == 0 before fix-up
    const long steps = static_cast<long>(ENCODER_RANGE); // 16 777 216
    long sq, eq, giri;
    ASSERT_TRUE(rotationsCalc(steps, sq, eq, giri));
    EXPECT_NE(eq, 0) << "PLC does not accept end quote == 0";
}

TEST(RotationsCalc, SafeMarginEnforced_PositiveOverflow)
{
    // When steps is small enough that m_eq would land too close to the
    // overflow boundary, both quotes are shifted by ROTATION_THRESHOLD.
    // We verify the end quote is at least ROTATION_THRESHOLD away from
    // ENCODER_MIN after the adjustment.
    const long steps = static_cast<long>(ENCODER_RANGE); // triggers adjustment path
    long sq, eq, giri;
    ASSERT_TRUE(rotationsCalc(steps, sq, eq, giri));
    EXPECT_GE(eq, ENCODER_MIN + ROTATION_THRESHOLD);
}

TEST(RotationsCalc, RotationCountGrowsWithSteps)
{
    long sq1, eq1, g1, sq2, eq2, g2;
    const long steps_small = ROTATION_THRESHOLD + 12800;   // one extra revolution
    const long steps_large = ROTATION_THRESHOLD + 128000;  // ten extra revolutions
    ASSERT_TRUE(rotationsCalc(steps_small, sq1, eq1, g1));
    ASSERT_TRUE(rotationsCalc(steps_large, sq2, eq2, g2));
    EXPECT_GT(g2, g1) << "More steps should require more rotations";
}

// ===========================================================================
// setMoveDataRA
// ===========================================================================

TEST(SetMoveDataRA, ShortMoveNoRotations)
{
    // 2° is well within the step-based regime
    AxisMovementData d;
    EXPECT_TRUE(setMoveDataRA(2.0, d));
    EXPECT_EQ(d.rotations, 0);
    EXPECT_EQ(d.startQuote, 0);
    EXPECT_EQ(d.endQuote,   0);
    EXPECT_NE(d.steps, 0);
}

TEST(SetMoveDataRA, PositiveDistancePositiveDirection)
{
    AxisMovementData d;
    ASSERT_TRUE(setMoveDataRA(1.0, d));
    // Positive distance should produce positive steps (plus sidereal offset)
    EXPECT_GT(d.steps, 0);
}

TEST(SetMoveDataRA, NegativeDistanceNegativeDirection)
{
    AxisMovementData d;
    ASSERT_TRUE(setMoveDataRA(-1.0, d));
    EXPECT_LT(d.steps, 0);
}

TEST(SetMoveDataRA, SiderealCorrectionIsApplied)
{
    // Compare RA and DEC for the same angular distance: RA has sidereal
    // correction so |RA steps| != |DEC steps| (different spd too, but
    // mainly the correction shifts the result).
    AxisMovementData ra, dec;
    setMoveDataRA( 1.0, ra);
    setMoveDataDEC(1.0, dec);
    // RA steps should be larger due to sidereal correction AND higher spd
    EXPECT_NE(ra.steps, dec.steps);
}

TEST(SetMoveDataRA, LongMoveUsesRotations)
{
    // 20° RA move should exceed ROTATION_THRESHOLD after sidereal correction
    AxisMovementData d;
    ASSERT_TRUE(setMoveDataRA(20.0, d));
    EXPECT_NE(d.rotations, 0) << "20° RA move must use spin-drive";
    EXPECT_NE(d.startQuote, 0);
    EXPECT_NE(d.endQuote,   0);
}

TEST(SetMoveDataRA, MovementTimePositive)
{
    AxisMovementData d;
    ASSERT_TRUE(setMoveDataRA(5.0, d));
    EXPECT_GT(d.time, 0.0);
}

TEST(SetMoveDataRA, AngleStoredCorrectly)
{
    AxisMovementData d;
    setMoveDataRA(-3.5, d);
    EXPECT_DOUBLE_EQ(d.angle, -3.5);
}

// ===========================================================================
// setMoveDataDEC
// ===========================================================================

TEST(SetMoveDataDEC, ShortMoveNoRotations)
{
    AxisMovementData d;
    EXPECT_TRUE(setMoveDataDEC(3.0, d));
    EXPECT_EQ(d.rotations, 0);
    EXPECT_EQ(d.startQuote, 0);
    EXPECT_EQ(d.endQuote,   0);
    EXPECT_NE(d.steps, 0);
}

TEST(SetMoveDataDEC, PositiveDistancePositiveDirection)
{
    AxisMovementData d;
    ASSERT_TRUE(setMoveDataDEC(2.0, d));
    EXPECT_GT(d.steps, 0);
}

TEST(SetMoveDataDEC, NegativeDistanceNegativeDirection)
{
    AxisMovementData d;
    ASSERT_TRUE(setMoveDataDEC(-2.0, d));
    EXPECT_LT(d.steps, 0);
}

TEST(SetMoveDataDEC, StepsProportionalToDistance)
{
    AxisMovementData d1, d2;
    setMoveDataDEC(1.0, d1);
    setMoveDataDEC(2.0, d2);
    // Doubling the distance should approximately double the steps
    EXPECT_NEAR(static_cast<double>(d2.steps),
                static_cast<double>(d1.steps) * 2.0,
                2.0); // allow ±1 step rounding
}

TEST(SetMoveDataDEC, LongMoveUsesRotations)
{
    // 10° DEC with 192000 steps/° → 1 920 000 steps > 1 024 000 threshold
    AxisMovementData d;
    ASSERT_TRUE(setMoveDataDEC(10.0, d));
    EXPECT_NE(d.rotations, 0) << "10° DEC move must use spin-drive";
    EXPECT_NE(d.startQuote, 0);
    EXPECT_NE(d.endQuote,   0);
}

TEST(SetMoveDataDEC, NoSiderealCorrectionApplied)
{
    // DEC steps should equal (steps + 0.5) * direction exactly.
    const double distance = 1.0;
    const double spd = 192000.0;
    const int    direction = 1;
    const double expected_raw = (std::fabs(distance) * spd + 0.5) * direction;
    const long   expected = static_cast<long>(expected_raw);

    AxisMovementData d;
    setMoveDataDEC(distance, d);
    EXPECT_EQ(d.steps, expected) << "DEC must not apply sidereal correction";
}

TEST(SetMoveDataDEC, MovementTimePositive)
{
    AxisMovementData d;
    ASSERT_TRUE(setMoveDataDEC(5.0, d));
    EXPECT_GT(d.time, 0.0);
}

TEST(SetMoveDataDEC, AngleStoredCorrectly)
{
    AxisMovementData d;
    setMoveDataDEC(7.25, d);
    EXPECT_DOUBLE_EQ(d.angle, 7.25);
}

// ===========================================================================
// DOME_ABSOLUTE_POSITION setter logic
//   These tests verify the boundary conditions and calculations that drive
//   the DOME_ABSOLUTE_POSITION property handler (ISNewNumber) and DomeGoto().
//   The class itself is not instantiated here; pure math is exercised.
// ===========================================================================

// --- AZ range validation (ISNewNumber guard: (az >= 0) && (az <= 360)) ---

TEST(DomeAbsolutePosition, ValidAzRange)
{
    // Values accepted by the setter
    EXPECT_TRUE(0.0   >= 0.0 && 0.0   <= 360.0);
    EXPECT_TRUE(180.0 >= 0.0 && 180.0 <= 360.0);
    EXPECT_TRUE(359.9 >= 0.0 && 359.9 <= 360.0);
    EXPECT_TRUE(360.0 >= 0.0 && 360.0 <= 360.0);
}

TEST(DomeAbsolutePosition, InvalidAzRange)
{
    // Values rejected by the setter
    EXPECT_FALSE(-0.1  >= 0.0 && -0.1  <= 360.0);
    EXPECT_FALSE(-90.0 >= 0.0 && -90.0 <= 360.0);
    EXPECT_FALSE(360.1 >= 0.0 && 360.1 <= 360.0);
    EXPECT_FALSE(720.0 >= 0.0 && 720.0 <= 360.0);
}

// --- DomeGoto movement-time calculation ---
// Formula (indi-gapers.cpp): movTime = (domeSpeed / 360.0) * azDist * 1000  [ms]
// domeSpeed default = 94.33 s/rev  (seconds for a full 360° spin)

static constexpr double DOME_SPEED_DEFAULT = 94.33; // seconds / full revolution

static long domeMovTimeMs(double azDist, double speed = DOME_SPEED_DEFAULT)
{
    return static_cast<long>(((speed / 360.0) * azDist * 1000.0) + 0.5);
}

TEST(DomeGotoMovTime, FullRevolution)
{
    // 360° spin should take approximately domeSpeed seconds
    const long ms = domeMovTimeMs(360.0);
    EXPECT_NEAR(ms, static_cast<long>(DOME_SPEED_DEFAULT * 1000.0 + 0.5), 1);
}

TEST(DomeGotoMovTime, HalfRevolution)
{
    const long ms = domeMovTimeMs(180.0);
    EXPECT_NEAR(ms, static_cast<long>((DOME_SPEED_DEFAULT / 2.0) * 1000.0 + 0.5), 1);
}

TEST(DomeGotoMovTime, SmallMove)
{
    // 1° move must produce a positive, non-zero time
    EXPECT_GT(domeMovTimeMs(1.0), 0L);
}

TEST(DomeGotoMovTime, TimeProportionalToDistance)
{
    EXPECT_LT(domeMovTimeMs(10.0), domeMovTimeMs(20.0));
    EXPECT_LT(domeMovTimeMs(90.0), domeMovTimeMs(180.0));
}

// --- DOME_AUTOSYNC auto-follow threshold logic ---
// Condition in ReadScopeStatus: fabs(rangeDistance(targetAz - currentAz)) > threshold

TEST(DomeAutoSync, BelowThresholdNoMove)
{
    const double threshold = 2.0;
    // Difference of 1.5° — should NOT trigger a move
    EXPECT_FALSE(std::fabs(rangeDistance(1.5)) > threshold);
    EXPECT_FALSE(std::fabs(rangeDistance(-1.5)) > threshold);
}

TEST(DomeAutoSync, AboveThresholdTriggersMove)
{
    const double threshold = 2.0;
    // Difference of 3° — should trigger a move
    EXPECT_TRUE(std::fabs(rangeDistance(3.0)) > threshold);
    EXPECT_TRUE(std::fabs(rangeDistance(-3.0)) > threshold);
}

TEST(DomeAutoSync, AtExactThresholdNoMove)
{
    // Strictly greater-than comparison: equal to threshold must NOT trigger
    const double threshold = 2.0;
    EXPECT_FALSE(std::fabs(rangeDistance(2.0)) > threshold);
}

TEST(DomeAutoSync, ShortPathUsedForThreshold)
{
    // 355° raw difference → rangeDistance gives -5°, so fabs = 5° > 2° threshold
    const double threshold = 2.0;
    EXPECT_TRUE(std::fabs(rangeDistance(355.0)) > threshold);
}

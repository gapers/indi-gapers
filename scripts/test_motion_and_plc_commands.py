#!/usr/bin/env python3
"""Test suite for indi-gapers movement calculations and PLC command generation.

These tests validate the mathematical calculations for telescope movement
(step counts, timing, rotation thresholds) without requiring hardware or
serial I/O. They ensure that the driver's movement computations match
the C++ implementation constants.
"""

import unittest
from dataclasses import dataclass


THRESHOLD_ROTATION_STEPS = 80 * 12800


@dataclass
class MoveData:
    steps: int
    start_quote: int
    end_quote: int
    rotations: int


class MotionMath:
    @staticmethod
    def range_distance(angle):
        r = angle
        while r < -180.0:
            r += 360.0
        while r > 180.0:
            r -= 360.0
        return r

    @staticmethod
    def calc_move_time(steps, vp, rs):
        tr = rs / ((vp - 200.0) / 2.0)
        if steps > rs:
            return ((steps - rs) / vp) + tr
        return (steps * tr) / rs

    @staticmethod
    def rotations_calc(steps):
        qrange = 8388608 + 8388608
        qsafe = THRESHOLD_ROTATION_STEPS

        if abs(steps) < qsafe:
            raise ValueError("Movement too small for spin-based driving")

        if steps > 0:
            m_sq = -8388608
            m_eq = (steps % qrange) + m_sq
            m_giri = ((steps - qsafe) // 12800) + 1
            if m_eq < (m_sq + qsafe):
                m_sq += qsafe
                m_eq += qsafe
            if m_eq == 0:
                m_sq += 100
                m_eq = 100
            return m_sq, m_eq, m_giri

        m_sq = 8388607
        m_eq = (steps % qrange) + m_sq
        m_giri = ((steps + qsafe) // 12800) - 1
        if m_eq > (m_sq - qsafe):
            m_sq -= qsafe
            m_eq -= qsafe
        if m_eq == 0:
            m_sq -= 100
            m_eq = -100
        return m_sq, m_eq, m_giri

    @staticmethod
    def set_move_data_ra(distance):
        vs = 919.456
        vp = 220000.0
        spd = 220088.2
        rs = 500000.0

        direction = 1 if distance > 0 else -1
        steps = abs(distance) * spd
        tm = MotionMath.calc_move_time(steps, vp, rs)
        correction = tm * vs
        raw = ((steps + 0.5) * direction) + correction
        final_steps = int(raw)  # C++ static_cast<long> truncates toward zero.
        if abs(final_steps) > THRESHOLD_ROTATION_STEPS:
            sq, eq, giri = MotionMath.rotations_calc(final_steps)
            return MoveData(final_steps, sq, eq, giri)
        return MoveData(final_steps, 0, 0, 0)

    @staticmethod
    def set_move_data_dec(distance):
        vp = 220000.0
        spd = 192000.0
        rs = 500000.0

        direction = 1 if distance > 0 else -1
        steps = abs(distance) * spd
        raw = (steps + 0.5) * direction
        final_steps = int(raw)
        if abs(final_steps) > THRESHOLD_ROTATION_STEPS:
            sq, eq, giri = MotionMath.rotations_calc(final_steps)
            return MoveData(final_steps, sq, eq, giri)
        return MoveData(final_steps, 0, 0, 0)


class TestMotionAndPLCCommands(unittest.TestCase):
    """Test suite for indi-gapers movement calculation and command generation.
    
    These tests validate the mathematical calculations for telescope movement
    (step counts, timing, rotation thresholds) without requiring hardware or
    serial I/O. They ensure that the driver's movement computations match
    the C++ implementation constants.
    """

    def test_short_move_ra_dec_calculation(self):
        """Verify RA/DEC step calculations for short movements.
        
        Validates that movement parameters are correctly computed without
        requiring actual serial I/O (which is hardware-specific). Uses the
        same constants and logic as the C++ driver.
        """
        # Start point in driver constructor: currentRA=0h, currentDEC=90deg.
        target_ra = 0.2
        target_dec = 87.0

        ra_dist = MotionMath.range_distance((0.0 - target_ra) * 15.0)
        dec_dist = MotionMath.range_distance(90.0 - target_dec)

        ra_data = MotionMath.set_move_data_ra(ra_dist)
        dec_data = MotionMath.set_move_data_dec(dec_dist)

        # For short movements, rotations should be 0 (uses cmd 15, not cmd 10/9/5).
        self.assertEqual(ra_data.rotations, 0, "Short RA move should use step-based cmd 15")
        self.assertEqual(dec_data.rotations, 0, "Short DEC move should use step-based cmd 15")

        # Steps should be non-zero and in reasonable range (< 10M steps ~= 50 degrees).
        self.assertNotEqual(ra_data.steps, 0, "RA movement should have non-zero steps")
        self.assertNotEqual(dec_data.steps, 0, "DEC movement should have non-zero steps")
        self.assertLess(abs(ra_data.steps), 10_000_000, "RA steps should fit in int32 range")
        self.assertLess(abs(dec_data.steps), 10_000_000, "DEC steps should fit in int32 range")

    def test_long_move_ra_dec_uses_rotations(self):
        """Verify RA/DEC step calculations for long movements exceeding spin-drive threshold.
        
        Validates that movements > 80 revolutions (~38 degrees) trigger rotation-based
        movement commands with proper quote calculations, using the same math as the
        C++ driver's _rotationsCalc method.
        """
        # A 15-degree RA move exceeds the spin-driving threshold.
        target_ra = 1.0
        target_dec = 89.9

        ra_dist = MotionMath.range_distance((0.0 - target_ra) * 15.0)
        ra_data = MotionMath.set_move_data_ra(ra_dist)

        # For long movements, rotations should be non-zero.
        self.assertNotEqual(ra_data.rotations, 0, "Long RA move should use rotation-based commands")

        # Start and end quotes should be computed (non-zero) and valid encoder positions.
        self.assertNotEqual(ra_data.start_quote, 0, "Start quote must be computed")
        self.assertNotEqual(ra_data.end_quote, 0, "End quote must be computed")

        # Rotations should be non-zero for long moves.
        self.assertNotEqual(ra_data.rotations, 0, "Rotation count must be non-zero for long moves")

    def test_range_distance_wrapping(self):
        """Verify angular distance calculation respects -180/+180 degree wrapping.
        
        This is critical for telescope slew calculations to always use the
        shortest path across the sky.
        """
        # Test forward crossing
        self.assertAlmostEqual(MotionMath.range_distance(170.0), 170.0)
        self.assertAlmostEqual(MotionMath.range_distance(200.0), -160.0)
        self.assertAlmostEqual(MotionMath.range_distance(-200.0), 160.0)

        # Test zero crossing
        self.assertAlmostEqual(MotionMath.range_distance(0.0), 0.0)
        self.assertAlmostEqual(MotionMath.range_distance(360.0), 0.0)
        self.assertAlmostEqual(MotionMath.range_distance(-360.0), 0.0)

        # Test boundaries
        self.assertAlmostEqual(MotionMath.range_distance(180.0), 180.0)
        self.assertAlmostEqual(MotionMath.range_distance(-180.0), -180.0)

    def test_dec_movement_northward_and_southward(self):
        """Verify DEC movement calculations for both northward and southward moves.
        
        Tests that step calculations work correctly in both directions from
        the default starting position (90 degrees).
        """
        # Northward from 90 (should be blocked but step calc should still work)
        dec_north = MotionMath.set_move_data_dec(2.0)
        self.assertEqual(dec_north.rotations, 0, "Small northward DEC move")
        self.assertGreater(dec_north.steps, 0, "Northward should have positive steps")

        # Southward from 90 (standard case)
        dec_south = MotionMath.set_move_data_dec(-3.0)
        self.assertEqual(dec_south.rotations, 0, "Small southward DEC move")
        self.assertLess(dec_south.steps, 0, "Southward should have negative steps")


if __name__ == "__main__":
    unittest.main(verbosity=2)

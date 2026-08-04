#!/usr/bin/python3

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rig_calibration import (  # noqa: E402
    G_MPS2,
    fit_aoa_axis,
    fit_imu_six_position,
    fit_linear_range,
    validate_aoa_axis_fit,
    wrap_deg,
)


class CalibrationMathTest(unittest.TestCase):
    def test_wrap_boundary(self):
        self.assertAlmostEqual(wrap_deg(181.0), -179.0)
        self.assertAlmostEqual(wrap_deg(-181.0), 179.0)

    def test_aoa_sign_and_offset(self):
        expected = {"front": 0.0, "left": 90.0, "right": -90.0, "back": 180.0}
        sign = -1
        offset = -17.0
        samples = {
            name: (angle, np.full(30, sign * (angle - offset)))
            for name, angle in expected.items()
        }
        result = fit_aoa_axis(samples)
        self.assertEqual(result["azimuth_sign"], sign)
        self.assertAlmostEqual(result["mount_yaw_offset_deg"], offset)
        self.assertAlmostEqual(result["rms_error_deg"], 0.0)

    def test_aoa_validation_rejects_unstable_direction(self):
        fit = {
            "rms_error_deg": 22.6,
            "max_abs_error_deg": 179.8,
            "stages": {
                "front": {"raw_std_deg": 5.7},
                "left": {"raw_std_deg": 5.1},
                "right": {"raw_std_deg": 32.3},
                "back": {"raw_std_deg": 3.7},
            },
        }
        validation = validate_aoa_axis_fit(fit, 8.0, 30.0, 10.0)
        self.assertFalse(validation["passed"])
        self.assertFalse(validation["checks"]["rms_error"])
        self.assertFalse(validation["checks"]["max_abs_error"])
        self.assertFalse(validation["checks"]["stage_stability"])

    def test_range_equation_matches_runtime_contract(self):
        truth = np.array([1.0, 2.5, 5.0, 8.0])
        raw = 0.97 * truth - 0.12
        result = fit_linear_range(truth, raw)
        self.assertAlmostEqual(result["range_scale"], 0.97)
        self.assertAlmostEqual(result["range_bias_m"], -0.12)
        self.assertLess(result["max_abs_error_m"], 1e-12)

    def test_six_position_fit(self):
        bias = np.array([0.1, -0.07, 0.04])
        gain = np.array([1.02, 0.99, 1.04])
        gyro_bias = np.array([0.004, -0.002, 0.001])
        samples = {}
        for index, axis in enumerate("xyz"):
            for direction, prefix in ((1.0, "+"), (-1.0, "-")):
                values = np.zeros((50, 6))
                values[:, :3] = bias
                values[:, index] += direction * G_MPS2 / gain[index]
                values[:, 3:6] = gyro_bias
                samples[f"{prefix}{axis}"] = values
        result = fit_imu_six_position(samples)
        np.testing.assert_allclose(result["accelerometer_bias_mps2"], bias)
        np.testing.assert_allclose(result["accelerometer_gain"], gain)
        np.testing.assert_allclose(result["gyroscope_bias_rad_s"], gyro_bias)


if __name__ == "__main__":
    unittest.main()

"""Tests for the Rust math backend"""

import numpy as np
import pytest

try:
    from opensourceleg.rust import math as rust_math

    HAS_RUST = True
except ImportError:
    HAS_RUST = False


@pytest.mark.skipif(not HAS_RUST, reason="Rust backend not available")
class TestRustMath:
    """Test cases for Rust math functions"""

    def test_cross_product(self):
        """Test cross product calculation"""
        a = np.array([1.0, 0.0, 0.0])
        b = np.array([0.0, 1.0, 0.0])
        expected = [0.0, 0.0, 1.0]

        result = rust_math.cross_product(a, b)
        assert np.allclose(result, expected)

    def test_magnitude(self):
        """Test vector magnitude calculation"""
        vec = np.array([3.0, 4.0, 0.0])
        expected = 5.0

        result = rust_math.magnitude(vec)
        assert abs(result - expected) < 1e-10

    def test_normalize(self):
        """Test vector normalization"""
        vec = np.array([3.0, 4.0, 0.0])
        expected = [0.6, 0.8, 0.0]

        result = rust_math.normalize(vec)
        assert np.allclose(result, expected)

    def test_normalize_zero_vector(self):
        """Test that normalizing zero vector raises error"""
        vec = np.array([0.0, 0.0, 0.0])

        with pytest.raises(ValueError):
            rust_math.normalize(vec)

    def test_quat_to_rotation_matrix(self):
        """Test quaternion to rotation matrix conversion"""
        # Identity quaternion should give identity matrix
        quat = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]

        result = rust_math.quat_to_rotation_matrix(quat)
        expected = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

        assert np.allclose(result, expected)

    def test_quat_to_rotation_matrix_invalid_size(self):
        """Test that invalid quaternion size raises error"""
        quat = np.array([1.0, 0.0, 0.0])  # Only 3 elements

        with pytest.raises(ValueError):
            rust_math.quat_to_rotation_matrix(quat)

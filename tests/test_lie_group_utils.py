from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from models.lie_group_utils import (
    compose,
    exp_se23,
    exp_so3,
    hat_se23,
    hat_so3,
    inverse,
    jacobian_exp,
    jacobian_log,
    log_se23,
    log_so3,
    minus_right,
    plus_right,
    symmetrize_covariance,
    vee_se23,
    vee_so3,
)


def assert_close(actual: np.ndarray, expected: np.ndarray, tol: float = 1.0e-8) -> None:
    if not np.allclose(actual, expected, atol=tol, rtol=tol):
        raise AssertionError(f"\nactual:\n{actual}\nexpected:\n{expected}")


def test_so3() -> None:
    w = np.array([0.2, -0.1, 0.05])
    W = hat_so3(w)
    assert_close(vee_so3(W), w)
    assert_close(hat_so3(vee_so3(W)), W)
    assert_close(exp_so3(np.zeros(3)), np.eye(3))
    assert_close(log_so3(np.eye(3)), np.zeros(3))
    assert_close(exp_so3(log_so3(exp_so3(w))), exp_so3(w))
    assert_close(log_so3(exp_so3(w)), w)


def test_se23() -> None:
    xi = np.array([0.1, -0.2, 0.05, 1.0, 0.2, -0.3, -0.4, 0.7, 0.5])
    tau = np.array([-0.03, 0.02, 0.01, 0.05, -0.02, 0.03, 0.01, 0.04, -0.05])
    Xi = hat_se23(xi)
    X = exp_se23(xi)
    Y = plus_right(X, tau)
    assert_close(vee_se23(Xi), xi)
    assert_close(hat_se23(vee_se23(Xi)), Xi)
    assert_close(exp_se23(np.zeros(9)), np.eye(5))
    assert_close(log_se23(np.eye(5)), np.zeros(9))
    assert_close(log_se23(X), xi)
    assert_close(compose(X, inverse(X)), np.eye(5))
    assert_close(compose(inverse(X), X), np.eye(5))
    assert_close(minus_right(Y, X), tau)
    assert_close(plus_right(X, minus_right(Y, X)), Y)


def test_jacobian_shapes_and_covariance() -> None:
    xi = np.array([0.02, -0.03, 0.01, 0.1, 0.2, -0.1, -0.2, 0.3, 0.4])
    X = exp_se23(xi)
    assert jacobian_exp(xi).shape == (9, 9)
    assert jacobian_log(X).shape == (9, 9)

    P = np.array([[2.0, 0.3], [0.31, 1.0]])
    P_sym = symmetrize_covariance(P, floor=1.0e-12)
    assert_close(P_sym, P_sym.T)
    eigvals = np.linalg.eigvalsh(P_sym)
    if np.min(eigvals) < -1.0e-10:
        raise AssertionError(f"covariance not PSD enough: {eigvals}")


if __name__ == "__main__":
    test_so3()
    test_se23()
    test_jacobian_shapes_and_covariance()
    print("lie_group_utils tests passed")

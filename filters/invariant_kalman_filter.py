from __future__ import annotations

from typing import Iterable

import numpy as np

from models import invariant_inekf as lie
from models.lie_group_utils import correction_left, exp_se23, symmetrize_covariance
from utils.filter_math import diagonal_covariance, kalman_update
from utils.math_utils import fit_diag, fit_vector


class InvariantKalmanFilter:
    """Small SE_2(3) InEKF-style filter for repository comparison."""

    def __init__(
        self,
        pose_type: str = "3d",
        mode: str = "fused",
        motion_config: dict | None = None,
        measurement_config: dict | None = None,
        initialization_config: dict | None = None,
    ) -> None:
        if pose_type == "6d":
            pose_type = "3d"
        if pose_type != "3d":
            raise ValueError("InvariantKalmanFilter supports only 3d pose.")

        self.pose_type = pose_type
        self.mode = mode
        self.error_dim = 9

        motion_cfg = motion_config or {}
        meas_cfg = measurement_config or {}
        init_cfg = initialization_config or {}
        self.use_imu_velocity = bool(motion_cfg.get("use_imu_velocity", True))
        self.use_imu_rotation = bool(motion_cfg.get("use_imu_rotation", True))
        self.translation_input_frame = str(motion_cfg.get("translation_input_frame", "world"))
        self.translation_input_type = str(motion_cfg.get("translation_input_type", "velocity"))
        self.rotation_input_type = str(motion_cfg.get("rotation_input_type", "increment"))
        self.rotation_representation = str(motion_cfg.get("rotation_representation", "rotvec"))
        self.gravity = np.asarray(motion_cfg.get("gravity", [0.0, 0.0, -9.81]), dtype=float).reshape(3)
        self.accel_bias = fit_vector(motion_cfg.get("accel_bias", [0.0, 0.0, 0.0]), 3)
        self.gyro_bias = fit_vector(motion_cfg.get("gyro_bias", [0.0, 0.0, 0.0]), 3)
        self.velocity_blend = float(motion_cfg.get("velocity_blend", 0.0))
        self.process_noise_diag = fit_diag(
            motion_cfg.get("process_noise_diag", [1e-6, 1e-6, 1e-6, 1e-5, 1e-5, 1e-5, 1e-8, 1e-8, 1e-8]),
            self.error_dim,
        )

        self.measurement_indices = np.asarray(meas_cfg.get("position_indices", [0, 1, 2]), dtype=int)
        self.measurement_noise_diag = fit_diag(
            meas_cfg.get("measurement_noise_diag", [1.0, 1.0, 1.0]),
            self.measurement_indices.size,
        )
        self.innovation_gate_m = float(meas_cfg.get("innovation_gate_m", 0.0))
        self.mahalanobis_gate = float(meas_cfg.get("mahalanobis_gate", 0.0))

        self.Rot = np.eye(3, dtype=float)
        self.v = np.zeros(3, dtype=float)
        self.p = np.zeros(3, dtype=float)
        self.P = np.eye(self.error_dim, dtype=float)
        self.Phi = np.eye(self.error_dim, dtype=float)
        self.Q = diagonal_covariance(self.process_noise_diag)
        self.H = lie.position_measurement_matrix(self.p, self.measurement_indices)
        self.Rm = diagonal_covariance(self.measurement_noise_diag)
        self.innovation = np.zeros(self.measurement_indices.size, dtype=float)
        self.S = np.eye(self.measurement_indices.size, dtype=float)
        self.K = np.zeros((self.error_dim, self.measurement_indices.size), dtype=float)
        self.delta = np.zeros(self.error_dim, dtype=float)
        self.X = lie.as_matrix(self.Rot, self.v, self.p)
        self.dX = np.eye(5, dtype=float)
        self.initialized = False
        self.initialize(init_cfg.get("mean"), init_cfg.get("cov_diag"), init_cfg.get("velocity_mean"))

    @classmethod
    def from_configs(cls, dataset_config: dict, compare_config: dict) -> "InvariantKalmanFilter":
        cfg = compare_config.get("invariant_kalman_filter", compare_config)
        return cls(
            pose_type=dataset_config.get("pose_type", cfg.get("pose_type", "3d")),
            mode=dataset_config.get("mode", cfg.get("mode", "fused")),
            motion_config=cfg.get("motion_model", {}),
            measurement_config=cfg.get("measurement_model", {}),
            initialization_config=cfg.get("initialization", {}),
        )

    def initialize(
        self,
        mean: Iterable[float] | None = None,
        cov_diag: Iterable[float] | None = None,
        velocity_mean: Iterable[float] | None = None,
    ) -> None:
        pose = fit_vector(np.zeros(6) if mean is None else np.asarray(mean, dtype=float).reshape(-1), 6)
        self.p, self.Rot = lie.pose_to_state(pose)
        self.v = fit_vector(np.zeros(3) if velocity_mean is None else np.asarray(velocity_mean, dtype=float).reshape(-1), 3)
        cov = fit_diag(np.ones(self.error_dim) * 1e-3 if cov_diag is None else cov_diag, self.error_dim)
        self.P = diagonal_covariance(cov)
        self.X = lie.as_matrix(self.Rot, self.v, self.p)
        self.initialized = True

    def predict(self, control: Iterable[float] | None, dt: float) -> np.ndarray:
        if not self.initialized:
            self.initialize()
        if control is not None:
            # INEKF-P1: bias-correct control and propagate mean on SE_2(3).
            u = self._correct_control(control)
            self.Rot, self.v, self.p = lie.propagate_mean(
                self.Rot,
                self.v,
                self.p,
                u,
                dt,
                use_imu_velocity=self.use_imu_velocity,
                use_imu_rotation=self.use_imu_rotation,
                translation_input_frame=self.translation_input_frame,
                translation_input_type=self.translation_input_type,
                rotation_input_type=self.rotation_input_type,
                rotation_representation=self.rotation_representation,
                gravity=self.gravity,
                velocity_blend=self.velocity_blend,
            )

        # INEKF-P2: build error-state transition and process covariance.
        self.Phi = lie.error_transition(dt)
        self.Q = diagonal_covariance(self.process_noise_diag * max(float(dt), 1e-3))
        # INEKF-P3: propagate error covariance and synchronize matrix state X_k^-.
        self.P = symmetrize_covariance(self.Phi @ self.P @ self.Phi.T + self.Q)
        self.X = lie.as_matrix(self.Rot, self.v, self.p)
        return self.estimate_pose()

    def measurement_update(self, measurement: Iterable[float] | None) -> np.ndarray:
        if measurement is None:
            return self.estimate_pose()
        z = np.asarray(measurement, dtype=float).reshape(-1)
        if z.size != self.measurement_indices.size:
            raise ValueError("measurement size must match measurement indices.")

        # INEKF-U1: compute position innovation and apply optional outlier gates.
        self.innovation = z - self.p[self.measurement_indices]
        if self._reject_measurement():
            return self.estimate_pose()

        # INEKF-U2: update linearized error covariance and gain.
        self.H = lie.position_measurement_matrix(self.p, self.measurement_indices)
        self.Rm = diagonal_covariance(self.measurement_noise_diag)
        _, P_update, self.innovation, self.S, self.K = kalman_update(
            np.zeros(self.error_dim, dtype=float), self.P, self.innovation, self.H, self.Rm
        )
        # INEKF-U3: map error correction through exp(delta) and inject it into X.
        self.delta = self.K @ self.innovation
        self.dX = exp_se23(self.delta)
        self.Rot, self.v, self.p = lie.from_matrix(correction_left(lie.as_matrix(self.Rot, self.v, self.p), self.delta))
        self.X = lie.as_matrix(self.Rot, self.v, self.p)
        self.P = symmetrize_covariance(P_update)
        return self.estimate_pose()

    def velocity_update(self, measurement: Iterable[float] | None) -> np.ndarray:
        if measurement is None:
            return self.estimate_pose()
        z = np.asarray(measurement, dtype=float).reshape(3)
        self.innovation = z - self.v
        if self.innovation_gate_m > 0.0 and np.linalg.norm(self.innovation) > self.innovation_gate_m:
            return self.estimate_pose()

        H = np.zeros((3, self.error_dim), dtype=float)
        H[:, 3:6] = np.eye(3)
        Rm = diagonal_covariance(fit_diag(self.measurement_noise_diag, 3))
        S = H @ self.P @ H.T + Rm + 1e-12 * np.eye(3)
        if self.mahalanobis_gate > 0.0:
            maha = float(self.innovation.T @ np.linalg.solve(S, self.innovation))
            if maha > self.mahalanobis_gate:
                return self.estimate_pose()

        _, P_update, self.innovation, self.S, self.K = kalman_update(
            np.zeros(self.error_dim, dtype=float), self.P, self.innovation, H, Rm
        )
        self.delta = self.K @ self.innovation
        self.dX = exp_se23(self.delta)
        self.Rot, self.v, self.p = lie.from_matrix(correction_left(lie.as_matrix(self.Rot, self.v, self.p), self.delta))
        self.X = lie.as_matrix(self.Rot, self.v, self.p)
        self.P = symmetrize_covariance(P_update)
        return self.estimate_pose()

    def step(
        self,
        control: Iterable[float] | None,
        measurement: Iterable[float] | None,
        dt: float,
        mode: str | None = None,
    ) -> np.ndarray:
        run_mode = self.mode if mode is None else mode
        if run_mode in {"imu_only", "fused"}:
            # INEKF-S1: run Lie-group prediction when control/IMU is enabled.
            self.predict(control, dt)
        if run_mode in {"gnss_only", "fused"}:
            # INEKF-S2: run position correction when GNSS/position is enabled.
            self.measurement_update(measurement)
        # INEKF-S3: expose the benchmark pose format.
        return self.estimate_pose()

    def run(self, dataset, mode: str | None = None) -> np.ndarray:
        estimates = [
            self.step(sample.get("control"), sample.get("measurement"), float(sample.get("dt", 1.0)), mode=mode)
            for sample in dataset
        ]
        return np.vstack(estimates) if estimates else np.zeros((0, 6), dtype=float)

    def estimate_pose(self) -> np.ndarray:
        return lie.pose_from_state(self.Rot, self.p)

    def _correct_control(self, control: Iterable[float]) -> np.ndarray:
        u = np.asarray(control, dtype=float).reshape(-1)
        if u.size < 6:
            raise ValueError("3D InEKF control must contain [linear(3), angular(3)].")
        u = u.copy()
        u[0:3] -= self.accel_bias
        u[3:6] -= self.gyro_bias
        return u

    def _reject_measurement(self) -> bool:
        if self.innovation_gate_m > 0.0 and np.linalg.norm(self.innovation) > self.innovation_gate_m:
            return True
        self.H = lie.position_measurement_matrix(self.p, self.measurement_indices)
        self.Rm = diagonal_covariance(self.measurement_noise_diag)
        self.S = self.H @ self.P @ self.H.T + self.Rm + 1e-12 * np.eye(self.innovation.size)
        if self.mahalanobis_gate <= 0.0:
            return False
        maha = float(self.innovation.T @ np.linalg.solve(self.S, self.innovation))
        return maha > self.mahalanobis_gate

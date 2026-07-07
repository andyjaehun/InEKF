# InEKF State Estimation Benchmark

This working copy focuses on the Invariant Extended Kalman Filter (InEKF)
implementation for IMU/GNSS state estimation.

The InEKF code is implemented in Python/NumPy and keeps the benchmark data
adapter, trajectory output, and visualization flow needed to run InEKF checks.
Other filter families are intentionally outside the scope of this README.

## InEKF Scope

Main InEKF files:

- `filters/invariant_kalman_filter.py`
- `filters/invariant_kalman_filter_15D.py`
- `models/invariant_inekf.py`
- `models/lie_group_utils.py`

The 9D InEKF error state is:

```text
[delta_phi, delta_v, delta_p]
```

The 15D InEKF error state is:

```text
[delta_phi, delta_v, delta_p, delta_bg, delta_ba]
```

The covariance matrix `P` is defined on the tangent error vector, not directly
on the nominal matrix state.

## Lie Group Utilities

Lie group operations are factored into `models/lie_group_utils.py`.

Implemented `SO(3)` operations:

- `hat_so3`, `vee_so3`
- `exp_so3`, `log_so3`
- `left_jacobian_so3`, `right_jacobian_so3`
- `left_jacobian_inv_so3`, `right_jacobian_inv_so3`

Implemented `SE_2(3)` operations:

- `hat_se23`, `vee_se23`
- `exp_se23`, `log_se23`
- `adjoint_se23`
- `compose`, `inverse`
- `plus_right`, `minus_right`
- Jacobian block helpers
- `symmetrize_covariance`

`SE_2(3)` tangent vectors use this order:

```text
[phi, rho_v, rho_p]
```

## Micro Lie Theory Convention Audit

The core operators in `models/lie_group_utils.py` follow the notation used in
Sola, Deray, and Atchuthan, "A micro Lie theory for state estimation in
robotics":

- `hat_so3`, `vee_so3`, `exp_so3`, and `log_so3` use the standard `SO(3)`
  skew map, Rodrigues exponential, and logarithm.
- `left_jacobian_so3`, `right_jacobian_so3`,
  `left_jacobian_inv_so3`, and `right_jacobian_inv_so3` use the standard
  closed-form `SO(3)` Jacobian expressions.
- `hat_se23`, `vee_se23`, `exp_se23`, and `log_se23` use the embedded
  `SE_2(3)` matrix state
  `[R, v, p; 0, 1, 0; 0, 0, 1]` with tangent vector order
  `[phi, rho_v, rho_p]`.
- `adjoint_se23` implements the `SE_2(3)` adjoint for that same tangent order:
  the rotation block is `R`, and the velocity/position cross blocks are
  `[v]x R` and `[p]x R`.

The repository exposes the right perturbation operators:

```text
plus_right(X, tau)  = X @ Exp(tau)
minus_right(Y, X)   = Log(X^{-1} @ Y)
```

The inverse and composition Jacobian helpers are written for this right
perturbation convention. The `Exp`, `Log`, plus, and minus Jacobian helpers are
central finite-difference implementations in tangent coordinates. They are
therefore convention-compatible with the micro Lie theory API, but they are not
claimed to be closed-form analytic `SE_2(3)` Jacobians.

One implementation detail is intentionally different: the current benchmark
filter update injects corrections on the left,

```text
X_corrected = Exp(delta_xi) @ X_predicted
```

instead of using `plus_right(X, delta_xi)`. This preserves the existing InEKF
benchmark behavior. In short: the reusable plus/minus/Jacobian utilities are
right-perturbation utilities, while the filter's correction injection is a
left-injection step.

## Nominal State

The InEKF nominal matrix state is represented as:

```text
X =
[ R  v  p
  0  1  0
  0  0  1 ]
```

where:

- `R`: rotation matrix in `SO(3)`
- `v`: velocity in `R^3`
- `p`: position in `R^3`

The 15D filter additionally tracks:

- `bg`: gyro bias
- `ba`: accelerometer bias

## Prediction

Prediction uses bias-corrected IMU inputs:

```text
omega_corrected = omega - bg
acc_corrected = acc - ba
```

The rotation, velocity, position, and covariance propagation call the shared
Lie group utilities where applicable. Gravity direction, frame convention, and
time-step handling are kept consistent with the existing benchmark settings.

## Update

Measurement updates compute the innovation in the tangent-error convention used
by the active InEKF implementation. The correction has the form:

```text
delta = K @ innovation
delta_xi = delta[0:9]
X_corrected = Exp(delta_xi) @ X_predicted
```

This is the left-injection convention described above. It should not be read as
a call to the right plus operator, whose definition is `X @ Exp(delta_xi)`.

For the 15D filter, bias corrections are then applied as:

```text
bg = bg + delta[9:12]
ba = ba + delta[12:15]
```

Covariance updates use Joseph-form Kalman updates through the shared filter
math helper, followed by explicit symmetrization or stabilization.

## Run InEKF Benchmark

The InEKF benchmark entry point is:

```bash
python3 benchmarks/invariant_ekf_kaist_vio_benchmark.py --max-steps 500
```

By default the script reads `config/compare.yaml`, uses the dataset paths in
that file, and writes outputs under `outputs/benchmarks/`.

To include an external C++ `invariant-ekf` runner, build the runner locally and
pass its path:

```bash
cmake -S compare_repos/invariant-ekf/inekf -B compare_repos/invariant-ekf/inekf/build -DBUILD_TESTS=OFF
cmake --build compare_repos/invariant-ekf/inekf/build --target kaist_vio_runner --parallel

python3 benchmarks/invariant_ekf_kaist_vio_benchmark.py \
  --runner compare_repos/invariant-ekf/inekf/build/bin/kaist_vio_runner
```

Dataset setup examples and dataset-specific notes live in `datasets/README.md`.

## Tests

Run the Lie group utility tests:

```bash
python3 tests/test_lie_group_utils.py
```

The tests cover:

- `SO(3)` Exp/Log/Hat/Vee round trips
- `SE_2(3)` Exp/Log/plus/minus/inverse round trips
- Jacobian helper shapes
- covariance symmetrization

## References

- Joan Sola, Jeremie Deray, Dinesh Atchuthan, "A micro Lie theory for state estimation in robotics"
- `manif`: https://github.com/artivis/manif
- `invariant-ekf`: https://github.com/RossHartley/invariant-ekf

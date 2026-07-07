# InEKF State Estimation Benchmark

이 레포지토리는 IMU/GNSS 상태 추정을 위한 Invariant Extended Kalman
Filter, 즉 InEKF 구현에 집중한 Python/NumPy 프로젝트입니다.

기존 benchmark 구조에서 InEKF 실행에 필요한 데이터 어댑터, trajectory
출력, 시각화 흐름은 유지했습니다. PF, KF, EKF, UKF처럼 InEKF와 직접
관련 없는 필터 계열은 이 README의 범위에서 제외했습니다.

## InEKF 범위

주요 InEKF 파일은 다음과 같습니다.

- `filters/invariant_kalman_filter.py`
- `filters/invariant_kalman_filter_15D.py`
- `models/invariant_inekf.py`
- `models/lie_group_utils.py`

9D InEKF error state는 다음 순서를 사용합니다.

```text
[delta_phi, delta_v, delta_p]
```

15D InEKF error state는 다음 순서를 사용합니다.

```text
[delta_phi, delta_v, delta_p, delta_bg, delta_ba]
```

공분산 행렬 `P`는 nominal matrix state 자체가 아니라 tangent error
vector 위에 정의됩니다.

## Lie Group 유틸리티

Lie group 관련 연산은 `models/lie_group_utils.py`에 모아두었습니다.

구현된 `SO(3)` 연산은 다음과 같습니다.

- `hat_so3`, `vee_so3`
- `exp_so3`, `log_so3`
- `left_jacobian_so3`, `right_jacobian_so3`
- `left_jacobian_inv_so3`, `right_jacobian_inv_so3`

구현된 `SE_2(3)` 연산은 다음과 같습니다.

- `hat_se23`, `vee_se23`
- `exp_se23`, `log_se23`
- `adjoint_se23`
- `compose`, `inverse`
- `plus_right`, `minus_right`
- Jacobian helper 함수들
- `symmetrize_covariance`

`SE_2(3)` tangent vector는 다음 순서를 사용합니다.

```text
[phi, rho_v, rho_p]
```

## Micro Lie Theory Convention 확인

`models/lie_group_utils.py`의 핵심 연산들은 Sola, Deray, Atchuthan의
논문 "A micro Lie theory for state estimation in robotics"에서 사용하는
표기와 맞도록 구성했습니다.

- `hat_so3`, `vee_so3`, `exp_so3`, `log_so3`는 표준 `SO(3)` skew map,
  Rodrigues exponential, logarithm을 사용합니다.
- `left_jacobian_so3`, `right_jacobian_so3`,
  `left_jacobian_inv_so3`, `right_jacobian_inv_so3`는 표준 closed-form
  `SO(3)` Jacobian 수식을 사용합니다.
- `hat_se23`, `vee_se23`, `exp_se23`, `log_se23`는 아래의 embedded
  `SE_2(3)` matrix state를 사용합니다.

```text
X =
[ R  v  p
  0  1  0
  0  0  1 ]
```

- `adjoint_se23`는 같은 tangent 순서 `[phi, rho_v, rho_p]`에 대해
  `SE_2(3)` adjoint를 구현합니다. 회전 block은 `R`, velocity/position
  cross block은 각각 `[v]x R`, `[p]x R`입니다.

이 레포지토리의 plus/minus 유틸리티는 right perturbation 기준입니다.

```text
plus_right(X, tau)  = X @ Exp(tau)
minus_right(Y, X)   = Log(X^{-1} @ Y)
```

inverse와 composition Jacobian helper도 이 right perturbation convention을
기준으로 작성되어 있습니다. 단, `Exp`, `Log`, plus, minus에 대한 일부
Jacobian helper는 tangent coordinate에서 central finite difference로
계산합니다. 따라서 Micro Lie Theory API와 convention은 맞지만, 모든
`SE_2(3)` Jacobian이 closed-form analytic 구현이라고 주장하는 구조는
아닙니다.

중요한 구현상 차이가 하나 있습니다. 현재 benchmark filter update는
correction을 왼쪽에서 주입합니다.

```text
X_corrected = Exp(delta_xi) @ X_predicted
```

즉, `plus_right(X, delta_xi) = X @ Exp(delta_xi)`와는 다릅니다.
정리하면 다음과 같습니다.

- reusable plus/minus/Jacobian 유틸리티: right perturbation 기준
- 현재 filter correction injection: 기존 benchmark 호환을 위한 left injection

## Nominal State

InEKF nominal matrix state는 다음과 같이 표현합니다.

```text
X =
[ R  v  p
  0  1  0
  0  0  1 ]
```

각 항의 의미는 다음과 같습니다.

- `R`: `SO(3)` 회전 행렬
- `v`: `R^3` velocity
- `p`: `R^3` position

15D filter는 추가로 다음 bias를 추정합니다.

- `bg`: gyro bias
- `ba`: accelerometer bias

## Prediction

Prediction 단계에서는 bias가 보정된 IMU 입력을 사용합니다.

```text
omega_corrected = omega - bg
acc_corrected = acc - ba
```

회전, 속도, 위치, 공분산 propagation은 가능한 곳에서 공통 Lie group
유틸리티를 호출합니다. 중력 방향, frame convention, timestep 처리는 기존
benchmark 설정과 일관되게 유지했습니다.

## Update

Measurement update는 현재 InEKF 구현에서 사용하는 tangent-error convention에
맞춰 innovation을 계산합니다. correction은 다음 형태입니다.

```text
delta = K @ innovation
delta_xi = delta[0:9]
X_corrected = Exp(delta_xi) @ X_predicted
```

위 식은 앞에서 설명한 left injection입니다. right plus operator인
`X @ Exp(delta_xi)`와 혼동하면 안 됩니다.

15D filter에서는 bias correction을 다음처럼 적용합니다.

```text
bg = bg + delta[9:12]
ba = ba + delta[12:15]
```

공분산 update는 공통 filter math helper의 Joseph-form Kalman update를
사용하고, 이후 명시적으로 symmetrization 또는 stabilization을 적용합니다.

## InEKF Benchmark 실행

InEKF benchmark entry point는 다음과 같습니다.

```bash
python3 benchmarks/invariant_ekf_kaist_vio_benchmark.py --max-steps 500
```

기본적으로 script는 `config/compare.yaml`을 읽고, 해당 파일에 적힌 dataset
경로를 사용하며, 결과는 `outputs/benchmarks/` 아래에 저장합니다.

외부 C++ `invariant-ekf` runner까지 같이 비교하려면 runner를 로컬에서 빌드한
뒤 경로를 넘기면 됩니다.

```bash
cmake -S compare_repos/invariant-ekf/inekf -B compare_repos/invariant-ekf/inekf/build -DBUILD_TESTS=OFF
cmake --build compare_repos/invariant-ekf/inekf/build --target kaist_vio_runner --parallel

python3 benchmarks/invariant_ekf_kaist_vio_benchmark.py \
  --runner compare_repos/invariant-ekf/inekf/build/bin/kaist_vio_runner
```

Dataset 설정 예시와 dataset별 메모는 `datasets/README.md`에 있습니다.

## 테스트

Lie group utility 테스트는 다음 명령으로 실행합니다.

```bash
python3 tests/test_lie_group_utils.py
```

현재 테스트는 다음을 확인합니다.

- `SO(3)` Exp/Log/Hat/Vee round trip
- `SO(3)` left/right Jacobian inverse 관계
- `SE_2(3)` Exp/Log/plus/minus/inverse round trip
- `SE_2(3)` adjoint identity
- Jacobian helper shape
- covariance symmetrization

## References

- Joan Sola, Jeremie Deray, Dinesh Atchuthan, "A micro Lie theory for state estimation in robotics"
- `manif`: https://github.com/artivis/manif
- `invariant-ekf`: https://github.com/RossHartley/invariant-ekf

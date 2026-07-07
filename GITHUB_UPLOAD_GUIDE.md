# GitHub Upload Guide

This guide describes how to upload this InEKF-only working copy to GitHub.

Current project state:

- The directory is not initialized as a Git repository yet.
- The repository has been reduced to InEKF-related code, configuration, tests, and small InEKF result artifacts.
- Raw datasets are ignored and should not be committed.
- Current result videos are small enough to commit normally.

## 1. Pre-Upload Checklist

Run these checks before creating the first commit:

```powershell
cd <project-root>

& '<python.exe>' tests\test_lie_group_utils.py

& '<python.exe>' -c "import numpy as np; from filters.invariant_kalman_filter import InvariantKalmanFilter; from filters.invariant_kalman_filter_15D import InvariantKalmanFilter15D; u=np.array([0.1,0,0,0,0,0.01]); z=np.array([0.01,0,0]); f=InvariantKalmanFilter(); g=InvariantKalmanFilter15D(); print(f.step(u,z,0.1).shape, g.step(u,z,0.1).shape, np.allclose(f.P,f.P.T), np.allclose(g.P,g.P.T))"
```

Expected outputs:

```text
lie_group_utils tests passed
(6,) (6,) True True
```

Also confirm that unrelated filter keywords do not appear in text sources:

```powershell
Get-ChildItem -Recurse -File -Include *.py,*.md,*.yaml,*.yml,*.html,Dockerfile,.gitmodules,*.txt |
  Select-String -Pattern 'FilterPy','Stone Soup','stonesoup','filterpy','Particle','PF','UKF','DRIFT','drift','extended_kalman_filter','unscented_kalman_filter','particle_filter','constant_velocity','sigma_points','resampling'
```

Expected output: no matches.

## 2. Files That Should Be Committed

Core InEKF code:

```text
models/lie_group_utils.py
models/invariant_inekf.py
filters/invariant_kalman_filter.py
filters/invariant_kalman_filter_15D.py
```

Benchmark and configuration:

```text
benchmarks/invariant_ekf_kaist_vio_benchmark.py
config/compare.yaml
utils/
requirements.txt
Dockerfile
```

Documentation and tests:

```text
README.md
GITHUB_UPLOAD_GUIDE.md
tests/test_lie_group_utils.py
index.html
```

Small result artifacts may be committed:

```text
outputs/benchmarks/invariant_ekf/
outputs/benchmarks/summary/report/*_inekf.png
```

## 3. Files That Should Not Be Committed

Do not commit raw datasets or generated caches:

```text
datasets/
__pycache__/
*.pyc
.DS_Store
```

These are already covered by `.gitignore`.

If future result videos become large, either remove them from Git or use Git LFS:

```powershell
git lfs install
git lfs track "*.mp4"
git add .gitattributes
```

For the current files, Git LFS is not required because the MP4 files are small.

## 4. Create a New GitHub Repository

On GitHub:

1. Create a new empty repository.
2. Recommended name: `InEKF` or `LieGroup-InEKF`.
3. Do not initialize it with README, license, or `.gitignore` because this folder already has those project files.
4. Copy the repository URL.

Example URL:

```text
https://github.com/<your-user-or-org>/InEKF.git
```

## 5. Initialize Local Git Repository

Run:

```powershell
cd <project-root>

git init
git branch -M main
git status --short
```

Review the file list carefully. You should see InEKF code, docs, tests, config,
and small result artifacts. You should not see raw dataset files.

## 6. Commit

Run:

```powershell
git add .
git status --short
git commit -m "Refactor benchmark to InEKF-only Lie group implementation"
```

Suggested first commit summary:

```text
Refactor benchmark to InEKF-only Lie group implementation
```

Suggested commit details:

```text
- Add SO(3) and SE_2(3) Lie group utility functions
- Refactor 9D and 15D InEKF filters to call shared Lie group utilities
- Remove unrelated KF/EKF/UKF/PF benchmark code
- Keep InEKF-only benchmark, config, docs, and tests
```

## 7. Connect Remote and Push

Replace the URL with your repository URL:

```powershell
git remote add origin https://github.com/<your-user-or-org>/InEKF.git
git push -u origin main
```

If GitHub asks for authentication, use GitHub CLI, browser login, or a personal access token depending on your local Git setup.

## 8. Optional: Enable GitHub Pages

This repository contains `index.html`, so GitHub Pages can show the InEKF result page.

On GitHub:

1. Go to repository `Settings`.
2. Open `Pages`.
3. Source: `Deploy from a branch`.
4. Branch: `main`.
5. Folder: `/root`.
6. Save.

The page will be available at:

```text
https://<your-user-or-org>.github.io/<repo-name>/
```

## 9. After Upload Verification

After pushing, check:

```powershell
git status
git log --oneline -5
```

Expected:

```text
nothing to commit, working tree clean
```

Then verify on GitHub:

- `models/lie_group_utils.py` is visible.
- `filters/invariant_kalman_filter.py` and `filters/invariant_kalman_filter_15D.py` are visible.
- `README.md` describes only InEKF.
- No unrelated filter files are present under `filters/`.

## 10. Recommended Repository Description

Use this short GitHub description:

```text
Python/NumPy InEKF benchmark with SO(3) and SE_2(3) Lie group utilities.
```

Recommended topics:

```text
inekf
lie-groups
state-estimation
robotics
imu
gnss
so3
se23
numpy
```

FROM ros:humble-ros-base-jammy

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    MPLBACKEND=Agg \
    MPLCONFIGDIR=/tmp/matplotlib \
    VIRTUAL_ENV=/opt/venv \
    PYTHONPATH=/app \
    INVARIANT_EKF_RUNNER=/app/compare_repos/invariant-ekf/inekf/build/bin/kaist_vio_runner \
    PATH="/opt/venv/bin:$PATH"

WORKDIR /app

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        python3-dev \
        python3-venv \
        ffmpeg \
        libboost-test-dev \
        libeigen3-dev \
        libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /app/requirements.txt

RUN python3 -m venv "${VIRTUAL_ENV}" \
    && "${VIRTUAL_ENV}/bin/pip" install --no-cache-dir --upgrade pip \
    && "${VIRTUAL_ENV}/bin/pip" install --no-cache-dir -r /app/requirements.txt

COPY . /app

CMD ["python3", "benchmarks/invariant_ekf_kaist_vio_benchmark.py", "--help"]

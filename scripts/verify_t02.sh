#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash
set -u
mkdir -p artifacts/T02
timeout 180 cmake -S transplant_ws/src/path_searching -B transplant_ws/build-t02/path_searching \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug 2>&1 | tee artifacts/T02/verify-configure.log
timeout 180 cmake --build transplant_ws/build-t02/path_searching --parallel 2 \
  2>&1 | tee artifacts/T02/verify-build.log
ctest --test-dir transplant_ws/build-t02/path_searching --output-on-failure --timeout 30 \
  2>&1 | tee artifacts/T02/verify-tests.log
transplant_ws/build-t02/path_searching/smoother_simplification_test \
  2>&1 | tee artifacts/T02/integration-observations.log
transplant_ws/build-t02/path_searching/path_simplifier_test --export artifacts/T02/observations.csv \
  2>&1 | tee artifacts/T02/observations.log
python3 scripts/render_t02_observations.py artifacts/T02/observations.csv artifacts/T02/observations.svg \
  2>&1 | tee artifacts/T02/render.log

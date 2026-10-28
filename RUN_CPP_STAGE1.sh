#!/usr/bin/env bash
set -euo pipefail
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${HOME}/ur3_variant_offline_build"

cmake -S "${PACKAGE_DIR}" -B "${BUILD_DIR}" \
  -DUR3_VARIANT_STANDALONE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "${BUILD_DIR}" -j 2
ctest --test-dir "${BUILD_DIR}" --output-on-failure

"${BUILD_DIR}/variant_offline_generator" \
  "${PACKAGE_DIR}/config/variant_motion_parameters.csv" \
  "${PACKAGE_DIR}/trajectories/synthetic" fixed
"${BUILD_DIR}/variant_offline_generator" \
  "${PACKAGE_DIR}/config/variant_motion_parameters.csv" \
  "${PACKAGE_DIR}/trajectories/synthetic" speed

echo
echo "READY: read the ANALYSIS_ONLY CSVs under trajectories/synthetic/."
echo "DO NOT send these fixture-relative analysis CSVs to MoveIt or the robot."

#!/usr/bin/env bash
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${HOME}/ur3_variant_offline_build"
cmake -S "$DIR" -B "$BUILD" -DUR3_VARIANT_STANDALONE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD" -j 2
ctest --test-dir "$BUILD" --output-on-failure
"$BUILD/plate_layout_preview" \
  "$DIR/config/pickup_plate.csv" \
  "$DIR/config/placement_plate.csv" \
  "$DIR/config/variant_motion_parameters.csv" \
  "$DIR/trajectories/plate_preview" fixed
"$BUILD/plate_layout_preview" \
  "$DIR/config/pickup_plate.csv" \
  "$DIR/config/placement_plate.csv" \
  "$DIR/config/variant_motion_parameters.csv" \
  "$DIR/trajectories/plate_preview" speed
printf '\nStage 2 is a plate-local analysis preview only. NEVER send these CSVs to the robot.\n'

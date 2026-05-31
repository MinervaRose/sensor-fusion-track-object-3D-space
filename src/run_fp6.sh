#!/usr/bin/env bash
set -euo pipefail

proj="$(pwd)"
build="$proj/build"
mkdir -p "$build" results

# Full detector and descriptor sets from previous lessons
DETS=(HARRIS SHITOMASI FAST BRISK ORB AKAZE SIFT)
DESCS=(BRISK BRIEF ORB FREAK AKAZE SIFT)

rebuild() {
  rm -rf "$build" && mkdir -p "$build"
  cd "$build"
  cmake ..
  make -j2
  cd "$proj"
}

run_combo() { # det desc
  echo "=== Running $1 + $2 ==="
  cd "$build"
  if ./3D_object_tracking "$1" "$2"; then
    cd "$proj"
    cp "$build/ttc_lidar_log.csv"  "results/ttc_lidar_${1}_${2}.csv"
    cp "$build/ttc_camera_log.csv" "results/ttc_camera_${1}_${2}.csv"
  else
    cd "$proj"
    echo "!!! Program crashed for $1 + $2 (continuing)"
  fi
}

rebuild

for det in "${DETS[@]}"; do
  for desc in "${DESCS[@]}"; do
    # Invalid pair: AKAZE descriptor must go with AKAZE detector
    if [[ "$desc" == "AKAZE" && "$det" != "AKAZE" ]]; then
      echo "Skipping $det + $desc (invalid pair)"
      continue
    fi
    run_combo "$det" "$desc"
  done
done

echo "All runs done. CSVs in ./results/"


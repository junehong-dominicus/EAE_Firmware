#!/usr/bin/env bash
# Configures, builds, and runs the unit tests for EAE_Firmware.
# Works on Linux, and under MSYS2/MinGW on Windows.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel

ctest --test-dir build --output-on-failure

echo
echo "Run the simulation with, e.g.:"
echo "  ./build/eae_firmware --setpoint 55 --kp 5 --ki 2 --kd 0.5"

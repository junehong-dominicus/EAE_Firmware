# EAE_Firmware

Optional Section 7.1 ("Firmware") submission for the EAE Electrical and
Controls Challenge. Extends the Section 7 cooling-loop control logic into a
small C++17 firmware simulation, restructured as a state machine and driven
over a simulated CAN bus.

## What it demonstrates

| 7.1 requirement | Where |
|---|---|
| Simulate sending/receiving data over CANBUS | `include/can_bus.hpp`, `src/can_bus.cpp` (in-process CAN queue) + `include/can_codec.hpp`, `src/can_codec.cpp` (frame encode/decode) — used both directions in `src/main.cpp` |
| Use a PID loop | `include/pid_controller.hpp`, `src/pid_controller.cpp` — drives fan speed (0-100%) against a coolant-temperature setpoint |
| Create a state machine | `include/cooling_state_machine.hpp`, `src/cooling_state_machine.cpp` — `Init -> Idle -> Running -> Fault`, same safety intent as the Section 7 submission (latched over-temp/sensor faults, low-level pump inhibit) |
| Pass command line arguments for setpoints | `src/main.cpp` parses `--setpoint`, `--kp`, `--ki`, `--kd`, `--dt` |
| External dependencies managed by CMake | `tests/CMakeLists.txt` fetches GoogleTest via CMake `FetchContent` at configure time - nothing vendored in this repo |
| Build on Linux / MSYS2 | `build.sh`; also builds directly with `cmake`/`gcc` under MSYS2 on Windows |
| Use GTest to implement unit testing | `tests/test_can_bus.cpp`, `tests/test_can_codec.cpp`, `tests/test_pid_controller.cpp`, `tests/test_cooling_state_machine.cpp` |
| Do not ship dependencies; static link / use your build system | `CMakeLists.txt` passes `-static`; GTest is fetched by CMake, not committed to the repo |

## Build & run

```bash
./build.sh
```

or manually:

```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure

./build/eae_firmware --setpoint 55 --kp 5 --ki 2 --kd 0.5
```

`eae_firmware` runs a simulated sequence of CAN scan cycles (warm-up, PID
fan control, a low coolant level fault, an over-temperature trip, a sensor
fault, and an ignition-off reset) and prints the resulting state, pump, fan
percent, and alarm for each cycle - the "emulated data" required by
Section 7.

## Layout

```
include/   public headers for each component
src/       implementations + main.cpp (the CLI simulation entry point)
tests/     GTest unit tests, one file per component
```

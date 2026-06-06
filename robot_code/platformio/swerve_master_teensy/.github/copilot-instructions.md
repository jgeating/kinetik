Purpose
-------
This file gives concise, actionable guidance for AI coding agents working on the `swerve_master_teensy` firmware. It focuses on the repository's architecture, developer workflows (build / upload / monitor), project-specific conventions, and integration points discovered in the code.

**Big Picture**
- **Platform & goal:** Teensy 4.x microcontroller (PlatformIO `teensy` platform). The project implements swerve-drive control firmware for two main variants: `penny_v3` and `penny_v4`.
- **Main runtime variants:** `src/penny_v3.cpp` (v3) and `src/penny_v4.cpp` (v4). Each variant configures different motor stacks and CAN topologies.
- **Core runtime responsibilities:** Read RC inputs (SBUS), plan motion (`Planner`), run an inner high-rate loop to command steering and drive motors, and a slower outer loop for mode switching, calibration and telemetry.
- **Key components & boundaries:**
  - `include/` – central headers that define data structs and hardware interfaces (e.g. `Swerve.h`, `Kinematics.h`, `Planner.h`, `Constants.h`).
  - `src/` – variant entrypoints and implementations (main programs live here: `penny_v3.cpp`, `penny_v4.cpp`, plus `Swerve.cpp`, `Kinematics.cpp`, etc.).
  - `penny/` – robot-specific module implementations (e.g. `Drive.h/.cpp`, `Steer.h/.cpp`, `Pads.h/.cpp`).
  - `lib/` – bundled third-party libraries (Adafruit IMU, FlexCAN_T4, etc.).

**Runtime flow (high-level)**
- Boot -> Serial & CAN init -> instantiate `Planner`, motor/steer objects -> `loop()` runs two nested rates:
  - Inner loop: target microsecond period `LoopTiming.tInner` (tight control, commands motors, critical timing, avoid blocking I/O).
  - Outer loop: `LoopTiming.tOuter` (lower-frequency tasks: mode selection, calibration, safety checks, telemetry).
- `Planner` is the canonical source of desired wheel angles and speeds used by motor drivers.

**Hardware & integration points**
- CAN: `FlexCAN_T4` is used. In v4, code creates separate `frontCanBus` and `backCanBus` instances. In v3, a single `motors::canBus1` is used. Look at `include/Constants.h` and `src/penny_v4.cpp`.
- Motor drivers: `ODrive`, `Vesc`, and `RMD_M6` wrapper classes implement drive/steer interfaces. These are the main integration points to external motor firmware.
- IMU: Adafruit BNO055 shows up in v3 (`Adafruit_BNO055`); `shared/imu.h` and `lib/Adafruit_BNO055-master` are relevant.
- RC input: `SbusReceiver`—check `src/SbusReceiver.cpp` and usages in `penny_*.cpp`.
- Sensors: IR homing sensors wired to pins in `RobotState::irPin[]` used during homing.

**Important files to reference (examples)**
- Entry points: `src/penny_v3.cpp`, `src/penny_v4.cpp`
- High-level control: `include/Swerve.h`, `include/Kinematics.h`, `include/Planner.h`, `src/Planner.cpp`
- Motor interfaces: `include/ODrive.h`, `include/Vesc.h`, `include/RMD_M6.h`
- Config: `include/Constants.h`, `platformio.ini`
- Utilities: `shared/utils.h`, `shared/LowPassFilter.cpp` (note: `.cpp` is included directly in some files)

**Build / upload / monitor (concrete commands)**
- Build v4: `pio run -e penny_v4`
- Upload v4: `pio run -e penny_v4 -t upload` (PlatformIO will use `upload_protocol = teensy-gui` by default as set in `platformio.ini`)
- Serial monitor (uses monitor_speed from `platformio.ini`):
```bash
# open PlatformIO monitor using configured env (recommended):
pio device monitor -e penny_v4
# or explicitly set port and baud (macOS example):
pio device monitor -p /dev/tty.usbmodemXXXX -b 460800
```
- Note: `platformio.ini` defines `[common]` with `monitor_speed = 460800`, `platform = teensy`, and `board = teensy41`.
- The repository contains an environment `env:penny_v4`. There is a commented `env:penny_v3` stanza. Verify `build_src_filter` before switching variants — some filters currently exclude the main `penny_*.cpp` files.

**Project-specific conventions & gotchas**
- Timing-critical code uses microsecond timing (`micros()`) and `delayMicroseconds()`. Avoid inserting expensive Serial prints inside the inner loop.
- Safety checks are explicit: `sbusReceiver.rcLost()` and `sbusReceiver.getRedSwitch()` gate motor commands and often trigger `planner->eStop()`; maintain these checks when refactoring motor command paths.
- Homing: IR sensors and the `doneHoming` flag coordinate calibration. IR pins are defined in `RobotState::irPin[]` inside `include/Swerve.h`.
- Global singletons & dynamic allocation: code uses global namespace `motors::`, global pointers (`Planner *planner;`), and `new` for arrays. Be careful when changing initialization order — CAN must be started before creating CAN-dependent motor objects.
- Mixed inclusion: some source files include `.cpp` directly (e.g., `shared/LowPassFilter.cpp` in `penny_v3.cpp`) — follow existing pattern rather than moving to strictly header-only unless you update build settings.

**When changing loop timing or adding prints**
- Validate inner-loop time (`LoopTiming.tInner`) after changes; the code sets `loopTiming.behind` and toggles LED on pin `13` when the controller can't keep up.
- Use the slower outer loop for debugging prints and telemetry to avoid jitter.

**What to look for when editing motor control & CAN code**
- Inspect `include/Swerve.h` for motor arrays and CAN packet structures (`SwerveCAN`).
- Check `motors::steer[]` and `motors::drive[]` initialization to ensure CAN IDs and bus objects are used consistently.

**Testing / validation tips**
- Start with `pio device monitor` and watch for CAN/motor initialization messages printed at startup.
- For safety, ensure `rcLost()` paths are enforced locally when adding new motor command code.

Next steps
- If you'd like, I can (pick one):
  - Open and summarize `Planner` and `Drive/Steer` implementations in a short doc.
  - Add developer scripts (Makefile or simple `scripts/` directory) to standardize build/upload/monitor commands.

Please review this file and tell me any unclear or missing areas to iterate.

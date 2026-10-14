# Magic Wand Controller prototype

Single-device M5Stack Atom Matrix wand prototype with a Windows BLE HID gamepad firmware path and the existing p5.js visualizer. No mouse, keyboard, or XInput emulation is implemented; this is a standard BLE HID gamepad.

## Architecture

- `atommatrixm5/ble_hid_proof/ble_hid_proof.ino` is stage 1: it advertises a test BLE HID gamepad, sweeps two axes, and maps the Atom button to Button 1. Pair and verify this before flashing the motion-driven firmware.
- `atommatrixm5/atommatrixm5.ino` is stage 2: reads the onboard IMU and debounced Atom button through M5Unified, drives the BLE gamepad, and optionally streams newline-delimited JSON over USB serial.
- `atommatrixm5/ble_gamepad.h` owns the BLE HID report, pairing and reconnect behavior. `atommatrixm5/wand_motion.h` is a firmware port of the existing `web/motion.js` quaternion filter and calibration tuning.
- `web/serial.js` owns the one Web Serial connection and validates/dispatches frames.
- `web/buttons.js` exposes the button state plus `magicwand:button` press/release events for future game code.
- `web/motion.js` estimates attitude for the visualizer with quaternion gyro integration, accelerometer tilt correction, gyro calibration and adaptive bias. The firmware uses the same equations/tuning in `wand_motion.h` to drive HID when the browser is closed.
- `web/movement.js` filters gravity-compensated acceleration and recognizes thrust gestures. It does not estimate or apply position.
- `web/sketch.js` rotates the wand model from the IMU orientation and displays the world/local axes with a fixed initial camera. IMU-derived translation is disabled.

Future game code can subscribe with `window.addEventListener("magicwand:button", (event) => { ... })`; gesture events use the parallel `magicwand:gesture` event.

## Board, IMU and libraries

The firmware target is Atom Matrix and uses M5Unified rather than a sensor-specific driver. M5Stack documents MPU6886 on original Atom Matrix and BMI270 on Atom Matrix v1.1; firmware now prints the detected IMU and a revision estimate at boot. The local setup has M5Unified 0.2.25, M5GFX 0.2.32, and ESP32 board support 3.3.9. No serial device was connected during this check, so the actual unit and its IMU could not be probed here.

Required firmware: M5Stack ESP32 board support (`m5stack:esp32`), M5Unified, and the BLE/HID classes bundled in the board core. The inspected machine has M5Stack core 3.3.9, its bundled BLE library 3.3.9 (including `BLEHIDDevice` and the `Server_Gamepad` example), M5Unified 0.2.25, and M5GFX 0.2.32. The gamepad sketch uses that bundled API directly; no NimBLE-Arduino or third-party BLE gamepad library is added. Required visualizer: Chrome/Edge Web Serial and p5.js 1.9.4 from the CDN in `web/index.html`. The browser also runs if the BLE gamepad has already paired; the firmware HID does not depend on the browser. No StickC Plus 2 or extra npm runtime library is used.

## Coordinates and startup

The p5 scene uses +X right, +Y down, and +Z toward the viewer; the green −Y gizmo points upward on screen. Wand-local axes are +X right, +Y toward the tip, and +Z toward the LED/button face. The first valid accelerometer sample identifies the dominant gravity axis as the wand axis and captures the initial attitude as neutral.

At startup hold the assembled wand with its tip straight up and the Atom LED/button face toward you. Keep it still while gyro calibration counts to 100 stable samples (about 2.5 seconds including the stability window). If it does not progress, keep the device still and check that live sensor frames are arriving. “Reset / Recenter” records the current attitude as neutral, clears gesture offsets, and restores the camera. “Detect wand axis” is a manual correction; use it while the tip points straight up.

## Current motion mode

The browser calculates and displays orientation and rotates the model on all three axes. IMU-derived translation is disabled, so the model cannot slide across the scene. A short positive acceleration along the wand tip axis triggers a `magicwand:gesture` event and a visible THRUST state. The UI reports only THRUST or IDLE for this gesture detector.

The 6-axis IMU has no absolute position reference, and yaw can drift without a magnetometer. This prototype maps orientation to model rotation but does not map IMU acceleration to model position.

## Build, upload and run

1. In Arduino IDE select board package **M5Stack > M5Atom** (FQBN `m5stack:esp32:m5stack_atom`), choose **Partition Scheme > Huge APP (3MB No OTA/1MB SPIFFS)** if the default sketch space is too small, and select the Atom Matrix COM port.
2. **Stage 1—prove Windows BLE gamepad:** open `atommatrixm5/ble_hid_proof/ble_hid_proof.ino`, compile and upload. In Windows open **Settings > Bluetooth & devices > Add device > Bluetooth** and pair **Magic Wand HID Test**. Run `joy.cpl` (Win+R) or a gamepad tester: X/Y should sweep automatically; press/release the Atom button and verify Button 1 toggles. Do not continue to Stage 2 until pairing and both reports work on the physical board.
3. **Stage 2—enable real motion:** open `atommatrixm5/atommatrixm5.ino`, compile and upload. Pair **Magic Wand Matrix** from Windows. Hold the wand in the documented startup pose and still while serial reports calibration progress to 100/100. Default mapping is Roll→X, Pitch→Y and Yaw→Z. Change `kAxisXSource`, `kAxisYSource`, `kAxisZSource`, signs, or full-scale degrees near the top of the sketch to tune the mapping.
4. Serial Monitor runs at 115200 baud. JSON sensor telemetry remains at 50 Hz for the browser visualizer; the once-per-second HID summary is off by default, and `D` toggles it. Send `T` to run the deterministic HID axis/button test (12 stages, 2 seconds each) and `N` to return to motion mode; the test also ends automatically. Sensitivity can be changed with `S 1.5` (valid range 0.1–5.0); `S 1.0` restores default. Send `R` to recenter, `C` to recalibrate, and `?` for help. Holding the Atom button for 1.5 seconds also recenters while the button remains mapped to gamepad Button 1. Close Serial Monitor before opening Web Serial in the browser because Windows allows only one serial-port client at a time. BLE HID works without the browser.
5. For the visualizer, run `npm start` and open `http://localhost:3000` in Chrome or Edge, then connect the Atom Matrix over USB Serial. The server uses Node's built-in HTTP module and needs no npm package download. Reload the page with Ctrl+Shift+R after web edits.

The Codex Windows sandbox enables the `ForceRelocateImages` and `RequireInfo` process mitigations; in that environment `xtensa-esp32-elf-g++` fails at launch with “Illegal System DLL Relocation,” even when called directly with `--version`. This is a host launch restriction, not an Arduino board or sketch configuration issue. Run Arduino IDE or Arduino CLI from a normal Windows session outside the Codex sandbox; do not reinstall or downgrade the ESP32 core to work around it. Using that same installed CLI 1.1.1, M5Stack core 3.3.9, and Atom FQBN, both Stage 1 and Stage 2 compiled successfully outside the sandbox. The compile exposed and fixed one source error: the BLE report descriptor was declared as an incomplete array before `sizeof`; it is now declared with its exact 54-byte length. No BLE pairing or HID report has been physically verified here.

PlatformIO can use `board = m5stack-atom`, `framework = arduino`, and `M5Unified` as a library dependency; no PlatformIO config is included.

## Test checklist

- [ ] Stage 1: compile/upload test sketch; pair `Magic Wand HID Test` in Windows; verify the axes sweep and Atom button toggles Button 1.
- [ ] Stage 2: compile/upload wand sketch; pair `Magic Wand Matrix`; verify it reconnects after Windows disconnect/reconnect and power cycle.
- [ ] Verify firmware-reported IMU matches the physical board revision.
- [ ] Confirm 50 Hz sensor frames and raw accel/gyro response in Serial Monitor/browser.
- [ ] Hold still through initial firmware calibration; verify it reaches 100/100, then check axes remain near center.
- [ ] Rotate around roll, pitch, and yaw; verify the three gamepad axes move, signs match, and sensitivity responds to `S`.
- [ ] Send `R` or hold Atom button for 1.5 seconds; confirm the current orientation becomes axis center.
- [ ] Press and release Atom; verify exactly one Button 1 press/release without repeats.
- [ ] Disconnect from Windows, verify advertising resumes, then reconnect without rebooting.
- [ ] Open the browser visualizer over USB serial and confirm the existing p5.js model still tracks while BLE HID is active.
- [ ] Move the wand linearly; confirm no absolute position is sent. A 6-axis IMU cannot provide reliable absolute translation, and yaw can drift without a magnetometer.

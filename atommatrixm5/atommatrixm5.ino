// Stage 2: Atom Matrix BLE HID gamepad driven by the existing wand motion filter.
#include <M5Unified.h>
#include <stdlib.h>
#include <ctype.h>
#include "wand_motion.h"
#include "ble_gamepad.h"

namespace {
constexpr uint32_t kSamplePeriodMs = 20;      // IMU and HID reports at 50 Hz
constexpr uint32_t kButtonDebounceMs = 25;
constexpr uint32_t kLongPressRecenterMs = 1500;
constexpr uint32_t kSerialBaudRate = 115200;
constexpr bool kHidDiagnosticsEnabledAtStartup = false;
constexpr float kAxisFullScaleDegrees = 90.0f;
enum class AngleAxis : uint8_t { Roll, Pitch, Yaw };
constexpr AngleAxis kAxisXSource = AngleAxis::Roll;
constexpr AngleAxis kAxisYSource = AngleAxis::Pitch;
constexpr AngleAxis kAxisZSource = AngleAxis::Yaw;
constexpr float kAxisXSign = 1.0f;
constexpr float kAxisYSign = 1.0f;
constexpr float kAxisZSign = 1.0f;
constexpr uint32_t kHidTestStageMs = 2000;

struct HidTestStage {
  const char* name;
  int16_t x, y, z;
  bool buttonDown;
};

constexpr HidTestStage kHidTestStages[] = {
  {"X neutral", 0, 0, 0, false}, {"X minimum", -32767, 0, 0, false}, {"X maximum", 32767, 0, 0, false},
  {"Y neutral", 0, 0, 0, false}, {"Y minimum", 0, -32767, 0, false}, {"Y maximum", 0, 32767, 0, false},
  {"Z neutral", 0, 0, 0, false}, {"Z minimum", 0, 0, -32767, false}, {"Z maximum", 0, 0, 32767, false},
  {"Button released", 0, 0, 0, false}, {"Button pressed", 0, 0, 0, true}, {"Button released", 0, 0, 0, false},
};

WandMotion motion;
BleGamepadHID gamepad;
WandImuFrame latestFrame = {0, 0, 1, 0, 0, 0, 0};
bool pressPending = false;
bool pressedEventPending = false;
bool releasedEventPending = false;
bool recenteredDuringHold = false;
bool buttonHoldTracking = false;
bool previousBleConnected = false;
uint32_t buttonDownSinceMs = 0;
uint32_t lastSampleMs = 0;
uint32_t lastReportMs = 0;
uint32_t lastTelemetryMs = 0;
bool physicalButtonSeenInWindow = false;
uint32_t buttonSetReportsInWindow = 0;
uint8_t lastReportButtonByte = 0xFF;
bool lastReportAttempted = false;
int16_t latestHidX = 0;
int16_t latestHidY = 0;
int16_t latestHidZ = 0;
float hidSensitivity = 1.0f;
bool hidTestActive = false;
bool hidTestReportLogPending = false;
bool hidDiagnosticsEnabled = kHidDiagnosticsEnabledAtStartup;
uint8_t hidTestStage = 0;
uint32_t hidTestStageStartedMs = 0;
char serialCommand[32] = {};
size_t serialCommandLength = 0;

int16_t angleToGamepadAxis(float angleDegrees, float axisSign) {
  float normalized = angleDegrees * axisSign * hidSensitivity / kAxisFullScaleDegrees;
  if (normalized < -1.0f) normalized = -1.0f;
  if (normalized > 1.0f) normalized = 1.0f;
  return static_cast<int16_t>(lroundf(normalized * 32767.0f));
}

float angleForSource(const WandAngles& angles, AngleAxis source) {
  switch (source) {
    case AngleAxis::Roll: return angles.rollDeg;
    case AngleAxis::Pitch: return angles.pitchDeg;
    case AngleAxis::Yaw: return angles.yawDeg;
  }
  return 0.0f;
}

void printHelp() {
  Serial.println("# Serial commands: R=recenter, C=gyro calibration, S <gain>=sensitivity (0.1..5.0), D=toggle HID summary, T=start HID test, N=return to motion mode");
}

void printHidTestStage() {
  const HidTestStage& stage = kHidTestStages[hidTestStage];
  Serial.printf("# HID TEST stage %u/%u: %s; X=%d Y=%d Z=%d Button1=%u (button byte=0x%02X), hold=%lu ms\n",
                static_cast<unsigned>(hidTestStage + 1),
                static_cast<unsigned>(sizeof(kHidTestStages) / sizeof(kHidTestStages[0])),
                stage.name, stage.x, stage.y, stage.z, stage.buttonDown ? 1U : 0U,
                stage.buttonDown ? BleGamepadHID::kButton1Mask : 0U,
                static_cast<unsigned long>(kHidTestStageMs));
  hidTestReportLogPending = true;
}

void startHidTest(uint32_t now) {
  hidTestActive = true;
  hidTestStage = 0;
  hidTestStageStartedMs = now;
  Serial.println("# HID TEST started; normal motion and physical button are overridden until completion or N command");
  printHidTestStage();
}

void stopHidTest(const char* reason) {
  if (!hidTestActive) {
    Serial.println("# HID TEST is not active; normal motion mode remains active");
    return;
  }
  hidTestActive = false;
  hidTestReportLogPending = false;
  Serial.printf("# HID TEST stopped (%s); returned to normal motion mode\n", reason);
}

void updateHidTest(uint32_t now) {
  if (!hidTestActive || now - hidTestStageStartedMs < kHidTestStageMs) return;
  hidTestStageStartedMs = now;
  hidTestStage++;
  if (hidTestStage >= sizeof(kHidTestStages) / sizeof(kHidTestStages[0])) {
    hidTestActive = false;
    hidTestReportLogPending = false;
    Serial.println("# HID TEST complete; returned to normal motion mode");
    return;
  }
  printHidTestStage();
}

void handleSerialCommand() {
  if (serialCommandLength == 0) return;
  serialCommand[serialCommandLength] = '\0';
  const char command = static_cast<char>(toupper(static_cast<unsigned char>(serialCommand[0])));
  if (command == 'D') {
    hidDiagnosticsEnabled = !hidDiagnosticsEnabled;
    Serial.printf("# Periodic HID summary %s\n", hidDiagnosticsEnabled ? "enabled" : "disabled");
  } else if (command == 'T') {
    startHidTest(millis());
  } else if (command == 'N') {
    stopHidTest("N command");
  } else if (command == 'R') {
    motion.recenter();
    Serial.println("# Recentered gamepad axes to current wand pose");
  } else if (command == 'C') {
    motion.startCalibration();
    Serial.println("# Gyro calibration restarted; hold wand still");
  } else if (command == 'S') {
    char* valueStart = serialCommand + 1;
    while (*valueStart == ' ' || *valueStart == '=') valueStart++;
    char* end = nullptr;
    const float value = strtof(valueStart, &end);
    if (end != valueStart && value >= 0.1f && value <= 5.0f) {
      hidSensitivity = value;
      Serial.printf("# HID sensitivity set to %.2f\n", hidSensitivity);
    } else {
      Serial.println("# Sensitivity must be between 0.1 and 5.0 (example: S 1.5)");
    }
  } else if (command == '?') {
    printHelp();
  } else {
    Serial.println("# Unknown command; send ? for help");
  }
  serialCommandLength = 0;
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      handleSerialCommand();
    } else if (serialCommandLength + 1 < sizeof(serialCommand)) {
      serialCommand[serialCommandLength++] = c;
    } else {
      serialCommandLength = 0;
      Serial.println("# Command too long; ignored");
    }
  }
}

void updatePhysicalButton(uint32_t now) {
  if (M5.BtnA.wasPressed()) {
    pressPending = true;
    pressedEventPending = true;
    buttonDownSinceMs = now;
    buttonHoldTracking = true;
    recenteredDuringHold = false;
  }
  if (M5.BtnA.wasReleased()) {
    releasedEventPending = true;
    buttonHoldTracking = false;
  }

  if (M5.BtnA.isPressed() && buttonHoldTracking && !recenteredDuringHold &&
      now - buttonDownSinceMs >= kLongPressRecenterMs) {
    motion.recenter();
    recenteredDuringHold = true;
    Serial.println("# Long press: recentered gamepad axes");
  }
}

const char* imuName() {
  switch (M5.Imu.getType()) {
    case m5::imu_t::imu_mpu6886: return "MPU6886";
    case m5::imu_t::imu_bmi270: return "BMI270";
    case m5::imu_t::imu_unknown: return "unknown";
    default: return "none";
  }
}

const char* matrixRevisionEstimate() {
  switch (M5.Imu.getType()) {
    case m5::imu_t::imu_mpu6886: return "original Atom Matrix";
    case m5::imu_t::imu_bmi270: return "Atom Matrix v1.1";
    default: return "unknown";
  }
}

void sampleImu(uint32_t now) {
  if (now - lastSampleMs < kSamplePeriodMs) return;
  lastSampleMs = now;
  if (!M5.Imu.update()) return;

  const auto data = M5.Imu.getImuData();
  latestFrame = {data.accel.x, data.accel.y, data.accel.z,
                 data.gyro.x, data.gyro.y, data.gyro.z, now};
  motion.update(latestFrame);
}

void updateBleConnectionState() {
  const bool connected = gamepad.connected();
  if (connected && !previousBleConnected) {
    motion.recenter();
    Serial.println("# BLE connected; gamepad axes recentered");
  }
  previousBleConnected = connected;
}

void sendGamepadReport(uint32_t now) {
  if (now - lastReportMs < kSamplePeriodMs) return;
  lastReportMs = now;
  const WandAngles angles = motion.angles();
  int16_t axisX = motion.ready() ? angleToGamepadAxis(angleForSource(angles, kAxisXSource), kAxisXSign) : 0;
  int16_t axisY = motion.ready() ? angleToGamepadAxis(angleForSource(angles, kAxisYSource), kAxisYSign) : 0;
  int16_t axisZ = motion.ready() ? angleToGamepadAxis(angleForSource(angles, kAxisZSource), kAxisZSign) : 0;
  bool buttonDown = M5.BtnA.isPressed() || pressPending;
  if (hidTestActive) {
    const HidTestStage& stage = kHidTestStages[hidTestStage];
    axisX = stage.x;
    axisY = stage.y;
    axisZ = stage.z;
    buttonDown = stage.buttonDown;
  }
  latestHidX = axisX;
  latestHidY = axisY;
  latestHidZ = axisZ;
  const bool physicalButtonPressed = M5.BtnA.isPressed();
  lastReportAttempted = gamepad.send(axisX, axisY, axisZ, buttonDown, &lastReportButtonByte,
                                     hidTestActive && hidTestReportLogPending);
  if (lastReportAttempted && hidTestActive) hidTestReportLogPending = false;
  physicalButtonSeenInWindow = physicalButtonSeenInWindow || physicalButtonPressed;
  if (lastReportAttempted && (lastReportButtonByte & BleGamepadHID::kButton1Mask)) buttonSetReportsInWindow++;
  pressPending = false;

  if (now - lastTelemetryMs >= kSamplePeriodMs) {
    lastTelemetryMs = now;
    Serial.printf("{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
                  "\"buttonDown\":%s,\"buttonPressed\":%s,\"buttonReleased\":%s,"
                  "\"calibrating\":%s,\"calibrationCount\":%u,\"rollDeg\":%.2f,\"pitchDeg\":%.2f,\"yawDeg\":%.2f,"
                  "\"hidX\":%d,\"hidY\":%d,\"hidZ\":%d,\"bleConnected\":%s}\n",
                  latestFrame.ax, latestFrame.ay, latestFrame.az,
                  latestFrame.gx, latestFrame.gy, latestFrame.gz,
                  buttonDown ? "true" : "false",
                  pressedEventPending ? "true" : "false",
                  releasedEventPending ? "true" : "false",
                  motion.calibrating() ? "true" : "false",
                  motion.calibrationCount(), angles.rollDeg, angles.pitchDeg, angles.yawDeg,
                  axisX, axisY, axisZ, gamepad.connected() ? "true" : "false");
    pressedEventPending = false;
    releasedEventPending = false;
  }
}
} // namespace

void setup() {
  auto config = M5.config();
  M5.begin(config);
  Serial.begin(kSerialBaudRate);
  M5.BtnA.setDebounceThresh(kButtonDebounceMs);

  Serial.printf("# Board target: Atom Matrix | detected IMU: %s | revision estimate: %s | ESP32 core BLE HID gamepad\n",
                imuName(), matrixRevisionEstimate());
  Serial.println("# JSON sensor telemetry remains at 50 Hz for the visualizer; periodic HID summary is off by default (D toggles it)");
  printHelp();
  gamepad.begin("Magic Wand Matrix");
  Serial.println("# BLE HID advertising: Magic Wand Matrix (Gamepad)");
}

void loop() {
  M5.update();
  const uint32_t now = millis();
  updatePhysicalButton(now);
  readSerialCommands();
  updateHidTest(now);
  sampleImu(now);
  updateBleConnectionState();
  sendGamepadReport(now);
  if ((hidDiagnosticsEnabled || hidTestActive) && gamepad.printDiagnosticSummaryIfDue(
          now, M5.BtnA.isPressed(), physicalButtonSeenInWindow,
          buttonSetReportsInWindow, lastReportButtonByte, lastReportAttempted,
          latestHidX, latestHidY, latestHidZ)) {
    physicalButtonSeenInWindow = false;
    buttonSetReportsInWindow = 0;
  }
  delay(1);
  
}

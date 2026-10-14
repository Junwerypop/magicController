// Stage 1: Windows pairing and test HID reports, before enabling IMU mapping.
#include <M5Unified.h>
#include "../ble_gamepad.h"

BleGamepadHID gamepad;
bool pressPending = false;
uint32_t lastReportMs = 0;
uint32_t reportCount = 0;

int16_t triangleAxis(uint32_t phaseMs, uint32_t periodMs) {
  const uint32_t phase = phaseMs % periodMs;
  const uint32_t halfPeriod = periodMs / 2;
  const int32_t value = phase < halfPeriod
      ? -32767 + static_cast<int32_t>((65534LL * phase) / halfPeriod)
      : 32767 - static_cast<int32_t>((65534LL * (phase - halfPeriod)) / halfPeriod);
  return static_cast<int16_t>(value);
}

void setup() {
  auto config = M5.config();
  M5.begin(config);
  Serial.begin(115200);
  M5.BtnA.setDebounceThresh(25);
  gamepad.begin("Magic Wand HID Test");
  Serial.println("# Stage 1: Pair 'Magic Wand HID Test' in Windows Bluetooth settings.");
  Serial.println("# In joy.cpl or a gamepad tester, X/Y should sweep; Atom button should toggle Button 1.");
}

void loop() {
  M5.update();
  pressPending |= M5.BtnA.wasPressed();
  const bool released = M5.BtnA.wasReleased();

  const uint32_t now = millis();
  if (now - lastReportMs >= 20) {
    lastReportMs = now;
    const bool buttonDown = M5.BtnA.isPressed() || pressPending;
    const int16_t x = triangleAxis(now, 4000);
    const int16_t y = triangleAxis(now + 1000, 4000);
    gamepad.send(x, y, 0, buttonDown);
    pressPending = false;

    if (reportCount++ % 50 == 0) {
      Serial.printf("# HID test X=%d Y=%d button=%d connected=%d\n",
                    x, y, buttonDown ? 1 : 0, gamepad.connected() ? 1 : 0);
    }
  }

  if (released) Serial.println("# Atom button released");
  delay(1);
}

#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>
#include <BLESecurity.h>

// Uses the BLEHIDDevice implementation bundled with the installed M5Stack
// ESP32 Arduino core (no third-party gamepad or NimBLE dependency).
class BleGamepadHID {
 public:
  static constexpr uint8_t kInputReportId = 1;
  static constexpr uint8_t kButton1Mask = 0x01;

  void begin(const char* name = "Magic Wand Matrix") {
    BLEDevice::init(name);
    BLESecurity* security = new BLESecurity();
    security->setCapability(ESP_IO_CAP_NONE);
    security->setAuthenticationMode(true, false, true); // bonded Just Works
    BLEDevice::setSecurityCallbacks(new SecurityCallbacks());

    server_ = BLEDevice::createServer();
    server_->setCallbacks(new Callbacks(this));
    hid_ = new BLEHIDDevice(server_);
    hid_->manufacturer()->setValue("Magic Wand Prototype");
    hid_->pnp(0x02, 0x303A, 0x8101, 0x0100);
    hid_->hidInfo(0x00, 0x01);
    hid_->reportMap(const_cast<uint8_t*>(kReportDescriptor), sizeof(kReportDescriptor));
    report_ = hid_->inputReport(kInputReportId);
    report_->setCallbacks(new ReportCallbacks(this));
    hid_->setBatteryLevel(100);
    hid_->startServices();
    Serial.println("# HID input report configured: characteristic 0x2A4D READ|NOTIFY; Report Reference 0x2908={ID 1, Input 1}; Bluedroid CCCD 0x2902 added by inputReport(1)");
    Serial.println("# HID descriptor payload: X/Y/Z signed 16-bit LE + Button 1 bit + 7 padding bits = 7-byte value (Report ID 1 is in 0x2908, not in the value)");

    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->setAppearance(0x03C4); // HID Gamepad
    advertising->addServiceUUID(hid_->hidService()->getUUID());
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
  }

  bool connected() const { return connected_; }

  bool send(int16_t x, int16_t y, int16_t z, bool buttonDown, uint8_t* reportButtonByte = nullptr,
            bool logPayload = false) {
    if (reportButtonByte != nullptr) *reportButtonByte = 0xFF; // No report was formed.
    if (!connected_) { notifyStats_.skippedDisconnected++; return false; }
    if (report_ == nullptr) { notifyStats_.skippedMissingReport++; return false; }
    GamepadReport report = {x, y, z, static_cast<uint8_t>(buttonDown ? kButton1Mask : 0x00)};
    if (logPayload) {
      const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&report);
      Serial.printf("# HID TEST actual_payload len=%u bytes=", static_cast<unsigned>(sizeof(report)));
      for (size_t i = 0; i < sizeof(report); ++i) Serial.printf("%s%02X", i ? " " : "", bytes[i]);
      Serial.println("; setValue() then notify() (local BLE stack status is not proof of host receipt)");
    }
    report_->setValue(reinterpret_cast<uint8_t*>(&report), sizeof(report));
    if (reportButtonByte != nullptr) *reportButtonByte = report.buttons;
    notifyStats_.attempts++;
    report_->notify();
    return true;
  }

  bool printDiagnosticSummaryIfDue(uint32_t now, bool physicalButtonPressed,
                                   bool physicalButtonSeen, uint32_t buttonSetReports,
                                   uint8_t lastReportButtonByte, bool lastReportAttempted,
                                   int16_t x, int16_t y, int16_t z) {
    if (now - lastDiagnosticMs_ < 1000) return false;
    lastDiagnosticMs_ = now;
    Serial.printf("# HID notify/1s ble_connected=%u attempts=%lu success_notify=%lu error_notify_disabled=%lu error_no_client=%lu error_gatt=%lu last_gatt_code=%lu error_no_subscriber=%lu other_status=%lu skipped_disconnected=%lu skipped_missing_report=%lu physical_button_now=%u physical_button_seen=%u report_button_last=%s0x%02X button1_reports=%lu hidX=%d hidY=%d hidZ=%d\n",
                  connected_ ? 1U : 0U,
                  static_cast<unsigned long>(notifyStats_.attempts),
                  static_cast<unsigned long>(notifyStats_.success),
                  static_cast<unsigned long>(notifyStats_.notifyDisabled),
                  static_cast<unsigned long>(notifyStats_.noClient),
                  static_cast<unsigned long>(notifyStats_.gattError),
                  static_cast<unsigned long>(notifyStats_.lastGattCode),
                  static_cast<unsigned long>(notifyStats_.noSubscriber),
                  static_cast<unsigned long>(notifyStats_.other),
                  static_cast<unsigned long>(notifyStats_.skippedDisconnected),
                  static_cast<unsigned long>(notifyStats_.skippedMissingReport),
                  physicalButtonPressed ? 1U : 0U,
                  physicalButtonSeen ? 1U : 0U,
                  lastReportAttempted ? "" : "(no report) ",
                  lastReportButtonByte,
                  static_cast<unsigned long>(buttonSetReports),
                  x, y, z);
    notifyStats_ = {};
    return true;
  }

 private:
  struct NotificationStats {
    uint32_t attempts = 0;
    uint32_t success = 0;
    uint32_t notifyDisabled = 0;
    uint32_t noClient = 0;
    uint32_t gattError = 0;
    uint32_t lastGattCode = 0;
    uint32_t noSubscriber = 0;
    uint32_t other = 0;
    uint32_t skippedDisconnected = 0;
    uint32_t skippedMissingReport = 0;
  };

  // BLE HID (HOGP): the Report ID lives in the Report Reference descriptor (0x2908),
  // not in the characteristic value. The value must be exactly the 7 report bytes.
  struct __attribute__((packed)) GamepadReport {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t buttons;
  };

  class Callbacks : public BLEServerCallbacks {
   public:
    explicit Callbacks(BleGamepadHID* owner) : owner_(owner) {}
    void onConnect(BLEServer*) override { owner_->connected_ = true; }
    void onDisconnect(BLEServer*) override {
      owner_->connected_ = false;
      BLEDevice::startAdvertising();
    }
   private:
    BleGamepadHID* owner_;
  };

  class ReportCallbacks : public BLECharacteristicCallbacks {
   public:
    explicit ReportCallbacks(BleGamepadHID* owner) : owner_(owner) {}
    void onStatus(BLECharacteristic*, Status status, uint32_t code) override {
      switch (status) {
        case SUCCESS_NOTIFY: owner_->notifyStats_.success++; break;
        case ERROR_NOTIFY_DISABLED: owner_->notifyStats_.notifyDisabled++; break;
        case ERROR_NO_CLIENT: owner_->notifyStats_.noClient++; break;
        case ERROR_GATT:
          owner_->notifyStats_.gattError++;
          owner_->notifyStats_.lastGattCode = code;
          break;
        case ERROR_NO_SUBSCRIBER: owner_->notifyStats_.noSubscriber++; break;
        default: owner_->notifyStats_.other++; break;
      }
    }
   private:
    BleGamepadHID* owner_;
  };

  class SecurityCallbacks : public BLESecurityCallbacks {
   public:
    bool onSecurityRequest() override { return true; }
    void onAuthenticationComplete(esp_ble_auth_cmpl_t result) override {
      if (Serial) Serial.printf("# BLE pairing %s (status=%d)\n", result.success ? "complete" : "failed", result.fail_reason);
    }
    uint32_t onPassKeyRequest() override { return 0; }
    void onPassKeyNotify(uint32_t) override {}
    bool onConfirmPIN(uint32_t) override { return true; }
  };

  static_assert(sizeof(GamepadReport) == 7, "HID report value must be 7 bytes (no Report ID)");
  static const uint8_t kReportDescriptor[54];
  BLEServer* server_ = nullptr;
  BLEHIDDevice* hid_ = nullptr;
  BLECharacteristic* report_ = nullptr;
  volatile bool connected_ = false;
  NotificationStats notifyStats_;
  uint32_t lastDiagnosticMs_ = 0;
};

const uint8_t BleGamepadHID::kReportDescriptor[] = {
  0x05, 0x01,       // Usage Page (Generic Desktop)
  0x09, 0x05,       // Usage (Gamepad)
  0xA1, 0x01,       // Collection (Application)
  0x85, 0x01,       //   Report ID (1)
  0x09, 0x01,       //   Usage (Pointer)
  0xA1, 0x00,       //   Collection (Physical)
  0x09, 0x30,       //     Usage (X)
  0x09, 0x31,       //     Usage (Y)
  0x09, 0x32,       //     Usage (Z)
  0x16, 0x01, 0x80, //     Logical Minimum (-32767)
  0x26, 0xFF, 0x7F, //     Logical Maximum (32767)
  0x75, 0x10,       //     Report Size (16)
  0x95, 0x03,       //     Report Count (3)
  0x81, 0x02,       //     Input (Data, Variable, Absolute)
  0xC0,             //   End Collection
  0x05, 0x09,       //   Usage Page (Button)
  0x19, 0x01,       //   Usage Minimum (Button 1)
  0x29, 0x01,       //   Usage Maximum (Button 1)
  0x15, 0x00,       //   Logical Minimum (0)
  0x25, 0x01,       //   Logical Maximum (1)
  0x75, 0x01,       //   Report Size (1)
  0x95, 0x01,       //   Report Count (1)
  0x81, 0x02,       //   Input (Data, Variable, Absolute)
  0x75, 0x07,       //   Report Size (7)
  0x95, 0x01,       //   Report Count (1)
  0x81, 0x03,       //   Input (Constant, Variable, Absolute) padding
  0xC0              // End Collection
};

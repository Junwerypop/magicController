// arduino/controller/controller.ino
// Milestone 1 (StickC Plus2): อ่าน IMU แล้วส่ง JSON 1 บรรทัด/frame ผ่าน USB Serial

#include <M5StickCPlus2.h>

const unsigned long BAUD_RATE   = 115200;
const unsigned long SEND_PERIOD = 20;      // 20 ms = 50 Hz

float ax, ay, az;   // Accelerometer หน่วย g
float gx, gy, gz;   // Gyroscope หน่วย deg/s

unsigned long lastSend = 0;

// ---------- ส่วนอ่านเซ็นเซอร์ ----------
bool readSensors() {
  if (!StickCP2.Imu.update()) return false;   // false = ยังไม่มีข้อมูลใหม่

  auto data = StickCP2.Imu.getImuData();
  ax = data.accel.x;  ay = data.accel.y;  az = data.accel.z;
  gx = data.gyro.x;   gy = data.gyro.y;   gz = data.gyro.z;
  return true;
}

// ---------- ส่วนส่งข้อมูล ----------
void sendJson() {
  Serial.printf("{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f}\n",
                ax, ay, az, gx, gy, gz);
}

// ---------- แสดงสถานะบนจอ ----------
void showStatus() {
  StickCP2.Display.setRotation(1);
  StickCP2.Display.fillScreen(BLACK);
  StickCP2.Display.setTextSize(2);
  StickCP2.Display.setCursor(0, 0);
  StickCP2.Display.println("IMU OK");
  StickCP2.Display.println("50 Hz");
  StickCP2.Display.println("115200 baud");
}

void setup() {
  auto cfg = M5.config();
  StickCP2.begin(cfg);        // เริ่มจอ, IMU, power ทั้งหมด
  Serial.begin(BAUD_RATE);

  showStatus();
  Serial.println("# IMU ready");   // บรรทัดขึ้นต้น # = debug ให้ฝั่งเว็บข้าม
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend >= SEND_PERIOD) {
    lastSend = now;
    if (readSensors()) sendJson();
  }
}
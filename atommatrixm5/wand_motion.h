#pragma once

#include <Arduino.h>
#include <math.h>

// Firmware port of the browser's existing quaternion motion filter in
// web/motion.js. Keep the tuning and update order aligned with that file.
struct WandImuFrame {
  float ax, ay, az;  // g
  float gx, gy, gz;  // deg/s
  uint32_t timeMs;
};

struct WandAngles {
  float rollDeg;
  float pitchDeg;
  float yawDeg;
};

class WandMotion {
 public:
  static constexpr uint8_t kStillWindow = 25;
  static constexpr uint16_t kCalibrationSamples = 100;
  static constexpr float kCalibrationGyroRange = 8.0f;
  static constexpr float kStillGyroRange = 5.0f;
  static constexpr float kStillAccelRange = 0.10f;
  static constexpr float kStillMaxRate = 0.5f;
  static constexpr float kGyroDeadband = 0.5f;
  static constexpr float kStillLimit = 2.0f;
  static constexpr float kBiasAdapt = 0.005f;
  static constexpr float kKp = 1.0f;
  static constexpr float kAccelTrustRange = 0.3f;

  WandMotion() { begin(); }

  void begin() {
    q_ = identity();
    q0_ = identity();
    bias_ = {0.0f, 0.0f, 0.0f};
    ready_ = false;
    still_ = false;
    recentCount_ = 0;
    recentHead_ = 0;
    startCalibration();
  }

  void startCalibration() {
    calibrating_ = true;
    calibrationCount_ = 0;
    calibrationSum_ = {0.0f, 0.0f, 0.0f};
    recentCount_ = 0;
    recentHead_ = 0;
    still_ = false;
    hasTime_ = false;
  }

  void update(const WandImuFrame& frame) {
    pushRecent(frame);
    updateStillness();

    if (calibrating_) {
      collectCalibration(frame);
      hasTime_ = false;
      return;
    }

    if (!ready_) {
      initializeFromAccel(frame);
      lastTimeMs_ = frame.timeMs;
      hasTime_ = true;
      return;
    }

    const uint32_t dtMs = hasTime_ ? frame.timeMs - lastTimeMs_ : 0;
    lastTimeMs_ = frame.timeMs;
    hasTime_ = true;
    if (dtMs == 0 || dtMs > 100) return;
    integrate(frame, dtMs * 0.001f);
  }

  void recenter() {
    if (ready_) q0_ = q_;
  }

  bool ready() const { return ready_; }
  bool calibrating() const { return calibrating_; }
  bool still() const { return still_; }
  uint16_t calibrationCount() const { return calibrationCount_; }
  WandAngles angles() const {
    const Quaternion relative = normalize(multiply(conjugate(q0_), q_));
    const float roll = atan2f(2.0f * (relative.w * relative.x + relative.y * relative.z),
                              1.0f - 2.0f * (relative.x * relative.x + relative.y * relative.y));
    const float pitchTerm = clamp(2.0f * (relative.w * relative.y - relative.z * relative.x), -1.0f, 1.0f);
    const float pitch = asinf(pitchTerm);
    const float yaw = atan2f(2.0f * (relative.w * relative.z + relative.x * relative.y),
                             1.0f - 2.0f * (relative.y * relative.y + relative.z * relative.z));
    constexpr float kRadToDeg = 57.2957795131f;
    return {roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg};
  }

 private:
  struct Vec3 { float x, y, z; };
  struct Quaternion { float w, x, y, z; };
  struct Sample { float ax, ay, az, gx, gy, gz; };

  Quaternion q_ = {1, 0, 0, 0};
  Quaternion q0_ = {1, 0, 0, 0};
  Vec3 bias_ = {0, 0, 0};
  Vec3 calibrationSum_ = {0, 0, 0};
  Sample recent_[kStillWindow] = {};
  uint8_t recentCount_ = 0;
  uint8_t recentHead_ = 0;
  uint16_t calibrationCount_ = 0;
  uint32_t lastTimeMs_ = 0;
  bool hasTime_ = false;
  bool calibrating_ = true;
  bool ready_ = false;
  bool still_ = false;

  static Quaternion identity() { return {1.0f, 0.0f, 0.0f, 0.0f}; }
  static float clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
  }
  static Quaternion multiply(const Quaternion& a, const Quaternion& b) {
    return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
  }
  static Quaternion conjugate(const Quaternion& q) { return {q.w, -q.x, -q.y, -q.z}; }
  static Quaternion normalize(const Quaternion& q) {
    const float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm <= 0.0f) return identity();
    return {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
  }

  void pushRecent(const WandImuFrame& frame) {
    recent_[recentHead_] = {frame.ax, frame.ay, frame.az, frame.gx, frame.gy, frame.gz};
    recentHead_ = (recentHead_ + 1) % kStillWindow;
    if (recentCount_ < kStillWindow) recentCount_++;
  }

  bool gyroWindowStable(float maxRange) const {
    if (recentCount_ < kStillWindow) return false;
    const float low[3] = {INFINITY, INFINITY, INFINITY};
    float minimum[3] = {low[0], low[1], low[2]};
    float maximum[3] = {-low[0], -low[1], -low[2]};
    for (uint8_t i = 0; i < recentCount_; i++) {
      const Sample& sample = recent_[i];
      const float gyro[3] = {sample.gx, sample.gy, sample.gz};
      for (uint8_t axis = 0; axis < 3; axis++) {
        if (gyro[axis] < minimum[axis]) minimum[axis] = gyro[axis];
        if (gyro[axis] > maximum[axis]) maximum[axis] = gyro[axis];
      }
    }
    for (uint8_t axis = 0; axis < 3; axis++) {
      if (maximum[axis] - minimum[axis] > maxRange) return false;
    }
    return true;
  }

  void updateStillness() {
    if (recentCount_ < kStillWindow) { still_ = false; return; }
    float minimum[6] = {INFINITY, INFINITY, INFINITY, INFINITY, INFINITY, INFINITY};
    float maximum[6] = {-INFINITY, -INFINITY, -INFINITY, -INFINITY, -INFINITY, -INFINITY};
    Vec3 sumGyro = {0.0f, 0.0f, 0.0f};
    for (uint8_t i = 0; i < recentCount_; i++) {
      const Sample& s = recent_[i];
      const float values[6] = {s.gx, s.gy, s.gz, s.ax, s.ay, s.az};
      for (uint8_t axis = 0; axis < 6; axis++) {
        if (values[axis] < minimum[axis]) minimum[axis] = values[axis];
        if (values[axis] > maximum[axis]) maximum[axis] = values[axis];
      }
      sumGyro.x += s.gx; sumGyro.y += s.gy; sumGyro.z += s.gz;
    }
    const float n = static_cast<float>(recentCount_);
    const float meanX = sumGyro.x / n - (calibrating_ ? 0.0f : bias_.x);
    const float meanY = sumGyro.y / n - (calibrating_ ? 0.0f : bias_.y);
    const float meanZ = sumGyro.z / n - (calibrating_ ? 0.0f : bias_.z);
    const float meanRate = sqrtf(meanX * meanX + meanY * meanY + meanZ * meanZ);
    const float maxMeanRate = calibrating_ ? 10.0f : kStillMaxRate;
    bool accelStable = true;
    for (uint8_t axis = 3; axis < 6; axis++) {
      if (maximum[axis] - minimum[axis] > kStillAccelRange) accelStable = false;
    }
    still_ = gyroWindowStable(kStillGyroRange) && accelStable && meanRate < maxMeanRate;
  }

  void collectCalibration(const WandImuFrame& frame) {
    if (!gyroWindowStable(kCalibrationGyroRange)) {
      calibrationCount_ = 0;
      calibrationSum_ = {0.0f, 0.0f, 0.0f};
      return;
    }
    calibrationSum_.x += frame.gx;
    calibrationSum_.y += frame.gy;
    calibrationSum_.z += frame.gz;
    calibrationCount_++;
    if (calibrationCount_ >= kCalibrationSamples) {
      const float n = static_cast<float>(calibrationCount_);
      bias_ = {calibrationSum_.x / n, calibrationSum_.y / n, calibrationSum_.z / n};
      calibrating_ = false;
      hasTime_ = false;
    }
  }

  void initializeFromAccel(const WandImuFrame& frame) {
    const float magnitude = sqrtf(frame.ax * frame.ax + frame.ay * frame.ay + frame.az * frame.az);
    if (magnitude < 0.5f) return;
    const float ux = frame.ax / magnitude;
    const float uy = frame.ay / magnitude;
    const float uz = frame.az / magnitude;
    if (uz < -0.9999f) q_ = {0.0f, 1.0f, 0.0f, 0.0f};
    else q_ = normalize({1.0f + uz, uy, -ux, 0.0f});
    q0_ = q_;
    ready_ = true;
  }

  void integrate(const WandImuFrame& s, float dt) {
    float gx = s.gx - bias_.x;
    float gy = s.gy - bias_.y;
    float gz = s.gz - bias_.z;
    const float gyroMagnitude = sqrtf(gx * gx + gy * gy + gz * gz);
    const float accelMagnitude = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
    const bool accelTrusted = fabsf(accelMagnitude - 1.0f) < kAccelTrustRange;

    if (still_) {
      bias_.x += kBiasAdapt * gx;
      bias_.y += kBiasAdapt * gy;
      bias_.z += kBiasAdapt * gz;
      return; // Match motion.js: freeze q while stationary to prevent idle drift.
    }
    if (gyroMagnitude < kStillLimit && accelTrusted) {
      bias_.x += kBiasAdapt * gx;
      bias_.y += kBiasAdapt * gy;
      bias_.z += kBiasAdapt * gz;
    }
    if (gyroMagnitude < kGyroDeadband) gx = gy = gz = 0.0f;

    constexpr float kDegToRad = 0.017453292519943295f;
    float wx = gx * kDegToRad;
    float wy = gy * kDegToRad;
    float wz = gz * kDegToRad;
    if (accelTrusted) {
      const float ax = s.ax / accelMagnitude;
      const float ay = s.ay / accelMagnitude;
      const float az = s.az / accelMagnitude;
      const float vx = 2.0f * (q_.x * q_.z - q_.w * q_.y);
      const float vy = 2.0f * (q_.w * q_.x + q_.y * q_.z);
      const float vz = q_.w * q_.w - q_.x * q_.x - q_.y * q_.y + q_.z * q_.z;
      wx += kKp * (ay * vz - az * vy);
      wy += kKp * (az * vx - ax * vz);
      wz += kKp * (ax * vy - ay * vx);
    }
    Quaternion next = {
      q_.w + 0.5f * dt * (-q_.x * wx - q_.y * wy - q_.z * wz),
      q_.x + 0.5f * dt * ( q_.w * wx + q_.y * wz - q_.z * wy),
      q_.y + 0.5f * dt * ( q_.w * wy - q_.x * wz + q_.z * wx),
      q_.z + 0.5f * dt * ( q_.w * wz + q_.x * wy - q_.y * wx),
    };
    q_ = normalize(next);
  }
};

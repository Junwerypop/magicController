// web/motion.js
// orientation แบบ quaternion: gyro integration + accel แก้ tilt (Mahony แบบง่าย)

// ---------- ค่าที่ปรับได้ ----------
const KP = 1.0;                // แรงดึงด้วย accel (สูง = ตามเร็ว/noise มาก)
const ACCEL_TRUST_RANGE = 0.3; // เชื่อ accel เฉพาะตอน |a| อยู่ใน 1g ± 0.3
const GYRO_DEADBAND = 0.5;     // deg/s: เล็กกว่านี้ถือว่านิ่ง
const STILL_LIMIT = 2.0;       // deg/s: เล็กกว่านี้ถือว่า "นิ่ง" เพื่อปรับ bias
const BIAS_ADAPT = 0.005;      // ความเร็วปรับ bias ตอนนิ่ง
const CALIB_FRAMES = 100;      // ~2 วินาทีที่ 50 Hz
const CALIB_GYRO_RANGE = 8.0;  // raw deg/s range per axis; ignores constant sensor bias
const FLIP_ROTATION = 1;        // Match the reference magicController display direction.
const STILL_WINDOW = 25;         // ~0.5 วินาที ที่ 50 Hz
const STILL_GYRO_RANGE = 5.0;    // deg/s: gyro แกว่งไม่เกินนี้ในช่วง window ถือว่านิ่ง
const STILL_ACCEL_RANGE = 0.10;  // g: accel แกว่งไม่เกินนี้ถือว่านิ่ง
const STILL_MAX_RATE = 0.5;      // deg/s after bias removal; avoid freezing deliberate slow turns

const DEG = 180 / Math.PI;

// ---------- สถานะ ----------
let q  = { w: 1, x: 0, y: 0, z: 0 };   // orientation จริง (absolute)
let q0 = { w: 1, x: 0, y: 0, z: 0 };   // "ท่าตรง" ที่ผู้ใช้ตั้งไว้
let swordAxis = { x: 0, y: 1, z: 0 };  // แกนของ IMU ที่ชี้ไปตามความยาวดาบ
let gyroBias = { x: 0, y: 0, z: 0 };
let tracking = true;
let needInit = true;
let lastFrameTime = null;
let calib = null;                      // ไม่ null = กำลัง calibrate gyro

const orientation = { roll: 0, pitch: 0, yaw: 0 };  // เทียบกับท่าตรง (แสดงผล)

// ---------- quaternion helpers ----------
function qMul(a, b) {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}
function qConj(a) { return { w: a.w, x: -a.x, y: -a.y, z: -a.z }; }
function qNorm(a) {
  const n = Math.hypot(a.w, a.x, a.y, a.z) || 1;
  return { w: a.w / n, x: a.x / n, y: a.y / n, z: a.z / n };
}
// quaternion หมุนเวกเตอร์ a ไปเป็น b (unit vectors)
function quatBetween(a, b) {
  const d = a.x * b.x + a.y * b.y + a.z * b.z;
  if (d < -0.9999) return { w: 0, x: 0, y: 0, z: 1 };   // กลับด้าน 180°
  return qNorm({
    w: 1 + d,
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x,
  });
}

// ---------- เริ่ม orientation จาก accel (รู้ tilt, yaw = 0) ----------
function dominantGravityAxis(s) {
  const values = [Math.abs(s.ax), Math.abs(s.ay), Math.abs(s.az)];
  const axis = values.indexOf(Math.max(...values));
  const v = [s.ax, s.ay, s.az][axis];
  return { x: axis === 0 ? Math.sign(v) : 0, y: axis === 1 ? Math.sign(v) : 0, z: axis === 2 ? Math.sign(v) : 0 };
}

function initFromAccel(s) {
  const m = Math.hypot(s.ax, s.ay, s.az);
  if (m < 0.5) { needInit = true; return; }   // ยังไม่มีข้อมูล accel ที่ใช้ได้
  const ux = s.ax / m, uy = s.ay / m, uz = s.az / m;
  q = (uz < -0.9999) ? { w: 0, x: 1, y: 0, z: 0 }
                     : qNorm({ w: 1 + uz, x: uy, y: -ux, z: 0 });
  // Startup assumes the physical wand tip points upward; use gravity to identify its local axis.
  swordAxis = dominantGravityAxis(s);
  q0 = { ...q };
  needInit = false;
  document.getElementById("axisLabel").textContent = axisName(swordAxis) + " (startup)";
  if (typeof resetMovement === "function") resetMovement();
  setTrackStatus("TRACKING · neutral pose captured");
}

// ---------- Gyro calibration (วางนิ่ง) ----------
// ---------- Gyro calibration: ไม่บล็อกการ track แล้ว ----------
function startGyroCalibration() {
  calib = { count: 0, sum: { x: 0, y: 0, z: 0 } };
  recent = [];
  isStill = false;
  lastFrameTime = null;
  setTrackStatus("Calibrating gyro: hold still (0/100)");
}

function collectCalibration(s) {
  // Calibration has its own gyro-only stability gate. A constant gyro bias is
  // expected and is exactly what this average is meant to measure; accel noise
  // and the old bias must not hold the sample counter at zero.
  if (!calibrationWindowStable()) {
    if (calib.count > 0) {
      calib.count = 0;
      calib.sum = { x: 0, y: 0, z: 0 };
      setTrackStatus(`Calibrating gyro: hold still (0/${CALIB_FRAMES})`);
    }
    return;
  }
  calib.sum.x += s.gx; calib.sum.y += s.gy; calib.sum.z += s.gz;
  calib.count++;
  if (calib.count >= CALIB_FRAMES) {
    gyroBias = { x: calib.sum.x / calib.count, y: calib.sum.y / calib.count, z: calib.sum.z / calib.count };
    calib = null;
    lastFrameTime = null;
    setTrackStatus(tracking ? "TRACKING · gyro calibrated" : "PAUSED · gyro calibrated");
  } else if (calib.count % 10 === 0) {
    setTrackStatus(`Calibrating gyro: hold still (${calib.count}/${CALIB_FRAMES})`);
  }
}
let recent = [];
let isStill = false;

function calibrationWindowStable() {
  if (recent.length < STILL_WINDOW * 6) return false;
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < recent.length; i += 6) {
    for (let axis = 0; axis < 3; axis++) {
      const value = recent[i + axis];
      min[axis] = Math.min(min[axis], value);
      max[axis] = Math.max(max[axis], value);
    }
  }
  return max.every((value, axis) => value - min[axis] <= CALIB_GYRO_RANGE);
}

function updateStillness(s) {
  recent.push(s.gx, s.gy, s.gz, s.ax, s.ay, s.az);
  if (recent.length > STILL_WINDOW * 6) recent.splice(0, 6);
  if (recent.length < STILL_WINDOW * 6) { isStill = false; return; }

  const mn = [Infinity, Infinity, Infinity, Infinity, Infinity, Infinity];
  const mx = [-Infinity, -Infinity, -Infinity, -Infinity, -Infinity, -Infinity];
  let sumG = [0, 0, 0];
  for (let i = 0; i < recent.length; i += 6) {
    for (let c = 0; c < 6; c++) {
      const v = recent[i + c];
      if (v < mn[c]) mn[c] = v;
      if (v > mx[c]) mx[c] = v;
    }
    sumG[0] += recent[i]; sumG[1] += recent[i + 1]; sumG[2] += recent[i + 2];
  }
  const n = STILL_WINDOW;
  const meanG = [sumG[0] / n, sumG[1] / n, sumG[2] / n];
  // Before calibration only a broad raw-rate limit is possible; afterward,
  // assess stillness against the calibrated residual so sensor bias is ignored.
  const meanRate = calib
    ? Math.hypot(...meanG)
    : Math.hypot(meanG[0] - gyroBias.x, meanG[1] - gyroBias.y, meanG[2] - gyroBias.z);
  const rateLimit = calib ? 10 : STILL_MAX_RATE;

  let gyroOk = true, accelOk = true;
  for (let c = 0; c < 3; c++) if (mx[c] - mn[c] > STILL_GYRO_RANGE) gyroOk = false;
  for (let c = 3; c < 6; c++) if (mx[c] - mn[c] > STILL_ACCEL_RANGE) accelOk = false;

  isStill = gyroOk && accelOk && meanRate < rateLimit;
}
// ---------- serial.js เรียกทุก frame ที่ valid ----------
function onNewFrame(s) {
  updateStillness(s);                 // ตรวจความนิ่งจากค่าดิบ ทำทุก frame
  if (calib) {
    collectCalibration(s);
    lastFrameTime = null;
    return; // Avoid integrating residual bias while the stationary calibration is collecting.
  }
  if (!tracking) return;

  const now = performance.now();
  let dt = lastFrameTime === null ? 0 : (now - lastFrameTime) / 1000;
  lastFrameTime = now;

  if (needInit) { initFromAccel(s); return; }
  if (dt <= 0 || dt > 0.1) return;   // ข้าม frame ที่ค้างเพื่อไม่ให้มุมกระโดด

  // Match the working gyro handling from the supplied fixed project: adapt bias
  // only near rest, then suppress residual gyro noise below the deadband.
  let gx = s.gx - gyroBias.x, gy = s.gy - gyroBias.y, gz = s.gz - gyroBias.z;
  const gMag = Math.hypot(gx, gy, gz);
  const accelMag = Math.hypot(s.ax, s.ay, s.az);
  const accelOk = Math.abs(accelMag - 1) < ACCEL_TRUST_RANGE;

  if (isStill) {
    // Freeze attitude exactly while stationary. This prevents gyro noise and
    // tiny accel fluctuations from walking the displayed quaternion over time.
    gyroBias.x += BIAS_ADAPT * gx;
    gyroBias.y += BIAS_ADAPT * gy;
    gyroBias.z += BIAS_ADAPT * gz;
    return;
  }

  if (gMag < STILL_LIMIT && accelOk) {
    gyroBias.x += BIAS_ADAPT * gx;
    gyroBias.y += BIAS_ADAPT * gy;
    gyroBias.z += BIAS_ADAPT * gz;
  }
  if (gMag < GYRO_DEADBAND) { gx = 0; gy = 0; gz = 0; }

  let wx = gx / DEG, wy = gy / DEG, wz = gz / DEG;

  // accel แก้ tilt (ตอนนิ่งช่วยให้ tilt นิ่งที่ค่าจริง)
  if (accelOk) {
    const ax = s.ax / accelMag, ay = s.ay / accelMag, az = s.az / accelMag;
    const vx = 2 * (q.x * q.z - q.w * q.y);
    const vy = 2 * (q.w * q.x + q.y * q.z);
    const vz = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
    wx += KP * (ay * vz - az * vy);
    wy += KP * (az * vx - ax * vz);
    wz += KP * (ax * vy - ay * vx);
  }

  q = qNorm({
    w: q.w + 0.5 * dt * (-q.x * wx - q.y * wy - q.z * wz),
    x: q.x + 0.5 * dt * ( q.w * wx + q.y * wz - q.z * wy),
    y: q.y + 0.5 * dt * ( q.w * wy - q.x * wz + q.z * wx),
    z: q.z + 0.5 * dt * ( q.w * wz + q.x * wy - q.y * wx),
  });
}

// ---------- ผลลัพธ์สำหรับแสดงผล ----------
// หมุนสัมพัทธ์กับท่าตรง
function relativeQuat() { return qMul(qConj(q0), q); }

// คืน {angle, axis} ให้ p5 rotate() โดยจัดแกนให้ปลายดาบ (swordAxis) ชี้ขึ้นบนจอในท่าตรง
function getDisplayRotation() {
  const qC = quatBetween(swordAxis, { x: 0, y: 1, z: 0 });
  const d = qMul(qMul(qC, relativeQuat()), qConj(qC));
  const w = Math.max(-1, Math.min(1, d.w));
  const sinHalf = Math.sqrt(1 - w * w);
  if (sinHalf < 1e-6) return { angle: 0, axis: [0, 1, 0] };
  const angle = 2 * Math.acos(w);
  // Match the working coordinate conversion in the supplied project:
  // p5's screen Y points down, so flip the angle and Y component together.
  return { angle: -angle * FLIP_ROTATION, axis: [d.x / sinHalf, -d.y / sinHalf, d.z / sinHalf] };
}

function updateOrientationDisplay() {
  const r = relativeQuat();
  const roll  = Math.atan2(2 * (r.w * r.x + r.y * r.z), 1 - 2 * (r.x * r.x + r.y * r.y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (r.w * r.y - r.z * r.x))));
  const yaw   = Math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z));
  document.getElementById("roll").textContent  = (roll * DEG).toFixed(0);
  document.getElementById("pitch").textContent = (pitch * DEG).toFixed(0);
  document.getElementById("yaw").textContent   = (yaw * DEG).toFixed(0);
      document.getElementById("qDebug").textContent =
      `q = (${q.w.toFixed(2)}, ${q.x.toFixed(2)}, ${q.y.toFixed(2)}, ${q.z.toFixed(2)}) | still=${isStill} | bias=(${gyroBias.x.toFixed(2)}, ${gyroBias.y.toFixed(2)}, ${gyroBias.z.toFixed(2)})`;
}

// ---------- UI ----------
function setTrackStatus(text) { document.getElementById("trackStatus").textContent = text; }

function axisName(a) {
  const sign = (a.x + a.y + a.z) > 0 ? "+" : "-";
  return sign + (a.x ? "X" : a.y ? "Y" : "Z");
}

const pauseBtn = document.getElementById("pauseBtn");
pauseBtn.addEventListener("click", () => {
  tracking = !tracking;
  pauseBtn.textContent = tracking ? "Pause Tracking" : "Resume Tracking";
  lastFrameTime = null;                  // กันมุมกระโดดตอนกลับมา track
  setTrackStatus(tracking ? "TRACKING" : "PAUSED");
});

document.getElementById("resetBtn").addEventListener("click", () => {
  q0 = { ...q };                          // Current pose becomes neutral, without changing the live attitude.
  lastFrameTime = null;
  recent = [];
  if (typeof resetMovement === "function") resetMovement();
  if (typeof resetCameraView === "function") resetCameraView();
  setTrackStatus("TRACKING · recentered");
});

document.getElementById("gyroCalBtn").addEventListener("click", startGyroCalibration);

document.getElementById("detectAxisBtn").addEventListener("click", () => {
  const ax = Math.abs(sensor.ax), ay = Math.abs(sensor.ay), az = Math.abs(sensor.az);
  const maxV = Math.max(ax, ay, az);
  if (maxV < 0.7) {
    document.getElementById("axisLabel").textContent = "ตรวจไม่ได้ ถือให้ปลายดาบชี้ขึ้นตรงๆ แล้วลองใหม่";
    return;
  }
  // accel อ่านค่าบวก ≈ +1 ที่แกนซึ่งชี้ "ขึ้น"
  swordAxis = { x: 0, y: 0, z: 0 };
  if (maxV === ax) swordAxis.x = Math.sign(sensor.ax);
  else if (maxV === ay) swordAxis.y = Math.sign(sensor.ay);
  else swordAxis.z = Math.sign(sensor.az);
  q0 = { ...q };
  lastFrameTime = null;
  if (typeof resetMovement === "function") resetMovement();
  document.getElementById("axisLabel").textContent = axisName(swordAxis);
});

setTrackStatus("TRACKING");

// web/motion.js
// orientation แบบ quaternion: gyro integration + accel แก้ tilt (Mahony แบบง่าย)

// ---------- ค่าที่ปรับได้ ----------
const KP = 1.0;                // แรงดึงด้วย accel (สูง = ตามเร็ว/noise มาก)
const ACCEL_TRUST_RANGE = 0.3; // เชื่อ accel เฉพาะตอน |a| อยู่ใน 1g ± 0.3
const GYRO_DEADBAND = 0.5;     // deg/s: เล็กกว่านี้ถือว่านิ่ง
const STILL_LIMIT = 2.0;       // deg/s: เล็กกว่านี้ถือว่า "นิ่ง" เพื่อปรับ bias
const BIAS_ADAPT = 0.005;      // ความเร็วปรับ bias ตอนนิ่ง
const CALIB_FRAMES = 100;      // ~2 วินาทีที่ 50 Hz
const FLIP_ROTATION = 1;       // ถ้า model หมุนสวนทางทุกแกน เปลี่ยนเป็น -1
const STILL_WINDOW = 25;         // ~0.5 วินาที ที่ 50 Hz
const STILL_GYRO_RANGE = 1.5;    // deg/s: gyro แกว่งไม่เกินนี้ในช่วง window ถือว่านิ่ง
const STILL_ACCEL_RANGE = 0.03;  // g: accel แกว่งไม่เกินนี้ถือว่านิ่ง
const STILL_MAX_RATE = 10;       // deg/s: ค่าเฉลี่ย gyro ต้องไม่เกินนี้ (กันหมุนช้าสม่ำเสมอ)
const BIAS_FAST = 0.05;          // ความเร็วเรียนรู้ bias ตอนนิ่ง

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
function initFromAccel(s) {
  const m = Math.hypot(s.ax, s.ay, s.az);
  if (m < 0.5) { needInit = true; return; }   // ยังไม่มีข้อมูล accel ที่ใช้ได้
  const ux = s.ax / m, uy = s.ay / m, uz = s.az / m;
  q = (uz < -0.9999) ? { w: 0, x: 1, y: 0, z: 0 }
                     : qNorm({ w: 1 + uz, x: uy, y: -ux, z: 0 });
  needInit = false;
}

// ---------- Gyro calibration (วางนิ่ง) ----------
// ---------- Gyro calibration: ไม่บล็อกการ track แล้ว ----------
function startGyroCalibration() {
  calib = { count: 0, sum: { x: 0, y: 0, z: 0 }, startTime: performance.now() };
  setTrackStatus("Calibrating gyro (model ยังตามอยู่)...");
}

function collectCalibration(s) {
  // เก็บเฉพาะ frame ที่นิ่ง ถ้าขยับก็แค่ข้าม frame นั้น ไม่รีเซ็ตทั้งหมด
  if (Math.hypot(s.gx, s.gy, s.gz) < 15) {
    calib.sum.x += s.gx; calib.sum.y += s.gy; calib.sum.z += s.gz;
    calib.count++;
  }
  const timedOut = performance.now() - calib.startTime > 4000;   // เกิน 4 วินาทีเลิกรอ
  if (calib.count >= CALIB_FRAMES) {
    gyroBias = { x: calib.sum.x / calib.count, y: calib.sum.y / calib.count, z: calib.sum.z / calib.count };
    calib = null;
    setTrackStatus(tracking ? "TRACKING" : "PAUSED");
  } else if (timedOut) {
    calib = null;                                  // ขยับตลอด: ใช้ bias เดิม
    setTrackStatus(tracking ? "TRACKING (calibrate ไม่สำเร็จ ใช้ค่าเดิม)" : "PAUSED");
  }
}
let recent = [];
let isStill = false;

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
  const meanRate = Math.hypot(sumG[0] / n, sumG[1] / n, sumG[2] / n);

  let gyroOk = true, accelOk = true;
  for (let c = 0; c < 3; c++) if (mx[c] - mn[c] > STILL_GYRO_RANGE) gyroOk = false;
  for (let c = 3; c < 6; c++) if (mx[c] - mn[c] > STILL_ACCEL_RANGE) accelOk = false;

  isStill = gyroOk && accelOk && meanRate < STILL_MAX_RATE;
}
// ---------- serial.js เรียกทุก frame ที่ valid ----------
function onNewFrame(s) {
  if (calib) collectCalibration(s);
  updateStillness(s);                 // ตรวจความนิ่งจากค่าดิบ ทำทุก frame
  if (!tracking) return;

  const now = performance.now();
  let dt = lastFrameTime === null ? 0 : (now - lastFrameTime) / 1000;
  lastFrameTime = now;

  if (needInit) { initFromAccel(s); return; }
  if (dt <= 0) return;
  if (dt > 0.1) dt = 0.1;

  const accelMag = Math.hypot(s.ax, s.ay, s.az);
  const accelOk = Math.abs(accelMag - 1) < ACCEL_TRUST_RANGE;

  let gx, gy, gz;
  if (isStill) {
    // นิ่งจริง: เรียนรู้ bias จากค่าดิบ แล้วล็อกการหมุนเป็น 0
    gyroBias.x += BIAS_FAST * (s.gx - gyroBias.x);
    gyroBias.y += BIAS_FAST * (s.gy - gyroBias.y);
    gyroBias.z += BIAS_FAST * (s.gz - gyroBias.z);
    gx = 0; gy = 0; gz = 0;
  } else {
    gx = s.gx - gyroBias.x; gy = s.gy - gyroBias.y; gz = s.gz - gyroBias.z;
    if (Math.hypot(gx, gy, gz) < GYRO_DEADBAND) { gx = 0; gy = 0; gz = 0; }
  }

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
// ---------- serial.js เรียกทุก frame ที่ valid ----------
function onNewFrame(s) {
  if (calib) { collectCalibration(s); return; }
  if (!tracking) return;

  const now = performance.now();
  const dt = lastFrameTime === null ? 0 : (now - lastFrameTime) / 1000;
  lastFrameTime = now;

  if (needInit) { initFromAccel(s); return; }
  if (dt <= 0 || dt > 0.1) return;   // frame ค้างผิดปกติ ข้ามเพื่อไม่ให้มุมกระโดด

  // 1) ลบ bias
  let gx = s.gx - gyroBias.x, gy = s.gy - gyroBias.y, gz = s.gz - gyroBias.z;
  const gMag = Math.hypot(gx, gy, gz);
  const accelMag = Math.hypot(s.ax, s.ay, s.az);
  const accelOk = Math.abs(accelMag - 1) < ACCEL_TRUST_RANGE;

  // 2) ตอนนิ่ง: ปรับ bias ช้าๆ ให้ตามค่าจริง
  if (gMag < STILL_LIMIT && accelOk) {
    gyroBias.x += BIAS_ADAPT * gx;
    gyroBias.y += BIAS_ADAPT * gy;
    gyroBias.z += BIAS_ADAPT * gz;
  }

  // 3) deadband: เล็กมากถือว่านิ่ง
  if (gMag < GYRO_DEADBAND) { gx = 0; gy = 0; gz = 0; }

  // 4) rad/s
  let wx = gx / DEG, wy = gy / DEG, wz = gz / DEG;

  // 5) accel แก้ tilt: เทียบทิศ "ขึ้น" ที่วัดได้กับที่ quaternion คาดไว้
  if (accelOk) {
    const ax = s.ax / accelMag, ay = s.ay / accelMag, az = s.az / accelMag;
    const vx = 2 * (q.x * q.z - q.w * q.y);
    const vy = 2 * (q.w * q.x + q.y * q.z);
    const vz = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
    wx += KP * (ay * vz - az * vy);
    wy += KP * (az * vx - ax * vz);
    wz += KP * (ax * vy - ay * vx);
  }

  // 6) integrate gyro เข้า quaternion
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
  // p5 แกน Y ชี้ลง (ซ้าย-มือ) จึงต้องกลับแกน Y และมุมเพื่อให้ทิศการหมุนตรงกับของจริง
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
  q0 = { w: 1, x: 0, y: 0, z: 0 };       // ล้างท่าตรง
  needInit = true;
  lastFrameTime = null;
  recent = [];
  initFromAccel(sensor);                 // ใช้ได้แม้กำลัง Pause
});

document.getElementById("gyroCalBtn").addEventListener("click", startGyroCalibration);

document.getElementById("setStraightBtn").addEventListener("click", () => {
  q0 = { ...q };                         // ท่านี้ = 0°
});

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
  document.getElementById("axisLabel").textContent = axisName(swordAxis);
});

setTrackStatus("TRACKING");
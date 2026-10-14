// Thrust gesture detection, separated from orientation (motion.js).
const MOVEMENT_CONFIG = {
  smoothing: 0.28,           // low-pass alpha, 0..1
  thrustThreshold: 0.65,     // g along +Y (toward wand tip)
  thrustRelease: 0.30,       // g hysteresis threshold
  thrustSamples: 2,          // consecutive samples to reject single-frame spikes
  thrustCooldown: 450,       // ms between thrust gesture events
  gestureDisplayMs: 300,
};

const movement = {
  linearAccel: { x: 0, y: 0, z: 0 },
  filtered: { x: 0, y: 0, z: 0 },
  offset: { x: 0, y: 0, z: 0 },
  state: "IDLE",
  lastGesture: "—",
  gestureCount: 0,
  axisLabel: "—",
};
let lastMoveTime = null;
let thrustSamples = 0;
let thrustLatched = false;
let lastThrustTime = -Infinity;
let gestureUntil = 0;

function qRotate(r, v) {
  const p = qMul(qMul(r, { w: 0, x: v.x, y: v.y, z: v.z }), qConj(r));
  return { x: p.x, y: p.y, z: p.z };
}

// Convert measured specific force to the neutral wand coordinate system.
function getWandLinearAcceleration(s) {
  // Expected gravity in current sensor coordinates, using the same quaternion as motion.js.
  const gravity = {
    x: 2 * (q.x * q.z - q.w * q.y),
    y: 2 * (q.w * q.x + q.y * q.z),
    z: q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
  };
  const linearSensor = { x: s.ax - gravity.x, y: s.ay - gravity.y, z: s.az - gravity.z };
  // q0^-1*q moves the current sensor vector into its startup/recenter frame;
  // qC then aligns the detected wand axis to local +Y (the tip direction).
  const relative = relativeQuat();
  const neutralSensor = qRotate(relative, linearSensor);
  const qC = quatBetween(swordAxis, { x: 0, y: 1, z: 0 });
  return qRotate(qC, neutralSensor);
}

function resetMovement() {
  movement.linearAccel = { x: 0, y: 0, z: 0 };
  movement.filtered = { x: 0, y: 0, z: 0 };
  movement.offset = { x: 0, y: 0, z: 0 };
  movement.state = "IDLE";
  movement.lastGesture = "—";
  movement.gestureCount = 0;
  movement.axisLabel = "—";
  lastMoveTime = null;
  thrustSamples = 0;
  thrustLatched = false;
  lastThrustTime = -Infinity;
  gestureUntil = 0;
}

function emitThrust(intensity) {
  movement.gestureCount++;
  movement.lastGesture = "THRUST";
  gestureUntil = performance.now() + MOVEMENT_CONFIG.gestureDisplayMs;
  window.dispatchEvent(new CustomEvent("magicwand:gesture", {
    detail: { type: "thrust", intensity, count: movement.gestureCount },
  }));
}

function onMovementFrame(s) {
  const C = MOVEMENT_CONFIG;
  if (!tracking || needInit) { lastMoveTime = null; return; }

  const now = performance.now();
  const dt = lastMoveTime === null ? 0 : Math.min((now - lastMoveTime) / 1000, 0.1);
  lastMoveTime = now;
  if (dt <= 0) return;

  const raw = getWandLinearAcceleration(s);
  movement.linearAccel = raw;
  const keys = ["x", "y", "z"];

  // Learn residual gravity/tilt offset only while the stillness detector agrees.
  if (isStill) {
    for (const axis of keys) movement.offset[axis] += 0.015 * (raw[axis] - movement.offset[axis]);
  }
  for (const axis of keys) {
    const corrected = raw[axis] - movement.offset[axis];
    movement.filtered[axis] += C.smoothing * (corrected - movement.filtered[axis]);
  }

  // A thrust is a short positive acceleration along the wand's local tip axis.
  const thrustSignal = movement.filtered.y > C.thrustThreshold;
  if (thrustSignal && !thrustLatched) {
    thrustSamples++;
    if (thrustSamples >= C.thrustSamples && now - lastThrustTime >= C.thrustCooldown) {
      emitThrust(movement.filtered.y);
      lastThrustTime = now;
      thrustLatched = true;
    }
  } else if (!thrustSignal) {
    thrustSamples = 0;
    if (movement.filtered.y < C.thrustRelease) thrustLatched = false;
  }

  movement.state = now < gestureUntil ? "THRUST" : "IDLE";
}

function updateMovementDisplay() {
  document.getElementById("mvState").textContent = movement.state;
  document.getElementById("gestureCount").textContent = movement.gestureCount;
  document.getElementById("lastGesture").textContent = movement.lastGesture;

  const indicator = document.getElementById("mvIndicator");
  indicator.textContent = movement.state === "THRUST" ? "แทง / THRUST" : "พร้อม / IDLE";
}

document.getElementById("resetBtn").addEventListener("click", resetMovement);

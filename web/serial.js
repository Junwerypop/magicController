// web/serial.js
// Milestone 2: Web Serial -> อ่านทีละบรรทัด -> parse JSON -> validate -> แสดงผล

const BAUD_RATE = 115200;

let port = null;
let reader = null;
let keepReading = false;

// ค่า sensor ล่าสุด (raw) ไฟล์อื่นในอนาคตจะอ่านจากตัวนี้
const sensor = { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0 };

// ตัวนับสำหรับ debug
let frameCount = 0;
let badLineCount = 0;

// ---------- UI ----------
const connectBtn    = document.getElementById("connectBtn");
const disconnectBtn = document.getElementById("disconnectBtn");
const statusEl      = document.getElementById("status");

function setStatus(text, isOn) {
  statusEl.textContent = text;
  statusEl.className = isOn ? "on" : "off";
  connectBtn.disabled = isOn;
  disconnectBtn.disabled = !isOn;
}

function updateSensorDisplay() {
  for (const key of Object.keys(sensor)) {
    document.getElementById(key).textContent = sensor[key].toFixed(2);
  }
  document.getElementById("bad").textContent = badLineCount;
}

// นับจำนวน frame ต่อวินาที ไว้เช็กว่าได้ ~50 Hz จริงไหม
setInterval(() => {
  document.getElementById("rate").textContent = frameCount;
  frameCount = 0;
}, 1000);

// ---------- Parse + validate ----------
function handleLine(line) {
  if (line === "") return;
  if (line.startsWith("#")) return;        // debug จาก Arduino ข้ามได้

  try {
    const data = JSON.parse(line);
    for (const key of Object.keys(sensor)) {
      if (typeof data[key] !== "number" || !isFinite(data[key])) {
        throw new Error("invalid field: " + key);
      }
    }
    Object.assign(sensor, data);
    onNewFrame(sensor);
    frameCount++;
    updateSensorDisplay();
  } catch (err) {
    badLineCount++;                         // ข้อมูลเพี้ยน: นับไว้ ไม่ให้เว็บ crash
  }
}

// ---------- Connect / Read / Disconnect ----------
async function connectController() {
  try {
    port = await navigator.serial.requestPort();   // ผู้ใช้เลือก port เอง
    await port.open({ baudRate: BAUD_RATE });
  } catch (err) {
    console.warn("Connect failed:", err);
    setStatus("Disconnected (" + err.message + ")", false);
    return;
  }
  keepReading = true;
  setStatus("Connected", true);
  startGyroCalibration(); 
  readLoop();
}

async function readLoop() {
  const decoder = new TextDecoderStream();
  const pipeDone = port.readable.pipeTo(decoder.writable).catch(() => {});
  reader = decoder.readable.getReader();

  let buffer = "";   // เก็บข้อความที่ยังมาไม่ครบบรรทัด
  try {
    while (keepReading) {
      const { value, done } = await reader.read();
      if (done) break;
      buffer += value;
      const lines = buffer.split("\n");
      buffer = lines.pop();                // ท่อนสุดท้ายอาจยังไม่จบบรรทัด เก็บไว้ก่อน
      for (const line of lines) handleLine(line.trim());
    }
  } catch (err) {
    console.warn("Read error (unplugged?):", err);
  } finally {
    reader.releaseLock();
    await pipeDone;
    try { await port.close(); } catch (e) { /* ปิดไปแล้วหรือถูกถอดสาย */ }
    port = null;
    setStatus("Disconnected", false);
  }
}

async function disconnectController() {
  keepReading = false;
  if (reader) await reader.cancel();       // ทำให้ readLoop จบแล้วปิด port เอง
}

// ---------- Start ----------
if (!("serial" in navigator)) {
  document.getElementById("warning").textContent =
    "Browser นี้ไม่รองรับ Web Serial กรุณาใช้ Chrome หรือ Edge";
  connectBtn.disabled = true;
}
connectBtn.addEventListener("click", connectController);
disconnectBtn.addEventListener("click", disconnectController);
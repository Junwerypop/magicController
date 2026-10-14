// One Atom Matrix serial input. Parsing and input events stay separate from motion math.
const BAUD_RATE = 115200;
const SENSOR_KEYS = ["ax", "ay", "az", "gx", "gy", "gz"];
const sensor = { ax: 0, ay: 0, az: 0, gx: 0, gy: 0, gz: 0 };
let port = null;
let reader = null;
let keepReading = false;
let badLineCount = 0;
let framesThisSecond = 0;

const connectBtn = document.getElementById("connectBtn");
const disconnectBtn = document.getElementById("disconnectBtn");
const statusEl = document.getElementById("status");

function refreshStatus(connected) {
  statusEl.textContent = connected ? "Atom Matrix connected" : "Disconnected";
  statusEl.className = connected ? "on" : "off";
  connectBtn.disabled = connected;
  disconnectBtn.disabled = !connected;
}

function handleLine(line) {
  if (!line || line.startsWith("#")) return;
  try {
    const data = JSON.parse(line);
    for (const key of SENSOR_KEYS) {
      if (typeof data[key] !== "number" || !Number.isFinite(data[key])) throw new Error("bad " + key);
      sensor[key] = data[key];
    }

    // Motion input: raw IMU samples feed the existing orientation and movement calculations.
    onNewFrame(sensor);
    onMovementFrame(sensor);
    framesThisSecond++;

    handleButtonPacket(data);
    document.getElementById("ax").textContent = sensor.ax.toFixed(2);
    document.getElementById("ay").textContent = sensor.ay.toFixed(2);
    document.getElementById("az").textContent = sensor.az.toFixed(2);
    document.getElementById("gx").textContent = sensor.gx.toFixed(2);
    document.getElementById("gy").textContent = sensor.gy.toFixed(2);
    document.getElementById("gz").textContent = sensor.gz.toFixed(2);
  } catch (error) {
    badLineCount++;
  }
  document.getElementById("bad").textContent = badLineCount;
}

setInterval(() => {
  document.getElementById("rate").textContent = framesThisSecond;
  framesThisSecond = 0;
}, 1000);

async function connectController() {
  try {
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: BAUD_RATE });
    keepReading = true;
    refreshStatus(true);
    startGyroCalibration();
    readLoop();
  } catch (error) {
    document.getElementById("warning").textContent = "Connection failed: " + error.message;
  }
}

async function readLoop() {
  const decoder = new TextDecoderStream();
  const pipeDone = port.readable.pipeTo(decoder.writable).catch(() => {});
  reader = decoder.readable.getReader();
  let buffer = "";
  try {
    while (keepReading) {
      const { value, done } = await reader.read();
      if (done) break;
      buffer += value;
      const lines = buffer.split("\n");
      buffer = lines.pop();
      for (const line of lines) handleLine(line.trim());
    }
  } catch (error) {
    if (keepReading) document.getElementById("warning").textContent = "Serial read failed: " + error.message;
  } finally {
    reader.releaseLock();
    await pipeDone;
    try { await port.close(); } catch (error) {}
    port = null;
    reader = null;
    keepReading = false;
    refreshStatus(false);
  }
}

async function disconnectController() {
  keepReading = false;
  if (reader) await reader.cancel();
}

if (!("serial" in navigator)) {
  document.getElementById("warning").textContent = "Web Serial requires Chrome or Edge.";
  connectBtn.disabled = true;
}
connectBtn.addEventListener("click", connectController);
disconnectBtn.addEventListener("click", disconnectController);
refreshStatus(false);

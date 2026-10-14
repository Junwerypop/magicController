// web/sketch.js
// หมุน model ตาม orientation จาก IMU แต่ไม่เลื่อนตำแหน่งจาก accelerometer

const CANVAS_HEIGHT = 420;
let currentModel = "box";   // "box" | "staff" | "sword"

function setup() {
  const holder = document.getElementById("visualizer");
  const canvas = createCanvas(holder.clientWidth, CANVAS_HEIGHT, WEBGL);
  canvas.parent(holder);
  resetCameraView();

  document.querySelectorAll(".modelBtn").forEach(btn => {
    btn.addEventListener("click", () => {
      currentModel = btn.dataset.model;
      document.querySelectorAll(".modelBtn").forEach(b => b.classList.remove("active"));
      btn.classList.add("active");
    });
  });
}

// Stable oblique default view; mouse orbit remains available after startup.
function resetCameraView() {
  camera(360, -260, 620, 0, 0, 0, 0, 1, 0);
  perspective(PI / 3, width / height, 1, 3000);
}

function windowResized() {
  const holder = document.getElementById("visualizer");
  resizeCanvas(holder.clientWidth, CANVAS_HEIGHT);
  resetCameraView();
}

function draw() {
  background(0);
  orbitControl();
  ambientLight(90);
  directionalLight(255, 255, 255, 0.3, 0.5, -1);

  drawAxes(120);                       // แกนโลก อยู่นิ่ง

  const r = getDisplayRotation();
  push();
  // Keep translation disabled; preserve three-axis orientation tracking.
  rotate(r.angle, r.axis);
  drawModel();
  drawLocalAxes(45);
  pop();

  updateOrientationDisplay();
  updateMovementDisplay();
}

function drawModel() {
  noStroke();
  if (currentModel === "box")   drawBox();
  if (currentModel === "staff") drawStaff();
  if (currentModel === "sword") drawSword();
}

// ทุก model: ยาวตามแกน Y ของจอ ปลายชี้ขึ้น (p5 แกน Y ลบ = ขึ้น) จุดหมุน = ตำแหน่ง IMU

function drawBox() {
  fill(230);
  box(50, 100, 28);
  push(); translate(0, -55, 0); fill(255, 140, 0); box(12, 12, 12); pop();   // จุดส้ม = ปลาย
}

function drawStaff() {
  fill(150, 100, 60);
  cylinder(5, 180, 12, 1);
  push(); translate(0, -95, 0); fill(80, 180, 255); sphere(14); pop();       // ลูกแก้วปลายคทา
}

function drawSword() {
  fill(110, 70, 40);
  push(); translate(0, 45, 0); cylinder(4, 50, 12, 1); pop();                // ด้าม
  push(); translate(0, 18, 0); fill(200, 170, 60); box(50, 6, 8); pop();     // การ์ด
  push(); translate(0, -35, 0); fill(220); box(14, 100, 3); pop();           // ใบดาบ
  push(); translate(0, -85, 0); rotateZ(QUARTER_PI); fill(220); box(10, 10, 3); pop(); // ปลายแหลม
}

function drawAxes(len) {
  strokeWeight(2);
  stroke(255, 80, 80);  line(0, 0, 0, len, 0, 0);     // X แดง (ขวา)
  stroke(80, 255, 80);  line(0, 0, 0, 0, -len, 0);    // Y เขียว (ขึ้น)
  stroke(80, 150, 255); line(0, 0, 0, 0, 0, len);     // Z น้ำเงิน (เข้าหาเรา)
}

function drawLocalAxes(len) {
  strokeWeight(1.5);
  stroke(255, 100, 100);  line(-len, 0, 0, len, 0, 0);
  stroke(100, 255, 100);  line(0, len, 0, 0, -len, 0);
  stroke(100, 170, 255);  line(0, 0, -len, 0, 0, len);
}

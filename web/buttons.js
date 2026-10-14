// Browser-facing, game-ready button interface. Physical debouncing is performed by M5Unified.
const controllerInput = {
  button: { down: false, pressed: false, released: false, lastEvent: null, eventCount: 0 },
};

function handleButtonPacket(data) {
  const button = controllerInput.button;
  const down = data.buttonDown === true;
  const events = [];

  // Process both edges in order if a very short tap is reported in one serial frame.
  if (data.buttonPressed === true && !button.down) {
    button.down = true;
    events.push("press");
  }
  if (data.buttonReleased === true && button.down) {
    button.down = false;
    events.push("release");
  }

  // On connect, synchronize a button that was already held without inventing a press.
  if (!data.buttonPressed && !data.buttonReleased) button.down = down;
  button.pressed = events.includes("press");
  button.released = events.includes("release");

  document.getElementById("buttonState").textContent = button.down ? "Pressed" : "Released";
  for (const type of events) {
    button.lastEvent = type;
    button.eventCount++;
    document.getElementById("buttonEvent").textContent = `${type} (${button.eventCount})`;
    window.dispatchEvent(new CustomEvent("magicwand:button", {
      detail: { type, down: button.down, count: button.eventCount },
    }));
  }
}

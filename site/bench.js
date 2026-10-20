// Runs the robot's real firmware, firmware/main.py, in the browser.
//
// MicroPython compiled to WebAssembly executes the firmware file unchanged.
// bench.py replaces the hardware with fakes: a simulated test stand where
// the wheels spin in the air. This file loads both, forwards the remote's
// commands over the fake Bluetooth link, and draws what the firmware does.
(() => {
  "use strict";

  const MICROPYTHON =
    "https://cdn.jsdelivr.net/npm/@micropython/micropython-webassembly-pyscript@1.29.0-6/micropython.mjs";
  const MOVES = ["FORWARD", "BACKWARD", "LEFT", "RIGHT"];
  const STOP_CM = 15; // OBSTACLE_THRESHOLD_CM in the firmware
  const MAX_CM = 120;

  const $ = (id) => document.getElementById(id);
  const canvas = $("stand");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const ui = {
    loading: $("bench-loading"),
    status: $("bench-status"),
    state: $("bench-state"),
    power: $("bench-power"),
    pulses: $("bench-pulses"),
    led: $("bench-led"),
    fan: $("bench-fan"),
    log: $("bench-log"),
    name: $("bench-name"),
    link: $("bench-link"),
    sound: $("bench-sound"),
    distance: $("bench-distance"),
    distanceOut: $("bench-distance-out"),
    holdLeft: $("hold-left"),
    holdRight: $("hold-right"),
    lid: $("lid-button"),
    keys: [...document.querySelectorAll(".key[data-cmd]")],
  };
  const controls = [ui.link, ui.distance, ui.holdLeft, ui.holdRight, ui.lid, ...ui.keys];
  const reduceMotion = matchMedia("(prefers-reduced-motion: reduce)").matches;

  let mp = null;
  let bench = null; // the Python Bench object from bench.py
  let s = null; // latest state from bench.state()
  let distance = Number(ui.distance.value);

  // ---------------------------------------------------------------
  // Log: commands sent, stand events, and the firmware's own print output
  // ---------------------------------------------------------------
  const logLines = [];

  function log(text, kind) {
    const top = logLines[0];
    if (top && top.text === text) top.count += 1; // collapse repeats
    else logLines.unshift({ text, kind, count: 1 });
    logLines.length = Math.min(logLines.length, 5);
    ui.log.replaceChildren(
      ...logLines.map((line) => {
        const li = document.createElement("li");
        li.className = `log-${line.kind}`;
        li.textContent = line.count > 1 ? `${line.text}  ×${line.count}` : line.text;
        return li;
      }),
    );
  }

  // ---------------------------------------------------------------
  // Starting MicroPython and the firmware
  // ---------------------------------------------------------------
  const textOrThrow = (r) => {
    if (!r.ok) throw new Error(`${r.url}: ${r.status}`);
    return r.text();
  };
  const read = () => (s = JSON.parse(bench.state()));

  async function start() {
    try {
      const [{ loadMicroPython }, firmware, benchSource] = await Promise.all([
        import(MICROPYTHON),
        fetch("firmware/main.py").then(textOrThrow),
        fetch("bench.py").then(textOrThrow),
      ]);
      mp = await loadMicroPython({ stdout: (line) => log(line, "serial") });
      mp.FS.writeFile("/main.py", firmware);
      mp.runPython("import sys\nsys.path.insert(0, '/')");
      mp.runPython(benchSource);
      bench = mp.globals.get("bench");
      bench.set_distance(distance);
      read();
      ui.name.textContent = s.name;
      ui.loading.hidden = true;
      controls.forEach((c) => (c.disabled = false));
      updateUi();
      if (!reduceMotion) {
        setTimeout(() => {
          if (s.state === "IDLE" && !s.manual) send("CLEAN", "sent by the page to start the demo");
        }, 700);
      }
      wake();
    } catch (err) {
      ui.loading.textContent = "The firmware could not start in this browser. The clips show the real robot.";
      console.error(err);
    }
  }

  // ---------------------------------------------------------------
  // Inputs
  // ---------------------------------------------------------------
  let pendingNote = null;

  function send(command, why = "") {
    if (!bench) return;
    if (!s.connected) {
      log(`> ${command}  not sent: the phone is disconnected`, "cmd");
      return;
    }
    const before = s;
    bench.send(command);
    // Explain commands the firmware ignores, once it has handled them.
    pendingNote = () => {
      let note = why;
      if (MOVES.includes(command) && before.autonomous) note = "ignored while cleaning: press Stop cleaning first";
      else if (command === "FAN" && s.fan === before.fan) note = "ignored: the fan switches only at a full stop";
      else if (command === "HORN" && !before.horn && !s.horn) note = "ignored: the horn needs a full stop";
      log(note ? `> ${command}  ${note}` : `> ${command}`, "cmd");
    };
    if (command === "HORN" || command === "FORWARD") audio.unlock();
    wake();
  }

  ui.keys.forEach((key) => key.addEventListener("click", () => send(key.dataset.cmd)));

  const ARROWS = { ArrowUp: "FORWARD", ArrowDown: "BACKWARD", ArrowLeft: "LEFT", ArrowRight: "RIGHT" };
  $("remote").addEventListener("keydown", (e) => {
    const cmd = ARROWS[e.key];
    if (!cmd || e.target === ui.distance || e.altKey || e.ctrlKey || e.metaKey) return;
    e.preventDefault();
    send(cmd);
  });

  function setDistance(cm) {
    distance = Math.round(Math.min(MAX_CM, Math.max(3, cm)));
    ui.distance.value = distance;
    ui.distanceOut.textContent = `${distance} cm`;
    if (bench) bench.set_distance(distance);
    draw();
    wake();
  }
  ui.distance.addEventListener("input", () => setDistance(Number(ui.distance.value)));

  function toggleHold(button, index, label) {
    const held = button.getAttribute("aria-pressed") !== "true";
    button.setAttribute("aria-pressed", String(held));
    bench.hold_wheel(index, held);
    log(held ? `${label} wheel grabbed` : `${label} wheel let go`, "event");
    wake();
  }
  ui.holdLeft.addEventListener("click", () => toggleHold(ui.holdLeft, 0, "Left"));
  ui.holdRight.addEventListener("click", () => toggleHold(ui.holdRight, 1, "Right"));

  function pressLid() {
    if (!bench) return;
    bench.press_button();
    log("Lid button pressed", "event");
    wake();
  }
  ui.lid.addEventListener("click", pressLid);

  ui.link.addEventListener("click", () => {
    if (s.connected) bench.disconnect();
    else bench.connect();
    log(s.connected ? "Phone disconnected" : "Phone connected", "event");
    read();
    updateUi();
    wake();
  });

  // Drag the box along the ruler, or press the button drawn on the lid.
  let dragging = false;
  const onButton = (p) => Math.hypot(p.x - geo.buttonX, p.y - geo.cy) < 4;
  canvas.addEventListener("pointerdown", (e) => {
    if (!bench) return;
    const p = toCm(e);
    if (onButton(p)) {
      pressLid();
    } else if (p.x > geo.sensorX + 2) {
      dragging = true;
      canvas.setPointerCapture(e.pointerId);
      setDistance(p.x - geo.sensorX);
    }
  });
  canvas.addEventListener("pointermove", (e) => {
    const p = toCm(e);
    if (dragging) setDistance(p.x - geo.sensorX);
    canvas.style.cursor = onButton(p) ? "pointer" : p.x > geo.sensorX + 2 ? "ew-resize" : "default";
  });
  canvas.addEventListener("pointerup", () => (dragging = false));
  canvas.addEventListener("pointercancel", () => (dragging = false));

  // ---------------------------------------------------------------
  // Horn sound: a quiet buzz that follows the firmware's LED flash.
  // ---------------------------------------------------------------
  const audio = {
    ctx: null,
    gain: null,
    on: true,
    unlock() {
      if (this.ctx || !this.on || !window.AudioContext) return;
      this.ctx = new AudioContext();
      const osc = this.ctx.createOscillator();
      osc.type = "square";
      osc.frequency.value = 170;
      this.gain = this.ctx.createGain();
      this.gain.gain.value = 0;
      osc.connect(this.gain).connect(this.ctx.destination);
      osc.start();
    },
    set(level) {
      if (this.gain) this.gain.gain.setTargetAtTime(this.on ? level : 0, this.ctx.currentTime, 0.01);
    },
  };

  ui.sound.addEventListener("click", () => {
    audio.on = !audio.on;
    ui.sound.setAttribute("aria-pressed", String(audio.on));
    if (audio.on) audio.unlock();
    else audio.set(0);
  });

  // ---------------------------------------------------------------
  // Drawing, to scale in centimetres: the robot, the ruler, the box.
  // ---------------------------------------------------------------
  const geo = { w: 175, h: 87.5, cx: 26, cy: 43.75, r: 17.5, sensorX: 43.5, buttonX: 17.5 };
  let k = 4; // pixels per centimetre
  let dpr = 1;

  function toCm(e) {
    const rect = canvas.getBoundingClientRect();
    return {
      x: ((e.clientX - rect.left) / rect.width) * geo.w,
      y: ((e.clientY - rect.top) / rect.height) * geo.h,
    };
  }

  function resize() {
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(canvas.clientWidth * dpr);
    canvas.height = Math.round(canvas.clientWidth * (geo.h / geo.w) * dpr);
    k = canvas.width / geo.w;
    draw();
  }

  function draw() {
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = "#f5f5f5";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.setTransform(k, 0, 0, k, 0, 0);
    const { cx, cy, r, sensorX } = geo;
    const near = distance < STOP_CM;
    const objX = sensorX + distance;

    // Stop zone and ruler.
    ctx.fillStyle = near ? "rgba(235, 108, 54, 0.24)" : "rgba(235, 108, 54, 0.1)";
    ctx.fillRect(sensorX, 7, STOP_CM, geo.h - 21);
    // Labels never drop below 10 CSS pixels, even on a phone.
    const label = Math.max(2.6, (10 * dpr) / k);
    ctx.font = `500 ${label}px "Geist Mono", ui-monospace, monospace`;
    ctx.strokeStyle = "#4f5d75";
    ctx.fillStyle = "#4f5d75";
    ctx.lineWidth = 0.25;
    ctx.textAlign = "center";
    const base = geo.h - 10;
    ctx.beginPath();
    ctx.moveTo(sensorX, base);
    ctx.lineTo(sensorX + MAX_CM, base);
    for (let cm = 0; cm <= MAX_CM; cm += 5) {
      const len = cm % 30 === 0 ? 2.4 : 1.2;
      ctx.moveTo(sensorX + cm, base);
      ctx.lineTo(sensorX + cm, base - len);
      if (cm % 30 === 0) ctx.fillText(cm === 0 ? "0 cm" : `${cm}`, sensorX + cm, base + 1.6 + label);
    }
    ctx.stroke();
    ctx.fillStyle = "#a3401a";
    ctx.textAlign = "left";
    ctx.fillText("stop", sensorX + 1.2, base - 3.2);

    // Sonar beam, about 15 degrees wide, out to the box.
    const spread = Math.tan((7.5 * Math.PI) / 180) * distance;
    ctx.beginPath();
    ctx.moveTo(sensorX, cy);
    ctx.lineTo(objX, cy - spread);
    ctx.lineTo(objX, cy + spread);
    ctx.closePath();
    ctx.fillStyle = near ? "rgba(235, 108, 54, 0.45)" : "rgba(79, 93, 117, 0.14)";
    ctx.fill();

    // The box the visitor drags, with a grip.
    ctx.fillStyle = "#e4e6ea";
    ctx.strokeStyle = "#2d3142";
    ctx.lineWidth = 0.35;
    ctx.beginPath();
    ctx.roundRect(objX, cy - 14, 9, 28, 1);
    ctx.fill();
    ctx.stroke();
    ctx.strokeStyle = "#8a91a0";
    ctx.beginPath();
    for (const x of [3.4, 5.6]) {
      ctx.moveTo(objX + x, cy - 4);
      ctx.lineTo(objX + x, cy + 4);
    }
    ctx.stroke();

    // Robot. A horn buzz shakes it slightly.
    const shake = s && s.horn && s.duty[0] > 0 ? (Math.random() - 0.5) * 0.35 : 0;
    ctx.save();
    ctx.translate(cx + shake, cy);

    // Wheel direction arrows beside the shell, longer when faster.
    ctx.strokeStyle = "#eb6c36";
    ctx.lineWidth = 0.7;
    ctx.lineCap = "round";
    for (const i of [0, 1]) {
      const speed = s ? s.speed[i] : 0;
      if (speed < 1) continue;
      const y = i === 0 ? -r - 4 : r + 4;
      const len = 4 + Math.min(speed, 120) / 10;
      const d = s.forward[i] ? 1 : -1;
      ctx.beginPath();
      ctx.moveTo((-len / 2) * d, y);
      ctx.lineTo((len / 2) * d, y);
      ctx.lineTo((len / 2 - 1.8) * d, y - 1.4);
      ctx.moveTo((len / 2) * d, y);
      ctx.lineTo((len / 2 - 1.8) * d, y + 1.4);
      ctx.stroke();
    }

    // Shell.
    ctx.shadowColor = "rgba(45, 49, 66, 0.3)";
    ctx.shadowBlur = 2.5 * k;
    ctx.shadowOffsetY = 0.8 * k;
    ctx.beginPath();
    ctx.arc(0, 0, r, 0, Math.PI * 2);
    ctx.fillStyle = "#23262f";
    ctx.fill();
    ctx.shadowColor = "transparent";

    // Wheels under the shell, shown as an x-ray with moving treads.
    for (const i of [0, 1]) {
      const y = i === 0 ? -13 : 13;
      const held = s && s.held[i];
      ctx.fillStyle = "#3b4054";
      ctx.fillRect(-3.3, y - 1.3, 6.6, 2.6);
      const offset = s ? (((s.turns[i] * 20.7) % 1.6) + 1.6) % 1.6 : 0;
      ctx.strokeStyle = "#6b7285";
      ctx.lineWidth = 0.3;
      ctx.beginPath();
      for (let x = -3.3 + offset; x < 3.3; x += 1.6) {
        ctx.moveTo(x, y - 1.3);
        ctx.lineTo(x, y + 1.3);
      }
      ctx.stroke();
      ctx.strokeStyle = held ? "#eb6c36" : "#8a91a0";
      ctx.lineWidth = held ? 0.6 : 0.25;
      ctx.setLineDash(held ? [] : [0.8, 0.6]);
      ctx.strokeRect(-3.3, y - 1.3, 6.6, 2.6);
      ctx.setLineDash([]);
    }

    // Filter, with the fan turning under it.
    const f = 11;
    ctx.fillStyle = "#b9bec8";
    ctx.fillRect(-f / 2, -f / 2, f, f);
    if (s && s.fan) {
      const a = performance.now() / 90;
      ctx.strokeStyle = "rgba(45, 49, 66, 0.35)";
      ctx.lineWidth = 0.5;
      ctx.beginPath();
      for (let b = 0; b < 5; b++) {
        const t = a + (b * Math.PI * 2) / 5;
        ctx.moveTo(0, 0);
        ctx.lineTo(Math.cos(t) * 4.6, Math.sin(t) * 4.6);
      }
      ctx.stroke();
    }
    ctx.strokeStyle = "#868d9c";
    ctx.lineWidth = 0.2;
    ctx.beginPath();
    for (let i = 1; i < 10; i++) {
      const x = -f / 2 + (f * i) / 10;
      ctx.moveTo(x, -f / 2);
      ctx.lineTo(x, f / 2);
    }
    ctx.stroke();

    // Green status LED in front of the filter, start button behind it.
    ctx.beginPath();
    ctx.arc(f / 2 + 2.2, 0, 1.2, 0, Math.PI * 2);
    ctx.fillStyle = s && s.led ? "#46e07a" : "#2d4a37";
    if (s && s.led) {
      ctx.shadowColor = "rgba(70, 224, 122, 0.9)";
      ctx.shadowBlur = 3 * k;
      ctx.shadowOffsetY = 0;
    }
    ctx.fill();
    ctx.shadowColor = "transparent";
    ctx.beginPath();
    ctx.arc(geo.buttonX - cx, 0, 1.6, 0, Math.PI * 2);
    ctx.fillStyle = "#2fae5a";
    ctx.fill();
    ctx.strokeStyle = "rgba(255, 255, 255, 0.5)";
    ctx.lineWidth = 0.25;
    ctx.stroke();

    // Ultrasonic sensor at the front edge.
    ctx.fillStyle = "#c9ccd3";
    for (const y of [-2.8, 2.8]) {
      ctx.beginPath();
      ctx.arc(r - 1.6, y, 1.6, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  // ---------------------------------------------------------------
  // Readout
  // ---------------------------------------------------------------
  let lastStatus = "";

  function plainStatus() {
    if (!s.connected) return "Phone disconnected, so the firmware stopped the robot. Press Connect to link again.";
    if (s.blocked && s.horn) return "Something is closer than 15 cm, so it refuses to drive forward and honks. Move the box, or turn.";
    if (s.blocked) return "Still blocked. Turn, reverse, or move the box away.";
    if (s.horn) return "Honking: the motors buzz at low power. Press Honk again to stop.";
    if (s.autonomous) {
      const reversing = !s.forward[0] && !s.forward[1] && s.speed[0] + s.speed[1] > 2;
      if (s.state === "STALL_TURN" || (s.state === "FWD" && reversing)) {
        return "A wheel stopped turning for 800 ms, so it reverses and turns away.";
      }
      if (s.state === "AVOID" || s.state === "START_AVOID") {
        return "Something within 15 cm: turning left for 39 wheel pulses. On a stand it can't get away, so it turns again until you move the box.";
      }
      return "Cleaning on its own: driving straight until something is within 15 cm. Press Stop cleaning to drive it yourself.";
    }
    if (s.manual === "FORWARD") return "Driving forward, with the PID keeping both wheels level. Press Drive again to brake.";
    if (s.manual === "BACKWARD") return "Reversing. Press Reverse again to brake.";
    if (s.manual) return `Turning ${s.manual.toLowerCase()} on the spot. Press again to brake.`;
    return "Ready. Press Start cleaning or the lid button, or drive it with the remote.";
  }

  function updateUi() {
    if (!s) return;
    if (pendingNote) {
      pendingNote();
      pendingNote = null;
    }
    const status = plainStatus();
    if (status !== lastStatus) {
      ui.status.textContent = status;
      lastStatus = status;
    }
    ui.state.textContent = s.blocked ? "BLOCKED" : s.manual || s.state;
    ui.power.textContent = `${s.duty[0]} / ${s.duty[1]} %`;
    ui.pulses.textContent = `${s.pulses[0]} / ${s.pulses[1]}`;
    ui.led.textContent = s.ledPattern.replace("_", " ").toLowerCase();
    ui.fan.textContent = s.fan ? "on" : "off";
    ui.link.textContent = s.connected ? "Disconnect" : "Connect";
    ui.link.classList.toggle("is-off", !s.connected);

    for (const key of ui.keys) {
      const cmd = key.dataset.cmd;
      // Dim each key the firmware would ignore right now (handle_command).
      let ignored = !s.connected;
      if (MOVES.includes(cmd)) ignored ||= s.autonomous;
      else if (cmd === "FAN") ignored ||= !s.fullStop;
      else if (cmd === "HORN") ignored ||= !s.fullStop && !s.horn;
      key.setAttribute("aria-disabled", String(ignored));
      let pressed = s.manual === cmd;
      if (cmd === "CLEAN") {
        pressed = s.autonomous;
        key.querySelector(".key-label").textContent = pressed ? "Stop cleaning" : "Start cleaning";
      } else if (cmd === "HORN") {
        pressed = s.horn;
      } else if (cmd === "FAN") {
        pressed = Boolean(s.fan);
      }
      key.setAttribute("aria-pressed", String(pressed));
    }
    audio.set(s.horn && s.led ? 0.04 : 0);
  }

  // ---------------------------------------------------------------
  // Loop: runs only while the stand is on screen and the tab is visible.
  // ---------------------------------------------------------------
  let running = false;
  let visible = false;
  let last = 0;
  let frames = 0;

  function frame(t) {
    if (!running) return;
    const ms = Math.min(Math.max(t - last, 0), 100);
    last = t;
    if (bench && ms >= 1) {
      bench.step(Math.round(ms));
      read();
    }
    draw();
    frames += 1;
    if (frames % 4 === 0 || pendingNote) updateUi();
    requestAnimationFrame(frame);
  }

  function wake() {
    if (running || !visible || document.hidden) return;
    running = true;
    last = performance.now();
    requestAnimationFrame(frame);
  }

  function pause() {
    running = false;
    audio.set(0);
  }

  new IntersectionObserver(([entry]) => {
    visible = entry.isIntersecting;
    if (visible) wake();
    else pause();
  }).observe(canvas);
  document.addEventListener("visibilitychange", () => (document.hidden ? pause() : wake()));
  new ResizeObserver(resize).observe(canvas);

  controls.forEach((c) => (c.disabled = true));
  setDistance(distance);
  start();
})();

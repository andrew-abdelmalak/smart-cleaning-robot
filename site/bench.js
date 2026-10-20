// Runs the robot's real firmware, firmware/main.py, in a drawn room.
//
// MicroPython compiled to WebAssembly executes the firmware file unchanged.
// bench.py fakes the hardware: motors, encoders, Bluetooth, the lid button,
// and the ultrasonic sensor. This file moves the robot through the room from
// the wheel turns the firmware produces, feeds back what the sensor sees and
// when the robot is pushing against furniture, and draws the result.
(() => {
  "use strict";

  // Served by the site itself; scripts/vendor.json pins its version and hash.
  const MICROPYTHON = "./vendor/micropython/micropython.mjs";
  const MOVES = ["FORWARD", "BACKWARD", "LEFT", "RIGHT"];
  const STOP_CM = 15; // OBSTACLE_THRESHOLD_CM in the firmware

  // Robot geometry in metres: body and wheel from the thesis; the track
  // width is assumed. The floor load slows the wheels to about a third of
  // their free speed, chosen to match the pace of the real driving clip.
  const ROBOT_R = 0.175;
  const WHEEL_CIRCUMFERENCE = Math.PI * 0.066;
  const TRACK = 0.26;
  const FLOOR_LOAD = 0.27;
  const NOZZLE_R = 0.1;

  // Room, 4.2 m by 2.36 m, with simple furniture.
  const ROOM_W = 4.2;
  const ROOM_H = 2.3625;
  const RECTS = [
    { x: 0.15, y: 0, w: 1.5, h: 0.74, kind: "sofa" },
    { x: 2.85, y: 0, w: 0.66, h: 0.63, kind: "chair" },
    { x: 3.72, y: 0.87, w: 0.48, h: 1.05, kind: "cabinet" },
  ];
  const CIRCLES = [
    { x: 2.58, y: 1.53, r: 0.32, kind: "table" },
    { x: 0.28, y: 2.08, r: 0.19, kind: "plant" },
  ];

  const $ = (id) => document.getElementById(id);
  const canvas = $("room");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const ui = {
    loading: $("bench-loading"),
    status: $("bench-status"),
    state: $("bench-state"),
    sonar: $("bench-sonar"),
    power: $("bench-power"),
    pulses: $("bench-pulses"),
    led: $("bench-led"),
    fan: $("bench-fan"),
    log: $("bench-log"),
    name: $("bench-name"),
    link: $("bench-link"),
    sound: $("bench-sound"),
    lid: $("lid-button"),
    reset: $("room-reset"),
    caption: $("room-caption"),
    score: $("room-score"),
    keys: [...document.querySelectorAll(".key[data-cmd]")],
  };
  const controls = [ui.link, ui.lid, ui.reset, ...ui.keys];
  const reduceMotion = matchMedia("(prefers-reduced-motion: reduce)").matches;

  let loadBench = null; // imports bench.py again for a freshly booted robot
  let bench = null; // the Python Bench object from bench.py
  let s = null; // latest state from bench.state()
  let robot;
  let dust;
  let dustTotal = 1;
  let trail; // offscreen canvas: the floor the nozzle has passed over
  let trailPoints = [];
  let trailCells = new Set();
  let distanceCm = 999;

  // ---------------------------------------------------------------
  // Room geometry
  // ---------------------------------------------------------------
  function seededRandom(seed) {
    return () => {
      seed = (seed * 1664525 + 1013904223) % 4294967296;
      return seed / 4294967296;
    };
  }

  function hitsFurniture(x, y, r) {
    for (const o of RECTS) {
      const cx = Math.max(o.x, Math.min(x, o.x + o.w));
      const cy = Math.max(o.y, Math.min(y, o.y + o.h));
      if ((x - cx) ** 2 + (y - cy) ** 2 < r * r) return true;
    }
    for (const c of CIRCLES) {
      if ((x - c.x) ** 2 + (y - c.y) ** 2 < (r + c.r) ** 2) return true;
    }
    return false;
  }

  function collides(x, y) {
    if (x - ROBOT_R < 0 || y - ROBOT_R < 0 || x + ROBOT_R > ROOM_W || y + ROBOT_R > ROOM_H) {
      return true;
    }
    return hitsFurniture(x, y, ROBOT_R);
  }

  // Distance from (x, y) along the unit vector (dx, dy) to the nearest surface, in metres.
  function castRay(x, y, dx, dy) {
    let best = Infinity;
    if (dx > 0) best = Math.min(best, (ROOM_W - x) / dx);
    if (dx < 0) best = Math.min(best, -x / dx);
    if (dy > 0) best = Math.min(best, (ROOM_H - y) / dy);
    if (dy < 0) best = Math.min(best, -y / dy);
    for (const o of RECTS) {
      let t0 = -Infinity;
      let t1 = Infinity;
      for (const [p, d, lo, hi] of [
        [x, dx, o.x, o.x + o.w],
        [y, dy, o.y, o.y + o.h],
      ]) {
        if (Math.abs(d) < 1e-9) {
          if (p < lo || p > hi) t0 = Infinity;
        } else {
          const a = (lo - p) / d;
          const b = (hi - p) / d;
          t0 = Math.max(t0, Math.min(a, b));
          t1 = Math.min(t1, Math.max(a, b));
        }
      }
      if (t0 <= t1 && t0 > 0) best = Math.min(best, t0);
    }
    for (const c of CIRCLES) {
      const fx = x - c.x;
      const fy = y - c.y;
      const b = fx * dx + fy * dy;
      const disc = b * b - (fx * fx + fy * fy - c.r * c.r);
      if (disc >= 0) {
        const t = -b - Math.sqrt(disc);
        if (t > 0) best = Math.min(best, t);
      }
    }
    return best;
  }

  // What the HC-SR04 sees: three rays across a 15-degree cone from the front
  // edge. Past its 4 m range there is no echo, which the firmware reads as 999.
  function measureDistance() {
    const sx = robot.x + Math.cos(robot.th) * ROBOT_R;
    const sy = robot.y + Math.sin(robot.th) * ROBOT_R;
    let d = Infinity;
    for (const a of [-0.13, 0, 0.13]) {
      d = Math.min(d, castRay(sx, sy, Math.cos(robot.th + a), Math.sin(robot.th + a)));
    }
    return d > 4 ? 999 : d * 100;
  }

  function resetRoom() {
    robot = { x: 1.0, y: 1.6, th: 0, turns: [0, 0] };
    distanceCm = measureDistance();
    const rand = seededRandom(7);
    dust = [];
    while (dust.length < 2200) {
      const x = rand() * ROOM_W;
      const y = rand() * ROOM_H;
      if (!hitsFurniture(x, y, 0.02)) dust.push({ x, y, s: 0.6 + rand() * 0.9 });
    }
    dustTotal = dust.length;
    trailPoints = [];
    trailCells = new Set();
    paintTrail();
  }

  // ---------------------------------------------------------------
  // Moving the robot from the firmware's wheel turns
  // ---------------------------------------------------------------
  // True when both wheels drive the same way and a 1 cm step that way would
  // hit something: the wheels then stall, as they do against a wall.
  function pushingAgainstFurniture() {
    if (!s || s.duty[0] === 0 || s.duty[1] === 0 || s.forward[0] !== s.forward[1]) return false;
    const step = s.forward[0] ? 0.01 : -0.01;
    return collides(robot.x + Math.cos(robot.th) * step, robot.y + Math.sin(robot.th) * step);
  }

  function moveRobot() {
    const dl = (s.turns[0] - robot.turns[0]) * WHEEL_CIRCUMFERENCE;
    const dr = (s.turns[1] - robot.turns[1]) * WHEEL_CIRCUMFERENCE;
    robot.turns = s.turns.slice();
    const th = robot.th + (dl - dr) / TRACK;
    const v = (dl + dr) / 2;
    const x = robot.x + Math.cos(th) * v;
    const y = robot.y + Math.sin(th) * v;
    robot.th = th;
    // An uneven pivot scrapes along furniture: keep the turn, drop the slide.
    if (!collides(x, y)) {
      robot.x = x;
      robot.y = y;
    }
    if (s.fan) {
      dust = dust.filter((p) => (p.x - robot.x) ** 2 + (p.y - robot.y) ** 2 > NOZZLE_R ** 2);
      // One stamp per 3 cm cell: the list stays bounded by the room's area
      // however long the robot cleans, and the 13 cm stamps hide the grid.
      const cell = `${Math.round(robot.x / 0.03)},${Math.round(robot.y / 0.03)}`;
      if (!trailCells.has(cell)) {
        trailCells.add(cell);
        trailPoints.push({ x: robot.x, y: robot.y });
        stampTrail(robot.x, robot.y);
      }
    }
  }

  // The vacuumed floor shows as a lighter swath, like stripes on a carpet.
  function stampTrail(x, y) {
    const t = trail.getContext("2d");
    t.fillStyle = "#fcfcfd"; // opaque, so overlapping passes don't pile up
    t.beginPath();
    t.arc(x * scale, y * scale, NOZZLE_R * 1.3 * scale, 0, Math.PI * 2);
    t.fill();
  }

  function paintTrail() {
    if (!trail) trail = document.createElement("canvas");
    trail.width = canvas.width;
    trail.height = canvas.height;
    for (const p of trailPoints) stampTrail(p.x, p.y);
  }

  // Short plain-language captions on the map for the moments that matter.
  let captionTimer = 0;
  function caption(text) {
    ui.caption.textContent = text;
    ui.caption.classList.add("is-on");
    clearTimeout(captionTimer);
    captionTimer = setTimeout(() => ui.caption.classList.remove("is-on"), 2600);
  }

  let prev = null;
  function captionEvents() {
    if (prev) {
      const reversing = (x) => x.autonomous && !x.forward[0] && !x.forward[1] && x.speed[0] + x.speed[1] > 2;
      if (s.state === "AVOID" && prev.state !== "AVOID") caption("Obstacle 15 cm ahead: turning left");
      else if (reversing(s) && !reversing(prev) && s.state === "FWD") caption("Wheel stuck: backing out");
      else if (s.blocked && !prev.blocked) caption("Too close to drive forward: honking");
      else if (s.horn && !prev.horn && !s.blocked) caption("Honk!");
      else if (s.autonomous && !prev.autonomous) caption("Cleaning on its own");
      else if (!s.autonomous && prev.autonomous && s.connected) caption("Stopped");
      else if (!s.connected && prev.connected) caption("Phone disconnected: robot stopped");
    }
    prev = s;
  }

  // ---------------------------------------------------------------
  // Log: commands sent, events, and the firmware's own print output
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
      const mp = await loadMicroPython({ stdout: (line) => log(line, "serial") });
      mp.FS.writeFile("/main.py", firmware);
      mp.runPython("import sys\nsys.path.insert(0, '/')");
      loadBench = () => {
        mp.runPython("sys.modules.pop('main', None)"); // the next import boots it afresh
        mp.runPython(benchSource);
        bench = mp.globals.get("bench");
        bench.set_load(FLOOR_LOAD);
        resetRoom();
        bench.set_distance(distanceCm);
        read();
      };
      loadBench();
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
      const message = "The firmware could not start in this browser. The clips show the real robot.";
      ui.loading.textContent = message;
      ui.status.textContent = message; // the status line is announced; the overlay is not
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

  for (const key of ui.keys) key.addEventListener("click", () => send(key.dataset.cmd));

  const ARROWS = { ArrowUp: "FORWARD", ArrowDown: "BACKWARD", ArrowLeft: "LEFT", ArrowRight: "RIGHT" };
  $("remote").addEventListener("keydown", (e) => {
    const cmd = ARROWS[e.key];
    // A held key repeats, and a repeated drive command means brake.
    if (!cmd || e.repeat || e.altKey || e.ctrlKey || e.metaKey) return;
    e.preventDefault();
    send(cmd);
  });

  ui.lid.addEventListener("click", () => {
    bench.press_button();
    log("Lid button pressed", "event");
    caption("Lid button pressed");
    wake();
  });

  ui.reset.addEventListener("click", () => {
    prev = null;
    pendingNote = null;
    loadBench();
    logLines.length = 0;
    log("Room reset and firmware restarted", "event");
    updateUi();
    draw();
    wake(); // restarts the loop if an error stopped it
  });

  ui.link.addEventListener("click", () => {
    if (s.connected) bench.disconnect();
    else bench.connect();
    log(s.connected ? "Phone disconnected" : "Phone connected", "event");
    read();
    updateUi();
    wake();
  });

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
  // Drawing
  // ---------------------------------------------------------------
  let scale = 100;
  let dpr = 1;

  function resize() {
    const w = canvas.clientWidth;
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(w * (ROOM_H / ROOM_W) * dpr);
    scale = (w / ROOM_W) * dpr;
    paintTrail();
    draw();
  }

  function draw() {
    const k = scale;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = "#f5f5f5";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.drawImage(trail, 0, 0);

    // Floor boards every 0.3 m.
    ctx.strokeStyle = "rgba(45, 49, 66, 0.07)";
    ctx.lineWidth = dpr;
    ctx.beginPath();
    for (let y = 0.3; y < ROOM_H; y += 0.3) {
      ctx.moveTo(0, y * k);
      ctx.lineTo(canvas.width, y * k);
    }
    ctx.stroke();

    // Dust.
    ctx.fillStyle = "rgba(79, 93, 117, 0.6)";
    for (const p of dust) {
      const r = p.s * dpr;
      ctx.fillRect(p.x * k - r / 2, p.y * k - r / 2, r, r);
    }

    // Furniture, drawn like a floor plan.
    ctx.lineWidth = 1.25 * dpr;
    ctx.strokeStyle = "#2d3142";
    ctx.fillStyle = "#e4e6ea";
    for (const o of RECTS) {
      ctx.beginPath();
      ctx.roundRect(o.x * k, o.y * k, o.w * k, o.h * k, 4 * dpr);
      ctx.fill();
      ctx.stroke();
      ctx.beginPath();
      if (o.kind === "sofa") {
        ctx.rect(o.x * k, o.y * k, o.w * k, 0.2 * k);
        for (let i = 1; i < 3; i++) {
          const x = (o.x + (o.w * i) / 3) * k;
          ctx.moveTo(x, 0.2 * k);
          ctx.lineTo(x, (o.y + o.h) * k);
        }
      } else if (o.kind === "chair") {
        ctx.rect(o.x * k, o.y * k, o.w * k, 0.18 * k);
      } else {
        ctx.moveTo(o.x * k, (o.y + o.h / 2) * k);
        ctx.lineTo((o.x + o.w) * k, (o.y + o.h / 2) * k);
      }
      ctx.stroke();
    }
    for (const c of CIRCLES) {
      ctx.beginPath();
      ctx.arc(c.x * k, c.y * k, c.r * k, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
      if (c.kind === "plant") {
        ctx.beginPath();
        ctx.arc(c.x * k, c.y * k, c.r * 0.55 * k, 0, Math.PI * 2);
        ctx.stroke();
      }
    }

    // Walls.
    ctx.lineWidth = 4 * dpr;
    ctx.strokeRect(2 * dpr, 2 * dpr, canvas.width - 4 * dpr, canvas.height - 4 * dpr);

    drawRobot(k);
  }

  function drawRobot(k) {
    const near = distanceCm < STOP_CM;
    const buzzing = s && s.horn && s.duty[0] > 0;
    const jx = buzzing ? (Math.random() - 0.5) * 1.6 * dpr : 0;

    ctx.save();
    ctx.translate(robot.x * k + jx, robot.y * k);
    ctx.rotate(robot.th);
    const r = ROBOT_R * k;

    // Sonar cone out to what the sensor measures (drawn up to 1.2 m).
    const reach = (Math.min(distanceCm, 120) / 100) * k;
    ctx.beginPath();
    ctx.moveTo(r, 0);
    ctx.lineTo(r + reach * Math.cos(0.13), reach * Math.sin(0.13));
    ctx.lineTo(r + reach * Math.cos(0.13), -reach * Math.sin(0.13));
    ctx.closePath();
    ctx.fillStyle = near ? "rgba(235, 108, 54, 0.45)" : "rgba(79, 93, 117, 0.13)";
    ctx.fill();

    // Body with a soft offset shadow.
    ctx.shadowColor = "rgba(45, 49, 66, 0.3)";
    ctx.shadowBlur = 8 * dpr;
    ctx.shadowOffsetY = 3 * dpr;
    ctx.beginPath();
    ctx.arc(0, 0, r, 0, Math.PI * 2);
    ctx.fillStyle = "#23262f";
    ctx.fill();
    ctx.shadowColor = "transparent";
    ctx.lineWidth = dpr;
    ctx.strokeStyle = "rgba(255, 255, 255, 0.12)";
    ctx.beginPath();
    ctx.arc(0, 0, r * 0.93, 0, Math.PI * 2);
    ctx.stroke();

    // Filter grille on the lid, with the fan turning under it.
    const f = r * 0.62;
    ctx.fillStyle = "#b9bec8";
    ctx.fillRect(-f / 2, -f / 2, f, f);
    if (s && s.fan) {
      const a = performance.now() / 90;
      ctx.strokeStyle = "rgba(45, 49, 66, 0.35)";
      ctx.lineWidth = 1.5 * dpr;
      ctx.beginPath();
      for (let b = 0; b < 5; b++) {
        const t = a + (b * Math.PI * 2) / 5;
        ctx.moveTo(0, 0);
        ctx.lineTo(Math.cos(t) * f * 0.42, Math.sin(t) * f * 0.42);
      }
      ctx.stroke();
    }
    ctx.strokeStyle = "#868d9c";
    ctx.lineWidth = dpr;
    ctx.beginPath();
    for (let i = 1; i < 8; i++) {
      const x = -f / 2 + (f * i) / 8;
      ctx.moveTo(x, -f / 2);
      ctx.lineTo(x, f / 2);
    }
    ctx.stroke();

    // Green status LED at the front, start button at the back.
    ctx.beginPath();
    ctx.arc(f / 2 + r * 0.12, 0, r * 0.07, 0, Math.PI * 2);
    ctx.fillStyle = s && s.led ? "#46e07a" : "#2d4a37";
    if (s && s.led) {
      ctx.shadowColor = "rgba(70, 224, 122, 0.8)";
      ctx.shadowBlur = 10 * dpr;
      ctx.shadowOffsetY = 0;
    }
    ctx.fill();
    ctx.shadowColor = "transparent";
    ctx.beginPath();
    ctx.arc(-f / 2 - r * 0.12, 0, r * 0.05, 0, Math.PI * 2);
    ctx.fillStyle = "#2fae5a";
    ctx.fill();

    // Ultrasonic sensor on the front edge.
    ctx.fillStyle = "#c9ccd3";
    for (const y of [-r * 0.16, r * 0.16]) {
      ctx.beginPath();
      ctx.arc(r * 0.9, y, r * 0.09, 0, Math.PI * 2);
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
    if (s.blocked && s.horn) return "Something is closer than 15 cm, so it refuses to drive forward and honks. Turn or reverse.";
    if (s.blocked) return "Still blocked. Turn or reverse away from the obstacle.";
    if (s.horn) return "Honking: the motors buzz at low power. Press Honk again to stop.";
    if (s.autonomous) {
      const reversing = !s.forward[0] && !s.forward[1] && s.speed[0] + s.speed[1] > 2;
      if (s.state === "STALL_TURN" || (s.state === "FWD" && reversing)) {
        return "A wheel stopped turning for 800 ms, so it backs up and turns away.";
      }
      if (s.state === "AVOID" || s.state === "START_AVOID") {
        return "Something within 15 cm: turning left for 39 wheel pulses, then carrying on.";
      }
      return "Cleaning on its own: straight ahead until something is within 15 cm. Press Stop cleaning to drive it yourself.";
    }
    if (s.manual === "FORWARD") return "Driving forward, with the PID keeping both wheels level. Press Drive again to brake.";
    if (s.manual === "BACKWARD") return "Reversing. Press Reverse again to brake.";
    if (s.manual) return `Turning ${s.manual.toLowerCase()} on the spot. Press again to brake.`;
    return "Ready. Press Start cleaning or the lid button, or drive it with the remote.";
  }

  function updateUi() {
    if (!s) return;
    // Explain a command only after the firmware has read it.
    if (pendingNote && !s.pending) {
      pendingNote();
      pendingNote = null;
    }
    const status = plainStatus();
    if (status !== lastStatus) {
      ui.status.textContent = status;
      lastStatus = status;
    }
    ui.state.textContent = s.blocked ? "BLOCKED" : s.manual || s.state;
    ui.sonar.textContent = distanceCm >= 999 ? "no echo" : `${Math.round(distanceCm)} cm`;
    ui.sonar.classList.toggle("is-near", distanceCm < STOP_CM);
    ui.power.textContent = `${s.duty[0]} / ${s.duty[1]} %`;
    ui.pulses.textContent = `${s.pulses[0]} / ${s.pulses[1]}`;
    ui.led.textContent = s.ledPattern.replace("_", " ").toLowerCase();
    ui.fan.textContent = s.fan ? "on" : "off";
    ui.score.textContent = `${Math.round((1 - dust.length / dustTotal) * 100)} %`;
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
        // The label carries the state, so a screen reader hears it once,
        // not as "Stop cleaning, pressed".
        key.querySelector(".key-label").textContent = s.autonomous ? "Stop cleaning" : "Start cleaning";
        key.classList.toggle("is-on", s.autonomous);
        continue;
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
  // Loop: runs only while the room is on screen and the tab is visible.
  // ---------------------------------------------------------------
  let running = false;
  let raf = 0;
  let visible = false;
  let last = 0;
  let carry = 0; // fractions of a millisecond, so the pace is the same at any refresh rate
  let frames = 0;

  function frame(t) {
    if (!running) return;
    try {
      carry += Math.min(Math.max(t - last, 0), 100);
      last = t;
      const ms = Math.floor(carry);
      carry -= ms;
      if (bench && ms >= 1) {
        distanceCm = measureDistance();
        bench.set_distance(distanceCm >= 999 ? null : distanceCm);
        const stuck = pushingAgainstFurniture();
        bench.hold_wheel(0, stuck);
        bench.hold_wheel(1, stuck);
        bench.step(ms);
        read();
        moveRobot();
        captionEvents();
      }
      draw();
      frames += 1;
      if (frames % 4 === 0 || pendingNote) updateUi();
    } catch (err) {
      // Stop and say so rather than freeze silently; Reset room starts it again.
      running = false;
      ui.status.textContent = "The simulation stopped with an error. Press Reset room to start again.";
      console.error(err);
      return;
    }
    raf = requestAnimationFrame(frame);
  }

  function wake() {
    if (running || !visible || document.hidden) return;
    running = true;
    last = performance.now();
    audio.ctx?.resume();
    raf = requestAnimationFrame(frame);
  }

  function pause() {
    running = false;
    cancelAnimationFrame(raf); // a pending frame would start a second loop
    audio.set(0);
    audio.ctx?.suspend();
  }

  // Watch the whole remote, not just the room: on a phone the keys sit below
  // the room, and the firmware must keep running while they are on screen.
  new IntersectionObserver((entries) => {
    visible = entries[entries.length - 1].isIntersecting;
    if (visible) wake();
    else pause();
  }).observe($("remote"));
  document.addEventListener("visibilitychange", () => (document.hidden ? pause() : wake()));
  new ResizeObserver(resize).observe(canvas);

  controls.forEach((c) => (c.disabled = true));
  resetRoom();
  start();
})();

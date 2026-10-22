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
  // Copied from firmware/main.py for the page's text and the signal trace;
  // tests/test_site.py checks that they still match.
  const FW = {
    OBSTACLE_THRESHOLD_CM: 15,
    TURN_PULSES: 39,
    STALL_TIME_MS: 800,
    STALL_REVERSE_MS: 700,
    BRAKE_DURATION_MS: 50,
  };
  const STOP_CM = FW.OBSTACLE_THRESHOLD_CM;
  const TRACE_MS = 6000; // the signal trace shows this much firmware time

  // Robot geometry in metres: body and wheel from the thesis; the track
  // width is assumed. The floor load slows the wheels to about a third of
  // their free speed, chosen to match the pace of the real driving clip.
  const ROBOT_R = 0.175;
  const WHEEL_CIRCUMFERENCE = Math.PI * 0.066;
  const TRACK = 0.26;
  const FLOOR_LOAD = 0.27;
  const NOZZLE_R = 0.1;

  // A 4.8 m by 2.7 m living room. The layout and start were chosen by
  // running the firmware in thousands of realistic rooms and keeping the
  // one with the most floor cleaned, the least driving over cleaned floor,
  // and the fewest stalls, averaged over runs with slightly different
  // timing. Furniture is flush with a wall or leaves room for the robot.
  const ROOM_W = 4.8;
  const ROOM_H = 2.7;
  const RECTS = [
    { x: 3.0, y: 1.8, w: 1.8, h: 0.9, kind: "sofa" },
    { x: 3.24, y: 0.76, w: 0.96, h: 0.5, kind: "table" },
    { x: 0, y: 0, w: 0.8, h: 0.8, kind: "chair" },
  ];
  const CIRCLES = [{ x: 4.65, y: 0.15, r: 0.15, kind: "plant" }];
  const START = { x: 0.76, y: 1.59, th: 0 };

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
    speed: $("room-speed"),
    caption: $("room-caption"),
    score: $("room-score"),
    keys: [...document.querySelectorAll(".key[data-cmd]")],
    scope: $("scope"),
    stepMode: $("step-mode"),
    next: $("step-next"),
    states: [...document.querySelectorAll(".fsm [data-state]")],
    gears: [$("gear-left"), $("gear-right")],
    motors: [$("motor-left"), $("motor-right")],
  };
  const controls = [ui.link, ui.lid, ui.reset, ui.speed, ui.stepMode, ui.next, ...ui.keys];
  // Fast-forward: the simulated clock runs this many times faster than real
  // time. The firmware and the physics are the same; only the clock changes.
  let timeScale = 1;
  let stepMode = false; // the firmware runs one loop pass per press of ui.next
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

  // What the HC-SR04 sees: five rays across its 30-degree measuring angle
  // (datasheet) from the front edge. Past its 4 m range there is no echo,
  // which the firmware reads as 999.
  function measureDistance() {
    const sx = robot.x + Math.cos(robot.th) * ROBOT_R;
    const sy = robot.y + Math.sin(robot.th) * ROBOT_R;
    let d = Infinity;
    for (const a of [-0.26, -0.13, 0, 0.13, 0.26]) {
      d = Math.min(d, castRay(sx, sy, Math.cos(robot.th + a), Math.sin(robot.th + a)));
    }
    return d > 4 ? 999 : d * 100;
  }

  // True when the nozzle can pass over (x, y): some robot position that fits
  // in the room lies within NOZZLE_R of it. The thin strips along the walls
  // and furniture that no robot this size can reach get no dust, so the
  // score counts the floor it could clean.
  function nozzleReaches(x, y) {
    if (!collides(x, y)) return true;
    for (const rr of [0.05, 0.095]) {
      for (let i = 0; i < 16; i++) {
        const a = (i / 16) * Math.PI * 2;
        if (!collides(x + rr * Math.cos(a), y + rr * Math.sin(a))) return true;
      }
    }
    return false;
  }

  function resetRoom() {
    robot = { ...START, turns: [0, 0] };
    distanceCm = measureDistance();
    const rand = seededRandom(7);
    dust = [];
    while (dust.length < 2200) {
      const x = rand() * ROOM_W;
      const y = rand() * ROOM_H;
      const s = 0.6 + rand() * 0.9;
      if (!hitsFurniture(x, y, 0.02) && nozzleReaches(x, y)) dust.push({ x, y, s });
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
      if (s.state === "AVOID" && prev.state !== "AVOID") caption(`Obstacle ${STOP_CM} cm ahead: turning left`);
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
        clearTrace();
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
    if (stepMode) {
      // The firmware reads the command on its next pass; until then only
      // the latest one waits, as in last_ble_command.
      log(`> ${command}  waiting for the next loop pass`, "cmd");
      read();
      updateUi();
    }
    wake();
  }

  for (const key of ui.keys) key.addEventListener("click", () => send(key.dataset.cmd));

  const ARROWS = { ArrowUp: "FORWARD", ArrowDown: "BACKWARD", ArrowLeft: "LEFT", ArrowRight: "RIGHT" };
  $("remote").addEventListener("keydown", (e) => {
    const cmd = ARROWS[e.key];
    // A held key repeats, and a repeated drive command means brake.
    if (!cmd || e.repeat || e.altKey || e.ctrlKey || e.metaKey) return;
    if (e.target.matches("input") || e.target.closest(".scope-panel")) return;
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

  ui.speed.addEventListener("change", () => {
    timeScale = ui.speed.checked ? 4 : 1;
    log(timeScale > 1 ? "Running at 4× speed" : "Running at real speed", "event");
  });

  ui.stepMode.addEventListener("change", () => {
    stepMode = ui.stepMode.checked;
    ui.next.hidden = !stepMode;
    log(stepMode ? "Step by step: one control-loop pass per press" : "Running continuously", "event");
    if (stepMode) {
      pause();
      updateUi();
    } else {
      wake();
    }
  });

  ui.next.addEventListener("click", () => {
    try {
      const before = s.t;
      advance(() => bench.step_once());
      log(`One loop pass: ${s.t - before} ms of firmware time`, "event");
      updateUi();
      draw();
    } catch (err) {
      fail(err);
    }
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

  ui.sound.addEventListener("change", () => {
    audio.on = ui.sound.checked;
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
    drawScope();
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
      if (o.kind === "sofa" || o.kind === "chair") {
        // Backrest along the wall the seat stands against; a sofa also
        // shows its seat cushions.
        const back = o.kind === "sofa" ? 0.2 : 0.18;
        const onBottom = o.y + o.h >= ROOM_H - 0.01;
        const by = onBottom ? o.y + o.h - back : o.y;
        ctx.rect(o.x * k, by * k, o.w * k, back * k);
        if (o.kind === "sofa") {
          const y0 = onBottom ? o.y : o.y + back;
          for (let i = 1; i < 3; i++) {
            const x = (o.x + (o.w * i) / 3) * k;
            ctx.moveTo(x, y0 * k);
            ctx.lineTo(x, (y0 + o.h - back) * k);
          }
        }
      } else if (o.kind === "table") {
        const inset = 0.06;
        ctx.rect((o.x + inset) * k, (o.y + inset) * k, (o.w - 2 * inset) * k, (o.h - 2 * inset) * k);
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
    ctx.lineTo(r + reach * Math.cos(0.26), reach * Math.sin(0.26));
    ctx.lineTo(r + reach * Math.cos(0.26), -reach * Math.sin(0.26));
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
  // Signal trace: what the firmware wrote to the motor pins, the encoder
  // pulses its interrupt handlers counted, the distance the sensor saw,
  // and its state changes, over the last TRACE_MS of firmware time.
  // ---------------------------------------------------------------
  let trace;

  function clearTrace() {
    const signed = s.duty.map((d, i) => (s.forward[i] ? d : -d));
    trace = { pwm: [[s.t, ...signed]], pulses: [[], []], sonar: [], states: [] };
    readTrace();
  }

  function readTrace() {
    for (const e of JSON.parse(bench.trace())) {
      if (e[0] === "pwm") trace.pwm.push([e[1], e[2], e[3]]);
      else if (e[0] === "pulse") trace.pulses[e[2]].push(e[1]);
      else trace.states.push([e[1], e[2]]);
    }
  }

  function recordTrace() {
    readTrace();
    trace.sonar.push([s.t, distanceCm]);
    // Keep the last value before the window, so lines start at its left edge.
    const from = s.t - TRACE_MS;
    const trim = (list, time) => {
      let i = 0;
      while (i < list.length - 1 && time(list[i + 1]) < from) i++;
      if (i) list.splice(0, i);
    };
    trim(trace.pwm, (e) => e[0]);
    trim(trace.sonar, (e) => e[0]);
    trim(trace.states, (e) => e[0]);
    for (const p of trace.pulses) trim(p, (e) => e);
  }

  const SCOPE_ROWS = [
    ["PWM left", 52],
    ["PWM right", 52],
    ["Encoder left", 22],
    ["Encoder right", 22],
    ["Sonar", 52],
  ];
  const SCOPE_TOP = 40;
  const SCOPE_H = SCOPE_TOP + SCOPE_ROWS.reduce((n, [, h]) => n + h + 6, 0) + 14;
  const SONAR_MAX_CM = 100;
  ui.scope.style.height = `${SCOPE_H}px`;

  function drawScope() {
    const c = ui.scope;
    const W = c.clientWidth;
    if (!W || !trace) return;
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    if (c.width !== Math.round(W * ratio) || c.height !== Math.round(SCOPE_H * ratio)) {
      c.width = Math.round(W * ratio);
      c.height = Math.round(SCOPE_H * ratio);
    }
    const g = c.getContext("2d");
    g.setTransform(ratio, 0, 0, ratio, 0, 0);
    g.clearRect(0, 0, W, SCOPE_H);
    g.font = "11px 'Geist Mono', ui-monospace, monospace";
    g.textBaseline = "middle";
    const left = Math.max(...SCOPE_ROWS.map(([name]) => g.measureText(name).width)) + 12;
    const t1 = s.t;
    const t0 = t1 - TRACE_MS;
    const x = (t) => left + ((t - t0) / TRACE_MS) * (W - left - 6);

    // Rows: a label, a baseline, and the row's top and bottom.
    const rows = [];
    let y = SCOPE_TOP;
    for (const [name, h] of SCOPE_ROWS) {
      rows.push({ name, top: y, h });
      g.fillStyle = "#bcc2d1";
      g.fillText(name, 0, y + h / 2);
      y += h + 6;
    }
    g.fillStyle = "rgba(188, 194, 209, 0.75)";
    g.fillText(`last ${TRACE_MS / 1000} s of firmware time`, left, SCOPE_H - 6);

    // State changes: dashed marks with their names in up to three lanes.
    const laneEnd = [-Infinity, -Infinity, -Infinity];
    for (const [t, label] of trace.states) {
      if (t < t0) {
        // The state the window opens in, from before it.
        g.fillStyle = "#bcc2d1";
        g.fillText(label, left, 6);
        laneEnd[0] = left + g.measureText(label).width + 6;
        continue;
      }
      const w = g.measureText(label).width;
      const lx = Math.min(x(t) + 3, W - w - 2);
      // A one-pass state such as START_AVOID often sits a few pixels from
      // the next; with no free lane its mark stays and its name is left out.
      const lane = laneEnd.findIndex((end) => lx > end);
      g.strokeStyle = "rgba(245, 245, 245, 0.22)";
      g.setLineDash([3, 3]);
      g.beginPath();
      g.moveTo(x(t), 4 + Math.max(lane, 0) * 12);
      g.lineTo(x(t), y - 4);
      g.stroke();
      g.setLineDash([]);
      if (lane < 0) continue;
      laneEnd[lane] = lx + w + 6;
      g.fillStyle = "#f5f5f5";
      g.fillText(label, lx, 6 + lane * 12);
    }

    // PWM duty, above the midline driving forward, below it reversing.
    for (const wheel of [0, 1]) {
      const { top, h } = rows[wheel];
      const mid = top + h / 2;
      g.strokeStyle = "rgba(245, 245, 245, 0.14)";
      g.beginPath();
      g.moveTo(left, mid);
      g.lineTo(W - 6, mid);
      g.stroke();
      g.strokeStyle = "#dfe2ea";
      g.lineWidth = 1.5;
      g.beginPath();
      let prevY = null;
      for (let i = 0; i < trace.pwm.length; i++) {
        const [t, ...duty] = trace.pwm[i];
        const px = Math.max(x(t), left);
        const py = mid - (duty[wheel] / 100) * (h / 2 - 2);
        if (prevY === null) {
          g.moveTo(px, py);
        } else {
          g.lineTo(px, prevY); // duty holds until the next write
          g.lineTo(px, py);
        }
        prevY = py;
      }
      if (prevY !== null) g.lineTo(W - 6, prevY);
      g.stroke();
      g.lineWidth = 1;
    }

    // Encoder pulses: one tick per rising edge the firmware counted.
    g.strokeStyle = "#dfe2ea";
    for (const wheel of [0, 1]) {
      const { top, h } = rows[2 + wheel];
      g.beginPath();
      for (const t of trace.pulses[wheel]) {
        if (t < t0) continue;
        g.moveTo(x(t), top + 3);
        g.lineTo(x(t), top + h - 3);
      }
      g.stroke();
    }

    // Distance the sensor saw, against the firmware's stop threshold.
    const sonar = rows[4];
    const sy = (cm) => sonar.top + sonar.h - (Math.min(cm, SONAR_MAX_CM) / SONAR_MAX_CM) * (sonar.h - 6);
    g.strokeStyle = "rgba(255, 154, 107, 0.7)";
    g.setLineDash([4, 3]);
    g.beginPath();
    g.moveTo(left, sy(STOP_CM));
    g.lineTo(W - 6, sy(STOP_CM));
    g.stroke();
    g.setLineDash([]);
    g.fillStyle = "#ff9a6b";
    g.fillText(`stop ${STOP_CM} cm`, 0, sy(STOP_CM));
    // Held from one frame's reading to the next; a gap where no echo returned.
    g.strokeStyle = "#46e07a";
    g.lineWidth = 1.5;
    g.beginPath();
    let held = null;
    for (const [t, cm] of trace.sonar) {
      const px = Math.max(x(t), left);
      if (held !== null) g.lineTo(px, held);
      held = cm >= 999 ? null : sy(cm);
      if (held !== null) g.moveTo(px, held);
    }
    if (held !== null) g.lineTo(W - 6, held);
    g.stroke();
    g.lineWidth = 1;
  }

  // The state machine, the motors, and their gears, from the firmware state.
  function updateSignals() {
    const label = s.blocked ? "BLOCKED" : s.horn ? "HORN" : s.manual || s.state;
    for (const el of ui.states) {
      if (el.dataset.state === label) el.setAttribute("aria-current", "step");
      else el.removeAttribute("aria-current");
    }
    // Wheel duty as the motor driver sees it: left on ENB, right on ENA.
    s.duty.forEach((duty, i) => {
      const dir = duty === 0 ? "off" : s.forward[i] ? "forward" : "reverse";
      ui.motors[i].textContent = duty === 0 ? "off" : `${duty} % ${dir}`;
      ui.motors[i].closest(".motor").dataset.on = String(duty > 0);
    });
  }

  function turnGears() {
    if (reduceMotion) return;
    // The gearbox output turns with the wheel: one turn per 20 encoder pulses.
    robot.turns.forEach((turns, i) => (ui.gears[i].style.transform = `rotate(${(turns * 360) % 360}deg)`));
  }

  // ---------------------------------------------------------------
  // Readout
  // ---------------------------------------------------------------
  let lastStatus = "";

  function plainStatus() {
    if (!s.connected) return "Phone disconnected, so the firmware stopped the robot. Press Connect to link again.";
    if (s.blocked && s.horn) return `Something is closer than ${STOP_CM} cm, so it refuses to drive forward and honks. Turn or reverse.`;
    if (s.blocked) return "Still blocked. Turn or reverse away from the obstacle.";
    if (s.horn) return "Honking: the motors buzz at low power. Press Honk again to stop.";
    if (s.autonomous) {
      const reversing = !s.forward[0] && !s.forward[1] && s.speed[0] + s.speed[1] > 2;
      if (s.state === "STALL_TURN" || (s.state === "FWD" && reversing)) {
        return `A wheel stopped turning for ${FW.STALL_TIME_MS} ms, so it backs up for ${FW.STALL_REVERSE_MS} ms and turns away.`;
      }
      if (s.state === "AVOID" || s.state === "START_AVOID") {
        return `Something within ${STOP_CM} cm: braking, then turning left for ${FW.TURN_PULSES} wheel pulses.`;
      }
      return `Cleaning on its own: straight ahead until something is within ${STOP_CM} cm. Press Stop cleaning to drive it yourself.`;
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
    audio.set(running && s.horn && s.led ? 0.04 : 0);
    updateSignals();
    if (stepMode) {
      drawScope();
      turnGears();
    }
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
      if (bench && ms >= 1) advance(() => bench.step(ms * timeScale));
      draw();
      drawScope();
      turnGears();
      frames += 1;
      if (frames % 4 === 0 || pendingNote) updateUi();
    } catch (err) {
      fail(err);
      return;
    }
    raf = requestAnimationFrame(frame);
  }

  // Stop and say so rather than freeze silently; Reset room starts it again.
  function fail(err) {
    running = false;
    ui.status.textContent = "The simulation stopped with an error. Press Reset room to start again.";
    console.error(err);
  }

  // One step of the world around the firmware: what the sensor sees and
  // whether the wheels are blocked go in, the firmware runs, the robot moves.
  function advance(run) {
    distanceCm = measureDistance();
    bench.set_distance(distanceCm >= 999 ? null : distanceCm);
    const stuck = pushingAgainstFurniture();
    bench.hold_wheel(0, stuck);
    bench.hold_wheel(1, stuck);
    run();
    read();
    moveRobot();
    captionEvents();
    recordTrace();
  }

  function wake() {
    if (running || stepMode || !visible || document.hidden) return;
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
  new ResizeObserver(() => drawScope()).observe(ui.scope);

  controls.forEach((c) => (c.disabled = true));
  resetRoom();
  start();
})();

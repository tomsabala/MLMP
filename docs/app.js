/* MLMP trace player. Replays committed mlmp.traceviz/1 fixtures; no build step, no deps.
 *
 * Rendering rules are mirrored verbatim from mlmp/tools/traceGif.py so the page and the README
 * GIF cannot disagree. Forward kinematics follow KinematicChainValidityChecker
 * (mlmp/src/state/kinematic_chain.h:199-222): cumulative angles, index q[link*robots+robot],
 * one constant jointLength per robot. A fixture vertex at bundle level L stores only its first
 * L+1 links per robot; the tail above L is identity-fibre padding and is never drawn.
 *
 * A fixture stores every vertex and edge once, in order of first appearance, and each frame is
 * [t_s, vertexCount, edgeCount] — so frame k is the prefix vertices[:nv] / edges[:ne]. See
 * mlmp/tools/packTrace.py for why the raw mlmp.trace/1 stream cannot be served directly. */
'use strict';

const TREE_COLORS = {
  start: '#1f77b4',
  goal: '#d62728',
  section: '#9467bd',
  untagged: '#7f7f7f',
};
const PADDING = 0.05;

const el = {
  scene: document.getElementById('scene'),
  play: document.getElementById('play'),
  prev: document.getElementById('prev'),
  next: document.getElementById('next'),
  slider: document.getElementById('slider'),
  counter: document.getElementById('counter'),
  levels: document.getElementById('levels'),
  legend: document.getElementById('legend'),
};

const panels = [
  { planner: 'biqrrt', canvas: document.getElementById('canvasA'), caption: document.getElementById('captionA') },
  { planner: 'qrrt', canvas: document.getElementById('canvasB'), caption: document.getElementById('captionB') },
];

const state = {
  scene: null,
  bounds: null,
  frame: 0,
  total: 1,
  playing: false,
  levels: new Set(),
  intervalMs: 50,
  lastTick: 0,
  accumulator: 0,
  token: 0,
};

function hexToRgba(hex, alpha) {
  const r = parseInt(hex.slice(1, 3), 16);
  const g = parseInt(hex.slice(3, 5), 16);
  const b = parseInt(hex.slice(5, 7), 16);
  return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

async function loadJson(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${url}: ${response.status}`);
  return response.json();
}

function chainTips(q, level, robots, sceneRobots) {
  const tips = [];
  for (let r = 0; r < sceneRobots.length; r++) {
    const robot = sceneRobots[r];
    let x = robot.pinnedPosition.x;
    let y = robot.pinnedPosition.y;
    const length = robot.jointLength;
    let theta = 0;
    for (let j = 0; j <= level; j++) {
      const idx = j * robots + r;
      if (idx >= q.length) break;
      theta += q[idx];
      x += length * Math.cos(theta);
      y += length * Math.sin(theta);
    }
    tips.push([x, y]);
  }
  return tips;
}

function chainJoints(robot, anglesDeg) {
  let x = robot.pinnedPosition.x;
  let y = robot.pinnedPosition.y;
  const length = robot.jointLength;
  let theta = 0;
  const pts = [[x, y]];
  for (const a of anglesDeg) {
    theta += (a * Math.PI) / 180;
    x += length * Math.cos(theta);
    y += length * Math.sin(theta);
    pts.push([x, y]);
  }
  return pts;
}

function sceneBounds(scene, links) {
  const xs = [];
  const ys = [];
  for (const obstacle of scene.obstacles) {
    for (const p of obstacle.points) {
      xs.push(p.x);
      ys.push(p.y);
    }
  }
  for (const robot of scene.robots) {
    const reach = links * robot.jointLength;
    xs.push(robot.pinnedPosition.x - reach, robot.pinnedPosition.x + reach);
    ys.push(robot.pinnedPosition.y - reach, robot.pinnedPosition.y + reach);
  }
  return {
    minX: Math.min(...xs) - PADDING,
    maxX: Math.max(...xs) + PADDING,
    minY: Math.min(...ys) - PADDING,
    maxY: Math.max(...ys) + PADDING,
  };
}

/* Frame-independent work done once per fixture: FK per vertex, colour per tree, alpha per
 * level. Playback then only slices these arrays. */
function preparePanel(packed, scene, maxLevel) {
  const codes = packed.tree_codes || {};
  const robots = packed.robots;
  const vertices = packed.vertices.map((row) => {
    const level = row[0];
    const tree = codes[String(row[1])] || 'untagged';
    return {
      level,
      tree,
      alpha: 0.35 + (0.65 * level) / Math.max(1, maxLevel),
      tips: chainTips(row.slice(2), level, robots, scene.robots),
    };
  });
  return { packed, vertices, frames: packed.frames, run: packed.run || {} };
}

function sizeCanvas(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const w = Math.max(1, Math.round(rect.width));
  const h = Math.max(1, Math.round(rect.height || rect.width));
  canvas.width = Math.round(w * ratio);
  canvas.height = Math.round(h * ratio);
  return { w, h, ratio };
}

/* Equal-aspect scene -> pixel map; y is flipped because canvas y grows downward. */
function projector(bounds, w, h) {
  const scale = Math.min(w / (bounds.maxX - bounds.minX), h / (bounds.maxY - bounds.minY));
  const offsetX = (w - (bounds.maxX - bounds.minX) * scale) / 2;
  const offsetY = (h - (bounds.maxY - bounds.minY) * scale) / 2;
  return (x, y) => [
    offsetX + (x - bounds.minX) * scale,
    h - offsetY - (y - bounds.minY) * scale,
  ];
}

function drawStatic(ctx, project, scene) {
  ctx.lineWidth = 1;
  for (const obstacle of scene.obstacles) {
    ctx.beginPath();
    obstacle.points.forEach((p, i) => {
      const [px, py] = project(p.x, p.y);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.closePath();
    ctx.fillStyle = 'rgba(128, 128, 128, 0.5)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.stroke();
  }
  for (const robot of scene.robots) {
    for (const angles of [robot.startAngles, robot.goalAngles]) {
      const pts = chainJoints(robot, angles);
      ctx.beginPath();
      pts.forEach(([x, y], i) => {
        const [px, py] = project(x, y);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.strokeStyle = 'rgba(0, 0, 0, 0.25)';
      ctx.stroke();
      for (const [x, y] of pts) {
        const [px, py] = project(x, y);
        ctx.beginPath();
        ctx.arc(px, py, 1.6, 0, 2 * Math.PI);
        ctx.fillStyle = 'rgba(0, 0, 0, 0.25)';
        ctx.fill();
      }
    }
  }
}

function drawFrame(ctx, project, panel, index) {
  const [, nv, ne] = panel.frames[index];
  const vertices = panel.vertices;

  ctx.lineWidth = 0.8;
  const edges = panel.packed.edges;
  for (let i = 0; i < ne && i < edges.length; i++) {
    const va = vertices[edges[i][0]];
    const vb = vertices[edges[i][1]];
    if (!va || !vb) continue;
    if (!state.levels.has(va.level) || !state.levels.has(vb.level)) continue;
    ctx.strokeStyle = hexToRgba(TREE_COLORS[va.tree] || TREE_COLORS.untagged, 0.25);
    for (let r = 0; r < va.tips.length && r < vb.tips.length; r++) {
      const [x0, y0] = project(va.tips[r][0], va.tips[r][1]);
      const [x1, y1] = project(vb.tips[r][0], vb.tips[r][1]);
      ctx.beginPath();
      ctx.moveTo(x0, y0);
      ctx.lineTo(x1, y1);
      ctx.stroke();
    }
  }

  /* `section` last so the findSection() configurations stay legible on top of both trees. */
  for (const sectionLayer of [false, true]) {
    for (let i = 0; i < nv && i < vertices.length; i++) {
      const v = vertices[i];
      if (!state.levels.has(v.level) || (v.tree === 'section') !== sectionLayer) continue;
      ctx.fillStyle = hexToRgba(TREE_COLORS[v.tree] || TREE_COLORS.untagged, v.alpha);
      const radius = v.tree === 'section' ? 2.4 : 1.5;
      for (const [x, y] of v.tips) {
        const [px, py] = project(x, y);
        ctx.beginPath();
        ctx.arc(px, py, radius, 0, 2 * Math.PI);
        ctx.fill();
      }
    }
  }
}

function captionOf(panel, index) {
  const run = panel.run;
  const bits = [panel.packed.planner, run.status || '?'];
  bits.push(run.solved ? 'solved' : 'unsolved');
  if (typeof run.path_length_raw === 'number') bits.push(`len ${run.path_length_raw.toFixed(3)}`);
  if (typeof run.time_first_solution_s === 'number') {
    bits.push(`first ${run.time_first_solution_s.toFixed(2)} s`);
  }
  const frame = panel.frames[index];
  bits.push(
    `frame ${index + 1}/${panel.frames.length} · t = ${(frame ? frame[0] : 0).toFixed(2)} s`
  );
  return bits.join(' · ');
}

function render() {
  if (!state.scene) return;
  for (const panel of panels) {
    if (!panel.data) continue;
    const { w, h, ratio } = sizeCanvas(panel.canvas);
    const ctx = panel.canvas.getContext('2d');
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const project = projector(state.bounds, w, h);
    drawStatic(ctx, project, state.scene);
    if (panel.data.frames.length > 0) {
      const index = Math.min(state.frame, panel.data.frames.length - 1);
      drawFrame(ctx, project, panel.data, index);
      panel.caption.textContent = captionOf(panel.data, index);
    } else {
      panel.caption.textContent = `${panel.data.packed.planner} · no frames recorded`;
    }
  }
  el.slider.value = String(state.frame);
  el.counter.textContent = `${state.frame + 1} / ${state.total}`;
}

function setFrame(index) {
  state.frame = Math.max(0, Math.min(state.total - 1, index));
  render();
}

function buildLevelToggles(links) {
  el.levels.textContent = '';
  state.levels = new Set();
  for (let i = 0; i < links; i++) {
    state.levels.add(i);
    const label = document.createElement('label');
    const box = document.createElement('input');
    box.type = 'checkbox';
    box.checked = true;
    box.addEventListener('change', () => {
      if (box.checked) state.levels.add(i);
      else state.levels.delete(i);
      render();
    });
    label.appendChild(box);
    label.appendChild(document.createTextNode(`L${i}`));
    el.levels.appendChild(label);
  }
}

function buildLegend() {
  el.legend.textContent = '';
  for (const [name, color] of Object.entries(TREE_COLORS)) {
    const item = document.createElement('span');
    const swatch = document.createElement('span');
    swatch.className = 'swatch';
    swatch.style.background = color;
    item.appendChild(swatch);
    item.appendChild(document.createTextNode(name));
    el.legend.appendChild(item);
  }
}

async function loadScene(name) {
  const token = ++state.token;
  state.playing = false;
  el.play.textContent = 'Play';
  for (const panel of panels) panel.caption.textContent = 'loading…';

  const [scene, ...fixtures] = await Promise.all([
    loadJson(`scenes/${name}.json`),
    ...panels.map((p) => loadJson(`traces/${name}-${p.planner}.json`)),
  ]);
  if (token !== state.token) return;

  const links = fixtures[0].links;
  state.scene = scene;
  state.bounds = sceneBounds(scene, links);
  panels.forEach((panel, i) => {
    panel.data = preparePanel(fixtures[i], scene, links - 1);
  });
  state.total = Math.max(1, ...panels.map((p) => p.data.frames.length));
  state.intervalMs = fixtures[0].interval_ms || 50;
  state.accumulator = 0;
  el.slider.max = String(state.total - 1);
  buildLevelToggles(links);
  setFrame(0);
}

function tick(now) {
  if (state.playing) {
    if (state.lastTick > 0) state.accumulator += now - state.lastTick;
    while (state.accumulator >= state.intervalMs) {
      state.accumulator -= state.intervalMs;
      state.frame = (state.frame + 1) % state.total;
    }
    render();
  }
  state.lastTick = now;
  requestAnimationFrame(tick);
}

function reportError(err) {
  panels.forEach((p) => {
    p.caption.textContent = `error: ${err.message}`;
  });
}

el.play.addEventListener('click', () => {
  state.playing = !state.playing;
  state.accumulator = 0;
  el.play.textContent = state.playing ? 'Pause' : 'Play';
});
el.prev.addEventListener('click', () => setFrame(state.frame - 1));
el.next.addEventListener('click', () => setFrame(state.frame + 1));
el.slider.addEventListener('input', () => setFrame(Number(el.slider.value)));
el.scene.addEventListener('change', () => {
  loadScene(el.scene.value).catch(reportError);
});
window.addEventListener('resize', render);

buildLegend();
requestAnimationFrame(tick);
loadScene(el.scene.value).catch(reportError);

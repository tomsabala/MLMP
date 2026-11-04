/* MLMP trace player. Replays committed mlmp.traceviz/1 fixtures; no build step, no deps.
 *
 * All drawing — forward kinematics, extents, layers, colours — lives in renderer.js, a
 * byte-identical copy of web/src/renderer.js that the wasm playground under web/ also uses
 * (CI fails on drift). This file owns only the page: fetching, playback and the controls.
 *
 * A fixture stores every vertex and edge once, in order of first appearance, and each frame is
 * [t_s, vertexCount, edgeCount] — so frame k is the prefix vertices[:nv] / edges[:ne], which is
 * exactly what pushFrame's vertexCount/edgeCount limits express. See mlmp/tools/packTrace.py for
 * why the raw mlmp.trace/1 stream cannot be served directly.
 *
 * The sibling *.run.json holds the mlmp.run/1 result, whose `path` is a real full-dimension
 * solution; showPath draws it as connected arms. Only the two solved runs have a non-empty one. */
import { TREE_COLORS, createTracePlayer } from './renderer.js';

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
  frame: 0,
  total: 1,
  playing: false,
  levels: new Set(),
  intervalMs: 50,
  lastTick: 0,
  accumulator: 0,
  token: 0,
};

async function loadJson(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${url}: ${response.status}`);
  return response.json();
}

function preparePanel(packed) {
  return { packed, frames: packed.frames, run: packed.run || {} };
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

/* Re-pushing the same fixture arrays every render is what keeps the renderer's FK memo warm:
 * only the prefix limits change, so nothing is recomputed. */
function render() {
  if (!state.scene) return;
  for (const panel of panels) {
    if (!panel.data || !panel.player) continue;
    panel.player.setLevels(state.levels);
    if (panel.data.frames.length > 0) {
      const index = Math.min(state.frame, panel.data.frames.length - 1);
      const [, nv, ne] = panel.data.frames[index];
      panel.player.pushFrame({
        vertices: panel.data.packed.vertices,
        edges: panel.data.packed.edges,
        vertexCount: nv,
        edgeCount: ne,
      });
      panel.caption.textContent = captionOf(panel.data, index);
    } else {
      panel.player.redraw();
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
  /* TREE_COLORS covers the graph's component tags only; showPath draws the run's solution in
   * PATH_COLOR, which would otherwise be missing from the legend entirely. */
  const entries = [...Object.entries(TREE_COLORS), ['solution path', PATH_COLOR]];
  for (const [name, color] of entries) {
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

  const loaded = await Promise.all([
    loadJson(`scenes/${name}.json`),
    ...panels.map((p) => loadJson(`traces/${name}-${p.planner}.json`)),
    ...panels.map((p) => loadJson(`traces/${name}-${p.planner}.run.json`)),
  ]);
  if (token !== state.token) return;

  const scene = loaded[0];
  const fixtures = loaded.slice(1, 1 + panels.length);
  const runs = loaded.slice(1 + panels.length);

  /* The scene is the authority on dimension; a trace header merely echoes it. */
  const links = scene.metadata.n;
  state.scene = scene;
  panels.forEach((panel, i) => {
    panel.data = preparePanel(fixtures[i]);
    if (panel.player) panel.player.setScene(scene);
    else panel.player = createTracePlayer({ canvas: panel.canvas, scene });
    panel.player.showPath(runs[i].path ?? null);
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

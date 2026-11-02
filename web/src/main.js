/* The playground page: controls, the solve queue and the result panel. Drawing is renderer.js,
 * editing is editor.js, planning is worker.js — this file owns none of the three.
 *
 * Trace parameters match the CLI defaults (main.cpp's --trace-interval-ms 50 /
 * --trace-max-frames 400) and mlmp/tools/recordTraces.sh, so an in-browser run is comparable to
 * the committed fixtures. At the 10 s maximum the frame cap cannot be reached; truncation is
 * surfaced anyway. */
import { TREE_COLORS, createTracePlayer } from './renderer.js';
import { createSceneEditor } from './editor.js';

const TRACE_INTERVAL_MS = 50;
const TRACE_MAX_FRAMES = 400;
const DEFAULT_SCENE = 'scene1.json';

const el = {
  scene: document.getElementById('scene'),
  planner: document.getElementById('planner'),
  time: document.getElementById('time'),
  timeValue: document.getElementById('timeValue'),
  seed: document.getElementById('seed'),
  solve: document.getElementById('solve'),
  addObstacle: document.getElementById('addObstacle'),
  levels: document.getElementById('levels'),
  legend: document.getElementById('legend'),
  canvas: document.getElementById('canvas'),
  status: document.getElementById('status'),
  result: document.getElementById('result'),
};

const state = {
  player: null,
  editor: null,
  levels: new Set(),
  ready: false,
  /* Only the newest edit survives: `inFlight` is the id the worker is solving, `pending` the one
   * scene waiting for it. Dragging stays live while a solve runs, which is the point of queueing
   * rather than blocking. */
  inFlight: null,
  pending: null,
  nextId: 1,
};

const worker = new Worker(new URL('./worker.js', import.meta.url), { type: 'module' });

function setStatus(text) {
  el.status.textContent = text;
}

function showResult(rows) {
  el.result.textContent = '';
  for (const [key, value] of rows) {
    const dt = document.createElement('dt');
    dt.textContent = key;
    const dd = document.createElement('dd');
    dd.textContent = value;
    el.result.appendChild(dt);
    el.result.appendChild(dd);
  }
}

function num(value, digits = 3) {
  return typeof value === 'number' ? value.toFixed(digits) : '—';
}

/* run_result.h's statusToString values, translated for someone who just dragged a box. */
function explain(result) {
  switch (result.status) {
    case 'invalid_start':
      return "this obstacle blocks a robot's start configuration";
    case 'invalid_goal':
      return "this obstacle blocks a robot's goal configuration";
    case 'timeout':
      return `no solution found in ${result.time_limit_s} s`;
    case 'approximate_solution':
      return 'approximate solution only — the goal was not reached exactly';
    case 'exact_solution':
      return 'solved';
    default:
      return result.status;
  }
}

/* The two planner families report a blocked endpoint differently: single-level planners return
 * PlannerStatus::INVALID_START / INVALID_GOAL, while the bundle-space ones throw out of
 * BundleSpaceGraph::setup ("Invalid initial states." / "Invalid goal states."), which reaches
 * here as a runtime error object. Both are the C++ checker's verdict; only the wording differs. */
function explainError(error) {
  if (/invalid initial state/i.test(error.message)) {
    return "this obstacle blocks a robot's start configuration";
  }
  if (/invalid goal state/i.test(error.message)) {
    return "this obstacle blocks a robot's goal configuration";
  }
  return `${error.kind}: ${error.message}`;
}

function renderResult(result) {
  if (result.error) {
    setStatus(explainError(result.error));
    showResult([
      ['error', result.error.kind],
      ['message', result.error.message],
    ]);
    state.player.showPath(null);
    state.player.redraw();
    return;
  }

  setStatus(`${result.planner} · ${explain(result)}`);
  showResult([
    ['status', result.status],
    ['exact', String(result.exact)],
    ['path_length_raw', num(result.path_length_raw)],
    ['path_length_simplified', num(result.path_length_simplified)],
    ['time_first_solution_s', num(result.time_first_solution_s)],
    ['graph_vertices', String(result.graph_vertices)],
    ['states_sampled', String(result.states_sampled)],
    [
      'trace',
      result.trace
        ? `${result.trace.frames} frames · truncated ${result.trace.truncated} · errors ${result.trace.errors}`
        : 'none',
    ],
  ]);

  state.player.showPath(result.solved ? result.path : null);
  state.player.redraw();
  state.editor.drawOverlay();
}

function requestSolve(scene) {
  const message = {
    type: 'solve',
    id: state.nextId++,
    sceneJson: JSON.stringify(scene),
    algo: el.planner.value,
    timeLimitS: Number(el.time.value),
    seed: Number(el.seed.value),
    traceIntervalMs: TRACE_INTERVAL_MS,
    traceMaxFrames: TRACE_MAX_FRAMES,
  };
  if (state.inFlight !== null) {
    state.pending = message;
    return;
  }
  state.inFlight = message.id;
  state.pending = null;
  el.solve.disabled = true;
  setStatus(`solving ${message.algo} for ${message.timeLimitS} s…`);
  state.player.reset();
  worker.postMessage(message);
}

worker.onmessage = (event) => {
  const msg = event.data;
  if (msg.type === 'ready') {
    state.ready = !msg.error;
    if (msg.error) setStatus(`planner failed to load: ${msg.error}`);
    else if (state.player) setStatus('ready');
    el.solve.disabled = !state.ready;
    return;
  }
  if (msg.id !== state.inFlight) return; // a superseded run's tail
  if (msg.type === 'frame') {
    state.player.pushFrame(msg.frame);
    state.editor.drawOverlay();
    return;
  }
  if (msg.type === 'result') {
    state.inFlight = null;
    el.solve.disabled = false;
    renderResult(msg.result);
    if (state.pending) {
      const next = state.pending;
      state.pending = null;
      state.inFlight = next.id;
      el.solve.disabled = true;
      setStatus(`solving ${next.algo} for ${next.timeLimitS} s…`);
      state.player.reset();
      worker.postMessage(next);
    }
  }
};

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
      state.player.setLevels(state.levels);
      state.player.redraw();
      state.editor.drawOverlay();
    });
    label.appendChild(box);
    label.appendChild(document.createTextNode(`L${i}`));
    el.levels.appendChild(label);
  }
  state.player.setLevels(state.levels);
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

async function loadJson(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${url}: ${response.status}`);
  return response.json();
}

async function loadScene(name) {
  const scene = await loadJson(`scenes/${name}`);
  if (state.player) state.player.setScene(scene);
  else state.player = createTracePlayer({ canvas: el.canvas, scene });
  if (state.editor) state.editor.setScene(scene);
  else state.editor = createSceneEditor({ canvas: el.canvas, player: state.player, scene });
  state.editor.setAddMode(el.addObstacle.checked);
  buildLevelToggles(scene.metadata.n);
  state.player.redraw();
  showResult([['status', '—']]);
  setStatus(state.ready ? `${name} loaded` : 'loading the planner…');
}

/* An edit invalidates the previous solution: setScene inside the editor has already dropped the
 * trees and the path, so this only queues the re-solve. */
function attachEditor() {
  state.editor.events.addEventListener('change', (event) => {
    showResult([['status', 'editing…']]);
    requestSolve(event.detail);
  });
}

el.time.addEventListener('input', () => {
  el.timeValue.textContent = `${el.time.value} s`;
});
el.solve.addEventListener('click', () => requestSolve(state.editor.scene));
el.addObstacle.addEventListener('change', () => state.editor.setAddMode(el.addObstacle.checked));
el.scene.addEventListener('change', () => {
  loadScene(el.scene.value).catch((err) => setStatus(`error: ${err.message}`));
});
window.addEventListener('resize', () => {
  if (!state.player) return;
  state.player.redraw();
  state.editor.drawOverlay();
});

el.solve.disabled = true;
buildLegend();

(async () => {
  const names = await loadJson('scenes/index.json');
  for (const name of names) {
    const option = document.createElement('option');
    option.value = name;
    option.textContent = name.replace(/\.json$/, '');
    option.selected = name === DEFAULT_SCENE;
    el.scene.appendChild(option);
  }
  await loadScene(el.scene.value || names[0]);
  attachEditor();
})().catch((err) => setStatus(`error: ${err.message}`));

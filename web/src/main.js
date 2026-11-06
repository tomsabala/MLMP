/* The playground page: controls, the solve queue and the result panel. Drawing is renderer.js,
 * editing is editor.js, planning is worker.js — this file owns none of the three.
 *
 * Trace parameters match the CLI defaults (main.cpp's --trace-interval-ms 50 /
 * --trace-max-frames 400) and mlmp/tools/recordTraces.sh, so an in-browser run is comparable to
 * the committed fixtures. At the 10 s maximum the frame cap cannot be reached; truncation is
 * surfaced anyway. */
import { PATH_COLOR, TREE_COLORS, createTracePlayer } from './renderer.js';
import { createCSpaceView, isValidConfig } from './cspace.js';
import { createSceneEditor } from './editor.js';

const TRACE_INTERVAL_MS = 50;
const TRACE_MAX_FRAMES = 400;
/* Interpolated poses per second of solution playback. Same rate as the trace explorer, so the
 * two surfaces move at the same speed. */
const POSES_PER_SECOND = 45;
const DEFAULT_SCENE = 'scene1.json';

const el = {
  scene: document.getElementById('scene'),
  planner: document.getElementById('planner'),
  time: document.getElementById('time'),
  timeValue: document.getElementById('timeValue'),
  seed: document.getElementById('seed'),
  solve: document.getElementById('solve'),
  addObstacle: document.getElementById('addObstacle'),
  levelSelect: document.getElementById('levelSelect'),
  legend: document.getElementById('legend'),
  view: document.getElementById('view'),
  treeSource: document.getElementById('treeSource'),
  canvas: document.getElementById('canvas'),
  cspaceCanvas: document.getElementById('cspace'),
  cspaceCaption: document.getElementById('cspaceCaption'),
  status: document.getElementById('status'),
  result: document.getElementById('result'),
};

const state = {
  player: null,
  cspace: null,
  editor: null,
  levels: new Set(),
  /* Solution playback: `phase` is where the arms stand along the interpolated path, advanced by
   * the animation loop while `animating`. A run with no solution never animates. */
  phase: 0,
  animating: false,
  lastTick: 0,
  /* legend entry name -> its <span>, so a redraw can dim the colours this run has none of. */
  legend: new Map(),
  hasPath: false,
  ready: false,
  /* Only the newest edit survives: `inFlight` is the id the worker is solving, `pending` the one
   * scene waiting for it. Dragging stays live while a solve runs, which is the point of queueing
   * rather than blocking. */
  inFlight: null,
  pending: null,
  nextId: 1,
};

/* The worker is replaced, never reused, when a run has to be cancelled: solveScene blocks
 * inside wasm for the whole budget and there is no in-band abort, so terminate + respawn is the
 * only cancel available from JS. `worker` is therefore a binding, and the factory owns the
 * handler wiring so a respawned worker is wired identically. */
function createWorker() {
  let instance;
  try {
    instance = new Worker(new URL('./worker.js', import.meta.url), { type: 'module' });
  } catch (err) {
    /* Module workers: Safari <15, Firefox <114. Without this the page throws at module
     * evaluation and sits on its literal "loading the planner…" text with no explanation. */
    setStatus(`this browser cannot run the planner: ${err && err.message ? err.message : err}`);
    return null;
  }
  instance.onmessage = onWorkerMessage;
  instance.onerror = (event) =>
    handleWorkerFailure(event.message || 'the planner worker failed');
  instance.onmessageerror = () =>
    handleWorkerFailure('the planner worker sent an undeliverable message');
  return instance;
}

let worker = createWorker();

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

/* While a run is in flight the page shows the search whatever the view select says: `motion` has
 * nothing to play until a solution exists, and watching the trees grow is the reason to run a
 * planner in a browser at all. The status line says so, so it is not a silent override. */
function effectiveView() {
  if (state.inFlight !== null && el.view.value === 'motion') return 'graph';
  return el.view.value;
}

/* One redraw of both canvases plus the editor's handles, which have to go back on top of a fresh
 * frame or dragging an obstacle loses its selection outline. */
function redrawAll() {
  state.player.redraw();
  if (state.cspace) state.cspace.redraw();
  if (state.editor) state.editor.drawOverlay();
}

/* The playback clock. It runs only while there is a solution to play and the view asks for it, so
 * an idle page costs no frames. */
function animate(now) {
  if (!state.animating) return;
  const states = state.player.motionStates;
  if (states > 1) {
    const elapsed = state.lastTick > 0 ? now - state.lastTick : 0;
    state.phase += (elapsed / 1000) * (POSES_PER_SECOND / (states - 1));
    /* Loop with a pause at the goal: the solution reads as a repeated motion, not a cycle that
     * snaps back mid-stride. */
    if (state.phase > 1.25) state.phase = 0;
    const shown = Math.min(1, state.phase);
    state.player.setPhase(shown);
    if (state.cspace) state.cspace.setPhase(shown);
    redrawAll();
  }
  state.lastTick = now;
  requestAnimationFrame(animate);
}

function syncAnimation() {
  const wanted = effectiveView() === 'motion' && state.player.motionStates > 1;
  if (wanted === state.animating) return;
  state.animating = wanted;
  state.lastTick = 0;
  if (wanted) requestAnimationFrame(animate);
}

/* The single place a solution enters the page: both canvases get the same interpolated path, so
 * the arms in the workspace and the dot in configuration space are the same configuration. */
function showSolution(path) {
  state.phase = 0;
  /* The same predicate the C++ checker runs, applied to the poses actually drawn: the planner
   * guarantees its path states, not what the arms sweep through between them, and its motion
   * validator can sample a segment as coarsely as one interior check. */
  const scene = state.player.scene;
  const top = scene.metadata.n - 1;
  state.player.showPath(path, (q) => isValidConfig(scene, top, q));
  state.player.setPhase(0);
  if (state.cspace) {
    state.cspace.showPath(state.player.motionPath);
    state.cspace.setPhase(0);
  }
  state.player.setViewMode(effectiveView());
  redrawAll();
  syncAnimation();
}

function renderResult(result) {
  if (result.error) {
    setStatus(explainError(result.error));
    showResult([
      ['error', result.error.kind],
      ['message', result.error.message],
    ]);
    state.hasPath = false;
    showSolution(null);
    syncLabels();
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
    /* Not a mlmp.run/1 field: the frames carry tree_source, and the player keeps the last one
     * it was handed. A planner that tags nothing gets its start/goal split derived from the
     * graph's connectivity in solver.h, and the reader has to know which they are looking at. */
    [
      'trees',
      state.player.treeSource === 'derived' ? 'derived from connectivity' : 'planner-reported',
    ],
  ]);

  state.hasPath = Boolean(result.solved && result.path && result.path.length > 0);
  showSolution(state.hasPath ? result.path : null);
  /* Appended after showSolution, because that is what runs the checker over the drawn poses. A
   * solved run whose animation passes through an obstacle is the most important thing this page
   * can tell its reader, so it belongs in the result list and not in a tooltip. */
  if (state.hasPath) {
    const { count, total, from, to } = state.player.motionCollisions();
    const where = from === to ? `pose ${from + 1}` : `poses ${from + 1}-${to + 1}`;
    const dt = document.createElement('dt');
    dt.textContent = 'collisions';
    const dd = document.createElement('dd');
    dd.textContent = count === 0
      ? `none — all ${total} drawn poses are collision-free`
      : `${count} of ${total} drawn poses (${where})`;
    el.result.appendChild(dt);
    el.result.appendChild(dd);
  }
  syncLabels();
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
  startSolve(message);
}

function startSolve(message) {
  if (!worker) return;
  state.inFlight = message.id;
  state.pending = null;
  setStatus(
    el.view.value === 'motion'
      ? `solving ${message.algo} for ${message.timeLimitS} s — showing the search until it finds one…`
      : `solving ${message.algo} for ${message.timeLimitS} s…`
  );
  state.hasPath = false;
  state.player.reset();
  state.player.setViewMode(effectiveView());
  if (state.cspace) {
    state.cspace.showPath([]);
    state.cspace.reset();
  }
  syncAnimation();
  syncLabels();
  worker.postMessage(message);
}

/* Solve stays clickable during a run: clicking it cancels the current run and starts a new one
 * immediately, which is the only way to escape a 10 s budget. It is disabled only while the
 * planner or the editor does not exist yet — state.editor is built by the awaited IIFE at the
 * bottom of this file, so wasm readiness alone is not enough to make a click safe. */
function syncSolveEnabled() {
  el.solve.disabled = !(state.ready && state.editor !== null);
}

function cancelInFlight() {
  if (state.inFlight === null) return;
  if (worker) worker.terminate();
  state.inFlight = null;
  state.pending = null;
  /* The replacement posts `ready` again, which the handler below already deals with. */
  worker = createWorker();
  syncSolveEnabled();
}

function handleWorkerFailure(message) {
  /* A wasm abort or a failed memory growth never produces a `result`, so without this the page
   * keeps Solve disabled and the caption stuck on "solving …" until a reload. */
  state.inFlight = null;
  state.pending = null;
  setStatus(`planner error: ${message}`);
  worker = createWorker();
  syncSolveEnabled();
}

function onWorkerMessage(event) {
  const msg = event.data;
  if (msg.type === 'ready') {
    state.ready = !msg.error;
    if (msg.error) {
      setStatus(`planner failed to load: ${msg.error}`);
    } else if (state.player && state.inFlight === null) {
      /* A respawned worker reports ready while the run that replaced it is already going, so
       * the caption must not be reset underneath that run. */
      setStatus('ready');
    }
    syncSolveEnabled();
    return;
  }
  if (msg.id !== state.inFlight) return; // a superseded run's tail
  if (msg.type === 'frame') {
    state.player.pushFrame(msg.frame);
    if (state.cspace) state.cspace.pushFrame(msg.frame);
    syncLabels();
    state.editor.drawOverlay();
    return;
  }
  if (msg.type === 'result') {
    state.inFlight = null;
    renderResult(msg.result);
    syncSolveEnabled();
    if (state.pending) {
      startSolve(state.pending);
    }
  }
}

/* One level at a time, defaulting to the top one — the level that is the whole robot. Every level
 * at once is what turned four levels of two robots into 37 000 superimposed dots. */
function buildLevelSelect(links) {
  el.levelSelect.textContent = '';
  const options = [['all', 'all levels']];
  for (let i = 0; i < links; i++) {
    options.push([String(i), `level ${i} · ${i + 1} link${i ? 's' : ''}`]);
  }
  for (const [value, label] of options) {
    const option = document.createElement('option');
    option.value = value;
    option.textContent = label;
    el.levelSelect.appendChild(option);
  }
  el.levelSelect.value = String(links - 1);
  applyLevelSelection(links);
}

function applyLevelSelection(links) {
  state.levels = new Set();
  if (el.levelSelect.value === 'all') {
    for (let i = 0; i < links; i++) state.levels.add(i);
  } else {
    state.levels.add(Number(el.levelSelect.value));
  }
  state.player.setLevels(state.levels);
}

function buildLegend() {
  el.legend.textContent = '';
  state.legend = new Map();
  /* TREE_COLORS covers the graph's component tags only; the returned solution is drawn in
   * PATH_COLOR and would otherwise be missing from the legend entirely. */
  const entries = [...Object.entries(TREE_COLORS), ['solution path', PATH_COLOR]];
  for (const [name, color] of entries) {
    const item = document.createElement('span');
    const swatch = document.createElement('span');
    swatch.className = 'swatch';
    swatch.style.background = color;
    item.appendChild(swatch);
    item.appendChild(document.createTextNode(name));
    el.legend.appendChild(item);
    state.legend.set(name, item);
  }
}

/* Run after every redraw. A colour the current frame has none of is dimmed rather than removed,
 * so the reader can tell "this run produced no goal tree" from "this page cannot show one"; and
 * the caption states whether the start/goal split is the planner's own tagging or labels
 * solver.h derived from graph connectivity. */
function syncLabels() {
  if (!state.player) return;
  const counts = state.player.treeCounts();
  for (const [name, item] of state.legend) {
    const empty = name === 'solution path' ? !state.hasPath : counts[name] === 0;
    item.classList.toggle('dim', empty);
  }
  /* Derivation can only follow edges the planner publishes, and some publish almost none:
   * EITStar reports 28 302 vertices with 120 edges and flags no start or goal vertex at all, so
   * nothing is reachable from an endpoint and every vertex stays untagged. Saying "derived"
   * there without saying it found nothing is how the grey hairball looked like a bug. */
  const labelled = counts.start + counts.goal + counts.section;
  el.treeSource.textContent =
    state.player.treeSource === 'derived'
      ? labelled > 0
        ? 'trees: derived from graph connectivity — this planner tags none of its own'
        : 'trees: none — this planner publishes no graph edges reaching the start or goal'
      : 'trees: reported by the planner';
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
  if (state.cspace) state.cspace.setScene(scene);
  else state.cspace = createCSpaceView({ canvas: el.cspaceCanvas, scene });
  if (state.editor) state.editor.setScene(scene);
  else state.editor = createSceneEditor({ canvas: el.canvas, player: state.player, scene });
  state.editor.setAddMode(el.addObstacle.checked);
  state.player.setViewMode(effectiveView());
  state.hasPath = false;
  buildLevelSelect(scene.metadata.n);
  showSolution(null);
  syncCSpaceCaption();
  syncLabels();
  showResult([['status', '—']]);
  syncSolveEnabled();
  setStatus(state.ready ? `${name} loaded` : 'loading the planner…');
}

function syncCSpaceCaption() {
  if (!state.cspace) return;
  if (!state.cspace.available) {
    const robots = state.player.scene.robots.length;
    el.cspaceCaption.textContent =
      `no 2-D bundle level in this scene — ${robots} robots, so the lowest level is already `
      + 'higher-dimensional';
    return;
  }
  const blocked = state.cspace.blockedFraction;
  const level = state.cspace.level;
  /* An almost-empty square is a fact about the problem, not a missing layer: the bottom of the
   * hierarchy is nearly unobstructed, which is why it is cheap to solve and tells the level
   * above very little. */
  el.cspaceCaption.textContent = blocked < 0.02
    ? `configuration space of level ${level} · ${(100 * (1 - blocked)).toFixed(1)}% free — links `
      + 'this short can barely reach an obstacle'
    : `configuration space of level ${level} · grey is where the arms collide `
      + `(${(100 * blocked).toFixed(0)}% of it)`;
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
el.solve.addEventListener('click', () => {
  if (!state.editor) return;
  cancelInFlight();
  requestSolve(state.editor.scene);
});
el.addObstacle.addEventListener('change', () => state.editor.setAddMode(el.addObstacle.checked));
el.view.addEventListener('change', () => {
  if (!state.player) return;
  state.player.setViewMode(effectiveView());
  el.levelSelect.disabled = el.view.value === 'motion';
  state.phase = 0;
  state.player.setPhase(0);
  if (state.cspace) state.cspace.setPhase(0);
  redrawAll();
  syncAnimation();
  syncLabels();
});
el.levelSelect.addEventListener('change', () => {
  if (!state.player) return;
  applyLevelSelection(state.player.links);
  redrawAll();
  syncLabels();
});
el.scene.addEventListener('change', () => {
  /* Cancel before the scene swaps: a surviving frame from the old run would be drawn with the
   * new scene's stride and pins (robots and links differ across the committed scenes). */
  cancelInFlight();
  loadScene(el.scene.value).catch((err) => setStatus(`error: ${err.message}`));
});
window.addEventListener('resize', () => {
  if (!state.player) return;
  redrawAll();
});

el.solve.disabled = true;
el.levelSelect.disabled = el.view.value === 'motion';
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

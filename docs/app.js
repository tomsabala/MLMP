/* MLMP trace explorer. Replays committed mlmp.traceviz/1 fixtures; no build step, no deps.
 *
 * All workspace drawing — forward kinematics, extents, layers, colours — lives in renderer.js, a
 * byte-identical copy of web/src/renderer.js that the wasm playground under web/ also uses (CI
 * fails on drift); the configuration-space canvas is cspace.js, copied the same way. This file
 * owns only the page: fetching, playback and the controls.
 *
 * A fixture stores every vertex and edge once, in order of first appearance, and each frame is
 * [t_s, vertexCount, edgeCount] — so frame k is the prefix vertices[:nv] / edges[:ne], which is
 * exactly what pushFrame's vertexCount/edgeCount limits express. See mlmp/tools/packTrace.py for
 * why the raw mlmp.trace/1 stream cannot be served directly.
 *
 * The sibling *.run.json holds the mlmp.run/1 result, whose `path` is a real full-dimension
 * solution. The packed fixture deliberately does not carry it (packTrace.py's RUN_KEYS copies
 * scalars only), so both files are fetched per panel.
 *
 * Two timelines share one slider, because they are never both meaningful at once: in `motion`
 * the slider scrubs the interpolated solution, in the search views it scrubs recorded trace
 * frames. The counter says which one is under the thumb. */
import { PATH_COLOR, TREE_COLORS, createTracePlayer } from './renderer.js';
import { createCSpaceView } from './cspace.js';

const el = {
  scene: document.getElementById('scene'),
  play: document.getElementById('play'),
  prev: document.getElementById('prev'),
  next: document.getElementById('next'),
  slider: document.getElementById('slider'),
  counter: document.getElementById('counter'),
  levelSelect: document.getElementById('levelSelect'),
  legend: document.getElementById('legend'),
  view: document.getElementById('view'),
};

const panels = [
  {
    planner: 'biqrrt',
    canvas: document.getElementById('canvasA'),
    caption: document.getElementById('captionA'),
    cspaceCanvas: document.getElementById('cspaceA'),
    cspaceCaption: document.getElementById('cspaceCaptionA'),
  },
  {
    planner: 'qrrt',
    canvas: document.getElementById('canvasB'),
    caption: document.getElementById('captionB'),
    cspaceCanvas: document.getElementById('cspaceB'),
    cspaceCaption: document.getElementById('cspaceCaptionB'),
  },
];

/* Motion playback runs at a fixed number of interpolated poses per second, independent of the
 * fixture's frame interval: the two timelines measure different things (planner wall clock
 * versus distance along the solution) and tying them together made short paths flash past. */
const POSES_PER_SECOND = 45;

const state = {
  scene: null,
  /* Index into whichever timeline the current view uses. */
  frame: 0,
  total: 1,
  playing: false,
  levels: new Set(),
  /* legend entry name -> its <span>, so a redraw can dim the colours this scene has none of. */
  legend: new Map(),
  intervalMs: 50,
  lastTick: 0,
  accumulator: 0,
  token: 0,
};

function motionMode() {
  return el.view.value === 'motion';
}

async function loadJson(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${url}: ${response.status}`);
  return response.json();
}

/* `levelsReached` is read once per fixture so a caption can tell "not at this frame yet" from
 * "not in the whole run": QRRT on scene1 never puts a vertex above level 2 in 60 s, which is a
 * result, while BiQRRT's level 3 is simply empty in the first frames. */
function preparePanel(packed, run) {
  const levelsReached = new Set();
  for (const row of packed.vertices) levelsReached.add(Array.isArray(row) ? row[0] : row.level || 0);
  return { packed, frames: packed.frames, run: run || {}, levelsReached };
}

/* The step the slider measures, which the two views disagree about. */
function stepCount() {
  if (motionMode()) {
    return Math.max(1, ...panels.map((p) => (p.player ? p.player.motionStates : 0)));
  }
  return Math.max(1, ...panels.map((p) => (p.data ? p.data.frames.length : 0)));
}

function captionOf(panel, index) {
  const run = panel.run;
  const bits = [panel.packed.planner, run.status || '?'];
  if (run.solved && typeof run.path_length_raw === 'number') {
    bits.push(`len ${run.path_length_raw.toFixed(3)}`);
  }
  if (run.solved && typeof run.time_first_solution_s === 'number') {
    bits.push(`first ${run.time_first_solution_s.toFixed(2)} s`);
  }
  /* mlmp.traceviz/1 carries packTrace.py's propagated tree_source: either the planner tagged
   * its own trees or solver.h derived the labels from graph connectivity. Fixtures recorded
   * before the field exists are planner-tagged BiQRRT/QRRT runs. */
  bits.push(
    panel.packed.tree_source === 'derived'
      ? 'trees: derived from connectivity'
      : 'trees: planner-reported'
  );
  if (!motionMode()) {
    const frame = panel.frames[Math.min(index, panel.frames.length - 1)];
    bits.push(
      `frame ${Math.min(index + 1, panel.frames.length)}/${panel.frames.length} · t = ${
        (frame ? frame[0] : 0).toFixed(2)
      } s`
    );
  }
  return bits.join(' · ');
}

/* The one state a reader must not have to guess at: a run that never solved has nothing to play,
 * and that is a result, not a loading failure. */
function emptyMotionNote(panel) {
  const budget = panel.packed.time_limit_s;
  const limit = typeof budget === 'number' ? `${budget.toFixed(0)} s` : 'the budget';
  return `${panel.packed.planner} · no solution in ${limit} of traced planning — nothing to `
    + 'play. Switch the view to search tree to see how far it got.';
}

/* A level the run never reached draws an empty canvas, and an empty canvas reads as a broken
 * page. QRRT on scene1 spends its whole budget without putting a single vertex on the top level,
 * which is the comparison the explorer exists to make — so say it. */
function emptyLevelNote(panel) {
  const counts = panel.player.treeCounts();
  if (counts.start + counts.goal + counts.section + counts.untagged > 0) return '';
  const choice = el.levelSelect.value;
  if (choice === 'all') return ' · no vertices in this frame yet';
  const level = Number(choice);
  return panel.data.levelsReached.has(level)
    ? ` · nothing on level ${level} in this frame yet`
    : ` · nothing on level ${level} in the whole run — it never got that far`;
}

/* Re-pushing the same fixture arrays every render is what keeps the renderer's FK memo warm:
 * only the prefix limits change, so nothing is recomputed. */
function render() {
  if (!state.scene) return;
  const motion = motionMode();
  for (const panel of panels) {
    if (!panel.data || !panel.player) continue;
    panel.player.setLevels(state.levels);

    /* In motion the graph is not drawn at all, but the C-space canvas still shows where the
     * planner searched, so it is handed the whole recording's last frame rather than a scrub
     * position that has no meaning in this view. */
    const frames = panel.data.frames;
    const frameIndex = motion
      ? frames.length - 1
      : Math.min(state.frame, frames.length - 1);
    if (frames.length > 0) {
      const [, nv, ne] = frames[frameIndex];
      const next = {
        vertices: panel.data.packed.vertices,
        edges: panel.data.packed.edges,
        vertexCount: nv,
        edgeCount: ne,
        treeSource: panel.data.packed.tree_source,
      };
      panel.player.pushFrame(next);
      if (panel.cspace) panel.cspace.pushFrame(next);
    }

    if (motion) {
      const states = panel.player.motionStates;
      const t = states > 1 ? Math.min(state.frame, states - 1) / (states - 1) : 1;
      panel.player.setPhase(t);
      if (panel.cspace) panel.cspace.setPhase(t);
      panel.player.redraw();
      panel.caption.textContent = states > 1
        ? `${captionOf(panel.data, state.frame)} · pose ${Math.min(state.frame + 1, states)}/${states}`
        : emptyMotionNote(panel.data);
    } else if (frames.length > 0) {
      panel.caption.textContent = captionOf(panel.data, frameIndex) + emptyLevelNote(panel);
    } else {
      panel.player.redraw();
      panel.caption.textContent = `${panel.data.packed.planner} · no frames recorded`;
    }
    if (panel.cspace) {
      panel.cspace.redraw();
      panel.cspaceCaption.textContent = cspaceCaptionOf(panel);
    }
  }
  updateLegend();
  el.slider.value = String(state.frame);
  el.counter.textContent = motionMode()
    ? `pose ${state.frame + 1} / ${state.total}`
    : `frame ${state.frame + 1} / ${state.total}`;
}

function cspaceCaptionOf(panel) {
  if (!panel.cspace.available) {
    return `no 2-D bundle level in this scene — ${state.scene.robots.length} robots, so the `
      + 'lowest level is already higher-dimensional';
  }
  const blocked = panel.cspace.blockedFraction;
  const level = panel.cspace.level;
  /* An almost-empty square is a fact about the problem, not a missing layer, and it is one of
   * the more interesting ones: the bottom of the hierarchy is nearly unobstructed, so it is
   * cheap to solve and tells the level above very little. */
  if (blocked < 0.02) {
    return `configuration space of level ${level} · ${(100 * (1 - blocked)).toFixed(1)}% free — `
      + 'links this short can barely reach an obstacle';
  }
  return `configuration space of level ${level} · grey is where the arms collide `
    + `(${(100 * blocked).toFixed(0)}% of it)`;
}


function setFrame(index) {
  state.frame = Math.max(0, Math.min(state.total - 1, index));
  render();
}

/* One level at a time is the default, and the top level is the one that is the whole robot. The
 * old page checked every level on at once, which is how four levels of two robots became 37 000
 * superimposed dots. */
function buildLevelSelect(links) {
  el.levelSelect.textContent = '';
  const options = [['all', 'all levels']];
  for (let i = 0; i < links; i++) options.push([String(i), `level ${i} · ${i + 1} link${i ? 's' : ''}`]);
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
}

function buildLegend() {
  el.legend.textContent = '';
  state.legend = new Map();
  /* TREE_COLORS covers the graph's component tags only; the solution is drawn in PATH_COLOR,
   * which would otherwise be missing from the legend entirely. */
  const entries = [...Object.entries(TREE_COLORS), ['solution', PATH_COLOR]];
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

/* A colour this frame has none of stays in the legend and is dimmed instead of removed: the
 * reader has to be able to tell "this run has no goal tree" from "this page cannot show one".
 * Counts are summed over both panels, because the legend is shared. */
function updateLegend() {
  const totals = { start: 0, goal: 0, section: 0, untagged: 0 };
  let hasPath = false;
  for (const panel of panels) {
    if (!panel.player) continue;
    const counts = panel.player.treeCounts();
    for (const key of Object.keys(totals)) totals[key] += counts[key];
    hasPath = hasPath || panel.player.motionStates > 1;
  }
  for (const [name, item] of state.legend) {
    const empty = name === 'solution' ? !hasPath : totals[name] === 0;
    item.classList.toggle('dim', empty);
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
    panel.data = preparePanel(fixtures[i], runs[i]);
    if (panel.player) panel.player.setScene(scene);
    else panel.player = createTracePlayer({ canvas: panel.canvas, scene });
    if (panel.cspace) panel.cspace.setScene(scene);
    else panel.cspace = createCSpaceView({ canvas: panel.cspaceCanvas, scene });
    panel.player.setViewMode(el.view.value);
    const path = Array.isArray(runs[i].path) ? runs[i].path : null;
    panel.player.showPath(path);
    panel.cspace.showPath(panel.player.motionPath);
  });
  state.intervalMs = fixtures[0].interval_ms || 50;
  state.accumulator = 0;
  buildLevelSelect(links);
  resetTimeline();
}

/* Both timelines are rebuilt from the panels rather than remembered, because switching view or
 * scene changes what a step means and how many there are. */
function resetTimeline() {
  state.total = stepCount();
  el.slider.max = String(state.total - 1);
  setFrame(0);
}

function tick(now) {
  if (state.playing) {
    if (state.lastTick > 0) state.accumulator += now - state.lastTick;
    const step = motionMode() ? 1000 / POSES_PER_SECOND : state.intervalMs;
    while (state.accumulator >= step) {
      state.accumulator -= step;
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
el.view.addEventListener('change', () => {
  for (const panel of panels) {
    if (panel.player) panel.player.setViewMode(el.view.value);
  }
  el.levelSelect.disabled = motionMode();
  resetTimeline();
});
el.levelSelect.addEventListener('change', () => {
  if (!state.scene) return;
  applyLevelSelection(state.scene.metadata.n);
  render();
});
el.scene.addEventListener('change', () => {
  loadScene(el.scene.value).catch(reportError);
});
window.addEventListener('resize', render);

buildLegend();
el.levelSelect.disabled = motionMode();
requestAnimationFrame(tick);
loadScene(el.scene.value).catch(reportError);

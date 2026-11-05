/* Forward-kinematics and parity proof for web/src/renderer.js. No framework, no DOM:
 *
 *     node web/test/renderer.test.mjs      (from the repo root)
 *
 * The load-bearing case is the first one. A renderer that ignores bundle-level truncation looks
 * entirely plausible on screen — every dot is in the right neighbourhood — while drawing padding
 * that the planner never produced. */
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { readFileSync } from 'node:fs';

import {
  PADDING,
  PATH_COLOR,
  TREE_CODES,
  TREE_COLORS,
  chainTips,
  createTracePlayer,
  densifyPath,
  interpolateConfig,
  hexToRgba,
  levelAlpha,
  sceneBounds,
} from '../src/renderer.js';

const scene = JSON.parse(readFileSync(new URL('../../docs/scenes/scene1.json', import.meta.url)));
const links = scene.metadata.n;
const robots = scene.robots.length;
assert.equal(links, 4);
assert.equal(robots, 2);

/* A canvas that records what it was asked to paint. Groups 7-9 exercise the player itself and
 * not just its pure helpers, and node has no DOM. */
function stubCanvas() {
  const ops = [];
  const ctx = {
    lineWidth: 0,
    fillStyle: '',
    strokeStyle: '',
    setTransform() {},
    clearRect() {},
    beginPath() {},
    closePath() {},
    moveTo() {},
    lineTo() {},
    setLineDash() {},
    fill() {},
    stroke() {
      ops.push({ op: 'stroke', style: ctx.strokeStyle });
    },
    arc() {
      ops.push({ op: 'arc', style: ctx.fillStyle });
    },
  };
  return {
    ops,
    canvas: {
      width: 0,
      height: 0,
      getBoundingClientRect: () => ({ width: 240, height: 240 }),
      getContext: () => ctx,
    },
  };
}

/* 'rgba(31, 119, 180' — the alpha varies with level and layer, the colour identifies the layer. */
const rgb = (hex) => hexToRgba(hex, 1).replace(/, 1\)$/, '');
const countOps = (ops, kind, hex) =>
  ops.filter((o) => o.op === kind && o.style.startsWith(rgb(hex))).length;

const q8 = (base) => Array.from({ length: 8 }, (_, i) => base + i / 10);

/* 1. Truncation: a level-0 vertex owns link 0 only, so anything above it is identity-fibre
 *    padding and must not reach the tip. */
{
  const garbage = chainTips([0.3, -0.2, 9, 9, 9, 9, 9, 9], 0, robots, scene.robots);
  const zeros = chainTips([0.3, -0.2, 0, 0, 0, 0, 0, 0], 0, robots, scene.robots);
  assert.deepEqual(garbage, zeros, 'level-0 FK consumed links above the owned prefix');
  /* …and the same q at the top level must differ, or the truncation is vacuous. */
  const full = chainTips([0.3, -0.2, 9, 9, 9, 9, 9, 9], links - 1, robots, scene.robots);
  assert.notDeepEqual(full, zeros, 'top-level FK ignored the upper links');
}

/* 2. Short q: a packed mlmp.traceviz/1 row carries only the owned prefix; padding it to full
 *    dimension must not change the tips. Row taken from docs/traces/scene1-biqrrt.json. */
{
  const packedRow = [0, 1, 3.1416, 0.0];
  const short = chainTips(packedRow.slice(2), 0, robots, scene.robots);
  const padded = chainTips([3.1416, 0.0, 0, 0, 0, 0, 0, 0], 0, robots, scene.robots);
  assert.deepEqual(short, padded, 'short packed row disagreed with its padded equivalent');
}

/* 3. Interleaving: q[link * robots + robot], matching kinematic_chain.h:199-222. Index 1 is
 *    robot 1's first link. */
{
  const base = chainTips([0, 0, 0, 0, 0, 0, 0, 0], links - 1, robots, scene.robots);
  const bumped = chainTips([0, 0.7, 0, 0, 0, 0, 0, 0], links - 1, robots, scene.robots);
  assert.deepEqual(bumped[0], base[0], 'index 1 moved robot 0');
  assert.notDeepEqual(bumped[1], base[1], 'index 1 did not move robot 1');
}

/* 4. Extents: obstacle points plus every robot's full reach, padded. Recomputed here from the
 *    same rule rather than from stored numbers. */
{
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
  assert.deepEqual(sceneBounds(scene, links), {
    minX: Math.min(...xs) - PADDING,
    maxX: Math.max(...xs) + PADDING,
    minY: Math.min(...ys) - PADDING,
    maxY: Math.max(...ys) + PADDING,
  });
}

/* 5. Tag vocabulary mirrors treeLabel (mlmp/src/models/run_result.h:91-98), and the committed
 *    fixtures' own tree_codes map must agree with it. */
{
  assert.deepEqual(TREE_CODES, { 0: 'untagged', 1: 'start', 2: 'goal', 3: 'section' });
  const packed = JSON.parse(
    readFileSync(new URL('../../docs/traces/scene1-biqrrt.json', import.meta.url))
  );
  assert.deepEqual(packed.tree_codes, TREE_CODES);
}

/* 6. Python parity, restricted to the values the canvas renderer and mlmp/tools/traceGif.py
 *    genuinely share. Marker sizes, line widths and obstacle stroke alpha deliberately differ
 *    (matplotlib areas vs canvas radii); harmonising them would mean re-rendering the committed
 *    5.7 MB GIF for a cosmetic match. */
{
  const out = execFileSync(
    'python3',
    [
      '-c',
      "import json,sys; sys.path.insert(0,'mlmp/tools'); import traceGif as g; " +
        "print(json.dumps({'colors': g.TREE_COLORS, 'padding': g.PADDING, " +
        "'alpha': [0.35 + 0.65 * L / max(1, 3) for L in (0, 3)]}))",
    ],
    /* PYTHONDONTWRITEBYTECODE: importing traceGif would otherwise drop a __pycache__ into
     * mlmp/tools, which nothing ignores. */
    { encoding: 'utf8', env: { ...process.env, PYTHONDONTWRITEBYTECODE: '1' } }
  );
  const py = JSON.parse(out);
  assert.deepEqual(py.colors, TREE_COLORS, 'tree colours drifted from traceGif.py');
  assert.equal(py.padding, PADDING, 'PADDING drifted from traceGif.py');
  assert.equal(levelAlpha(0, 4), py.alpha[0]);
  assert.equal(levelAlpha(3, 4), py.alpha[1]);
  assert.equal(levelAlpha(3, 4), 1);
}

/* 7. View modes. The edge web is what collapses a dense graph into a blob, so `vertices` must
 *    drop the edges while keeping the dots, and `motion` must drop the graph entirely — it is
 *    the view that exists to show the robot rather than the search. The solution survives all
 *    three, because it is what the reader came for. */
{
  const { canvas, ops } = stubCanvas();
  const player = createTracePlayer({ canvas, scene });
  const frame = {
    vertices: [
      { level: 3, component: 1, q: q8(0.1) },
      { level: 3, component: 2, q: q8(0.2) },
    ],
    edges: [[0, 1]],
  };
  player.showPath([q8(0), q8(0.05)]);

  const draw = (mode) => {
    ops.length = 0;
    player.setViewMode(mode);
    player.pushFrame(frame);
    return {
      edges: countOps(ops, 'stroke', TREE_COLORS.start),
      dots: countOps(ops, 'arc', TREE_COLORS.goal),
      path: countOps(ops, 'stroke', PATH_COLOR),
    };
  };

  const graph = draw('graph');
  assert.ok(graph.edges > 0 && graph.dots > 0 && graph.path > 0, 'graph mode drew nothing');

  const vertices = draw('vertices');
  assert.equal(vertices.edges, 0, 'vertices mode still drew the edge web');
  assert.equal(vertices.dots, graph.dots, 'vertices mode lost the dots');
  assert.equal(vertices.path, graph.path, 'vertices mode lost the solution');

  const motion = draw('motion');
  assert.equal(motion.edges, 0);
  assert.equal(motion.dots, 0, 'motion mode still drew graph vertices');
  assert.ok(motion.path > graph.path, 'motion mode drew no more of the solution than a tip curve');

  assert.throws(() => player.setViewMode('blob'), /unknown mode/);
}

/* 8. treeCounts counts the vertices actually drawn — the level filter and the frame's
 *    vertexCount prefix both apply. A legend dimmed from a tally that ignored either would
 *    claim a colour is absent while it is on screen. */
{
  const { canvas } = stubCanvas();
  const player = createTracePlayer({ canvas, scene });
  const frame = {
    vertices: [
      { level: 0, component: 1, q: [0.1, 0.2] },
      { level: 3, component: 2, q: q8(0.2) },
      { level: 3, component: 3, q: q8(0.3) },
      { level: 3, component: 0, q: q8(0.4) },
    ],
    edges: [],
  };
  player.pushFrame(frame);
  assert.deepEqual(player.treeCounts(), { start: 1, goal: 1, section: 1, untagged: 1 });

  player.setLevels(new Set([3]));
  player.redraw();
  assert.deepEqual(
    player.treeCounts(),
    { start: 0, goal: 1, section: 1, untagged: 1 },
    'the level filter did not reach the tally'
  );

  player.setLevels(null);
  player.pushFrame({ ...frame, vertexCount: 2 });
  assert.deepEqual(
    player.treeCounts(),
    { start: 1, goal: 1, section: 0, untagged: 0 },
    'the vertexCount prefix did not reach the tally'
  );

  /* `motion` draws no vertices, so this answer comes from the standalone walk, not drawFrame. */
  player.setViewMode('motion');
  player.redraw();
  assert.deepEqual(player.treeCounts(), { start: 1, goal: 1, section: 0, untagged: 0 });
}

/* 9. tree_source. The committed fixtures carry none and are BiQRRT/QRRT runs that tagged their
 *    own trees, so an absent field must read as 'planner' — claiming 'derived' there would
 *    misattribute the planner's own labels to solver.h's connectivity fallback. */
{
  const { canvas } = stubCanvas();
  const player = createTracePlayer({ canvas, scene });
  const vertices = [{ level: 0, component: 1, q: [0.1, 0.2] }];
  player.pushFrame({ vertices, edges: [] });
  assert.equal(player.treeSource, 'planner');
  player.pushFrame({ vertices, edges: [], treeSource: 'derived' });
  assert.equal(player.treeSource, 'derived');
  /* A live mlmp.trace/1 frame reaches pushFrame verbatim, spelling it tree_source. */
  player.pushFrame({ vertices, edges: [], tree_source: 'planner' });
  assert.equal(player.treeSource, 'planner');
}

/* 10. Interpolation. densifyPath is what makes the animation the planner's motion rather than a
 *     drawing of it: the space goes the short way round the branch cut (kinematic_chain.h:114-139)
 *     and the motion validator checked every solution segment under exactly that rule. A naive
 *     lerp between +PI and -1.27 would sweep 4.41 rad through poses nobody validated instead of
 *     1.87 the other way, and on scene1 it reports a tip arc of 2.33 against the true 0.875. */
{
  const from = [Math.PI, 0, 0, 0, 0, 0, 0, 0];
  const to = [-1.27, 0, 0, 0, 0, 0, 0, 0];
  const dense = densifyPath([from, to], 16);

  assert.equal(dense.length, 17, 'one segment at 16 substeps is 17 configurations');
  assert.deepEqual(dense[0], from, 'the start configuration was not kept exactly');
  assert.deepEqual(dense[dense.length - 1], to, 'the goal configuration was not kept exactly');

  /* Every step is short and every configuration stays inside the space's own bounds. */
  let travelled = 0;
  for (let i = 1; i < dense.length; i++) {
    const step = dense[i][0] - dense[i - 1][0];
    const wrapped = Math.abs(step) > Math.PI ? 2 * Math.PI - Math.abs(step) : Math.abs(step);
    travelled += wrapped;
    assert.ok(wrapped < 0.2, `step ${i} jumped ${wrapped.toFixed(3)} rad`);
    for (const v of dense[i]) assert.ok(v >= -Math.PI && v <= Math.PI, `${v} left [-PI, PI]`);
  }
  assert.ok(
    Math.abs(travelled - 1.8716) < 0.01,
    `joint 0 travelled ${travelled.toFixed(4)} rad, expected the short way round (1.8716)`
  );

  /* Half way along the short route is past the branch cut: +PI plus 0.936 wraps to -2.206, where
   * a naive lerp would sit at +0.936, on the other side of the circle. */
  assert.ok(
    Math.abs(interpolateConfig(from, to, 0.5)[0] + 2.2058) < 0.01,
    'interpolation did not cross the branch cut the short way'
  );

  assert.deepEqual(densifyPath([], 4), []);
  assert.deepEqual(densifyPath([from], 4), [from]);
}

console.log('renderer.test.mjs: 10 groups ok');

/* Proof that web/src/cspace.js draws the real configuration space and not a plausible one.
 *
 * The load-bearing group is 2: isValidConfig is a hand port of a C++ collision test, so it is
 * replayed against every vertex the planner itself committed in the recorded runs. The planner
 * only ever adds states its own checker accepted, so one disagreement on 15 000 vertices means
 * the port is wrong — which is the only way to catch a transcription slip in a routine whose
 * epsilons decide the answer. No framework, no DOM: group 6 drives the view through a canvas
 * stub that records what it was asked to paint. */
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

import {
  buildFreeSpace,
  createCSpaceView,
  cspaceAxisLabels,
  cspaceLevel,
  isValidConfig,
  wrapSegments,
} from '../src/cspace.js';
import { PATH_COLOR, TREE_COLORS, hexToRgba } from '../src/renderer.js';

const PI = Math.PI;
const load = (path) => JSON.parse(readFileSync(new URL(path, import.meta.url)));

const scene1 = load('../../mlmp/src/inputs/experiment/scene1.json');
const scene2 = load('../../mlmp/src/inputs/experiment/scene2.json');
const scene6 = load('../../mlmp/src/inputs/experiment/scene6.json');
assert.equal(scene1.robots.length, 2);
assert.equal(scene2.robots.length, 1);
assert.equal(scene6.robots.length, 4);

/* Records every stroke/arc/text with the style in force, the way renderer.test.mjs does. */
function stubCanvas() {
  const ops = [];
  const ctx = {
    lineWidth: 0,
    font: '',
    fillStyle: '',
    strokeStyle: '',
    textAlign: '',
    textBaseline: '',
    setTransform() {},
    clearRect() {},
    save() {},
    restore() {},
    translate() {},
    rotate() {},
    beginPath() {},
    closePath() {},
    moveTo() {},
    lineTo() {},
    fill() {},
    createImageData: (w, h) => ({ width: w, height: h, data: new Uint8ClampedArray(w * h * 4) }),
    putImageData(image) {
      ops.push({ op: 'image', image });
    },
    fillText(text) {
      ops.push({ op: 'text', text, style: ctx.fillStyle });
    },
    stroke() {
      ops.push({ op: 'stroke', style: ctx.strokeStyle });
    },
    arc(x, y, r) {
      ops.push({ op: 'arc', style: ctx.fillStyle, strokeStyle: ctx.strokeStyle, r });
    },
  };
  return {
    ops,
    canvas: {
      width: 0,
      height: 0,
      getBoundingClientRect: () => ({ width: 320, height: 320 }),
      getContext: () => ctx,
    },
  };
}

const rgb = (hex) => hexToRgba(hex, 1).replace(/, 1\)$/, '');
const countOps = (ops, kind, hex) =>
  ops.filter((o) => o.op === kind && o.style.startsWith(rgb(hex))).length;

/* 1. Which level is 2-D: dim(level k) = robots * (k + 1). Two robots make level 0 the square
 *    (one joint each); one robot makes it level 1 (the textbook two-link arm); four robots never
 *    reach 2 at all, and the module has to admit that rather than slice something else. */
{
  assert.equal(cspaceLevel(scene1), 0);
  assert.equal(cspaceLevel(scene2), 1);
  assert.equal(cspaceLevel(scene6), -1);

  assert.deepEqual(cspaceAxisLabels(scene1, 0), [
    '\u03b8 robot 1 \u00b7 joint 1',
    '\u03b8 robot 2 \u00b7 joint 1',
  ]);
  assert.deepEqual(cspaceAxisLabels(scene2, 1), ['\u03b8 joint 1', '\u03b8 joint 2']);
  assert.deepEqual(cspaceAxisLabels(scene6, -1), []);

  /* The axis mapping is the interleaved state layout, not a convention this module picked:
   * coordinate i is robot i % robots, link floor(i / robots). */
  for (const [scene, level] of [[scene1, 0], [scene2, 1]]) {
    assert.equal(scene.robots.length * (level + 1), 2);
  }
}

/* 2. Agreement with the C++ on data the C++ produced. Every committed fixture vertex is a state
 *    KinematicChainValidityChecker accepted at its own level, so isValidConfig must accept all of
 *    them. Vertices above the 2-D level are checked twice: at their own level, and — because a
 *    level is a strict relaxation of the one above it, the same environment with fewer links —
 *    projected down to the 2-D level, which must also hold.
 *
 *    One wrinkle, and it is the fixtures' and not the port's: packTrace.py rounds angles to four
 *    decimals (--round, default 4), so a committed row is the planner's state ±5e-5 per
 *    coordinate. Measured, 5 of the 49 377 rows across these four fixtures graze an obstacle
 *    corner by ~7e-5 after that rounding — one of them, scene1-biqrrt's level-3
 *    (-0.9296, -2.0755, ...), passes through the corner (0.5, 0.7) at parameter 0.9994 of the
 *    obstacle edge. Each such row is only accepted here if some configuration inside its own
 *    rounding box is valid, which is a statement about the recorded state and not a tolerance in
 *    the checker: the checker itself stays exact. */
{
  const PACK_HALF_STEP = 5e-5;

  /* Deterministic probe of the rounding box: a fixed LCG, so a run that passes passes every time.
   * 512 draws finds a valid neighbour of a 7e-5 graze in the first handful. */
  function withinPackingNoise(scene, level, q) {
    let seed = 0x9e3779b9;
    const next = () => {
      seed = (seed * 1664525 + 1013904223) >>> 0;
      return seed / 4294967296;
    };
    const probe = new Array(q.length);
    for (let attempt = 0; attempt < 512; attempt++) {
      for (let i = 0; i < q.length; i++) {
        probe[i] = q[i] + (next() - 0.5) * 2 * PACK_HALF_STEP;
      }
      if (isValidConfig(scene, level, probe)) return true;
    }
    return false;
  }

  let checked = 0;
  let projected = 0;
  let grazed = 0;
  for (const [file, scene, cLevel] of [
    ['scene1-biqrrt', scene1, 0],
    ['scene1-qrrt', scene1, 0],
    ['scene2-biqrrt', scene2, 1],
    ['scene2-qrrt', scene2, 1],
  ]) {
    const fixture = load(`../../docs/traces/${file}.json`);
    assert.equal(fixture.robots, scene.robots.length);
    assert.equal(fixture.links, scene.metadata.n);

    let atLevel = 0;
    for (const row of fixture.vertices) {
      const level = row[0];
      const q = row.slice(2);
      assert.equal(q.length, fixture.robots * (level + 1), `${file}: packed row width`);

      if (!isValidConfig(scene, level, q)) {
        assert.ok(
          withinPackingNoise(scene, level, q),
          `${file}: planner vertex at level ${level} rejected by the JS checker, and no ` +
            `configuration within the packer's rounding is valid either: ${q.join(', ')}`,
        );
        grazed++;
      }
      checked++;
      if (level === cLevel) atLevel++;
      if (level > cLevel) {
        assert.ok(
          isValidConfig(scene, cLevel, q),
          `${file}: projection of a level-${level} vertex is blocked at level ${cLevel}`,
        );
        projected++;
      }
    }
    assert.ok(atLevel > 0, `${file}: no vertex at the 2-D level ${cLevel}`);
  }
  console.log(
    `  group 2: ${checked} committed vertices accepted (${grazed} only within the packer's ` +
      `4-decimal rounding), ${projected} of them also valid as 2-D projections`,
  );
  assert.ok(checked > 10000, 'too few vertices replayed to prove anything');
  /* A port that was wrong in any structural way would not miss 5 rows out of 49 377; it would
   * miss thousands, or none because it accepted everything. */
  assert.ok(grazed * 1000 < checked, `${grazed} of ${checked} vertices disagree: that is a bug`);

  /* The other direction: a deliberately broken configuration must be rejected, or the group above
   * would pass with a checker that returns true. Robot 0's first two links folded back on each
   * other is a self-intersection by construction. */
  const folded = new Array(scene2.metadata.n).fill(0);
  folded[1] = PI;
  folded[2] = 0.3;
  assert.equal(isValidConfig(scene2, 2, folded), false, 'a folded chain was accepted');
}

/* 3. The environment bites, both halves of it.
 *
 *    (a) Obstacles, taken from scene1's own first rectangle: robot 0 aimed at its centre must be
 *        blocked, aimed away must be free.
 *    (b) The padded boundary box. No committed scene can exercise it: addBoundarySegments pads by
 *        1.0 around every robot's *full* reach, so a chain is always at least 1.0 short of its own
 *        box — measured below and asserted, because that is a property of the scene files and not
 *        of this port. What does reach the box is the one asymmetry in the C++: the space carries a
 *        single linkLength, Scene::getJointLength() = robots_[0].jointLength (scene.h:98), while the
 *        box accounts every robot with its *own* declared jointLength. A second robot declaring a
 *        short link therefore swings a long one. The scene below is scene2's link length and joint
 *        count in that two-robot shape, with no obstacles so the box is the only thing to hit. */
{
  const robot0 = scene1.robots[0];
  const rect = scene1.obstacles[0].points;
  const cx = rect.reduce((s, p) => s + p.x, 0) / rect.length;
  const cy = rect.reduce((s, p) => s + p.y, 0) / rect.length;
  const toward = Math.atan2(cy - robot0.pinnedPosition.y, cx - robot0.pinnedPosition.x);
  /* Four links of 0.1 against a centre 0.24 away: the arm reaches it when it points there. */
  const reach = robot0.jointLength * scene1.metadata.n;
  assert.ok(Math.hypot(cx - robot0.pinnedPosition.x, cy - robot0.pinnedPosition.y) < reach);

  const straight = (a0, a1) => {
    const q = new Array(scene1.metadata.n * 2).fill(0);
    q[0] = a0;
    q[1] = a1;
    return q;
  };
  assert.equal(isValidConfig(scene1, 3, straight(toward, 0)), false, 'obstacle 1 was not hit');
  assert.equal(isValidConfig(scene1, 3, straight(toward + PI, 0)), true, 'free pose rejected');

  for (const scene of [scene1, scene2, scene6]) {
    const links = scene.metadata.n;
    const xs = scene.robots.map((r) => r.pinnedPosition.x + r.jointLength * links);
    const own = scene.robots.map((r) => r.pinnedPosition.x + scene.robots[0].jointLength * links);
    assert.ok(
      Math.max(...own) + 1.0 - Math.max(...xs) >= 1.0,
      'a committed scene can reach its own boundary box; the box test below is no longer the only way',
    );
  }

  const links = scene2.metadata.n;
  const length = scene2.robots[0].jointLength;
  const pin = scene2.robots[0].pinnedPosition;
  const zeros = new Array(links).fill(0);
  const boxScene = {
    obstacles: [],
    robots: [
      { ...scene2.robots[0], startAngles: zeros, goalAngles: zeros },
      {
        id: 2,
        pinnedPosition: { x: pin.x + 2, y: pin.y + 0.5 },
        jointLength: 0.01,
        startAngles: zeros,
        goalAngles: zeros,
      },
    ],
    metadata: { n: links, r: 2 },
  };
  /* The right wall, by Scene::addBoundarySegments' own rule. */
  const right = Math.max(...boxScene.robots.map((r) => r.pinnedPosition.x + r.jointLength * links)) + 1.0;
  assert.ok(boxScene.robots[1].pinnedPosition.x + length * links > right, 'robot 2 cannot leave the box');

  const pose = (a0, a1) => {
    const q = new Array(links * 2).fill(0);
    q[0] = a0;
    q[1] = a1;
    return q;
  };
  /* Robot 0 straight up, robot 1 straight out to +x past the wall, then straight down inside it.
   * Only robot 1's direction changes between the two. */
  assert.equal(isValidConfig(boxScene, links - 1, pose(PI / 2, 0)), false, 'the box let an arm out');
  assert.equal(isValidConfig(boxScene, links - 1, pose(PI / 2, -PI / 2)), true, 'the box blocked an arm inside it');
}

/* 4. The map itself: cell centres over [-PI, PI]^2, free[iy * size + ix] with ix along q[0] and
 *    iy along q[1] upward. Both values must occur — an all-free map would make the panel a blank
 *    square, an all-blocked one a grey one — and the scene's own start configuration must land in
 *    a free cell, since the planner starts there. */
{
  const size = 64;
  const { size: reported, free } = buildFreeSpace(scene1, 0, size);
  assert.equal(reported, size);
  assert.equal(free.length, size * size);

  const blocked = free.length - free.reduce((s, v) => s + v, 0);
  assert.ok(blocked > 0, 'no blocked cell: the obstacles are missing from the map');
  assert.ok(blocked < free.length, 'every cell blocked: the environment is wrong');

  const cell = (q) => Math.min(size - 1, Math.max(0, Math.floor(((q + PI) / (2 * PI)) * size)));
  const startQ = [0, 1].map((i) => (PI * scene1.robots[i].startAngles[0]) / 180);
  assert.equal(isValidConfig(scene1, 0, startQ), true, 'the scene start is not valid');
  assert.equal(free[cell(startQ[1]) * size + cell(startQ[0])], 1, 'the start cell is blocked');

  /* Spot check against the checker itself, so the indexing convention cannot drift. */
  const step = (2 * PI) / size;
  for (const [ix, iy] of [[0, 0], [7, 31], [31, 7], [size - 1, size - 1], [40, 12]]) {
    const q = [-PI + (ix + 0.5) * step, -PI + (iy + 0.5) * step];
    assert.equal(free[iy * size + ix], isValidConfig(scene1, 0, q) ? 1 : 0, `cell ${ix},${iy}`);
  }

  assert.throws(() => buildFreeSpace(scene6, 0, 8), /not 2-D/);
}

/* 5. Wrapping. KinematicChainSpace::interpolate takes the short way round, so a motion whose
 *    coordinates differ by more than PI leaves the square and re-enters on the other side. Drawn
 *    as one chord it would be a line straight across the picture, in the wrong direction. */
{
  const single = wrapSegments([0.1, 0.2], [0.3, 0.4]);
  assert.equal(single.length, 1);
  assert.deepEqual(single[0], [[0.1, 0.2], [0.3, 0.4]]);

  const wrapped = wrapSegments([3.0, 0.5], [-3.0, 0.6]);
  assert.equal(wrapped.length, 2, 'a wrapped motion was drawn as one chord');
  assert.ok(Math.abs(wrapped[0][1][0] - PI) < 1e-12, 'first piece does not stop at +PI');
  assert.ok(Math.abs(wrapped[1][0][0] + PI) < 1e-12, 'second piece does not resume at -PI');
  assert.deepEqual(wrapped[1][1], [-3.0, 0.6]);
  /* The y coordinate is carried across the cut, not reset. */
  assert.ok(wrapped[0][1][1] > 0.5 && wrapped[0][1][1] < 0.6);

  const both = wrapSegments([3.0, 3.0], [-3.0, -3.0]);
  assert.ok(both.length >= 2, 'a double wrap was drawn as one chord');
  for (const [p0, p1] of both) {
    for (const v of [p0[0], p0[1], p1[0], p1[1]]) {
      assert.ok(v >= -PI - 1e-12 && v <= PI + 1e-12, 'a piece left the square');
    }
  }
}

/* 6. The view. A scene with no 2-D level must say so and draw nothing else; a scene with one must
 *    draw its own level solid and the levels above it as faint projections, which is the whole
 *    reason the panel shows more than one level's worth of dots. */
{
  const unavailable = stubCanvas();
  const none = createCSpaceView({ canvas: unavailable.canvas, scene: { ...scene6, name: 'scene6' } });
  assert.equal(none.available, false);
  assert.equal(none.level, -1);
  none.redraw();
  const text = unavailable.ops.filter((o) => o.op === 'text');
  assert.equal(text.length, 1, 'an unavailable panel drew more than its one line');
  assert.equal(text[0].text, 'scene6: the lowest bundle level is 4-D \u2014 no exact 2-D view');
  assert.equal(unavailable.ops.filter((o) => o.op === 'image').length, 0, 'drew a map anyway');

  const { canvas, ops } = stubCanvas();
  const view = createCSpaceView({ canvas, scene: scene1 });
  assert.equal(view.available, true);
  assert.equal(view.level, 0);

  const frame = {
    vertices: [
      [0, 1, 0.2, 0.3],
      [0, 2, -0.4, 0.8],
      [3, 1, 0.5, 0.6, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
    ],
    edges: [[0, 1], [0, 2]],
  };
  ops.length = 0;
  view.pushFrame(frame);

  /* Dots are the small arcs; the start ring (radius 5) and the phase marker (3.5) are told apart
   * by radius, because a stroked arc carries whatever fillStyle was last set. */
  const dotsOf = (hex) =>
    ops.filter((o) => o.op === 'arc' && o.r <= 2.2 && o.style.startsWith(rgb(hex)));
  assert.equal(dotsOf(TREE_COLORS.start).length, 2, 'own-level and projected dots');
  assert.equal(dotsOf(TREE_COLORS.goal).length, 1);
  const own = dotsOf(TREE_COLORS.start).find((o) => o.style.endsWith('0.9)'));
  const projectedDot = dotsOf(TREE_COLORS.start).find((o) => o.style.endsWith('0.25)'));
  assert.ok(own && projectedDot, 'projected vertices are not drawn at 25% alpha');
  assert.equal(projectedDot.r, own.r / 2, 'projected vertices are not drawn at half radius');

  /* Only the edge between two own-level vertices is drawn; the one climbing to level 3 is not a
   * motion in this space. Edges stroke at 0.3 alpha, the start ring at 1. */
  assert.equal(
    ops.filter((o) => o.op === 'stroke' && o.style === hexToRgba(TREE_COLORS.start, 0.3)).length,
    1,
    'edge filtering is wrong',
  );

  /* The start ring and the goal cross are always there, in the tree colours. */
  assert.ok(countOps(ops, 'stroke', TREE_COLORS.goal) >= 1, 'no goal marker');
  assert.ok(
    ops.some((o) => o.op === 'arc' && o.r === 5 && o.strokeStyle.startsWith(rgb(TREE_COLORS.start))),
    'no start ring',
  );

  ops.length = 0;
  view.showPath([
    [0.1, 0.2, 0, 0, 0, 0, 0, 0],
    [0.4, 0.5, 0, 0, 0, 0, 0, 0],
    [0.9, 0.7, 0, 0, 0, 0, 0, 0],
  ]);
  view.setPhase(0.5);
  view.redraw();
  assert.equal(countOps(ops, 'stroke', PATH_COLOR), 2, 'the path polyline is missing a leg');
  assert.equal(
    ops.filter((o) => o.op === 'arc' && o.r === 3.5 && o.style.startsWith(rgb(PATH_COLOR))).length,
    1,
    'the phase marker is missing',
  );

  /* showPath([]) clears, same as null. */
  ops.length = 0;
  view.showPath([]);
  view.redraw();
  assert.equal(countOps(ops, 'stroke', PATH_COLOR), 0, 'showPath([]) left the old path on screen');

  /* The background is an ImageData, and it is built once per canvas geometry: a second redraw
   * must not rebuild it. */
  const first = ops.filter((o) => o.op === 'image');
  assert.equal(first.length, 1);
  ops.length = 0;
  view.redraw();
  const second = ops.filter((o) => o.op === 'image');
  assert.equal(second.length, 1);
  assert.equal(second[0].image, first[0].image, 'the free-space map was rasterised twice');

  /* reset() is a new run on the same scene: the tree and the path go, the rasterised map stays,
   * because nothing it depends on changed. */
  view.pushFrame(frame);
  view.showPath([[0.1, 0.2, 0, 0, 0, 0, 0, 0], [0.4, 0.5, 0, 0, 0, 0, 0, 0]]);
  view.setPhase(1);
  ops.length = 0;
  view.reset();
  assert.equal(dotsOf(TREE_COLORS.start).length, 0, 'reset left the previous run on screen');
  assert.equal(countOps(ops, 'stroke', PATH_COLOR), 0, 'reset left the previous path on screen');
  const afterReset = ops.filter((o) => o.op === 'image');
  assert.equal(afterReset.length, 1, 'reset stopped drawing the map');
  assert.equal(afterReset[0].image, first[0].image, 'reset threw the free-space map away');
  assert.ok(
    ops.some((o) => o.op === 'arc' && o.r === 5),
    'reset dropped the start ring, which belongs to the scene and not to the run',
  );

  view.setScene(scene2);
  assert.equal(view.level, 1);
  assert.equal(view.available, true);
  view.destroy();
}

/* 7. Cost. The map is rebuilt whenever the scene changes, in front of a user waiting for a
 *    panel, so it has a budget: 192 x 192 = 36 864 exact validity tests. */
{
  const size = 192;
  const started = process.hrtime.bigint();
  const { free } = buildFreeSpace(scene2, 1, size);
  const scene2Ms = Number(process.hrtime.bigint() - started) / 1e6;

  const startedScene1 = process.hrtime.bigint();
  const map = buildFreeSpace(scene1, 0, size);
  const scene1Ms = Number(process.hrtime.bigint() - startedScene1) / 1e6;

  assert.equal(map.free.length, size * size);
  assert.ok(free.some((v) => v === 1) && free.some((v) => v === 0));
  console.log(
    `  group 7: buildFreeSpace ${size}x${size} — scene1 level 0 ${scene1Ms.toFixed(1)} ms, ` +
      `scene2 level 1 ${scene2Ms.toFixed(1)} ms`,
  );
}

console.log('cspace.test.mjs: 7 groups ok');

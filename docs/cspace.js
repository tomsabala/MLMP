/* The configuration-space panel: the exact free/blocked map of one bundle level, computed here
 * rather than sketched. It exists because the workspace view can only ever show chain tips —
 * a 2-D shadow of an 8-D search — while a bundle level whose dimension is exactly 2 *is* a
 * square, and drawing it costs nothing but honesty.
 *
 * dim(level k) = robots * (k + 1), so the 2-D level is k = 0 for a two-robot scene (x = robot 1's
 * first joint, y = robot 2's first joint) and k = 1 for a one-robot scene (the textbook two-link
 * arm picture). A scene with three or more robots has no 2-D level at all and this module says so
 * instead of slicing a higher level and calling it a configuration space.
 *
 * isValidConfig is a literal port of KinematicChainValidityChecker::isValidImpl
 * (mlmp/src/state/kinematic_chain.h:168-281): same forward kinematics, same test order (a robot's
 * own links, then robot against robot, then everything against the environment), and in
 * particular the same segment-intersection routine, float epsilon and all. The branches are
 * copied rather than replaced by a general-position predicate on purpose: that routine treats
 * parallel segments as missing and touching endpoints as clear, and the padded boundary box is
 * full of exactly those cases, so a prettier test disagrees along the edges of the square where
 * the picture is most load-bearing. The proof it agrees is web/test/cspace.test.mjs group 2,
 * which replays every committed level-k vertex the planner ever added.
 *
 * The environment mirrors Scene::loadObstacles/addBoundarySegments (mlmp/src/models/scene.h:350-385):
 * every obstacle polygon closed into segments, plus a box padded by 1.0 around the obstacle points
 * and every robot's *full* reach — jointLength * metadata.n, independent of the level, which is
 * why the same box bounds level 0 and the top level.
 *
 * No DOM lookup beyond the canvas handed in, no event listeners, no module state other than the
 * per-scene map cache. The page owns resize and calls redraw(). */

import { PATH_COLOR, TREE_COLORS, hexToRgba } from './renderer.js';

const PI = Math.PI;
const TWO_PI = 2 * Math.PI;

/* std::numeric_limits<float>::epsilon() and <double>::epsilon(). The C++ compares the
 * intersection numerators against the *float* epsilon while rejecting the denominator against the
 * double one (kinematic_chain.h:255-277); both constants are reproduced because that asymmetry is
 * what decides the degenerate cases. */
const FLT_EPSILON = 1.1920928955078125e-7;
const DBL_EPSILON = Number.EPSILON;

/* Blocked cells use the obstacle grey of the workspace view (renderer.js drawStatic fills
 * rgba(128,128,128,0.5)), a touch denser because here it is a flat region and not an outlined
 * polygon. Free space is left transparent so the page background shows through. */
const BLOCKED_RGBA = [128, 128, 128, Math.round(0.55 * 255)];

const TREE_BY_COMPONENT = { 0: 'untagged', 1: 'start', 2: 'goal', 3: 'section' };

/* Per-scene derived geometry, keyed by the parsed scene object: the obstacle+boundary segment
 * list is the same for every level and every cell of the map, so it is built once. */
const sceneCache = new WeakMap();

/* Per-scene free-space maps, keyed by `${level}:${size}`. A map costs ~10 ms (measured below);
 * recomputing it inside redraw would make every frame of playback pay it. */
const mapCache = new WeakMap();

export function cspaceLevel(scene) {
  const robots = scene.robots.length;
  const links = scene.metadata.n;
  for (let k = 0; k < links; k++) {
    if (robots * (k + 1) === 2) return k;
  }
  return -1;
}

/* The two coordinates a 2-D level owns are q[0] and q[1], and the state layout is interleaved by
 * link — q[link * robots + robot] (kinematic_chain.h:210) — so coordinate i belongs to
 * robot i % robots, link floor(i / robots). That is the whole reason a lower level is a prefix of
 * a higher one, and it is why the axes differ between the two shapes of 2-D level. */
export function cspaceAxisLabels(scene, level) {
  if (level < 0) return [];
  const robots = scene.robots.length;
  return [0, 1].map((i) => {
    const robot = (i % robots) + 1;
    const joint = Math.floor(i / robots) + 1;
    return robots === 1 ? `\u03b8 joint ${joint}` : `\u03b8 robot ${robot} \u00b7 joint ${joint}`;
  });
}

function sceneGeometry(scene) {
  let geom = sceneCache.get(scene);
  if (geom !== undefined) return geom;

  const robots = scene.robots.length;
  const links = scene.metadata.n;
  const pins = new Float64Array(robots * 2);
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;

  for (let r = 0; r < robots; r++) {
    const robot = scene.robots[r];
    pins[r * 2] = robot.pinnedPosition.x;
    pins[r * 2 + 1] = robot.pinnedPosition.y;
    const reach = robot.jointLength * links;
    minX = Math.min(minX, robot.pinnedPosition.x - reach);
    maxX = Math.max(maxX, robot.pinnedPosition.x + reach);
    minY = Math.min(minY, robot.pinnedPosition.y - reach);
    maxY = Math.max(maxY, robot.pinnedPosition.y + reach);
  }

  const segments = [];
  for (const obstacle of scene.obstacles) {
    const points = obstacle.points;
    for (let i = 0; i < points.length; i++) {
      const next = points[(i + 1) % points.length];
      segments.push(points[i].x, points[i].y, next.x, next.y);
      minX = Math.min(minX, points[i].x);
      maxX = Math.max(maxX, points[i].x);
      minY = Math.min(minY, points[i].y);
      maxY = Math.max(maxY, points[i].y);
    }
  }

  const padding = 1.0;
  const left = minX - padding;
  const right = maxX + padding;
  const bottom = minY - padding;
  const top = maxY + padding;
  segments.push(left, bottom, right, bottom);
  segments.push(right, bottom, right, top);
  segments.push(right, top, left, top);
  segments.push(left, top, left, bottom);

  geom = {
    robots,
    links,
    /* One length for every robot, because KinematicChainSpace carries a single linkLength and
     * Solver hands it Scene::getJointLength() — robots_[0].jointLength (scene.h:98). A scene whose
     * robots disagreed would already be planned with robot 0's length, so the picture must lie the
     * same way the planner does. */
    linkLength: scene.robots[0].jointLength,
    pins,
    env: Float64Array.from(segments),
    scratch: new Map(),
  };
  sceneCache.set(scene, geom);
  return geom;
}

/* kinematic_chain.h:246-281, branch for branch. `a`/`b` are flat [x0,y0,x1,y1,...] arrays and
 * `ia`/`ib` the offset of the segment inside them. */
function segmentsIntersect(a, ia, b, ib) {
  const s10x = a[ia + 2] - a[ia];
  const s10y = a[ia + 3] - a[ia + 1];
  const s32x = b[ib + 2] - b[ib];
  const s32y = b[ib + 3] - b[ib + 1];
  const denom = s10x * s32y - s32x * s10y;

  if (Math.abs(denom) < DBL_EPSILON) return false;

  const denomPositive = denom > 0;

  const s02x = a[ia] - b[ib];
  const s02y = a[ia + 1] - b[ib + 1];
  const sNumer = s10x * s02y - s10y * s02x;

  if ((sNumer < FLT_EPSILON) === denomPositive) return false;

  const tNumer = s32x * s02y - s32y * s02x;

  if ((tNumer < FLT_EPSILON) === denomPositive) return false;

  if (
    (sNumer - denom > -FLT_EPSILON) === denomPositive ||
    (tNumer - denom > FLT_EPSILON) === denomPositive
  ) {
    return false;
  }

  return true;
}

/* Forward kinematics of every robot truncated to its first `numLinks` links, written into `out` as
 * robot-major blocks of `numLinks` segments. Mirrors computeRobotSegments (kinematic_chain.h:199-222). */
function buildSegments(geom, q, numLinks, out) {
  const robots = geom.robots;
  const length = geom.linkLength;
  for (let r = 0; r < robots; r++) {
    let theta = 0;
    let x = geom.pins[r * 2];
    let y = geom.pins[r * 2 + 1];
    let o = r * numLinks * 4;
    for (let j = 0; j < numLinks; j++) {
      theta += q[j * robots + r];
      const xNext = x + Math.cos(theta) * length;
      const yNext = y + Math.sin(theta) * length;
      out[o] = x;
      out[o + 1] = y;
      out[o + 2] = xNext;
      out[o + 3] = yNext;
      o += 4;
      x = xNext;
      y = yNext;
    }
  }
}

function hitsEnvironment(geom, segs, offset) {
  const env = geom.env;
  for (let e = 0; e < env.length; e += 4) {
    if (segmentsIntersect(segs, offset, env, e)) return true;
  }
  return false;
}

/* isValidImpl's three tests, in its order. Splitting them out of isValidConfig keeps buildFreeSpace
 * from owning a second copy of the rules. */
function segmentsValid(geom, segs, numLinks) {
  const robots = geom.robots;

  for (let r = 0; r < robots; r++) {
    const base = r * numLinks * 4;
    for (let i = 0; i < numLinks; i++) {
      for (let j = i + 1; j < numLinks; j++) {
        if (segmentsIntersect(segs, base + i * 4, segs, base + j * 4)) return false;
      }
    }
  }

  for (let a = 0; a < robots; a++) {
    for (let b = a + 1; b < robots; b++) {
      for (let i = 0; i < numLinks; i++) {
        for (let j = 0; j < numLinks; j++) {
          if (segmentsIntersect(segs, (a * numLinks + i) * 4, segs, (b * numLinks + j) * 4)) {
            return false;
          }
        }
      }
    }
  }

  const total = robots * numLinks;
  for (let i = 0; i < total; i++) {
    if (hitsEnvironment(geom, segs, i * 4)) return false;
  }

  return true;
}

function scratchFor(geom, numLinks) {
  let buffer = geom.scratch.get(numLinks);
  if (buffer === undefined) {
    buffer = new Float64Array(geom.robots * numLinks * 4);
    geom.scratch.set(numLinks, buffer);
  }
  return buffer;
}

/* `q` may be the level's own prefix (robots * (level + 1) values, which is what a packed
 * mlmp.traceviz/1 row carries) or a longer full-dimension configuration; only the prefix the level
 * owns is read, exactly as the level's own SpaceInformation would. */
export function isValidConfig(scene, level, q) {
  const geom = sceneGeometry(scene);
  const numLinks = level + 1;
  if (q.length < geom.robots * numLinks) {
    throw new Error(`isValidConfig: level ${level} needs ${geom.robots * numLinks} coordinates`);
  }
  const segs = scratchFor(geom, numLinks);
  buildSegments(geom, q, numLinks, segs);
  return segmentsValid(geom, segs, numLinks);
}

/* The free/blocked map of a 2-D level over [-PI, PI]^2, sampled at cell centres.
 * `free[iy * size + ix]` is 1 when the configuration is valid; ix runs along q[0] and iy along
 * q[1] *upward*, so a reader who wants screen rows has to flip y (drawBackground does).
 *
 * Measured on this workstation (node 22, web/test/cspace.test.mjs group 7): size 192 = 36 864
 * exact validity tests in 8-12 ms warm and ~21 ms on the first build of a page's life, both for
 * scene1 level 0 (2 robots x 1 link, 12 environment segments) and scene2 level 1 (1 robot x
 * 2 links, 20 segments) — inside the 60 ms the panel is allowed even cold, so the sampling stays
 * at 192 and nothing is interpolated. The column hoist below is part of why: robot 0's first link
 * is fixed by q[0] alone, so a column that is already in an obstacle is blocked outright and its
 * 192 cells never run the full test. */
export function buildFreeSpace(scene, level, size) {
  const geom = sceneGeometry(scene);
  const numLinks = level + 1;
  const free = new Uint8Array(size * size);
  if (level < 0 || geom.robots * numLinks !== 2) {
    throw new Error(`buildFreeSpace: level ${level} of this scene is not 2-D`);
  }

  const segs = scratchFor(geom, numLinks);
  const q = new Float64Array(2);
  const step = TWO_PI / size;

  for (let ix = 0; ix < size; ix++) {
    q[0] = -PI + (ix + 0.5) * step;

    /* Necessary condition, not a second copy of the rules: segment 0 of robot 0 is fixed by q[0]
     * alone (it is the only one whose indices stop at 0), so if it is already in the environment
     * no value of q[1] can rescue the column. Cells that survive still run segmentsValid. */
    q[1] = 0;
    buildSegments(geom, q, numLinks, segs);
    if (hitsEnvironment(geom, segs, 0)) continue;

    for (let iy = 0; iy < size; iy++) {
      q[1] = -PI + (iy + 0.5) * step;
      buildSegments(geom, q, numLinks, segs);
      if (segmentsValid(geom, segs, numLinks)) free[iy * size + ix] = 1;
    }
  }

  return { size, free };
}

function cachedFreeSpace(scene, level, size) {
  let perScene = mapCache.get(scene);
  if (perScene === undefined) {
    perScene = new Map();
    mapCache.set(scene, perScene);
  }
  const key = `${level}:${size}`;
  let map = perScene.get(key);
  if (map === undefined) {
    map = buildFreeSpace(scene, level, size);
    perScene.set(key, map);
  }
  return map;
}

/* The short way round, matching KinematicChainSpace::interpolate (kinematic_chain.h:112-139): a
 * coordinate pair more than PI apart moves through the wrap, not across the square. */
function shortDelta(from, to) {
  const diff = to - from;
  if (Math.abs(diff) <= PI) return diff;
  return diff > 0 ? diff - TWO_PI : diff + TWO_PI;
}

/* The far side of the square, for a coordinate that has just arrived at a wall travelling in
 * direction `delta`. A plain fmod-style wrap is not enough here: the cut lands *exactly* on +PI
 * or -PI, which both wrap to themselves, and the next iteration would then re-cut at t = 0
 * forever. */
function reenter(v, delta) {
  if (delta > 0 && v >= PI - 1e-12) return -PI;
  if (delta < 0 && v <= -PI + 1e-12) return PI;
  return v;
}

/* One wrap-aware motion, cut into the pieces that fit inside [-PI, PI]^2. Drawing the straight
 * chord instead would put a line across the whole square where the planner's own interpolation
 * moved a few hundredths of a radian. Both coordinates can wrap on the same motion, so this
 * iterates; three cuts is the most a two-coordinate motion can need (one per coordinate, plus the
 * degenerate start-on-a-wall case). */
export function wrapSegments(a, b) {
  const pieces = [];
  let from = a;
  for (let guard = 0; guard < 3; guard++) {
    const d0 = shortDelta(from[0], b[0]);
    const d1 = shortDelta(from[1], b[1]);
    let t = 1;
    if (d0 !== 0) {
      const ti = ((d0 > 0 ? PI : -PI) - from[0]) / d0;
      if (ti < t) t = ti;
    }
    if (d1 !== 0) {
      const ti = ((d1 > 0 ? PI : -PI) - from[1]) / d1;
      if (ti < t) t = ti;
    }
    if (t >= 1) {
      pieces.push([from, b]);
      return pieces;
    }
    const cut = [from[0] + d0 * t, from[1] + d1 * t];
    /* t <= 0 means the motion starts on the wall it is leaving — a scene whose start angle is
     * exactly 180 degrees does — so there is no piece to draw, only a side to swap. */
    if (t > 0) pieces.push([from, cut]);
    from = [reenter(cut[0], d0), reenter(cut[1], d1)];
  }
  pieces.push([from, b]);
  return pieces;
}

function canvasSize(canvas) {
  const rect = canvas.getBoundingClientRect();
  return {
    w: Math.max(1, Math.round(rect.width)),
    h: Math.max(1, Math.round(rect.height || rect.width)),
  };
}

/* Same device-pixel-ratio handling as renderer.js sizeCanvas: the backing store is scaled, the
 * drawing code keeps working in CSS pixels. */
function sizeCanvas(canvas) {
  const ratio = (typeof window !== 'undefined' && window.devicePixelRatio) || 1;
  const { w, h } = canvasSize(canvas);
  canvas.width = Math.round(w * ratio);
  canvas.height = Math.round(h * ratio);
  return { w, h, ratio };
}

/* Room for the y tick labels on the left and the x axis label underneath; the plot itself stays
 * square, because the space it shows is. */
const MARGIN = { left: 38, right: 10, top: 10, bottom: 30 };
const MAP_SIZE = 192;

export function createCSpaceView({ canvas, scene } = {}) {
  if (!canvas || !scene) {
    throw new Error('createCSpaceView: canvas and scene are required');
  }

  let sceneData = null;
  let level = -1;
  let labels = [];
  let frame = null;
  let packedRows = false;
  let path = null;
  let phase = 0;
  let endpoints = null;
  /* The background is rasterised at device resolution, so it is rebuilt only when the canvas
   * geometry or the scene changes — never per frame. */
  let background = null;

  function applyScene(next) {
    sceneData = next;
    level = cspaceLevel(next);
    labels = cspaceAxisLabels(next, level);
    frame = null;
    path = null;
    phase = 0;
    background = null;
    endpoints = level < 0 ? null : sceneEndpoints(next, level);
  }

  /* Scene JSON angles are integer degrees (scene.h:340-344). Coordinate i of the 2-D level belongs
   * to robot i % robots, link floor(i / robots), the same mapping cspaceAxisLabels names. */
  function sceneEndpoints(next, lvl) {
    const robots = next.robots.length;
    const pick = (key) =>
      [0, 1].map((i) => {
        const robot = next.robots[i % robots];
        return (PI * robot[key][Math.floor(i / robots)]) / 180;
      });
    return { start: pick('startAngles'), goal: pick('goalAngles'), level: lvl };
  }

  function layout(w, h) {
    const side = Math.max(
      1,
      Math.min(w - MARGIN.left - MARGIN.right, h - MARGIN.top - MARGIN.bottom),
    );
    const x = MARGIN.left + (w - MARGIN.left - MARGIN.right - side) / 2;
    const y = MARGIN.top + (h - MARGIN.top - MARGIN.bottom - side) / 2;
    return { x, y, side };
  }

  function projector(box) {
    return (q0, q1) => [
      box.x + ((q0 + PI) / TWO_PI) * box.side,
      box.y + box.side - ((q1 + PI) / TWO_PI) * box.side,
    ];
  }

  /* putImageData ignores the transform, so the map is written straight into device pixels and the
   * nearest map cell is read per pixel. One ImageData per canvas size, not per redraw. */
  function drawBackground(ctx, box, ratio) {
    if (typeof ctx.createImageData !== 'function' || typeof ctx.putImageData !== 'function') {
      return;
    }
    const px = Math.round(box.x * ratio);
    const py = Math.round(box.y * ratio);
    const pw = Math.max(1, Math.round(box.side * ratio));
    const ph = pw;

    if (background === null || background.pw !== pw || background.px !== px || background.py !== py) {
      const { size, free } = cachedFreeSpace(sceneData, level, MAP_SIZE);
      const image = ctx.createImageData(pw, ph);
      const data = image.data;
      const [br, bg, bb, ba] = BLOCKED_RGBA;
      for (let j = 0; j < ph; j++) {
        const iy = Math.min(size - 1, Math.floor(((ph - 1 - j) / ph) * size));
        const row = iy * size;
        for (let i = 0; i < pw; i++) {
          const ix = Math.min(size - 1, Math.floor((i / pw) * size));
          if (free[row + ix] === 1) continue;
          const o = (j * pw + i) * 4;
          data[o] = br;
          data[o + 1] = bg;
          data[o + 2] = bb;
          data[o + 3] = ba;
        }
      }
      background = { px, py, pw, image };
    }
    ctx.putImageData(background.image, background.px, background.py);
  }

  function drawAxes(ctx, box, project) {
    ctx.lineWidth = 1;
    ctx.strokeStyle = 'rgba(128, 128, 128, 0.8)';
    ctx.beginPath();
    ctx.moveTo(box.x, box.y);
    ctx.lineTo(box.x + box.side, box.y);
    ctx.lineTo(box.x + box.side, box.y + box.side);
    ctx.lineTo(box.x, box.y + box.side);
    ctx.closePath();
    ctx.stroke();

    if (typeof ctx.fillText !== 'function') return;
    ctx.font = '11px system-ui, -apple-system, Segoe UI, sans-serif';
    ctx.fillStyle = 'rgba(128, 128, 128, 1)';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (const [value, text] of [[-PI, '-\u03c0'], [0, '0'], [PI, '\u03c0']]) {
      const [tx] = project(value, 0);
      ctx.fillText(text, tx, box.y + box.side + 3);
    }
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (const [value, text] of [[-PI, '-\u03c0'], [0, '0'], [PI, '\u03c0']]) {
      const [, ty] = project(0, value);
      ctx.fillText(text, box.x - 4, ty);
    }
    ctx.textAlign = 'center';
    ctx.textBaseline = 'alphabetic';
    ctx.fillText(labels[0], box.x + box.side / 2, box.y + box.side + 25);
    ctx.save();
    ctx.translate(box.x - 24, box.y + box.side / 2);
    ctx.rotate(-PI / 2);
    ctx.fillText(labels[1], 0, 0);
    ctx.restore();
  }

  function vertexAt(i) {
    const vertices = frame.vertices;
    if (i < 0 || i >= vertices.length) return null;
    const row = vertices[i];
    if (packedRows) {
      return { level: row[0], component: row[1], q0: row[2], q1: row[3] };
    }
    const q = row.q || [];
    return { level: row.level || 0, component: row.component || 0, q0: q[0], q1: q[1] };
  }

  function strokeMotion(ctx, project, a, b) {
    for (const [p0, p1] of wrapSegments(a, b)) {
      const [x0, y0] = project(p0[0], p0[1]);
      const [x1, y1] = project(p1[0], p1[1]);
      ctx.beginPath();
      ctx.moveTo(x0, y0);
      ctx.lineTo(x1, y1);
      ctx.stroke();
    }
  }

  function drawFrame(ctx, project) {
    const vertices = frame.vertices;
    const edges = frame.edges || [];
    const vertexCount = frame.vertexCount === undefined ? vertices.length : frame.vertexCount;
    const edgeCount = frame.edgeCount === undefined ? edges.length : frame.edgeCount;
    const drawnVertices = Math.min(vertexCount, vertices.length);
    const drawnEdges = Math.min(edgeCount, edges.length);

    ctx.lineWidth = 0.8;
    for (let i = 0; i < drawnEdges; i++) {
      const va = vertexAt(edges[i][0]);
      const vb = vertexAt(edges[i][1]);
      if (!va || !vb || va.level !== level || vb.level !== level) continue;
      ctx.strokeStyle = hexToRgba(TREE_COLORS[TREE_BY_COMPONENT[va.component]] || TREE_COLORS.untagged, 0.3);
      strokeMotion(ctx, project, [va.q0, va.q1], [vb.q0, vb.q1]);
    }

    /* A vertex of a higher level projects onto this one — the interleaved layout makes its q[0]
     * and q[1] the very same two joints — and that projection is the point of the panel: it shows
     * where the search above went in the coordinates this level owns. Faint and small so the
     * level's own graph still reads on top of it. A vertex of a *lower* level cannot exist; the
     * 2-D level is the bottom of the hierarchy for every scene that has one. */
    for (let i = 0; i < drawnVertices; i++) {
      const v = vertexAt(i);
      if (!v || v.level < level) continue;
      const own = v.level === level;
      ctx.fillStyle = hexToRgba(
        TREE_COLORS[TREE_BY_COMPONENT[v.component]] || TREE_COLORS.untagged,
        own ? 0.9 : 0.25,
      );
      const [px, py] = project(v.q0, v.q1);
      ctx.beginPath();
      ctx.arc(px, py, own ? 2.2 : 1.1, 0, TWO_PI);
      ctx.fill();
    }
  }

  function drawPath(ctx, project) {
    ctx.lineWidth = 2;
    ctx.strokeStyle = hexToRgba(PATH_COLOR, 0.95);
    for (let i = 1; i < path.length; i++) {
      strokeMotion(ctx, project, [path[i - 1][0], path[i - 1][1]], [path[i][0], path[i][1]]);
    }

    const at = path[Math.min(path.length - 1, Math.max(0, Math.round(phase * (path.length - 1))))];
    const [px, py] = project(at[0], at[1]);
    ctx.fillStyle = hexToRgba(PATH_COLOR, 1);
    ctx.beginPath();
    ctx.arc(px, py, 3.5, 0, TWO_PI);
    ctx.fill();
  }

  /* Always last and always opaque: the two configurations the run is defined by are the only
   * fixed points a reader can orient from. */
  function drawEndpoints(ctx, project) {
    ctx.lineWidth = 2;
    const [sx, sy] = project(endpoints.start[0], endpoints.start[1]);
    ctx.strokeStyle = hexToRgba(TREE_COLORS.start, 1);
    ctx.beginPath();
    ctx.arc(sx, sy, 5, 0, TWO_PI);
    ctx.stroke();

    const [gx, gy] = project(endpoints.goal[0], endpoints.goal[1]);
    ctx.strokeStyle = hexToRgba(TREE_COLORS.goal, 1);
    ctx.beginPath();
    ctx.moveTo(gx - 5, gy - 5);
    ctx.lineTo(gx + 5, gy + 5);
    ctx.moveTo(gx + 5, gy - 5);
    ctx.lineTo(gx - 5, gy + 5);
    ctx.stroke();
  }

  /* No fake slice: a scene with three or more robots has no level of dimension 2, and saying which
   * dimension it does start at is more use than a plausible-looking square of the wrong space. */
  function drawUnavailable(ctx, w, h) {
    if (typeof ctx.fillText !== 'function') return;
    const name = sceneData.name || (sceneData.metadata && sceneData.metadata.name);
    const dim = sceneData.robots.length;
    ctx.font = '12px system-ui, -apple-system, Segoe UI, sans-serif';
    ctx.fillStyle = 'rgba(128, 128, 128, 1)';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const prefix = name ? `${name}: ` : '';
    ctx.fillText(
      `${prefix}the lowest bundle level is ${dim}-D \u2014 no exact 2-D view`,
      w / 2,
      h / 2,
    );
  }

  function redraw() {
    const { w, h, ratio } = sizeCanvas(canvas);
    const ctx = canvas.getContext('2d');
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, w, h);

    if (level < 0) {
      drawUnavailable(ctx, w, h);
      return;
    }

    const box = layout(w, h);
    const project = projector(box);
    drawBackground(ctx, box, ratio);
    drawAxes(ctx, box, project);
    if (frame) drawFrame(ctx, project);
    if (path && path.length > 0) drawPath(ctx, project);
    drawEndpoints(ctx, project);
  }

  applyScene(scene);

  return {
    setScene(next) {
      applyScene(next);
      redraw();
    },

    /* The frame is read straight off, row by row, with no per-vertex derivation to memoise: a
     * vertex's two coordinates *are* q[0] and q[1], so unlike the workspace view there is no
     * forward kinematics to cache. The explorer may therefore push the same `vertices` array with
     * only vertexCount/edgeCount moving, 9 000 rows at a time, and pay nothing but the draw. */
    pushFrame(next) {
      if (!next || !next.vertices) return;
      packedRows = next.vertices.length > 0 && Array.isArray(next.vertices[0]);
      frame = next;
      redraw();
    },

    /* A densely interpolated array of full-dimension configurations — the same array the workspace
     * animation plays — of which this panel draws coordinates 0 and 1. An empty array or null
     * clears it. */
    showPath(dense) {
      path = dense && dense.length > 0 ? dense : null;
      phase = 0;
    },

    setPhase(t) {
      phase = Math.min(1, Math.max(0, t));
    },

    redraw,

    /* A new run on the same scene: drop the old tree, path and phase, keep the free-space map and
     * its rasterisation, which depend on the scene alone. Mirrors createTracePlayer's reset(). */
    reset() {
      frame = null;
      path = null;
      phase = 0;
      redraw();
    },

    get level() {
      return level;
    },

    get available() {
      return level >= 0;
    },

    /* Share of the level's configuration space the arms cannot stand in, measured off the same
     * cached map the background is drawn from. The pages put it in their caption: level 0 of a
     * two-robot scene with short links is 99.9% free, and an empty-looking square is a fact about
     * the problem — the first bundle level is nearly unobstructed, which is why it is cheap to
     * solve and a weak guide for the level above. */
    get blockedFraction() {
      if (level < 0 || !sceneData) return 0;
      const { free } = cachedFreeSpace(sceneData, level, MAP_SIZE);
      let blocked = 0;
      for (let i = 0; i < free.length; i++) if (!free[i]) blocked++;
      return blocked / free.length;
    },

    destroy() {
      sceneData = null;
      frame = null;
      path = null;
      endpoints = null;
      background = null;
      labels = [];
    },
  };
}

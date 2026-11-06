/* The one JS implementation of MLMP's 2D drawing: forward kinematics, scene extents and the
 * canvas layers. Both surfaces draw through it — docs/ replays committed mlmp.traceviz/1
 * fixtures, web/ streams live mlmp.trace/1 frames out of the wasm planner — so they cannot
 * disagree.
 *
 * Forward kinematics follow KinematicChainValidityChecker (mlmp/src/state/kinematic_chain.h:199-222):
 * cumulative angles, index q[link*robots + robot], one constant jointLength per robot. A vertex at
 * bundle level L owns only its first L+1 links per robot; the tail above L is identity-fibre
 * padding, not planner output, and is never drawn. Solution paths (mlmp.run/1 `path`) are the
 * exception: those are real full-dimension configurations, so showPath walks every link.
 *
 * Four layers are drawn, in a fixed order that is itself half the fix for the old picture:
 * obstacles, then the planner's graph, then the start and goal arms, then the solution. The arms
 * used to be drawn first and disappeared under ~37 000 projected tip dots, so the one thing a
 * reader needs to see — the robot — was never visible.
 *
 * View modes. `motion` plays the solution as real arms moving and draws no graph at all; it is
 * the default because it is the only view that shows the geometry the planner was solving for.
 * `graph` draws the search tree with its edges and `vertices` drops the edges; both are meant to
 * be read one bundle level at a time (setLevels) and both thin themselves out past MAX_DOTS /
 * MAX_EDGES instead of saturating into a blob.
 *
 * Solution geometry is interpolated in joint space (densifyPath), never chorded. The returned
 * path is the simplified one — 4 states on scene1, 18 on scene3 — and the straight workspace
 * lines the old drawPath joined them with were in the wrong place. Measured against the
 * wrap-aware interpolation the space actually uses: on scene3 robot 1's tip travels 16.83 along
 * the real curve against 8.92 along the chords (1.89x) and leaves the drawn line by up to 0.776
 * on a scene 2.01 wide — 38% of the frame. Even on scene1, where the lengths nearly agree
 * (0.911 against 0.845), the line is off by 0.074 on a 1.09-wide scene. It was the shape that
 * was false, and on the longer paths the length too.
 *
 * No DOM lookup beyond the canvas handed in, no event listeners, no module state: the page owns
 * resize and calls redraw(). */

export const TREE_COLORS = {
  start: '#1f77b4',
  goal: '#d62728',
  section: '#9467bd',
  untagged: '#7f7f7f',
};
/* Not a tree code: the returned mlmp.run/1 `path`, drawn as whole arms by drawPath. It is kept
 * out of TREE_COLORS because that map must stay equal to traceGif.py's palette, which colours
 * component tags only. */
export const PATH_COLOR = '#2ca02c';

/* Poses on the drawn solution that the collision checker rejects. Drawing them green would hide
 * a planner defect behind a smooth animation, so a page hands showPath a validity predicate and
 * the poses it rejects are drawn in this colour. That is how the planar collision-resolution bug
 * was found: 22 of 73 interpolated poses on scene1/BiQRRT, 0.049 units inside an obstacle, on a
 * path segment the motion validator had sampled exactly once. Since
 * KinematicChainSpace::validSegmentCount was fixed the deepest penetration is 0.0004 units,
 * 0.2 px at a 600 px canvas — so red should now appear only as isolated grazes, and a contiguous
 * red stretch means the resolution regressed. */
export const COLLISION_COLOR = '#e8000d';
export const PADDING = 0.05;

/* The JS mirror of treeLabel (mlmp/src/models/run_result.h:91-98) — the component tag a
 * BundleSpaceGraph writes per vertex. A packed fixture carries an identical `tree_codes` map by
 * construction; it is ignored in favour of this one so there is a single definition. Never infer
 * a tree from the planner name. */
export const TREE_CODES = { 0: 'untagged', 1: 'start', 2: 'goal', 3: 'section' };

/* setViewMode's vocabulary. `motion` is the default: the solution playing as arms. */
export const VIEW_MODES = ['motion', 'graph', 'vertices'];

/* Drawing caps for the graph views. A frame past these is strided, not truncated: scene2/QRRT
 * reaches 15 000 vertices, and two tip dots per vertex in one 2D projection is the blob this
 * redesign exists to kill. Striding keeps the shape of the tree while leaving individual dots
 * legible. */
export const MAX_DOTS = 4000;
export const MAX_EDGES = 5000;

/* How finely one solution segment is sampled for drawing. 24 is where the scene1 tip curve stops
 * visibly faceting at full canvas width. */
export const PATH_SUBSTEPS = 24;

export function hexToRgba(hex, alpha) {
  const r = parseInt(hex.slice(1, 3), 16);
  const g = parseInt(hex.slice(3, 5), 16);
  const b = parseInt(hex.slice(5, 7), 16);
  return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

/* Opacity encodes bundle level: the top level is opaque, level 0 is faint but legible. Mirrored
 * in mlmp/tools/traceGif.py:126. */
export function levelAlpha(level, links) {
  return 0.35 + (0.65 * level) / Math.max(1, links - 1);
}

/* Tip of each robot's chain truncated to its first `level + 1` links. `q` may be a full-dimension
 * configuration or a packed row already truncated to the owned prefix — the length guard covers
 * both. */
export function chainTips(q, level, robots, sceneRobots) {
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

/* Scene JSON angles are integer degrees (mlmp/src/models/scene.h:116,119). */
export function chainJoints(robot, anglesDeg) {
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

/* KinematicChainSpace::interpolate (mlmp/src/state/kinematic_chain.h:114-139), coordinate by
 * coordinate: inside half a turn move straight, otherwise go the short way round and re-wrap
 * into [-PI, PI]. The planner's own motion validator checked each solution segment under this
 * rule, so every configuration densifyPath produces between two consecutive path states is a
 * configuration the checker accepted — the animation is not an artist's guess at the motion, it
 * is the motion. */
export function interpolateConfig(from, to, t) {
  const out = new Array(from.length);
  for (let i = 0; i < from.length; i++) {
    let diff = to[i] - from[i];
    if (Math.abs(diff) <= Math.PI) {
      out[i] = from[i] + diff * t;
    } else {
      diff = diff > 0 ? 2 * Math.PI - diff : -2 * Math.PI - diff;
      let v = from[i] - diff * t;
      if (v > Math.PI) v -= 2 * Math.PI;
      else if (v < -Math.PI) v += 2 * Math.PI;
      out[i] = v;
    }
  }
  return out;
}

/* A simplified path turned into the sequence of configurations the arms actually pass through.
 * The endpoints are kept exactly; only the interior is synthesised. */
export function densifyPath(path, substeps = PATH_SUBSTEPS) {
  if (!Array.isArray(path) || path.length === 0) return [];
  if (path.length === 1) return [path[0].slice()];
  const steps = Math.max(1, substeps);
  const dense = [path[0].slice()];
  for (let i = 0; i < path.length - 1; i++) {
    for (let k = 1; k <= steps; k++) {
      dense.push(k === steps ? path[i + 1].slice() : interpolateConfig(path[i], path[i + 1], k / steps));
    }
  }
  return dense;
}

/* Obstacle points plus every robot's full reach, so the view never moves when levels are toggled
 * or frames advance. */
export function sceneBounds(scene, links) {
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

function canvasSize(canvas) {
  const rect = canvas.getBoundingClientRect();
  return {
    w: Math.max(1, Math.round(rect.width)),
    h: Math.max(1, Math.round(rect.height || rect.width)),
  };
}

function sizeCanvas(canvas) {
  const ratio = (typeof window !== 'undefined' && window.devicePixelRatio) || 1;
  const { w, h } = canvasSize(canvas);
  canvas.width = Math.round(w * ratio);
  canvas.height = Math.round(h * ratio);
  return { w, h, ratio };
}

/* `scene` is a parsed scene JSON; `scene.metadata.n` is the link count. It is read from the scene
 * and never from a trace header, which a live stream may not have delivered yet. */
export function createTracePlayer({ canvas, scene } = {}) {
  if (!canvas || !scene) {
    throw new Error('createTracePlayer: canvas and scene are required');
  }

  let sceneData = null;
  let links = 0;
  let robots = 0;
  let bounds = null;
  let levels = null; // null => every level visible
  let frame = null;  // one retained reference, not an accumulation
  let path = null;
  /* The solution as it is drawn: every configuration the arms pass through (densifyPath), plus
   * one tip polyline per robot through it. Both are rebuilt only in showPath, because a motion
   * frame redraws them 60 times a second. */
  let dense = [];
  let tipTraces = [];
  /* Parallel to `dense`: true where the page's validity predicate rejected that pose. Empty when
   * no predicate was supplied. */
  let colliding = [];
  /* Which configuration of `dense` the solid arms stand at, 0..1. The page owns the clock. */
  let phase = 1;
  let viewMode = 'motion';
  /* 'planner' (the planner tagged its own trees) or 'derived' (mlmp.trace/1 labels computed from
   * graph connectivity). A frame that says nothing is a planner-tagged one. */
  let treeSource = 'planner';
  /* Per-redraw tally of the vertices actually drawn, filled by the vertex walk in drawFrame so
   * treeCounts() adds no second pass; null means "not walked since the last redraw". */
  let counts = null;

  /* FK memo. Load-bearing, not an optimisation: the scrubber pushes the same `vertices` array
   * object every frame (scene1/BiQRRT is 9 722 vertices), so it must hit fully or playback gets
   * slower than the pre-retrofit page. A live Worker stream hands over a fresh array each frame
   * and correctly recomputes, because its vertices changed. */
  let derived = [];
  let cacheSource = null;
  let packedRows = false;

  function applyScene(next) {
    sceneData = next;
    links = next.metadata.n;
    robots = next.robots.length;
    bounds = sceneBounds(next, links);
    derived = [];
    cacheSource = null;
    frame = null;
    path = null;
    dense = [];
    tipTraces = [];
    colliding = [];
    phase = 1;
    counts = null;
    treeSource = 'planner';
  }

  function deriveVertex(i) {
    const vertices = frame.vertices;
    if (i < 0 || i >= vertices.length) return null;
    let record = derived[i];
    if (record !== undefined) return record;

    const row = vertices[i];
    let level;
    let component;
    let tree;
    let q;
    if (packedRows) {
      level = row[0];
      component = row[1];
      q = row.slice(2);
    } else {
      level = row.level || 0;
      component = row.component || 0;
      tree = typeof row.tree === 'string' && row.tree.length > 0 ? row.tree : undefined;
      q = row.q || [];
    }
    if (tree === undefined) tree = TREE_CODES[component] || 'untagged';

    record = {
      level,
      tree,
      alpha: levelAlpha(level, links),
      tips: chainTips(q, level, robots, sceneData.robots),
    };
    derived[i] = record;
    return record;
  }

  function visible(level) {
    return levels === null || levels.has(level);
  }

  /* The fallback for the one case drawFrame cannot cover: view mode `path` draws no vertices, so
   * a page asking for the tally still has to walk them. It reads the level and the label straight
   * off the row and never calls deriveVertex, because computing forward kinematics for 15 000
   * vertices nobody draws is exactly the cost `path` mode exists to avoid. */
  function tallyVertices() {
    const tally = { start: 0, goal: 0, section: 0, untagged: 0 };
    if (!frame) return tally;
    const vertices = frame.vertices;
    const vertexCount = frame.vertexCount === undefined ? vertices.length : frame.vertexCount;
    for (let i = 0; i < vertexCount && i < vertices.length; i++) {
      const row = vertices[i];
      const level = packedRows ? row[0] : row.level || 0;
      if (!visible(level)) continue;
      let tree = packedRows ? undefined : row.tree;
      if (typeof tree !== 'string' || tree.length === 0) {
        tree = TREE_CODES[packedRows ? row[1] : row.component || 0] || 'untagged';
      }
      tally[tally[tree] === undefined ? 'untagged' : tree]++;
    }
    return tally;
  }

  function drawObstacles(ctx, project) {
    ctx.lineWidth = 1;
    for (const obstacle of sceneData.obstacles) {
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
  }

  /* The arms the run starts and ends at. Drawn after the graph, never before it: at 0.25 black
   * under nine thousand dots they were invisible, which is how a picture of two robot arms ended
   * up showing no robot. When exactly one bundle level is on, they are truncated to that level's
   * links — level 0 of scene1 really is two one-link stubs, and seeing that is the whole point of
   * looking at a level on its own. */
  function drawPoses(ctx, project) {
    const only = soleLevel();
    const shown = only === null ? links : only + 1;
    for (const robot of sceneData.robots) {
      for (const [angles, dash] of [[robot.startAngles, []], [robot.goalAngles, [4, 3]]]) {
        const pts = chainJoints(robot, angles.slice(0, shown));
        ctx.setLineDash(dash);
        ctx.lineWidth = 2;
        ctx.strokeStyle = 'rgba(0, 0, 0, 0.75)';
        strokePolyline(ctx, project, pts);
        ctx.setLineDash([]);
        for (const [x, y] of pts) {
          const [px, py] = project(x, y);
          ctx.beginPath();
          ctx.arc(px, py, 2.2, 0, 2 * Math.PI);
          ctx.fillStyle = 'rgba(0, 0, 0, 0.75)';
          ctx.fill();
        }
      }
    }
  }

  /* The single visible level, or null when zero or several are on. */
  function soleLevel() {
    if (levels === null || levels.size !== 1) return null;
    for (const level of levels) return level;
    return null;
  }

  function drawFrame(ctx, project) {
    const vertices = frame.vertices;
    const edges = frame.edges || [];
    const vertexCount = frame.vertexCount === undefined ? vertices.length : frame.vertexCount;
    const edgeCount = frame.edgeCount === undefined ? edges.length : frame.edgeCount;
    const drawnEdges = Math.min(edgeCount, edges.length);
    const drawnVertices = Math.min(vertexCount, vertices.length);

    /* Crowded frames are drawn thinner and fainter: at the ~15 000 vertices scene1/QRRT reaches,
     * the fixed 0.25 alpha saturates whole regions to flat grey and the graph stops being
     * readable at all. */
    const crowded = drawnEdges > 2000;
    const edgeAlpha = crowded ? 0.1 : 0.25;

    /* Pass one: tally every visible vertex, which is what the legend reports, and learn how many
     * there are so pass two can stride down to MAX_DOTS. `section` is tallied but never strided —
     * findSection() output is sparse and is the thing worth seeing. */
    const tally = { start: 0, goal: 0, section: 0, untagged: 0 };
    let visibleVertices = 0;
    for (let i = 0; i < drawnVertices; i++) {
      const v = deriveVertex(i);
      if (!visible(v.level)) continue;
      visibleVertices++;
      tally[tally[v.tree] === undefined ? 'untagged' : v.tree]++;
    }
    counts = tally;

    const dots = Math.max(1, visibleVertices * Math.max(1, robots));
    const vertexStride = Math.max(1, Math.ceil(dots / MAX_DOTS));
    const edgeStride = Math.max(1, Math.ceil(drawnEdges / MAX_EDGES));
    const dotRadius = vertexStride > 1 ? 1.5 : visibleVertices > 2000 ? 1.4 : 1.8;

    if (viewMode === 'graph') {
      ctx.lineWidth = crowded ? 0.6 : 0.8;
      for (let i = 0; i < drawnEdges; i += edgeStride) {
        const va = deriveVertex(edges[i][0]);
        const vb = deriveVertex(edges[i][1]);
        if (!va || !vb) continue;
        if (!visible(va.level) || !visible(vb.level)) continue;
        ctx.strokeStyle = hexToRgba(TREE_COLORS[va.tree] || TREE_COLORS.untagged, edgeAlpha);
        for (let r = 0; r < va.tips.length && r < vb.tips.length; r++) {
          const [x0, y0] = project(va.tips[r][0], va.tips[r][1]);
          const [x1, y1] = project(vb.tips[r][0], vb.tips[r][1]);
          ctx.beginPath();
          ctx.moveTo(x0, y0);
          ctx.lineTo(x1, y1);
          ctx.stroke();
        }
      }
    }

    /* `section` last so those configurations stay legible on top of both trees. */
    let seen = 0;
    for (const sectionLayer of [false, true]) {
      for (let i = 0; i < drawnVertices; i++) {
        const v = deriveVertex(i);
        if (!visible(v.level)) continue;
        const isSection = v.tree === 'section';
        if (!sectionLayer) seen++;
        if (isSection !== sectionLayer) continue;
        if (!isSection && (seen - 1) % vertexStride !== 0) continue;
        ctx.fillStyle = hexToRgba(TREE_COLORS[v.tree] || TREE_COLORS.untagged, v.alpha);
        const radius = isSection ? 2.4 : dotRadius;
        for (const [x, y] of v.tips) {
          const [px, py] = project(x, y);
          ctx.beginPath();
          ctx.arc(px, py, radius, 0, 2 * Math.PI);
          ctx.fill();
        }
      }
      if (!sectionLayer) seen = 0;
    }
  }

  /* One path state's arm for one robot. A solution is a real full-dimension configuration, so
   * unlike a graph vertex it is never truncated. */
  function pathArm(q, r) {
    const robot = sceneData.robots[r];
    let x = robot.pinnedPosition.x;
    let y = robot.pinnedPosition.y;
    let theta = 0;
    const pts = [[x, y]];
    for (let j = 0; j < links; j++) {
      const idx = j * robots + r;
      if (idx >= q.length) break;
      theta += q[idx];
      x += robot.jointLength * Math.cos(theta);
      y += robot.jointLength * Math.sin(theta);
      pts.push([x, y]);
    }
    return pts;
  }

  function strokePolyline(ctx, project, pts) {
    ctx.beginPath();
    pts.forEach(([x, y], i) => {
      const [px, py] = project(x, y);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }
  /* The tip curve each robot's chain traces along the interpolated solution. Thin, because in the
   * graph views it is context, and in motion it is the trail the arms leave behind. */
  function drawTipTraces(ctx, project, alpha, upto) {
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = hexToRgba(PATH_COLOR, alpha);
    for (const trace of tipTraces) {
      const end = upto === undefined ? trace.length : Math.max(2, Math.min(trace.length, upto));
      if (end < 2) continue;
      strokePolyline(ctx, project, trace.slice(0, end));
    }
  }
  function drawArms(ctx, project, q, { alpha, width, joints, color = PATH_COLOR }) {
    ctx.lineWidth = width;
    ctx.strokeStyle = hexToRgba(color, alpha);
    ctx.fillStyle = hexToRgba(color, alpha);
    for (let r = 0; r < robots; r++) {
      const pts = pathArm(q, r);
      strokePolyline(ctx, project, pts);
      if (!joints) continue;
      for (const [x, y] of pts) {
        const [px, py] = project(x, y);
        ctx.beginPath();
        ctx.arc(px, py, 2.6, 0, 2 * Math.PI);
        ctx.fill();
      }
    }
  }

  /* The solution playing: every arm the robots pass through, at the configuration `phase` picks
   * out, with a short fading wake behind it and the tip curve drawn up to that point. The wake is
   * what makes the direction of travel readable in a still frame — a screenshot of this view has
   * to work as well as the animation, because that screenshot is what ends up in a README. */
  function drawMotion(ctx, project) {
    const last = dense.length - 1;
    const at = Math.max(0, Math.min(last, Math.round(phase * last)));
    drawTipTraces(ctx, project, 0.35, at + 1);

    const wake = Math.max(1, Math.round(dense.length / 24));
    for (let back = 4; back >= 1; back--) {
      const i = at - back * wake;
      if (i < 0) continue;
      drawArms(ctx, project, dense[i], {
        alpha: 0.1 + 0.05 * (4 - back),
        width: 1.5,
        joints: false,
        color: colliding[i] ? COLLISION_COLOR : PATH_COLOR,
      });
    }
    drawArms(ctx, project, dense[at], {
      alpha: 1,
      width: 3,
      joints: true,
      color: colliding[at] ? COLLISION_COLOR : PATH_COLOR,
    });
  }

  /* Layer order is the design: obstacles, graph, arms, solution. Anything the reader needs to
   * recognise the scene by goes on top of the point cloud, never under it. */
  function redraw() {
    const { w, h, ratio } = sizeCanvas(canvas);
    const ctx = canvas.getContext('2d');
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const project = projector(bounds, w, h);
    counts = null;
    drawObstacles(ctx, project);
    if (frame && viewMode !== 'motion') drawFrame(ctx, project);
    drawPoses(ctx, project);
    if (dense.length > 1) {
      if (viewMode === 'motion') drawMotion(ctx, project);
      else drawTipTraces(ctx, project, 0.9);
    }
  }

  applyScene(scene);

  return {
    pushFrame(next) {
      if (!next || !next.vertices) return;
      if (next.vertices !== cacheSource) {
        derived = [];
        cacheSource = next.vertices;
        packedRows = next.vertices.length > 0 && Array.isArray(next.vertices[0]);
      }
      frame = next;
      /* mlmp.trace/1 spells it tree_source; the docs player passes the fixture's top-level value
       * through as treeSource. Absent means the frame predates the field, i.e. planner-tagged. */
      treeSource = next.treeSource || next.tree_source || 'planner';
      redraw();
    },

    setScene(next) {
      applyScene(next);
      redraw();
    },

    /* The Set is held by reference: the page mutates its own level set and re-renders. */
    setLevels(next) {
      levels = next || null;
    },
    /* 'motion' | 'graph' | 'vertices'. The page redraws; this only records the choice, like
     * setLevels. */
    setViewMode(mode) {
      if (!VIEW_MODES.includes(mode)) {
        throw new Error(`setViewMode: unknown mode ${mode}`);
      }
      viewMode = mode;
    },

    /* Densifying, running forward kinematics over the whole solution and checking each pose
     * happens here, once, and never in redraw: motion playback redraws at animation rate.
     * `isValid` is optional and comes from the page (cspace.js's isValidConfig bound to this
     * scene's top level); without it nothing is flagged and every pose is drawn as valid. */
    showPath(next, isValid) {
      path = next && next.length > 0 ? next : null;
      dense = path ? densifyPath(path) : [];
      tipTraces = [];
      for (let r = 0; r < robots; r++) {
        tipTraces.push(dense.map((q) => {
          const pts = pathArm(q, r);
          return pts[pts.length - 1];
        }));
      }
      colliding = typeof isValid === 'function' ? dense.map((q) => !isValid(q)) : [];
      phase = 1;
    },

    /* 0..1 along the interpolated solution. Clamped rather than wrapped: a page that loops owns
     * the wrap, and a scrubber that overshoots should sit at the goal, not jump to the start. */
    setPhase(t) {
      phase = Number.isFinite(t) ? Math.max(0, Math.min(1, t)) : 1;
    },

    /* How many of the drawn poses the checker rejects and the first contiguous stretch of them,
     * so a page can say "22 of 73 poses collide (26-47)" rather than leave a red arm
     * unexplained. */
    motionCollisions() {
      let count = 0;
      let from = -1;
      let to = -1;
      for (let i = 0; i < colliding.length; i++) {
        if (!colliding[i]) continue;
        count++;
        if (from < 0) from = i;
        if (to < 0 || to === i - 1) to = i;
      }
      return { count, total: dense.length, from, to };
    },

    /* Number of interpolated configurations; 0 when this run has no solution. The page sizes its
     * scrubber from it and shows its empty state when it is 0. */
    get motionStates() {
      return dense.length;
    },

    /* The interpolated solution itself, for a second view that has to stay in step with this one
     * (the C-space panel draws the same configurations in its own coordinates). */
    get motionPath() {
      return dense;
    },

    redraw,

    reset() {
      frame = null;
      derived = [];
      cacheSource = null;
      path = null;
      dense = [];
      tipTraces = [];
      colliding = [];
      phase = 1;
      counts = null;
      treeSource = 'planner';
      redraw();
    },

    toScene(px, py) {
      const { w, h } = canvasSize(canvas);
      const scale = Math.min(w / (bounds.maxX - bounds.minX), h / (bounds.maxY - bounds.minY));
      const offsetX = (w - (bounds.maxX - bounds.minX) * scale) / 2;
      const offsetY = (h - (bounds.maxY - bounds.minY) * scale) / 2;
      return [bounds.minX + (px - offsetX) / scale, bounds.minY + (h - offsetY - py) / scale];
    },

    toCanvas(x, y) {
      const { w, h } = canvasSize(canvas);
      return projector(bounds, w, h)(x, y);
    },

    get scene() {
      return sceneData;
    },

    get links() {
      return links;
    },
    /* Tree membership of the visible vertices of the current frame — the level filter and the
     * frame's vertexCount prefix both apply, striding does not, because the legend reports what
     * the run contains and not what fitted on the canvas. Pages dim entries that read 0. */
    treeCounts() {
      if (counts === null) counts = tallyVertices();
      return { ...counts };
    },

    get treeSource() {
      return treeSource;
    },
    destroy() {
      sceneData = null;
      bounds = null;
      frame = null;
      path = null;
      dense = [];
      tipTraces = [];
      colliding = [];
      derived = [];
      cacheSource = null;
      counts = null;
    },
  };
}

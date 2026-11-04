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
 * Three view modes exist because the grey edge web is what collapses a 10 000-vertex graph into
 * a blob: `graph` draws edges, vertices and the path, `vertices` drops the edges, `path` drops
 * the graph entirely. Edge and dot styling also thins out once a frame is dense.
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
export const PADDING = 0.05;

/* The JS mirror of treeLabel (mlmp/src/models/run_result.h:91-98) — the component tag a
 * BundleSpaceGraph writes per vertex. A packed fixture carries an identical `tree_codes` map by
 * construction; it is ignored in favour of this one so there is a single definition. Never infer
 * a tree from the planner name. */
export const TREE_CODES = { 0: 'untagged', 1: 'start', 2: 'goal', 3: 'section' };

/* setViewMode's vocabulary; `graph` is the default and the historical picture. */
export const VIEW_MODES = ['graph', 'vertices', 'path'];

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
  let viewMode = 'graph';
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

  function drawStatic(ctx, project) {
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
    for (const robot of sceneData.robots) {
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

  function drawFrame(ctx, project) {
    const vertices = frame.vertices;
    const edges = frame.edges || [];
    const vertexCount = frame.vertexCount === undefined ? vertices.length : frame.vertexCount;
    const edgeCount = frame.edgeCount === undefined ? edges.length : frame.edgeCount;
    const drawnEdges = Math.min(edgeCount, edges.length);
    const drawnVertices = Math.min(vertexCount, vertices.length);

    /* Dense frames are drawn thinner and fainter: at the ~15 000 vertices scene1/QRRT reaches,
     * the fixed 0.25 alpha saturates whole regions to flat grey and the graph stops being
     * readable at all. */
    const dense = drawnEdges > 2000;
    const edgeAlpha = dense ? 0.1 : 0.25;
    const dotRadius = drawnVertices > 4000 ? 1.1 : 1.5;

    if (viewMode === 'graph') {
      ctx.lineWidth = dense ? 0.6 : 0.8;
      for (let i = 0; i < drawnEdges; i++) {
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

    /* `section` last so those configurations stay legible on top of both trees. The first of the
     * two passes also tallies, which is what keeps treeCounts() free. */
    const tally = { start: 0, goal: 0, section: 0, untagged: 0 };
    for (const sectionLayer of [false, true]) {
      for (let i = 0; i < drawnVertices; i++) {
        const v = deriveVertex(i);
        if (!visible(v.level)) continue;
        if (!sectionLayer) tally[tally[v.tree] === undefined ? 'untagged' : v.tree]++;
        if ((v.tree === 'section') !== sectionLayer) continue;
        ctx.fillStyle = hexToRgba(TREE_COLORS[v.tree] || TREE_COLORS.untagged, v.alpha);
        const radius = v.tree === 'section' ? 2.4 : dotRadius;
        for (const [x, y] of v.tips) {
          const [px, py] = project(x, y);
          ctx.beginPath();
          ctx.arc(px, py, radius, 0, 2 * Math.PI);
          ctx.fill();
        }
      }
    }
    counts = tally;
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

  /* The returned solution, which is already the simplified path (solver.h simplifies in place
   * before exporting) — there is no raw path to overlay. Drawing every state as a solid arm
   * turns a dozen states into a scribble of crossing arms, so what carries the motion is the
   * tip trajectory: one polyline per robot through its chain tip across the states, the first
   * and last arms solid, the intermediate arms ghosted behind them. */
  function drawPath(ctx, project) {
    const last = path.length - 1;
    for (let r = 0; r < robots; r++) {
      const arms = path.map((q) => pathArm(q, r));

      ctx.lineWidth = 1;
      ctx.strokeStyle = hexToRgba(PATH_COLOR, 0.15);
      for (let i = 1; i < last; i++) strokePolyline(ctx, project, arms[i]);

      ctx.lineWidth = 2;
      ctx.strokeStyle = hexToRgba(PATH_COLOR, 1);
      strokePolyline(ctx, project, arms.map((pts) => pts[pts.length - 1]));

      ctx.fillStyle = hexToRgba(PATH_COLOR, 1);
      for (const i of last > 0 ? [0, last] : [0]) {
        strokePolyline(ctx, project, arms[i]);
        for (const [x, y] of arms[i]) {
          const [px, py] = project(x, y);
          ctx.beginPath();
          ctx.arc(px, py, 2, 0, 2 * Math.PI);
          ctx.fill();
        }
      }
    }
  }

  function redraw() {
    const { w, h, ratio } = sizeCanvas(canvas);
    const ctx = canvas.getContext('2d');
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const project = projector(bounds, w, h);
    counts = null;
    drawStatic(ctx, project);
    if (frame && viewMode !== 'path') drawFrame(ctx, project);
    if (path && path.length > 0) drawPath(ctx, project);
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

    /* 'graph' | 'vertices' | 'path'. The page redraws; this only records the choice, like
     * setLevels. */
    setViewMode(mode) {
      if (!VIEW_MODES.includes(mode)) {
        throw new Error(`setViewMode: unknown mode ${mode}`);
      }
      viewMode = mode;
    },

    showPath(next) {
      path = next && next.length > 0 ? next : null;
    },

    redraw,

    reset() {
      frame = null;
      derived = [];
      cacheSource = null;
      path = null;
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

    /* Tree membership of the vertices actually drawn for the current frame — the level filter
     * and the frame's vertexCount prefix both apply. Pages dim legend entries that read 0. */
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
      derived = [];
      cacheSource = null;
      counts = null;
    },
  };
}

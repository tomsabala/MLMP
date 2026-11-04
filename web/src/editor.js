/* Obstacle editing on top of the shared renderer. Owns a mutable copy of the loaded scene JSON
 * and dispatches a `change` CustomEvent (detail = the scene) once an edit is committed.
 *
 * Only `obstacles` is ever touched. Robots, metadata and every angle pass through verbatim, which
 * keeps Scene::loadRobots' integer-degree requirement (angle.get<int>(), mlmp/src/models/scene.h)
 * satisfied with no rounding logic anywhere.
 *
 * There is no JS-side collision check: the C++ validity checker is the only authority, so an
 * obstacle dropped on a robot's start pose is reported by the planner as invalid_start. */
import { TREE_COLORS } from './renderer.js';

function round4(v) {
  return Math.round(v * 10000) / 10000;
}

/* Even-odd point-in-polygon; obstacles are arbitrary point lists closed by Scene::loadObstacles. */
function inside(points, x, y) {
  let hit = false;
  for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
    const xi = points[i].x;
    const yi = points[i].y;
    const xj = points[j].x;
    const yj = points[j].y;
    if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) hit = !hit;
  }
  return hit;
}

export function createSceneEditor({ canvas, player, scene }) {
  if (!canvas || !player || !scene) {
    throw new Error('createSceneEditor: canvas, player and scene are required');
  }

  const events = new EventTarget();
  let current = scene;
  let addMode = false;
  let provisional = [];
  let selected = null;
  let dragFrom = null;
  let dragOrigin = null;
  let dragged = false;

  /* `info` is written by convention (Scene never reads it) and kept consistent with the index so
   * an edited scene stays diff-able against the committed ones. */
  function renumber() {
    current.obstacles.forEach((obstacle, i) => {
      obstacle.info = String(i + 1);
    });
  }

  function commit() {
    renumber();
    player.setScene(current);
    drawOverlay();
    events.dispatchEvent(new CustomEvent('change', { detail: current }));
  }

  function pointerScene(event) {
    const rect = canvas.getBoundingClientRect();
    return player.toScene(event.clientX - rect.left, event.clientY - rect.top);
  }

  function onPointerDown(event) {
    canvas.focus();
    const [x, y] = pointerScene(event);
    if (addMode) {
      provisional.push({ x: round4(x), y: round4(y) });
      drawOverlay();
      return;
    }
    selected = null;
    for (let i = current.obstacles.length - 1; i >= 0; i--) {
      if (inside(current.obstacles[i].points, x, y)) {
        selected = i;
        break;
      }
    }
    dragged = false;
    if (selected !== null) {
      dragFrom = [x, y];
      dragOrigin = current.obstacles[selected].points.map((p) => ({ x: p.x, y: p.y }));
      canvas.setPointerCapture(event.pointerId);
    }
    drawOverlay();
  }

  function onPointerMove(event) {
    if (selected === null || dragFrom === null) return;
    const [x, y] = pointerScene(event);
    const dx = x - dragFrom[0];
    const dy = y - dragFrom[1];
    if (dx !== 0 || dy !== 0) dragged = true;
    current.obstacles[selected].points = dragOrigin.map((p) => ({
      x: round4(p.x + dx),
      y: round4(p.y + dy),
    }));
    /* Extents include obstacle points, so the view follows an obstacle dragged outside the
     * robots' reach; setScene is also what clears the stale run's trees. */
    player.setScene(current);
    drawOverlay();
  }

  function onPointerUp(event) {
    if (selected === null || dragFrom === null) return;
    if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
    dragFrom = null;
    dragOrigin = null;
    if (dragged) commit();
    dragged = false;
  }

  function closeProvisional() {
    const points = provisional;
    provisional = [];
    /* Fewer than three vertices is not a polygon; discard it silently rather than feeding the
     * checker a degenerate segment pair. */
    if (points.length < 3) {
      drawOverlay();
      return;
    }
    current.obstacles.push({ points, info: String(current.obstacles.length + 1) });
    selected = current.obstacles.length - 1;
    commit();
  }

  function onDblClick() {
    if (addMode && provisional.length > 0) closeProvisional();
  }

  function onKeyDown(event) {
    if (addMode && event.key === 'Enter' && provisional.length > 0) {
      event.preventDefault();
      closeProvisional();
      return;
    }
    if ((event.key === 'Delete' || event.key === 'Backspace') && selected !== null) {
      event.preventDefault();
      current.obstacles.splice(selected, 1);
      selected = null;
      /* An empty obstacle list is legal: Scene::addBoundarySegments still walls the scene in. */
      commit();
    }
  }

  /* Drawn on top of whatever the player last rendered; cheap no-op when nothing is being edited,
   * because the page calls it after every frame push. */
  function drawOverlay() {
    if (provisional.length === 0 && selected === null) return;
    const ctx = canvas.getContext('2d');
    const ratio = (typeof window !== 'undefined' && window.devicePixelRatio) || 1;
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);

    if (selected !== null && current.obstacles[selected]) {
      ctx.save();
      ctx.lineWidth = 2;
      ctx.strokeStyle = TREE_COLORS.goal;
      ctx.beginPath();
      current.obstacles[selected].points.forEach((p, i) => {
        const [px, py] = player.toCanvas(p.x, p.y);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.closePath();
      ctx.stroke();
      ctx.restore();
    }

    if (provisional.length > 0) {
      ctx.save();
      ctx.setLineDash([5, 4]);
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = TREE_COLORS.start;
      ctx.beginPath();
      provisional.forEach((p, i) => {
        const [px, py] = player.toCanvas(p.x, p.y);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();
      for (const p of provisional) {
        const [px, py] = player.toCanvas(p.x, p.y);
        ctx.beginPath();
        ctx.arc(px, py, 3, 0, 2 * Math.PI);
        ctx.fillStyle = TREE_COLORS.start;
        ctx.fill();
      }
      ctx.restore();
    }
  }

  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointermove', onPointerMove);
  canvas.addEventListener('pointerup', onPointerUp);
  canvas.addEventListener('pointercancel', onPointerUp);
  canvas.addEventListener('dblclick', onDblClick);
  window.addEventListener('keydown', onKeyDown);

  return {
    events,

    setScene(next) {
      current = next;
      selected = null;
      provisional = [];
      dragFrom = null;
      dragOrigin = null;
    },

    setAddMode(on) {
      addMode = on;
      if (!on) provisional = [];
      drawOverlay();
    },

    get scene() {
      return current;
    },

    drawOverlay,

    destroy() {
      canvas.removeEventListener('pointerdown', onPointerDown);
      canvas.removeEventListener('pointermove', onPointerMove);
      canvas.removeEventListener('pointerup', onPointerUp);
      canvas.removeEventListener('pointercancel', onPointerUp);
      canvas.removeEventListener('dblclick', onDblClick);
      window.removeEventListener('keydown', onKeyDown);
    },
  };
}

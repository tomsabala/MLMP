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
  TREE_CODES,
  TREE_COLORS,
  chainTips,
  levelAlpha,
  sceneBounds,
} from '../src/renderer.js';

const scene = JSON.parse(readFileSync(new URL('../../docs/scenes/scene1.json', import.meta.url)));
const links = scene.metadata.n;
const robots = scene.robots.length;
assert.equal(links, 4);
assert.equal(robots, 2);

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
  assert.deepEqual(TREE_CODES, { 0: 'untagged', 1: 'start', 2: 'goal', 3: 'path' });
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

console.log('renderer.test.mjs: 6 groups ok');

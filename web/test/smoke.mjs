/* Headless proof that the wasm planner actually runs, not just links:
 *
 *     docker build -f Dockerfile.wasm --target dist --output type=local,dest=web/dist .
 *     node web/test/smoke.mjs
 *
 * This is the wasm equivalent of CI's per-planner native smoke loop. A build that links but
 * aborts inside PRM's std::thread, or on a /proc/self/stat read, passes the size gate and fails
 * here. The Worker is deliberately not involved: solveScene is called directly.
 *
 * Nothing here is compared against the native build's numbers — differing libm implementations
 * make cross-target floating-point equality an invalid assertion. */
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

import createMlmpModule from '../dist/src/mlmp.mjs';

/* Every planner except PRMStar, which the browser build rejects by design. */
const PLANNERS = [
  'BiQRRT',
  'QRRT',
  'QRRTStar',
  'QMP',
  'QMPStar',
  'RRT',
  'RRTConnect',
  'BITStar',
  'AITStar',
  'EITStar',
  'LBKPIECE',
];

const sceneJson = readFileSync(new URL('../dist/scenes/scene1.json', import.meta.url), 'utf8');

const Module = await createMlmpModule();
Module.FS.writeFile('/scene1.json', sceneJson);

/* `onFrame` left undefined means the module installs no trace sink at all. */
function solve(algo, timeLimitS, seed, onFrame = undefined) {
  const json = Module.solveScene('/scene1.json', algo, timeLimitS, seed, 0.05, 400, onFrame);
  return JSON.parse(json);
}

let failed = 0;
for (const algo of PLANNERS) {
  const result = solve(algo, 2, 42);
  if (result.schema !== 'mlmp.run/1' || result.error) {
    console.error(`FAIL ${algo}: ${JSON.stringify(result.error ?? result.schema)}`);
    failed = 1;
  } else {
    console.log(`ok   ${algo} (${result.status})`);
  }
}
assert.equal(failed, 0, 'at least one planner failed to run in wasm');

{
  const result = solve('PRMStar', 2, 42);
  assert.equal(result.error?.kind, 'usage', JSON.stringify(result.error));
  console.log(`ok   PRMStar rejected: ${result.error.message}`);
}

{
  const result = solve('BiQRRT', 2, 42);
  /* MEMFS path, not a CLI scene name: resolveScenePath is a main.cpp concern. */
  assert.equal(result.scene, '/scene1.json');
  assert.equal(result.robots, 2);
  assert.equal(result.links, 4);
}

/* A traced run spends a large share of its budget on frame capture and JSON.parse, charged to
 * the planner's own thread, so whether it solves within 10 s is host-speed dependent — the same
 * reason ci.yml's native trace step accepts exit 0 or 1. Solve success is proven by the
 * untraced block below; this block proves the trace plumbing. */
let frames = 0;
const traced = solve('BiQRRT', 10, 42, () => {
  frames++;
});
assert.equal(traced.seed, 42, String(traced.seed));
assert.ok(frames > 0, 'no trace frame reached the callback');
assert.equal(traced.trace.frames, frames, `${traced.trace.frames} vs ${frames}`);
if (traced.solved) {
  assert.equal(traced.path.length, traced.path_states);
  assert.equal(traced.path[0].length, traced.robots * traced.links);
}
console.log(
  `ok   BiQRRT traced scene1: ${frames} frames, status ${traced.status}` +
    (traced.solved ? `, len ${traced.path_length_raw.toFixed(3)}` : '')
);

/* Seed reproducibility covers path fields, and only for an untraced run: capturing a frame
 * calls getPlannerData, whose path-restriction code draws from the global RNG, and captures
 * land on wall-clock boundaries — so a traced multilevel run consumes a host-speed-dependent
 * number of draws. Counters are not compared even here, because the planner spends the whole
 * budget and its iteration count tracks host speed; that is the same reasoning the native
 * determinism gate records. */
const a = solve('BiQRRT', 10, 42);
const b = solve('BiQRRT', 10, 42);
assert.equal(a.solved, true, a.status);
assert.equal(a.trace, null, 'an untraced run still reported a trace object');
for (const key of ['path_length_raw', 'path_length_simplified']) {
  assert.equal(b[key], a[key], `non-deterministic ${key}`);
}
console.log(
  `ok   two untraced seed-42 runs agree: raw ${a.path_length_raw.toFixed(6)}, ` +
    `simplified ${a.path_length_simplified.toFixed(6)}`
);

/* The reported seed must be the seed that was applied. ompl::RNG::setSeed records firstSeed_
 * only on the process's first call and this module outlives every solve, so before entry.cpp
 * overrode the field, every run after the first advertised run 1's seed. */
const seeded = solve('BiQRRT', 2, 7);
assert.equal(seeded.seed, 7, String(seeded.seed));
console.log(`ok   a later solve reports its own seed (${seeded.seed})`);

/* Nonsense inputs are reachable from the page's number fields: they must come back as usage
 * errors rather than a microsecond-clock seed or a null time_limit_s in the frozen schema. */
for (const seed of [0, -1, NaN, 1.5, 4294967301]) {
  const result = solve('BiQRRT', 2, seed);
  assert.equal(result.error?.kind, 'usage', `seed ${seed}: ${JSON.stringify(result)}`);
}
{
  const result = solve('BiQRRT', NaN, 42);
  assert.equal(result.error?.kind, 'usage', JSON.stringify(result));
}
console.log('ok   invalid seed and time limit rejected as usage errors');

console.log('smoke.mjs: all checks ok');

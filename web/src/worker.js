/* Module worker around the wasm planner. solveScene blocks for the whole time budget — that is
 * the entire reason it lives here and not on the page's thread.
 *
 * Protocol
 *   main -> worker: {type:"solve", id, sceneJson, algo, timeLimitS, seed, traceIntervalMs,
 *                    traceMaxFrames}
 *   worker -> main: {type:"ready"} | {type:"frame", id, frame} | {type:"result", id, result}
 *
 * The module instance is created once and kept for the worker's lifetime; nothing is terminated
 * or respawned between solves. Solver state cannot leak across calls because entry.cpp builds a
 * fresh Scene and Solver each time. */
import createMlmpModule from './mlmp.mjs';

const modulePromise = createMlmpModule();

modulePromise.then(
  () => postMessage({ type: 'ready' }),
  (err) => postMessage({ type: 'ready', error: String(err && err.message ? err.message : err) })
);

self.onmessage = async (event) => {
  const msg = event.data;
  if (!msg || msg.type !== 'solve') return;

  let result;
  try {
    const Module = await modulePromise;
    /* MEMFS handoff: the scene reaches C++ as a file because Scene::loadScene opens its own
     * ifstream, and the browser build has no other filesystem. */
    Module.FS.writeFile('/scene.json', msg.sceneJson);
    const json = Module.solveScene(
      '/scene.json',
      msg.algo,
      msg.timeLimitS,
      msg.seed,
      msg.traceIntervalMs / 1000,
      msg.traceMaxFrames,
      (line) => postMessage({ type: 'frame', id: msg.id, frame: JSON.parse(line) })
    );
    result = JSON.parse(json);
  } catch (err) {
    /* A wasm abort or an OOM surfaces here; report it in the same shape errorJson uses so the
     * page has exactly one error path. */
    result = {
      schema: 'mlmp.run/1',
      error: { kind: 'runtime', message: String(err && err.message ? err.message : err) },
    };
  }
  postMessage({ type: 'result', id: msg.id, result });
};

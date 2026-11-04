/* Module worker around the wasm planner. solveScene blocks for the whole time budget — that is
 * the entire reason it lives here and not on the page's thread.
 *
 * Protocol
 *   main -> worker: {type:"solve", id, sceneJson, algo, timeLimitS, seed, traceIntervalMs,
 *                    traceMaxFrames}
 *   worker -> main: {type:"ready"} | {type:"frame", id, frame} | {type:"result", id, result}
 *
 * The module instance is created once and kept for this worker's lifetime; solveScene cannot be
 * interrupted, so the page cancels a run by terminating the whole worker and spawning a new
 * one. Solver state cannot leak across calls because entry.cpp builds a fresh Scene and Solver
 * each time. The handler is async, so two solves could otherwise interleave past
 * `await modulePromise` and the second would overwrite /scene.json while the first is still
 * reading it — hence the busy flag. */
import createMlmpModule from './mlmp.mjs';

const modulePromise = createMlmpModule();

modulePromise.then(
  () => postMessage({ type: 'ready' }),
  (err) => postMessage({ type: 'ready', error: String(err && err.message ? err.message : err) })
);

/* Every field crosses a structured-clone boundary from a page that reads them out of DOM
 * inputs, so a missing or NaN field is reachable. Report it in the same shape errorJson uses
 * instead of throwing, so the page keeps exactly one error path. */
function usageError(message) {
  return { schema: 'mlmp.run/1', error: { kind: 'usage', message } };
}

function validate(msg) {
  if (typeof msg.sceneJson !== 'string' || msg.sceneJson.length === 0) {
    return 'sceneJson must be a non-empty string';
  }
  for (const key of ['timeLimitS', 'seed', 'traceIntervalMs', 'traceMaxFrames']) {
    if (typeof msg[key] !== 'number' || !Number.isFinite(msg[key])) {
      return `${key} must be a finite number`;
    }
  }
  return null;
}

let busy = false;

self.onmessage = async (event) => {
  const msg = event.data;
  if (!msg || msg.type !== 'solve') return;

  const invalid = validate(msg);
  if (invalid !== null) {
    postMessage({ type: 'result', id: msg.id, result: usageError(invalid) });
    return;
  }
  if (busy) {
    postMessage({
      type: 'result',
      id: msg.id,
      result: usageError('a solve is already running in this worker'),
    });
    return;
  }

  busy = true;
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
  } finally {
    busy = false;
  }
  postMessage({ type: 'result', id: msg.id, result });
};

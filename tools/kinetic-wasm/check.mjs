// check.mjs -- the wasm half of the kinetic.wasm determinism proof: replays the
// script recorded in test/fixtures/kinetic_trace.json through kinetic.wasm and
// compares every 1 ms sample, bit for bit, against the native run.
//
//   node tools/kinetic-wasm/check.mjs [kinetic.wasm] [kinetic_trace.json]
//
// Exit 0 only when every block hash matches. The fixture is the native
// suite's output (test/native/test_kinetic_wasm_trace); regenerate it there.
import { readFile } from 'node:fs/promises';

const here = new URL('.', import.meta.url);
const wasmPath = process.argv[2] ?? new URL('build/kinetic.wasm', here);
const fixPath = process.argv[3] ?? new URL('../../test/fixtures/kinetic_trace.json', here);

const fx = JSON.parse(await readFile(fixPath, 'utf8'));
const { instance } = await WebAssembly.instantiate(await readFile(wasmPath), {});
const k = instance.exports;
k._initialize?.();

const SAMPLE = 64;           // sizeof(kinetic_sample)
const TUNE = 52;             // sizeof(kinetic_tuning)
const TUNE_POLICY = 48;      // offsetof(kinetic_tuning, infeasible_policy)
const OFFSET = 0xcbf29ce484222325n;
const PRIME = 0x100000001b3n;

const bytes = () => new Uint8Array(k.memory.buffer);
const cstr = (p) => {
  const b = bytes();
  let e = p;
  while (b[e]) e++;
  return new TextDecoder().decode(b.subarray(p, e));
};
// Doubles as a monotonic integer line, so |a - b| on it counts ULPs.
const ordinal = (x) => {
  const v = new DataView(new ArrayBuffer(8));
  v.setFloat64(0, x);
  const i = v.getBigInt64(0);
  return i < 0n ? -(i & 0x7fffffffffffffffn) : i;
};

console.log(`kinetic.wasm: ${cstr(k.kinetic_version())}`);
const [vmax, amax, jmax, rail, horizon] = fx.create;
const h = k.kinetic_create(vmax, amax, jmax, rail, horizon);
if (!h) throw new Error('kinetic_create refused the fixture limits');
if (k.kinetic_set_window(h, fx.window[0], fx.window[1]) !== 1) throw new Error('window refused');
const out = k.malloc(SAMPLE);
const tune = k.malloc(TUNE);

let next = 0;
let hash = OFFSET;
let accepted = 0;
let firstBad = -1;
let badBlocks = 0;
let maxUlp = 0n;
let badTrace = 0;
for (let tick = 0; tick < fx.steps; tick++) {
  for (; next < fx.events.length && fx.events[next][0] === tick; next++) {
    const e = fx.events[next];
    if (e[1] === 'tune') {
      k.kinetic_default_tuning(tune);
      bytes()[tune + TUNE_POLICY] = e[2];
      k.kinetic_set_tuning(h, tune);
    } else if (k.kinetic_submit_segment(h, e[2], e[3], e[4], e[5], e[6]) === 1) {
      accepted++;
    }
  }
  k.kinetic_step(h, fx.dt_s, out);
  const s = bytes().subarray(out, out + SAMPLE);
  for (const b of s) hash = BigInt.asUintN(64, (hash ^ BigInt(b)) * PRIME);
  if ((tick + 1) % fx.block === 0) {
    const i = (tick + 1) / fx.block - 1;
    if (hash.toString(16).padStart(16, '0') !== fx.hashes[i]) {
      badBlocks++;
      if (firstBad < 0) firstBad = i;
    }
    hash = OFFSET;
  }
  if ((tick + 1) % fx.trace_every === 0) {
    const want = fx.trace[(tick + 1) / fx.trace_every - 1];
    const dv = new DataView(k.memory.buffer, out, SAMPLE);
    [dv.getFloat64(8, true), dv.getFloat64(16, true), dv.getFloat64(24, true)].forEach((got, j) => {
      let d = ordinal(got) - ordinal(want[j]);
      if (d < 0n) d = -d;
      if (d > 0n) badTrace++;
      if (d > maxUlp) maxUlp = d;
    });
  }
}
k.kinetic_destroy(h);

const blocks = fx.hashes.length;
console.log(`events ${fx.events.length}, accepted ${accepted} (native ${fx.summary.accepted})`);
console.log(`samples ${fx.steps} at ${fx.dt_s * 1000} ms: ${blocks - badBlocks}/${blocks} blocks of ${fx.block} bit-identical`);
console.log(`p/v/a every ${fx.trace_every} ms: ${badTrace} of ${fx.trace.length * 3} differ, max ${maxUlp} ULP`);
if (badBlocks > 0 || accepted !== fx.summary.accepted) {
  if (firstBad >= 0) console.log(`FIRST DIVERGENCE in block ${firstBad}: ticks ${firstBad * fx.block}..${(firstBad + 1) * fx.block - 1} ms`);
  console.log('FAIL');
  process.exit(1);
}
console.log('PASS: kinetic.wasm reproduces the native planner bit for bit');

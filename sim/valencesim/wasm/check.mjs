// check.mjs -- Integral's in-process front, end to end in node: boot, a
// valence-js session over an in-memory socket, the catalog against its etag,
// one jog that moves the carriage. Exit 0 pass, 1 fail.
//
//   node sim/valencesim/wasm/check.mjs [build/integral.js] [--etag HEX]
//
// The socket below is the shape a Phosphor worker host follows (README.md,
// "The host contract"); this file is its acceptance proof, not a library.

import { pathToFileURL, fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { createSession, CH, PRIORITY, toHex, fromHex } from '../../../../Valence/clients/js/index.js';

const here = dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const flag = (f, d) => { const i = args.indexOf(f); return i >= 0 ? args[i + 1] : d; };
const modPath = resolve(args.find((a) => a.endsWith('.js')) || here + '/build/integral.js');
const ETAG = flag('--etag', 'a722d0009da17123');

let fails = 0;
const ok = (name, cond, extra) => {
  console.log('  [' + (cond ? 'PASS' : 'FAIL') + '] ' + name + (extra !== undefined ? '  -- ' + extra : ''));
  if (!cond) fails++;
};

const logs = [];
const { default: createIntegral } = await import(pathToFileURL(modPath).href);
const M = await createIntegral({ print: (l) => logs.push(l), printErr: (l) => logs.push(l) });

// ---- the C ABI's plumbing: wasm32, so pointers and size_t are u32 -----------
const cell = M._malloc(8);
const u32 = (p) => M.HEAPU32[p >> 2];
const bytesAt = (p, n) => M.HEAPU8.slice(p, p + n);
function call(fn, bytes, ...rest) {
  const p = M._malloc(Math.max(1, bytes.length));
  M.HEAPU8.set(bytes, p);
  try { return fn(...rest.slice(0, 1), p, bytes.length); } finally { M._free(p); }
}
function http(method, path) {
  const m = M.stringToNewUTF8(method), pa = M.stringToNewUTF8(path);
  const code = M._integral_http(m, pa, 0, 0, cell, cell + 4);
  M._free(m); M._free(pa);
  return { code, body: new TextDecoder().decode(bytesAt(u32(cell), u32(cell + 4))) };
}
function stateBlob() {
  M._integral_state_get(cell, cell + 4);
  return bytesAt(u32(cell), u32(cell + 4));
}

const opts = M.stringToNewUTF8(JSON.stringify({ homed: true }));
ok('integral_create boots', M._integral_create(opts, 0, 0) === 1);
M._free(opts);
const bootEtag = (logs.join('\n').match(/etag ([0-9a-f]+)/) || [])[1];
ok('boot log names the etag', !!bootEtag, bootEtag);

// ---- the socket a worker hands createSession --------------------------------
const sockets = new Map();
let nextId = 1;
class IntegralSocket {
  constructor(url) {
    this.url = url;
    this.protocol = 'valence.v1';
    this.binaryType = 'arraybuffer';
    this.readyState = 0;
    this.onopen = this.onmessage = this.onclose = this.onerror = null;
    this.id = nextId++;
    queueMicrotask(() => {
      if (M._integral_connect(this.id) !== 1) { this._closed(1013); return; }
      sockets.set(this.id, this);
      this.readyState = 1;
      if (this.onopen) this.onopen({});
    });
  }
  send(data) {
    if (this.readyState !== 1) return;
    const b = data instanceof Uint8Array ? data : new Uint8Array(data.buffer ? data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength) : data);
    call((id, p, n) => M._integral_send(id, p, n), b, this.id);
  }
  close() {
    if (this.readyState >= 2) return;
    M._integral_disconnect(this.id);
    this._closed(1000);
  }
  _closed(code) {
    this.readyState = 3;
    sockets.delete(this.id);
    queueMicrotask(() => { if (this.onclose) this.onclose({ code, reason: '', wasClean: true }); });
  }
}

// ---- the host clock: 1 ms passes, paced to the wall -------------------------
let simUs = 0;
let dirtySeen = false;
const t0 = performance.now();
function pump() {
  const target = Math.floor(performance.now() - t0) * 1000;
  while (simUs < target) {
    simUs += 1000;
    if (M._integral_tick(BigInt(simUs)) & 1) dirtySeen = true;
  }
  for (const s of [...sockets.values()]) {
    for (;;) {
      const r = M._integral_poll(s.id, cell, cell + 4);
      if (r === 0) break;
      if (r < 0) { s._closed(1006); break; }
      const msg = bytesAt(u32(cell), u32(cell + 4));
      if (s.onmessage) s.onmessage({ data: msg.buffer });
    }
  }
}
const timer = setInterval(pump, 1);

// ---- the session -------------------------------------------------------------
const cache = new Map();
const motion = [];
const s = createSession({
  host: 'integral', port: 0, clientKind: 'webui', clientName: 'integral check', autoReconnect: false,
  WebSocketImpl: IntegralSocket,
  catalogStore: { load: (h) => cache.get(h) || null, save: (h, etag, bytes) => cache.set(h, { etag, bytes }), clear: (h) => cache.delete(h) },
  token: async () => {
    const r = http('GET', '/uitoken');
    return r.code === 200 ? fromHex(JSON.parse(r.body).token) : null;
  },
  subscriptions: [[CH.SAFETY, 0, PRIORITY.critical], [CH.MOTION, 50, PRIORITY.elevated]],
});
let meta = null;
s.on('catalog', (e, m, mt) => { meta = mt; });
s.on('state', (ch, v) => { if (ch === CH.MOTION && v && Number.isFinite(v.pos_10um)) motion.push(v.pos_10um); });

const until = (pred, ms) => new Promise((res) => {
  const t = setInterval(() => { if (pred()) { clearInterval(t); res(true); } }, 10);
  setTimeout(() => { clearInterval(t); res(false); }, ms);
});
let live = false;
s.on('live', () => { live = true; });
s.connect();
ok('session reaches LIVE', await until(() => live, 8000));

if (live) {
  const etag = toHex(s.state.catalogEtag);
  ok('catalog fetched and verified against WELCOME', !!meta && meta.verified === true, s.catalogBytes && s.catalogBytes.length + ' B');
  ok('etag ' + ETAG, etag === ETAG, etag);
  ok('boot log etag matches WELCOME', bootEtag === etag);
  ok('control tier from /uitoken', s.state.roles >= 1, 'roles=' + s.state.roles);
  ok('GET /nope is 404', http('GET', '/nope').code === 404);

  const move = (s.catalog || []).find((e) => e.name === 'move');
  ok('catalog carries "move"', !!move);
  await until(() => motion.length > 0, 2000);
  const p0 = motion.at(-1);
  if (move) await s.sendIntent(move.id, { 1: 80 }).catch((e) => ok('jog accepted', false, e.message));
  const moved = await until(() => motion.length && Math.abs(motion.at(-1) - 80) < 0.5, 8000);
  ok('jog lands at 80 mm', moved, p0 + ' -> ' + motion.at(-1) + ' mm');
  s.close();
}

await until(() => false, 50);
ok('state blob reported dirty and non-empty', dirtySeen && stateBlob().length > 0, stateBlob().length + ' B');
clearInterval(timer);
console.log(fails ? '\nFAIL (' + fails + ')' : '\nALL PASS');
process.exit(fails ? 1 : 0);

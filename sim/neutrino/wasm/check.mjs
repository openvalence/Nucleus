// check.mjs -- Neutrino's in-process front, end to end in node: boot, a
// valence-js session over an in-memory socket, the catalog against its etag,
// one jog that moves the carriage, a config-set a control session is refused
// and the configure ask a PAIR double press grants (RFC-114), a discovery
// probe before and after a PAIR press. Exit 0 pass, 1 fail.
//
//   node sim/neutrino/wasm/check.mjs [build/neutrino.js] [--etag HEX]
//
// The socket below is the shape a Phosphor worker host follows (README.md,
// "The host contract"); this file is its acceptance proof, not a library.

import { pathToFileURL, fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { createSession, CH, PRIORITY, toHex, fromHex, encodeDiscoverProbe, decodeDiscoverReply } from '../../../../Valence/clients/js/index.js';

const here = dirname(fileURLToPath(import.meta.url));
const args = process.argv.slice(2);
const flag = (f, d) => { const i = args.indexOf(f); return i >= 0 ? args[i + 1] : d; };
const modPath = resolve(args.find((a) => a.endsWith('.js')) || here + '/build/neutrino.js');
const ETAG = flag('--etag', 'd5895d9c2c5bd88d');

let fails = 0;
const ok = (name, cond, extra) => {
  console.log('  [' + (cond ? 'PASS' : 'FAIL') + '] ' + name + (extra !== undefined ? '  -- ' + extra : ''));
  if (!cond) fails++;
};

const logs = [];
const { default: createNeutrino } = await import(pathToFileURL(modPath).href);
const M = await createNeutrino({ print: (l) => logs.push(l), printErr: (l) => logs.push(l) });

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
  const code = M._neutrino_http(m, pa, 0, 0, cell, cell + 4);
  M._free(m); M._free(pa);
  return { code, body: new TextDecoder().decode(bytesAt(u32(cell), u32(cell + 4))) };
}
function stateBlob() {
  M._neutrino_state_get(cell, cell + 4);
  return bytesAt(u32(cell), u32(cell + 4));
}

const opts = M.stringToNewUTF8(JSON.stringify({ homed: true }));
ok('neutrino_create boots', M._neutrino_create(opts, 0, 0) === 1);
M._free(opts);
const bootEtag = (logs.join('\n').match(/etag ([0-9a-f]+)/) || [])[1];
ok('boot log names the etag', !!bootEtag, bootEtag);

// ---- the socket a worker hands createSession --------------------------------
const sockets = new Map();
let nextId = 1;
class NeutrinoSocket {
  constructor(url) {
    this.url = url;
    this.protocol = 'valence.v1';
    this.binaryType = 'arraybuffer';
    this.readyState = 0;
    this.onopen = this.onmessage = this.onclose = this.onerror = null;
    this.id = nextId++;
    queueMicrotask(() => {
      if (M._neutrino_connect(this.id) !== 1) { this._closed(1013); return; }
      sockets.set(this.id, this);
      this.readyState = 1;
      if (this.onopen) this.onopen({});
    });
  }
  send(data) {
    if (this.readyState !== 1) return;
    const b = data instanceof Uint8Array ? data : new Uint8Array(data.buffer ? data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength) : data);
    call((id, p, n) => M._neutrino_send(id, p, n), b, this.id);
  }
  close() {
    if (this.readyState >= 2) return;
    M._neutrino_disconnect(this.id);
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
    if (M._neutrino_tick(BigInt(simUs)) & 1) dirtySeen = true;
  }
  for (const s of [...sockets.values()]) {
    for (;;) {
      const r = M._neutrino_poll(s.id, cell, cell + 4);
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
  host: 'neutrino', port: 0, clientKind: 'webui', clientName: 'neutrino check', autoReconnect: false,
  WebSocketImpl: NeutrinoSocket,
  catalogStore: { load: (h) => cache.get(h) || null, save: (h, etag, bytes) => cache.set(h, { etag, bytes }), clear: (h) => cache.delete(h) },
  token: async () => {
    const r = http('GET', '/uitoken');
    return r.code === 200 ? fromHex(JSON.parse(r.body).token) : null;
  },
  subscriptions: [[CH.SAFETY, 0, PRIORITY.critical], [CH.MOTION, 50, PRIORITY.elevated]],
});
let meta = null;
const nacks = [];
let grant = null;
s.on('catalog', (e, m, mt) => { meta = mt; });
s.on('nack', (n) => nacks.push(n));
s.on('pairGrant', (g) => { grant = g; });
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

  // ---- configure tier: control drives, only configure sets (SPEC 12.2) ------
  const cfg = (s.catalog || []).find((e) => e.name === 'config-set');
  const jogSpeed = cfg && (cfg.schema || []).find((f) => f.key === 3);
  ok('"config-set" floor control, a limit key raised to configure', !!cfg && cfg.access === 1 && !!jogSpeed
    && jogSpeed.access === 2, cfg && 'access=' + cfg.access + ' key3=' + (jogSpeed && jogSpeed.access));
  if (cfg) {
    ok('control: the stroke window is accepted', await s.sendIntent(cfg.id, { 2: 300 }).then(() => true,
      (e) => (ok('', false, e.message), false)));
    const refused = await s.sendIntent(cfg.id, { 3: 60 }).then(() => false, () => true);
    const n = nacks.at(-1);
    ok('control: config-set refused ACCESS_DENIED "needs configure"', refused && !!n && n.name === 'ACCESS_DENIED'
      && n.detail === 'needs configure', n && n.name + ' ' + n.detail);
    ok('configure ask sent', s.sendPairReq({ role: 2 }));
    await until(() => false, 50);
    ok('no grant before the double press', grant === null);
    M._neutrino_pair_double_press();
    ok('PAIR double press grants configure', await until(() => grant && grant.role === 2, 2000),
      grant && 'role=' + grant.role);
    ok('session upgraded in place', s.state.roles === 2, 'roles=' + s.state.roles);
    ok('configure: config-set accepted', await s.sendIntent(cfg.id, { 3: 60 }).then(() => true, (e) => (ok('', false, e.message), false)));
  }
  s.close();
}

await until(() => false, 50);
ok('state blob reported dirty and non-empty', dirtySeen && stateBlob().length > 0, stateBlob().length + ' B');

// ---- the host's datagrams: the board's responder, then the PAIR press ----------
function probe(src, nonce) {
  const n = call((ip, p, len) => M._neutrino_datagram(p, len, ip, 8282, cell), encodeDiscoverProbe(nonce), src);
  return n ? decodeDiscoverReply(bytesAt(u32(cell), n)) : null;
}
const r1 = probe(0x0a000001, 0x1234abcd);
ok('a probe gets the Neutrino reply', !!r1 && r1.nonce === 0x1234abcd && r1.hub_name === 'Neutrino' && r1.ws_port === 8282
  && r1.catalog_etag === bootEtag, JSON.stringify(r1));
ok('the same source inside its window gets none', probe(0x0a000001, 1) === null);
ok('no window before the press', !!r1 && !r1.pairing_window_open);
M._neutrino_pair_press();
await until(() => false, 30);
const r2 = probe(0x0a000002, 7);
ok('PAIR press opens the presence window', !!r2 && r2.pairing_window_open, JSON.stringify(r2 && r2.flags));
clearInterval(timer);
console.log(fails ? '\nFAIL (' + fails + ')' : '\nALL PASS');
process.exit(fails ? 1 : 0);

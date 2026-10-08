// lab.js -- the Kinetic² lab page: controls, the plot, the stats; the planning runs in host.js
// Constraints:
// - Every control change re-renders the whole run (a newer render supersedes the older one in the
//   worker); nothing here plans or shapes motion. The reference curve, the knots on the wire and the
//   scheduler are Phosphor's own modules (served at /player/).
// - Units on screen are the machine's: mm, mm/s, mm/s^2, ms. Script positions 0..1 map through the
//   Range (T) onto the window, as the player maps them.
// See: README.md

import { parseFunscript } from '/player/funscript.js';
import { mountInterp, INTERP } from '/player/interp.js';

const $ = (id) => document.getElementById(id);
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const fmt = (v, d = 2) => (Number.isFinite(v) ? v.toFixed(d) : '-');
const ms = (t) => (Math.abs(t) >= 10000 ? (t / 1000).toFixed(2) + ' s' : Math.round(t) + ' ms');

// kinetic2::AnomalyKind by number; reserved kinds are blank. 12 renders: never a drop.
const ANOMALY = ['', 'plan failed (knot dropped)', 'settle (ran dry, braked)', 'end velocity clamped', 'stretched', '', 'trimmed', '', '', '', 'dwell zeroed', 'knot refused', 'piece over a ceiling (least-over trim)'];
const ANOMALY_COLOR = { 1: '#E05BFF', 2: '#E05BFF', 3: '#F5A524', 4: '#FF4D4D', 6: '#F5A524', 10: '#F5A524', 11: '#E05BFF', 12: '#FF4D4D' };
const FLAG = { busy: 1, shaped: 2, fallback: 4, clamped: 8, refused: 16 };

// ---- state ------------------------------------------------------------------------------------------
const st = {
  source: null,                  // {kind: 'script', script, name} | {kind: 'events', events, name}
  interp: { ...INTERP, mode: 'pchip' },
  T: { lo: 0, hi: 1, invert: 0 },
  machine: { rail: 500, lo: 0, hi: 500, vmax: 1200, amax: 100000, jmax: 20000000 },
  tuning: {},                    // member -> value, seeded from the twin's factory set
  defaults: {},
  link: { exact: 0, family: 1, netMs: 5, clockErrMs: 0, playerTickMs: 16.667, horizonMs: 250, low: 0 },
  gen: { kind: 'sine', periodMs: 1300, amp: 0.3, knotMs: 100, seconds: 20 },
  view: 'position',
  run: null, ghost: null, cursorMs: null,
  dom: [-1500, 20000],
};

// ---- built-in scripts ----------------------------------------------------------------------------
const GEN = {
  sine: (g, t) => 0.5 + g.amp * Math.sin(2 * Math.PI * t / g.periodMs),
  triangle: (g, t) => { const u = (t / g.periodMs) % 1; return 0.5 + g.amp * (u < 0.5 ? 4 * u - 1 : 3 - 4 * u); },
  sawtooth: (g, t) => { const u = (t / g.periodMs) % 1; return 0.5 + g.amp * (u < 0.2 ? -1 + 10 * u : 1 - 2.5 * (u - 0.2)); },
  square: (g, t) => 0.5 + g.amp * ((t / g.periodMs) % 1 < 0.5 ? 1 : -1),
  chirp: (g, t) => { const T = g.seconds * 1000, f0 = 1000 / g.periodMs, f1 = 4 * f0; return 0.5 + g.amp * Math.sin(2 * Math.PI * (f0 * t + (f1 - f0) * t * t / (2 * T)) / 1000); },
  'sine + dwell': (g, t) => { const u = (t / g.periodMs) % 1; return u < 0.6 ? 0.5 + g.amp * Math.sin(2 * Math.PI * u / 0.6) : 0.5; },
};
function generate(g) {
  const f = GEN[g.kind], n = Math.floor(g.seconds * 1000 / g.knotMs) + 1;
  const at = new Float64Array(n), pos = new Float32Array(n);
  for (let i = 0; i < n; i++) { at[i] = i * g.knotMs; pos[i] = clamp(f(g, at[i]), 0, 1); }
  return { at, pos, durationMs: at[n - 1], ignored: [], notes: [], name: g.kind };
}

// ---- controls ----------------------------------------------------------------------------------
function field(grid, spec, obj, onChange) {
  const label = document.createElement('label');
  label.textContent = spec.label;
  label.title = spec.title || spec.key;
  let input;
  if (spec.options) {
    input = document.createElement('select');
    for (const [v, text] of spec.options) { const o = document.createElement('option'); o.value = String(v); o.textContent = text; input.append(o); }
    input.value = String(obj[spec.key]);
    input.addEventListener('change', () => { obj[spec.key] = Number(input.value); onChange(); });
  } else {
    input = document.createElement('input');
    input.type = 'number';
    if (spec.min != null) input.min = spec.min;
    if (spec.max != null) input.max = spec.max;
    input.step = spec.step ?? 'any';
    input.value = String(obj[spec.key]);
    input.addEventListener('change', () => {
      const v = Number(input.value);
      if (!Number.isFinite(v)) { input.value = String(obj[spec.key]); return; }
      obj[spec.key] = v; onChange();
    });
  }
  input.setAttribute('aria-label', spec.label);
  const unit = document.createElement('span');
  unit.className = 'unit';
  unit.textContent = spec.unit || '';
  grid.append(label, input, unit);
  return input;
}
const inputs = {};
function fields(grid, specs, obj, onChange) { for (const s of specs) inputs[s.key] = field(grid, s, obj, onChange); }
function refill(specs, obj) { for (const s of specs) if (inputs[s.key]) inputs[s.key].value = String(obj[s.key]); }

const MACHINE = [
  { key: 'rail', label: 'rail', unit: 'mm', min: 1, step: 1 },
  { key: 'lo', label: 'window lo', unit: 'mm', min: 0, step: 1 },
  { key: 'hi', label: 'window hi', unit: 'mm', min: 1, step: 1 },
  { key: 'vmax', label: 'input speed', unit: 'mm/s', min: 0, step: 10 },
  { key: 'amax', label: 'input accel', unit: 'mm/s²', min: 0, step: 100 },
  { key: 'jmax', label: 'input jerk', unit: 'mm/s³', min: 0, step: 100000 },
];
const TUNE = [
  { key: 'infeasible_policy', label: 'policy', options: [[0, 'stretch (keep the stroke)'], [1, 'blend (keep the deadline)']] },
  { key: 'amplitude_budget', label: 'amplitude floor', unit: 'share', min: 0, max: 1, step: 0.05, title: 'Blend never trims a stroke below this share of it' },
  { key: 'corner', label: 'corner', options: [[1, 'cubic (author\'s corners)'], [0, 'continuous']] },
  { key: 'curve_policy', label: 'curve policy', options: [[0, 'follow the sender'], [1, 'C1'], [2, 'C2']] },
  { key: 'react_us', label: 'react', unit: 'µs', min: 0, step: 500, title: 'the reaction horizon: a knot arriving mid-motion re-plans from this far ahead' },
  { key: 'lookahead_us', label: 'lookahead', unit: 'µs', min: 0, step: 10000 },
  { key: 'chase_dense_us', label: 'chase dense', unit: 'µs', min: 0, step: 1000 },
  { key: 'vmax_ovr', label: 'vmax override', unit: 'w/s', min: 0, step: 0.1, title: 'window units per second; 0 derives from the mm limits' },
  { key: 'amax_ovr', label: 'amax override', unit: 'w/s²', min: 0, step: 1 },
  { key: 'jmax_ovr', label: 'jmax override', unit: 'w/s³', min: 0, step: 100 },
];
const LINK = [
  { key: 'exact', label: 'sender', options: [[0, 'Phosphor\'s scheduler'], [1, 'exact knots (no cap, no merge)']], title: 'the scheduler caps end velocities (knotSlope) and merges knots within the dwell span (dwellMerge); exact sends the author\'s knots as they are' },
  { key: 'family', label: 'family wish', options: [[1, 'C1 cubic (the player\'s)'], [2, 'C2 quintic'], [0, 'unspecified']] },
  { key: 'horizonMs', label: 'horizon', options: [[250, '250 ms'], [500, '500 ms'], [1000, '1000 ms']] },
  { key: 'netMs', label: 'network', unit: 'ms', min: 0, step: 1, title: 'one-way, client to hub' },
  { key: 'clockErrMs', label: 'clock error', unit: 'ms', step: 1, title: 'the client\'s hub-clock estimate minus the hub\'s clock' },
  { key: 'playerTickMs', label: 'player tick', unit: 'ms', min: 1, step: 0.1, title: 'the scheduler runs once per animation frame' },
  { key: 'low', label: 'low latency', options: [[0, 'off'], [1, 'on (50 ms lead)']] },
];
const RANGE = [
  { key: 'lo', label: 'low', unit: 'share', min: 0, max: 1, step: 0.01 },
  { key: 'hi', label: 'high', unit: 'share', min: 0, max: 1, step: 0.01 },
  { key: 'invert', label: 'invert', options: [[0, 'no'], [1, 'yes']] },
];
const GENF = [
  { key: 'periodMs', label: 'period', unit: 'ms', min: 50, step: 10 },
  { key: 'amp', label: 'amplitude', unit: 'share', min: 0, max: 0.5, step: 0.05 },
  { key: 'knotMs', label: 'knot every', unit: 'ms', min: 5, step: 5 },
  { key: 'seconds', label: 'length', unit: 's', min: 1, max: 600, step: 1 },
];

// ---- the worker ----------------------------------------------------------------------------------
const worker = new Worker('host.js', { type: 'module' });
let seq = 0, pending = null, ready = false;
worker.onmessage = (e) => {
  const r = e.data;
  if (r.ready) { ready = true; $('version').textContent = r.ready; st.defaults = r.defaults; st.tuning = { ...r.defaults }; refill(TUNE, st.tuning); render(); return; }
  if (r.error && r.id == null) { status('kinetic.wasm: ' + r.error, true); return; }
  if (r.id !== pending) return;
  pending = null;
  if (r.error) { status(r.error, true); $('runstate').textContent = 'failed'; return; }
  st.run = r;
  r.seq = r.id;
  $('runstate').textContent = `rendered in ${Math.round(r.ms)} ms`;
  if (!st.fitted) { fit(); st.fitted = true; }
  stats();
  draw();
};
worker.onerror = (e) => { e.preventDefault(); status('worker: ' + e.message, true); };
worker.postMessage({ init: { wasmUrl: '/kinetic.wasm' } });

let timer = 0;
function render() {
  if (!ready || !st.source) return;
  clearTimeout(timer);
  timer = setTimeout(() => {
    const m = st.machine;
    const req = {
      source: st.source.kind === 'script'
        ? { kind: 'script', script: { at: st.source.script.at, pos: st.source.script.pos }, interp: st.interp, T: { lo: st.T.lo, hi: st.T.hi, invert: !!st.T.invert }, spanMm: m.hi - m.lo }
        : { kind: 'events', events: st.source.events },
      machine: { vmax: m.vmax, amax: m.amax, jmax: m.jmax, rail: m.rail, horizonMs: st.link.horizonMs, window: [m.lo, m.hi] },
      tuning: st.tuning,
      link: { ...st.link },
    };
    pending = ++seq;
    $('runstate').textContent = 'rendering…';
    worker.postMessage({ id: pending, req });
  }, 40);
}

// ---- sources ---------------------------------------------------------------------------------------
function useScript(script, name) {
  st.source = { kind: 'script', script, name };
  st.fitted = false;
  const n = script.at.length;
  $('scriptinfo').textContent = `${name}: ${n} actions, ${ms(script.at[n - 1])}` + (script.notes && script.notes.length ? '; ' + script.notes.join(', ') : '');
  render();
}
function useEvents(events, name) {
  st.source = { kind: 'events', events, name };
  st.fitted = false;
  $('scriptinfo').textContent = `${name}: recording, ${events.length} events (the player is bypassed; the curve is unknown)`;
  render();
}
function loadText(text, name) {
  try {
    const doc = JSON.parse(text);
    if (doc && Array.isArray(doc.events) && !doc.actions) {
      // The lab's recording, or the Nucleus trace fixture (tick index = arrival ms, tune = policy).
      const events = doc.events.map((e) => (e[1] === 'tune' && typeof e[2] === 'number' ? [e[0], 'tune', { infeasible_policy: e[2] }] : e));
      if (doc.create) { const [vmax, amax, jmax, rail, horizon] = doc.create; Object.assign(st.machine, { vmax, amax, jmax, rail }); if (horizon) st.link.horizonMs = horizon; }
      if (doc.window) { st.machine.lo = doc.window[0]; st.machine.hi = doc.window[1]; }
      if (doc.machine) Object.assign(st.machine, doc.machine);
      if (doc.tuning) Object.assign(st.tuning, doc.tuning);
      if (doc.link) Object.assign(st.link, doc.link);
      refill(MACHINE, st.machine); refill(TUNE, st.tuning); refill(LINK, st.link);
      useEvents(events, name);
      return;
    }
    useScript(parseFunscript(doc, name), name);
  } catch (e) { status(name + ': ' + (e.message || e), true); }
}
async function loadFile(f) { loadText(await f.text(), f.name); }

function saveRecording() {
  const r = st.run;
  if (!r) return;
  const events = [];
  for (const b of r.bundles) for (const x of b.recs) events.push([+b.arriveMs.toFixed(3), 'seg', ...x]);
  const doc = { about: 'Kinetic² lab recording: what the hub received, by arrival ms. Load it back into the lab (replayable without the player).',
    source: st.source.name, machine: { ...st.machine }, link: { ...st.link }, tuning: { ...st.tuning }, interp: st.interp, T: st.T, playAtMs: r.playAtMs, events };
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([JSON.stringify(doc)], { type: 'application/json' }));
  a.download = (st.source.name || 'run').replace(/\.[^.]*$/, '') + '.recording.json';
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

// ---- stats ----------------------------------------------------------------------------------------
function stats() {
  const r = st.run, dl = $('stats');
  dl.replaceChildren();
  const row = (k, v, cls = '') => { const dt = document.createElement('dt'); dt.textContent = k; const dd = document.createElement('dd'); dd.textContent = v; dd.className = cls; dl.append(dt, dd); };
  const counts = {};
  for (const [, kind] of r.anomalies) counts[kind] = (counts[kind] || 0) + 1;
  // Flag counts over the script's own span: the preroll from the machine's rest (outside the window) is not a clamp.
  let shaped = 0, stretched = 0, clampedTicks = 0;
  for (let i = Math.ceil(r.playAtMs); i < r.flags.length; i++) { const f = r.flags[i]; if (f & FLAG.shaped) shaped++; if (f & FLAG.fallback) stretched++; if (f & FLAG.clamped) clampedTicks++; }
  if (r.ref) {
    const i0 = Math.ceil(r.playAtMs), i1 = Math.min(r.plan.length, Math.floor(r.playAtMs + (st.source.script.at[st.source.script.at.length - 1] || 0)));
    let sum = 0, sq = 0, mx = 0, at = 0, n = 0;
    const errs = [];
    for (let i = i0; i < i1; i++) { const e = Math.abs(r.plan[i] - r.ref[i]); sum += e; sq += e * e; n++; errs.push(e); if (e > mx) { mx = e; at = i; } }
    errs.sort((a, b) => a - b);
    const p95 = errs.length ? errs[Math.floor(errs.length * 0.95)] : NaN;
    row('plan vs curve', `mean ${fmt(sum / n, 3)} mm, rms ${fmt(Math.sqrt(sq / n), 3)} mm, p95 ${fmt(p95, 3)} mm`);
    row('worst', `${fmt(mx, 2)} mm at ${ms(at - r.playAtMs)}`, mx > 2 ? 'warn' : '');
    st.worstMs = at - r.playAtMs;
    // The emitter is what the rail does: the plan one tick later, on the step grid.
    let esum = 0, emx = 0;
    for (let i = i0; i < i1; i++) { const e = Math.abs(r.position[i] - r.ref[i]); esum += e; if (e > emx) emx = e; }
    row('emitter vs curve', `mean ${fmt(esum / n, 3)} mm, worst ${fmt(emx, 2)} mm`);
    // Timing: the shift (ms) at which the plan best matches the curve, and what is left then (shape error alone).
    let best = 0, bestMean = Infinity;
    for (let s = -12; s <= 12; s++) {
      let acc = 0, cnt = 0;
      for (let i = i0 + 200; i < i1 - 200; i += 2) { acc += Math.abs(r.plan[i + s] - r.ref[i]); cnt++; }
      if (acc / cnt < bestMean) { bestMean = acc / cnt; best = s; }
    }
    row('timing', best === 0 ? `the plan is on time; shape error alone mean ${fmt(bestMean, 3)} mm` : `the plan ${best < 0 ? 'leads' : 'lags'} the curve by ${Math.abs(best)} ms; shifted, the shape error alone is mean ${fmt(bestMean, 3)} mm (after the first 200 ms)`);
  }
  let recs = 0, acc = 0, drop = 0;
  for (const b of r.bundles) { recs += b.n; acc += b.accepted; drop += b.dropped; }
  row('bundles', `${r.bundles.length} bundles, ${recs} segments: ${acc} accepted, ${drop} dropped` + (r.refused ? `, ${r.refused} refused` : ''), drop || r.refused ? 'bad' : '');
  const spends = Object.entries(counts).filter(([k]) => ANOMALY[k]).map(([k, n]) => `${ANOMALY[k]} ×${n}`).join(', ') || 'none';
  row('anomalies', spends, Object.keys(counts).length ? 'warn' : '');
  row('ticks', `${r.plan.length} ms: shaped ${shaped}, stretched ${stretched}, window-clamped ${clampedTicks}`);
  if (r.capped && r.capped.length) row('handoff cap', `${r.capped.length} end velocities sent below the curve's slope (scheduler.js knotSlope, 1.5 × the lesser chord): the plan leaves those knots slower than the purple line`, 'warn');
  row('play', `preroll ${ms(r.playAtMs)}, then ${ms(r.endMs - r.playAtMs - 1500)} of script`);
  // The table: every anomaly and every knot the solver spent on, by script time.
  const tb = $('anoms').querySelector('tbody');
  tb.replaceChildren();
  const rows = r.anomalies.map(([t, kind]) => ({ t: t - r.playAtMs, what: ANOMALY[kind] || 'kind ' + kind, detail: '', kind }));
  const seen = new Set();
  for (const snap of r.knotsLog) for (const kn of snap.knots) {
    if (!(kn.share < 0.999 || kn.stretched_s > 0 || kn.dropped || kn.clamped)) continue;
    const key = Math.round(kn.t_us / 1000) + ':' + (kn.dropped ? 'd' : kn.stretched_s > 0 ? 's' : kn.clamped ? 'c' : 'b');
    if (seen.has(key)) continue;
    seen.add(key);
    const what = kn.dropped ? 'knot dropped' : kn.stretched_s > 0 ? 'knot stretched' : kn.clamped ? 'end velocity cut' : 'stroke trimmed';
    const detail = kn.stretched_s > 0 ? `+${(kn.stretched_s * 1000).toFixed(1)} ms` : kn.share < 0.999 ? `share ${kn.share.toFixed(2)}` : '';
    rows.push({ t: kn.t_us / 1000 - r.playAtMs, what, detail: detail + (kn.worst ? `, worst ratio ${kn.worst.toFixed(2)}` : ''), kind: kn.dropped ? 1 : kn.stretched_s > 0 ? 4 : 6, knot: kn });
  }
  for (const [t, sent, want] of r.capped || []) rows.push({ t, what: 'end velocity capped by the player', detail: `sent ${fmt(sent, 0)} mm/s, the curve's slope ${fmt(want, 0)} mm/s`, kind: 3 });
  rows.sort((a, b) => a.t - b.t);
  st.events = rows;
  $('ancount').textContent = rows.length ? `(${rows.length})` : '';
  for (const x of rows.slice(0, 400)) {
    const tr = document.createElement('tr');
    tr.className = 'pick';
    for (const v of [ms(x.t), x.what, x.detail]) { const td = document.createElement('td'); td.textContent = v; tr.append(td); }
    tr.addEventListener('click', () => { st.cursorMs = x.t; center(x.t); });
    tb.append(tr);
  }
}

// ---- the plot --------------------------------------------------------------------------------------
const plot = $('plot'), overlay = $('overlay'), box = $('plotbox');
const PAD = { l: 52, r: 12, t: 22, b: 70 };
let W = 0, H = 0, dpr = 1;
function size() {
  dpr = window.devicePixelRatio || 1;
  W = box.clientWidth; H = box.clientHeight;
  for (const c of [plot, overlay]) { c.width = Math.round(W * dpr); c.height = Math.round(H * dpr); }
  draw();
}
const xOf = (t) => PAD.l + (t - st.dom[0]) / (st.dom[1] - st.dom[0]) * (W - PAD.l - PAD.r);
const tOf = (x) => st.dom[0] + (x - PAD.l) / (W - PAD.l - PAD.r) * (st.dom[1] - st.dom[0]);
function yScale() {
  const r = st.run, m = st.machine;
  if (st.view === 'position') { const pad = (m.hi - m.lo) * 0.08; return { lo: m.lo - pad, hi: m.hi + pad, unit: 'mm' }; }
  const arr = st.view === 'velocity' ? r.vel : r.acc;
  let mx = 1;
  for (const v of arr) if (Math.abs(v) > mx) mx = Math.abs(v);
  return { lo: -mx * 1.1, hi: mx * 1.1, unit: st.view === 'velocity' ? 'mm/s' : 'mm/s²' };
}
function fit() {
  const r = st.run;
  if (!r) return;
  st.dom = [-r.playAtMs, r.endMs - r.playAtMs];
  draw();
}
function center(t) {
  const w = Math.min(st.dom[1] - st.dom[0], 4000);
  st.dom = [t - w / 2, t + w / 2];
  draw();
}
function niceStep(span, px, want) {
  const raw = span / (px / want);
  const p = Math.pow(10, Math.floor(Math.log10(raw)));
  for (const m of [1, 2, 5, 10]) if (m * p >= raw) return m * p;
  return 10 * p;
}
function draw() {
  const g = plot.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);
  const r = st.run;
  if (!r) return;
  const ys = yScale(), laneTop = PAD.t, laneBot = H - PAD.b, errTop = laneBot + 14, errBot = H - 22;
  const yOf = (v) => laneTop + (ys.hi - v) / (ys.hi - ys.lo) * (laneBot - laneTop);
  // value labels, outside the clip
  const stepY0 = niceStep(ys.hi - ys.lo, laneBot - laneTop, 44);
  g.fillStyle = '#666C78'; g.font = '11px ' + getComputedStyle(document.body).getPropertyValue('--mono'); g.textAlign = 'right';
  for (let v = Math.ceil(ys.lo / stepY0) * stepY0; v <= ys.hi; v += stepY0) g.fillText(fmt(v, stepY0 < 1 ? 2 : 0), PAD.l - 6, yOf(v) + 4);
  g.save();
  g.beginPath(); g.rect(PAD.l, 0, W - PAD.l - PAD.r, H); g.clip();
  // the window band (position view)
  if (st.view === 'position') { g.fillStyle = '#0D1017'; g.fillRect(PAD.l, yOf(st.machine.hi), W - PAD.l - PAD.r, yOf(st.machine.lo) - yOf(st.machine.hi)); }
  // time grid
  const span = st.dom[1] - st.dom[0], stepT = niceStep(span, W - PAD.l - PAD.r, 90);
  g.strokeStyle = '#1A1D24'; g.fillStyle = '#666C78'; g.font = '11px ' + getComputedStyle(document.body).getPropertyValue('--mono');
  g.textAlign = 'center';
  for (let t = Math.ceil(st.dom[0] / stepT) * stepT; t <= st.dom[1]; t += stepT) {
    const x = xOf(t);
    g.beginPath(); g.moveTo(x, laneTop); g.lineTo(x, errBot); g.stroke();
    g.fillText(stepT >= 1000 ? (t / 1000).toFixed(stepT >= 10000 ? 0 : 1) + ' s' : Math.round(t) + ' ms', x, H - 8);
  }
  // the script zero line
  g.strokeStyle = '#2A2F38'; g.beginPath(); g.moveTo(xOf(0), laneTop); g.lineTo(xOf(0), errBot); g.stroke();
  // value grid
  for (let v = Math.ceil(ys.lo / stepY0) * stepY0; v <= ys.hi; v += stepY0) {
    const y = yOf(v);
    g.strokeStyle = '#1A1D24'; g.beginPath(); g.moveTo(PAD.l, y); g.lineTo(W - PAD.r, y); g.stroke();
  }
  g.textAlign = 'left';
  g.fillText(ys.unit, PAD.l + 4, laneTop + 11);

  const i0 = Math.max(0, Math.floor(st.dom[0] + r.playAtMs)), i1 = Math.min(r.plan.length - 1, Math.ceil(st.dom[1] + r.playAtMs));
  const stride = Math.max(1, Math.floor((i1 - i0) / ((W - PAD.l - PAD.r) * 2)));
  const line = (arr, color, width, dash, offset = r.playAtMs) => {
    g.strokeStyle = color; g.lineWidth = width; g.setLineDash(dash || []);
    g.beginPath();
    let first = true;
    for (let i = i0; i <= i1; i += stride) { const x = xOf(i - offset), y = yOf(arr[i]); if (first) { g.moveTo(x, y); first = false; } else g.lineTo(x, y); }
    g.stroke(); g.setLineDash([]);
  };
  if (st.view === 'position') {
    // the actions: the raw script through the range, in mm
    if (st.source.kind === 'script') {
      const s = st.source.script, m = st.machine, T = st.T;
      const toMm = (p) => m.lo + (T.lo + (T.invert ? 1 - p : p) * (T.hi - T.lo)) * (m.hi - m.lo);
      g.strokeStyle = '#8A8F99'; g.lineWidth = 1; g.beginPath();
      let first = true;
      for (let i = 0; i < s.at.length; i++) {
        if (s.at[i] < st.dom[0] - 5000 || s.at[i] > st.dom[1] + 5000) continue;
        const x = xOf(s.at[i]), y = yOf(toMm(s.pos[i]));
        if (first) { g.moveTo(x, y); first = false; } else g.lineTo(x, y);
      }
      g.stroke();
      if (span < 20000) { g.fillStyle = '#8A8F99'; for (let i = 0; i < s.at.length; i++) { if (s.at[i] < st.dom[0] || s.at[i] > st.dom[1]) continue; g.fillRect(xOf(s.at[i]) - 1.5, yOf(toMm(s.pos[i])) - 1.5, 3, 3); } }
    }
    if (r.ref) line(r.ref, '#A78BFA', 1.5);
    if (st.ghost) { const gh = st.ghost; const save = i1; line(gh.plan, '#5B6B8A', 1.2, [4, 3], gh.playAtMs); void save; }
    line(r.plan, '#4DA6FF', 1.5);
  } else {
    line(st.view === 'velocity' ? r.vel : r.acc, '#4DA6FF', 1.2);
  }
  // the knots the solver placed, when zoomed in: a dot per knot, a ring where it spent
  if (span < 12000 && st.view === 'position') {
    const seenK = new Set();
    for (let si = r.knotsLog.length - 1; si >= 0; si--) {
      const snap = r.knotsLog[si];
      if (snap.atMs - r.playAtMs > st.dom[1] + 1000 || snap.atMs - r.playAtMs < st.dom[0] - 2000) continue;
      for (const kn of snap.knots) {
        const tt = kn.t_us / 1000 - r.playAtMs, key = Math.round(tt);
        if (seenK.has(key) || tt < st.dom[0] || tt > st.dom[1]) continue;
        seenK.add(key);
        const x = xOf(tt), y = yOf(st.machine.lo + kn.p * (st.machine.hi - st.machine.lo));
        g.fillStyle = kn.dropped ? '#E05BFF' : kn.stretched_s > 0 ? '#FF4D4D' : kn.share < 0.999 ? '#F5A524' : '#D7DCE5';
        g.beginPath(); g.arc(x, y, kn.share < 0.999 || kn.stretched_s > 0 || kn.dropped ? 3.5 : 2, 0, 2 * Math.PI); g.fill();
      }
    }
  }
  // error lane
  if (r.ref) {
    let mx = 0.5;
    for (let i = Math.max(i0, Math.ceil(r.playAtMs)); i <= i1; i++) mx = Math.max(mx, Math.abs(r.plan[i] - r.ref[i]));
    g.fillStyle = '#666C78'; g.textAlign = 'left'; g.fillText(`error ±${fmt(mx, 2)} mm`, PAD.l + 4, errTop + 9);
    g.strokeStyle = '#1A1D24'; g.beginPath(); g.moveTo(PAD.l, (errTop + errBot) / 2); g.lineTo(W - PAD.r, (errTop + errBot) / 2); g.stroke();
    g.strokeStyle = '#FF5CB3'; g.lineWidth = 1; g.beginPath();
    let first = true;
    for (let i = i0; i <= i1; i += stride) { const x = xOf(i - r.playAtMs), y = (errTop + errBot) / 2 - (r.plan[i] - r.ref[i]) / mx * (errBot - errTop) / 2; if (first) { g.moveTo(x, y); first = false; } else g.lineTo(x, y); }
    g.stroke();
  }
  // bands: anomalies at the top, bundle arrivals at the bottom
  for (const [t, kind] of r.anomalies) {
    const tt = t - r.playAtMs;
    if (tt < st.dom[0] || tt > st.dom[1]) continue;
    g.fillStyle = ANOMALY_COLOR[kind] || '#E05BFF';
    g.fillRect(xOf(tt) - 1, 2, 2, 10);
  }
  g.fillStyle = '#2F7A3A';
  for (const b of r.bundles) {
    const tt = b.arriveMs - r.playAtMs;
    if (tt < st.dom[0] || tt > st.dom[1]) continue;
    g.fillRect(xOf(tt) - 0.5, laneBot + 2, 1, 8);
    if (b.dropped) { g.fillStyle = '#E05BFF'; g.fillRect(xOf(tt) - 1, laneBot + 2, 2, 8); g.fillStyle = '#2F7A3A'; }
  }
  // the knots on the wire (their starts), when zoomed in
  if (span < 12000) {
    g.fillStyle = '#3C8F49';
    for (const b of r.bundles) for (const x of b.recs) { const tt = x[3] / 1000 - r.playAtMs; if (tt >= st.dom[0] && tt <= st.dom[1]) g.fillRect(xOf(tt) - 0.5, laneBot + 10, 1, 3); }
  }
  g.restore();
  drawOverlay();
}
function drawOverlay(hoverX = null) {
  const g = overlay.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);
  const r = st.run;
  if (!r) return;
  const mark = (t, color) => { const x = xOf(t); if (x < PAD.l || x > W - PAD.r) return; g.strokeStyle = color; g.beginPath(); g.moveTo(x, PAD.t); g.lineTo(x, H - 22); g.stroke(); };
  if (st.cursorMs != null) mark(st.cursorMs, '#ECEFF4');
  if (hoverX != null) mark(tOf(hoverX), '#4A5060');
}
function readout(t) {
  const r = st.run;
  if (!r) return;
  const i = Math.round(t + r.playAtMs);
  if (i < 0 || i >= r.plan.length) { $('readout').textContent = ' '; return; }
  const parts = [`t ${ms(t)}`, `plan ${fmt(r.plan[i])} mm`];
  if (r.ref) parts.push(`curve ${fmt(r.ref[i])} mm`, `error ${fmt(r.plan[i] - r.ref[i])} mm`);
  parts.push(`v ${fmt(r.vel[i], 0)} mm/s`, `a ${fmt(r.acc[i], 0)} mm/s²`, `emitter ${fmt(r.position[i])} mm`);
  const f = r.flags[i], tags = [];
  for (const [n, b] of Object.entries(FLAG)) if (f & b) tags.push(n);
  if (tags.length) parts.push(tags.join(' '));
  $('readout').textContent = parts.join('   ');
}

// interaction
let drag = null;
overlay.addEventListener('wheel', (e) => {
  e.preventDefault();
  const t = tOf(e.offsetX), k = Math.exp(e.deltaY * 0.0015);
  const span = clamp((st.dom[1] - st.dom[0]) * k, 50, 3600000);
  const f = (t - st.dom[0]) / (st.dom[1] - st.dom[0]);
  st.dom = [t - f * span, t + (1 - f) * span];
  draw();
}, { passive: false });
overlay.addEventListener('pointerdown', (e) => { drag = { x: e.offsetX, dom: st.dom.slice(), moved: false }; overlay.setPointerCapture(e.pointerId); });
overlay.addEventListener('pointermove', (e) => {
  if (drag) {
    const dt = (e.offsetX - drag.x) / (W - PAD.l - PAD.r) * (drag.dom[1] - drag.dom[0]);
    if (Math.abs(e.offsetX - drag.x) > 2) drag.moved = true;
    st.dom = [drag.dom[0] - dt, drag.dom[1] - dt];
    draw();
  } else { drawOverlay(e.offsetX); readout(tOf(e.offsetX)); }
});
overlay.addEventListener('pointerup', (e) => {
  if (drag && !drag.moved) { st.cursorMs = tOf(e.offsetX); readout(st.cursorMs); drawOverlay(); }
  drag = null;
});
overlay.addEventListener('pointerleave', () => { if (!drag) drawOverlay(); });
document.addEventListener('keydown', (e) => {
  if (e.target && /^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName)) return;
  if (e.key === 'Home') { fit(); e.preventDefault(); }
  if ((e.key === '[' || e.key === ']') && st.events && st.events.length) {
    const cur = st.cursorMs ?? -Infinity;
    const next = e.key === ']' ? st.events.find((x) => x.t > cur + 0.5) : [...st.events].reverse().find((x) => x.t < cur - 0.5);
    if (next) { st.cursorMs = next.t; center(next.t); readout(next.t); }
  }
});
$('fit').addEventListener('click', fit);
$('pin').addEventListener('click', () => { if (st.run) { st.ghost = { plan: st.run.plan, playAtMs: st.run.playAtMs }; draw(); } });
$('unpin').addEventListener('click', () => { st.ghost = null; draw(); });
$('save').addEventListener('click', saveRecording);
for (const v of ['position', 'velocity', 'acceleration']) {
  const b = document.createElement('button');
  b.textContent = v;
  b.setAttribute('aria-pressed', String(v === st.view));
  b.addEventListener('click', () => { st.view = v; for (const x of $('view').children) x.setAttribute('aria-pressed', String(x === b)); draw(); });
  $('view').append(b);
}
new ResizeObserver(size).observe(box);

// ---- the controls ----------------------------------------------------------------------------------
fields($('machine'), MACHINE, st.machine, () => { st.machine.hi = Math.min(st.machine.hi, st.machine.rail); st.machine.lo = clamp(st.machine.lo, 0, st.machine.hi - 1); refill(MACHINE, st.machine); render(); });
fields($('tuning'), TUNE, st.tuning, render);
fields($('link'), LINK, st.link, render);
fields($('range'), RANGE, st.T, render);
fields($('gen'), GENF, st.gen, () => { if (st.gen.kind in GEN) useScript(generate(st.gen), st.gen.kind); });
$('defaults').addEventListener('click', () => { st.tuning = { ...st.defaults }; refill(TUNE, st.tuning); render(); });
mountInterp($('curve'), { value: st.interp, onChange: (v) => { st.interp = v; render(); } });
const sel = $('builtin');
for (const name of [...Object.keys(GEN), 'file']) { const o = document.createElement('option'); o.value = name; o.textContent = name; sel.append(o); }
sel.addEventListener('change', () => { if (sel.value in GEN) { st.gen.kind = sel.value; useScript(generate(st.gen), sel.value); } });
$('open').addEventListener('click', () => $('file').click());
$('file').addEventListener('change', () => { const f = $('file').files[0]; if (f) { sel.value = 'file'; loadFile(f); } $('file').value = ''; });
document.addEventListener('dragover', (e) => { e.preventDefault(); document.body.classList.add('drop'); });
document.addEventListener('dragleave', (e) => { if (!e.relatedTarget) document.body.classList.remove('drop'); });
document.addEventListener('drop', (e) => { e.preventDefault(); document.body.classList.remove('drop'); const f = e.dataTransfer.files[0]; if (f) { sel.value = 'file'; loadFile(f); } });

let statusTimer = 0;
function status(text, bad = false) {
  const el = $('status');
  el.textContent = text;
  el.style.borderColor = bad ? 'var(--bad)' : 'var(--line-4)';
  el.classList.add('show');
  clearTimeout(statusTimer);
  statusTimer = setTimeout(() => el.classList.remove('show'), bad ? 8000 : 2500);
  if (bad) console.error(text);
}

useScript(generate(st.gen), st.gen.kind);
size();
// For check.mjs.
window.__lab = { state: st, render, loadText, draw, fit };

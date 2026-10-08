// host.js -- the lab's worker: the board's own planner (tools/kinetic-wasm) under a virtual clock,
// fed by the funscript player's own scheduler or by a recording of what a hub received
// Constraints:
// - THE PLANNER IS THE BOARD'S (kinetic.wasm: MotionArbiter + Kinetic², the sources the P4 links) and
//   THE PLAYER IS PHOSPHOR'S (funscript.js, interp.js, scheduler.js, served verbatim from the Phosphor
//   checkout at /player/). This file is the host between them: the virtual clocks, the segments door
//   (a port of Phosphor src/model/motion.js submit.segments, restated because that module imports the
//   whole client), the hub's anchor and supersede rules (ValenceDevice::onStreamBundle) and the motion
//   task's wake order (ValenceMotion.cpp MotionTask::run). A planning rule written here is one the
//   machine does not run.
// - DETERMINISTIC: integer microsecond clocks, no wall clock, no randomness. The same request renders
//   the same bits.
// - Time: the player's wall ms and the hub's clock start together at 0; the client's hub-clock
//   estimate is off by link.clockErrMs; a bundle arrives link.netMs after it is sent; the hub wakes
//   on every arrival and on each 1 ms tick (the board's notify-or-tick loop).
// See: README.md

import { wire, shape, sample, cleanInterp } from '/player/interp.js';
import { createScheduler, PREROLL_MIN_MS, PREROLL_STROKE_MS, PREROLL_SKIP, OFFER_MAX } from '/player/scheduler.js';
import { posAt } from '/player/funscript.js';

/**
 * scheduler.js's surface (preroll, restart, tick) with the author's knots on the wire as they are: span k
 * starts at knot k-1's time, ends at knot k with the mode's own slope (the chords' mean without one; 0
 * at the ends and at a reversal), no handoff cap, no dwell merge.
 */
function exactScheduler(wired, T, submit, now) {
  const { at, pos, vel } = wired, n = at.length;
  const applyT = (p) => T.lo + (T.invert ? 1 - p : p) * (T.hi - T.lo);
  const scale = 1000 * (T.invert ? -1 : 1) * (T.hi - T.lo);
  const slope = (k) => {
    if (!(k > 0 && k < n - 1)) return 0;
    if (vel) return vel[k];
    const a = (pos[k] - pos[k - 1]) / (at[k] - at[k - 1]), b = (pos[k + 1] - pos[k]) / (at[k + 1] - at[k]);
    return a * b > 0 ? (a + b) / 2 : 0;
  };
  let cursor = 1;
  return {
    preroll(mediaMs, here) {
      const target = applyT(pos[0]), delta = Number.isFinite(here) ? Math.abs(here - target) : 1;
      if (Number.isFinite(here) && delta <= PREROLL_SKIP) return null;
      return { atMs: now(), norm: target, durationMs: PREROLL_MIN_MS + PREROLL_STROKE_MS * delta, endVel: 0 };
    },
    restart() { cursor = 1; },
    tick(clock) {
      const t = now(), list = [];
      while (cursor < n && clock.displayAt(at[cursor]) <= t) cursor++;
      for (let k = cursor; k < n && list.length < OFFER_MAX; k++)
        list.push({ atMs: clock.displayAt(at[k - 1]), norm: applyT(pos[k]), durationMs: at[k] - at[k - 1], endVel: slope(k) * scale });
      if (!list.length) return { ok: true, sent: 0 };
      const r = submit(list);
      cursor += r.sent;
      return r;
    },
  };
}

// ---- the ABI (tools/kinetic-wasm/kinetic_wasm.cpp) -----------------------------------------------
const SAMPLE = 64, KNOT = 40, TUNE = 32;
const S = { p: 8, v: 16, a: 24, plan_mm: 32, velocity_mm_s: 36, accel_mm_s2: 40, position_mm: 44, target_mm: 48,
  anomalies: 52, mode: 56, plan_kind: 57, flags: 58, plans: 60 };
// kinetic_tuning members the lab binds, by offset: f float32, u uint32.
export const TUNING = [['jmax_ovr', 0, 'f'], ['vmax_ovr', 4, 'f'], ['amax_ovr', 8, 'f'], ['chase_dense_us', 12, 'u'],
  ['react_us', 16, 'u'], ['smoothness', 20, 'f'], ['handle_floor', 24, 'f'], ['trim_max', 28, 'f']];

// ---- the registry numbers the door uses (Valence spec/registry/registry.yaml) ----------------------
const LIMITS = { bundle_max_samples: 32, min_transport_payload: 242, segment_t_off_unit_us: 100,
  segment_end_vel_unspecified: -32768, max_future_schedule_ms: 250 };
const BUNDLE_HEAD = 6, RECORD_BYTES = 6;      // 0x2101: u16 pos_e4, u16 dur_ms, i16 end_vel_e3
const SEG_FLOOR_MS = 10;                      // Phosphor motion.js: shorter is consumed, never packed
const LEAD_LOW_MS = 50;                       // scheduler.js: the low-latency offer cap
// RFC-059 schedule_latency_us of a segments grant: the motion tick (ValenceDevice::scheduleLatencyUs).
const SEGMENT_LATENCY_US = 1000;
const HUB_TICK_US = 1000;

const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
// Valence clients/js catalog.js: wire = round(physical * scale), half away from zero.
const wireInt = (x) => Math.sign(x) * Math.round(Math.abs(x));

let k = null, mem = null, dv = null;
const refresh = () => { if (dv === null || dv.buffer !== k.memory.buffer) { mem = new Uint8Array(k.memory.buffer); dv = new DataView(k.memory.buffer); } };
const cstr = (p) => { refresh(); let e = p; while (mem[e]) e++; return new TextDecoder().decode(mem.subarray(p, e)); };

function readTuning(ptr) {
  refresh();
  const out = {};
  for (const [name, off, type] of TUNING)
    out[name] = type === 'f' ? dv.getFloat32(ptr + off, true) : type === 'u' ? dv.getUint32(ptr + off, true) : dv.getUint8(ptr + off);
  return out;
}

function writeTuning(ptr, values) {
  k.kinetic_default_tuning(ptr);
  refresh();
  for (const [name, off, type] of TUNING) {
    const v = values[name];
    if (v == null || !Number.isFinite(v)) continue;
    if (type === 'f') dv.setFloat32(ptr + off, v, true);
    else if (type === 'u') dv.setUint32(ptr + off, Math.max(0, Math.round(v)), true);
    else dv.setUint8(ptr + off, clamp(Math.round(v), 0, 255));
  }
}

/** One 0x2101 record from a scheduler Seg {norm, durationMs, endVel}: [pos_e4, dur_ms, end_vel_e3]. */
function record(norm, durationMs, endVel) {
  const pos = wireInt(clamp(norm, 0, 1) * 10000);
  const dur = wireInt(clamp(durationMs || 0, 0, 65535));
  const vel = endVel == null ? LIMITS.segment_end_vel_unspecified : wireInt(clamp(endVel, -32.767, 32.767) * 1000);
  return [pos, dur, vel];
}

/**
 * Phosphor's segments door (src/model/motion.js submit.segments), one bundle per call: every start
 * clipped to the hub's latency, a short remainder consumed, stamps on a 100 us grid from the first,
 * the head within half the horizon (or the low-latency lead) and the payload. -> {bundle|null, sent}.
 */
function door(list, pnowMs, hubEstUs, horizonMs, low) {
  const lat = SEGMENT_LATENCY_US, unit = LIMITS.segment_t_off_unit_us;
  const packed = [];
  let s0 = null, lastOff = -1;
  for (let i = 0; i < list.length; i++) {
    const x = list[i];
    let e = hubEstUs + (x.atMs - pnowMs) * 1000;
    const end = e + x.durationMs * 1000;
    if (e < hubEstUs + lat) e = hubEstUs + lat;
    if (end - e < SEG_FLOOR_MS * 1000) continue;
    const stamp = e - lat;
    if (s0 === null) s0 = Math.round(stamp);
    const off = Math.round((stamp - s0) / unit) * unit;
    if (off <= lastOff) continue;
    lastOff = off;
    packed.push({ off, rec: record(x.norm, (end - e) / 1000, x.endVel), i });
  }
  const maxN = Math.min(LIMITS.bundle_max_samples, Math.floor((LIMITS.min_transport_payload - BUNDLE_HEAD) / (2 + RECORD_BYTES)));
  const until = hubEstUs + (low ? LEAD_LOW_MS : horizonMs / 2) * 1000;
  let n = 0;
  while (n < packed.length && n < maxN && s0 + packed[n].off <= until) n++;
  if (!n) return { bundle: null, sent: 0 };
  return { bundle: { tBaseUs: s0, offs: packed.slice(0, n).map((x) => x.off), recs: packed.slice(0, n).map((x) => x.rec) },
    sent: packed[n - 1].i + 1 };
}

/** The machine for one run: limits, window, tuning. */
function machine(req) {
  const m = req.machine;
  const h = k.kinetic_create(m.vmax, m.amax, m.jmax, m.rail, m.horizonMs || 0);
  if (!h) throw new Error('limits refused (every limit must be finite and positive)');
  if (k.kinetic_set_window(h, m.window[0], m.window[1]) !== 1) { k.kinetic_destroy(h); throw new Error('window refused (0 <= lo < hi <= rail)'); }
  const tb = k.malloc(TUNE);
  writeTuning(tb, req.tuning || {});
  k.kinetic_set_tuning(h, tb);
  k.free(tb);
  return h;
}

/**
 * Run one request. source.kind 'script': {script: {at, pos}, interp, T, spanMm}; 'events': {events}
 * in the recording shape [arrivalMs, 'seg', pos_e4, dur_ms, end_vel_e3, start_us, supersede]
 * | [ms, 'manual', target_mm] | [ms, 'tune', {member: value}]. Yields between chunks.
 */
function* run(req) {
  const h = machine(req);
  const out = k.malloc(SAMPLE), kb = k.malloc(KNOT), tb = k.malloc(TUNE);
  const link = req.link, m = req.machine;
  const horizonMs = m.horizonMs || LIMITS.max_future_schedule_ms;
  const netUs = Math.round((link.netMs || 0) * 1000), clockErrUs = Math.round((link.clockErrMs || 0) * 1000);
  const queue = [];        // bundles in flight, by arrival: {sendUs, arriveUs, tBaseUs, offs, recs}
  const bundles = [];      // what the hub received: the recording
  const events = [];       // 'events' kind: the recording, by arrival
  const knotsLog = [];     // per delivery: {atMs, knots: [...]}
  const anomalies = [];    // [tMs, kind]
  let endUs = 0, playAtMs = 0, tHub = 0;
  let accepted = 0, dropped = 0, refused = 0;

  // ---- the player (kind 'script') ----
  let sch = null, clock = null, playerNextUs = Infinity, prerollDone = false, playing = false;
  const tickUs = Math.max(1000, Math.round((link.playerTickMs || 1000 / 60) * 1000));
  let vnowMs = 0;
  if (req.source.kind === 'script') {
    const src = req.source, T = { offsetMs: 0, lo: 0, hi: 1, invert: false, ...src.T };
    const script = { ...src.script, at: Float64Array.from(src.script.at), pos: Float32Array.from(src.script.pos), ignored: [], notes: [] };
    const wired = wire(script, src.interp, { spanMm: src.spanMm, lo: T.lo, hi: T.hi });
    const submit = (list) => {
      const r = door(list, vnowMs, Math.round(vnowMs * 1000) + clockErrUs, horizonMs, !!link.low);
      if (r.bundle) {
        const sendUs = Math.round(vnowMs * 1000);
        queue.push({ ...r.bundle, sendUs, arriveUs: sendUs + netUs });
      }
      return { ok: true, sent: r.sent, rateHz: 100 };
    };
    if (link.exact) {
      // The author's curve on the wire, exactly: one segment per action, the mode's own slope as the end
      // velocity (uncapped), nothing merged. The same door and timing as the scheduler; the player's
      // knotSlope cap and dwellMerge are what this bypasses, so the planner's own fidelity shows alone.
      sch = exactScheduler(wired, T, submit, () => vnowMs);
    } else {
      sch = createScheduler({ submit, now: () => vnowMs, log: () => {} });
      sch.setTransform(T);
      sch.load(wired);
    }
    playerNextUs = 0;
    endUs = Infinity;   // set once the preroll places playAt
    // The reference curve the player draws (shape: the mode, smoothing, slew, scale), in mm.
    const I = cleanInterp(src.interp), filtered = I.smoothMs > 0 || (I.slewMmS > 0 && src.spanMm > 0);
    const shaped = shape(script, src.interp, { spanMm: src.spanMm, lo: T.lo, hi: T.hi }) || script;
    // The curve itself (interp.js sample: the mode's value, exact) unless a filter is on, when only the
    // shaped knots (a 40 ms resample) carry it. sample() is unscaled; the scale moves the actions first,
    // as wire() and shape() do.
    const scaled = I.scale === 1 ? script : { ...script, pos: Float32Array.from(script.pos, (p) => 0.5 + (p - 0.5) * I.scale) };
    const curve = filtered ? (t) => posAt(shaped, t) : (t) => sample(scaled, I, t);
    req._ref = { curve, T, wired, durationMs: script.at[script.at.length - 1] };
  } else {
    for (const e of req.source.events) events.push(e);
    events.sort((a, b) => a[0] - b[0]);
    endUs = Math.round(((events.length ? events[events.length - 1][0] : 0) + 2000) * 1000);
  }

  const lo = m.window[0], span = m.window[1] - m.window[0];
  // Grid samples, index = engine ms: plan_mm, position_mm, vel, acc, raw p, flags, plan kind.
  const planA = [], posA = [], velA = [], accA = [], rawA = [], flagA = [], kindA = [];

  const deliverSeg = (tUs, pos_e4, dur_ms, vel_e3, stampUs, supersede) => {
    // ValenceDevice::onStreamBundle: delta against the hub's clock, clamped to the lead cap and to now.
    let delta = stampUs - tUs;
    if (delta > horizonMs * 1000) delta = horizonMs * 1000;
    if (delta < 0) delta = 0;
    const anchor = tUs + delta;
    const r = k.kinetic_submit_segment2(h, pos_e4, dur_ms, vel_e3, anchor, supersede ? 1 : 0);
    if (r === 1) accepted++; else if (r === 0) refused++; else dropped++;
    return { r, anchor };
  };

  const snapshotKnots = (tUs) => {
    const n = k.kinetic_pending(h), knots = [];
    for (let i = 0; i < n && i < 64; i++) {
      if (!k.kinetic_solved(h, i, kb)) break;
      refresh();
      knots.push({ t_us: dv.getFloat64(kb, true), p: dv.getFloat32(kb + 8, true), v: dv.getFloat32(kb + 12, true), a: dv.getFloat32(kb + 16, true),
        share: dv.getFloat32(kb + 20, true), stretched_s: dv.getFloat32(kb + 24, true), worst: dv.getFloat32(kb + 28, true),
        dropped: mem[kb + 32], clamped: mem[kb + 33], pin_v: mem[kb + 34], pin_a: mem[kb + 35] });
    }
    knotsLog.push({ atMs: tUs / 1000, knots });
  };

  const evaluate = (tUs, grid) => {
    k.kinetic_evaluate(h, out);
    refresh();
    const mask = dv.getUint32(out + S.anomalies, true);
    for (let a = mask; a; a &= a - 1) anomalies.push([tUs / 1000, 31 - Math.clz32(a & -a)]);
    if (grid) {
      planA.push(dv.getFloat32(out + S.plan_mm, true)); posA.push(dv.getFloat32(out + S.position_mm, true));
      velA.push(dv.getFloat32(out + S.velocity_mm_s, true)); accA.push(dv.getFloat32(out + S.accel_mm_s2, true));
      rawA.push(dv.getFloat64(out + S.p, true)); flagA.push(mem[out + S.flags]); kindA.push(mem[out + S.plan_kind]);
    }
  };

  // t = 0 is the first sample (slot 0): the machine at rest, where the player's preroll starts from.
  evaluate(0, true);
  const hereNorm = (posA[0] - lo) / span;

  let ev = 0, chunk = 0;
  try {
  for (;;) {
    // The tick grid is the RTOS tick, whole milliseconds; a wake on arrival never moves it.
    const nextGrid = (Math.floor(tHub / HUB_TICK_US) + 1) * HUB_TICK_US;
    const nextArr = queue.length ? queue[0].arriveUs : Infinity;
    const nextEv = ev < events.length ? Math.round(events[ev][0] * 1000) : Infinity;
    const t = Math.min(nextGrid, playerNextUs, nextArr, nextEv);
    if (t > endUs) break;

    // The player's tick: the scheduler offers what starts within the horizon; bundles join the queue.
    if (t === playerNextUs) {
      vnowMs = t / 1000;
      if (!prerollDone) {
        prerollDone = true;
        const pre = sch.preroll(0, hereNorm);
        if (pre) { const r = door([pre], vnowMs, t + clockErrUs, horizonMs, false); if (r.bundle) queue.push({ ...r.bundle, sendUs: t, arriveUs: t + netUs }); playAtMs = pre.atMs + pre.durationMs; }
        else playAtMs = vnowMs;
        clock = { ready: true, rate: 1, displayAt: (mm) => playAtMs + mm, mediaAt: (d) => d - playAtMs };
        endUs = Math.round((playAtMs + req._ref.durationMs + 1500) * 1000);
        playerNextUs = Math.round(playAtMs * 1000);
      } else {
        if (!playing) { playing = true; sch.restart(clock, 0); }
        sch.tick(clock);
        playerNextUs = t + tickUs;
      }
      queue.sort((a, b) => a.arriveUs - b.arriveUs);
    }

    const arrivals = [];
    while (queue.length && queue[0].arriveUs <= t) arrivals.push(queue.shift());
    const due = [];
    while (ev < events.length && Math.round(events[ev][0] * 1000) <= t) due.push(events[ev++]);
    const grid = t === nextGrid;
    if (!arrivals.length && !due.length && !grid) continue;

    // The hub's wake: the clock, the intents that woke it, the tick.
    if (t > tHub) { k.kinetic_advance(h, (t - tHub) / 1e6); tHub = t; }
    let woke = false;
    for (const b of arrivals) {
      let flushed = false, acc = 0, drop = 0;
      const recs = [];
      for (let i = 0; i < b.recs.length; i++) {
        const [pos, dur, vel] = b.recs[i];
        const { r, anchor } = deliverSeg(t, pos, dur, vel, b.tBaseUs + b.offs[i], !flushed);
        if (r === 1) { flushed = true; acc++; } else drop++;
        recs.push([pos, dur, vel, anchor, r === 1 && acc === 1 ? 1 : 0]);
      }
      bundles.push({ sendMs: b.sendUs / 1000, arriveMs: t / 1000, tBaseUs: b.tBaseUs, n: b.recs.length, accepted: acc, dropped: drop, recs });
      woke = true;
    }
    for (const e of due) {
      if (e[1] === 'seg') { const { r } = deliverSeg(t, e[2], e[3], e[4], e[5], !!e[6]); bundles.push({ sendMs: NaN, arriveMs: t / 1000, tBaseUs: e[5], n: 1, accepted: r === 1 ? 1 : 0, dropped: r === 1 ? 0 : 1, recs: [[e[2], e[3], e[4], e[5], e[6] ? 1 : 0]] }); }
      else if (e[1] === 'manual') { if (k.kinetic_submit_manual(h, e[2]) === 1) accepted++; else refused++; }
      else if (e[1] === 'tune') { writeTuning(tb, { ...(req.tuning || {}), ...e[2] }); k.kinetic_set_tuning(h, tb); }
      woke = true;
    }
    evaluate(t, grid);
    if (woke) snapshotKnots(t);
    if (grid && (++chunk & 8191) === 0) yield t;
  }
  } finally {
    k.free(out); k.free(kb); k.free(tb);
    k.kinetic_destroy(h);
  }

  // The reference curve on the same grid, mm: the player's shaped line through the range.
  let ref = null;
  const capped = [];   // end velocities the scheduler sent below the curve's own slope (its handoff bound)
  if (req._ref) {
    const { curve, T, wired } = req._ref;
    ref = new Float32Array(planA.length);
    for (let i = 0; i < ref.length; i++) {
      const media = i - playAtMs;
      const n = clamp(curve(media), 0, 1);
      ref[i] = lo + (T.lo + (T.invert ? 1 - n : n) * (T.hi - T.lo)) * span;
    }
    // What the curve's slope would have been on the wire at each knot the hub received (scheduler.js
    // wireVel without knotSlope's cap), against what was sent.
    if (wired.vel) {
      const sign = T.invert ? -1 : 1, scale = 1000 * (T.hi - T.lo);
      for (const b of bundles) for (const x of b.recs) {
        if (x[2] === LIMITS.segment_end_vel_unspecified) continue;
        const endMs = x[3] / 1000 + x[1] - playAtMs;
        let j = -1;
        for (let a = 0, z = wired.at.length - 1; a <= z;) { const m = (a + z) >> 1; if (wired.at[m] < endMs - 1.5) a = m + 1; else if (wired.at[m] > endMs + 1.5) z = m - 1; else { j = m; break; } }
        if (j < 0) continue;
        const want = wireInt(clamp(wired.vel[j] * scale * sign, -32.767, 32.767) * 1000);
        if (Math.abs(x[2]) < Math.abs(want) - 1 && Math.sign(x[2]) === Math.sign(want)) capped.push([endMs, x[2] / 1000 * span, want / 1000 * span]);
      }
    }
  }
  return { plan: Float32Array.from(planA), position: Float32Array.from(posA), vel: Float32Array.from(velA), acc: Float32Array.from(accA),
    raw: Float32Array.from(rawA), flags: Uint8Array.from(flagA), kind: Uint8Array.from(kindA), ref, playAtMs, endMs: endUs / 1000,
    anomalies, bundles, knotsLog, capped, accepted, dropped, refused, window: m.window.slice() };
}

let latest = 0;
onmessage = async (e) => {
  const q = e.data;
  if (q.init) {
    try {
      const bytes = await (await fetch(q.init.wasmUrl)).arrayBuffer();
      const inst = (await WebAssembly.instantiate(bytes, {})).instance.exports;
      inst._initialize();
      k = inst;
      const v = cstr(k.kinetic_version());
      if (!/ kinetic2 /.test(v)) throw new Error('not a Kinetic² build: ' + v);
      const tb = k.malloc(TUNE);
      k.kinetic_default_tuning(tb);
      const defaults = readTuning(tb);
      k.free(tb);
      postMessage({ ready: v, defaults });
    } catch (err) { postMessage({ error: String(err && err.message || err) }); }
    return;
  }
  latest = q.id;
  if (!k) { postMessage({ id: q.id, error: 'kinetic.wasm not ready' }); return; }
  const t0 = performance.now();
  try {
    const it = run(q.req);
    for (;;) {
      const r = it.next();
      if (r.done) {
        const v = r.value;
        postMessage({ id: q.id, ms: performance.now() - t0, ...v },
          [v.plan.buffer, v.position.buffer, v.vel.buffer, v.acc.buffer, v.raw.buffer, v.flags.buffer, v.kind.buffer, ...(v.ref ? [v.ref.buffer] : [])]);
        return;
      }
      await new Promise((go) => setTimeout(go));
      if (latest !== q.id) { it.return(); return; }
    }
  } catch (err) { postMessage({ id: q.id, error: String(err && err.stack || err) }); }
};

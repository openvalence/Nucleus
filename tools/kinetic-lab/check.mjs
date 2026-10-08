// check.mjs -- the lab's smoke test: serves the page, drives it headless, and asserts the run shape.
//
//   node tools/kinetic-lab/check.mjs
//
// Needs the twin built (tools/kinetic-wasm/build/kinetic.wasm) and Playwright with Chromium
// (PLAYWRIGHT_DIR, default ../Phosphor/node_modules). SHOTS_DIR takes the screenshots (default: temp).
// Asserts: zero page errors; the built-in sine renders with nothing dropped or refused; a saved
// recording replays to the same plan bit for bit (the twin's determinism through the lab's loop);
// the Nucleus trace fixture loads as a recording and accepts what the native run accepted.
import { createRequire } from 'node:module';
import { mkdir } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { serve } from './serve.mjs';

const here = fileURLToPath(new URL('.', import.meta.url));
const pwDir = process.env.PLAYWRIGHT_DIR || join(here, '..', '..', '..', 'Phosphor', 'node_modules');
const { chromium } = createRequire(join(pwDir, 'x.js'))('playwright');
const shots = process.env.SHOTS_DIR || join(tmpdir(), 'kinetic-lab');
await mkdir(shots, { recursive: true });

const { server, url } = await serve(0);
const browser = await chromium.launch();
const page = await browser.newPage({ viewport: { width: 1400, height: 950 } });
const errors = [];
page.on('pageerror', (e) => errors.push('pageerror: ' + e.message));
page.on('console', (m) => { if (m.type() === 'error') errors.push('console: ' + m.text()); });
let failed = 0;
const check = (ok, what) => { console.log((ok ? 'ok   ' : 'FAIL ') + what); if (!ok) failed++; };

const seqNow = () => page.evaluate(() => (window.__lab.state.run ? window.__lab.state.run.seq : 0));
const rendered = async (prev) => {
  await page.waitForFunction((p) => { const s = window.__lab.state; return s.run && s.run.seq > p && document.getElementById('runstate').textContent.startsWith('rendered'); }, prev, { timeout: 120000 });
  return page.evaluate(() => {
    const r = window.__lab.state.run, dl = [...document.querySelectorAll('#stats dd')].map((d) => d.textContent);
    return { accepted: r.accepted, dropped: r.dropped, refused: r.refused, bundles: r.bundles.length, anomalies: r.anomalies.length, ticks: r.plan.length,
      playAtMs: r.playAtMs, stats: dl, version: document.getElementById('version').textContent };
  });
};

await page.goto(url);
const a = await rendered(0);
console.log('twin:', a.version);
console.log('sine (pchip, factory):', JSON.stringify({ ...a, stats: undefined }));
for (const s of a.stats) console.log('   ', s);
check(/ kinetic2 /.test(a.version), 'the twin is a Kinetic² build');
check(a.ticks > 20000 && a.bundles > 100, 'the sine ran through the player: ' + a.bundles + ' bundles over ' + a.ticks + ' ms');
check(a.dropped === 0 && a.refused === 0, 'nothing dropped or refused');
// One sample per millisecond of engine time: a wake between ticks never costs a sample.
const samples = await page.evaluate(() => [window.__lab.state.run.plan.length, window.__lab.state.run.endMs]);
check(samples[0] === Math.round(samples[1]) + 1, 'one sample per ms: ' + samples[0] + ' samples over ' + samples[1] + ' ms');
await page.screenshot({ path: join(shots, 'lab-sine.png') });

// Zoom in around 5 s and shoot the knots.
await page.evaluate(() => { const s = window.__lab.state; s.dom = [4000, 7000]; s.cursorMs = 5000; window.__lab.draw(); });
await page.screenshot({ path: join(shots, 'lab-sine-zoom.png') });

// The recording round trip: the events the hub saw, replayed without the player, plan the same bits.
const plan1 = await page.evaluate(() => Array.from(window.__lab.state.run.plan));
const s1 = await seqNow();
await page.evaluate(() => {
  const r = window.__lab.state.run, s = window.__lab.state, events = [];
  for (const b of r.bundles) for (const x of b.recs) events.push([+b.arriveMs.toFixed(3), 'seg', ...x]);
  window.__lab.loadText(JSON.stringify({ events, machine: { ...s.machine }, link: { ...s.link }, tuning: { ...s.tuning } }), 'roundtrip.recording.json');
});
const b = await rendered(s1);
const plan2 = await page.evaluate(() => Array.from(window.__lab.state.run.plan));
// The replay's tail is its own (last event + 2 s); the live run's is the script's end + 1.5 s.
const common = Math.min(plan1.length, plan2.length);
let firstDiff = -1;
for (let i = 0; i < common; i++) if (plan1[i] !== plan2[i]) { firstDiff = i; break; }
check(b.accepted === a.accepted, 'the recording accepts what the live run accepted (' + b.accepted + ')');
check(firstDiff < 0 && common >= plan1.length - 2500, 'the recording replays the plan bit for bit over ' + common + ' ms' + (firstDiff >= 0 ? ' (first difference at ' + firstDiff + ' ms)' : ''));

// The Nucleus trace fixture as a recording.
const fx = await (await fetch(url + 'fixtures/kinetic_trace.json')).json();
const s2 = await seqNow();
await page.evaluate((text) => window.__lab.loadText(text, 'kinetic_trace.json'), JSON.stringify(fx));
const c = await rendered(s2);
console.log('fixture:', JSON.stringify({ ...c, stats: undefined }));
check(c.accepted === fx.summary.accepted, 'the fixture accepts ' + fx.summary.accepted + ' segments through the lab (' + c.accepted + ')');
await page.screenshot({ path: join(shots, 'lab-fixture.png') });

check(errors.length === 0, 'no page errors' + (errors.length ? ': ' + errors.join(' | ') : ''));
await browser.close();
server.close();
console.log('screenshots in ' + shots);
console.log(failed ? 'FAIL' : 'PASS');
process.exit(failed ? 1 : 0);

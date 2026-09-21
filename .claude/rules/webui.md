---
paths:
  - "**"
---

# WebUI integration, build chain, and render traps

Carried from the archived machine repo (SlopDrive-32
`.claude/rules/webui.md`). Phosphor is unchanged: the design home is
`docs/phosphor/DESIGN.md` in whichever repo hosts it, and the Prime Rule
stands -- through the protocol, never around it.
The `webui/` tree itself lands with the UI phase; these rules bind it the day
it does.
# WebUI integration, build chain, and render traps

## Build chain

- **Compile-time asset bundling.** Web assets are an independent front-end
  project (Vite) built via PlatformIO extra scripts, never C++ strings.
  Outputs are minified and gzipped into the LittleFS image.
- **API-driven modularity.** The UI loads capability and config data
  dynamically and builds settings cards from what the firmware advertises.

## Ground Truth Doctrine (NON-NEGOTIABLE)

The UI must NEVER display machine state that differs from the device's, in
either direction.

- Page load ADOPTS device state.
- Echoes report APPLIED, post-clamp values.
- Optimistic UI state is prohibited: shadow desired/reported, pending until
  echo-confirmed.
- Every new or modified control is verified end to end against the live device
  before it is done. A control that renders but drives nothing is a defect; a
  UI that lies about machine state is a SAFETY defect on this product.

## House look is ground truth (operator ruling 2026-07-29)

`webui/src/style.css` IS the visual language: tokens, type ramp, accent
semantics. Not restated here (C-1); read that file's header. Three properties
are load-bearing rather than decorative:

- The neutral chassis is FROZEN. New surfaces consume the tokens; they do not
  introduce a parallel palette or a second type ramp.
- `--warn` amber and `--bad` red are identical in EVERY theme, on purpose.
  Hazard styling must read the same however the operator dresses the
  instrument. Restyling safety colors for aesthetic reasons is a safety
  defect, not a design change.
- Only the two accent hues vary by theme.

Generative design tooling is for surfaces with no precedent yet (new Phosphor
widgets, docs-site). On the existing webui it consumes this file; it does not
re-litigate it. Aesthetic drift already cost one CSS drift audit and one
OG-realignment pass.

## T18 -- arrival-time stamping destroys a stream's timeline

**Rule:** never stamp streamed samples with local receive time and then
interpolate or derive against those stamps. Reconstruct the SOURCE's timeline
(re-space by the known or estimated production cadence, future-anchored), or
carry source timestamps on the wire. Treat arrival time as a hint only.
**Mechanism:** the network batches. TCP clumps several STATE frames into one
segment, so decode-time `Date.now()` gives them IDENTICAL stamps followed by a
gap. Measured on-device: 71 of 393 motion samples in 12 s carried a duplicate
stamp, p95 arrival gap 90 ms against a ~30 ms true period. Downstream a
not-newer-than-the-last guard silently DISCARDED every duplicate, about 18% of
all motion, and the interpolator played the missing span in one frame (snap)
then starved to the next clump (freeze): 107 snap frames and 17 multi-frame
freezes in 719 rendered. No interpolation upgrade survives garbage timestamps.
A Hermite pass shipped first and changed nothing visible, which is itself the
diagnostic: when smoothing math does not help, question the time base, not the
curve.
**Corollary:** any cadence ESTIMATOR feeding the reconstruction must learn
only from plausible streaming gaps. Idle, shed and dwell gaps are mode
switches, and one 600 ms gap taught the estimator a garbage period whose first
post-resume spans rendered as a one-tick wrong position.
**Fix:** `webui/src/ui/hero/telebuf.js` `push()` reconstructs timestamps
(max of arrival plus lead and prev plus EMA period, capped, monotonic, never
dropping a sample); EMA gated to gaps under min(4x period, 200 ms); a gap over
500 ms resyncs the schedule. Regression tests in
`webui/test/telebuf-sim.mjs`.

## T22 -- a sticky offset is measured from a DIFFERENT box depending on who scrolls

**Rule:** when fixed chrome reserves its height as a container's padding, the
sticky bars inside that container must NOT restate the reserve as their own
`top`, unless the page rather than the container is the scrollport. The
correct offset is not a property of the layout, it is a property of WHICH
ELEMENT SCROLLS, and this codebase has one of each at the 960px breakpoint.
**Mechanism:** a sticky element's offset resolves against the nearest
scrollport INSET BY that scroll container's padding (CSS Position §6.3).
Mobile: the page scrolls, the scrollport is the viewport, its padding is zero,
so `top: <chrome>` parks the bar correctly below the chrome. Desktop: `.app`
is the scroll container (height-capped flex column, `overflow: hidden`, which
still establishes a scrollport), so the constraint rectangle already starts at
`.app`'s CONTENT box, below the padding that reserved the chrome. The same
`top: <chrome>` insets a second time and the bar lands at exactly twice the
chrome height. Both modes read as correct in code review; only one is.
**Fix:** one reserve (`.app` padding-top), chrome pinned at `top: 0`, and the
sticky offset restated ONLY in the mode where the page is the scrollport.
`webui/test/shell-chrome-geometry.test.mjs` asserts flush stacking in both
modes with no device present.
**Companion:** exactly one bar may absorb `env(safe-area-inset-top)`. The
topmost one owns it via `--chrome-inset-top`; two bars padding for the same
notch is the same double-gap bug wearing a phone.

## T23 -- a synchronous read inside an effect is a SUBSCRIPTION; the async callback next to it is not

**Rule:** in a reactive effect that installs a timer or observer and keeps
state across ticks, every reactive value the effect body touches
SYNCHRONOUSLY must be read through `untrack()`. Seeding a local from reactive
state is a subscription, and re-running the effect re-runs its initializers.
**Mechanism:** Svelte 5 records dependencies during the effect's synchronous
execution. Reads from a `setInterval`, `ResizeObserver` or
`requestAnimationFrame` callback happen outside that window and register
nothing. So the two halves of the same function behave oppositely: `tick()`
called once at the bottom of the effect subscribes to everything it touches,
while the identical `tick()` fired by the interval subscribes to nothing. When
the subscribed value updates at telemetry rate the effect tears down and
re-runs tens of times a second, re-executing `let data = []` and every buffer
fill above it. The timer keeps running and the drawing keeps happening, so the
feature looks ALIVE. Only the accumulated history is gone.
**Bit us:** LinkBar's activity heatmap. Two synchronous reads leaked. At
~25 Hz the 14-column history was refilled with zeros before it could fill, so
every column but the newest painted empty. Read as "the grid does not scroll
left", and the scroll was never the broken part.
**Fix:** `untrack()` on both reads. The guard counts columns sitting at the
v=0 baseline alpha and fails above 2, because the bug's signature is 13 of 14
empty and any threshold on "distinct values" would have passed the broken
version.

## T24 -- a control's own box is not the box you can see

**Rule:** never derive geometry from an interactive element's rect without
checking what the user agent and the design system did to that rect first.
Measure the rendered box; do not reason about it from the stylesheet.
**Mechanism:** two independent ways the box lies. (1) The UA gives `button` a
default `padding: 1px 6px`. Under `box-sizing: border-box` on a small fixed
control that padding is subtracted from the INSIDE: an 18px button keeps a
4px-wide content box. A grid item wider than its own track cannot be centered
in it, so alignment resolves to start and the item is pinned to the content
edge, overflowing one side only. `place-items: center` is then present,
correct-looking, and doing nothing. (2) A styled `input[type=range]` is
`height: 2px`, so its box is the hairline TRACK. The thumb is
`::-webkit-slider-thumb`, a pseudo-element that overflows the box entirely, so
the input's rect excludes the very part the operator grabs.
**Fix:** `padding: 0` on the icon button; the echo expands by half
`--slider-thumb-h`, which style.css defines ONCE and the thumb rules read, so
the outline and the thing it encloses cannot drift. Guards assert a non-zero
box BEFORE trusting an offset: a `display: none` element reports an all-zero
rect, which had made the centering assertion pass while measuring nothing.

## T25 -- an unregistered custom property animates as a discrete swap

**Rule:** a custom property driving an animation or transition must be
declared with `@property` and a real `syntax`, never `*`.
**Mechanism:** CSS custom properties are substituted as raw token streams.
With no registered `syntax` the engine cannot know `0%` and `150%` are
lengths, so it falls back to discrete interpolation: the value flips at the
keyframe boundary instead of sweeping. Registration gives it a type, an
initial value, and `inherits: false`, after which it interpolates like any
other animatable length.
**Bit us:** the intent echo's wavefront radius. The failure mode is the
dangerous kind: the animation still plays, the timing is right, and the shape
jumps from nothing to fully expanded, which at 500 ms reads as a slightly
janky pulse rather than as a bug with a name.
**Fix:** `@property --pr { syntax: '<percentage>'; inherits: false;
initial-value: 0% }` in style.css. The guard samples the property MID-FLIGHT
and requires a real intermediate radius; asserting only that the animation is
running would have passed the broken version.

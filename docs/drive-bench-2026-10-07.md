# Drive bench 2026-10-07: the AIM drive through the P4, unloaded and loaded

What the 57AIM30 drive and motor do when the P4 emits quadrature at them, and
what the drive's own registers do, measured on a bare, clamped motor at 36 V.
The numbers are the unloaded ceiling; the machine's 60AIM40F with a carriage
and a toy is measured in the last section (2026-10-08). Firmware consequences,
the factory input set ruled 2026-10-08 (val-sgj) among them, come before the
ladder; the open decisions are on the board (val-d66, val-25a).

## Setup

- 57AIM30 (the 1500 rpm winding), bare shaft, clamped, 36 V bus, programmed to
  the machine's registers: gear 4/1 (8192 pulses per rev), 0x02 1500, 0x03
  60000, 0x04 495, 0x05 3000, 0x06 10, 0x07 3000, 0x08 3900, 0x18 600, 0x19 2.
- P4 devkit, 0.1.22-p4hub-bench (Kinetic c3af1ed), quadrature out on LPG10 and
  LPG12. Five wires to the P4: 5 V, GND, A, B, home interrupt. The hub is
  output only: its reported position is the LP core's count of the edges it
  emitted, never the motor. 208.608 edges per mm, 39.27 mm per rev.
- The drive's RS485 goes to the PC (COM2, 19200 8N1, slave 1). The motor's
  truth is the drive's own encoder (0x16/0x17, 32768 counts per rev) read
  before and after each move, and polled at 75 to 90 Hz during moves.
- The machine's 60AIM40F answers at 115200, the bench 57AIM30 at 19200; the
  tools/aim scripts hunt both unless --baud is given.
- Moves came two ways: hub jogs (0x3100 point moves at jog_speed and
  jog_accel, a clean trapezoid), and the drive's own moves over Modbus
  (FC 0x10 delta to 0x0C/0x0D with 0x00 armed), which take the hub out
  entirely.

## Speed

| wall | rpm | mm/s | set by | moved by |
|---|---|---|---|---|
| drive speed register 0x02 = 1500 | 1530 | 1000 | one register, same value on the machine's drive | writing 0x02 (max 3200) and saving |
| this winding at 36 V | 2150 to 2300 | 1410 to 1510 | back-EMF; current doubles and it sounds ragged | a different winding or more bus voltage |
| LP emitter | 3360 | about 2200 | the LP core sustains about 460 kHz of edges | firmware: MAX_SPEED_MM_S now 2000 |
| drive pulse input | 3670 | 2400 | datasheet 500 kHz; took 626 kHz bursts without loss | gear 8/1 halves the rate |

- 0x02 is a real clamp: with it at 1500 the encoder cruises 930 to 1020 mm/s,
  at 2000 exactly 2038 rpm, at 3000 the motor stops at 2150 to 2300 rpm with
  the current register at 530 to 976 against 270 at 1500. 4000 and 5000 read
  back 3200.
- The drive never dropped a pulse. After every burst, including 3000 mm/s
  commanded (626 kHz), the encoder landed on the hub's emitted count to
  0.01 mm. The drive buffers the pulses as position error and the motor works
  it off at its clamp.
- The LP emitter falls behind a 3000 mm/s plan: about 450 of 600 mm emitted
  when the plan ends. Once the plan's velocity is zero the only thing moving
  the count is the arbiter's residual term, capped at input_accel times one
  millisecond (100 mm/s at 100000), so the last 160 mm crawl. That is the
  "fast then crawl" the operator heard. val-d66.
- Polling the drive over Modbus at 10 to 40 Hz during motion changed nothing
  in the pulse following.

## Acceleration

The motor is not the limit. During a 16,000 mm/s2 ramp the current register
reads about 300; when the position loop was pushed into oscillation it read
16,000. The drive's position loop is the limit, and its registers run out
before the motor does. Hub jogs at 1000 mm/s over 600 mm, 100,000 mm/s2 asked:

| drive setting | accel mm/s2 (rpm/s) | decel mm/s2 | following error p95 | overshoot | note |
|---|---|---|---|---|---|
| as programmed (KP 3000, 0x03 60000) | 16,000 (25,000) | 12,000 | 10 mm | 0.02 mm | soft |
| 0x03 50000 | lag 18 ms | | 26 mm | | the ramp field binds on pulse following |
| 0x07 KP 8000 | 27,700 (42,000) | 21,000 | 12 mm | 0.29 mm | clean |
| 0x07 KP 12000 | 27,700 | 32,700 | 15 mm | 0.61 mm | clean, current peaks 1500 |
| 0x07 KP 20000 | 10,000 | 16,000 | 15 mm | 4.4 mm | oscillates, 1.2 mm jitter at rest |
| 0x03 60060 (FF 60) | 22,500 (34,000) | 13,000 | 11 mm | 0.02 mm | leads the pulses by 32 mm |
| 0x03 60098 (FF max) | 30,000 (46,000) | 22,500 | 18 mm | 0.00 mm | leads by 34 mm, lands 20 ms early |
| 0x03 60098 + KP 5000 | 33,000 (50,000) | 24,000 | 17 mm | 0.13 mm | clean |
| 0x05 speed KP 6000 / 10000 | 17,000 | 12,000 | 12 mm | 0.02 mm | no accel change, less lead |
| 0x06 KI 3 | 12,000 | 11,000 | 18 mm | 1.0 mm | unstable |
| 0x06 KI 30 | 18,000 | 12,000 | 11 mm | 0.06 mm | slightly softer |

- 0x03 is a ramp in rpm per second with 60000 as its ceiling; 60100 reads back
  60098. Values above 60000 are a feed-forward percent (0 to 98) on top of
  the full ramp. In the drive's own moves the bare rotor follows 60000 rpm/s
  exactly (90% of speed in 39 ms). On pulse following the as-programmed loop
  delivers 25,000 rpm/s.
- The feed-forward percent is not a ramp bypass. The drive anticipates the
  pulse train, runs 25 to 35 mm ahead of what the hub emitted mid-move and
  arrives early. At 1000 mm/s that is about 25 ms early, the same size as the
  lag without it, sign flipped. 40 to 60 gives about a third more accel with
  the error where it is today.
- 0x08 (speed feed-forward) clamps at 4905 and changed nothing useful.
- The ceiling through the registers is about 30,000 mm/s2 up and 20,000 to
  33,000 down, unloaded: double the as-programmed drive, half the planner's
  50,000 default. The next step of gain rings.

## Registers

| register | meaning measured | note |
|---|---|---|
| 0x02 | motor speed clamp, rpm, max 3200 | a saved value; the machine runs 1500 |
| 0x03 | ramp in rpm/s up to 60000; 60000 + n is feed-forward n percent, n up to 98 | |
| 0x08 | speed feed-forward, clamps at 4905 | |
| 0x0F | current, peaks 200 to 1500 in normal moves, 16,000 oscillating | units unverified |
| 0x10 | motor speed in 0.1 rpm, accurate | reads the clamp when clamped |
| 0x11 | not the bus voltage: 11,325 on a 36 V bus | |
| 0x0E | alarm; stayed 0 through everything | |
| 0x00 | Modbus arm; 1 makes the drive deaf to pulses; leave by 506 then 0 | |

Live writes without saving go through the output-off gate (0x00 1, 0x01 0,
write, 0x01 back, 0x00 506, 0x00 0) and revert on power cycle; a save needs
0x14 1 last, polled for 2. Writes made with the output live were refused.

## Measurement rules

- The hub's reported position is its emitted count. Comparing it with the
  encoder measures pulses the drive received, not where the motor is.
- The 0x1100 speed field is the plan's velocity: it reads zero once the plan
  ends while the emitter is still creeping.
- A derivative of the 60 Hz position frames overstates peaks; read the
  motor's speed from the encoder series.
- A leg that never settles leaves the hub creeping; an encoder read at the
  next leg's start is then off by the lag. One such read produced a false
  position-drift finding, retracted (val-o8y).
- The jog's trapezoid uses jog_accel; the arbiter's residual kick is capped
  by input_accel times one millisecond. Setting input_accel low isolates the
  emitted trapezoid from the catch-up.

## Firmware consequences

- Done in 0.1.23: MAX_SPEED_MM_S 10000 to 2000 and the catalog's speed
  ceiling with it (`flagship_p4/src/hub/valence_config.h`,
  `ValenceCatalog.h`): no setting can ask the LP for more than it sustains.
  val-d66.
- Ruled 2026-10-08 (val-sgj): the factory input set is 1200 mm/s,
  100,000 mm/s2 and 2e7 mm/s3 (`valence_config.h`, `ValenceCatalog.h`),
  because at 1000 mm/s with the programmed drive gains a 100,000 mm/s2 plan
  followed with 13 mm following error, 0.08 mm overshoot, a 21 ms settle and
  no buzz, the motor's 36 V ceiling of about 1130 mm/s makes 1200 ask
  slightly more than it delivers while the drive buffers the difference
  losslessly, and a jerk of 2e7 reaches full accel in 5 ms, under the
  drive's own ramp, so the planner trims only when the speed or accel
  ceilings bite.
- The drive tune that was clean unloaded, position KP 8000 and 0x03
  feed-forward 40 to 60, was tried loaded (the last section): KP 8000 and
  feed-forward 60 are not worth it; feed-forward 40 is the one trade left.
  Under a toy the buzz rule wins.
- The stream path on Kinetic c3af1ed plays a 564 mm triangle about a second
  late with the stepgen surging (kin-8qv); jogs are clean. The handle
  renderer (kin-y6e) replaces that path.
- A jog above input_speed is clipped by the hub's steer cap (input_speed
  plus one tick of input_accel) and then crawls at the kick cap
  (input_accel times 1 ms): 227 mm at 1500 mm/s took 1.1 s with input_speed
  1000. val-25a.

## The loaded ladder on the 60AIM40F

Same wiring (RS485 to the PC, the P4 emitting), carriage and a toy on.
1. `python tools/aim/aim_program.py COM2 --verify` to read the drive; program
   it the same way if it differs.
2. `python tools/aim/aim_move.py COM2 --rpm 1500 --revs 20`, then 2000, then
   3000: the plateau rpm, ramp time and current from the encoder, listening
   for the ragged edge. The drive's own moves; the hub stays idle. On a rail
   this step is skipped: the drive's own moves bypass the hub's window and
   fence (20 revs is 785 mm) and desync the hub's count; step 3 reads the
   plateau.
3. `python tools/aim/jog_ceiling.py --speed 500 --accel 20000 --legs 4` then
   1000 and 1500: lost pulses per leg and cruise. --lo and --hi come from
   the measured rail (step 4 too), and --input-speed must be at least
   --speed, else the steer cap clips the jog (val-25a).
4. `python tools/aim/ff_probe.py --speed 1000 --accel 100000 --legs 4 --regs
   0x07=8000` and the 0x03 60040 / 60060 variants: accel, decel, following
   error, overshoot, rest jitter; the operator's ear decides buzz.
5. `python tools/aim/hub_config.py` resets config-set keys 1 to 7 (window
   0 to 100, the factory jog and input limits) after any run that changed
   them; it never touches max_rail unless given an eighth argument (val-3kd).

Every script restores the registers it changed; `aim_program.py --verify`
confirms it. Results append to `*_results.jsonl` beside the scripts
(ignored by git).

## The loaded ladder on the 60AIM40F, measured 2026-10-08

- The OSSM Flagship rail with the carriage and a toy. The 60AIM40F at the
  machine's registers; `aim_program.py --verify` read MATCH before and after.
  36 V bus, RS485 on COM2 at 115200.
- P4 at Nucleus 0.1.27-p4hub-bench (kinetic.pin 2518117).
- A real two-sided home measured 267.7 mm of usable rail, adopted into
  max_rail. Every leg ran 20 to 247 mm (227 mm), inside the margins.
- Step 2 was skipped; the plateau was read through the hub's jogs.
- The hub's eight limits were restored afterwards to their values before the
  run: window 43 to 143, jog 50 mm/s and 200 mm/s2, input 1000 mm/s,
  50,000 mm/s2 and 5e6 mm/s3, max_rail 267.69
  (`hub_config.py 43 143 50 200 1000 50000 5e6 267.69`).
- ff_probe values follow its summary line: lag, p95 and settle are the mean
  of the legs; following error max, overshoot and current are the largest
  leg. Settle counts from the plan's end; early means the motor arrived
  before it.

### Speed through the hub

`jog_ceiling.py`, accel 20,000 mm/s2, four legs each. Alarm 0 and lost
pulses 0.03 mm or less on every leg.

| jog mm/s | input_speed | moving per leg | average mm/s | drive peak rpm | current peak |
|---|---|---|---|---|---|
| 500 | 1000 | 505 to 515 ms | 441 to 450 | 764 to 766 | 1424 |
| 1000 | 1000 | 295 to 315 ms | 721 to 769 | 1508 to 1513 | 4672 |
| 1500 | 1000 | 1055 to 1100 ms | 206 to 215 | 1510 to 1514 | 4496 |
| 1500 | 2000 | 240 to 270 ms | 841 to 946 | 1509 to 1514 | 5552 |
| 2000 | 2000 | 240 to 270 ms | 841 to 946 | 1510 to 1513 | 5304 |

- From 1000 up the 0x02 clamp bites: 1500 rpm is 982 mm/s.
- 1500 with input_speed 1000: the hub's steer cap (input_speed plus one tick
  of input_accel) clipped the jog to about 1050 mm/s, and the residual kick
  (input_accel times 1 ms, 50 mm/s) crawled the rest. The drive lost
  nothing. val-25a.
- 2000 with input_speed 2000: the drive, pinned at the clamp, buffers the
  417 kHz bursts and catches up; lost 0.02 mm.

### The motor's ceiling, clamp lifted

`ff_probe.py --regs 0x02=3200` (restored after), accel 20,000 mm/s2, four
legs each, alarm 0.

| commanded mm/s | motor peak rpm | following error max | p95 | lag | overshoot | settle | current peak |
|---|---|---|---|---|---|---|---|
| 1500 | 1722 to 1738 | 38 mm | 25 mm | 14 ms | 0.12 mm | 30 ms | 5688 |
| 2000 | 1716 to 1734 | 57 mm | 37 mm | 26 ms | 0.10 mm | 43 ms | 4720 |

- Both commands peak at the same speed: the motor's ceiling on the 36 V bus
  under this load is about 1730 rpm, 1130 mm/s.
- 2000 mm/s needs more bus voltage or field weakening (0x04, untested
  loaded). The factory input ceiling of 1200 mm/s (val-sgj) sits just above
  the motor's ceiling; the drive buffers the difference.

### Acceleration at 1000 mm/s

`ff_probe.py`, clamp 3200, four legs each, alarm 0, lag 1 ms or less. Every
leg landed on the hub's count to 0.04 mm. The programmed gains are KP 3000
(0x07) and 0x03 60000.

| plan accel mm/s2 | drive setting | following error max | p95 | motor ahead, max | overshoot | settle | current peak |
|---|---|---|---|---|---|---|---|
| 50,000 | as programmed | 12.3 mm | 6.2 mm | 11.8 mm | 0.08 mm | 28 ms | 4672 |
| 100,000 | as programmed | 13.0 mm | 5.4 mm | 11.4 mm | 0.08 mm | 21 ms | 4656 |
| 100,000 | 0x07 KP 8000 | 33.0 mm | 12.5 mm | 33.0 mm | 0.29 mm | 36 ms early | 6696 |
| 100,000 | 0x03 60040 (FF 40) | 16.6 mm | 7.2 mm | 16.6 mm | 0.11 mm | 7 ms | 5504 |
| 100,000 | 0x03 60060 (FF 60) | 35.2 mm | 12.1 mm | 35.2 mm | 0.07 mm | 0 ms | 5320 |

- The drive and the toy on it take a 100,000 mm/s2 plan at 1000 mm/s
  cleanly with the programmed gains (following error 13 mm, settle 21 ms),
  so the catalog's 100,000 ceiling (MAX_ACCEL_MM_S2) is real for this toy.
  The factory default is 100,000 (val-sgj); heavier toys are torque-bound.
- KP 8000 and feed-forward 60 are not worth it loaded: following error p95
  more than doubles and the motor runs 33 to 35 mm ahead of the pulses.
- Feed-forward 40 trades 5 mm more lead for a 14 ms faster settle.

### The operator's verdict

- The encoder is backdriven by the rail, so any leg within 0.05 mm of the
  command is at the commanded position. Every leg of the ladder was
  (0.034 mm or less).
- Nothing buzzed and nothing sounded wrong on any run, KP 8000 and the
  feed-forward runs included.
- Motion was clean up to the motor's ceiling; the operator names 1200 mm/s
  as clean.

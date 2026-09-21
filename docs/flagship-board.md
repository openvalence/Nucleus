# Flagship board -- design considerations

The fully-populated variant: everything worth having fitted, with cheaper
builds as reductions of it rather than the other way round. Target motor is the
YZ 60AIM series; the board must also drive the 350 W 60AIM40, which nothing on
the market currently does.

**Status and measured values live on the dev board, not here.** Numbers quoted
below are context for the reasoning; their home is `bd` (sd-jg9 for the power
and regen work, sd-4k1.31 for thermal, sd-jg9.1 for the emitter, sd-cmp,
sd-4z9, sd-lfp). This file is why, not what-is-true-today.

Each section gives the leading option and the alternative, because most of
these are still open.

---

## 1. What the measurements decided

Everything below rests on a bench session of 2026-09-19/20. The short version:

| | |
|---|---|
| back-EMF, hard throw, unclamped | 58.6 V |
| back-EMF, normal hand movement | under 40 V |
| regen while powered | 6.8 W mean, 141 W peak, 4.8% duty |
| regen unpowered (back-drive) | 13.6 W mean, 189 W peak, 678 J over 50 s |
| bus capacitance measured | ~2600 uF, essentially all external |
| motor thermal time constant | 41 min |
| motor continuous dissipation | 7.5-15 W, still air |

Three of these changed the design:

- **Capacitance does almost nothing for regen.** 3000 uF buys ~0.9 J, about
  30 ms at 29 W. Regen is a continuous-power problem, not a transient one, so
  the bulk bank is sized for supply decoupling and nothing else.
- **Powered regen is easier than unpowered.** The drive absorbs most of it into
  motoring work while it is running; unpowered it is a passive rectifier and
  everything lands outside. The worst case is therefore a back-drive with the
  machine off, which is a routine user action, not abuse.
- **The supply's OVP is the real ceiling**, not the drive's rating. Mean Well
  GST360A36 trips between 37.8 and 48.6 V and latches.

---

## 2. Power path rating

**Likely: 10 A continuous with peak headroom.** Covers the operator's 200 W
60AIM40F at 7 A and the 350 W 60AIM40 at 9.5-10 A. Being the only board that
can drive the big motor is a feature worth the margin.

**Alternative: 7 A.** Smaller FETs, cheaper connectors, smaller everything --
and no 350 W compatibility. Rejected because the margin is the differentiator.

Consequences that follow from 10 A and are not separately negotiable: series
devices in packages bonded to the heatsink, and a power connector above
Mini-Fit Jr's 9 A per circuit.

---

## 3. Input protection

Right at the 36 V input. **This section bootstraps** -- it protects the rail
that powers the board, so nothing in it may depend on firmware, an enable, or
any rail it is itself responsible for. All of it must be self-acting.

Jobs: reverse polarity, inrush limiting into the bulk capacitance, overcurrent.

**Likely: an eFuse with an integrated FET** (TI TPS259x / TPS168x class), if one
covers 10 A at 36 V. The integrated FET is the whole appeal -- it **deletes the
SOA question**, because the die is characterized for its own inrush behavior.

**Alternative: LTC4364 or LM5069 with external N-FETs.** LTC4364 is a surge
stopper plus ideal diode and covers reverse polarity, reverse current,
over/undervoltage, current limit and inrush in one part. LM5069 is the cheaper
hot-swap-only version. Both are SOA-timed, which helps, but FET selection and
its consequences remain ours.

> **The inrush FET is SOA-limited, not Rds(on)-limited.** 1000 uF at 36 V puts
> ~0.65 J into one device during the ramp, in its linear region. Pick the part
> off its pulse safe-operating-area curve. A part with excellent milliohms and
> a weak SOA passes the bench test and fails on the twentieth plug-in. Slower
> ramps are not automatically safer -- lower peak, longer duration, and SOA
> falls off with time, so there is an optimum rather than a direction.

**Rejected: NTC inrush limiter.** Dodges SOA entirely, but its hot resistance
costs 2-5 W continuously at 10 A -- the same objection that rules out P-channel
below.

**Rejected: scaling up the existing dev board's P-channel circuit.** It is a
correct reverse-polarity design, but a 60 V P-FET at that class is 60-90 mOhm,
so 6-9 W at 10 A. It also does not block reverse current: a P-channel body
diode conducts whenever drain exceeds source, so back-EMF above the rail passes
straight back to the supply. On the current machine it always has.

Because the motor switch (section 4) isolates the board from back-EMF when
open, the input section needs reverse **polarity** protection but not
necessarily reverse **current** blocking. That widens the part choice. An
ideal-diode controller supplies both anyway.

---

## 4. Motor switch

Between the board's protected rail and the motor output. Separate section from
input protection, and unlike it, this one may be firmware-controlled because
the board is alive by then.

Jobs: e-stop cut, and isolating the PCB from back-EMF whenever the motor is not
meant to be energized.

**Likely: back-to-back N-channel FETs, common source, charge-pump gate drive.**

Behavior, which is the point of the arrangement:

| state | motor | board |
|---|---|---|
| unpowered | floating | isolated |
| powered, EN low | floating | isolated |
| powered, EN high | energized | shunt and caps active on the bus |

Open means genuinely open in both directions, so a back-drive has no path into
the bulk caps, the shunt, the regulators or anything else. The motor's own
drive clamps its back-EMF internally, which it has been doing reliably for
months. **This is what removes the unpowered-abuse case from the board's
problem space entirely**, and it is why the shunt sizes for the powered figure
(6.8 W) rather than the unpowered one (13.6 W).

**Alternative: a single P-channel FET.** Simpler, no charge pump, default-off
comes free. It does block forward current when off -- a P-channel body diode
faces backwards relative to board-to-motor flow -- so it works as a cut. But it
passes reverse, so back-EMF still reaches the board's caps, and back-to-back
P-channel would double an already poor Rds(on) to ~7 W at 10 A. Rejected on
heat.

Two constraints on the driver:

- **Charge pump, not bootstrap.** The switch is statically on for minutes; a
  bootstrap cap only recharges when the node swings, so it droops and turns the
  FETs off. Look for "100% duty cycle" or "DC operation" in the datasheet.
- Common-source back-to-back means both gates share a reference and switch
  together: **one driver channel, two gates**, not two channels.

**Hard requirement: a pulldown on EN.** That GPIO floats during P4 boot and
while the MCU is in reset, and this is the one signal where a float must read
as off.

---

## 5. Regen shunt

**Likely: on-board, two TO-263 power resistors over a via field into the
heatsink, plus a connector for an external resistor as an escape hatch.**

Sizing: ~7 W continuous and ~150 W peak once the motor switch removes the
unpowered case. Two parts rather than one halves the power density at the pads,
which is where the thermal path actually bottlenecks, and lands nearer the
8 ohm that would hold the bus down rather than saturating the way 10 ohm did.

If the external connector is fitted, decide how it combines. **Parallel** lowers
total resistance and raises sink current -- the fix if the clamp saturates.
**Replacement** (via a solder jumper cutting the on-board array) relocates the
heat -- the fix if thermals disappoint. One jumper next to the connector buys
both, and costs nothing at layout. Size the FET for the lowest combined
resistance either path can produce.

**Alternative: a separate open-air module on its own card**, as the reference
add-on board does -- vertical mounting, both faces in airflow, 12 W with no
heatsink at all. Keeps all shunt heat off the main board. Costs a connector and
an assembly step.

Fit a TVS or RC snubber across the switch either way, and especially if an
external resistor on flying leads is an option -- those leads add series
inductance whose energy lands on the drain at turn-off.

**Clamp threshold must sit below the supply's OVP.** With the input section
blocking reverse current that is a free choice around 45 V. Without it, the
window between the 36 V rail and a worst-case 37.8 V trip is under two volts,
which is not designable.

---

## 6. Bulk capacitance

**Likely: aluminum polymer or hybrid polymer, 63 V minimum, a few hundred uF.**

The job is **supply decoupling against a cheap brick's cable inductance**, not
regen absorption -- the measurement settled that. So the requirement is low ESR
and modest capacitance, which is exactly polymer's strength, and the quantity
needed is small enough that the price premium stops mattering.

Hybrid polymer is worth checking before committing: pure polymer thins out
above 63 V, while hybrids reach 80-100 V with more uF per part and lower
leakage, keeping most of the thermal-life advantage.

Supplement with MLCC for high frequency. **Do not try to substitute MLCC for
bulk** -- at 63 V with DC bias derating you would need hundreds of parts.

**The tiering is deliberate, not "more is better":** a bank of nothing but
ceramics can be too low-impedance and ring against trace inductance. The bulk
part's ESR damps that. The slow high-capacity device is doing two jobs.

**Alternative: wet aluminum electrolytic.** Cheaper and higher capacitance per
part. Lifetime halves per 10 degC against polymer's 20, which mattered when the
board was going to be motor-mounted and matters much less now that it is not.

The motor's own 1000 uF recommendation is an impedance specification wearing a
capacitance label, and the dev board has run 2.5 m of cable with nothing on the
motor side without trouble. At the planned 100-150 mm it is not a constraint.

---

## 7. Fail-safe brake -- considered and dropped

Shorting the motor when unpowered gives genuine dynamic braking: braking force
rises with speed, and bench testing with a shorted XT30 capped hand-pushing at
roughly 300 mm/s. It would have prevented the high-speed back-drive case
outright rather than absorbing it.

**Dropped** because the back-to-back motor switch isolates the board anyway, so
the case it protects against is no longer the board's problem. It also needed a
normally-closed element, which solid state does not offer directly: the
synthesis was a low-side FET self-powered from the rising bus with an inhibit
driven from the supply side of a series diode. Workable, but four or five parts
and a diode that existed only to create the sense node.

**If it is ever revisited:** the brake FET must sit downstream of the reverse
protection. Its body diode runs from ground up to motor positive, so on the raw
input a reversed supply forward-biases it into a dead short regardless of the
gate.

---

## 8. Compute

**Likely: M5Stack Stamp-P4 (ESP32-P4NRW32) plus the Stamp-AddOn C6, with an
RP2350 for motion.**

The RP2350 keeps the proven emitter on the first-party board. P4-native
emission (LP core or PARLIO, sd-jg9.1) stays the path for anyone running the
firmware on off-the-shelf hardware. Both live behind `MotorDriver` and are
selected at compile time, which needs the architecture amendment on sd-5qf.

Why the stamp rather than a bare P4: it takes the 32 MB octal PSRAM and 16 MB
flash interface off our board entirely, which is the single largest layout risk
removed, and it is castellated and hand-solderable.

Relevant stamp facts: 360 MHz HP dual-core (not 320 or 400 -- IDF caps
engineering samples), 40 MHz LP core, 44 GPIO broken out as G0-G39/G41/G49/G50/
G52 with the LP pins labeled LPG0-LPG15, LP UART on LPG15/LPG14, I2C on
LPG9/LPG11 shared with the add-on connector, ADC1 on G16-G23. The C6 add-on
occupies G42-G48 over SDIO, which are not castellated, so the radio costs zero
usable pins.

**Alternative: bare P4 and a discrete RP2350.** More design work, more layout
risk, no module premium. The operator's stated preference for discrete design
makes this a live option rather than a straw man, but the PSRAM interface is
the part that argues hardest against it.

**Radio alternative: C5 instead of C6** for dual-band. Not pin-compatible --
C6-MINI-1 is 53 pads, C5-MINI-1 is 65 -- so it needs its own carrier, but the
add-on is a module, an LDO and a connector, so a variant is a layout afternoon.
Gated on sd-4k1.35, the 2.4 vs 5 GHz bench.

---

## 9. Mechanical and thermal

**Likely: a twist-on end cap behind the motor, carrying a 50x50 heatsink, with
through-hole connectors around the edge and the shunt resistors in the middle.**

Not motor-mounted. That decision removed a whole class of problem at once: the
cap lifetime argument against an 85 degC ambient, the shunt sharing a thermal
path with the drive and stator, the absence of rear mounting holes, and the
height competition between tall parts. 40x40 heatsinks are common for NEMA 23
if a smaller cap is wanted.

**Alternative: mounted to the motor's rear face with a thermal pad**, with the
board acting as the heatsink people already add by hand. The motor sheds only
7.5-15 W in still air, so a conducting board is a large relative improvement.
Rejected for now because the mechanical case (alignment, port registration,
securing) is much easier with a cap, and because the board's own dissipation
turned out small enough not to need the motor's mass.

Assembly assumption: **factory reflows the top side, operator hand-solders the
back.** That is the cheapest assembly tier and a clean boundary if the back
holds only the resistor array and the power connector. Package choices follow
from it -- prefer gull-wing over bottom-thermal-pad parts anywhere the operator
solders, and DPAK-class tabs over PowerPAK for anything carrying current.

---

## 10. Accessory rails

**Likely: 3.3 V, 5 V and 12 V switchers, a few amps each.**

People add pumps and mods; a board sold as the heart of a machine that cannot
power an accessory fails at being the heart. At these currents they must be
switchers -- 36 V to 3.3 V linear at 2 A is 65 W.

**Likely topology: independent bucks from 36 V.** **Alternative: cascade
36-12-5-3.3**, cheaper and more efficient per stage, but all current passes
through the 12 V converter.

> **The layout tension on this board is amp-class switching nodes sharing it
> with an INA228 resolving +-5 uV.** The sense resistor, its Kelvin traces and
> the INA want to be as far from the switchers as the board allows, with their
> own quiet ground. This is the thing most likely to cost a revision.

**Fan and temperature headers: fitted, not populated.** A PWM fan header with
tach, and a two-pin analog thermistor header into one of the P4's eight spare
ADC1 channels -- any NTC from any drawer works, where I2C would force the user
to buy a specific part. The fan needs a rail: either leave a 12 V buck footprint
unpopulated, or standardize on 5 V fans off the logic rail.

Active cooling is **not** in the base design. The shunt is single-digit watts
and intermittent; a 40 mm fan is the worst acoustic size on a product where
noise matters. Revisit only if a logged normal session shows the motor
climbing, which has never been measured.

---

## 11. Connectors

The motor presents two ports: a 6-way power and motion block (+V, GND, PU+,
PU-, DIR+, DIR-, the pulse pairs carrying A and B in quadrature mode), and a
10-way communication and status block.

**Likely: XT30 for power plus a separate signal connector.** At 10 A, Mini-Fit
Jr's 9 A per circuit is marginal. XT30's gold bullets are 30 A in a smaller
package than a 6-way Mini-Fit, and putting bus voltage and RS485 in different
shells means they cannot bridge to each other at all.

**Alternative: one 6-way Mini-Fit Jr or JST VH.** Single mating action, keyed,
ubiquitous. If taken, **order the pins so GND sits between the supply pin and
the signals**, so a bent pin or whisker lands on ground rather than driving bus
voltage into an RS485 transceiver. Keep each differential pair on adjacent
positions and twisted in the cable.

XT30 is also the one through-hole part likely to survive the all-SMD rule.
Budget its height -- it becomes the tallest component.

---

## 12. Sensing and drive status

**INA228 on the main bus.** Expensive at around five dollars, and it earns it
here: +-5 uV offset lets a 1 mOhm shunt dissipate nothing, it has hardware
tempco correction for the sense resistor, and an integrated die temperature
sensor that gives board temperature for free.

> **Its ALERT pin is a hardware overcurrent trip we are not currently using.**
> Limit registers for over-current, over-power and over-temperature assert a
> pin with no MCU involvement. The stall guard presently polls at 10 Hz in
> firmware and everything downstream depends on the MCU being alive. This is
> the cheapest version of the bus-independent cutoff sd-4k1.31 asks for.

**Second channel, for shunt dissipation: likely none.** Bring the chopper gate
to a GPIO and measure duty -- R is known to 0.1% and bus voltage is already
measured, so `duty x V^2 / R` is the dissipation for the cost of one trace.
**Alternative:** a second INA228 at a different address (it has A0/A1, so up to
16), or an INA238 at roughly half the price and 16 bits, which drops the energy
accumulator the odometer uses. If a sense part is fitted it needs a milliohm
element in series with the array; the INA cannot sit across the array itself,
whose 40-plus volts is far outside its +-163 mV input range.

**From the drive's communication port, three lines worth wiring:**

- **WR (pin 7): alarm output**, opto NPN, pulls to COM. Gives the 0x14 block
  (stall) and 0x15 overpressure alarms as a hardware signal with no Modbus
  latency. 0x14 in particular is a second, independent stall detector living in
  the thing that actually delivers the torque, and its timeout is settable.
- **RDY/PF (pin 8): following error** in hardware, on below 0.5 degrees.
- **ZO (pin 9): encoder index**, once per motor revolution. The encoder is
  single-loop, so this is an index rather than an absolute zero -- a hard
  reference every 39.27 mm at the current geometry. Enables self-calibrating
  drive ratio (sd-lfp) and index-refined homing to replace the wall-find's
  1-3 mm phantom-step error.

Pin 10 (485_5V) must be supplied by us; pin 6 (COM) is the reference for the
opto outputs as well as the 485 section.

**Dropped: galvanic isolation on the I2C.** The v1 ISO1640 bought noise
immunity and convenient debugging, not safety -- one isolated brick, one common
return. What replaces it is layout: Kelvin connections on the sense resistor,
the INA beside it, sense traces routed as a tight pair, switching return
current kept off the path between them.

---

## 13. Open questions

- Does an integrated-FET eFuse cover 10 A at 36 V? If so it is the simplest
  possible input section and deletes the SOA problem.
- sd-kjy, the e-gear sweep, gates any emitter sizing conclusion. It is runnable
  on the current machine and needs no new hardware.
- Confirm LPG0-LPG15 map to GPIO0-15 against the stamp pinmap before committing
  A/B to pins.
- Are SCL/SDA on the add-on connector the P4's own I2C, and does the C6 module
  occupy that bus? Decides where the INA228 sits.
- What actually limits the drive to 4.1-4.5 A, given the 60 series runs to 7 A?
- The 60AIM40F's rotor inertia and weight are not in any datasheet held here.
  Nothing on the board currently depends on either.

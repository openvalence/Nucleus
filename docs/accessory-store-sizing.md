# Accessory store sizing

What N Isotope accessories cost the hub, with each one declaring a full channel
slice. This is paper arithmetic for bd `val-9u0.5`; the bench numbers that
bead also asks for (unpack time, a measured encode) are still owed. **Picking
N is an operator ruling**, recorded on the bead, not here.

## Inputs

| Input | Value | Source |
|---|---|---|
| Worst-case declaration, 128 actuators | 15,052 B (116.8 B per channel) | Isotope `docs/DESIGN.md` section 16 (commit 3a02189) |
| Legal maximum, 126 own ids | 14,818 B | same |
| Entry costs (encoded) | actuator 117 B, readout 115 B, mirrored STATE+INTENT pair 232 B, accessory-status 96 B | same |
| Channels per slice | 126 (RFC-076 item 1 reserves relative ids 0x00, 0x01); 63 if every actuator is a mirrored pair | same |
| Accessory ceiling | 19 by unicast (ESP-NOW's 20 peers, less the broadcast entry) | Valence `spec/RFC-QUEUE.md` RFC-075 item 9 |
| PSRAM free, steady state | 33,551,348 B | `.claude/rules/memory-budget.md` |
| Unallocated flash tail | 0x820000 to 0xFFFFFF = 8,257,536 B | `flagship_p4/partitions.csv` |
| Pool slot sizes, 32-bit target | CatalogEntry 40 B, LayoutField 92 B, SchemaField 92 B | Valence `lib/valence/include/valence/channel/catalog.hpp` header (measured on xtensa; the P4 is also ILP32) |
| Catalog32 | 48 entries, 200 layout, 160 schema, 192 labels, 32,768 B encode scratch | `catalog.hpp` line 880, `hub/hub.hpp` `kCatalogScratchBytes` |

**Catalog32 in use today** [verified 2026-10-01: scratch host build of
`buildValenceCatalog` with the board's features, `encodeCatalog` into an
unbounded buffer]: 42/48 entries, 188/200 layout, 109/160 schema, 164/192
labels, 23,595 B encoded. With the power channel enabled (val-091.22):
43 entries, 192 layout, 23,982 B. The 80% headroom floor on the scratch is
26,214 B.

## Per accessory

Both slice shapes cost the same pool slots: 126 readouts take 126 layout
fields; 63 mirrored pairs take 63 layout plus 63 schema fields. Either way it
is 127 entries (channels plus the status entry) and 130 field slots, taking the
status entry as up to 4 layout fields (an assumption until RFC-078 fixes it).

| Item | Arithmetic | Bytes |
|---|---|---|
| Declaration, stored verbatim | budget at the worst case | 15,052 |
| Catalog pool slots | 127 x 40 + 130 x 92 | 17,040 |
| Encoded catalog growth (scratch) | the declaration is a section 8.1 catalog fragment | ~15,052 |
| Hub-authored names | 126 x 32 B | 4,032 |
| **PSRAM total** | | **~51,176** |
| Flash record | declaration 4 sectors + metadata/names 2 sectors = 24 KiB, kept A/B for power-loss safety | 49,152 |

The catalog's names and units are `string_view`s. For an accessory they can
only point into the stored declaration, so that declaration must stay at a
fixed PSRAM address for as long as any catalog references it. A join or a
leave rebuilds the catalog first and frees the old declaration second.

## Totals by N

| N | PSRAM | % of free | Flash (A/B) | Entries | Layout slots | Schema slots | Encoded catalog | Scratch at 80% | BLOB chunks to a client |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 50 KiB | 0.15% | 48 KiB | 175 | 330 | 223 | 39,034 B | 48,792 B | 204 |
| 3 | 150 KiB | 0.46% | 144 KiB | 429 | 590 | 349 | 69,138 B | 86,422 B | 361 |
| 8 | 400 KiB | 1.2% | 384 KiB | 1,064 | 1,240 | 664 | 144,398 B | 180,498 B | 753 |
| 19 | 950 KiB | 2.9% | 912 KiB | 2,461 | 2,670 | 1,357 | 309,970 B | 387,462 B | 1,615 |

Entries = 48 + 127 N. Layout = 200 + 130 N (126-readout shape). Schema =
160 + 63 N (63-pair shape). Encoded = 23,982 + 15,052 N. Chunks are the
encoded catalog over 192 B BLOB_CHUNK payloads.

## Where it binds

- **PSRAM never binds.** N = 19 at the worst case is under 3% of free PSRAM.
- **Flash never binds.** N = 19 with A/B records is 912 KiB of the 7.9 MiB
  tail. The bead's proposed 256 KiB partition holds N = 5 with A/B records; a
  1 MiB partition at 0x820000 holds N = 19. Either is a partition-table change,
  so one serial flash (`build-test-deploy.md`).
- **Catalog32 binds at N = 0 for a full slice.** It has 6 free entries, 12 free
  layout slots and 2,619 B under the scratch's 80% floor. One full slice needs
  127 entries, 130 field slots and ~15 KB of scratch. Any N >= 1 at full size
  needs a larger catalog type and scratch. Both live in Valence's `catalog.hpp`
  and `hub.hpp`, so the change goes through the RFC process, together with the
  live catalog growth that `val-9u0.12` is blocked on.
- **The client transfer is the soft wall.** At N = 19, every connect and every
  re-announce sends a ~310 KB catalog in ~1,600 chunks. That transfer time has
  not been measured. Entry counts past 256 also go beyond the spec's
  `catalog_max_entries` floor, which every client has to handle.
- **The id space may bind before memory.** Isotope `DESIGN.md` section 8 counts
  18 free slices if multi-axis never claims its domains. RFC-076 picks the
  range.

## Smaller channel budgets

All costs except the 48/200/160 base and the fixed 23,982 B are linear in
channels per accessory. A hub that advertises a smaller per-accessory budget
(Isotope `DESIGN.md` section 8 proposal) scales every column above by
budget / 126. At 16 channels each, N = 19 is about 120 KiB of PSRAM and a
~61 KB catalog.

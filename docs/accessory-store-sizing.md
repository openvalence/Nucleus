# Accessory store sizing

What N Isotope accessories cost the hub, with each one declaring a full channel
slice, and the capacities the Flagship build is compiled with. The numbers the
build uses live in `flagship_p4/valence_capacity.cmake` (C-1); this page is the
arithmetic behind them. Bench numbers (unpack time, a measured encode on the
P4, the hub box's PSRAM size at boot) are still owed on bd `val-9u0.5`.

## Inputs

| Input | Value | Source |
|---|---|---|
| Slice | 32 ids (`accessory_slice_ids` 0x20): r 0x00 never a channel, r 0x01 accessory-status, r 0x02-0x1F the accessory's own, **30 at most** | Valence `spec/SPEC.md` section 8.10, `registry.yaml` limits |
| Declaration ceiling | 4,096 B (`accessory_declaration_max_bytes`) | same |
| Accessory-status layout | state u8 + fault u8 + beacon_seq u16: 3 fields | `registry.yaml` `accessory_status` |
| Catalog entry floor every client handles | 256 (`catalog_max_entries`) | Valence `spec/SPEC.md` section 8.1 |
| Accessory ceiling by radio | 19 by unicast (ESP-NOW's 20 peers, less broadcast) | Valence RFC-075 item 9 |
| Pool slot sizes, 32-bit target | CatalogEntry 40 B, LayoutField 92 B, SchemaField 92 B, label 8 B + 1 B access | Valence `catalog.hpp` header (measured on xtensa; the P4 is also ILP32) |

**The machine's own catalog** [verified 2026-10-02 -- host build of
`buildValenceCatalog` with the board's features into an oversized
`Catalog32`, `encodeCatalog` into an unbounded buffer, at Valence 715e21c]:
42 entries, 188 layout, 107 schema, 164 labels, 2 stores, 23,927 B encoded.
With the power channel (val-091.22): 43 entries, 192 layout, 24,322 B. The
machine keeps the budget it had before RFC-077: 48 entries, 200 layout, 160
schema, 192 labels, and 26,214 B of encoded catalog (80% of 32,768).

## Per accessory, at 30 channels

| Pool | Budget | Shape that spends it |
|---|---|---|
| Entries | 31 | 30 channels + the status entry |
| Layout fields | 33 | 30 one-field readouts + the status entry's 3 |
| Schema fields | 30 | 30 one-field actuators |
| Labels | 64 | selects and bitfields; not advertised, so sized never to bind first |
| Encoded catalog | 5,120 B | the 4,096 B declaration + 25% for hub-authored text |

## The count, and the build

N is the most that keeps total entries at or under 256:
(256 - 48) / 31 = **6**. Seven would be 265 entries, past the floor a client
is built to handle, so a radio that could carry 19 is not the bound; the
catalog is.

| Flag | Value | Arithmetic |
|---|---|---|
| `VALENCE_CATALOG_ENTRIES` | 234 | 48 + 6 x 31 |
| `VALENCE_CATALOG_LAYOUT_FIELDS` | 398 | 200 + 6 x 33 |
| `VALENCE_CATALOG_SCHEMA_FIELDS` | 340 | 160 + 6 x 30 |
| `VALENCE_CATALOG_LABELS` | 576 | 192 + 6 x 64 |
| `VALENCE_CATALOG_STORES` | 6 | the machine's 2, the accessories and relationships STOREs, one spare |
| `VALENCE_CATALOG_SCRATCH_BYTES` | 73,728 | (26,214 + 6 x 5,120) / 0.8 = 71,168, rounded up to 72 KiB |

Cost, paper arithmetic at the 32-bit slot sizes: the pools grow from ~36.8 KB
to ~82.4 KB and the hub's encode scratch from 32 KiB to 72 KiB, ~87 KB more in
total. Both live in the PSRAM `HubBox` (T2), never on a stack. PSRAM free is
33.5 MB (`memory-budget.md`), so this is 0.26% of it.

**Headroom as the hub computes it** [verified 2026-10-02 -- valencesim boot
log at this build]: 6 accessories; free 192 entries, 210 layout, 233 schema,
35,055 B. `catalogHeadroom()` in `ValenceDevice.h` is the one computation; the
accessories roster (val-9u0.19) advertises its answer and nothing more.

## Where it binds

- **The catalog binds, at 256 entries.** Neither PSRAM nor flash comes close:
  six accessories' records at 4 KiB each, kept A/B, are 48 KiB of the 7.9 MiB
  flash tail.
- **The client transfer stays modest.** At N = 6 the catalog is at most
  ~59 KB, about 300 BLOB chunks of 192 B per connect or re-announce.

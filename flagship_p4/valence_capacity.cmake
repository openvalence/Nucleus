# The Flagship's Valence catalog capacities (RFC-077 item 8): the ONE home.
# flagship_p4/CMakeLists.txt applies them to every component of the P4 build
# and sim/neutrino/CMakeLists.txt to the twin, so both instantiate the same
# Catalog32 and the hub's encode scratch at the same size. They are BUILD
# FLAGS, never a #define in a source file: two translation units seeing two
# values would be two Catalog32 types (ODR), and nothing would say so.
#
# Sizing (val-9u0.5, val-1vs): the machine's pools are 54 entries, 292 layout,
# 176 schema and 200 labels; its encoded bytes have no fixed budget. Each
# accessory gets the per-accessory budget below. What binds:
# - catalog_max_entries (256), the floor every client handles: the accessory
#   count is the most that keeps total entries at or under it,
#   (256 - 54) / 31 = 6.
# - catalog_max_entry_bytes (4,096): the encoder refuses a larger entry.
# - The 80% headroom rule: the machine catalog plus 6 x 5,120 B stays under
#   80% of VALENCE_CATALOG_SCRATCH_BYTES (209,715 B at 256 KiB);
#   catalogHeadroom() is the check.
# - catalog_max_bytes (1 MiB, Valence RFC-121, DRAFT): the scratch never
#   exceeds it.
# The scratch is a member of the Hub, which lives in the PSRAM HubBox
# (ValenceHub.cpp): never build a Hub on a stack or in static RAM.
# Machine use: 53 entries, 288 layout, 141 schema, 31,169 B encoded
# [verified 2026-10-20 -- neutrino-sim boot log, etag d5895d9c2c5bd88d].

set(NUCLEUS_ACCESSORIES 6)
# Per accessory: 30 channels (r 0x02-0x1F) + the accessory-status entry (r 0x01).
set(NUCLEUS_ACCESSORY_ENTRIES 31)
# 30 one-field readouts + the status entry's 3 fields (state, fault, beacon_seq).
set(NUCLEUS_ACCESSORY_LAYOUT_FIELDS 33)
# 30 one-field actuators.
set(NUCLEUS_ACCESSORY_SCHEMA_FIELDS 30)
set(NUCLEUS_ACCESSORY_LABELS 64)
# RFC-076 `safe` values: every schema field plus every layout field but the
# status entry's 3 may be one that must declare `safe` (SPEC 8.10), so this pool
# never binds before the two field pools do. The machine declares none.
math(EXPR NUCLEUS_ACCESSORY_SAFE_FIELDS
     "${NUCLEUS_ACCESSORY_SCHEMA_FIELDS} + ${NUCLEUS_ACCESSORY_LAYOUT_FIELDS} - 3")
# accessory_declaration_max_bytes (4096) plus 25% for hub-authored text.
set(NUCLEUS_ACCESSORY_CATALOG_BYTES 5120)

math(EXPR _entries "54 + ${NUCLEUS_ACCESSORIES} * ${NUCLEUS_ACCESSORY_ENTRIES}")
math(EXPR _layout  "292 + ${NUCLEUS_ACCESSORIES} * ${NUCLEUS_ACCESSORY_LAYOUT_FIELDS}")
math(EXPR _schema  "176 + ${NUCLEUS_ACCESSORIES} * ${NUCLEUS_ACCESSORY_SCHEMA_FIELDS}")
math(EXPR _labels  "200 + ${NUCLEUS_ACCESSORIES} * ${NUCLEUS_ACCESSORY_LABELS}")
math(EXPR _safe    "${NUCLEUS_ACCESSORIES} * ${NUCLEUS_ACCESSORY_SAFE_FIELDS}")

set(VALENCE_CAPACITY_DEFINITIONS
    VALENCE_CATALOG_ENTRIES=${_entries}
    VALENCE_CATALOG_LAYOUT_FIELDS=${_layout}
    VALENCE_CATALOG_SCHEMA_FIELDS=${_schema}
    VALENCE_CATALOG_LABELS=${_labels}
    # Two more than the machine's 2 + spare: the accessories and relationships
    # STOREs (RFC-076, RFC-078) are the host's own when they land.
    VALENCE_CATALOG_STORES=6
    VALENCE_CATALOG_SAFE_SLOTS=${_safe}
    VALENCE_CATALOG_SCRATCH_BYTES=262144
    NUCLEUS_ACCESSORIES=${NUCLEUS_ACCESSORIES}
    NUCLEUS_ACCESSORY_ENTRIES=${NUCLEUS_ACCESSORY_ENTRIES}
    NUCLEUS_ACCESSORY_LAYOUT_FIELDS=${NUCLEUS_ACCESSORY_LAYOUT_FIELDS}
    NUCLEUS_ACCESSORY_SCHEMA_FIELDS=${NUCLEUS_ACCESSORY_SCHEMA_FIELDS}
    NUCLEUS_ACCESSORY_SAFE_FIELDS=${NUCLEUS_ACCESSORY_SAFE_FIELDS}
    NUCLEUS_ACCESSORY_CATALOG_BYTES=${NUCLEUS_ACCESSORY_CATALOG_BYTES})

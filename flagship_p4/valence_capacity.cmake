# The Flagship's Valence catalog capacities (RFC-077 item 8): the ONE home.
# flagship_p4/CMakeLists.txt applies them to every component of the P4 build
# and sim/valencesim/CMakeLists.txt to the twin, so both instantiate the same
# Catalog32 and the hub's encode scratch at the same size. They are BUILD
# FLAGS, never a #define in a source file: two translation units seeing two
# values would be two Catalog32 types (ODR), and nothing would say so.
#
# Sizing (val-9u0.5, recomputed 2026-10-02 for the 32-id slice of RFC-076):
# the machine keeps the 48 / 200 / 160 / 192 budget it had (42 entries, 192
# layout, 107 schema, 164 labels, 24,322 B encoded with the power channel), and
# each accessory gets the per-accessory budget below. The accessory count is
# the most that keeps the total entries at or under catalog_max_entries (256),
# the floor every client is built to handle: (256 - 48) / 31 = 6.
# The encode scratch keeps the 80% headroom floor over the machine's 26,214 B
# plus 6 x 5,120 B, rounded up to 72 KiB.

set(NUCLEUS_ACCESSORIES 6)
# Per accessory: 30 channels (r 0x02-0x1F) + the accessory-status entry (r 0x01).
set(NUCLEUS_ACCESSORY_ENTRIES 31)
# 30 one-field readouts + the status entry's 3 fields (state, fault, beacon_seq).
set(NUCLEUS_ACCESSORY_LAYOUT_FIELDS 33)
# 30 one-field actuators.
set(NUCLEUS_ACCESSORY_SCHEMA_FIELDS 30)
set(NUCLEUS_ACCESSORY_LABELS 64)
# accessory_declaration_max_bytes (4096) plus 25% for hub-authored text.
set(NUCLEUS_ACCESSORY_CATALOG_BYTES 5120)

math(EXPR _entries "48 + ${NUCLEUS_ACCESSORIES} * ${NUCLEUS_ACCESSORY_ENTRIES}")
math(EXPR _layout  "200 + ${NUCLEUS_ACCESSORIES} * ${NUCLEUS_ACCESSORY_LAYOUT_FIELDS}")
math(EXPR _schema  "160 + ${NUCLEUS_ACCESSORIES} * ${NUCLEUS_ACCESSORY_SCHEMA_FIELDS}")
math(EXPR _labels  "192 + ${NUCLEUS_ACCESSORIES} * ${NUCLEUS_ACCESSORY_LABELS}")

set(VALENCE_CAPACITY_DEFINITIONS
    VALENCE_CATALOG_ENTRIES=${_entries}
    VALENCE_CATALOG_LAYOUT_FIELDS=${_layout}
    VALENCE_CATALOG_SCHEMA_FIELDS=${_schema}
    VALENCE_CATALOG_LABELS=${_labels}
    # Two more than the machine's 2 + spare: the accessories and relationships
    # STOREs (RFC-076, RFC-078) are the host's own when they land.
    VALENCE_CATALOG_STORES=6
    VALENCE_CATALOG_SCRATCH_BYTES=73728
    NUCLEUS_ACCESSORIES=${NUCLEUS_ACCESSORIES}
    NUCLEUS_ACCESSORY_ENTRIES=${NUCLEUS_ACCESSORY_ENTRIES}
    NUCLEUS_ACCESSORY_LAYOUT_FIELDS=${NUCLEUS_ACCESSORY_LAYOUT_FIELDS}
    NUCLEUS_ACCESSORY_SCHEMA_FIELDS=${NUCLEUS_ACCESSORY_SCHEMA_FIELDS}
    NUCLEUS_ACCESSORY_CATALOG_BYTES=${NUCLEUS_ACCESSORY_CATALOG_BYTES})

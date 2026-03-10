# Mesh architecture: ROOT / RELAY / LEAF

Canonical reference for who does what. Use these task names when discussing code or behavior.

---

## 1. ROOT

- **Many tasks**, but **normally only TX in free slot** (downlink).
- **Root free-slot downlink:**
  - Resync tier-1 (SYNC + TDMA when needed).
  - Periodic SYNC unicast to tier-1 (every 21st cycle); relay then downlinks SYNC to its children in the next slot.

*(Code: `components/root_cli/root_cli.c` — root_sync_task, free slot only.)*

---

## 2. RELAY

- **Downlink:** Notify children with info received from parent in last cycle’s free slot (e.g. TDMA, SYNC).
- **Uplink:** Send own DATA packet to parent.
- **Uplink:** Forward children’s packets to parent (drain forward queue after own DATA).

*(Code: `components/mesh/mesh.c` — slot loop when `has_children`.)*

---

## 3. LEAF

- **Uplink:** Send own DATA packet to parent.
- **Uplink:** Request info/changes (later); for now same path as relay forward (queue to parent).

*(Code: `components/mesh/mesh.c` — slot loop when `!has_children`.)*

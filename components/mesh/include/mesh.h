#pragma once

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 *  Mesh Packet Types
 * ============================================================ */

typedef enum {
    MESH_PKT_JOIN = 0,  // Join request
    MESH_PKT_SYNC = 1,  // Synchronization packet
    MESH_PKT_ASSIGN = 2, // Assign topology packet
    MESH_PKT_TDMA = 3,  // TDMA sync packet
    MESH_PKT_DATA = 4   // Data packet
} mesh_pkt_type_t;

/* ============================================================
 *  TDMA Payload
 * ============================================================ */

/* TDMA payload: supports variable slots per cycle (e.g. cycles 1-9: 2 slots, cycle 10: 3 slots). */
typedef struct {
    uint64_t anchor_epoch_ms;        /* Start of first base cycle (epoch ms). */
    uint32_t base_period_ms;         /* Length of one base TDMA cycle (e.g. 60000 ms). */
    uint32_t slot_duration_ms;       /* Duration of this node's slot when it reports. */
    uint16_t full_cycle_cycles;      /* Base cycles in one full (hyper) period (e.g. 10). */
    uint16_t report_every_n_cycles;  /* Report once every N base cycles (1 = every cycle). */
    uint8_t  report_cycle_offset;    /* Report when (cycle_index % report_every_n_cycles) == this. */
    uint8_t  slot_index;             /* Slot index when this node reports (0 = first, etc.). */
    uint32_t schedule_hash;          /* Deterministic TDMA schedule identity (root-computed). */
} __attribute__((packed)) mesh_tdma_payload_t;

/* ============================================================
 *  Public API
 * ============================================================ */

uint16_t mesh_get_root_id(void);

void mesh_start_runtime(void);

/**
 * @brief Initialize mesh layer.
 *
 * Must be called once at startup.
 * Initializes PHY, NET, routing callbacks,
 * and internal mesh state.
 */
void mesh_init(void);


bool ping(uint16_t target);
void restart_root(void);
void reset_lora(void);
/** Return radio to RX continuous (e.g. after root CLI ping so root can receive again). */
void mesh_start_rx_continuous(void);

/**
 * @brief Get current mesh node ID.
 *
 * @return Node ID assigned from MAC mapping table.
 */
uint16_t mesh_get_node_id(void);


/**
 * @brief Send a mesh packet.
 *
 * For unicast packets, this waits for MAC-level ACK (with retries inside
 * the MAC layer) and returns true on success, false on timeout. For
 * broadcast packets (SYNC, JOIN), it always returns true.
 *
 * @param type     Mesh packet type
 * @param payload  Pointer to payload buffer
 * @param len      Payload length
 * @return true on successful unicast delivery (ACK received) or broadcast queued, false on failure.
 */
bool mesh_send(mesh_pkt_type_t type,
               uint8_t *payload,
               uint16_t len);

/**
 * @brief Send a mesh packet to a specific destination (directed unicast via NET layer).
 *
 * @param dst      Final destination node ID
 * @param type     Mesh packet type
 * @param payload  Pointer to payload buffer
 * @param len      Payload length
 */
bool mesh_send_to(uint16_t dst, mesh_pkt_type_t type,
                  uint8_t *payload, uint16_t len);

/* ============================================================
 *  Node Status Table (root)
 * ============================================================ */

/** Non-root node state at root: OFFLINE, ONLINE (reachable), RUNNING (sending DATA). */
typedef enum {
    MESH_NODE_OFFLINE = 0,
    MESH_NODE_ONLINE  = 1,
    MESH_NODE_RUNNING = 2,
} mesh_node_state_t;

bool mesh_is_node_online(uint16_t node_id);
void mesh_set_node_online(uint16_t node_id, bool online);
/** Get current state; returns MESH_NODE_OFFLINE if node_id out of range. */
mesh_node_state_t mesh_get_node_state(uint16_t node_id);
/** Set state and log change if different from current (root only). */
void mesh_set_node_state(uint16_t node_id, mesh_node_state_t state);
int64_t mesh_get_node_last_seen(uint16_t node_id);

/** Set payload to send in RUNNING state during this node's TDMA slot (max 64 bytes). */
void mesh_set_app_payload(const uint8_t *data, uint16_t len);

/** Optional callback when root receives a DATA packet (for cycle tracking / CSV). */
typedef void (*mesh_data_at_root_cb_t)(uint16_t node_id, const uint8_t *payload, uint16_t len, int rssi, float snr);
void mesh_register_data_at_root_cb(mesh_data_at_root_cb_t cb);

/** Optional callback when root receives JOIN (node_id, interval_byte). Root may set online and decide remesh. */
void mesh_register_join_callback(void (*cb)(uint16_t node_id, uint8_t interval_byte));


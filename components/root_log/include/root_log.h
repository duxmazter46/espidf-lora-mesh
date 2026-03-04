#pragma once

#include <stdint.h>

typedef enum {
    ROOT_LOG_MESH_START = 0,
    ROOT_LOG_TDMA_ENTRY = 1,
    ROOT_LOG_TOPOLOGY_NODE = 2,
    ROOT_LOG_DATA_RX = 3,
    ROOT_LOG_NODE_STATUS = 4,
} root_log_event_type_t;

/** Non-root node state as seen by root (for NODE_STATUS and status reporting). */
typedef enum {
    ROOT_LOG_NODE_OFFLINE = 0,
    ROOT_LOG_NODE_ONLINE  = 1,
    ROOT_LOG_NODE_RUNNING = 2,
} root_log_node_state_t;

typedef struct {
    uint8_t  type;          /* root_log_event_type_t */
    uint16_t node_id;       /* node involved (if any) */
    uint16_t parent_id;     /* topology parent (if any) */
    uint16_t payload_len;   /* data payload length (for DATA_RX) */

    uint8_t  slot_index;    /* TDMA slot index (if any) */
    uint8_t  total_slots;   /* TDMA total slots (if any) */

    uint32_t slot_ms;       /* TDMA slot duration (ms) */

    uint64_t ts_ms;         /* log timestamp (epoch ms) */
    uint64_t anchor_ms;     /* TDMA anchor or mesh start (epoch ms) */

    int16_t  rssi;          /* RX RSSI dBm (for DATA_RX) */
    int16_t  snr_times_10;  /* RX SNR * 10 (for DATA_RX) */
} __attribute__((packed)) root_log_entry_t;

void root_log_init(void);

void root_log_mesh_start(void);

void root_log_topology_node(uint16_t node_id,
                            uint16_t parent_id,
                            const uint16_t *children,
                            uint8_t child_count);

void root_log_tdma_entry(uint16_t node_id,
                         uint8_t slot_index,
                         uint8_t total_slots,
                         uint32_t slot_ms,
                         uint64_t anchor_ms);

void root_log_data_rx(uint16_t src_node_id,
                      uint16_t payload_len,
                      int rssi,
                      float snr);

/** Log a non-root node state change (OFFLINE / ONLINE / RUNNING). */
void root_log_node_status(uint16_t node_id, root_log_node_state_t state);

void root_log_dump(void);

/** Print only the last n entries (oldest to newest of that slice). n=0 means all. */
void root_log_dump_tail(unsigned n);

/** Print summary: total count, counts per type, DATA_RX count per node. */
void root_log_dump_summary(void);

void root_log_clear(void);


#pragma once
#include <stdint.h>
#include <stdbool.h>

/* =========================
   Limits
========================= */

#define NODECFG_MAX_CHILDREN            5
#define NODECFG_MAX_CONSECUTIVE_FAIL    3

/* =========================
   Routing Policy
========================= */

typedef enum {
    ROUTING_STRICT_TREE = 0,
    ROUTING_LOOSE_TREE,
    ROUTING_OPPORTUNISTIC_FORWARD,
    ROUTING_BEST_LINK,
    ROUTING_FLOOD
} routing_policy_type_t;

/* =========================
   Topology Structure
========================= */

typedef struct {
    uint16_t parent_id;
    uint16_t children[NODECFG_MAX_CHILDREN];
    uint8_t  child_count;
} nodecfg_topology_t;

/* =========================
   Public API
========================= */

uint16_t nodecfg_get_node_id(void);

uint16_t nodecfg_get_root_id(void);

routing_policy_type_t nodecfg_get_routing_policy(void);

uint8_t nodecfg_get_max_children(void);

uint8_t nodecfg_get_max_failures(void);

/** True if this firmware has static topology (node knows own id/topo at boot).
 *  If false, non-root nodes wait for ASSIGN packet from root before proceeding. */
bool nodecfg_has_static_topology(void);

void nodecfg_get_topology(uint16_t node_id,
                          nodecfg_topology_t *topo);

/** Per-node reporting interval in seconds (e.g. 60 = 1 min, 600 = 10 min). Default 60. */
uint32_t nodecfg_get_reporting_interval_s(uint16_t node_id);

uint16_t nodecfg_get_topo_table_size(void);
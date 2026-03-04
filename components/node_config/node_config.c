#include "node_config.h"
#include "esp_mac.h"
#include <string.h>

/* Set to 1 if this build has static topology (node knows own id/topo).
 * Set to 0 so non-root nodes wait for ASSIGN from root (no LoRa reset/restart loop). */
#define NODECFG_STATIC_TOPOLOGY_FLASHED  1

/* Default reporting interval (seconds); base period when all nodes use same interval. */
#define REPORTING_INTERVAL_S 60

typedef struct {
    uint8_t mac[6];
    uint16_t node_id;
} node_map_t;

static const node_map_t node_map[] = {

    {{0x20,0x6E,0xF1,0xC3,0x0A,0xF4}, 0},
    {{0x20,0x6E,0xF1,0xC3,0x0B,0x34}, 1},
    {{0x20,0x6E,0xF1,0xC3,0x0A,0xD4}, 2},
    {{0x20,0x6E,0xF1,0xC3,0x0B,0x48}, 5},
    {{0x20,0x6E,0xF1,0xC3,0x0B,0x1C}, 6},
    {{0x20,0x6E,0xF1,0xCA,0x35,0xB0}, 7},
    {{0x20,0x6E,0xF1,0xCA,0x35,0xBC}, 8},
    {{0x20,0x6E,0xF1,0xC3,0x0A,0x8C}, 9},
    {{0x20,0x6E,0xF1,0xCA,0x35,0x94}, 10},
    {{0x20,0x6E,0xF1,0xC3,0x0B,0x60}, 11},
    {{0x20,0x6E,0xF1,0xC3,0x0A,0xF0}, 12},
    {{0x20,0x6E,0xF1,0xC3,0x0A,0x78}, 13},
    {{0x20,0x6E,0xF1,0xC3,0x8A,0xA8}, 14},
    {{0x20,0x6E,0xF1,0xCA,0x35,0x9C}, 15},
    {{0x20,0x6E,0xF1,0xCA,0x35,0x88}, 16},
    {{0x20,0x6E,0xF1,0xC3,0x0A,0xDC}, 17},
    {{0x20,0x6E,0xF1,0xC3,0x8A,0x8C}, 18},
    {{0x20,0x6E,0xF1,0xC3,0x0B,0x08}, 19},

    //Lab setup
    {{0x20,0x6E,0xF1,0xC3,0x0A,0xC8}, 20},
    {{0x20,0x6E,0xF1,0xCB,0xE8,0xDC}, 21},
 
};

static const uint16_t ROOT_NODE_ID = 0;

static const routing_policy_type_t ROUTING_POLICY =
    ROUTING_STRICT_TREE;


    static const nodecfg_topology_t topo_table[] = {

    [0]  = { .parent_id = 0xFFFF, .children = {1, 2}, .child_count = 2 },
    [1]  = { .parent_id = 0, .child_count = 0 },
    [2]  = { .parent_id = 0, .child_count = 0 },

    //Lab setup
    //[21]  = { .parent_id = 0xFFFF, .children = {20}, .child_count = 1 },

    /*[7]  = { .parent_id = 10, .children = {8}, .child_count = 1 },
    [8]  = { .parent_id = 7, .child_count = 0 },
    [10] = { .parent_id = 0xFFFF, .children = {7}, .child_count = 1 },
    */
};


/* Per-node reporting interval in seconds (0 = use default REPORTING_INTERVAL_S).
 * Example: node 1 = 60 (1 min), node 2 = 60 or 600 (10 min) for variable slots per cycle.
 * Current setup: all 60 so every cycle has same number of slots (node1, node2, free). */
static const uint32_t node_reporting_interval_s[] = {
    [0] = 0,   /* root */
    [1] = 60,  /* 1 minute */
    [2] = 60,  /* 1 minute (use 600 for 10-min: cycles 1-9 = 2 slots, cycle 10 = 3 slots) */
};

uint16_t nodecfg_get_node_id(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    for (int i = 0; i < sizeof(node_map)/sizeof(node_map[0]); i++) {
        if (memcmp(mac, node_map[i].mac, 6) == 0) {
            return node_map[i].node_id;
        }
    }

    return 0xFFFF;
}

uint16_t nodecfg_get_root_id(void)
{
    return ROOT_NODE_ID;
}

routing_policy_type_t nodecfg_get_routing_policy(void)
{
    return ROUTING_POLICY;
}

uint8_t nodecfg_get_max_children(void)
{
    return NODECFG_MAX_CHILDREN;
}

uint8_t nodecfg_get_max_failures(void)
{
    return NODECFG_MAX_CONSECUTIVE_FAIL;
}

bool nodecfg_has_static_topology(void)
{
    return (bool)NODECFG_STATIC_TOPOLOGY_FLASHED;
}

void nodecfg_get_topology(uint16_t node_id,
                          nodecfg_topology_t *topo)
{
    uint16_t table_size =
        sizeof(topo_table) / sizeof(topo_table[0]);

    /* Out of table range */
    if (node_id >= table_size) {
        abort();  // TDMA test should not allow unknown node
    }

    *topo = topo_table[node_id];
}

uint32_t nodecfg_get_reporting_interval_s(uint16_t node_id)
{
    uint16_t table_size =
        sizeof(node_reporting_interval_s) / sizeof(node_reporting_interval_s[0]);

    if (node_id < table_size) {
        uint32_t v = node_reporting_interval_s[node_id];
        if (v != 0)
            return v;
    }
    return (uint32_t)REPORTING_INTERVAL_S;
}

uint16_t nodecfg_get_topo_table_size(void)
{
    return sizeof(topo_table) / sizeof(topo_table[0]);
}

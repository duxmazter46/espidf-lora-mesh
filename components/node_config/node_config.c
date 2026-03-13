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
    {{0x20,0x6E,0xF1,0xC3,0x0A,0xC8}, 20},
    {{0x20,0x6E,0xF1,0xCB,0xE8,0xDC}, 21},
 
};

static const uint16_t ROOT_NODE_ID = 7;

static const routing_policy_type_t ROUTING_POLICY = ROUTING_STRICT_TREE;


    /* Strict tree: root → node1 → node2 (node2 only via node1). */
static const nodecfg_topology_t topo_table[] = {

    [7]  = { .parent_id = 0xFFFF, .children = {18}, .child_count = 1 },
    [18]  = { .parent_id = 7, .children = {19}, .child_count = 1 },
    [19]  = { .parent_id = 18, .child_count = 0 },

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
    [7] = 0,   /* root */
    [18] = 60,  /* 1 minute */
    [19] = 60,  /* 1 minute (use 600 for 10-min: cycles 1-9 = 2 slots, cycle 10 = 3 slots) */
};

/* Power policy: default for all nodes. Manual edit per node in node_power_policy[] below if needed.
 * FULL = no sleep, idle when not in slot. BATTERY_SAVER = light sleep, wake on free slot.
 * VERY_BATTERY_SAVER = deep sleep, wake on free slot. MINIMAL_WAKE = deep sleep, wake only on root resync. */
#define NODECFG_POWER_POLICY_DEFAULT  NODECFG_POWER_FULL

static const uint8_t node_power_policy[] = {
    [0] = NODECFG_POWER_FULL,  /* root (unused) */
    [1] = NODECFG_POWER_POLICY_DEFAULT,
    [2] = NODECFG_POWER_POLICY_DEFAULT,
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

static bool target_in_subtree(uint16_t node_id, uint16_t target, uint16_t table_size)
{
    if (node_id >= table_size || node_id == 0xFFFF)
        return false;
    if (node_id == target)
        return true;
    nodecfg_topology_t topo;
    topo = topo_table[node_id];
    for (uint8_t i = 0; i < topo.child_count; i++) {
        if (target_in_subtree(topo.children[i], target, table_size))
            return true;
    }
    return false;
}

uint16_t nodecfg_get_first_hop_toward(uint16_t from_node, uint16_t target)
{
    uint16_t table_size = (uint16_t)(sizeof(topo_table) / sizeof(topo_table[0]));
    if (from_node >= table_size || target >= table_size)
        return 0xFFFF;
    if (from_node == target)
        return target;
    nodecfg_topology_t topo;
    topo = topo_table[from_node];
    for (uint8_t i = 0; i < topo.child_count; i++) {
        uint16_t c = topo.children[i];
        if (c == target)
            return c;
        if (target_in_subtree(c, target, table_size))
            return c;
    }
    return 0xFFFF;
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

uint8_t nodecfg_get_power_policy(uint16_t node_id)
{
    uint16_t table_size =
        sizeof(node_power_policy) / sizeof(node_power_policy[0]);
    if (node_id < table_size)
        return node_power_policy[node_id];
    return (uint8_t)NODECFG_POWER_POLICY_DEFAULT;
}

uint16_t nodecfg_get_topo_table_size(void)
{
    return sizeof(topo_table) / sizeof(topo_table[0]);
}

#define NODECFG_TOPO_LIST_MAX 32
static uint16_t s_topo_node_list[NODECFG_TOPO_LIST_MAX];
static uint16_t s_topo_node_count;
static bool s_topo_node_list_valid;

static void build_topo_node_list(void)
{
    uint16_t table_size = (uint16_t)(sizeof(topo_table) / sizeof(topo_table[0]));
    uint16_t root_id = ROOT_NODE_ID;
    if (root_id >= table_size) {
        s_topo_node_count = 0;
        s_topo_node_list_valid = true;
        return;
    }
    uint16_t queue[NODECFG_TOPO_LIST_MAX];
    int head = 0, tail = 0;
    queue[tail++] = root_id;
    s_topo_node_count = 0;
    while (head < tail && s_topo_node_count < NODECFG_TOPO_LIST_MAX) {
        uint16_t n = queue[head++];
        s_topo_node_list[s_topo_node_count++] = n;
        const nodecfg_topology_t *t = &topo_table[n];
        for (uint8_t i = 0; i < t->child_count && i < NODECFG_MAX_CHILDREN; i++) {
            uint16_t c = t->children[i];
            if (c < table_size && (unsigned)tail < NODECFG_TOPO_LIST_MAX)
                queue[tail++] = c;
        }
    }
    s_topo_node_list_valid = true;
}

uint16_t nodecfg_get_topo_node_count(void)
{
    if (!s_topo_node_list_valid)
        build_topo_node_list();
    return s_topo_node_count;
}

uint16_t nodecfg_get_topo_node_id(uint16_t index)
{
    if (!s_topo_node_list_valid)
        build_topo_node_list();
    if (index >= s_topo_node_count)
        return 0xFFFF;
    return s_topo_node_list[index];
}

bool nodecfg_is_direct_child(uint16_t parent_id, uint16_t node_id)
{
    uint16_t table_size = (uint16_t)(sizeof(topo_table) / sizeof(topo_table[0]));
    if (parent_id >= table_size)
        return false;
    const nodecfg_topology_t *t = &topo_table[parent_id];
    for (uint8_t i = 0; i < t->child_count; i++) {
        if (t->children[i] == node_id)
            return true;
    }
    return false;
}

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <time.h>

#include "mesh.h"
#include "node_config.h"
#include "root_cli.h"
#include "root_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <math.h>

#define ROOT_CSV_MAX_PAYLOAD 64
#define ROOT_CSV_MAX_ROWS    512

#define TAG "ROOT_CLI"

/* ROOT role (see docs/ARCHITECTURE.md): many tasks; normally only TX in free slot (downlink: resync, periodic SYNC). */

/* Drift model: SYNC every N cycles so relative drift stays under DRIFT_BOUND_MS.
   DRIFT_PPM = worst-case relative clock drift (ppm); 40 = 20 ppm per node × 2. */
#define DRIFT_BOUND_MS  50
#define DRIFT_PPM       40

/** Returns ceil(DRIFT_BOUND_MS / (DRIFT_PPM * 1e-6 * cycle_ms)); min 1. */
static uint32_t root_sync_every_cycles(uint32_t cycle_ms)
{
    double drift_per_cycle_ms = (double)DRIFT_PPM * 1e-6 * (double)cycle_ms;
    if (drift_per_cycle_ms <= 0)
        return 1;
    double n = (double)DRIFT_BOUND_MS / drift_per_cycle_ms;
    uint32_t cycles = (uint32_t)ceil(n);
    return (cycles < 1) ? 1 : cycles;
}

#define ROOT_MAX_SLOT_NODES 32
#define ROOT_MAX_FULL_CYCLE  64  /* max base cycles in one full (hyper) period */

static uint32_t gcd_u32(uint32_t a, uint32_t b)
{
    while (b != 0) {
        uint32_t t = b;
        b = a % b;
        a = t;
    }
    return a;
}

static uint32_t lcm_u32(uint32_t a, uint32_t b)
{
    if (a == 0 || b == 0)
        return 0;
    return (a / gcd_u32(a, b)) * b;
}

/* Per-cycle: which nodes sent DATA (set by data-at-root callback). */
static bool received_this_cycle[ROOT_MAX_SLOT_NODES];

/* Per-cycle: last DATA payload from each node (for end-of-cycle CSV log). */
#define CYCLE_DATA_PAYLOAD_MAX  64
static uint64_t cycle_data_ts_ms[ROOT_MAX_SLOT_NODES];
static uint8_t  cycle_data_payload[ROOT_MAX_SLOT_NODES][CYCLE_DATA_PAYLOAD_MAX];
static uint16_t cycle_data_len[ROOT_MAX_SLOT_NODES];

/* Per-node resync retry count (free-slot healing). Index by node id; reset on DATA or when marking ONLINE. */
static uint8_t resync_retry_count[ROOT_MAX_SLOT_NODES];

/* Snapshot of node ids in current TDMA schedule (for JOIN callback: reuse TDMA vs remesh). */
static uint16_t current_schedule_node_ids[ROOT_MAX_SLOT_NODES];
static uint16_t num_current_schedule = 0;

/* Set when JOIN reports different reporting interval; sync task runs remesh at start of free slot. */
static volatile bool remesh_pending = false;

/* One-time "all online" notification; reset when commander runs "start" so we can notify again after remesh. */
static bool all_online_notified = false;

/* Consecutive cycles with no DATA from any child; used to reset LoRa or mesh. */
static uint8_t s_cycles_no_pkt = 0;

/* TDMA table: for each base cycle in the full period, which nodes report and their slot index.
 * tdma_cycle_nodes[c][0..count-1] = node ids; count = tdma_cycle_slot_count[c] - 1 (last slot is free). */
static uint16_t tdma_cycle_nodes[ROOT_MAX_FULL_CYCLE][ROOT_MAX_SLOT_NODES];
static uint8_t  tdma_cycle_slot_count[ROOT_MAX_FULL_CYCLE];
static uint16_t tdma_full_cycle_cycles;

/* Stored when TDMA starts so "cycle" command can show current cycle. */
static uint64_t s_anchor_epoch_ms = 0;
static uint32_t s_base_period_ms  = 0;
static uint16_t s_full_cycle_cycles = 0;
static uint32_t s_tdma_hash = 0;
/* Bootstrap/runtime TDMA parameters (anchor for runtime and number of bootstrap base cycles). */
static uint16_t s_bootstrap_cycles = 1;          /* default: 1 bootstrap cycle */
static uint64_t s_runtime_anchor_epoch_ms = 0;   /* computed from anchor + bootstrap_cycles * base_period_ms */

static uint32_t tdma_compute_hash(void)
{
    uint32_t h = 5381U;

    h = ((h << 5) + h) ^ s_base_period_ms;
    h = ((h << 5) + h) ^ s_full_cycle_cycles;

    for (uint16_t c = 0; c < s_full_cycle_cycles && c < ROOT_MAX_FULL_CYCLE; c++) {
        uint8_t n = tdma_cycle_slot_count[c];
        h = ((h << 5) + h) ^ n;

        if (n == 0) {
            continue;
        }

        for (uint8_t i = 0; i < (uint8_t)(n - 1) && i < ROOT_MAX_SLOT_NODES; i++) {
            h = ((h << 5) + h) ^ tdma_cycle_nodes[c][i];
        }
    }

    return h;
}

/* Periodic SYNC in free slot: task parameters and handle */
typedef struct {
    uint64_t anchor_epoch_ms;
    uint32_t base_period_ms;       /* one base TDMA cycle length */
    uint32_t sync_every_cycles;
    uint16_t slot_to_node[ROOT_MAX_SLOT_NODES];
    uint16_t num_online;
    uint16_t full_cycle_cycles;
    uint8_t  slot_count_per_cycle[ROOT_MAX_FULL_CYCLE];  /* slots in each cycle (nodes + 1 free) */
} root_sync_task_params_t;

static TaskHandle_t root_sync_task_handle = NULL;

/** Build TDMA payload for a node using current schedule (s_*, tdma_cycle_*). Returns true if node in schedule. */
static bool root_build_tdma_payload_for_node(uint16_t nid, mesh_tdma_payload_t *out)
{
    if (s_base_period_ms == 0 || s_full_cycle_cycles == 0)
        return false;
    uint32_t base_period_s = s_base_period_ms / 1000;
    if (base_period_s == 0)
        base_period_s = 1;
    uint32_t interval_s = nodecfg_get_reporting_interval_s(nid);
    uint16_t report_every = (uint16_t)(interval_s / base_period_s);
    if (report_every == 0)
        report_every = 1;
    uint8_t report_cycle_offset = (uint8_t)(report_every - 1);

    uint8_t slot_index = 0;
    uint32_t slot_duration_ms = s_base_period_ms;
    bool found = false;
    for (uint16_t c = 0; c < s_full_cycle_cycles && c < ROOT_MAX_FULL_CYCLE; c++) {
        if ((c % report_every) != report_cycle_offset)
            continue;
        uint8_t n_slots = tdma_cycle_slot_count[c];
        slot_duration_ms = (n_slots > 0) ? (s_base_period_ms / n_slots) : s_base_period_ms;
        for (uint8_t k = 0; k < n_slots - 1 && k < ROOT_MAX_SLOT_NODES; k++) {
            if (tdma_cycle_nodes[c][k] == nid) {
                slot_index = k;
                found = true;
                break;
            }
        }
        break;
    }
    if (!found)
        return false;

    out->anchor_epoch_ms       = s_anchor_epoch_ms;
    out->base_period_ms        = s_base_period_ms;
    out->slot_duration_ms      = slot_duration_ms;
    out->full_cycle_cycles     = s_full_cycle_cycles;
    out->report_every_n_cycles = report_every;
    out->report_cycle_offset   = report_cycle_offset;
    out->slot_index            = slot_index;
    out->schedule_hash         = s_tdma_hash;
    out->bootstrap_cycles      = s_bootstrap_cycles;
    out->runtime_anchor_epoch_ms = s_runtime_anchor_epoch_ms;
    return true;
}

/* Forward declare so join callback can set remesh_pending; sync task will call it. */
static void cmd_start(int argc, char **argv);
static void root_sync_task(void *arg);

/* True if static topology is used and every non-root node in the topology tree is ONLINE. */
static bool root_all_nodes_in_topo_online(void)
{
    if (!nodecfg_has_static_topology())
        return false;
    uint16_t topo_size = nodecfg_get_topo_table_size();
    uint16_t root_id  = nodecfg_get_root_id();
    for (uint16_t i = 0; i < topo_size; i++) {
        if (i == root_id)
            continue;
        nodecfg_topology_t t;
        nodecfg_get_topology(i, &t);
        if (t.parent_id == 0xFFFF && t.child_count == 0)
            continue;
        if (!mesh_is_node_online(i))
            return false;
    }
    return true;
}

static void root_join_cb(uint16_t node_id, uint8_t interval_byte)
{
    mesh_set_node_online(node_id, true);
    if (node_id < ROOT_MAX_SLOT_NODES)
        resync_retry_count[node_id] = 0;

    /* Notify commander when all nodes in static topology are ONLINE (once per start; reset by "start"). */
    if (nodecfg_has_static_topology() && !all_online_notified && root_all_nodes_in_topo_online()) {
        all_online_notified = true;
        ESP_LOGW(TAG, "COMMANDER: All nodes in topology are ONLINE.");
    }

    bool in_schedule = false;
    for (uint16_t i = 0; i < num_current_schedule; i++) {
        if (current_schedule_node_ids[i] == node_id) {
            in_schedule = true;
            break;
        }
    }
    if (in_schedule) {
        uint32_t config_interval = nodecfg_get_reporting_interval_s(node_id);
        uint8_t config_byte = (config_interval > 255) ? 255 : (uint8_t)config_interval;
        if (interval_byte != config_byte) {
            remesh_pending = true;
            ESP_LOGI(TAG, "JOIN from node %u: interval changed (%u -> config %lu) -> remesh pending",
                     (unsigned)node_id, (unsigned)interval_byte, (unsigned long)config_interval);
        }
    }
}

/** Rebuild TDMA from current schedule (reporting intervals may have changed) and restart sync task. Called from sync task when remesh_pending. */
static void root_do_remesh(void)
{
    remesh_pending = false;
    uint16_t num_online = num_current_schedule;
    if (num_online == 0)
        return;

    /* Static to avoid large stack use inside root_sync_task (prevents stack overflow when JOIN triggers remesh). */
    static uint16_t online_ids[ROOT_MAX_SLOT_NODES];
    for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES; i++)
        online_ids[i] = current_schedule_node_ids[i];

    uint64_t epoch_ms = root_get_epoch_ms();
    uint32_t base_period_s = nodecfg_get_reporting_interval_s(online_ids[0]);
    for (uint16_t i = 1; i < num_online; i++) {
        uint32_t iv = nodecfg_get_reporting_interval_s(online_ids[i]);
        if (iv < base_period_s)
            base_period_s = iv;
    }
    uint32_t full_period_s = nodecfg_get_reporting_interval_s(online_ids[0]);
    for (uint16_t i = 1; i < num_online; i++) {
        full_period_s = lcm_u32(full_period_s, nodecfg_get_reporting_interval_s(online_ids[i]));
    }
    uint16_t full_cycle_cycles = (uint16_t)(full_period_s / base_period_s);
    if (full_cycle_cycles == 0)
        full_cycle_cycles = 1;
    if (full_cycle_cycles > ROOT_MAX_FULL_CYCLE)
        full_cycle_cycles = ROOT_MAX_FULL_CYCLE;

    uint32_t base_period_ms = base_period_s * 1000;
    uint64_t anchor_epoch_ms = ((epoch_ms / (base_period_s * 1000)) + 1) * (uint64_t)base_period_s * 1000;
    if (base_period_s >= 60)
        anchor_epoch_ms = ((epoch_ms / 60000) + 1) * 60000;

    tdma_full_cycle_cycles = full_cycle_cycles;
    for (uint16_t c = 0; c < full_cycle_cycles; c++) {
        uint8_t idx = 0;
        for (uint16_t i = 0; i < num_online && idx < ROOT_MAX_SLOT_NODES; i++) {
            uint16_t nid = online_ids[i];
            uint32_t interval_s = nodecfg_get_reporting_interval_s(nid);
            uint16_t report_every = (uint16_t)(interval_s / base_period_s);
            if (report_every == 0)
                report_every = 1;
            uint8_t report_cycle_offset = (uint8_t)(report_every - 1);
            if ((c % report_every) == report_cycle_offset) {
                tdma_cycle_nodes[c][idx++] = nid;
            }
        }
        tdma_cycle_slot_count[c] = idx + 1;
    }

    s_tdma_hash = tdma_compute_hash();
    s_anchor_epoch_ms   = anchor_epoch_ms;
    s_base_period_ms    = base_period_ms;
    s_full_cycle_cycles = full_cycle_cycles;

    for (uint16_t i = 0; i < num_online; i++) {
        uint16_t nid = online_ids[i];
        mesh_tdma_payload_t tdma;
        if (root_build_tdma_payload_for_node(nid, &tdma))
            mesh_send_to(nid, MESH_PKT_TDMA, (uint8_t *)&tdma, sizeof(tdma));
        vTaskDelay(pdMS_TO_TICKS(400));
    }

    uint32_t sync_every = root_sync_every_cycles(base_period_ms);
    memset(received_this_cycle, 0, sizeof(received_this_cycle));
    for (uint16_t i = 0; i < ROOT_MAX_SLOT_NODES; i++)
        resync_retry_count[i] = 0;

    static root_sync_task_params_t remesh_sync_params;
    remesh_sync_params.anchor_epoch_ms    = anchor_epoch_ms;
    remesh_sync_params.base_period_ms     = base_period_ms;
    remesh_sync_params.sync_every_cycles  = sync_every;
    remesh_sync_params.num_online         = num_online;
    remesh_sync_params.full_cycle_cycles  = full_cycle_cycles;
    for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES; i++)
        remesh_sync_params.slot_to_node[i] = online_ids[i];
    for (uint16_t c = 0; c < full_cycle_cycles && c < ROOT_MAX_FULL_CYCLE; c++)
        remesh_sync_params.slot_count_per_cycle[c] = tdma_cycle_slot_count[c];

    ESP_LOGI(TAG, "Remesh: new TDMA hash 0x%08X, creating new sync task", (unsigned)s_tdma_hash);
    TaskHandle_t new_handle = NULL;
    xTaskCreate(root_sync_task, "root_sync", 4096, &remesh_sync_params, 3, &new_handle);
    root_sync_task_handle = new_handle;
    vTaskDelete(NULL);
}

static const char *mesh_node_state_str(mesh_node_state_t s)
{
    switch (s) {
    case MESH_NODE_OFFLINE: return "OFFLINE";
    case MESH_NODE_ONLINE:  return "ONLINE";
    case MESH_NODE_RUNNING: return "RUNNING";
    default: return "?";
    }
}

static void root_sync_task(void *arg)
{
    root_sync_task_params_t *p = (root_sync_task_params_t *)arg;
    uint16_t root_id      = nodecfg_get_root_id();
    uint64_t anchor       = p->anchor_epoch_ms;
    uint32_t base_period_ms = p->base_period_ms;
    uint32_t every        = p->sync_every_cycles;
    uint16_t num_online   = p->num_online;
    uint16_t full_cycles  = p->full_cycle_cycles;

    ESP_LOGI(TAG, "Periodic SYNC task: base_period=%lu ms, full_cycles=%u, every %lu cycles",
             (unsigned long)base_period_ms, (unsigned)full_cycles, (unsigned long)every);

    for (;;) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t now_ms = (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
        if (now_ms >= anchor)
            break;
        uint64_t delay_ms = anchor - now_ms;
        if (delay_ms > 0)
            vTaskDelay(pdMS_TO_TICKS((TickType_t)(delay_ms > 30000 ? 30000 : delay_ms)));
    }

    uint32_t cycle_index = 0;
    for (;;) {
        uint16_t c = (uint16_t)(cycle_index % full_cycles);
        uint8_t n_slots = p->slot_count_per_cycle[c];
        uint32_t slot_ms = (n_slots > 0) ? (base_period_ms / n_slots) : base_period_ms;
        uint8_t free_ix = (n_slots > 0) ? (n_slots - 1) : 0;
        uint64_t free_slot_start = anchor + (uint64_t)cycle_index * base_period_ms + (uint64_t)free_ix * slot_ms;

        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t now_ms = (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;

        if (now_ms < free_slot_start) {
            uint64_t delay_ms = free_slot_start - now_ms;
            if (delay_ms > 0)
                vTaskDelay(pdMS_TO_TICKS((TickType_t)(delay_ms > 30000 ? 30000 : delay_ms)));
            continue;
        }

        /* We are now in the free slot for this cycle (ROOT free-slot downlink only). */

        if (remesh_pending) {
            root_do_remesh();
            return;
        }

        /* Re-sync tier-1 nodes only (strict tree: root does not PING/SYNC tier-2; parents do). */
        {
            uint64_t epoch_ms = root_get_epoch_ms();
            for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES; i++) {
                uint16_t nid = p->slot_to_node[i];
                if (nid >= ROOT_MAX_SLOT_NODES)
                    continue;
                if (mesh_get_node_state(nid) != MESH_NODE_ONLINE)
                    continue;
                if (resync_retry_count[nid] >= 3)
                    continue;
                /* Only resync direct children of root (tier-1). */
                if (!nodecfg_is_direct_child(root_id, nid))
                    continue;
                bool ack = mesh_send_to(nid, MESH_PKT_SYNC, (uint8_t *)&epoch_ms, sizeof(epoch_ms));
                if (ack) {
                    mesh_tdma_payload_t tdma;
                    if (root_build_tdma_payload_for_node(nid, &tdma)) {
                        mesh_send_to(nid, MESH_PKT_TDMA, (uint8_t *)&tdma, sizeof(tdma));
                        ESP_LOGI(TAG, "Resync node %u: SYNC+TDMA sent", (unsigned)nid);
                        ESP_LOGW(TAG, "COMMANDER: Node %u resynced (SYNC+TDMA sent).", (unsigned)nid);
                    }
                    resync_retry_count[nid] = 0;
                } else {
                    resync_retry_count[nid]++;
                    ESP_LOGW(TAG, "Resync node %u: no ACK (attempt %u/3)", (unsigned)nid, (unsigned)resync_retry_count[nid]);
                    if (resync_retry_count[nid] >= 3) {
                        ESP_LOGW(TAG, "COMMANDER: Node %u resync failed after 3 attempts.", (unsigned)nid);
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(400));
            }
        }

        /* Periodic SYNC: only in free slot and only every 21st cycle; unicast to tier-1 only; relay downlinks SYNC in next slot. */
        if (cycle_index % every == 0) {
            uint64_t epoch_ms = root_get_epoch_ms();
            for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES; i++) {
                uint16_t nid = p->slot_to_node[i];
                if (nid >= ROOT_MAX_SLOT_NODES)
                    continue;
                if (!nodecfg_is_direct_child(root_id, nid))
                    continue;
                mesh_send_to(nid, MESH_PKT_SYNC, (uint8_t *)&epoch_ms, sizeof(epoch_ms));
                vTaskDelay(pdMS_TO_TICKS(400));
            }
            ESP_LOGI(TAG, "Periodic SYNC unicast to tier-1 in free slot (cycle %lu)", (unsigned long)cycle_index);
        }

        /* End of cycle: did we get any DATA from any child this cycle? */
        bool any_received = false;
        for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES; i++) {
            uint16_t nid = p->slot_to_node[i];
            if (nid < ROOT_MAX_SLOT_NODES && received_this_cycle[nid]) {
                any_received = true;
                break;
            }
        }
        if (any_received) {
            s_cycles_no_pkt = 0;
        } else {
            if (num_online > 0) {
                reset_lora();
                s_cycles_no_pkt++;
                ESP_LOGW(TAG, "No packet from any child this cycle (consecutive=%u)", (unsigned)s_cycles_no_pkt);
                if (s_cycles_no_pkt >= 3) {
                    ESP_LOGW(TAG, "COMMANDER: No DATA from any node for 3 cycles; root restarting.");
                    ESP_LOGE(TAG, "3 cycles with no packet from any child -> resetting mesh");
                    fflush(stdout);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    restart_root();
                }
            }
        }

        /* Notify commander which nodes sent DATA this cycle (before we clear received_this_cycle). */
        {
            char data_buf[64];
            size_t off = 0;
            for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES && off < sizeof(data_buf) - 8; i++) {
                uint16_t nid = p->slot_to_node[i];
                if (nid < ROOT_MAX_SLOT_NODES && received_this_cycle[nid]) {
                    int n = snprintf(data_buf + off, sizeof(data_buf) - off, "%u ", (unsigned)nid);
                    if (n > 0 && (size_t)n < sizeof(data_buf) - off)
                        off += (size_t)n;
                    else
                        break;
                }
            }
            if (off > 0) {
                if (data_buf[off - 1] == ' ')
                    data_buf[off - 1] = '\0';
                else
                    data_buf[off] = '\0';
                ESP_LOGW(TAG, "COMMANDER: DATA this cycle: %s", data_buf);
            } else {
                ESP_LOGW(TAG, "COMMANDER: DATA this cycle: none");
            }
        }

        /* CSV lines for DATA received this cycle (no header; commander can parse). */
        for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES; i++) {
            uint16_t nid = p->slot_to_node[i];
            if (nid >= ROOT_MAX_SLOT_NODES || !received_this_cycle[nid])
                continue;
            uint64_t ts_s = cycle_data_ts_ms[nid] / 1000;
            uint16_t plen = cycle_data_len[nid];
            uint8_t *pb = cycle_data_payload[nid];
            printf("%u,%llu,", (unsigned)nid, (unsigned long long)ts_s);
            bool printable = true;
            bool has_comma = false;
            for (uint16_t j = 0; j < plen; j++) {
                if (pb[j] < 32 || pb[j] > 126) printable = false;
                if (pb[j] == ',') has_comma = true;
            }
            if (printable && plen > 0 && !has_comma) {
                for (uint16_t j = 0; j < plen; j++)
                    putchar((char)pb[j]);
            } else if (printable && plen > 0) {
                putchar('"');
                for (uint16_t j = 0; j < plen; j++) {
                    if (pb[j] == '"') printf("\"\"");
                    else putchar((char)pb[j]);
                }
                putchar('"');
            } else {
                for (uint16_t j = 0; j < plen && j < CYCLE_DATA_PAYLOAD_MAX; j++)
                    printf("%02X", pb[j]);
            }
            printf("\n");
        }

        /* Update node status (no DATA -> back to ONLINE), report to log and terminal */
        for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES; i++) {
            uint16_t nid = p->slot_to_node[i];
            if (nid >= ROOT_MAX_SLOT_NODES)
                continue;
            if (!received_this_cycle[nid]) {
                mesh_set_node_state(nid, MESH_NODE_ONLINE);
                if (nid < ROOT_MAX_SLOT_NODES)
                    resync_retry_count[nid] = 0;
            }
            received_this_cycle[nid] = false;
        }
        /* Cycle start is on .00 boundary; label as yymmdd-hh:mm (24h UTC). */
        {
            uint64_t cycle_start_ms = anchor + (uint64_t)cycle_index * base_period_ms;
            time_t sec = (time_t)(cycle_start_ms / 1000);
            struct tm tm_info;
            gmtime_r(&sec, &tm_info);
            char cycle_label[32];
            strftime(cycle_label, sizeof(cycle_label), "%y%m%d-%H:%M", &tm_info);
            printf("--- %s complete ---\n", cycle_label);
        }
        for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES; i++) {
            uint16_t nid = p->slot_to_node[i];
            mesh_node_state_t s = mesh_get_node_state(nid);
            printf("  Node %u: %s\n", (unsigned)nid, mesh_node_state_str(s));
        }
        printf("------------------------\n");

        {
            char status_buf[128];
            size_t off = 0;
            for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES && off < sizeof(status_buf) - 16; i++) {
                uint16_t nid = p->slot_to_node[i];
                mesh_node_state_t s = mesh_get_node_state(nid);
                int n = snprintf(status_buf + off, sizeof(status_buf) - off, "n%u=%s ", (unsigned)nid, mesh_node_state_str(s));
                if (n > 0 && (size_t)n < sizeof(status_buf) - off)
                    off += (size_t)n;
                else
                    break;
            }
            if (off > 0 && status_buf[off - 1] == ' ')
                status_buf[off - 1] = '\0';
            else
                status_buf[off] = '\0';
            ESP_LOGW(TAG, "COMMANDER: CYCLE_STATUS cycle=%lu %s", (unsigned long)cycle_index, status_buf);
        }

        cycle_index++;
        bool printed_1s = false;
        for (;;) {
            gettimeofday(&tv, NULL);
            now_ms = (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
            c = (uint16_t)(cycle_index % full_cycles);
            n_slots = p->slot_count_per_cycle[c];
            slot_ms = (n_slots > 0) ? (base_period_ms / n_slots) : base_period_ms;
            free_ix = (n_slots > 0) ? (n_slots - 1) : 0;
            free_slot_start = anchor + (uint64_t)cycle_index * base_period_ms + (uint64_t)free_ix * slot_ms;
            if (now_ms >= free_slot_start)
                break;
            uint64_t delay_ms = free_slot_start - now_ms;
            if (delay_ms <= 1000 && delay_ms > 0 && !printed_1s) {
                printf("Cycle will end in 1 second.\n");
                printed_1s = true;
            }
            vTaskDelay(pdMS_TO_TICKS((TickType_t)(delay_ms > 30000 ? 30000 : delay_ms)));
        }
    }
}

/* =========================================================
   data.csv (in-memory buffer): node_id, timestamp, payload
   Use "csvdump" CLI to print CSV to terminal.
   ========================================================= */
typedef struct {
    uint16_t node_id;
    uint64_t ts_ms;
    uint16_t len;
    uint8_t  payload[ROOT_CSV_MAX_PAYLOAD];
} root_csv_row_t;

static root_csv_row_t s_csv_rows[ROOT_CSV_MAX_ROWS];
static uint16_t s_csv_write_idx = 0;
static uint16_t s_csv_count = 0;
static char     s_csv_filename[40] = "";

void root_data_csv_start(uint64_t anchor_epoch_ms)
{
    s_csv_write_idx = 0;
    s_csv_count = 0;
    time_t sec = (time_t)(anchor_epoch_ms / 1000ULL);
    struct tm t;
    gmtime_r(&sec, &t);
    snprintf(s_csv_filename, sizeof(s_csv_filename), "%04d%02d%02d_%02d%02d%02d.csv",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
    ESP_LOGI(TAG, "data.csv buffer started: %s", s_csv_filename);
}

void root_data_csv_append(uint16_t node_id, uint64_t ts_ms, const uint8_t *payload, uint16_t len)
{
    if (payload == NULL)
        return;
    if (len > ROOT_CSV_MAX_PAYLOAD)
        len = ROOT_CSV_MAX_PAYLOAD;
    root_csv_row_t *r = &s_csv_rows[s_csv_write_idx];
    r->node_id = node_id;
    r->ts_ms   = ts_ms;
    r->len     = len;
    memcpy(r->payload, payload, len);
    s_csv_write_idx = (uint16_t)((s_csv_write_idx + 1) % ROOT_CSV_MAX_ROWS);
    if (s_csv_count < ROOT_CSV_MAX_ROWS)
        s_csv_count++;
}

static void root_data_csv_dump(void)
{
    printf("node_id,timestamp,payload\n");
    if (s_csv_count == 0) {
        printf("(no rows)\n");
        return;
    }
    uint16_t start = (s_csv_count < ROOT_CSV_MAX_ROWS) ? 0 : s_csv_write_idx;
    for (uint16_t i = 0; i < s_csv_count; i++) {
        uint16_t idx = (uint16_t)((start + i) % ROOT_CSV_MAX_ROWS);
        const root_csv_row_t *r = &s_csv_rows[idx];
        printf("%u,%llu,\"", (unsigned)r->node_id, (unsigned long long)r->ts_ms);
        for (uint16_t j = 0; j < r->len; j++) {
            uint8_t c = r->payload[j];
            if (c == '"')
                printf("\"\"");
            else if (c >= 32 && c <= 126)
                putchar(c);
            else
                printf("\\x%02x", c);
        }
        printf("\"\n");
    }
    if (s_csv_filename[0])
        printf("(file: %s, %u rows)\n", s_csv_filename, (unsigned)s_csv_count);
}

/* Max payload bytes to include in DATA_RX log (hex string = 2* this). */
#define DATA_RX_LOG_PAYLOAD_MAX  64

static void root_data_at_root_cb(uint16_t node_id, const uint8_t *payload, uint16_t len, int rssi, float snr)
{
    /* Notify commander if root has no TDMA (e.g. after restart): nodes are running but root cannot schedule. */
    if (s_base_period_ms == 0 || s_full_cycle_cycles == 0) {
        static bool no_tdma_warned = false;
        if (!no_tdma_warned) {
            no_tdma_warned = true;
            ESP_LOGW(TAG, "COMMANDER: Node %u sent DATA but root has no TDMA schedule (cached TDMA lost). Run 'start' to remesh.", (unsigned)node_id);
        }
    }

    if (node_id < ROOT_MAX_SLOT_NODES) {
        received_this_cycle[node_id] = true;
        resync_retry_count[node_id] = 0;
    }

    uint64_t ts_ms = root_get_epoch_ms();

    /* Log each DATA RX so pi service or user can grep/save (COMMANDER: DATA_RX ...). */
    {
        char hex_buf[2 * DATA_RX_LOG_PAYLOAD_MAX + 1];
        uint16_t plen = len > DATA_RX_LOG_PAYLOAD_MAX ? DATA_RX_LOG_PAYLOAD_MAX : len;
        hex_buf[0] = '\0';
        for (uint16_t i = 0; i < plen; i++)
            snprintf(hex_buf + 2 * i, (size_t)3, "%02X", payload[i]);
        hex_buf[2 * plen] = '\0';
        ESP_LOGI(TAG, "COMMANDER: DATA_RX node=%u ts=%llu len=%u rssi=%d snr=%.2f hex=%s",
                 (unsigned)node_id, (unsigned long long)ts_ms, (unsigned)len, rssi, snr, hex_buf);
    }

    /* If payload carries TDMA hash prefix, verify it and strip it before CSV logging. */
    if (len >= sizeof(uint32_t)) {
        uint32_t node_hash = 0;
        memcpy(&node_hash, payload, sizeof(uint32_t));

        if (s_tdma_hash != 0 && node_hash != s_tdma_hash) {
            ESP_LOGW(TAG,
                     "TDMA hash mismatch from node %u: node=0x%08X root=0x%08X",
                     (unsigned)node_id, (unsigned)node_hash, (unsigned)s_tdma_hash);
            /* Mark node as out-of-sync with schedule while still recording its data. */
            mesh_set_node_state(node_id, MESH_NODE_ONLINE);
        }

        const uint8_t *app_payload = payload + sizeof(uint32_t);
        uint16_t app_len = (uint16_t)(len - (uint16_t)sizeof(uint32_t));
        if (node_id < ROOT_MAX_SLOT_NODES) {
            cycle_data_ts_ms[node_id] = ts_ms;
            cycle_data_len[node_id] = app_len < CYCLE_DATA_PAYLOAD_MAX ? app_len : (uint16_t)CYCLE_DATA_PAYLOAD_MAX;
            memcpy(cycle_data_payload[node_id], app_payload, (size_t)cycle_data_len[node_id]);
        }
        root_data_csv_append(node_id, ts_ms, app_payload, app_len);
    } else {
        /* Legacy/short payload without hash. */
        if (node_id < ROOT_MAX_SLOT_NODES) {
            cycle_data_ts_ms[node_id] = ts_ms;
            cycle_data_len[node_id] = len < CYCLE_DATA_PAYLOAD_MAX ? len : (uint16_t)CYCLE_DATA_PAYLOAD_MAX;
            memcpy(cycle_data_payload[node_id], payload, (size_t)cycle_data_len[node_id]);
        }
        root_data_csv_append(node_id, ts_ms, payload, len);
    }
}

#define NVS_NAMESPACE  "root_cli"
#define NVS_KEY_EPOCH "epoch_ms"
/* Timezone offset key: signed seconds relative to UTC (e.g. +7h = 25200). */
#define NVS_KEY_TZ_OFFSET "tz_offset_s"
/* Minimum saved epoch ms to consider valid (e.g. year 2001). */
#define MIN_VALID_EPOCH_MS  978307200000ULL
#define MAX_CMD_LEN   128
#define MAX_ARGS      8

/* Global variable to store the last executed command (for potential future use) */
char last_cmd[MAX_CMD_LEN] = {0};

/* Default timezone: Bangkok (UTC+7) expressed in seconds. */
#define ROOT_DEFAULT_TZ_OFFSET_S  (7 * 3600)
static int32_t s_tz_offset_s = ROOT_DEFAULT_TZ_OFFSET_S;


/* =========================================================
   Time Synchronization  (uses system RTC via settimeofday)
   ========================================================= */

static void root_persist_epoch_ms(uint64_t epoch_ms)
{
    if (nvs_flash_init() != ESP_OK)
        return;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
        return;
    nvs_set_u64(h, NVS_KEY_EPOCH, epoch_ms);
    nvs_commit(h);
    nvs_close(h);
}

/* Load timezone offset (seconds from UTC) from NVS; default to Bangkok UTC+7 if not present. */
static void root_load_timezone_from_nvs(void)
{
    if (nvs_flash_init() != ESP_OK)
        return;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
        return;
    int32_t tz_s = 0;
    esp_err_t err = nvs_get_i32(h, NVS_KEY_TZ_OFFSET, &tz_s);
    nvs_close(h);
    if (err == ESP_OK) {
        s_tz_offset_s = tz_s;
    } else {
        s_tz_offset_s = ROOT_DEFAULT_TZ_OFFSET_S;
    }
}

static void root_persist_timezone(void)
{
    if (nvs_flash_init() != ESP_OK)
        return;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
        return;
    nvs_set_i32(h, NVS_KEY_TZ_OFFSET, s_tz_offset_s);
    nvs_commit(h);
    nvs_close(h);
}

/** Restore root clock from NVS after reboot. Call once at root startup. */
static void root_restore_time_from_nvs(void)
{
    if (nvs_flash_init() != ESP_OK)
        return;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
        return;
    uint64_t epoch_ms = 0;
    esp_err_t err = nvs_get_u64(h, NVS_KEY_EPOCH, &epoch_ms);
    nvs_close(h);
    if (err != ESP_OK || epoch_ms < MIN_VALID_EPOCH_MS)
        return;
    struct timeval tv;
    tv.tv_sec  = (time_t)(epoch_ms / 1000);
    tv.tv_usec = (suseconds_t)((epoch_ms % 1000) * 1000);
    settimeofday(&tv, NULL);
    ESP_LOGI(TAG, "Time restored from NVS: %llu ms", (unsigned long long)epoch_ms);

    /* Also restore timezone offset so CLI commands behave consistently after reboot. */
    root_load_timezone_from_nvs();
}

static void root_set_epoch_ms(uint64_t epoch_ms)
{
    struct timeval tv;
    tv.tv_sec  = (time_t)(epoch_ms / 1000);
    tv.tv_usec = (suseconds_t)((epoch_ms % 1000) * 1000);
    settimeofday(&tv, NULL);
    root_persist_epoch_ms(epoch_ms);
}

uint64_t root_get_epoch_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
}

/* Print epoch_ms (Unix UTC) in local time using s_tz_offset_s. UTC+7 => add 7h to get display time. */
static void print_human_time(uint64_t epoch_ms)
{
    time_t utc_s = (time_t)(epoch_ms / 1000);
    time_t display_s = utc_s + (time_t)s_tz_offset_s;

    struct tm tm_info;
    gmtime_r(&display_s, &tm_info);

    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_info);

    int32_t h = s_tz_offset_s / 3600;
    if (h >= 0)
        printf("%s.%03llu UTC+%ld\n", buffer, epoch_ms % 1000, (long)h);
    else
        printf("%s.%03llu UTC%ld\n", buffer, epoch_ms % 1000, (long)h);
}


/* =========================================================
   CLI Command Infrastructure
   ========================================================= */

typedef void (*cli_cmd_func_t)(int argc, char **argv);

typedef struct {
    const char      *name;
    cli_cmd_func_t   handler;
    const char      *help;
} cli_command_t;

/* =========================================================
   Command Handlers
   ========================================================= */
static void cmd_time(int argc, char **argv)
{
    uint64_t now_ms = root_get_epoch_ms();

    printf("Epoch ms: %llu  (Unix s (UTC): %llu)\n", (unsigned long long)now_ms, (unsigned long long)(now_ms / 1000));
    print_human_time(now_ms);
    /* Next minute boundary = anchor for TDMA first cycle. */
    uint64_t next_minute_ms = ((now_ms / 60000) + 1) * 60000;
    printf("To set clock so first TDMA cycle starts at next minute, run:\n  sync %llu\n", (unsigned long long)next_minute_ms);
    printf("(sync interprets small values as local seconds with timezone offset %ld s)\n", (long)s_tz_offset_s);
}

/* ===========================
   TIMEZONE COMMAND
   =========================== */

static void cmd_timezone(int argc, char **argv)
{
    if (argc < 2) {
        printf("Current timezone offset: %ld s (UTC%+ldh)\n",
               (long)s_tz_offset_s,
               (long)(s_tz_offset_s / 3600));
        printf("Usage: timezone <offset_hours>\n");
        printf("Example (Bangkok UTC+7): timezone 7\n");
        return;
    }

    long hours = strtol(argv[1], NULL, 10);
    if (hours < -12 || hours > 14) {
        printf("Invalid offset_hours (must be between -12 and +14).\n");
        return;
    }

    s_tz_offset_s = (int32_t)(hours * 3600L);
    root_persist_timezone();

    printf("Timezone set to UTC%+ldh (%ld s offset). Future sync/ brdsync local seconds will use this.\n",
           hours, (long)s_tz_offset_s);
}

static void cmd_tdma(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (s_base_period_ms == 0 || s_full_cycle_cycles == 0) {
        printf("TDMA is not active. Run 'start' first.\n");
        return;
    }

    uint64_t now_ms = root_get_epoch_ms();

    uint32_t base_period_s = s_base_period_ms / 1000;
    uint32_t full_period_s = base_period_s * s_full_cycle_cycles;

    uint64_t cycle_index = (now_ms >= s_anchor_epoch_ms)
        ? (now_ms - s_anchor_epoch_ms) / s_base_period_ms
        : 0;

    uint16_t cycle_mod = cycle_index % s_full_cycle_cycles;

    uint64_t cycle_start_ms = s_anchor_epoch_ms + cycle_index * s_base_period_ms;
    uint64_t next_cycle_ms  = cycle_start_ms + s_base_period_ms;

    printf("\n");
    printf("=====================================\n");
    printf(" TDMA SCHEDULE\n");
    printf("=====================================\n");

    printf("Schedule hash      : 0x%08X\n", (unsigned)s_tdma_hash);
    printf("Base period        : %lu s\n", (unsigned long)base_period_s);
    printf("Full cycle length  : %u cycles\n", (unsigned)s_full_cycle_cycles);
    printf("Full period        : %lu s\n", (unsigned long)full_period_s);

    printf("\nAnchor (cycle 0 start): ");
    print_human_time(s_anchor_epoch_ms);

    printf("Current time           : ");
    print_human_time(now_ms);

    printf("\n");

    printf("Current cycle index : %llu\n",
           (unsigned long long)cycle_index);

    printf("Cycle within period : %u / %u\n",
           (unsigned)cycle_mod,
           (unsigned)s_full_cycle_cycles);

    int64_t sec_to_next = (int64_t)(next_cycle_ms - now_ms) / 1000;
    if (sec_to_next < 0)
        sec_to_next = 0;

    printf("Next cycle in       : %lld s\n", (long long)sec_to_next);

    uint32_t sync_every = root_sync_every_cycles(s_base_period_ms);
    printf("Periodic SYNC every : %lu cycles (drift bound %u ms)\n",
           (unsigned long)sync_every, (unsigned)DRIFT_BOUND_MS);
    printf("Resync (healing)    : in free slot, up to 3 attempts per ONLINE node per cycle\n");

    printf("\n");

    printf("Cycle schedule:\n");

    for (uint16_t c = 0; c < s_full_cycle_cycles && c < ROOT_MAX_FULL_CYCLE; c++) {

        uint8_t n = tdma_cycle_slot_count[c];
        uint32_t slot_ms = s_base_period_ms / n;

        printf("  Cycle %-3u | slots=%u | slot_ms=%lu | ",
               (unsigned)c,
               (unsigned)n,
               (unsigned long)slot_ms);

        if (c == cycle_mod)
            printf("* ");

        printf("[");

        for (uint8_t k = 0; k < n - 1; k++) {
            printf("%u", (unsigned)tdma_cycle_nodes[c][k]);
            if (k < n - 2)
                printf(" ");
        }

        printf(" free]\n");
    }

    printf("=====================================\n\n");
}

static void cmd_cycle(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    if (s_base_period_ms == 0) {
        printf("TDMA not started. Run 'start' first.\n");
        return;
    }

    uint64_t now_ms = root_get_epoch_ms();

    uint64_t cycle_index = (now_ms >= s_anchor_epoch_ms)
        ? (now_ms - s_anchor_epoch_ms) / s_base_period_ms
        : 0;

    uint16_t cycle_mod = cycle_index % s_full_cycle_cycles;

    uint64_t cycle_start_ms = s_anchor_epoch_ms + cycle_index * s_base_period_ms;
    uint64_t next_cycle_ms  = cycle_start_ms + s_base_period_ms;

    uint64_t cycle_elapsed_ms = now_ms - cycle_start_ms;
    uint64_t cycle_remaining_ms = next_cycle_ms - now_ms;

    uint8_t n_slots = tdma_cycle_slot_count[cycle_mod];
    uint32_t slot_ms = s_base_period_ms / n_slots;

    uint8_t slot_index = cycle_elapsed_ms / slot_ms;
    if (slot_index >= n_slots)
        slot_index = n_slots - 1;

    uint64_t slot_start_ms = cycle_start_ms + (uint64_t)slot_index * slot_ms;
    uint64_t slot_end_ms   = slot_start_ms + slot_ms;

    uint64_t slot_remaining_ms = (slot_end_ms > now_ms)
        ? (slot_end_ms - now_ms)
        : 0;

    printf("\n");
    printf("=====================================\n");
    printf(" TDMA CYCLE STATUS\n");
    printf("=====================================\n");

    printf("Current time      : ");
    print_human_time(now_ms);

    printf("Anchor time       : ");
    print_human_time(s_anchor_epoch_ms);

    printf("\n");

    printf("Cycle index       : %llu\n",
           (unsigned long long)cycle_index);

    printf("Cycle in pattern  : %u / %u\n",
           (unsigned)cycle_mod,
           (unsigned)s_full_cycle_cycles);

    printf("Cycle length      : %lu s\n",
           (unsigned long)(s_base_period_ms / 1000));

    printf("Cycle remaining   : %llu s\n",
           (unsigned long long)(cycle_remaining_ms / 1000));

    printf("\n");

    printf("Schedule hash     : 0x%08X\n", (unsigned)s_tdma_hash);

    printf("\n");

    printf("Slots in cycle    : %u\n", n_slots);
    printf("Slot duration     : %lu ms\n",
           (unsigned long)slot_ms);

    printf("Current slot      : %u\n", slot_index);

    if (slot_index < n_slots - 1) {
        printf("Slot owner        : Node %u\n",
               tdma_cycle_nodes[cycle_mod][slot_index]);
    } else {
        printf("Slot owner        : FREE (sync slot)\n");
    }

    printf("Slot remaining    : %llu ms\n",
           (unsigned long long)slot_remaining_ms);

    printf("\nSlot layout:\n");

    for (uint8_t i = 0; i < n_slots; i++) {

        printf("  Slot %-2u | ", i);

        if (i < n_slots - 1)
            printf("Node %-3u | ", tdma_cycle_nodes[cycle_mod][i]);
        else
            printf("FREE     | ");

        if (i == slot_index)
            printf("<-- current");

        printf("\n");
    }

    printf("=====================================\n\n");
}

static void cmd_ping(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: ping <node_id>\n");
        return;
    }

    uint16_t target = atoi(argv[1]);

    bool ok = ping(target);

    printf("Ping result: %s\n", ok ? "OK" : "FAIL");
    mesh_start_rx_continuous();  /* return root to RX so it can receive again */
}

static void cmd_start(int argc, char **argv)
{
    printf("Starting mesh runtime...\n");
    mesh_start_runtime();

    if (!nodecfg_has_static_topology()) {
        printf("No static topology — manual ASSIGN required.\n");
        return;
    }

    printf("Static topology detected — bypassing ASSIGN.\n");

    uint16_t topo_size = nodecfg_get_topo_table_size();
    uint16_t root_id   = nodecfg_get_root_id();

    uint16_t nonroot_ids[32];
    uint16_t num_nonroot = 0;

    for (uint16_t i = 0; i < topo_size; i++) {
        if (i == root_id)
            continue;
        nodecfg_topology_t t;
        nodecfg_get_topology(i, &t);
        if (t.parent_id == 0xFFFF && t.child_count == 0)
            continue;
        nonroot_ids[num_nonroot++] = i;
    }

    if (num_nonroot == 0) {
        printf("No non-root nodes in topology table.\n");
        return;
    }

    /* Ping direct children (tier-1) only; no SYNC at start — full adjacency from ping. */
    for (uint16_t i = 0; i < num_nonroot; i++) {
        uint16_t nid = nonroot_ids[i];
        nodecfg_topology_t t;
        nodecfg_get_topology(nid, &t);
        if (t.parent_id != root_id)
            continue;
        printf("  Pinging tier-1 node %u...\n", nid);
        bool ok = ping(nid);
        if (ok) {
            mesh_set_node_online(nid, true);
            printf("  Node %u (tier-1) reachable\n", nid);
        } else {
            mesh_set_node_online(nid, false);
            printf("  Node %u (tier-1) no response\n", nid);
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }

    /* After ping, require all tier-1 nodes in flashed topology to be ONLINE before building TDMA. */
    bool all_tier1_online = true;
    for (uint16_t i = 0; i < num_nonroot; i++) {
        uint16_t nid = nonroot_ids[i];
        nodecfg_topology_t t;
        nodecfg_get_topology(nid, &t);
        if (t.parent_id != root_id)
            continue;
        if (!mesh_is_node_online(nid)) {
            all_tier1_online = false;
            printf("COMMANDER: Tier-1 node %u OFFLINE at start (topology flashed).\n", nid);
            if (t.child_count > 0) {
                printf("  Node %u has %u children; its subtree cannot be scheduled.\n",
                       nid, (unsigned)t.child_count);
            }
        }
    }
    if (!all_tier1_online) {
        printf("At least one tier-1 node is OFFLINE — stopping mesh (no TDMA scheduled). Ensure all tier-1 nodes are online, then run 'start' again.\n");
        return;
    }

    /* Build TDMA table and send per-node schedules */
    uint16_t online_ids[32];
    uint16_t num_online = 0;
    for (uint16_t i = 0; i < num_nonroot; i++) {
        if (mesh_is_node_online(nonroot_ids[i]))
            online_ids[num_online++] = nonroot_ids[i];
    }

    if (num_online == 0) {
        printf("No nodes ONLINE — skipping TDMA.\n");
        return;
    }

    /* Base period = min of online nodes' reporting intervals (seconds). */
    uint32_t base_period_s = nodecfg_get_reporting_interval_s(online_ids[0]);
    for (uint16_t i = 1; i < num_online; i++) {
        uint32_t iv = nodecfg_get_reporting_interval_s(online_ids[i]);
        if (iv < base_period_s)
            base_period_s = iv;
    }
    /* Full period = LCM of online nodes' reporting intervals. */
    uint32_t full_period_s = nodecfg_get_reporting_interval_s(online_ids[0]);
    for (uint16_t i = 1; i < num_online; i++) {
        full_period_s = lcm_u32(full_period_s, nodecfg_get_reporting_interval_s(online_ids[i]));
    }
    uint16_t full_cycle_cycles = (uint16_t)(full_period_s / base_period_s);
    if (full_cycle_cycles == 0)
        full_cycle_cycles = 1;
    if (full_cycle_cycles > ROOT_MAX_FULL_CYCLE) {
        printf("Full cycle too large (%u), capping to %u\n", (unsigned)full_cycle_cycles, (unsigned)ROOT_MAX_FULL_CYCLE);
        full_cycle_cycles = ROOT_MAX_FULL_CYCLE;
    }

    uint32_t base_period_ms = base_period_s * 1000;
    uint64_t epoch_ms = root_get_epoch_ms();
    uint64_t anchor_epoch_ms = ((epoch_ms / (base_period_s * 1000)) + 1) * (uint64_t)base_period_s * 1000;
    if (base_period_s >= 60)
        anchor_epoch_ms = ((epoch_ms / 60000) + 1) * 60000;  /* align to minute when base is 60s */

    /* Bootstrap TDMA vs runtime TDMA:
     * - Bootstrap anchor: anchor_epoch_ms (first base cycle start).
     * - Runtime anchor:   anchor_epoch_ms + bootstrap_cycles * base_period_ms.
     * For now bootstrap_cycles is fixed to 1; later this can be made configurable.
     */
    s_bootstrap_cycles = 1;
    s_runtime_anchor_epoch_ms = anchor_epoch_ms + (uint64_t)s_bootstrap_cycles * base_period_ms;

    /* Build TDMA table: for each base cycle c, which nodes report (report_cycle_offset = last in period). */
    tdma_full_cycle_cycles = full_cycle_cycles;
    for (uint16_t c = 0; c < full_cycle_cycles; c++) {
        uint8_t idx = 0;
        for (uint16_t i = 0; i < num_online && idx < ROOT_MAX_SLOT_NODES; i++) {
            uint16_t nid = online_ids[i];
            uint32_t interval_s = nodecfg_get_reporting_interval_s(nid);
            uint16_t report_every = (uint16_t)(interval_s / base_period_s);
            if (report_every == 0)
                report_every = 1;
            uint8_t report_cycle_offset = (uint8_t)(report_every - 1);  /* report in last cycle of period */
            if ((c % report_every) == report_cycle_offset) {
                tdma_cycle_nodes[c][idx++] = nid;
            }
        }
        tdma_cycle_slot_count[c] = idx + 1;  /* +1 for free slot */
    }

    printf("TDMA table: base_period=%lu s, full_period=%lu s, full_cycle_cycles=%u\n",
           (unsigned long)base_period_s, (unsigned long)full_period_s, (unsigned)full_cycle_cycles);
    printf("Bootstrap anchor (first base cycle start): ");
    print_human_time(anchor_epoch_ms);
    printf("Runtime anchor (after %u bootstrap cycle(s)): ", (unsigned)s_bootstrap_cycles);
    print_human_time(s_runtime_anchor_epoch_ms);
    for (uint16_t c = 0; c < full_cycle_cycles; c++) {
        uint8_t n = tdma_cycle_slot_count[c];
        printf("  Cycle %u: %u slots (", (unsigned)c, (unsigned)n);
        for (uint8_t k = 0; k < n - 1; k++)
            printf("%u ", (unsigned)tdma_cycle_nodes[c][k]);
        printf("free)\n");
    }

    /* Compute deterministic TDMA schedule hash now that table is final. */
    s_tdma_hash = tdma_compute_hash();
    printf("TDMA schedule hash: 0x%08X\n", (unsigned)s_tdma_hash);

    /* Send per-node TDMA payload (slot_index and slot_duration_ms for the cycle(s) where node reports). */
    s_anchor_epoch_ms   = anchor_epoch_ms;
    s_base_period_ms    = base_period_ms;
    s_full_cycle_cycles = full_cycle_cycles;
    for (uint16_t i = 0; i < num_online; i++) {
        uint16_t nid = online_ids[i];
        mesh_tdma_payload_t tdma;
        if (!root_build_tdma_payload_for_node(nid, &tdma)) {
            printf("  -> Node %u: not in schedule, skip\n", (unsigned)nid);
            continue;
        }
        printf("  -> Node %u: report_every=%u, offset=%u, slot=%u, slot_ms=%lu\n",
               nid, (unsigned)tdma.report_every_n_cycles, (unsigned)tdma.report_cycle_offset,
               (unsigned)tdma.slot_index, (unsigned long)tdma.slot_duration_ms);
        mesh_send_to(nid, MESH_PKT_TDMA, (uint8_t *)&tdma, sizeof(tdma));
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    printf("TDMA schedule sent to %u online nodes.\n", num_online);

    uint32_t sync_every = root_sync_every_cycles(base_period_ms);
    printf("Periodic SYNC every %lu cycles (drift bound %u ms)\n",
           (unsigned long)sync_every, (unsigned)DRIFT_BOUND_MS);

    if (root_sync_task_handle != NULL) {
        vTaskDelete(root_sync_task_handle);
        root_sync_task_handle = NULL;
    }
    memset(received_this_cycle, 0, sizeof(received_this_cycle));
    static root_sync_task_params_t sync_params;
    sync_params.anchor_epoch_ms    = anchor_epoch_ms;
    sync_params.base_period_ms     = base_period_ms;
    sync_params.sync_every_cycles  = sync_every;
    sync_params.num_online         = num_online;
    sync_params.full_cycle_cycles  = full_cycle_cycles;
    for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES; i++)
        sync_params.slot_to_node[i] = online_ids[i];
    for (uint16_t c = 0; c < full_cycle_cycles && c < ROOT_MAX_FULL_CYCLE; c++)
        sync_params.slot_count_per_cycle[c] = tdma_cycle_slot_count[c];

    num_current_schedule = num_online;
    for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES; i++)
        current_schedule_node_ids[i] = online_ids[i];
    for (uint16_t i = 0; i < ROOT_MAX_SLOT_NODES; i++)
        resync_retry_count[i] = 0;

    all_online_notified = false;  /* Allow "all nodes ONLINE" notification again after this start/remesh. */

    root_data_csv_start(anchor_epoch_ms);
    mesh_register_data_at_root_cb(root_data_at_root_cb);
    mesh_register_join_callback(root_join_cb);

    xTaskCreate(root_sync_task, "root_sync", 4096, &sync_params, 3, &root_sync_task_handle);
}

static void cmd_restart(int argc, char **argv)
{
    printf("Restarting root...\n");
    restart_root();
}

static void cmd_reset_lora(int argc, char **argv)
{
    printf("Resetting LoRa chip...\n");
    reset_lora();
}
/* ===========================
   SYNC COMMAND
   =========================== */

/* Values below this are treated as Unix seconds (e.g. from date +%s); else milliseconds. */
#define EPOCH_MS_THRESHOLD 10000000000ULL  /* 10 billion */

static void cmd_sync(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: sync <t_s_or_ms>  (if <1e10, interpreted as *local* seconds with timezone offset %ld s; else milliseconds since Unix epoch)\n",
               (long)s_tz_offset_s);
        return;
    }

    uint64_t val = strtoull(argv[1], NULL, 10);
    uint64_t epoch_ms;
    if (val < EPOCH_MS_THRESHOLD) {
        /* Interpret as local seconds; convert to UTC epoch by subtracting timezone offset. */
        int64_t local_s = (int64_t)val;
        int64_t utc_s = local_s - (int64_t)s_tz_offset_s;
        if (utc_s < 0)
            utc_s = 0;
        epoch_ms = (uint64_t)utc_s * 1000ULL;
    } else {
        /* Milliseconds since Unix epoch (UTC) directly. */
        epoch_ms = val;
    }

    root_set_epoch_ms(epoch_ms);

    printf("Root clock synchronized to:\n");
    print_human_time(epoch_ms);
}

/* ===========================
   BRDSYNC COMMAND
   =========================== */
static void cmd_brdsync(int argc, char **argv)
{
    if (argc >= 2) {
        uint64_t val = strtoull(argv[1], NULL, 10);
        uint64_t epoch_ms;
        if (val < EPOCH_MS_THRESHOLD) {
            int64_t local_s = (int64_t)val;
            int64_t utc_s = local_s - (int64_t)s_tz_offset_s;
            if (utc_s < 0)
                utc_s = 0;
            epoch_ms = (uint64_t)utc_s * 1000ULL;
        } else {
            epoch_ms = val;
        }
        root_set_epoch_ms(epoch_ms);
    }

    uint64_t epoch_ms = root_get_epoch_ms();
    printf("Broadcasting SYNC with clock: ");
    print_human_time(epoch_ms);

    mesh_send(MESH_PKT_SYNC, (uint8_t *)&epoch_ms, sizeof(epoch_ms));
}

/* ===========================
   STATUS COMMAND
   =========================== */
static void cmd_status(int argc, char **argv)
{
    uint16_t topo_size = nodecfg_get_topo_table_size();
    uint16_t root_id   = nodecfg_get_root_id();
    uint64_t now_ms    = root_get_epoch_ms();

    printf("Node status:\n");
    for (uint16_t i = 0; i < topo_size; i++) {
        nodecfg_topology_t t;
        nodecfg_get_topology(i, &t);
        if (t.parent_id == 0xFFFF && t.child_count == 0 && i != root_id)
            continue;
        if (i == root_id) {
            printf("  Node %u: ROOT\n", i);
            continue;
        }
        bool tier1 = (t.parent_id == root_id);
        const char *tier_str = tier1 ? " (tier-1)" : "";
        mesh_node_state_t s = mesh_get_node_state(i);
        if (s == MESH_NODE_OFFLINE) {
            printf("  Node %u: OFFLINE%s\n", i, tier_str);
        } else {
            int64_t last = mesh_get_node_last_seen(i);
            int64_t ago  = (int64_t)now_ms - last;
            printf("  Node %u: %s%s (%.1fs ago)\n", i, mesh_node_state_str(s), tier_str, ago / 1000.0);
        }
    }
}

/* ===========================
   UPTIME COMMAND (unused; uncomment and add to command_table when needed)
   ===========================
static void cmd_uptime(int argc, char **argv)
{
    int64_t us = esp_timer_get_time();
    printf("ESP uptime: %lld ms\n", us / 1000);
}
*/

/* ===========================
   HELP COMMAND
   =========================== */

static void cmd_help(int argc, char **argv);

static void cmd_logdump(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    root_log_dump();
}

static void cmd_logtail(int argc, char **argv)
{
    unsigned n = 20;
    if (argc >= 2) {
        int val = atoi(argv[1]);
        if (val > 0) {
            n = (unsigned)val;
        }
    }
    root_log_dump_tail(n);
}

static void cmd_logsummary(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    root_log_dump_summary();
}

static void cmd_logclear(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    root_log_clear();
    printf("Log cleared.\n");
}

static void cmd_csvdump(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    root_data_csv_dump();
}

static const cli_command_t command_table[] = {
    { "ping",       cmd_ping,    "ping <node_id>" },
    { "start",      cmd_start,   "start mesh runtime" },
    { "restart",    cmd_restart, "restart root node" },
    { "sync",       cmd_sync,    "sync <epoch_ms> : set root UTC clock (persistent)" },
    { "brdsync",    cmd_brdsync, "brdsync [epoch_ms] : broadcast SYNC with clock to network" },
    { "status",     cmd_status,  "show online/offline state of each topology node" },
    { "help",       cmd_help,    "show available commands" },
    { "time",       cmd_time,    "show current root time" },
    { "timezone",   cmd_timezone, "set utc timezone"},
    { "cycle",      cmd_cycle,   "show current TDMA cycle (yymmdd-hh:mm) and time to next" },
    { "tdma",       cmd_tdma,    "show TDMA schedule" },
    { "reset",      cmd_reset_lora, "reset LoRa chip" },
    { "logdump",    cmd_logdump, "dump full persistent root mesh log" },
    { "logtail",    cmd_logtail, "logtail [n] : show last n log entries (default 20)" },
    { "logsummary", cmd_logsummary, "show log summary (counts by type, DATA_RX per node)" },
    { "logclear",   cmd_logclear, "clear persistent root mesh log" },
    { "csvdump",    cmd_csvdump, "dump data.csv (node_id,timestamp,payload) to terminal" },
};

#define COMMAND_COUNT (sizeof(command_table) / sizeof(command_table[0]))

static void cmd_help(int argc, char **argv)
{
    printf("Available commands:\n");

    for (int i = 0; i < COMMAND_COUNT; i++) {
        printf("  %-10s - %s\n",
               command_table[i].name,
               command_table[i].help);
    }
}

/* =========================================================
   Parsing & Dispatch
   ========================================================= */

static int parse_command(char *line, char **argv, int max_args)
{
    int argc = 0;
    char *token = strtok(line, " ");

    while (token != NULL && argc < max_args) {
        argv[argc++] = token;
        token = strtok(NULL, " ");
    }

    return argc;
}

static void dispatch_command(char *line)
{
    char *argv[MAX_ARGS];
    int argc = parse_command(line, argv, MAX_ARGS);

    if (argc == 0) {
        return;
    }

    for (int i = 0; i < COMMAND_COUNT; i++) {

        if (strcmp(argv[0], command_table[i].name) == 0) {
            command_table[i].handler(argc, argv);
            return;
        }
    }

    printf("Unknown command. Type 'help'\n");
}

/* =========================================================
   CLI Task
   ========================================================= */

static void root_debug_cli_task(void *arg)
{
    char cmd[MAX_CMD_LEN];
    int index = 0;

    printf("\nroot> ");
    fflush(stdout);

    while (1) {

        int c = getchar();
        
        if (c == '\r' || c == '\n')
        {
            cmd[index] = '\0';
            printf("\n");

            if (index > 0) {
                strncpy(last_cmd, cmd, MAX_CMD_LEN - 1);
                last_cmd[MAX_CMD_LEN - 1] = '\0';
            }

            dispatch_command(cmd);

            index = 0;
            memset(cmd, 0, MAX_CMD_LEN);

            printf("\nroot> ");
            fflush(stdout);

            continue;   // important to skip rest of loop
        }
        else if (c >= 32 && c < 127) 
        {

            if (index < MAX_CMD_LEN - 1) {
                cmd[index++] = (char)c;
                putchar(c);
                fflush(stdout);
            }
        }
        else if (c == 0x08 || c == 0x7F)  // backspace or DEL
        {
            if (index > 0) {
                index--;
                printf("\b \b");  // erase character visually
                fflush(stdout);
            }
        }
        else if (c == 0x1B)  // ESC
        {
            int c1 = getchar();
            int c2 = getchar();

            if (c1 == '[' && c2 == 'A')  // Up arrow
            {
                // Erase current line visually
                while (index > 0) {
                    printf("\b \b");
                    index--;
                }

                // Clear command buffer completely
                memset(cmd, 0, MAX_CMD_LEN);

                // Copy last command safely
                strncpy(cmd, last_cmd, MAX_CMD_LEN - 1);
                cmd[MAX_CMD_LEN - 1] = '\0';

                index = strlen(cmd);

                printf("%s", cmd);
                fflush(stdout);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* =========================================================
   Public API
   ========================================================= */

void root_cli_start(void)
{
    root_restore_time_from_nvs();
    root_log_init();
    ESP_LOGI(TAG, "Starting Root CLI task");

    xTaskCreate(
        root_debug_cli_task,
        "root_cli",
        4096,
        NULL,
        5,
        NULL
    );
}
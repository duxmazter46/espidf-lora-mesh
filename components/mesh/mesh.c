#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "mesh.h"
#include "mac.h"
#include "phy.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "node_config.h"
#include "net.h"
#include "root_log.h"

#define TAG "MESH"

/* =========================
   Architecture: ROOT / RELAY / LEAF (see docs/ARCHITECTURE.md)
   - ROOT: many tasks; normally only TX in free slot (downlink).
   - RELAY: downlink notify from parent (TDMA/SYNC); uplink own DATA; uplink forward children.
   - LEAF:  uplink own DATA; uplink request info/changes (later).
   ========================= */

/* =========================
   Non-root state (clear indication for logging/debug)
   ========================= */

typedef enum {
    MESH_NONROOT_STATE_WAIT_FOR_SYNC,       /* Non-root with static topology: wait for SYNC/TDMA from root; LoRa reset every 60s, restart ESP after 2 resets */
    MESH_NONROOT_STATE_WAIT_FOR_ASSIGN,     /* Non-root without static topology: wait for ASSIGN from root; LoRa reset every 20s, never restart ESP */
    MESH_NONROOT_STATE_WAIT_FOR_TDMA,       /* Non-root (after topo received): wait for TDMA from root; LoRa reset every 20s, never restart ESP */
    MESH_NONROOT_STATE_RUNNING,             /* Got ASSIGN or SYNC; normal operation */
} mesh_nonroot_state_t;

static volatile mesh_nonroot_state_t mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_SYNC;

/* =========================
   Internal State
   ========================= */

static nodecfg_topology_t my_topology;

static uint16_t mesh_node_id = 0;
/* Consecutive slots with no ACK from anyone (root/parent) → restart to prevent deadlock. */
#define MESH_CONSECUTIVE_FAILED_SLOTS_BEFORE_RESTART  3
static uint8_t mesh_failed_slots = 0;
static volatile bool mesh_assigned = false;
static volatile bool mesh_sync_received = false;

static TaskHandle_t sync_tdma_wait_task = NULL;
static bool mesh_debug_mode = false;

static volatile bool mesh_tdma_received = false;
static mesh_tdma_payload_t my_tdma_slot;
static uint32_t my_schedule_hash = 0;

#define MESH_APP_PAYLOAD_MAX 64
static uint8_t app_tx_payload[MESH_APP_PAYLOAD_MAX];
static uint16_t app_tx_len = 0;
static portMUX_TYPE app_tx_mux = portMUX_INITIALIZER_UNLOCKED;

/* Updated on any RX "for me" (broadcast or unicast); used by wait tasks to avoid LoRa reset when root pings etc. */
static volatile TickType_t mesh_last_rx_for_me_ticks = 0;
/* Updated on any RX (for CAD simulation: channel busy if recent). */
static volatile TickType_t mesh_any_rx_ticks = 0;

/* =========================
   Node Status Table (root only)
   ========================= */

#define MESH_MAX_NODES 32

typedef struct {
    uint8_t state;       /* mesh_node_state_t: OFFLINE, ONLINE, RUNNING */
    int64_t last_seen_ms;
} mesh_node_status_t;

static mesh_node_status_t node_status_table[MESH_MAX_NODES];

/* Child TDMA cache: when relaying TDMA to a child, store for first-half slot downlink. */
#define MESH_CHILD_TDMA_CACHE_MAX NODECFG_MAX_CHILDREN
static mesh_tdma_payload_t child_tdma_cache[MESH_CHILD_TDMA_CACHE_MAX];
static bool child_tdma_valid[MESH_CHILD_TDMA_CACHE_MAX];

/* Relay: cache SYNC epoch from parent so we downlink SYNC to children in our next slot (battery-saver children need force wake). */
static uint64_t sync_epoch_for_children = 0;
static bool sync_epoch_for_children_valid = false;

typedef struct {
    uint8_t type;
} __attribute__((packed)) mesh_header_t;

typedef struct {
    uint16_t next_hop;
    uint8_t  buffer[256];
    uint16_t len;
} mesh_tx_job_t;

static QueueHandle_t mesh_tx_queue;

/* =========================
   MAC → Node Mapping Table
   ========================= */

typedef struct {
    uint8_t mac[6];
    uint16_t node_id;
} node_map_t;

/* =========================
   Forward Declaration
   ========================= */

static void mesh_forward_handler(
    mac_rx_event_t *rx,
    net_header_t *hdr,
    uint8_t *payload,
    uint16_t len);

static void mesh_net_rx_handler(mac_rx_event_t *rx);

/** Called by non-root when child JOIN is synced: forward JOIN to parent after 500 ms listen. */
void mesh_schedule_join_forward_to_parent(const uint8_t *payload, uint16_t len);

static mesh_data_at_root_cb_t s_data_at_root_cb = NULL;

typedef void (*mesh_join_cb_t)(uint16_t node_id, uint8_t interval_byte);
static mesh_join_cb_t s_join_cb = NULL;

/* =========================
   Internal Functions
   ========================= */
   static uint16_t mesh_get_next_hop(uint16_t final_dst)
   {
       if (mesh_node_id == final_dst)
           return final_dst;

       if (mesh_node_id == nodecfg_get_root_id()) {
           /* Multi-hop down: first hop toward target (direct child on path). */
           uint16_t first = nodecfg_get_first_hop_toward(mesh_node_id, final_dst);
           if (first != 0xFFFF)
               return first;
           return final_dst;  /* fallback: direct (single-hop) */
       }

       /* Non-root: downward to child/subtree → first hop toward target; else upward to parent. */
       uint16_t first = nodecfg_get_first_hop_toward(mesh_node_id, final_dst);
       if (first != 0xFFFF)
           return first;
       return my_topology.parent_id;      /* upward tree routing */
   }

   /* Forward TX is done in mesh_running_task during our TDMA slot (no separate task). */

   static void mesh_tx_result_handler(mac_event_type_t type)
{
    /* Slot-level reliability and reset logic is handled in mesh_running_task.
     * Here we only log MAC-level results for debugging. */
    if (type == MAC_EVENT_TX_DONE) {
        ESP_LOGD(TAG, "MAC TX done");
    } else if (type == MAC_EVENT_TIMEOUT) {
        ESP_LOGW(TAG, "MAC TX timeout");
    }
}

// State strings to reflect correct states.
static const char *mesh_nonroot_state_str(mesh_nonroot_state_t s)
{
    switch (s) {
        case MESH_NONROOT_STATE_WAIT_FOR_SYNC: return "WAIT_FOR_SYNC";   // Sync from root
        case MESH_NONROOT_STATE_WAIT_FOR_ASSIGN: return "WAIT_FOR_ASSIGN"; // Waiting for TDMA Assign from root
        case MESH_NONROOT_STATE_WAIT_FOR_TDMA: return "WAIT_FOR_TDMA"; // Waiting for TDMA Sync
        case MESH_NONROOT_STATE_RUNNING: return "RUNNING"; // Running after sync and TDMA received
        default: return "?";
    }
}

#define MESH_JOIN_PAYLOAD_MAX 8
#define MESH_JOIN_CAD_MS 500

static uint8_t s_join_forward_payload[MESH_JOIN_PAYLOAD_MAX];
static uint16_t s_join_forward_len = 0;
static uint16_t s_sync_child_id = 0xFFFF;

/* Root: when direct child sends broadcast JOIN, defer SYNC by 500ms + CAD (like non-root). */
static uint16_t s_root_sync_child_id = 0xFFFF;
static uint8_t  s_root_sync_interval_byte = 0;

static void mesh_root_sync_child_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(MESH_JOIN_CAD_MS));
        TickType_t now = xTaskGetTickCount();
        if ((now - mesh_any_rx_ticks) > pdMS_TO_TICKS(MESH_JOIN_CAD_MS)) {
            if (s_root_sync_child_id != 0xFFFF && mesh_node_id == nodecfg_get_root_id()) {
                struct timeval tv;
                gettimeofday(&tv, NULL);
                uint64_t epoch_ms = (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
                mesh_send_to(s_root_sync_child_id, MESH_PKT_SYNC, (uint8_t *)&epoch_ms, sizeof(epoch_ms));
                uint16_t id = s_root_sync_child_id;
                uint8_t interval_byte = s_root_sync_interval_byte;
                s_root_sync_child_id = 0xFFFF;
                mesh_set_node_online(id, true);
                if (s_join_cb)
                    s_join_cb(id, interval_byte);
                ESP_LOGI(TAG, "JOIN from direct child %u → SYNC sent, marked ONLINE", (unsigned)id);
            }
            vTaskDelete(NULL);
            return;
        }
    }
}

/** Runs in its own task so we do not block the PHY RX path (avoids stack overflow / deadlock). */
/* Delay so parent's MAC ACK finishes on air before we send SYNC (PHY TX is non-blocking). */
#define MESH_SYNC_AFTER_ACK_MS 150

static void mesh_sync_child_and_forward_task(void *arg)
{
    (void)arg;
    if (s_sync_child_id == 0xFFFF || my_topology.parent_id == 0xFFFF)
        goto done;
    /* Let parent's ACK complete and be received by child before we send SYNC. */
    vTaskDelay(pdMS_TO_TICKS(MESH_SYNC_AFTER_ACK_MS));
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t epoch_ms = (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
    bool ack = mesh_send_to(s_sync_child_id, MESH_PKT_SYNC, (uint8_t *)&epoch_ms, sizeof(epoch_ms));
    s_sync_child_id = 0xFFFF;
    if (ack && s_join_forward_len > 0 && s_join_forward_len <= MESH_JOIN_PAYLOAD_MAX) {
        ESP_LOGI(TAG, "Synced child, notifying parent");
        mesh_schedule_join_forward_to_parent(s_join_forward_payload, s_join_forward_len);
    }
done:
    vTaskDelete(NULL);
}

static void mesh_join_forward_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(MESH_JOIN_CAD_MS));
        TickType_t now = xTaskGetTickCount();
        if ((now - mesh_any_rx_ticks) > pdMS_TO_TICKS(MESH_JOIN_CAD_MS)) {
            uint16_t parent = my_topology.parent_id;
            if (parent != 0xFFFF && s_join_forward_len > 0 && s_join_forward_len <= MESH_JOIN_PAYLOAD_MAX) {
                mesh_send_to(parent, MESH_PKT_JOIN, s_join_forward_payload, s_join_forward_len);
                ESP_LOGI(TAG, "Forwarded JOIN to parent %u", (unsigned)parent);
            }
            vTaskDelete(NULL);
            return;
        }
    }
}

void mesh_schedule_join_forward_to_parent(const uint8_t *payload, uint16_t len)
{
    if (len > MESH_JOIN_PAYLOAD_MAX || my_topology.parent_id == 0xFFFF)
        return;
    memcpy(s_join_forward_payload, payload, len);
    s_join_forward_len = len;
    /* Stack 4KB: task calls mesh_send_to to parent; 2KB can overflow with MAC/NET stack. */
    xTaskCreate(mesh_join_forward_task, "join_fwd", 4096, NULL, 3, NULL);
}

static void mesh_wait_sync_task(void *arg)
{
    mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_SYNC;
    bool unicast_join = nodecfg_has_static_topology() && (my_topology.parent_id != 0xFFFF);
    ESP_LOGI(TAG, "State → %s (%s JOIN once, then listening for SYNC)",
             mesh_nonroot_state_str(mesh_nonroot_state),
             unicast_join ? "unicast" : "broadcasting");

    uint8_t join_payload[MESH_JOIN_PAYLOAD_MAX];
    uint16_t id_payload = mesh_node_id;
    memcpy(join_payload, &id_payload, sizeof(id_payload));
    uint32_t reporting_interval_s = nodecfg_get_reporting_interval_s(mesh_node_id);
    join_payload[sizeof(id_payload)] = (reporting_interval_s > 255) ? 255 : (uint8_t)reporting_interval_s;
    uint16_t join_len = sizeof(id_payload) + 1;

    /* CAD simulation: listen 500 ms; if channel clear, send JOIN; else postpone. */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(MESH_JOIN_CAD_MS));
        TickType_t now = xTaskGetTickCount();
        if ((now - mesh_any_rx_ticks) > pdMS_TO_TICKS(MESH_JOIN_CAD_MS)) {
            break;
        }
        ESP_LOGD(TAG, "Channel busy, postponing JOIN");
    }

    if (unicast_join) {
        ESP_LOGI(TAG, "[%s] Unicast JOIN to parent %u (node %u)",
                 mesh_nonroot_state_str(mesh_nonroot_state), (unsigned)my_topology.parent_id, mesh_node_id);
        mesh_send_to(my_topology.parent_id, MESH_PKT_JOIN, join_payload, join_len);
    } else {
        ESP_LOGI(TAG, "[%s] Broadcasting JOIN (node %u)",
                 mesh_nonroot_state_str(mesh_nonroot_state), mesh_node_id);
        mesh_send(MESH_PKT_JOIN, join_payload, join_len);
    }

    phy_start_rx_continuous();

    const TickType_t sync_timeout_ms = 60000;  /* wait 60 s before resetting LoRa / restart */
    const TickType_t loop_delay_ms   = 500;
    int lora_reset_count = 0;
    TickType_t last_reset_ticks = xTaskGetTickCount();

    while (!mesh_sync_received) {
        vTaskDelay(pdMS_TO_TICKS(loop_delay_ms));
        if (mesh_sync_received)
            break;

        TickType_t now = xTaskGetTickCount();
        if ((now - last_reset_ticks) < pdMS_TO_TICKS(sync_timeout_ms))
            continue;

        if (lora_reset_count >= 2) {
            ESP_LOGE(TAG,
                     "[%s] No SYNC after %d ms and %d LoRa resets → restarting node",
                     mesh_nonroot_state_str(mesh_nonroot_state),
                     (int)sync_timeout_ms * (lora_reset_count + 1),
                     lora_reset_count);
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        }

        lora_reset_count++;
        ESP_LOGW(TAG,
                 "[%s] No SYNC for %d ms → resetting LoRa chip (attempt %d/2)",
                 mesh_nonroot_state_str(mesh_nonroot_state),
                 (int)sync_timeout_ms,
                 lora_reset_count);

        reset_lora();
        phy_start_rx_continuous();
        last_reset_ticks = xTaskGetTickCount();
    }

    ESP_LOGI(TAG, "SYNC received, proceeding");

    if (nodecfg_has_static_topology()) {
        mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_TDMA;
        ESP_LOGI(TAG, "Static topology → State → WAIT_FOR_TDMA");
    } else {
        mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_ASSIGN;
        ESP_LOGI(TAG, "State → %s", mesh_nonroot_state_str(mesh_nonroot_state));
    }

    vTaskDelete(NULL);
}

/* Unused: uncomment and wire (e.g. from mesh_wait_sync_task) when ASSIGN path is needed.
static void mesh_wait_assign_task(void *arg)
{
    const int max_lora_resets = 3;
    const TickType_t no_activity_reset_ms = 20000; // reset LoRa only if no "for me" RX for 20s
    const TickType_t loop_delay_ms = 500;

    mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_ASSIGN;
    ESP_LOGI(TAG, "State → %s (waiting for ASSIGN from root; LoRa reset every 20s if no activity, restart ESP after 2 resets)",
             mesh_nonroot_state_str(mesh_nonroot_state));

    if (nodecfg_has_static_topology()) {
        mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_TDMA;
        ESP_LOGI(TAG, "Static topology detected, skipping ASSIGN, moving to WAIT_FOR_TDMA");
        vTaskDelete(NULL);
        return;
    }

    int lora_reset_count = 0;
    while (!mesh_assigned) {
        vTaskDelay(pdMS_TO_TICKS(loop_delay_ms));
        if (mesh_assigned)
            break;

        TickType_t now = xTaskGetTickCount();
        if ((now - mesh_last_rx_for_me_ticks) <= pdMS_TO_TICKS(no_activity_reset_ms))
            continue;

        if (lora_reset_count >= max_lora_resets) {
            ESP_LOGE(TAG,
                     "[%s] No TDMA ASSIGN after ~%d s and %d LoRa resets → restarting ESP32",
                     mesh_nonroot_state_str(mesh_nonroot_state),
                     (int)(no_activity_reset_ms / 1000) * max_lora_resets,
                     max_lora_resets);
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        }

        ESP_LOGW(TAG,
                 "[%s] No ASSIGN yet (no activity 20s) → resetting LoRa chip (attempt %d/%d)",
                 mesh_nonroot_state_str(mesh_nonroot_state),
                 lora_reset_count + 1,
                 max_lora_resets);

        phy_reset();
        phy_init();
        phy_start_rx_continuous();
        mesh_last_rx_for_me_ticks = xTaskGetTickCount();
        lora_reset_count++;
    }

    mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_TDMA;
    ESP_LOGI(TAG, "State → %s (ASSIGN received, now waiting for TDMA Sync)", mesh_nonroot_state_str(mesh_nonroot_state));
    vTaskDelete(NULL);
}
*/

/* Wait for TDMA from root (non-root with static topology); reset LoRa only if no "for me" RX for 20s, never restart ESP.
   Unused: uncomment and wire (e.g. from mesh_wait_sync_task) when TDMA wait task is needed.
static void mesh_wait_tdma_task(void *arg)
{
    const TickType_t no_activity_reset_ms = 20000;
    const TickType_t loop_delay_ms = 500;

    mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_TDMA;
    ESP_LOGI(TAG, "State → %s (waiting for TDMA from root; LoRa reset only if no activity 20s)",
             mesh_nonroot_state_str(mesh_nonroot_state));

    while (!mesh_tdma_received) {
        vTaskDelay(pdMS_TO_TICKS(loop_delay_ms));
        if (mesh_tdma_received)
            break;

        TickType_t now = xTaskGetTickCount();
        if ((now - mesh_last_rx_for_me_ticks) <= pdMS_TO_TICKS(no_activity_reset_ms))
            continue;

        ESP_LOGW(TAG,
                 "[%s] No TDMA from root yet (no activity 20s) → resetting LoRa chip (no ESP restart)",
                 mesh_nonroot_state_str(mesh_nonroot_state));

        phy_reset();
        phy_init();
        phy_start_rx_continuous();
        mesh_last_rx_for_me_ticks = xTaskGetTickCount();
    }

    mesh_nonroot_state = MESH_NONROOT_STATE_RUNNING;
    ESP_LOGI(TAG, "State → %s (TDMA received, running)", mesh_nonroot_state_str(mesh_nonroot_state));
    sync_tdma_wait_task = NULL;
    vTaskDelete(NULL);
}
*/

/* RUNNING: send in assigned slot when (cycle % report_every_n_cycles) == report_cycle_offset. */
static void mesh_running_task(void *arg)
{
    uint64_t anchor    = my_tdma_slot.anchor_epoch_ms;
    uint32_t base_ms   = my_tdma_slot.base_period_ms;
    uint32_t slot_ms   = my_tdma_slot.slot_duration_ms;
    uint16_t report_every = my_tdma_slot.report_every_n_cycles;
    uint8_t  report_offset = my_tdma_slot.report_cycle_offset;
    uint8_t  slot_ix   = my_tdma_slot.slot_index;
    static bool first_slot_done = false;  /* relay: first cycle = TDMA downlink then own DATA; second+ = own then forward */

    if (report_every == 0)
        report_every = 1;

    ESP_LOGI(TAG, "RUNNING: anchor=%llu base_ms=%lu slot_ms=%lu report_every=%u offset=%u slot_ix=%u children=%u",
             (unsigned long long)anchor, (unsigned long)base_ms, (unsigned long)slot_ms,
             (unsigned)report_every, (unsigned)report_offset, (unsigned)slot_ix, (unsigned)my_topology.child_count);

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

    for (;;) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t now_ms = (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
        uint64_t cycle_index = (now_ms >= anchor) ? ((now_ms - anchor) / base_ms) : 0;

        /* Report only when this cycle is our report cycle. */
        if ((cycle_index % report_every) != (uint64_t)report_offset) {
            /* Skip to start of next cycle and re-check. */
            uint64_t next_cycle_start = anchor + (cycle_index + 1) * base_ms;
            uint64_t delay_ms = next_cycle_start - now_ms;
            if (delay_ms > 0)
                vTaskDelay(pdMS_TO_TICKS((TickType_t)(delay_ms > 30000 ? 30000 : delay_ms)));
            continue;
        }

        uint64_t slot_start_ms = anchor + cycle_index * base_ms + (uint64_t)slot_ix * slot_ms;
        if (now_ms < slot_start_ms) {
            uint64_t delay_ms = slot_start_ms - now_ms;
            if (delay_ms > 0)
                vTaskDelay(pdMS_TO_TICKS((TickType_t)(delay_ms > 30000 ? 30000 : delay_ms)));
            continue;
        }

        const uint32_t guard_ms = 500;
        uint64_t slot_end_guard_ms = slot_start_ms + (slot_ms > guard_ms ? (slot_ms - guard_ms) : slot_ms);
        bool slot_success = false;
        bool has_children = (my_topology.child_count > 0);

        /* ----- RELAY DOWNLINK: notify children with info from parent (TDMA, SYNC from last cycle free slot). ----- */
        if (has_children && !first_slot_done) {
            uint32_t half_ms = (slot_ms > guard_ms) ? ((slot_ms - guard_ms) / 2) : 0;
            uint64_t phase1_end_ms = slot_start_ms + half_ms;
            for (int i = 0; i < my_topology.child_count && i < MESH_CHILD_TDMA_CACHE_MAX; i++) {
                if (!child_tdma_valid[i])
                    continue;
                gettimeofday(&tv, NULL);
                now_ms = (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
                if (now_ms >= phase1_end_ms)
                    break;
                mesh_send_to(my_topology.children[i], MESH_PKT_TDMA, (uint8_t *)&child_tdma_cache[i], sizeof(mesh_tdma_payload_t));
                vTaskDelay(pdMS_TO_TICKS(400));
            }
            /* Downlink SYNC to children (from root's periodic unicast); battery-saver children need force wake. */
            if (sync_epoch_for_children_valid) {
                for (int i = 0; i < my_topology.child_count; i++) {
                    mesh_send_to(my_topology.children[i], MESH_PKT_SYNC, (uint8_t *)&sync_epoch_for_children, sizeof(sync_epoch_for_children));
                    vTaskDelay(pdMS_TO_TICKS(400));
                }
                sync_epoch_for_children_valid = false;
                ESP_LOGI(TAG, "SYNC downlink to %u children", (unsigned)my_topology.child_count);
            }
            vTaskDelay(pdMS_TO_TICKS(guard_ms));
            /* Fall through to RELAY UPLINK own DATA (same loop below). */
        } else if (has_children && first_slot_done) {
            /* RELAY DOWNLINK (second+ cycle): SYNC to children if cached; then RELAY UPLINK own DATA + forward. */
            if (sync_epoch_for_children_valid) {
                for (int i = 0; i < my_topology.child_count; i++) {
                    mesh_send_to(my_topology.children[i], MESH_PKT_SYNC, (uint8_t *)&sync_epoch_for_children, sizeof(sync_epoch_for_children));
                    vTaskDelay(pdMS_TO_TICKS(400));
                }
                sync_epoch_for_children_valid = false;
                ESP_LOGI(TAG, "SYNC downlink to %u children", (unsigned)my_topology.child_count);
            }
        }

        /* ----- LEAF UPLINK: request info/changes (later); for now drain forward queue to parent. ----- */
        if (!has_children) {
            const int max_forwards_per_slot = 2;
            for (int i = 0; i < max_forwards_per_slot && mesh_tx_queue != NULL; i++) {
                mesh_tx_job_t job;
                if (xQueueReceive(mesh_tx_queue, &job, 0) != pdPASS)
                    break;
                ESP_LOGI(TAG, "Forwarding in slot → %u (%u bytes)", (unsigned)job.next_hop, (unsigned)job.len);
                mac_send(job.next_hop, job.buffer, job.len);
                vTaskDelay(pdMS_TO_TICKS(400));
            }
        }

        /* ----- RELAY/LEAF UPLINK: own DATA packet to parent. ----- */
        while (!slot_success) {
            gettimeofday(&tv, NULL);
            now_ms = (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
            if (now_ms >= slot_end_guard_ms)
                break;

            taskENTER_CRITICAL(&app_tx_mux);
            uint16_t len = app_tx_len;
            uint8_t app_buf[MESH_APP_PAYLOAD_MAX];
            if (len > 0 && len <= MESH_APP_PAYLOAD_MAX)
                memcpy(app_buf, app_tx_payload, len);
            taskEXIT_CRITICAL(&app_tx_mux);

            if (len == 0) {
                len = (uint16_t)snprintf((char *)app_buf, sizeof(app_buf), "Node%u", (unsigned)mesh_node_id);
                if (len > MESH_APP_PAYLOAD_MAX)
                    len = MESH_APP_PAYLOAD_MAX;
            }

            if (len == 0) {
                break;  /* nothing to send */
            }

            /* Prepend our TDMA schedule hash so root can verify alignment. */
            uint8_t tx_buf[MESH_APP_PAYLOAD_MAX + sizeof(uint32_t)];
            uint16_t tx_len = len + (uint16_t)sizeof(uint32_t);
            if (tx_len > sizeof(tx_buf))
                tx_len = sizeof(tx_buf);
            memcpy(tx_buf, &my_schedule_hash, sizeof(uint32_t));
            uint16_t copy_len = tx_len - (uint16_t)sizeof(uint32_t);
            if (copy_len > 0)
                memcpy(tx_buf + sizeof(uint32_t), app_buf, copy_len);

            bool ok = mesh_send(MESH_PKT_DATA, tx_buf, tx_len);
            if (ok) {
                ESP_LOGI(TAG, "Slot TX success: %u bytes (cycle_index=%llu)", (unsigned)tx_len, (unsigned long long)cycle_index);
                slot_success = true;
                break;
            }

            ESP_LOGW(TAG, "Slot TX failed (no ACK), retrying if time permits");
            /* 500 ms guard before next send attempt (we already retry up to 2 times). */
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        if (!slot_success) {
            /* Entire slot failed (no ACK from root/parent); stay in RX continuous so root can ping/resync. */
            ESP_LOGE(TAG, "Slot failed (cycle_index=%llu)", (unsigned long long)cycle_index);
            phy_start_rx_continuous();
            if (mesh_failed_slots < 0xFF)
                mesh_failed_slots++;
            if (mesh_failed_slots >= MESH_CONSECUTIVE_FAILED_SLOTS_BEFORE_RESTART) {
                ESP_LOGE(TAG, "%u consecutive slots with no ACK → restarting node (prevent deadlock)",
                        (unsigned)MESH_CONSECUTIVE_FAILED_SLOTS_BEFORE_RESTART);
                fflush(stdout);
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        } else {
            /* Got ACK → clear consecutive failed-slot counter. */
            mesh_failed_slots = 0;
        }

        /* ----- RELAY UPLINK: forward children's packets to parent. ----- */
        if (has_children && first_slot_done && mesh_tx_queue != NULL) {
            const int max_forwards_per_slot = 2;
            for (int i = 0; i < max_forwards_per_slot; i++) {
                mesh_tx_job_t job;
                if (xQueueReceive(mesh_tx_queue, &job, 0) != pdPASS)
                    break;
                ESP_LOGI(TAG, "Forwarding in slot → %u (%u bytes)", (unsigned)job.next_hop, (unsigned)job.len);
                mac_send(job.next_hop, job.buffer, job.len);
                vTaskDelay(pdMS_TO_TICKS(400));
            }
        }

        first_slot_done = true;

        /* Wait until next cycle (our next report cycle). */
        cycle_index++;
        while ((cycle_index % report_every) != (uint64_t)report_offset)
            cycle_index++;
        uint64_t next_slot_start = anchor + cycle_index * base_ms + (uint64_t)slot_ix * slot_ms;
        for (;;) {
            gettimeofday(&tv, NULL);
            now_ms = (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
            if (now_ms >= next_slot_start)
                break;
            uint64_t delay_ms = next_slot_start - now_ms;
            vTaskDelay(pdMS_TO_TICKS((TickType_t)(delay_ms > 30000 ? 30000 : delay_ms)));
        }
    }
}

/* =========================
   Public API
   ========================= */

// Public API for mesh runtime behavior
void mesh_start_runtime(void)
{
    nodecfg_get_topology(mesh_node_id, &my_topology);
    memset(child_tdma_valid, 0, sizeof(child_tdma_valid));

    net_init(mesh_node_id);
    net_register_routing_callback(mesh_get_next_hop);
    net_register_event_callback(mesh_tx_result_handler);

    mesh_tx_queue = xQueueCreate(5, sizeof(mesh_tx_job_t));

    net_register_forward_callback(mesh_forward_handler);
    net_register_rx_callback(mesh_net_rx_handler);

    mac_set_mode(MAC_MODE_PASSIVE);
    phy_start_rx_continuous();

    if (mesh_node_id == nodecfg_get_root_id()) {
        ESP_LOGI(TAG, "ROOT NODE STARTED (no SYNC at start; adjacency via ping only)");
    } else {
        mesh_sync_received = false;
        mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_SYNC;
        ESP_LOGI(TAG, "Non-root: starting WAIT_FOR_SYNC (%s JOIN)",
                 (nodecfg_has_static_topology() && my_topology.parent_id != 0xFFFF) ? "unicast" : "broadcast");
        xTaskCreate(mesh_wait_sync_task, "sync_wait", 4096, NULL, 4, &sync_tdma_wait_task);
    }
}

void mesh_init(void)
{
    // ------------------------
    // Init PHY
    // ------------------------
    if (!phy_init()) {
        ESP_LOGE(TAG, "PHY init failed!");
        return;
    }

    mesh_node_id = nodecfg_get_node_id();

    if (mesh_node_id == nodecfg_get_root_id()) {
        mesh_debug_mode = true;
        ESP_LOGI(TAG, "ROOT DEBUG MODE");

        net_init(mesh_node_id);
        net_register_rx_callback(mesh_net_rx_handler);

        mac_set_mode(MAC_MODE_PASSIVE);
        phy_start_rx_continuous();

        return;   // do NOT start mesh runtime yet
    }

    /* non-root start normal mesh runtime */
    mesh_start_runtime();
}

/* Payload is root's clock as uint64_t epoch ms (UTC). Apply LoRa airtime compensation (Class-B style). */
static void mesh_handle_sync_clock(uint8_t *payload, uint16_t len, uint16_t mac_payload_len)
{
    if (len < sizeof(uint64_t))
        return;
    uint64_t epoch_ms;
    memcpy(&epoch_ms, payload, sizeof(epoch_ms));

    /* Compensate for LoRa airtime: root timestamp ≈ TX start, we receive after airtime. */
    uint32_t airtime_us = mac_lora_airtime_us_for_payload(mac_payload_len);
    uint64_t airtime_ms = (uint64_t)(airtime_us / 1000);
    uint64_t corrected_ms = epoch_ms + airtime_ms;

    struct timeval tv;
    tv.tv_sec  = (time_t)(corrected_ms / 1000);
    tv.tv_usec = (suseconds_t)((corrected_ms % 1000) * 1000);
    settimeofday(&tv, NULL);

    ESP_LOGI(TAG, "Clock synced from SYNC: epoch_ms=%llu + airtime=%lu ms → corrected=%llu",
             (unsigned long long)epoch_ms, (unsigned long)airtime_ms, (unsigned long long)corrected_ms);
}

static void mesh_process_packet(mesh_header_t *mhdr, uint8_t *mesh_payload,
                                uint16_t mesh_len, mac_rx_event_t *rx,
                                const net_header_t *net_hdr)
{
    /* For unicast, net_hdr is set; use net_hdr->src as original source for multi-hop DATA. */
    uint16_t data_src = (net_hdr != NULL) ? net_hdr->src : rx->src;

    switch (mhdr->type) {
        case MESH_PKT_JOIN:
            if (mesh_node_id == nodecfg_get_root_id()) {
                /* ROUTING_STRICT_TREE: root only accepts JOIN from direct children (prev_hop). */
                if (nodecfg_get_routing_policy() == ROUTING_STRICT_TREE && rx != NULL) {
                    uint16_t prev = rx->prev_hop;
                    if (!nodecfg_is_direct_child(nodecfg_get_root_id(), prev)) {
                        ESP_LOGI(TAG, "Strict tree: ignore JOIN from non-child prev_hop=%u", (unsigned)prev);
                        break;
                    }
                }
                uint16_t joining_id = 0;
                uint8_t interval_byte = 0;
                if (mesh_len >= sizeof(uint16_t))
                    memcpy(&joining_id, mesh_payload, sizeof(joining_id));
                if (mesh_len >= 3)
                    interval_byte = mesh_payload[2];
                /* JOIN from direct child (broadcast or unicast): defer SYNC like non-root (500ms + CAD, then unicast SYNC). */
                if (nodecfg_is_direct_child(nodecfg_get_root_id(), joining_id)) {
                    s_root_sync_child_id = joining_id;
                    s_root_sync_interval_byte = interval_byte;
                    xTaskCreate(mesh_root_sync_child_task, "root_sync", 4096, NULL, 3, NULL);
                    ESP_LOGI(TAG, "JOIN from direct child %u → scheduled SYNC in 500ms (CAD)", (unsigned)joining_id);
                } else {
                    /* Forwarded JOIN (grandchild etc.): mark ONLINE immediately. */
                    if (s_join_cb) {
                        s_join_cb(joining_id, interval_byte);
                    } else {
                        mesh_set_node_online(joining_id, true);
                    }
                    ESP_LOGI(TAG, "JOIN from node %u (interval_byte=%u) -> marked ONLINE", joining_id, (unsigned)interval_byte);
                }
            } else {
                /* Non-root: if JOIN is from my child (broadcast or prev_hop), sync child then notify parent. */
                uint16_t joining_id = 0;
                if (mesh_len >= sizeof(uint16_t))
                    memcpy(&joining_id, mesh_payload, sizeof(joining_id));
                bool is_my_child = false;
                for (int i = 0; i < my_topology.child_count; i++) {
                    if (my_topology.children[i] == joining_id) {
                        is_my_child = true;
                        break;
                    }
                }
                if (is_my_child && rx != NULL && rx->prev_hop == joining_id) {
                    /* Defer sync+forward to a task so we do not block PHY RX path (stack overflow / deadlock). */
                    if (mesh_len > 0 && mesh_len <= MESH_JOIN_PAYLOAD_MAX) {
                        s_sync_child_id = joining_id;
                        memcpy(s_join_forward_payload, mesh_payload, mesh_len);
                        s_join_forward_len = mesh_len;
                        /* Stack 4KB: task calls mesh_send_to -> net_send -> mac_send (blocking + logging); 2KB overflows. */
                        xTaskCreate(mesh_sync_child_and_forward_task, "sync_ch", 4096, NULL, 3, NULL);
                    }
                }
            }
            break;

        case MESH_PKT_SYNC:
            if (mesh_node_id != nodecfg_get_root_id()) {
                mesh_handle_sync_clock(mesh_payload, mesh_len, rx ? rx->payload_len : 0);

                /* Relay: cache epoch to downlink SYNC to children in our next slot (children may need force wake if battery saver). */
                if (my_topology.child_count > 0 && mesh_len >= sizeof(uint64_t)) {
                    memcpy(&sync_epoch_for_children, mesh_payload, sizeof(uint64_t));
                    sync_epoch_for_children_valid = true;
                }

                /* Only leave WAIT_FOR_SYNC / transition state when we were waiting. Ignore periodic SYNC when already RUNNING. */
                if (mesh_nonroot_state == MESH_NONROOT_STATE_WAIT_FOR_SYNC) {
                    ESP_LOGI(TAG, "SYNC received → leaving WAIT_FOR_SYNC");
                    mesh_sync_received = true;
                    if (nodecfg_has_static_topology()) {
                        mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_TDMA;
                        ESP_LOGI(TAG, "Static topology → skipping ASSIGN, State → WAIT_FOR_TDMA");
                    } else {
                        mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_ASSIGN;
                        ESP_LOGI(TAG, "State → %s", mesh_nonroot_state_str(mesh_nonroot_state));
                    }
                    if (sync_tdma_wait_task != NULL) {
                        vTaskDelete(sync_tdma_wait_task);
                        sync_tdma_wait_task = NULL;
                    }
                    phy_start_rx_continuous();
                }
                /* If already WAIT_FOR_TDMA or RUNNING, we only applied clock; no state change. */
            }
            break;

        case MESH_PKT_ASSIGN:
            if (mesh_node_id != nodecfg_get_root_id()) {
                ESP_LOGI(TAG, "ASSIGN received, storing topology");
                mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_TDMA;
                ESP_LOGI(TAG, "State → %s", mesh_nonroot_state_str(mesh_nonroot_state));
            }
            break;

        case MESH_PKT_TDMA:
            if (mesh_node_id != nodecfg_get_root_id()) {
                if (mesh_len >= sizeof(mesh_tdma_payload_t)) {
                    memcpy(&my_tdma_slot, mesh_payload, sizeof(mesh_tdma_payload_t));
                    my_schedule_hash = my_tdma_slot.schedule_hash;
                    ESP_LOGI(TAG, "TDMA received: base_ms=%lu slot_ms=%lu full_cycles=%u report_every=%u offset=%u slot=%u bootstrap_cycles=%u anchor=%llu runtime_anchor=%llu hash=0x%08X",
                             (unsigned long)my_tdma_slot.base_period_ms,
                             (unsigned long)my_tdma_slot.slot_duration_ms,
                             (unsigned)my_tdma_slot.full_cycle_cycles,
                             (unsigned)my_tdma_slot.report_every_n_cycles,
                             (unsigned)my_tdma_slot.report_cycle_offset,
                             (unsigned)my_tdma_slot.slot_index,
                             (unsigned)my_tdma_slot.bootstrap_cycles,
                             (unsigned long long)my_tdma_slot.anchor_epoch_ms,
                             (unsigned long long)my_tdma_slot.runtime_anchor_epoch_ms,
                             (unsigned)my_schedule_hash);
                } else {
                    ESP_LOGW(TAG, "TDMA payload too short (%u < %u)",
                             mesh_len, (unsigned)sizeof(mesh_tdma_payload_t));
                }
                mesh_tdma_received = true;
                mesh_nonroot_state = MESH_NONROOT_STATE_RUNNING;
                ESP_LOGI(TAG, "State → %s", mesh_nonroot_state_str(mesh_nonroot_state));
                xTaskCreate(mesh_running_task, "mesh_run", 4096, NULL, 4, NULL);
            }
            break;

        case MESH_PKT_DATA:
            if (mesh_node_id == nodecfg_get_root_id()) {
                mesh_set_node_state(data_src, MESH_NODE_RUNNING);
                root_log_data_rx(data_src, mesh_len, rx->rssi, rx->snr);
                if (s_data_at_root_cb)
                    s_data_at_root_cb(data_src, mesh_payload, mesh_len, rx->rssi, rx->snr);
            }
            if (mesh_len > 0)
                ESP_LOGI(TAG, "DATA from node (see payload below)");
            break;

        default:
            ESP_LOGW(TAG, "Unknown mesh pkt type: %d", mhdr->type);
            break;
    }

    ESP_LOGI(TAG,
        "MESH RX: src_mac=%u type=%u len=%u RSSI=%d SNR=%.2f",
        rx->src, mhdr->type, mesh_len, rx->rssi, rx->snr);

}

static void mesh_net_rx_handler(mac_rx_event_t *rx)
{
    mesh_any_rx_ticks = xTaskGetTickCount();
    /* Any RX to this node (broadcast or unicast) restarts the "no progress" timer on non-root */
    if (mesh_node_id != nodecfg_get_root_id())
        mesh_last_rx_for_me_ticks = xTaskGetTickCount();

    /* -------------------------------------------------------
       Broadcast: payload = [mesh_header_t][mesh_payload]
       (sent via mac_broadcast, no net_header present)
    -------------------------------------------------------- */
    if (rx->dst == MAC_BROADCAST_ID) {
        if (rx->payload_len < sizeof(mesh_header_t))
            return;

        mesh_header_t *mhdr    = (mesh_header_t *)rx->payload;
        uint8_t *mesh_payload  = rx->payload + sizeof(mesh_header_t);
        uint16_t mesh_len      = rx->payload_len - sizeof(mesh_header_t);

        mesh_process_packet(mhdr, mesh_payload, mesh_len, rx, NULL);
        return;
    }

    /* -------------------------------------------------------
       Unicast: payload = [net_header_t][mesh_header_t][mesh_payload]
    -------------------------------------------------------- */
    if (rx->payload_len < sizeof(net_header_t) + sizeof(mesh_header_t))
        return;

    net_header_t *net_hdr  = (net_header_t *)rx->payload;
    uint8_t *mesh_ptr      = rx->payload + sizeof(net_header_t);
    uint16_t mesh_total_len = rx->payload_len - sizeof(net_header_t);

    mesh_header_t *mhdr    = (mesh_header_t *)mesh_ptr;
    uint8_t *mesh_payload  = mesh_ptr + sizeof(mesh_header_t);
    uint16_t mesh_len      = mesh_total_len - sizeof(mesh_header_t);

    mesh_process_packet(mhdr, mesh_payload, mesh_len, rx, net_hdr);
}

uint16_t mesh_get_root_id(void){
    return nodecfg_get_root_id();
}

uint16_t mesh_get_node_id(void)
{
    return mesh_node_id;
}

bool mesh_is_node_online(uint16_t node_id)
{
    if (node_id >= MESH_MAX_NODES)
        return false;
    return node_status_table[node_id].state != MESH_NODE_OFFLINE;
}

mesh_node_state_t mesh_get_node_state(uint16_t node_id)
{
    if (node_id >= MESH_MAX_NODES)
        return MESH_NODE_OFFLINE;
    return (mesh_node_state_t)node_status_table[node_id].state;
}

static void update_last_seen(uint16_t node_id)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    node_status_table[node_id].last_seen_ms =
        (int64_t)tv.tv_sec * 1000 + (int64_t)tv.tv_usec / 1000;
}

void mesh_set_node_state(uint16_t node_id, mesh_node_state_t state)
{
    if (node_id >= MESH_MAX_NODES)
        return;
    /* Always refresh last_seen on any non-OFFLINE update, even if state
     * stays the same (e.g. repeated DATA while already RUNNING). */
    if (state != MESH_NODE_OFFLINE)
        update_last_seen(node_id);

    mesh_node_state_t old = (mesh_node_state_t)node_status_table[node_id].state;
    if (old == state)
        return;

    node_status_table[node_id].state = (uint8_t)state;
    if (mesh_node_id == nodecfg_get_root_id()) {
        root_log_node_status(node_id, (root_log_node_state_t)state);
    }
}

void mesh_set_node_online(uint16_t node_id, bool online)
{
    mesh_set_node_state(node_id, online ? MESH_NODE_ONLINE : MESH_NODE_OFFLINE);
}

int64_t mesh_get_node_last_seen(uint16_t node_id)
{
    if (node_id >= MESH_MAX_NODES)
        return 0;
    return node_status_table[node_id].last_seen_ms;
}

void mesh_set_app_payload(const uint8_t *data, uint16_t len)
{
    taskENTER_CRITICAL(&app_tx_mux);
    if (data == NULL || len == 0) {
        app_tx_len = 0;
    } else {
        app_tx_len = len > MESH_APP_PAYLOAD_MAX ? MESH_APP_PAYLOAD_MAX : len;
        memcpy(app_tx_payload, data, app_tx_len);
    }
    taskEXIT_CRITICAL(&app_tx_mux);
}

void mesh_register_data_at_root_cb(mesh_data_at_root_cb_t cb)
{
    s_data_at_root_cb = cb;
}

void mesh_register_join_callback(void (*cb)(uint16_t node_id, uint8_t interval_byte))
{
    s_join_cb = (mesh_join_cb_t)cb;
}

bool ping(uint16_t target)
{
    uint8_t msg[] = "PING";
    return mac_send(target, msg, sizeof(msg));
}

void restart_root(void)
{
    ESP_LOGI(TAG, "Restarting...");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

void reset_lora(void)
{
    ESP_LOGI(TAG, "Resetting LoRa chip...");
    vTaskDelay(pdMS_TO_TICKS(200));
    /* phy_init() must only be called once at startup because it owns
     * SPI bus initialization. For a runtime "reset", just reset the
     * radio chip. Always leave in RX continuous so (non-root) can
     * be pinged by root and marked ONLINE. */
    phy_reset();
    phy_start_rx_continuous();
}

void mesh_start_rx_continuous(void)
{
    phy_start_rx_continuous();
}

static void mesh_forward_handler(
    mac_rx_event_t *rx,
    net_header_t *hdr,
    uint8_t *payload,
    uint16_t len)
{

    uint16_t last_hop = rx ? rx->prev_hop : 0xFFFF;

    if (mesh_node_id == nodecfg_get_root_id())
        return;

    if (hdr->ttl == 0)
        return;

    bool from_child = false;
    for (int i = 0; i < my_topology.child_count; i++) {
        if (last_hop == my_topology.children[i]) {
            from_child = true;
            break;
        }
    }

    bool from_parent = (last_hop == my_topology.parent_id);

    uint16_t next_hop = 0xFFFF;
    if (from_child) {
        next_hop = my_topology.parent_id;
        if (next_hop == 0xFFFF)
            return;
    } else if (from_parent) {
        /* Downward: forward to child on path to final destination. */
        next_hop = nodecfg_get_first_hop_toward(mesh_node_id, hdr->dst);
        if (next_hop == 0xFFFF) {
            ESP_LOGW(TAG, "Drop: dst %u not in my subtree (last_hop=%u)", (unsigned)hdr->dst, (unsigned)last_hop);
            return;
        }
        /* Cache TDMA for this child so we can re-send in our slot first half (TDMA downlink). */
        if (len >= sizeof(mesh_header_t) + sizeof(mesh_tdma_payload_t)) {
            mesh_header_t *mh = (mesh_header_t *)payload;
            if (mh->type == MESH_PKT_TDMA) {
                for (int i = 0; i < my_topology.child_count && i < MESH_CHILD_TDMA_CACHE_MAX; i++) {
                    if (my_topology.children[i] == next_hop) {
                        memcpy(&child_tdma_cache[i], payload + sizeof(mesh_header_t), sizeof(mesh_tdma_payload_t));
                        child_tdma_valid[i] = true;
                        break;
                    }
                }
            }
        }
    } else {
        ESP_LOGW(TAG, "Drop: not from my child or parent (last_hop=%u src=%u)",
                (unsigned)last_hop, (unsigned)hdr->src);
        return;
    }

    hdr->ttl--;

    uint8_t buffer[256];
    memcpy(buffer, hdr, sizeof(net_header_t));
    memcpy(buffer + sizeof(net_header_t), payload, len);

    mesh_tx_job_t job;
    job.next_hop = next_hop;
    job.len = sizeof(net_header_t) + len;
    memcpy(job.buffer, buffer, job.len);

    if (xQueueSend(mesh_tx_queue, &job, 0) != pdPASS) {
        ESP_LOGW(TAG, "Forward queue full, drop");
    }
}

void advertise(void)
{
    uint8_t advertising_payload[256]; 
    uint16_t node_id = mesh_node_id;
    
    uint32_t reporting_interval_s = nodecfg_get_reporting_interval_s(mesh_node_id);
    uint8_t interval_byte = (reporting_interval_s > 255) ? 255 : (uint8_t)reporting_interval_s;

    memcpy(advertising_payload, &node_id, sizeof(node_id));
    advertising_payload[sizeof(node_id)] = interval_byte;

    ESP_LOGI(TAG, "Sending advertising beacon: Node ID = %u, reporting_interval = %lu s", node_id, (unsigned long)reporting_interval_s);
    mac_broadcast(advertising_payload, sizeof(advertising_payload));  // MAC layer handles broadcasting
}

bool mesh_send(mesh_pkt_type_t type,
               uint8_t *payload,
               uint16_t len)
{
    mesh_header_t mhdr;
    mhdr.type = type;

    uint8_t buffer[256];

    memcpy(buffer, &mhdr, sizeof(mhdr));
    memcpy(buffer + sizeof(mhdr), payload, len);

    /* -------------------------
       SINGLE-HOP BROADCAST
    -------------------------- */
    if (type == MESH_PKT_SYNC || type == MESH_PKT_JOIN) {

        ESP_LOGI(TAG, "Single-hop broadcast");

        mac_broadcast(buffer,
                      sizeof(mhdr) + len);

        return true;
    }

    /* -------------------------
       NORMAL UNICAST (via NET)
    -------------------------- */

    uint16_t final_dst;

    if (mesh_node_id == nodecfg_get_root_id()) {
        final_dst = 0xFFFF;
    } else {
        final_dst = 0;
    }

    return net_send(final_dst,
                    buffer,
                    sizeof(mhdr) + len);
}

bool mesh_send_to(uint16_t dst, mesh_pkt_type_t type,
                  uint8_t *payload, uint16_t len)
{
    mesh_header_t mhdr;
    mhdr.type = type;

    uint8_t buffer[256];
    memcpy(buffer, &mhdr, sizeof(mhdr));
    memcpy(buffer + sizeof(mhdr), payload, len);

    ESP_LOGI(TAG, "Directed unicast → node %u, type=%u, len=%u", dst, type, len);
    return net_send(dst, buffer, sizeof(mhdr) + len);
}
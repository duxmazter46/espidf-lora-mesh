#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "mesh.h"
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
   Non-root state (clear indication for logging/debug)
   ========================= */

typedef enum {
    MESH_NONROOT_STATE_WAIT_FOR_SYNC,       /* Non-root with static topology: wait for SYNC/TDMA from root; LoRa reset every 20s, restart ESP after 2 resets */
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
/* Count consecutive TDMA slots where this node failed to TX successfully. */
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

/* =========================
   Node Status Table (root only)
   ========================= */

#define MESH_MAX_NODES 32

typedef struct {
    uint8_t state;       /* mesh_node_state_t: OFFLINE, ONLINE, RUNNING */
    int64_t last_seen_ms;
} mesh_node_status_t;

static mesh_node_status_t node_status_table[MESH_MAX_NODES];

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

static mesh_data_at_root_cb_t s_data_at_root_cb = NULL;

/* =========================
   Internal Functions
   ========================= */
   static uint16_t mesh_get_next_hop(uint16_t final_dst)
   {
       if (mesh_node_id == final_dst)
           return final_dst;
   
       if (mesh_node_id == nodecfg_get_root_id())
           return final_dst;  // root sending down
   
       return my_topology.parent_id;      // upward tree routing
   }

   static void mesh_tx_task(void *arg)
   {
       mesh_tx_job_t job;
   
       while (1) {
   
           if (xQueueReceive(mesh_tx_queue, &job, portMAX_DELAY)) {
   
               ESP_LOGI(TAG, "Forward scheduled, waiting 3s");
   
               vTaskDelay(pdMS_TO_TICKS(2000));
   
               ESP_LOGI(TAG, "Forwarding now → %u", job.next_hop);
   
               mac_send(job.next_hop, job.buffer, job.len);
           }
       }
   }

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

static void mesh_wait_sync_task(void *arg)
{
    mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_SYNC;
    ESP_LOGI(TAG, "State → %s (broadcasting JOIN once, then listening for SYNC)",
             mesh_nonroot_state_str(mesh_nonroot_state));

    uint16_t id_payload = mesh_node_id;
    ESP_LOGI(TAG, "[%s] Broadcasting JOIN (node %u)",
             mesh_nonroot_state_str(mesh_nonroot_state), mesh_node_id);
    mesh_send(MESH_PKT_JOIN, (uint8_t *)&id_payload, sizeof(id_payload));

    phy_start_rx_continuous();

    const TickType_t sync_timeout_ms = 20000;
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

    if (report_every == 0)
        report_every = 1;

    ESP_LOGI(TAG, "RUNNING: anchor=%llu base_ms=%lu slot_ms=%lu report_every=%u offset=%u slot_ix=%u",
             (unsigned long long)anchor, (unsigned long)base_ms, (unsigned long)slot_ms,
             (unsigned)report_every, (unsigned)report_offset, (unsigned)slot_ix);

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

        /* Within our slot: keep trying until success or slot end minus guard time. */
        const uint32_t guard_ms = 500;  /* leave ~500 ms before slot end */
        uint64_t slot_end_guard_ms = slot_start_ms + (slot_ms > guard_ms ? (slot_ms - guard_ms) : slot_ms);
        bool slot_success = false;

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
            /* Short backoff before retrying inside the same slot. */
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (!slot_success) {
            /* Entire slot failed: reset only the LoRa chip. */
            ESP_LOGE(TAG, "Slot failed (cycle_index=%llu) → resetting LoRa chip", (unsigned long long)cycle_index);
            reset_lora();
            if (mesh_failed_slots < 0xFF)
                mesh_failed_slots++;
            if (mesh_failed_slots >= 3) {
                ESP_LOGE(TAG, "Three consecutive failed TDMA slots → restarting node");
                fflush(stdout);
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        } else {
            /* Successful slot → clear consecutive failed-slot counter. */
            mesh_failed_slots = 0;
        }

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

    net_init(mesh_node_id);
    net_register_routing_callback(mesh_get_next_hop);
    net_register_event_callback(mesh_tx_result_handler);

    mesh_tx_queue = xQueueCreate(5, sizeof(mesh_tx_job_t));
    xTaskCreate(mesh_tx_task, "mesh_tx", 4096, NULL, 5, NULL);

    net_register_forward_callback(mesh_forward_handler);
    net_register_rx_callback(mesh_net_rx_handler);

    mac_set_mode(MAC_MODE_PASSIVE);
    phy_start_rx_continuous();

    if (mesh_node_id == nodecfg_get_root_id()) {
        ESP_LOGI(TAG, "ROOT NODE STARTED");
        struct timeval _tv;
        gettimeofday(&_tv, NULL);
        uint64_t epoch_ms = (uint64_t)_tv.tv_sec * 1000 + (uint64_t)_tv.tv_usec / 1000;
        ESP_LOGI(TAG, "Broadcasting SYNC with epoch_ms=%llu", (unsigned long long)epoch_ms);
        mesh_send(MESH_PKT_SYNC, (uint8_t *)&epoch_ms, sizeof(epoch_ms));
    } else {
        mesh_sync_received = false;
        mesh_nonroot_state = MESH_NONROOT_STATE_WAIT_FOR_SYNC;
        ESP_LOGI(TAG, "Non-root: starting WAIT_FOR_SYNC (broadcasting JOIN)");
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

/* Payload is root's clock as uint64_t epoch milliseconds (UTC). */
static void mesh_handle_sync_clock(uint8_t *payload, uint16_t len)
{
    if (len >= sizeof(uint64_t)) {
        uint64_t epoch_ms;
        memcpy(&epoch_ms, payload, sizeof(epoch_ms));

        struct timeval tv;
        tv.tv_sec  = (time_t)(epoch_ms / 1000);
        tv.tv_usec = (suseconds_t)((epoch_ms % 1000) * 1000);
        settimeofday(&tv, NULL);

        ESP_LOGI(TAG, "Clock synced from SYNC: epoch_ms=%llu", (unsigned long long)epoch_ms);
    }
}

static void mesh_process_packet(mesh_header_t *mhdr, uint8_t *mesh_payload,
                                uint16_t mesh_len, mac_rx_event_t *rx)
{
    switch (mhdr->type) {
        case MESH_PKT_JOIN:
            if (mesh_node_id == nodecfg_get_root_id()) {
                uint16_t joining_id = 0;
                if (mesh_len >= sizeof(uint16_t))
                    memcpy(&joining_id, mesh_payload, sizeof(joining_id));
                mesh_set_node_online(joining_id, true);
                ESP_LOGI(TAG, "JOIN from node %u -> marked ONLINE", joining_id);
            }
            break;

        case MESH_PKT_SYNC:
            if (mesh_node_id != nodecfg_get_root_id()) {
                ESP_LOGI(TAG, "SYNC received → leaving WAIT_FOR_SYNC");

                mesh_handle_sync_clock(mesh_payload, mesh_len);

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

                /* Ensure we are in RX so we can receive the upcoming TDMA unicast */
                phy_start_rx_continuous();
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
                    ESP_LOGI(TAG, "TDMA received: base_ms=%lu slot_ms=%lu full_cycles=%u report_every=%u offset=%u slot=%u anchor=%llu hash=0x%08X",
                             (unsigned long)my_tdma_slot.base_period_ms,
                             (unsigned long)my_tdma_slot.slot_duration_ms,
                             (unsigned)my_tdma_slot.full_cycle_cycles,
                             (unsigned)my_tdma_slot.report_every_n_cycles,
                             (unsigned)my_tdma_slot.report_cycle_offset,
                             (unsigned)my_tdma_slot.slot_index,
                             (unsigned long long)my_tdma_slot.anchor_epoch_ms,
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
                mesh_set_node_state(rx->src, MESH_NODE_RUNNING);
                root_log_data_rx(rx->src, mesh_len, rx->rssi, rx->snr);
                if (s_data_at_root_cb)
                    s_data_at_root_cb(rx->src, mesh_payload, mesh_len, rx->rssi, rx->snr);
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

    if (mesh_len > 0) {
        bool printable = true;
        for (int i = 0; i < mesh_len; i++) {
            if (mesh_payload[i] < 32 || mesh_payload[i] > 126) {
                printable = false;
                break;
            }
        }
        if (printable) {
            printf("RX Payload (ascii): ");
            for (int i = 0; i < mesh_len; i++)
                printf("%c", mesh_payload[i]);
            printf("\n");
        } else {
            printf("RX Payload (hex): ");
            for (int i = 0; i < mesh_len; i++)
                printf("%02X ", mesh_payload[i]);
            printf("\n");
        }
    }
}

static void mesh_net_rx_handler(mac_rx_event_t *rx)
{
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

        mesh_process_packet(mhdr, mesh_payload, mesh_len, rx);
        return;
    }

    /* -------------------------------------------------------
       Unicast: payload = [net_header_t][mesh_header_t][mesh_payload]
    -------------------------------------------------------- */
    if (rx->payload_len < sizeof(net_header_t) + sizeof(mesh_header_t))
        return;

    uint8_t *mesh_ptr       = rx->payload + sizeof(net_header_t);
    uint16_t mesh_total_len = rx->payload_len - sizeof(net_header_t);

    mesh_header_t *mhdr    = (mesh_header_t *)mesh_ptr;
    uint8_t *mesh_payload  = mesh_ptr + sizeof(mesh_header_t);
    uint16_t mesh_len      = mesh_total_len - sizeof(mesh_header_t);

    mesh_process_packet(mhdr, mesh_payload, mesh_len, rx);
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
     * SPI bus initialization. For a runtime \"reset\" from the CLI,
     * just reset the radio chip and return it to RX. */
    phy_reset();
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

    if (!from_child) {
        ESP_LOGW(TAG, "Drop: not from my child (last_hop=%u src=%u)",
                last_hop, hdr->src);
        return;
    }

    hdr->ttl--;

    uint16_t next_hop = my_topology.parent_id;
    if (my_topology.parent_id == 0xFFFF)
        return;


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
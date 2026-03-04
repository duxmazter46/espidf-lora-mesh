#include "root_log.h"

#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "ROOT_LOG";

#define ROOT_LOG_NAMESPACE   "root_log"
#define ROOT_LOG_KEY_ENTRIES "entries"

#define ROOT_LOG_MAGIC       0x524C4F47u /* "RLOG" */

/* Log storage limited to 1 MB. Entry size is sizeof(root_log_entry_t) (~33 bytes packed). */
#define ROOT_LOG_MAX_SIZE_BYTES  (1024 * 1024)
#define ROOT_LOG_HEADER_BYTES    (sizeof(uint32_t) + sizeof(uint16_t) + sizeof(uint16_t))
#define ROOT_LOG_ENTRIES_1MB    ((ROOT_LOG_MAX_SIZE_BYTES - ROOT_LOG_HEADER_BYTES) / sizeof(root_log_entry_t))
/* Cap at 4096 entries (~132 KB) to keep RAM/NVS usage reasonable on ESP32. */
#define ROOT_LOG_MAX_ENTRIES     ((ROOT_LOG_ENTRIES_1MB > 4096) ? 4096 : (unsigned)(ROOT_LOG_ENTRIES_1MB))

typedef struct {
    uint32_t magic;
    uint16_t write_index;
    uint16_t count;
    root_log_entry_t entries[ROOT_LOG_MAX_ENTRIES];
} root_log_buffer_t;

static root_log_buffer_t s_log_buffer;
static bool s_log_inited = false;

static uint64_t root_log_get_epoch_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)tv.tv_usec / 1000ULL;
}

static void root_log_persist(void)
{
    if (!s_log_inited) {
        return;
    }

    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK && err != ESP_ERR_NVS_NO_FREE_PAGES && err != ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %d", err);
        return;
    }

    nvs_handle_t h;
    err = nvs_open(ROOT_LOG_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %d", err);
        return;
    }

    err = nvs_set_blob(h, ROOT_LOG_KEY_ENTRIES, &s_log_buffer, sizeof(s_log_buffer));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob failed: %d", err);
        nvs_close(h);
        return;
    }

    nvs_commit(h);
    nvs_close(h);
}

void root_log_init(void)
{
    memset(&s_log_buffer, 0, sizeof(s_log_buffer));
    s_log_buffer.magic = ROOT_LOG_MAGIC;

    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK && err != ESP_ERR_NVS_NO_FREE_PAGES && err != ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %d", err);
        s_log_inited = false;
        return;
    }

    nvs_handle_t h;
    err = nvs_open(ROOT_LOG_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open (init) failed: %d, starting with empty buffer", err);
        s_log_inited = true;
        root_log_persist();
        return;
    }

    size_t required_size = sizeof(s_log_buffer);
    err = nvs_get_blob(h, ROOT_LOG_KEY_ENTRIES, &s_log_buffer, &required_size);
    if (err != ESP_OK || required_size != sizeof(s_log_buffer) || s_log_buffer.magic != ROOT_LOG_MAGIC) {
        ESP_LOGI(TAG, "No valid existing log blob, initializing new buffer");
        memset(&s_log_buffer, 0, sizeof(s_log_buffer));
        s_log_buffer.magic = ROOT_LOG_MAGIC;
        s_log_buffer.write_index = 0;
        s_log_buffer.count = 0;
        nvs_set_blob(h, ROOT_LOG_KEY_ENTRIES, &s_log_buffer, sizeof(s_log_buffer));
        nvs_commit(h);
    }

    nvs_close(h);
    s_log_inited = true;
}

static void root_log_append(root_log_entry_t *entry)
{
    if (!s_log_inited) {
        return;
    }

    entry->ts_ms = root_log_get_epoch_ms();

    uint16_t idx = s_log_buffer.write_index;
    s_log_buffer.entries[idx] = *entry;

    s_log_buffer.write_index = (uint16_t)((idx + 1) % ROOT_LOG_MAX_ENTRIES);
    if (s_log_buffer.count < ROOT_LOG_MAX_ENTRIES) {
        s_log_buffer.count++;
    }

    root_log_persist();
}

static void root_log_print_time(uint64_t epoch_ms)
{
    time_t seconds = (time_t)(epoch_ms / 1000ULL);
    struct tm tm_info;
    gmtime_r(&seconds, &tm_info);

    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_info);
    printf("%s.%03llu UTC", buffer, (unsigned long long)(epoch_ms % 1000ULL));
}

void root_log_mesh_start(void)
{
    root_log_entry_t e;
    memset(&e, 0, sizeof(e));
    e.type = ROOT_LOG_MESH_START;
    e.anchor_ms = root_log_get_epoch_ms();
    root_log_append(&e);
}

void root_log_topology_node(uint16_t node_id,
                            uint16_t parent_id,
                            const uint16_t *children,
                            uint8_t child_count)
{
    root_log_entry_t e;
    memset(&e, 0, sizeof(e));
    e.type = ROOT_LOG_TOPOLOGY_NODE;
    e.node_id = node_id;
    e.parent_id = parent_id;

    if (children != NULL && child_count > 0) {
        e.payload_len = child_count;
    } else {
        e.payload_len = 0;
    }

    root_log_append(&e);
}

void root_log_tdma_entry(uint16_t node_id,
                         uint8_t slot_index,
                         uint8_t total_slots,
                         uint32_t slot_ms,
                         uint64_t anchor_ms)
{
    root_log_entry_t e;
    memset(&e, 0, sizeof(e));
    e.type = ROOT_LOG_TDMA_ENTRY;
    e.node_id = node_id;
    e.slot_index = slot_index;
    e.total_slots = total_slots;
    e.slot_ms = slot_ms;
    e.anchor_ms = anchor_ms;

    root_log_append(&e);
}

void root_log_data_rx(uint16_t src_node_id,
                      uint16_t payload_len,
                      int rssi,
                      float snr)
{
    root_log_entry_t e;
    memset(&e, 0, sizeof(e));
    e.type = ROOT_LOG_DATA_RX;
    e.node_id = src_node_id;
    e.payload_len = payload_len;
    e.rssi = (int16_t)rssi;
    e.snr_times_10 = (int16_t)(snr * 10.0f);

    root_log_append(&e);
}

void root_log_node_status(uint16_t node_id, root_log_node_state_t state)
{
    root_log_entry_t e;
    memset(&e, 0, sizeof(e));
    e.type = ROOT_LOG_NODE_STATUS;
    e.node_id = node_id;
    e.slot_index = (uint8_t)state;  /* reuse for state: OFFLINE=0, ONLINE=1, RUNNING=2 */
    root_log_append(&e);
}

static const char *root_log_node_state_str(uint8_t state)
{
    switch (state) {
    case ROOT_LOG_NODE_OFFLINE: return "OFFLINE";
    case ROOT_LOG_NODE_ONLINE:  return "ONLINE";
    case ROOT_LOG_NODE_RUNNING: return "RUNNING";
    default: return "?";
    }
}

static void root_log_print_entry(uint16_t seq, const root_log_entry_t *e)
{
    printf("[%03u] ", (unsigned)seq);
    root_log_print_time(e->ts_ms);
    printf("  type=%u", (unsigned)e->type);

    switch (e->type) {
    case ROOT_LOG_MESH_START:
        printf("  MESH_START anchor_ms=%llu\n", (unsigned long long)e->anchor_ms);
        break;
    case ROOT_LOG_TOPOLOGY_NODE:
        printf("  TOPO node=%u parent=%u child_count=%u\n",
               (unsigned)e->node_id,
               (unsigned)e->parent_id,
               (unsigned)e->payload_len);
        break;
    case ROOT_LOG_TDMA_ENTRY:
        printf("  TDMA node=%u slot=%u/%u slot_ms=%lu anchor_ms=%llu\n",
               (unsigned)e->node_id,
               (unsigned)e->slot_index,
               (unsigned)e->total_slots,
               (unsigned long)e->slot_ms,
               (unsigned long long)e->anchor_ms);
        break;
    case ROOT_LOG_DATA_RX:
        printf("  DATA_RX node=%u len=%u rssi=%d snr=%.1f\n",
               (unsigned)e->node_id,
               (unsigned)e->payload_len,
               (int)e->rssi,
               (float)e->snr_times_10 / 10.0f);
        break;
    case ROOT_LOG_NODE_STATUS:
        printf("  NODE_STATUS node=%u -> %s\n",
               (unsigned)e->node_id,
               root_log_node_state_str(e->slot_index));
        break;
    default:
        printf("  UNKNOWN\n");
        break;
    }
}

void root_log_dump(void)
{
    if (!s_log_inited) {
        printf("root_log: not initialized\n");
        return;
    }

    printf("root_log: %u entries\n", (unsigned)s_log_buffer.count);

    if (s_log_buffer.count == 0) {
        return;
    }

    uint16_t start = (s_log_buffer.count < ROOT_LOG_MAX_ENTRIES)
                         ? 0
                         : s_log_buffer.write_index;

    for (uint16_t i = 0; i < s_log_buffer.count; i++) {
        uint16_t idx = (uint16_t)((start + i) % ROOT_LOG_MAX_ENTRIES);
        root_log_print_entry(i, &s_log_buffer.entries[idx]);
    }
}

void root_log_dump_tail(unsigned n)
{
    if (!s_log_inited) {
        printf("root_log: not initialized\n");
        return;
    }

    if (s_log_buffer.count == 0) {
        printf("root_log: 0 entries\n");
        return;
    }

    uint16_t total = s_log_buffer.count;
    if (n == 0 || n > total) {
        n = total;
    }

    uint16_t start = (total < ROOT_LOG_MAX_ENTRIES)
                         ? 0
                         : s_log_buffer.write_index;
    uint16_t first_seq = total - n;
    uint16_t first_idx = (uint16_t)((start + first_seq) % ROOT_LOG_MAX_ENTRIES);

    printf("root_log: last %u of %u entries\n", (unsigned)n, (unsigned)total);

    for (uint16_t i = 0; i < n; i++) {
        uint16_t idx = (uint16_t)((first_idx + i) % ROOT_LOG_MAX_ENTRIES);
        root_log_print_entry(first_seq + i, &s_log_buffer.entries[idx]);
    }
}

#define ROOT_LOG_SUMMARY_MAX_NODES 32

void root_log_dump_summary(void)
{
    if (!s_log_inited) {
        printf("root_log: not initialized\n");
        return;
    }

    uint32_t count_mesh_start = 0;
    uint32_t count_tdma = 0;
    uint32_t count_topo = 0;
    uint32_t count_data_rx = 0;
    uint32_t count_node_status = 0;
    uint32_t count_unknown = 0;
    uint32_t data_rx_per_node[ROOT_LOG_SUMMARY_MAX_NODES];

    memset(data_rx_per_node, 0, sizeof(data_rx_per_node));

    if (s_log_buffer.count == 0) {
        printf("root_log: 0 entries\n");
        return;
    }

    uint16_t start = (s_log_buffer.count < ROOT_LOG_MAX_ENTRIES)
                         ? 0
                         : s_log_buffer.write_index;

    for (uint16_t i = 0; i < s_log_buffer.count; i++) {
        uint16_t idx = (uint16_t)((start + i) % ROOT_LOG_MAX_ENTRIES);
        const root_log_entry_t *e = &s_log_buffer.entries[idx];

        switch (e->type) {
        case ROOT_LOG_MESH_START:  count_mesh_start++; break;
        case ROOT_LOG_TDMA_ENTRY:  count_tdma++;       break;
        case ROOT_LOG_TOPOLOGY_NODE: count_topo++;    break;
        case ROOT_LOG_DATA_RX:
            count_data_rx++;
            if (e->node_id < ROOT_LOG_SUMMARY_MAX_NODES) {
                data_rx_per_node[e->node_id]++;
            }
            break;
        case ROOT_LOG_NODE_STATUS: count_node_status++; break;
        default: count_unknown++; break;
        }
    }

    printf("root_log summary (%u entries):\n", (unsigned)s_log_buffer.count);
    printf("  MESH_START:  %lu\n", (unsigned long)count_mesh_start);
    printf("  TDMA_ENTRY: %lu\n", (unsigned long)count_tdma);
    printf("  TOPOLOGY:   %lu\n", (unsigned long)count_topo);
    printf("  DATA_RX:     %lu\n", (unsigned long)count_data_rx);
    printf("  NODE_STATUS:%lu\n", (unsigned long)count_node_status);
    if (count_unknown) {
        printf("  UNKNOWN:    %lu\n", (unsigned long)count_unknown);
    }
    if (count_data_rx > 0) {
        printf("  DATA_RX per node:\n");
        for (unsigned j = 0; j < ROOT_LOG_SUMMARY_MAX_NODES; j++) {
            if (data_rx_per_node[j] != 0) {
                printf("    node %u: %lu\n", (unsigned)j, (unsigned long)data_rx_per_node[j]);
            }
        }
    }
}

void root_log_clear(void)
{
    if (!s_log_inited) {
        return;
    }

    memset(&s_log_buffer, 0, sizeof(s_log_buffer));
    s_log_buffer.magic = ROOT_LOG_MAGIC;
    s_log_buffer.write_index = 0;
    s_log_buffer.count = 0;

    root_log_persist();
}


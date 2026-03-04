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

/* Drift model: SYNC every N cycles so relative drift stays under DRIFT_BOUND_MS.
   DRIFT_PPM = worst-case relative clock drift (ppm); 40 = 20 ppm per node × 2. */
#define DRIFT_BOUND_MS  100
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

        if (cycle_index % every == 0) {
            uint64_t epoch_ms = root_get_epoch_ms();
            mesh_send(MESH_PKT_SYNC, (uint8_t *)&epoch_ms, sizeof(epoch_ms));
            ESP_LOGI(TAG, "Periodic SYNC broadcast (cycle %lu)", (unsigned long)cycle_index);
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
                    ESP_LOGE(TAG, "3 cycles with no packet from any child -> resetting mesh");
                    fflush(stdout);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    restart_root();
                }
            }
        }

        /* Update node status (no DATA -> back to ONLINE), report to log and terminal */
        for (uint16_t i = 0; i < num_online && i < ROOT_MAX_SLOT_NODES; i++) {
            uint16_t nid = p->slot_to_node[i];
            if (nid >= ROOT_MAX_SLOT_NODES)
                continue;
            if (!received_this_cycle[nid])
                mesh_set_node_state(nid, MESH_NODE_ONLINE);
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

static void root_data_at_root_cb(uint16_t node_id, const uint8_t *payload, uint16_t len, int rssi, float snr)
{
    (void)rssi;
    (void)snr;
    if (node_id < ROOT_MAX_SLOT_NODES)
        received_this_cycle[node_id] = true;

    uint64_t ts_ms = root_get_epoch_ms();

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
        root_data_csv_append(node_id, ts_ms, app_payload, app_len);
    } else {
        /* Legacy/short payload without hash. */
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

static void print_human_time(uint64_t epoch_ms)
{
    time_t seconds = epoch_ms / 1000;

    struct tm tm_info;
    gmtime_r(&seconds, &tm_info);  // Always print in UTC

    char buffer[64];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_info);

    printf("%s.%03llu UTC\n", buffer, epoch_ms % 1000);
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
           (unsigned)(s_full_cycle_cycles - 1));

    int64_t sec_to_next = (int64_t)(next_cycle_ms - now_ms) / 1000;
    if (sec_to_next < 0)
        sec_to_next = 0;

    printf("Next cycle in       : %lld s\n", (long long)sec_to_next);

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
           cycle_mod,
           (unsigned)(s_full_cycle_cycles - 1));

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

    /* Phase 1: broadcast SYNC (already sent by mesh_start_runtime) */
    printf("Waiting 2s for broadcast SYNC to settle...\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Phase 2a: ping direct children (tier-1) to verify adjacency */
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

    /* Phase 2: probe OFFLINE nodes with unicast SYNC */
    uint64_t epoch_ms = root_get_epoch_ms();
    for (uint16_t i = 0; i < num_nonroot; i++) {
        uint16_t nid = nonroot_ids[i];
        if (mesh_is_node_online(nid)) {
            printf("  Node %u: already ONLINE\n", nid);
            continue;
        }
        printf("  Node %u OFFLINE, probing with unicast SYNC...\n", nid);
        bool ack = mesh_send_to(nid, MESH_PKT_SYNC,
                                (uint8_t *)&epoch_ms, sizeof(epoch_ms));
        if (ack) {
            mesh_set_node_online(nid, true);
            printf("  Node %u responded -> ONLINE\n", nid);
        } else {
            printf("  Node %u no response -> stays OFFLINE\n", nid);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Phase 3: build TDMA table and send per-node schedules */
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
    uint64_t anchor_epoch_ms = ((epoch_ms / (base_period_s * 1000)) + 1) * (uint64_t)base_period_s * 1000;
    if (base_period_s >= 60)
        anchor_epoch_ms = ((epoch_ms / 60000) + 1) * 60000;  /* align to minute when base is 60s */

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
    printf("Anchor (first cycle start): ");
    print_human_time(anchor_epoch_ms);
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
    for (uint16_t i = 0; i < num_online; i++) {
        uint16_t nid = online_ids[i];
        uint32_t interval_s = nodecfg_get_reporting_interval_s(nid);
        uint16_t report_every = (uint16_t)(interval_s / base_period_s);
        if (report_every == 0)
            report_every = 1;
        uint8_t report_cycle_offset = (uint8_t)(report_every - 1);

        /* Find a cycle where this node reports and get its slot index and slot_duration. */
        uint8_t slot_index = 0;
        uint32_t slot_duration_ms = base_period_ms;
        for (uint16_t c = 0; c < full_cycle_cycles; c++) {
            if ((c % report_every) != report_cycle_offset)
                continue;
            uint8_t n_slots = tdma_cycle_slot_count[c];
            slot_duration_ms = base_period_ms / n_slots;
            for (uint8_t k = 0; k < n_slots - 1; k++) {
                if (tdma_cycle_nodes[c][k] == nid) {
                    slot_index = k;
                    break;
                }
            }
            break;
        }

        mesh_tdma_payload_t tdma = {
            .anchor_epoch_ms       = anchor_epoch_ms,
            .base_period_ms        = base_period_ms,
            .slot_duration_ms      = slot_duration_ms,
            .full_cycle_cycles     = full_cycle_cycles,
            .report_every_n_cycles = report_every,
            .report_cycle_offset   = report_cycle_offset,
            .slot_index            = slot_index,
            .schedule_hash         = s_tdma_hash,
        };
        printf("  -> Node %u: report_every=%u, offset=%u, slot=%u, slot_ms=%lu\n",
               nid, (unsigned)report_every, (unsigned)report_cycle_offset,
               (unsigned)slot_index, (unsigned long)slot_duration_ms);
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

    s_anchor_epoch_ms   = anchor_epoch_ms;
    s_base_period_ms    = base_period_ms;
    s_full_cycle_cycles = full_cycle_cycles;

    root_data_csv_start(anchor_epoch_ms);
    mesh_register_data_at_root_cb(root_data_at_root_cb);

    xTaskCreate(root_sync_task, "root_sync", 3072, &sync_params, 3, &root_sync_task_handle);
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
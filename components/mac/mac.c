#include "mac.h"
#include "phy.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
#include <string.h>
#include <math.h>

#define TAG "MAC"

/* ==============================
   Forward Declarations
   ============================== */

static void mac_phy_event_handler(phy_event_type_t event);

#define MAC_MAX_PAYLOAD 220
#define MAC_BROADCAST_ID 0xFFFF

/* ==============================
   MAC HEADER 
   ============================== */
typedef struct {
    uint16_t src;        // original sender
    uint16_t dst;        // next-hop destination
    uint16_t prev_hop;   // last transmitter
    uint16_t frm_id;     // unique per src
    uint8_t  type;       // REQ, ACK, BRD
    uint8_t  flags;      // reserved for future
} __attribute__((packed)) mac_header_t;

/* ==============================
   INTERNAL STATE
   ============================== */

static uint16_t self_id;

static void (*mac_event_callback)(mac_event_type_t, void *) = NULL;

static uint16_t waiting_ack_frm_id = 0;

#define MAC_DUP_CACHE_SIZE 64
#define MAC_MAX_RETRIES 3

typedef struct {
    uint16_t src;
    uint16_t frm_id;
} mac_dup_entry_t;

static mac_dup_entry_t dup_cache[MAC_DUP_CACHE_SIZE];
static int dup_index = 0;

static volatile bool mac_busy = false;


static EventGroupHandle_t mac_event_group;

#define MAC_EVENT_ACK      BIT0
#define MAC_EVENT_TIMEOUT  BIT1

static mac_tx_type_t last_tx_type = MAC_TX_NONE;
static mac_operating_mode_t mac_mode = MAC_MODE_ACTIVE;


/* ==============================
   Airtime Calculation
   ============================== */

static uint32_t mac_calculate_lora_airtime_us(uint16_t payload_len)
{
    int sf  = phy_get_spreading_factor();
    int bw_index = phy_get_bandwidth();
    int cr  = phy_get_coding_rate();

    // Convert bandwidth index to Hz
    // 0=7.8kHz ... 9=500kHz (SX1276 table)
    const uint32_t bw_table[] = {
        7800, 10400, 15600, 20800, 31250,
        41700, 62500, 125000, 250000, 500000
    };

    uint32_t bw = bw_table[bw_index];

    float t_sym = (float)(1 << sf) / bw;

    float t_preamble = (phy_get_preamble_length() + 4.25f) * t_sym;

    int header_enabled = 1;
    int de = (sf >= 11) ? 1 : 0;

    float payload_symb_nb =
        8 +
        ((float)(
            8 * payload_len
            - 4 * sf
            + 28
            + 16   // CRC enabled
            - (header_enabled ? 20 : 0)
        ) /
        (4 * (sf - 2 * de))) * (cr + 4);

    if (payload_symb_nb < 0)
        payload_symb_nb = 0;

    payload_symb_nb = ceilf(payload_symb_nb);

    float t_payload = payload_symb_nb * t_sym;

    float t_packet = t_preamble + t_payload;

    return (uint32_t)(t_packet * 1e6);
}

static uint16_t mac_generate_frm_id(void)
{
    uint32_t rnd = esp_random();
    uint32_t time = (uint32_t)esp_timer_get_time();

    uint32_t mix = rnd ^ time ^ (self_id << 16);

    return (uint16_t)(mix & 0xFFFF);
}

/* ==============================
   PUBLIC API
   ============================== */

void mac_register_event_callback(void (*cb)(mac_event_type_t, void *))
{
    mac_event_callback = cb;
}

static bool mac_initialized = false;

void mac_init(uint16_t node_id)
{
    self_id = node_id;
    phy_register_event_callback(mac_phy_event_handler);

    if (!mac_initialized) {
        mac_event_group = xEventGroupCreate();
        mac_initialized = true;
    }

    ESP_LOGI(TAG, "MAC initialized. Node ID=%u", self_id);
}

void mac_set_mode(mac_operating_mode_t mode)
{
    mac_mode = mode;
}


/* ==============================
   SEND FUNCTIONS
   ============================== */

static void mac_send_frame(mac_header_t *hdr, uint8_t *payload, uint16_t len)
{
    uint8_t buffer[256];

    memcpy(buffer, hdr, sizeof(mac_header_t));

    if (payload && len > 0) {
        memcpy(buffer + sizeof(mac_header_t), payload, len);
    }

    ESP_LOGI(TAG,
        "TX: src=%u → dst=%u prev=%u frm=%u type=%u len=%u",
        hdr->src,
        hdr->dst,
        hdr->prev_hop,
        hdr->frm_id,
        hdr->type,
        len);

    /* -------------------------
       Payload Debug Print
    -------------------------- */
    if (payload && len > 0) {

        bool printable = true;

        for (int i = 0; i < len; i++) {
            if (payload[i] < 32 || payload[i] > 126) {
                printable = false;
                break;
            }
        }

        if (printable) {
            printf("TX Payload (ascii): ");
            for (int i = 0; i < len; i++) {
                printf("%c", payload[i]);
            }
            printf("\n");
        }
        else {
            printf("TX Payload (hex): ");
            for (int i = 0; i < len; i++) {
                printf("%02X ", payload[i]);
            }
            printf("\n");
        }
    }

    phy_send_bit(buffer, sizeof(mac_header_t) + len);
}

bool mac_send(uint16_t dst, uint8_t *payload, uint16_t len)
{   
    ESP_LOGI(TAG, "mac_busy=%d", mac_busy);
    last_tx_type = MAC_TX_DATA;
    if (len > MAC_MAX_PAYLOAD) return false;

    if (mac_busy) {
        ESP_LOGW(TAG, "MAC busy, drop send");
        return false;
    }
    mac_busy = true;

    mac_header_t hdr;

    hdr.src = self_id;
    hdr.dst = dst;
    hdr.prev_hop = self_id;
    hdr.frm_id = mac_generate_frm_id();
    hdr.type = MAC_TYPE_REQ;
    hdr.flags = 0;

    waiting_ack_frm_id = hdr.frm_id;

    xEventGroupClearBits(mac_event_group, MAC_EVENT_ACK);

    uint16_t total_len = sizeof(mac_header_t) + len;

    uint32_t tx_airtime_us = mac_calculate_lora_airtime_us(total_len);
    uint32_t ack_airtime_us = mac_calculate_lora_airtime_us(sizeof(mac_header_t));

    uint32_t processing_margin_us = 5000;

    uint32_t ack_timeout_us =
        tx_airtime_us + ack_airtime_us + processing_margin_us;

    uint32_t min_timeout_us = 500000;

    if (ack_timeout_us < min_timeout_us)
        ack_timeout_us = min_timeout_us;

    TickType_t timeout_ticks = pdMS_TO_TICKS(ack_timeout_us / 1000);

    int retries = 0;
    bool acked = false;

    while (retries < MAC_MAX_RETRIES && !acked) {

        xEventGroupClearBits(mac_event_group, MAC_EVENT_ACK);

        mac_send_frame(&hdr, payload, len);

        EventBits_t bits = xEventGroupWaitBits(
            mac_event_group,
            MAC_EVENT_ACK,
            pdTRUE,
            pdFALSE,
            timeout_ticks
        );

        if (bits & MAC_EVENT_ACK) {
            acked = true;
            break;
        }

        retries++;

        if (retries < MAC_MAX_RETRIES) {

            uint32_t backoff_ms = 20 + (esp_random() % 200);

            ESP_LOGW(TAG,
                     "ACK timeout, retrying in %lu ms",
                     backoff_ms);

            vTaskDelay(pdMS_TO_TICKS(backoff_ms));
        }
    }

    if (acked) {
        if (mac_event_callback)
            mac_event_callback(MAC_EVENT_TX_DONE, NULL);
        mac_busy = false;
        return true;
    }
    else {
        if (mac_event_callback)
            mac_event_callback(MAC_EVENT_TIMEOUT, NULL);
        mac_busy = false;
        return false;
    }

}

void mac_broadcast(uint8_t *payload, uint16_t len)
{
    if (len > MAC_MAX_PAYLOAD) return;

    mac_header_t hdr;

    hdr.src = self_id;
    hdr.dst = MAC_BROADCAST_ID;
    hdr.prev_hop = self_id;
    hdr.frm_id = 0;
    hdr.type = MAC_TYPE_BRD;
    hdr.flags = 0;

    /* So PHY TX_DONE handler will call phy_start_rx_continuous() and root stays in RX */
    last_tx_type = MAC_TX_DATA;

    mac_send_frame(&hdr, payload, len);
}

/* ==============================
   ACK FUNCTION
   ============================== */

static void mac_send_ack(mac_header_t *rx_hdr)
{
    last_tx_type = MAC_TX_ACK;
    mac_header_t ack;

    ack.src = self_id;
    ack.dst = rx_hdr->src;
    ack.prev_hop = self_id;
    ack.frm_id = rx_hdr->frm_id;
    ack.type = MAC_TYPE_ACK;
    ack.flags = 0;

    // Small deterministic guard time
    uint32_t sifs_us = 30000;  // 30ms is usually enough

    esp_rom_delay_us(sifs_us);

    ESP_LOGI(TAG, "ACK frm_id=%u", rx_hdr->frm_id);
    mac_send_frame(&ack, NULL, 0);
}

static bool mac_is_duplicate(uint16_t src, uint16_t frm_id)
{
    for (int i = 0; i < MAC_DUP_CACHE_SIZE; i++) {
        if (dup_cache[i].src == src &&
            dup_cache[i].frm_id == frm_id) {
            return true;
        }
    }
    return false;
}

static void mac_store_duplicate(uint16_t src, uint16_t frm_id)
{
    dup_cache[dup_index].src = src;
    dup_cache[dup_index].frm_id = frm_id;
    dup_index = (dup_index + 1) % MAC_DUP_CACHE_SIZE;
}


/* ==============================
   PHY EVENT HANDLER
   ============================== */
static const char *phy_event_to_str(phy_event_type_t e)
{
    switch(e) {
        case PHY_EVENT_TX_DONE: return "TX_DONE";
        case PHY_EVENT_RX_DONE: return "RX_DONE";
        case PHY_EVENT_CRC_ERROR: return "CRC_ERROR";
        default: return "UNKNOWN";
    }
}

void mac_phy_event_handler(phy_event_type_t event)
{   
    ESP_LOGI(TAG, "PHY_EVENT: %s", phy_event_to_str(event));

    if (event == PHY_EVENT_RX_DONE) {

        uint8_t buf[256];
        int len = phy_get_bit(buf, sizeof(buf));

        if (len < sizeof(mac_header_t))
            return;

        mac_header_t *hdr = (mac_header_t *)buf;

        int rssi = phy_bit_rssi();
        float snr = phy_bit_snr();

        /* -----------------------------
        Frame is FOR ME
        ------------------------------ */
        if (hdr->dst == self_id) {

            ESP_LOGI(TAG,
                "RX (FOR ME): src=%u frm=%u RSSI=%d SNR=%.2f",
                hdr->src,
                hdr->frm_id,
                rssi,
                snr);

            /* ACK received */
            if (hdr->type == MAC_TYPE_ACK &&
                hdr->frm_id == waiting_ack_frm_id) {

                xEventGroupSetBits(mac_event_group, MAC_EVENT_ACK);
                return;
            }

            /* Auto ACK for unicast */
            if (hdr->type == MAC_TYPE_REQ) {

                if (mac_is_duplicate(hdr->src, hdr->frm_id)) {
                    mac_send_ack(hdr);
                    return;
                }

                mac_store_duplicate(hdr->src, hdr->frm_id);
                mac_send_ack(hdr);
            }

            mac_rx_event_t rx_event;

            rx_event.src = hdr->src;
            rx_event.dst = hdr->dst;
            rx_event.prev_hop = hdr->prev_hop;
            rx_event.frm_id = hdr->frm_id;
            rx_event.type = hdr->type;
            rx_event.payload = buf + sizeof(mac_header_t);
            rx_event.payload_len = len - sizeof(mac_header_t);
            rx_event.rssi = rssi;
            rx_event.snr = snr;

            if (mac_event_callback)
                mac_event_callback(MAC_EVENT_RX, &rx_event);

            return;
        }

        /* -----------------------------
        Broadcast
        ------------------------------ */
        if (hdr->dst == MAC_BROADCAST_ID) {

            ESP_LOGI(TAG,
                "RX (BROADCAST): src=%u RSSI=%d SNR=%.2f",
                hdr->src,
                rssi,
                snr);

            mac_rx_event_t rx_event;

            rx_event.src = hdr->src;
            rx_event.dst = hdr->dst;
            rx_event.prev_hop = hdr->prev_hop;
            rx_event.frm_id = hdr->frm_id;
            rx_event.type = hdr->type;
            rx_event.payload = buf + sizeof(mac_header_t);
            rx_event.payload_len = len - sizeof(mac_header_t);
            rx_event.rssi = rssi;
            rx_event.snr  = snr;

            if (mac_event_callback)
                mac_event_callback(MAC_EVENT_RX, &rx_event);

            return;
        }

        /* -----------------------------
        Not for me → only channel busy log
        ------------------------------ */

        ESP_LOGW(TAG,
            "Channel busy (not for me). RSSI=%d SNR=%.2f",
            rssi,
            snr);

        if (mac_event_callback) {

            mac_forward_event_t fwd;

            fwd.src       = hdr->src;
            fwd.dst       = hdr->dst;
            fwd.prev_hop  = hdr->prev_hop;
            fwd.frm_id    = hdr->frm_id;
            fwd.type      = hdr->type;
            fwd.flags     = hdr->flags;

            fwd.payload     = buf + sizeof(mac_header_t);
            fwd.payload_len = len - sizeof(mac_header_t);

            mac_event_callback(MAC_EVENT_FORWARD, &fwd);

            phy_start_rx_continuous();
        }
    }
    else if (event == PHY_EVENT_TX_DONE) {
        
            if (last_tx_type == MAC_TX_DATA) {
                // Expect ACK
                phy_start_rx_continuous();
            }
            else if (last_tx_type == MAC_TX_ACK) {
            
                if (mac_mode == MAC_MODE_PASSIVE) {

                    phy_standby();
                    phy_start_rx_continuous();

                } else {

                    // ACK does NOT expect ACK
                    phy_standby();

                }

            }

            last_tx_type = MAC_TX_NONE;
            return;
    }


}
#include "net.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include <string.h>

#define TAG "NET"

    /* =========================
    Internal State
    ========================= */
static uint16_t net_self_id;

    /* =========================
    Forward Declaration
    ========================= */

static void net_mac_event_handler(mac_event_type_t type, void *event_data);
static void (*net_event_callback)(mac_event_type_t type) = NULL;

void net_register_event_callback(void (*cb)(mac_event_type_t type))
{
    net_event_callback = cb;
}

static net_rx_callback_t app_rx_callback = NULL;

void net_register_rx_callback(net_rx_callback_t cb)
{
        app_rx_callback = cb;
}

static net_forward_callback_t forward_callback = NULL;

void net_register_forward_callback(net_forward_callback_t cb)
{
        forward_callback = cb;
}

static net_routing_callback_t routing_callback = NULL;

void net_register_routing_callback(net_routing_callback_t cb)
{
    routing_callback = cb;
}

/* =========================
Init
========================= */

void net_init(uint16_t self_id)
{
        net_self_id = self_id;

        mac_init(net_self_id);
        /* Register NET as MAC upper-layer handler */
        mac_register_event_callback(net_mac_event_handler);

        ESP_LOGI(TAG, "NET initialized. Node ID=%u", net_self_id);
}

/* =========================
Send API
========================= */

bool net_send(uint16_t final_dst, uint8_t *payload, uint16_t len)
{
    net_header_t hdr;

    hdr.src = net_self_id;
    hdr.dst = final_dst;
    hdr.pkt_id = esp_random() & 0xFFFF;
    hdr.ttl = 5;

    uint8_t buffer[256];

    memcpy(buffer, &hdr, sizeof(hdr));
    memcpy(buffer + sizeof(hdr), payload, len);

    uint16_t next_hop = final_dst;

    if (final_dst == MAC_BROADCAST_ID) {
        ESP_LOGI(TAG, "Broadcasting packet to all nodes");
        mac_broadcast(buffer, sizeof(hdr) + len);
        return true;
    }

    if (routing_callback) {
        next_hop = routing_callback(final_dst);
    }

    ESP_LOGI(TAG,
        "TX: src=%u → final=%u next=%u pkt=%u ttl=%u len=%u",
        hdr.src,
        hdr.dst,
        next_hop,
        hdr.pkt_id,
        hdr.ttl,
        len);

    return mac_send(next_hop,
                    buffer,
                    sizeof(hdr) + len);
}

/* =========================
MAC Event Handler
========================= */

static void net_mac_event_handler(mac_event_type_t type, void *event_data)
{
        switch (type)
        {
            case MAC_EVENT_RX:
            {
                mac_rx_event_t *rx = (mac_rx_event_t *)event_data;

                if (rx->dst == MAC_BROADCAST_ID) {
                    ESP_LOGI(TAG, "RX broadcast from %u, len=%u → app",
                             rx->src, rx->payload_len);
                    if (app_rx_callback)
                        app_rx_callback(rx);
                    break;
                }

                /* Unicast MAC-for-me: check final destination; forward if not for us. */
                if (rx->dst == net_self_id) {
                    if (rx->payload_len >= sizeof(net_header_t)) {
                        net_header_t *hdr = (net_header_t *)rx->payload;
                        uint8_t *net_payload = rx->payload + sizeof(net_header_t);
                        uint16_t net_len = rx->payload_len - sizeof(net_header_t);
                        if (hdr->dst != net_self_id && forward_callback) {
                            /* Relay: final destination is another node. */
                            ESP_LOGI(TAG,
                                "FWD: src=%u → dst=%u pkt=%u ttl=%u len=%u",
                                hdr->src, hdr->dst, hdr->pkt_id, hdr->ttl, net_len);
                            forward_callback(rx, hdr, net_payload, net_len);
                            break;
                        }
                    }
                    if (app_rx_callback)
                        app_rx_callback(rx);
                    break;
                }

                if (rx->payload_len < sizeof(net_header_t))
                    break;

                net_header_t *hdr = (net_header_t *)rx->payload;

                uint8_t *net_payload = rx->payload + sizeof(net_header_t);
                uint16_t net_len = rx->payload_len - sizeof(net_header_t);
                
                ESP_LOGI(TAG,
                    "RX: src=%u → dst=%u pkt=%u ttl=%u len=%u",
                    hdr->src,
                    hdr->dst,
                    hdr->pkt_id,
                    hdr->ttl,
                    net_len);
                    
                if (hdr->dst == net_self_id) {

                    if (app_rx_callback)
                        app_rx_callback(rx);

                } else {

                if (forward_callback) {

                    ESP_LOGI(TAG,
                        "FWD: src=%u → dst=%u pkt=%u ttl=%u len=%u",
                        hdr->src,
                        hdr->dst,
                        hdr->pkt_id,
                        hdr->ttl,
                        net_len);

                    forward_callback(rx,
                                    hdr,
                                    net_payload,
                                    net_len);
                }
                }
                break;
            }

            case MAC_EVENT_FORWARD:
            {
                mac_forward_event_t *fwd =
                    (mac_forward_event_t *)event_data;

                if (fwd->payload_len < sizeof(net_header_t))
                    break;

                net_header_t *hdr =
                    (net_header_t *)fwd->payload;

                uint8_t *net_payload =
                    fwd->payload + sizeof(net_header_t);

                uint16_t net_len =
                    fwd->payload_len - sizeof(net_header_t);

                if (forward_callback) {
                    mac_rx_event_t faux = {0};
                    faux.src = fwd->src;
                    faux.dst = fwd->dst;
                    faux.prev_hop = fwd->prev_hop;     
                    faux.frm_id = fwd->frm_id;
                    faux.type = fwd->type;
                    faux.payload = fwd->payload;
                    faux.payload_len = fwd->payload_len;

                    forward_callback(&faux, hdr, net_payload, net_len);
                }

                break;
            }


            case MAC_EVENT_TX_DONE:
            {
                ESP_LOGI(TAG, "MAC TX done");

                if (net_event_callback)
                    net_event_callback(MAC_EVENT_TX_DONE);

                break;
            }

            case MAC_EVENT_TIMEOUT:
            {
                ESP_LOGW(TAG, "MAC timeout");

                if (net_event_callback)
                    net_event_callback(MAC_EVENT_TIMEOUT);

                break;
            }

            default:
                break;
        }
}

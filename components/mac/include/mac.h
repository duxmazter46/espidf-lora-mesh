#ifndef MAC_H
#define MAC_H

#include <stdint.h>
#include <stdbool.h>

/* =========================
   MAC Packet Types
   ========================= */

#define MAC_TYPE_REQ 0
#define MAC_TYPE_ACK 1
#define MAC_TYPE_BRD 2

#define MAC_BROADCAST_ID 0xFFFF

/* =========================
   MAC Event Types
   ========================= */

typedef enum {
    MAC_EVENT_RX,
    MAC_EVENT_TX_DONE,
    MAC_EVENT_TIMEOUT,
    MAC_EVENT_FORWARD
} mac_event_type_t;


/* =========================
   RX Event Structure
   ========================= */

typedef struct {
    uint16_t src;          // original sender
    uint16_t dst;          // next-hop destination
    uint16_t prev_hop;     // last transmitter (neighbor)
    uint16_t frm_id;
    uint8_t  type;
    uint8_t *payload;
    uint16_t payload_len;
    int      rssi;
    float    snr;
} mac_rx_event_t;

typedef enum {
    MAC_TX_NONE,
    MAC_TX_DATA,
    MAC_TX_ACK
} mac_tx_type_t;

typedef enum {
    MAC_MODE_ACTIVE,     // sender-style node
    MAC_MODE_PASSIVE     // always listening node
} mac_operating_mode_t;

/* =========================
   FORWARD Event Structure
   ========================= */

typedef struct {
    uint16_t src;
    uint16_t dst;
    uint16_t prev_hop;
    uint16_t frm_id;
    uint8_t  type;
    uint8_t  flags;
    uint8_t *payload;
    uint16_t payload_len;
} mac_forward_event_t;



/* =========================
   Callback Registration
   ========================= */

void mac_register_event_callback(void (*cb)(mac_event_type_t, void *event_data));

/* =========================
   Init
   ========================= */

void mac_init(uint16_t node_id);

/* =========================
   Send API
   ========================= */

bool mac_send(uint16_t dst, uint8_t *payload, uint16_t len);
void mac_broadcast(uint8_t *payload, uint16_t len);
void mac_set_mode(mac_operating_mode_t mode);

#endif

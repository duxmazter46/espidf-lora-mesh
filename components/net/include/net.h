#ifndef NET_H
#define NET_H

#include <stdint.h>
#include <stdbool.h>
#include "mac.h"

typedef struct {
    uint16_t src;      // original source
    uint16_t dst;      // final destination
    uint16_t pkt_id;   // end-to-end ID
    uint8_t  ttl;      // end-to-end hop limit
} __attribute__((packed)) net_header_t;

void net_init(uint16_t self_id);
bool net_send(
    uint16_t final_dst, 
    uint8_t *payload, 
    uint16_t len);

typedef void (*net_rx_callback_t)(mac_rx_event_t *rx);
void net_register_rx_callback(net_rx_callback_t cb);

typedef void (*net_forward_callback_t)(
    mac_rx_event_t *rx,
    net_header_t *hdr,
    uint8_t *payload,
    uint16_t len
);

void net_register_forward_callback(net_forward_callback_t cb);

typedef uint16_t (*net_routing_callback_t)(uint16_t final_dst);

void net_register_routing_callback(net_routing_callback_t cb);

void net_register_event_callback(void (*cb)(mac_event_type_t type));

#endif

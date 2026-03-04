#ifndef PHY_H
#define PHY_H

#include <stdbool.h>
#include <stdint.h>

/* ============================================================
 *  Mode Definitions
 * ============================================================ */

#define MODE_SLEEP            0x00
#define MODE_STDBY            0x01
#define MODE_TX               0x03
#define MODE_RX_CONTINUOUS    0x05
#define MODE_RX_SINGLE        0x06
#define MODE_CAD              0x07

typedef enum {
    LORA_STATE_SLEEP          = MODE_SLEEP,
    LORA_STATE_STANDBY        = MODE_STDBY,
    LORA_STATE_TX             = MODE_TX,
    LORA_STATE_RX_CONTINUOUS  = MODE_RX_CONTINUOUS,
    LORA_STATE_RX_SINGLE      = MODE_RX_SINGLE,
    LORA_STATE_CAD            = MODE_CAD,
    LORA_STATE_UNINIT         = -1
} lora_state_t;

/*
 * IRQ masks
 */
 #define IRQ_TX_DONE_MASK               0x08
 #define IRQ_PAYLOAD_CRC_ERROR_MASK     0x20
 #define IRQ_RX_DONE_MASK               0x40
 #define IRQ_CAD_DONE_MASK              0x08   /* same bit as TxDone; distinguish by state */
 #define IRQ_CAD_DETECTED_MASK          0x02


/* ============================================================
 *  Initialization & State
 * ============================================================ */

void phy_apply_kconfig_defaults(void);
int  phy_init(void);
void phy_reset(void);          
void phy_sleep(void);
void phy_standby(void);
void phy_start_rx_continuous(void);
void phy_start_rx_single(void);          
lora_state_t phy_get_state(void);


/* ============================================================
 *  Radio Configuration
 * ============================================================ */

void phy_set_frequency(long frequency);
void phy_set_tx_power(int level);

void phy_set_spreading_factor(int sf);
int  phy_get_spreading_factor(void);

void phy_set_bandwidth(int sbw);
int  phy_get_bandwidth(void);

void phy_set_coding_rate(int cr);
int  phy_get_coding_rate(void);

void phy_set_preamble_length(long length);
long phy_get_preamble_length(void);

void phy_set_sync_word(int sw);

void phy_enable_crc(void);
void phy_disable_crc(void);

void phy_explicit_header_mode(void);
void phy_implicit_header_mode(int size);


/* ============================================================
 *  Bit Operations
 * ============================================================ */

void phy_send_bit(uint8_t *buf, int size);
int  phy_get_bit(uint8_t *buf, int max_len);

int   phy_get_irq(void);
int   phy_bit_rssi(void);
float phy_bit_snr(void);

/* Channel Activity Detection: returns true if channel is clear, false if busy or timeout */
bool  phy_cad_is_channel_clear(void);




/* ============================================================
 *  Event Callback
 * ============================================================ */

typedef enum {
    PHY_EVENT_IRQ,
    PHY_EVENT_TX_DONE,
    PHY_EVENT_RX_DONE,
    PHY_EVENT_CRC_ERROR,
    PHY_EVENT_FHSS_CHANGE
} phy_event_type_t;

void phy_register_event_callback(void (*cb)(phy_event_type_t));


/* ============================================================
 *  Debug
 * ============================================================ */

void phy_dump_registers(void);   // still phy_dump_registers in .c

#endif

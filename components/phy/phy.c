/*
 * LoRa PHY driver for SX127x.
 * Based on code from: https://github.com/nopnop2002/esp-idf-sx127x
 * Credit: nopnop2002
 */

/* ============================================================
 *  Includes
 * ============================================================ */

 #include "phy.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/* ============================================================
 *  Hardware Configuration
 * ============================================================ */

#define HOST_ID SPI2_HOST 
#define CONFIG_RST_GPIO   4 
#define CONFIG_MOSI_GPIO  35
#define CONFIG_SCK_GPIO   36
#define CONFIG_MISO_GPIO  37
#define CONFIG_CS_GPIO   38
#define CONFIG_DIO0_GPIO  40
#define CONFIG_DIO1_GPIO  41
#define CONFIG_DIO2_GPIO  42

/*
 * Register definitions
 */
#define REG_FIFO                       0x00
#define REG_OP_MODE                    0x01
#define REG_FRF_MSB                    0x06
#define REG_FRF_MID                    0x07
#define REG_FRF_LSB                    0x08
#define REG_PA_CONFIG                  0x09
#define REG_LNA                        0x0c
#define REG_FIFO_ADDR_PTR              0x0d
#define REG_FIFO_TX_BASE_ADDR          0x0e
#define REG_FIFO_RX_BASE_ADDR          0x0f
#define REG_FIFO_RX_CURRENT_ADDR       0x10
#define REG_IRQ_FLAGS                  0x12
#define REG_RX_NB_BYTES                0x13
#define REG_BIT_SNR_VALUE              0x19
#define REG_BIT_RSSI_VALUE             0x1a
#define REG_MODEM_CONFIG_1             0x1d
#define REG_MODEM_CONFIG_2             0x1e
#define REG_PREAMBLE_MSB               0x20
#define REG_PREAMBLE_LSB               0x21
#define REG_PAYLOAD_LENGTH             0x22
#define REG_MODEM_CONFIG_3             0x26
#define REG_RSSI_WIDEBAND              0x2c
#define REG_DETECTION_OPTIMIZE         0x31
#define REG_DETECTION_THRESHOLD        0x37
#define REG_SYNC_WORD                  0x39
#define REG_DIO_MAPPING_1              0x40
#define REG_DIO_MAPPING_2              0x41
#define REG_VERSION                    0x42

/*
 * LoRa OpMode: high bit selects LoRa (vs FSK); low bits = mode
 */
#define MODE_LONG_RANGE_MODE           0x80

/*
 * PA configuration
 */
#define PA_BOOST                       0x80

#define PA_OUTPUT_RFO_PIN              0
#define PA_OUTPUT_PA_BOOST_PIN         1

#define TIMEOUT_RESET                  100

// SPI Stuff
#if CONFIG_SPI2_HOST
#define HOST_ID SPI2_HOST
#elif CONFIG_SPI3_HOST
#define HOST_ID SPI3_HOST
#endif

/* ============================================================
 *  Private State
 * ============================================================ */

#define TAG "PHY"

typedef struct {
    phy_event_type_t type;
} phy_event_t;

typedef struct {
    uint16_t len;
    uint8_t  data[256];
} phy_bit_t;

static QueueHandle_t phy_event_queue;
static QueueHandle_t phy_rx_queue;
static SemaphoreHandle_t phy_cad_done_sem;
static volatile bool phy_cad_detected_result;

static spi_device_handle_t _spi;
static lora_state_t _state = LORA_STATE_UNINIT;

static int _implicit;
static long _frequency;
static int _cr = 0;
static int _sbw = 0;
static int _sf = 0;


// use spi_device_transmit
#define SPI_TRANSMIT 1

// use buffer io
// A little faster
#define BUFFER_IO 1

static void (*phy_event_callback)(phy_event_type_t event) = NULL;

void phy_register_event_callback(void (*cb)(phy_event_type_t))
{
    phy_event_callback = cb;
}



/* ============================================================
 *  Forward Declarations
 * ============================================================ */

static void IRAM_ATTR phy_dio0_isr(void *arg);
static void phy_task(void *arg);
/**
 * Write a value to a register.  * @param reg Register index.
 * @param val Value to write.
 */
static void 
phy_write_reg(int reg, int val)
{
   uint8_t out[2] = { 0x80 | reg, val };
   uint8_t in[2];

   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in  
   };

   //gpio_set_level(CONFIG_CS_GPIO, 0);
#if SPI_TRANSMIT
   spi_device_transmit(_spi, &t);
#else
   spi_device_polling_transmit(_spi, &t);
#endif
   //gpio_set_level(CONFIG_CS_GPIO, 1);
}

/**
 * Write a buffer to a register.
 * @param reg Register index.
 * @param val Value to write.
 * @param len Byte length to write.
 */
static void
phy_write_reg_buffer(int reg, uint8_t *val, int len)
{
    uint8_t out[len + 1];

    out[0] = 0x80 | reg;
    memcpy(&out[1], val, len);

    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * (len + 1),
        .tx_buffer = out,
        .rx_buffer = NULL
    };

    spi_device_transmit(_spi, &t);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
static int
phy_read_reg(int reg)
{
   uint8_t out[2] = { reg, 0xff };
   uint8_t in[2];

   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in
   };

   //gpio_set_level(CONFIG_CS_GPIO, 0);
#if SPI_TRANSMIT
   spi_device_transmit(_spi, &t);
#else
   spi_device_polling_transmit(_spi, &t);
#endif
   //gpio_set_level(CONFIG_CS_GPIO, 1);
   return in[1];
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 * @param len Byte length to read.
 */
static void 
phy_read_reg_buffer(int reg, uint8_t *val, int len)
{
    uint8_t out[len + 1];
    uint8_t in[len + 1];

    out[0] = reg;
    memset(&out[1], 0xFF, len);

    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * (len + 1),
        .tx_buffer = out,
        .rx_buffer = in
    };

    spi_device_transmit(_spi, &t);

    memcpy(val, &in[1], len);
}

/**
 * Perform physical reset on the Lora chip
 */
void 
phy_reset(void)
{
   gpio_set_level(CONFIG_RST_GPIO, 0);
   vTaskDelay(pdMS_TO_TICKS(1));
   gpio_set_level(CONFIG_RST_GPIO, 1);
   vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Configure explicit header mode.
 * Bit size will be included in the frame.
 */
void 
phy_explicit_header_mode(void)
{
   _implicit = 0;
   phy_write_reg(REG_MODEM_CONFIG_1, phy_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
}

/**
 * Configure implicit header mode.
 * All Bits will have a predefined size.
 * @param size Size of the bits.
 */
void 
phy_implicit_header_mode(int size)
{
   _implicit = 1;
   phy_write_reg(REG_MODEM_CONFIG_1, phy_read_reg(REG_MODEM_CONFIG_1) | 0x01);
   phy_write_reg(REG_PAYLOAD_LENGTH, size);
}


static const char *phy_mode_to_str(uint8_t mode)
{
    switch (mode) {
        case MODE_SLEEP:            return "SLEEP";
        case MODE_STDBY:            return "STANDBY";
        case MODE_TX:               return "TX";
        case MODE_RX_CONTINUOUS:    return "RX_CONTINUOUS";
        case MODE_RX_SINGLE:        return "RX_SINGLE";
        case MODE_CAD:              return "CAD";
        default:                    return "UNKNOWN";
    }
}

/**
 * Sets the radio transceiver to mode.
 * Must be used to change registers and access the FIFO.
 */
static void phy_set_mode(uint8_t hw_mode)
{
    hw_mode &= 0x07;  // ensure valid mode bits only

    uint8_t previous = _state;

   if (_state == hw_mode) {
      return;   // already in this mode
   }
   phy_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | hw_mode);
   _state = hw_mode;

   ESP_LOGI(TAG, "MODE: %s -> %s",
            phy_mode_to_str(previous),
            phy_mode_to_str(hw_mode));

}



/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void 
phy_set_tx_power(int level)
{
   // RF9x module uses PA_BOOST pin
   if (level < 2) level = 2;
   else if (level > 17) level = 17;
   phy_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void 
phy_set_frequency(long frequency)
{
   _frequency = frequency;

   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   phy_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   phy_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   phy_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void 
phy_set_spreading_factor(int sf)
{
   if (sf < 6) sf = 6;
   else if (sf > 12) sf = 12;

   if (sf == 6) {
      phy_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
      phy_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
   } else {
      phy_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
      phy_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
   }

   phy_write_reg(REG_MODEM_CONFIG_2, (phy_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
   _sf = sf;
}

/**
 * Get spreading factor.
 */
int 
phy_get_spreading_factor(void)
{
   return (phy_read_reg(REG_MODEM_CONFIG_2) >> 4);
}

/**
 * Set Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 * @param mode mode of DIO(0 to 3)
 */
void 
phy_set_dio_mapping(int dio, int mode)
{
   if (dio < 4) {
      int _mode = phy_read_reg(REG_DIO_MAPPING_1);
      if (dio == 0) {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      } else if (dio == 1) {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      } else if (dio == 2) {
         _mode = _mode & 0xF3;
         _mode = _mode | (mode << 2);
      } else if (dio == 3) {
         _mode = _mode & 0xFC;
         _mode = _mode | mode;
      }
      phy_write_reg(REG_DIO_MAPPING_1, _mode);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
   } else if (dio < 6) {
      int _mode = phy_read_reg(REG_DIO_MAPPING_2);
      if (dio == 4) {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      } else if (dio == 5) {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      }
      ESP_LOGD(TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
      phy_write_reg(REG_DIO_MAPPING_2, _mode);
   }
}

/**
 * Get Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 */
int 
phy_get_dio_mapping(int dio)
{
   if (dio < 4) {
      int _mode = phy_read_reg(REG_DIO_MAPPING_1);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
      if (dio == 0) {
         return ((_mode >> 6) & 0x03);
      } else if (dio == 1) {
         return ((_mode >> 4) & 0x03);
      } else if (dio == 2) {
         return ((_mode >> 2) & 0x03);
      } else if (dio == 3) {
         return (_mode & 0x03);
      }
   } else if (dio < 6) {
      int _mode = phy_read_reg(REG_DIO_MAPPING_2);
      ESP_LOGD(TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
      if (dio == 4) {
         return ((_mode >> 6) & 0x03);
      } else if (dio == 5) {
         return ((_mode >> 4) & 0x03);
      }
   }
   return 0;
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
void 
phy_set_bandwidth(int sbw)
{
   if (sbw < 10) {
      phy_write_reg(REG_MODEM_CONFIG_1, (phy_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (sbw << 4));
      _sbw = sbw;
   }
}

/**
 * Get bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
int 
phy_get_bandwidth(void)
{
   //int bw;
   //bw = phy_read_reg(REG_MODEM_CONFIG_1) & 0xf0;
   //ESP_LOGD(TAG, "bw=0x%02x", bw);
   //bw = bw >> 4;
   //return bw;
   return ((phy_read_reg(REG_MODEM_CONFIG_1) & 0xf0) >> 4);
}

/**
 * Set coding rate 
 * @param cr Coding Rate(1 to 4)
 */ 
void 
phy_set_coding_rate(int cr)
{
   //if (denominator < 5) denominator = 5;
   //else if (denominator > 8) denominator = 8;

   //int cr = denominator - 4;
   if (cr < 1) cr = 1;
   else if (cr > 4) cr = 4;
   phy_write_reg(REG_MODEM_CONFIG_1, (phy_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
   _cr = cr;
}

/**
 * Get coding rate 
 */ 
int 
phy_get_coding_rate(void)
{
   return ((phy_read_reg(REG_MODEM_CONFIG_1) & 0x0E) >> 1);
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void 
phy_set_preamble_length(long length)
{
   phy_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
   phy_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Get the size of preamble.
 */
long
phy_get_preamble_length(void)
{
   long preamble;
   preamble = phy_read_reg(REG_PREAMBLE_MSB) << 8;
   preamble = preamble + phy_read_reg(REG_PREAMBLE_LSB);
   return preamble;
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void 
phy_set_sync_word(int sw)
{
   phy_write_reg(REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying bits CRC.
 */
void 
phy_enable_crc(void)
{
   phy_write_reg(REG_MODEM_CONFIG_2, phy_read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

/**
 * Disable appending/verifying bits CRC.
 */
void 
phy_disable_crc(void)
{
   phy_write_reg(REG_MODEM_CONFIG_2, phy_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void 
phy_sleep(void)
{ 
   phy_set_mode(MODE_SLEEP);
}

void phy_standby(void ){
    phy_set_mode(MODE_STDBY);
}

void phy_start_rx_continuous(void) { 
    phy_set_mode(MODE_RX_CONTINUOUS); 
}

void phy_start_rx_single(void) { 
    phy_set_mode(MODE_RX_SINGLE); 
}

void phy_apply_kconfig_defaults(void)
{
    phy_set_frequency(CONFIG_LORA_FREQUENCY);
    phy_set_spreading_factor(CONFIG_LORA_SPREADING_FACTOR);
    phy_set_bandwidth(CONFIG_LORA_BANDWIDTH);
    phy_set_coding_rate(CONFIG_LORA_CODING_RATE);
    phy_set_preamble_length(CONFIG_LORA_PREAMBLE_LENGTH);
    phy_set_sync_word(CONFIG_LORA_SYNC_WORD);

#if CONFIG_LORA_ENABLE_CRC
    phy_enable_crc();
#else
    phy_disable_crc();
#endif

    ESP_LOGI(TAG, "PHY Kconfig Applied:");
    ESP_LOGI(TAG, "  Freq: %u", CONFIG_LORA_FREQUENCY);
    ESP_LOGI(TAG, "  SF: %d", CONFIG_LORA_SPREADING_FACTOR);
    ESP_LOGI(TAG, "  BW: %d", CONFIG_LORA_BANDWIDTH);
    ESP_LOGI(TAG, "  CR: %d", CONFIG_LORA_CODING_RATE);
}


/**
 * Perform hardware initialization.
 */
int 
phy_init(void)
{
   esp_err_t ret;

   /*
    * Configure CPU hardware to communicate with the radio chip
    */
   gpio_reset_pin(CONFIG_RST_GPIO);
   gpio_set_direction(CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);
   gpio_reset_pin(CONFIG_CS_GPIO);
   gpio_set_direction(CONFIG_CS_GPIO, GPIO_MODE_OUTPUT);
   gpio_set_level(CONFIG_CS_GPIO, 1);

   spi_bus_config_t bus = {
      .miso_io_num = CONFIG_MISO_GPIO,
      .mosi_io_num = CONFIG_MOSI_GPIO,
      .sclk_io_num = CONFIG_SCK_GPIO,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0
   };
           
   //ret = spi_bus_initialize(VSPI_HOST, &bus, 0);
   ret = spi_bus_initialize(HOST_ID, &bus, SPI_DMA_CH_AUTO);
   assert(ret == ESP_OK);

   spi_device_interface_config_t dev = {
      .clock_speed_hz = 9000000,
      .mode = 0,
      .spics_io_num = CONFIG_CS_GPIO,
      .queue_size = 7,
      .flags = 0,
      .pre_cb = NULL
   };
   //ret = spi_bus_add_device(VSPI_HOST, &dev, &_spi);
   ret = spi_bus_add_device(HOST_ID, &dev, &_spi);
   assert(ret == ESP_OK);

   phy_event_queue = xQueueCreate(8, sizeof(phy_event_t));
   phy_rx_queue = xQueueCreate(8, sizeof(phy_bit_t));
   phy_cad_done_sem = xSemaphoreCreateBinary();
   if (phy_cad_done_sem == NULL) return 0;


    gpio_reset_pin(CONFIG_DIO0_GPIO);
    gpio_set_direction(CONFIG_DIO0_GPIO, GPIO_MODE_INPUT);
    gpio_set_intr_type(CONFIG_DIO0_GPIO, GPIO_INTR_POSEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONFIG_DIO0_GPIO, phy_dio0_isr, NULL);

    xTaskCreate(phy_task, "phy_task", 4096, NULL, 5, NULL);


   /*
    * Perform hardware reset.
    */
   phy_reset();

   /*
    * Check version.
    */
   uint8_t version;
   uint8_t i = 0;
   while(i++ < TIMEOUT_RESET) {
      version = phy_read_reg(REG_VERSION);
      ESP_LOGD(TAG, "version=0x%02x", version);
      if(version == 0x12) break;
      vTaskDelay(2);
   }
   ESP_LOGD(TAG, "i=%d, TIMEOUT_RESET=%d", i, TIMEOUT_RESET);
   if (i == TIMEOUT_RESET + 1) return 0; // Illegal version
   //assert(i < TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1

   /*
    * Default configuration.
    */
   phy_sleep();
   phy_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
   phy_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
   phy_write_reg(REG_LNA, phy_read_reg(REG_LNA) | 0x03);
   phy_write_reg(REG_MODEM_CONFIG_3, 0x04);
   phy_set_tx_power(5);

   phy_set_mode(MODE_STDBY);

   /* ------------------------
   Apply Kconfig Radio Settings
   ------------------------ */
   phy_apply_kconfig_defaults();

   return 1;
}

/**
 * Send bits.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void 
phy_send_bit(uint8_t *buf, int size)
{

   // map DIO0 to TX_DONE
   phy_set_dio_mapping(0, 1);
   
   phy_write_reg(REG_FIFO_ADDR_PTR, 0);

#if BUFFER_IO
   phy_write_reg_buffer(REG_FIFO, buf, size);
#else
   for(int i=0; i<size; i++)
      phy_write_reg(REG_FIFO, *buf++);
#endif

   phy_write_reg(REG_PAYLOAD_LENGTH, size);

   // Start transmission (non-blocking)
   phy_set_mode(MODE_TX);
}


/**
 * Read a received bits from the FIFO.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no bits available).
 */
int 
phy_hw_receive_bit(uint8_t *buf, int size)
{
    int len;

    if (_implicit)
        len = phy_read_reg(REG_PAYLOAD_LENGTH);
    else
        len = phy_read_reg(REG_RX_NB_BYTES);

    phy_write_reg(REG_FIFO_ADDR_PTR,
                   phy_read_reg(REG_FIFO_RX_CURRENT_ADDR));

    if(len > size)
        len = size;

#if BUFFER_IO
    phy_read_reg_buffer(REG_FIFO, buf, len);
#else
    for(int i=0; i<len; i++)
        *buf++ = phy_read_reg(REG_FIFO);
#endif

    return len;
}


/**
 * Returns RegIrqFlags.
 */
int
phy_get_irq(void)
{
   return (phy_read_reg(REG_IRQ_FLAGS));
}

/**
 * Channel Activity Detection (CAD).
 * Puts radio in CAD mode, waits for CadDone (or timeout), then returns
 * true if channel is clear (no LoRa signal detected), false if busy or timeout.
 */
bool
phy_cad_is_channel_clear(void)
{
    phy_standby();
    phy_cad_detected_result = false;
    phy_set_mode(MODE_CAD);

    /* Wait for CAD completion (phy_task will signal and clear flags) */
    const TickType_t cad_timeout_ms = 100;
    if (xSemaphoreTake(phy_cad_done_sem, pdMS_TO_TICKS(cad_timeout_ms)) != pdTRUE) {
        phy_standby();
        ESP_LOGW(TAG, "CAD timeout");
        return false;
    }

    return !phy_cad_detected_result;
}

static void IRAM_ATTR phy_dio0_isr(void *arg)
{
    phy_event_t evt;
    evt.type = PHY_EVENT_IRQ;
    xQueueSendFromISR(phy_event_queue, &evt, NULL);
}




/**
 * Return last bits's RSSI.
 */
int 
phy_bit_rssi(void)
{
   return (phy_read_reg(REG_BIT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}


/**
 * Return last bit's SNR (signal to noise ratio).
 */
float 
phy_bit_snr(void)
{
   return ((int8_t)phy_read_reg(REG_BIT_SNR_VALUE)) * 0.25;
}

lora_state_t phy_get_state(void)
{
    return _state;
}

void 
phy_dump_registers(void)
{
   int i;
   printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
   for(i=0; i<0x40; i++) {
      printf("%02X ", phy_read_reg(i));
      if((i & 0x0f) == 0x0f) printf("\n");
   }
   printf("\n");
}

static void phy_task(void *arg)
{
    phy_event_t evt;

    while (1) {

        if (xQueueReceive(phy_event_queue, &evt, portMAX_DELAY)) {

            // Read IRQ flags ONCE
            int irq = phy_read_reg(REG_IRQ_FLAGS);

            // --------------------
            // CAD DONE (must check before TX DONE; same IRQ bit)
            // --------------------
            if ((irq & IRQ_CAD_DONE_MASK) && (_state == LORA_STATE_CAD)) {
               phy_cad_detected_result = (irq & IRQ_CAD_DETECTED_MASK) ? true : false;
               phy_write_reg(REG_IRQ_FLAGS, irq);
               phy_standby();
               xSemaphoreGive(phy_cad_done_sem);
               continue;
            }

            // --------------------
            // TX DONE
            // --------------------
            if (irq & IRQ_TX_DONE_MASK) {
               
               // Map DIO0 back to RX_DONE
               phy_set_dio_mapping(0, 0);

               // notify upper layer
               if (phy_event_callback) {
                  phy_event_callback(PHY_EVENT_TX_DONE);
               }
            }

            // --------------------
            // RX DONE
            // --------------------
            if (irq & IRQ_RX_DONE_MASK) {

               // Keep current behavior (non-breaking)
               phy_bit_t bits;

               int len = phy_hw_receive_bit(bits.data, sizeof(bits.data));

               if (len > 0) {
                  bits.len = len;
                  xQueueSend(phy_rx_queue, &bits, 0);
               }

               // notify upper layer
               if (phy_event_callback) {
                  phy_event_callback(PHY_EVENT_RX_DONE);
               }
               
            }

            // CRC error is only relevant if RX_DONE is not set, otherwise the bits are discarded and upper layer is not notified at all.
            if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK) {
               if (phy_event_callback) {
                  phy_event_callback(PHY_EVENT_CRC_ERROR);
               }
            }

            // --------------------
            // CLEAR IRQ FLAGS
            // --------------------
            phy_write_reg(REG_IRQ_FLAGS, irq);
        }
    }
}


int phy_get_bit(uint8_t *buf, int max_len)
{
    phy_bit_t bits;

    if (xQueueReceive(phy_rx_queue, &bits, 0)) {

        int len = bits.len;
        if (len > max_len) len = max_len;

        memcpy(buf, bits.data, len);
        return len;
    }

    return 0;
}

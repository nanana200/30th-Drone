#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define LORA_MAX_PACKET_SIZE 128U

typedef enum
{
  LORA_TX_IDLE = 0,
  LORA_TX_BUSY,
  LORA_TX_DONE,
  LORA_TX_TIMEOUT,
  LORA_TX_ERROR
} lora_tx_status_t;

typedef enum
{
  LORA_RX_NONE = 0,
  LORA_RX_PACKET,
  LORA_RX_CRC_ERROR,
  LORA_RX_INVALID_LENGTH,
  LORA_RX_ERROR
} lora_rx_status_t;

typedef struct
{
  uint32_t spi_polarity;
  uint32_t spi_phase;
  uint32_t spi_first_bit;
  uint32_t spi_data_size;
  uint32_t spi_prescaler;
  uint8_t version_first;
  uint8_t version_last;
  uint8_t version_min;
  uint8_t version_max;
  uint8_t version_ok_count;
  uint8_t version_read_failures;
  uint8_t op_mode;
  uint8_t modem_config_1;
  uint8_t modem_config_2;
  uint8_t modem_config_3;
  uint8_t sync_word;
} lora_spi_debug_t;

typedef struct
{
  uint8_t irq_flags;
  uint8_t rx_nb_bytes;
  uint8_t fifo_rx_current_addr;
  uint8_t fifo_addr_ptr_before;
  uint8_t fifo_addr_ptr_after;
  uint8_t version_before_fifo;
  uint8_t version_after_fifo;
  uint8_t register_read_error_mask;
} lora_rx_debug_t;

HAL_StatusTypeDef lora_init(void);
void lora_rx_set(void);
lora_rx_status_t lora_receive_packet(uint8_t *buffer, uint8_t capacity,
                                     uint8_t *length, int16_t *rssi_dbm,
                                     int16_t *snr_x100);
HAL_StatusTypeDef lora_start_tx(const uint8_t *buffer, uint8_t length);
lora_tx_status_t lora_tx_process(void);
void lora_dio0_callback(uint16_t gpio_pin);
HAL_StatusTypeDef lora_spi_debug_snapshot(lora_spi_debug_t *debug);
void lora_get_last_rx_debug(lora_rx_debug_t *debug);

#endif /* INC_LORA_H_ */

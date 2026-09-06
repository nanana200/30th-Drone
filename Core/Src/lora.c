#include "lora.h"

#include <string.h>

#define LORA_REG_FIFO                 0x00U
#define LORA_REG_OP_MODE              0x01U
#define LORA_REG_FRF_MSB              0x06U
#define LORA_REG_PA_CONFIG            0x09U
#define LORA_REG_OCP                  0x0BU
#define LORA_REG_FIFO_ADDR_PTR        0x0DU
#define LORA_REG_FIFO_TX_BASE_ADDR    0x0EU
#define LORA_REG_FIFO_RX_BASE_ADDR    0x0FU
#define LORA_REG_FIFO_RX_CURRENT_ADDR 0x10U
#define LORA_REG_IRQ_FLAGS_MASK       0x11U
#define LORA_REG_IRQ_FLAGS            0x12U
#define LORA_REG_RX_NB_BYTES          0x13U
#define LORA_REG_PKT_SNR_VALUE        0x19U
#define LORA_REG_PKT_RSSI_VALUE       0x1AU
#define LORA_REG_MODEM_CONFIG_1       0x1DU
#define LORA_REG_MODEM_CONFIG_2       0x1EU
#define LORA_REG_PREAMBLE_MSB         0x20U
#define LORA_REG_PREAMBLE_LSB         0x21U
#define LORA_REG_PAYLOAD_LENGTH       0x22U
#define LORA_REG_MODEM_CONFIG_3       0x26U
#define LORA_REG_SYNC_WORD            0x39U
#define LORA_REG_DIO_MAPPING_1        0x40U
#define LORA_REG_VERSION              0x42U

#define LORA_MODE_SLEEP               0x80U
#define LORA_MODE_STANDBY             0x81U
#define LORA_MODE_TX                  0x83U
#define LORA_MODE_RX_CONTINUOUS       0x85U

#define LORA_IRQ_TX_DONE              0x08U
#define LORA_IRQ_PAYLOAD_CRC_ERROR    0x20U
#define LORA_IRQ_RX_DONE              0x40U
#define LORA_EXPECTED_VERSION         0x12U
#define LORA_TX_TIMEOUT_MS            300U

extern SPI_HandleTypeDef hspi4;

static volatile uint8_t lora_dio0_event;
static uint8_t lora_tx_active;
static uint32_t lora_tx_started_ms;
static lora_rx_debug_t lora_last_rx_debug;

static HAL_StatusTypeDef lora_write(uint8_t address, uint8_t value)
{
  uint8_t tx[2] = {(uint8_t)(address | 0x80U), value};
  uint8_t rx[2] = {0U, 0U};
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&hspi4, tx, rx, sizeof(tx), 100U);
  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
  return status;
}

static HAL_StatusTypeDef lora_read(uint8_t address, uint8_t *value)
{
  uint8_t tx[2] = {(uint8_t)(address & 0x7FU), 0U};
  uint8_t rx[2] = {0U, 0U};
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&hspi4, tx, rx, sizeof(tx), 100U);
  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
  *value = rx[1];
  return status;
}

static HAL_StatusTypeDef lora_write_burst(uint8_t address, const uint8_t *data, uint8_t length)
{
  static uint8_t tx[LORA_MAX_PACKET_SIZE + 1U];
  static uint8_t rx[LORA_MAX_PACKET_SIZE + 1U];
  HAL_StatusTypeDef status;

  if ((data == NULL) || (length == 0U) || (length > LORA_MAX_PACKET_SIZE))
  {
    return HAL_ERROR;
  }

  tx[0] = (uint8_t)(address | 0x80U);
  memcpy(&tx[1], data, length);

  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&hspi4, tx, rx,
                                   (uint16_t)length + 1U, 100U);
  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
  return status;
}

static HAL_StatusTypeDef lora_read_burst(uint8_t address, uint8_t *data, uint8_t length)
{
  static uint8_t tx[LORA_MAX_PACKET_SIZE + 1U];
  static uint8_t rx[LORA_MAX_PACKET_SIZE + 1U];
  HAL_StatusTypeDef status;

  if ((data == NULL) || (length == 0U) || (length > LORA_MAX_PACKET_SIZE))
  {
    return HAL_ERROR;
  }

  tx[0] = (uint8_t)(address & 0x7FU);
  memset(&tx[1], 0, length);

  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&hspi4, tx, rx,
                                   (uint16_t)length + 1U, 100U);
  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    /* rx[0] is the byte clocked in while the register address is sent. */
    memcpy(data, &rx[1], length);
  }
  return status;
}

HAL_StatusTypeDef lora_init(void)
{
  static const uint8_t frequency[3] = {0xE6U, 0x86U, 0x66U}; /* 922.1 MHz */
  uint8_t version = 0U;

  HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI4_RESET_GPIO_Port, SPI4_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(SPI4_RESET_GPIO_Port, SPI4_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(10U);

  if ((lora_read(LORA_REG_VERSION, &version) != HAL_OK) || (version != LORA_EXPECTED_VERSION))
  {
    return HAL_ERROR;
  }

  if ((lora_write(LORA_REG_OP_MODE, LORA_MODE_SLEEP) != HAL_OK) ||
      (lora_write_burst(LORA_REG_FRF_MSB, frequency, sizeof(frequency)) != HAL_OK) ||
      (lora_write(LORA_REG_PA_CONFIG, 0x8FU) != HAL_OK) ||
      (lora_write(LORA_REG_OCP, 0x31U) != HAL_OK) ||
      (lora_write(LORA_REG_MODEM_CONFIG_1, 0x72U) != HAL_OK) ||
      (lora_write(LORA_REG_MODEM_CONFIG_2, 0x74U) != HAL_OK) ||
      (lora_write(LORA_REG_MODEM_CONFIG_3, 0x04U) != HAL_OK) ||
      (lora_write(LORA_REG_PREAMBLE_MSB, 0x00U) != HAL_OK) ||
      (lora_write(LORA_REG_PREAMBLE_LSB, 0x08U) != HAL_OK) ||
      (lora_write(LORA_REG_SYNC_WORD, 0x12U) != HAL_OK) ||
      (lora_write(LORA_REG_FIFO_TX_BASE_ADDR, 0x80U) != HAL_OK) ||
      (lora_write(LORA_REG_FIFO_RX_BASE_ADDR, 0x00U) != HAL_OK) ||
      (lora_write(LORA_REG_IRQ_FLAGS_MASK, 0x00U) != HAL_OK) ||
      (lora_write(LORA_REG_IRQ_FLAGS, 0xFFU) != HAL_OK))
  {
    return HAL_ERROR;
  }

  lora_tx_active = 0U;
  lora_dio0_event = 0U;
  lora_rx_set();
  return HAL_OK;
}

HAL_StatusTypeDef lora_spi_debug_snapshot(lora_spi_debug_t *debug)
{
  uint8_t sample = 0U;
  uint8_t index;

  if (debug == NULL)
  {
    return HAL_ERROR;
  }

  memset(debug, 0, sizeof(*debug));
  debug->spi_polarity = hspi4.Init.CLKPolarity;
  debug->spi_phase = hspi4.Init.CLKPhase;
  debug->spi_first_bit = hspi4.Init.FirstBit;
  debug->spi_data_size = hspi4.Init.DataSize;
  debug->spi_prescaler = hspi4.Init.BaudRatePrescaler;
  debug->version_min = 0xFFU;

  for (index = 0U; index < 16U; ++index)
  {
    if (lora_read(LORA_REG_VERSION, &sample) != HAL_OK)
    {
      debug->version_read_failures++;
      continue;
    }
    if (index == 0U)
    {
      debug->version_first = sample;
    }
    debug->version_last = sample;
    if (sample < debug->version_min)
    {
      debug->version_min = sample;
    }
    if (sample > debug->version_max)
    {
      debug->version_max = sample;
    }
    if (sample == LORA_EXPECTED_VERSION)
    {
      debug->version_ok_count++;
    }
  }

  if ((lora_read(LORA_REG_OP_MODE, &debug->op_mode) != HAL_OK) ||
      (lora_read(LORA_REG_MODEM_CONFIG_1, &debug->modem_config_1) != HAL_OK) ||
      (lora_read(LORA_REG_MODEM_CONFIG_2, &debug->modem_config_2) != HAL_OK) ||
      (lora_read(LORA_REG_MODEM_CONFIG_3, &debug->modem_config_3) != HAL_OK) ||
      (lora_read(LORA_REG_SYNC_WORD, &debug->sync_word) != HAL_OK))
  {
    return HAL_ERROR;
  }

  return ((debug->version_ok_count == 16U) &&
          (debug->version_read_failures == 0U)) ? HAL_OK : HAL_ERROR;
}

void lora_get_last_rx_debug(lora_rx_debug_t *debug)
{
  if (debug != NULL)
  {
    *debug = lora_last_rx_debug;
  }
}

void lora_rx_set(void)
{
  (void)lora_write(LORA_REG_OP_MODE, LORA_MODE_STANDBY);
  (void)lora_write(LORA_REG_IRQ_FLAGS, 0xFFU);
  (void)lora_write(LORA_REG_FIFO_ADDR_PTR, 0x00U);
  (void)lora_write(LORA_REG_DIO_MAPPING_1, 0x00U); /* DIO0 = RxDone */
  lora_dio0_event = 0U;
  lora_tx_active = 0U;
  (void)lora_write(LORA_REG_OP_MODE, LORA_MODE_RX_CONTINUOUS);
}

lora_rx_status_t lora_receive_packet(uint8_t *buffer, uint8_t capacity,
                                     uint8_t *length, int16_t *rssi_dbm,
                                     int16_t *snr_x100)
{
  uint8_t irq = 0U;
  uint8_t received = 0U;
  uint8_t fifo_address = 0U;
  uint8_t raw_rssi = 0U;
  uint8_t raw_snr = 0U;

  if ((buffer == NULL) || (length == NULL) ||
      (rssi_dbm == NULL) || (snr_x100 == NULL))
  {
    return LORA_RX_ERROR;
  }

  *length = 0U;
  *rssi_dbm = 0;
  *snr_x100 = 0;

  if ((lora_dio0_event == 0U) &&
      (HAL_GPIO_ReadPin(SPI4_INT_GPIO_Port, SPI4_INT_Pin) == GPIO_PIN_RESET))
  {
    return LORA_RX_NONE;
  }

  lora_dio0_event = 0U;
  memset(&lora_last_rx_debug, 0, sizeof(lora_last_rx_debug));
  if (lora_read(LORA_REG_IRQ_FLAGS, &irq) != HAL_OK)
  {
    lora_last_rx_debug.register_read_error_mask |= 0x01U;
    lora_rx_set();
    return LORA_RX_ERROR;
  }
  lora_last_rx_debug.irq_flags = irq;

  if ((irq & LORA_IRQ_RX_DONE) == 0U)
  {
    (void)lora_write(LORA_REG_IRQ_FLAGS, 0xFFU);
    lora_rx_set();
    return LORA_RX_NONE;
  }

  if ((lora_read(LORA_REG_PKT_RSSI_VALUE, &raw_rssi) != HAL_OK) ||
      (lora_read(LORA_REG_PKT_SNR_VALUE, &raw_snr) != HAL_OK))
  {
    lora_rx_set();
    return LORA_RX_ERROR;
  }
  *rssi_dbm = (int16_t)(-157 + (int16_t)raw_rssi);
  *snr_x100 = (int16_t)((int16_t)(int8_t)raw_snr * 25);

  if ((irq & LORA_IRQ_PAYLOAD_CRC_ERROR) != 0U)
  {
    (void)lora_write(LORA_REG_IRQ_FLAGS, 0xFFU);
    lora_rx_set();
    return LORA_RX_CRC_ERROR;
  }

  (void)lora_write(LORA_REG_OP_MODE, LORA_MODE_STANDBY);
  if (lora_read(LORA_REG_VERSION, &lora_last_rx_debug.version_before_fifo) != HAL_OK)
  {
    lora_last_rx_debug.register_read_error_mask |= 0x02U;
  }
  if ((lora_read(LORA_REG_RX_NB_BYTES, &received) != HAL_OK) ||
      (lora_read(LORA_REG_FIFO_RX_CURRENT_ADDR, &fifo_address) != HAL_OK))
  {
    lora_last_rx_debug.register_read_error_mask |= 0x04U;
    lora_rx_set();
    return LORA_RX_ERROR;
  }
  lora_last_rx_debug.rx_nb_bytes = received;
  lora_last_rx_debug.fifo_rx_current_addr = fifo_address;

  if (lora_read(LORA_REG_FIFO_ADDR_PTR,
                &lora_last_rx_debug.fifo_addr_ptr_before) != HAL_OK)
  {
    lora_last_rx_debug.register_read_error_mask |= 0x08U;
  }

  if ((received == 0U) || (received > capacity))
  {
    lora_rx_set();
    return LORA_RX_INVALID_LENGTH;
  }

  if (lora_write(LORA_REG_FIFO_ADDR_PTR, fifo_address) != HAL_OK)
  {
    lora_last_rx_debug.register_read_error_mask |= 0x10U;
  }
  if (lora_read(LORA_REG_FIFO_ADDR_PTR,
                &lora_last_rx_debug.fifo_addr_ptr_after) != HAL_OK)
  {
    lora_last_rx_debug.register_read_error_mask |= 0x20U;
  }
  if (lora_read_burst(LORA_REG_FIFO, buffer, received) != HAL_OK)
  {
    lora_last_rx_debug.register_read_error_mask |= 0x40U;
    lora_rx_set();
    return LORA_RX_ERROR;
  }
  if (lora_read(LORA_REG_VERSION, &lora_last_rx_debug.version_after_fifo) != HAL_OK)
  {
    lora_last_rx_debug.register_read_error_mask |= 0x80U;
  }

  (void)lora_write(LORA_REG_IRQ_FLAGS, 0xFFU);
  *length = received;
  return LORA_RX_PACKET;
}

HAL_StatusTypeDef lora_start_tx(const uint8_t *buffer, uint8_t length)
{
  if ((buffer == NULL) || (length == 0U) || (length > LORA_MAX_PACKET_SIZE) ||
      (lora_tx_active != 0U))
  {
    return HAL_ERROR;
  }

  if ((lora_write(LORA_REG_OP_MODE, LORA_MODE_STANDBY) != HAL_OK) ||
      (lora_write(LORA_REG_IRQ_FLAGS, 0xFFU) != HAL_OK) ||
      (lora_write(LORA_REG_DIO_MAPPING_1, 0x40U) != HAL_OK) || /* DIO0 = TxDone */
      (lora_write(LORA_REG_FIFO_ADDR_PTR, 0x80U) != HAL_OK) ||
      (lora_write(LORA_REG_PAYLOAD_LENGTH, length) != HAL_OK) ||
      (lora_write_burst(LORA_REG_FIFO, buffer, length) != HAL_OK) ||
      (lora_write(LORA_REG_OP_MODE, LORA_MODE_TX) != HAL_OK))
  {
    lora_rx_set();
    return HAL_ERROR;
  }

  lora_dio0_event = 0U;
  lora_tx_started_ms = HAL_GetTick();
  lora_tx_active = 1U;
  return HAL_OK;
}

lora_tx_status_t lora_tx_process(void)
{
  uint8_t irq = 0U;

  if (lora_tx_active == 0U)
  {
    return LORA_TX_IDLE;
  }

  if ((lora_dio0_event != 0U) ||
      (HAL_GPIO_ReadPin(SPI4_INT_GPIO_Port, SPI4_INT_Pin) == GPIO_PIN_SET))
  {
    lora_dio0_event = 0U;
    if (lora_read(LORA_REG_IRQ_FLAGS, &irq) != HAL_OK)
    {
      lora_rx_set();
      return LORA_TX_ERROR;
    }
    if ((irq & LORA_IRQ_TX_DONE) != 0U)
    {
      lora_rx_set();
      return LORA_TX_DONE;
    }
  }

  if ((HAL_GetTick() - lora_tx_started_ms) > LORA_TX_TIMEOUT_MS)
  {
    lora_rx_set();
    return LORA_TX_TIMEOUT;
  }
  return LORA_TX_BUSY;
}

void lora_dio0_callback(uint16_t gpio_pin)
{
  if (gpio_pin == SPI4_INT_Pin)
  {
    lora_dio0_event = 1U;
  }
}

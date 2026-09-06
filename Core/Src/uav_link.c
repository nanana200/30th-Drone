#include "uav_link.h"

#include "debug.h"
#include "lora.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#define PROTOCOL_MAGIC          0xA4U
#define NODE_GCS                0xFFU
#define NODE_UAV                0xFDU
#define MSG_POLL                0x01U
#define MSG_WAYPOINT            0x02U
#define MSG_ACK                 0x03U
#define MSG_UAV_GPS             0x10U
#define HEADER_SIZE             7U
#define MAX_WAYPOINTS           15U
#define ACK_OK                  0x00U
#define ACK_BAD_PAYLOAD         0x01U
#define GNSS_FRESH_MS           1500U
#define POLL_RESPONSE_DELAY_MS  10U
#define HEX_DUMP_BYTES          32U
#define WAYPOINT_LOG_PERIOD_MS  20U

typedef struct
{
  int32_t latitude_deg_1e7;
  int32_t longitude_deg_1e7;
} waypoint_t;

typedef enum
{
  LINK_RX = 0,
  LINK_TX_DELAY,
  LINK_TX
} link_state_t;

static waypoint_t waypoints[MAX_WAYPOINTS];
static uint8_t waypoint_count;
static uint8_t waypoint_log_index;
static uint8_t waypoint_log_pending;
static uint32_t waypoint_log_ms;
static int32_t latest_latitude;
static int32_t latest_longitude;
static uint32_t latest_gnss_ms;
static uint8_t latest_fix_valid;
static uint8_t gnss_received;
static link_state_t link_state;
static uint8_t tx_message_type;
static uint16_t tx_sequence;
static uint8_t pending_tx_packet[HEADER_SIZE + 9U];
static uint8_t pending_tx_length;
static uint32_t pending_tx_at_ms;
static uint32_t last_rx_debug_log_ms;

static void log_rx_debug_throttled(const char *reason)
{
  lora_rx_debug_t debug;
  uint32_t now = HAL_GetTick();

  if ((last_rx_debug_log_ms != 0U) &&
      ((now - last_rx_debug_log_ms) < 1000U))
  {
    return;
  }
  last_rx_debug_log_ms = now;
  lora_get_last_rx_debug(&debug);
  (void)uart1_printf("[LORA DIAG RX] reason=%s IRQ=0x%02X RXDONE=%u CRCERR=%u RXBYTES=%u\r\n",
                     reason, debug.irq_flags,
                     ((debug.irq_flags & 0x40U) != 0U) ? 1U : 0U,
                     ((debug.irq_flags & 0x20U) != 0U) ? 1U : 0U,
                     debug.rx_nb_bytes);
  (void)uart1_printf("[LORA DIAG RX] CUR=0x%02X PTR_BEFORE=0x%02X PTR_AFTER=0x%02X ERRMASK=0x%02X\r\n",
                     debug.fifo_rx_current_addr, debug.fifo_addr_ptr_before,
                     debug.fifo_addr_ptr_after, debug.register_read_error_mask);
  (void)uart1_printf("[LORA DIAG RX] VERSION_BEFORE=0x%02X VERSION_AFTER=0x%02X expected=0x12\r\n",
                     debug.version_before_fifo, debug.version_after_fifo);
}

static uint16_t read_u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int32_t read_i32_le(const uint8_t *data)
{
  uint32_t value = (uint32_t)data[0] |
                   ((uint32_t)data[1] << 8) |
                   ((uint32_t)data[2] << 16) |
                   ((uint32_t)data[3] << 24);
  return (int32_t)value;
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
}

static void write_i32_le(uint8_t *data, int32_t signed_value)
{
  uint32_t value = (uint32_t)signed_value;
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
}

static bool coordinate_valid(int32_t latitude, int32_t longitude)
{
  return (latitude >= -900000000L) && (latitude <= 900000000L) &&
         (longitude >= -1800000000L) && (longitude <= 1800000000L);
}

static uint32_t coordinate_magnitude(int32_t coordinate)
{
  return (coordinate < 0) ? (uint32_t)(-(int64_t)coordinate) : (uint32_t)coordinate;
}

static void log_coordinates(const char *prefix, int32_t latitude, int32_t longitude)
{
  uint32_t lat = coordinate_magnitude(latitude);
  uint32_t lon = coordinate_magnitude(longitude);

  (void)uart1_printf("%s LAT=%s%lu.%07lu LON=%s%lu.%07lu\r\n",
                     prefix,
                     (latitude < 0) ? "-" : "", (unsigned long)(lat / 10000000U),
                     (unsigned long)(lat % 10000000U),
                     (longitude < 0) ? "-" : "", (unsigned long)(lon / 10000000U),
                     (unsigned long)(lon % 10000000U));
}

static const char *message_name(uint8_t type)
{
  if (type == MSG_POLL)
  {
    return "POLL";
  }
  if (type == MSG_WAYPOINT)
  {
    return "WAYPOINT";
  }
  if (type == MSG_ACK)
  {
    return "ACK";
  }
  if (type == MSG_UAV_GPS)
  {
    return "UAV_GPS";
  }
  return "UNKNOWN";
}

static void log_hex_dump(const uint8_t *packet, uint8_t packet_length)
{
  char line[112];
  static const char hex[] = "0123456789ABCDEF";
  uint8_t shown = (packet_length > HEX_DUMP_BYTES) ? HEX_DUMP_BYTES : packet_length;
  uint16_t offset = 0U;
  uint8_t index;

  line[offset++] = '[';
  line[offset++] = 'H';
  line[offset++] = 'E';
  line[offset++] = 'X';
  line[offset++] = ']';
  line[offset++] = ' ';
  for (index = 0U; index < shown; ++index)
  {
    line[offset++] = hex[packet[index] >> 4];
    line[offset++] = hex[packet[index] & 0x0FU];
    line[offset++] = ' ';
  }
  if (shown < packet_length)
  {
    line[offset++] = '.';
    line[offset++] = '.';
    line[offset++] = '.';
  }
  line[offset++] = '\r';
  line[offset++] = '\n';
  line[offset] = '\0';
  (void)uart1_printf("%s", line);
}

static bool start_response(uint8_t type, uint16_t sequence,
                           const uint8_t *payload, uint8_t payload_length,
                           uint32_t delay_ms)
{
  uint8_t index;

  if (payload_length > 9U)
  {
    return false;
  }

  pending_tx_packet[0] = PROTOCOL_MAGIC;
  pending_tx_packet[1] = NODE_UAV;
  pending_tx_packet[2] = NODE_GCS;
  pending_tx_packet[3] = type;
  write_u16_le(&pending_tx_packet[4], sequence);
  pending_tx_packet[6] = payload_length;
  for (index = 0U; index < payload_length; ++index)
  {
    pending_tx_packet[HEADER_SIZE + index] = payload[index];
  }

  pending_tx_length = (uint8_t)(HEADER_SIZE + payload_length);
  tx_message_type = type;
  tx_sequence = sequence;
  if (delay_ms != 0U)
  {
    pending_tx_at_ms = HAL_GetTick() + delay_ms;
    link_state = LINK_TX_DELAY;
    return true;
  }

  if (lora_start_tx(pending_tx_packet, pending_tx_length) != HAL_OK)
  {
    (void)uart1_printf("[LORA] TX START ERROR type=0x%02X seq=%u\r\n", type, sequence);
    return false;
  }

  link_state = LINK_TX;
  return true;
}

static void respond_to_poll(uint16_t sequence)
{
  uint8_t payload[9];
  uint8_t valid = latest_fix_valid;
  int32_t latitude = latest_latitude;
  int32_t longitude = latest_longitude;

  if ((gnss_received == 0U) || ((HAL_GetTick() - latest_gnss_ms) > GNSS_FRESH_MS))
  {
    valid = 0U;
  }
  if (valid == 0U)
  {
    latitude = 0;
    longitude = 0;
  }

  write_i32_le(&payload[0], latitude);
  write_i32_le(&payload[4], longitude);
  payload[8] = valid;

  (void)uart1_printf("[LORA TX] TYPE=UAV_GPS(0x10) SEQ=%u VALID=%u\r\n",
                     sequence, valid);
  (void)uart1_printf("[LORA TX] scheduled after %u ms RX/TX guard\r\n",
                     POLL_RESPONSE_DELAY_MS);
  log_coordinates("[LORA TX]", latitude, longitude);
  if (!start_response(MSG_UAV_GPS, sequence, payload, sizeof(payload),
                      POLL_RESPONSE_DELAY_MS))
  {
    lora_rx_set();
  }
}

static void handle_waypoints(uint16_t sequence, const uint8_t *payload, uint8_t length)
{
  uint8_t count = (length > 0U) ? payload[0] : 0U;
  uint8_t status = ACK_OK;
  uint8_t index;
  uint8_t ack[2] = {MSG_WAYPOINT, ACK_OK};

  if ((count == 0U) || (count > MAX_WAYPOINTS) ||
      (length != (uint8_t)(1U + (count * 8U))))
  {
    status = ACK_BAD_PAYLOAD;
  }
  else
  {
    for (index = 0U; index < count; ++index)
    {
      int32_t latitude = read_i32_le(&payload[1U + (index * 8U)]);
      int32_t longitude = read_i32_le(&payload[5U + (index * 8U)]);
      if (!coordinate_valid(latitude, longitude))
      {
        status = ACK_BAD_PAYLOAD;
        break;
      }
      waypoints[index].latitude_deg_1e7 = latitude;
      waypoints[index].longitude_deg_1e7 = longitude;
    }
  }

  if (status == ACK_OK)
  {
    waypoint_count = count;
    waypoint_log_index = 0U;
    waypoint_log_pending = 1U;
    waypoint_log_ms = HAL_GetTick();
    (void)uart1_printf("[UAV][LORA] WAYPOINT RX SEQ=%u COUNT=%u\r\n",
                       sequence, waypoint_count);
  }
  else
  {
    (void)uart1_printf("[LORA] INVALID LENGTH waypoint seq=%u len=%u count=%u\r\n",
                       sequence, length, count);
  }

  ack[1] = status;
  (void)uart1_printf("[LORA TX] TYPE=ACK(0x03) SEQ=%u STATUS=%u\r\n",
                     sequence, status);
  if (!start_response(MSG_ACK, sequence, ack, sizeof(ack), 0U))
  {
    lora_rx_set();
  }
}

static void handle_packet(const uint8_t *packet, uint8_t packet_length,
                          int16_t rssi_dbm, int16_t snr_x100)
{
  uint8_t payload_length;
  uint16_t sequence;
  uint16_t snr_magnitude = (snr_x100 < 0) ? (uint16_t)(-snr_x100) : (uint16_t)snr_x100;

  /* A1/A2 RTCM and A3 grant are legacy UGV-only packets on the shared channel. */
  if ((packet_length != 0U) && (packet[0] >= 0xA1U) && (packet[0] <= 0xA3U))
  {
    lora_rx_set();
    return;
  }

  log_hex_dump(packet, packet_length);
  if (packet_length < HEADER_SIZE)
  {
    (void)uart1_printf("[LORA] INVALID LENGTH packet=%u header=%u\r\n",
                       packet_length, HEADER_SIZE);
    lora_rx_set();
    return;
  }
  if (packet[0] != PROTOCOL_MAGIC)
  {
    (void)uart1_printf("[LORA] INVALID MAGIC value=0x%02X\r\n", packet[0]);
    log_rx_debug_throttled("INVALID_MAGIC");
    lora_rx_set();
    return;
  }

  payload_length = packet[6];
  sequence = read_u16_le(&packet[4]);
  (void)uart1_printf("[LORA RX] SRC=0x%02X DST=0x%02X TYPE=%s(0x%02X) SEQ=%u\r\n",
                     packet[1], packet[2], message_name(packet[3]), packet[3], sequence);
  (void)uart1_printf("[LORA RX] RSSI=%d dBm SNR=%s%u.%02u dB LEN=%u\r\n",
                     rssi_dbm, (snr_x100 < 0) ? "-" : "",
                     snr_magnitude / 100U, snr_magnitude % 100U, packet_length);

  if (packet_length != (uint8_t)(HEADER_SIZE + payload_length))
  {
    (void)uart1_printf("[LORA] INVALID LENGTH packet=%u payload=%u\r\n",
                       packet_length, payload_length);
    lora_rx_set();
    return;
  }
  if (packet[2] != NODE_UAV)
  {
    (void)uart1_printf("[LORA] WRONG DST 0x%02X (UAV=0xFD)\r\n", packet[2]);
    lora_rx_set();
    return;
  }
  if (packet[1] != NODE_GCS)
  {
    (void)uart1_printf("[LORA] WRONG SRC 0x%02X (GCS=0xFF)\r\n", packet[1]);
    lora_rx_set();
    return;
  }

  if (packet[3] == MSG_POLL)
  {
    if (payload_length == 0U)
    {
      respond_to_poll(sequence);
    }
    else
    {
      (void)uart1_printf("[LORA] INVALID LENGTH POLL payload=%u\r\n", payload_length);
      lora_rx_set();
    }
  }
  else if (packet[3] == MSG_WAYPOINT)
  {
    handle_waypoints(sequence, &packet[HEADER_SIZE], payload_length);
  }
  else
  {
    (void)uart1_printf("[LORA] UNKNOWN TYPE 0x%02X seq=%u\r\n", packet[3], sequence);
    lora_rx_set();
  }
}

static void process_waypoint_log(void)
{
  char prefix[24];

  if ((waypoint_log_pending == 0U) ||
      ((HAL_GetTick() - waypoint_log_ms) < WAYPOINT_LOG_PERIOD_MS))
  {
    return;
  }

  waypoint_log_ms = HAL_GetTick();
  if (waypoint_log_index < waypoint_count)
  {
    (void)snprintf(prefix, sizeof(prefix), "[WP %u]", waypoint_log_index);
    log_coordinates(prefix,
                    waypoints[waypoint_log_index].latitude_deg_1e7,
                    waypoints[waypoint_log_index].longitude_deg_1e7);
    waypoint_log_index++;
  }
  if (waypoint_log_index >= waypoint_count)
  {
    waypoint_log_pending = 0U;
  }
}

HAL_StatusTypeDef uav_link_init(void)
{
  lora_spi_debug_t spi_debug;
  HAL_StatusTypeDef diagnostic_status;

  link_state = LINK_RX;
  latest_fix_valid = 0U;
  gnss_received = 0U;
  waypoint_count = 0U;
  waypoint_log_pending = 0U;
  pending_tx_length = 0U;
  last_rx_debug_log_ms = 0U;

  (void)uart1_printf("[LORA] SPI initialized: SPI4 mode0 8-bit 7.5MHz\r\n");
  if (lora_init() != HAL_OK)
  {
    (void)uart1_printf("[LORA] SX127x INIT ERROR (SPI/version)\r\n");
    return HAL_ERROR;
  }

  diagnostic_status = lora_spi_debug_snapshot(&spi_debug);
  (void)uart1_printf("[LORA DIAG BOOT] MODE=%s FIRST=%s BITS=%s PRESCALER_CODE=%lu CONFIG=%s\r\n",
                     ((spi_debug.spi_polarity == SPI_POLARITY_LOW) &&
                      (spi_debug.spi_phase == SPI_PHASE_1EDGE)) ? "MODE0" : "NOT_MODE0",
                     (spi_debug.spi_first_bit == SPI_FIRSTBIT_MSB) ? "MSB" : "LSB",
                     (spi_debug.spi_data_size == SPI_DATASIZE_8BIT) ? "8" : "NOT_8",
                     (unsigned long)spi_debug.spi_prescaler,
                     ((spi_debug.spi_polarity == SPI_POLARITY_LOW) &&
                      (spi_debug.spi_phase == SPI_PHASE_1EDGE) &&
                      (spi_debug.spi_first_bit == SPI_FIRSTBIT_MSB) &&
                      (spi_debug.spi_data_size == SPI_DATASIZE_8BIT)) ? "PASS" : "FAIL");
  (void)uart1_printf("[LORA DIAG BOOT] VERSION first=0x%02X last=0x%02X min=0x%02X max=0x%02X ok=%u/16 read_fail=%u result=%s\r\n",
                     spi_debug.version_first, spi_debug.version_last,
                     spi_debug.version_min, spi_debug.version_max,
                     spi_debug.version_ok_count, spi_debug.version_read_failures,
                     (diagnostic_status == HAL_OK) ? "PASS" : "FAIL");
  (void)uart1_printf("[LORA DIAG BOOT] OPMODE=0x%02X MC1=0x%02X MC2=0x%02X MC3=0x%02X SYNC=0x%02X\r\n",
                     spi_debug.op_mode, spi_debug.modem_config_1,
                     spi_debug.modem_config_2, spi_debug.modem_config_3,
                     spi_debug.sync_word);

  (void)uart1_printf("[LORA] SX127x initialized: 922.1MHz BW125 SF7 CR4/5 CRC\r\n");
  (void)uart1_printf("[LORA] RX Continuous (UAV=0xFD)\r\n");
  return HAL_OK;
}

void uav_link_update_gnss(const gnss_pvt_t *pvt)
{
  if (pvt == NULL)
  {
    return;
  }

  latest_gnss_ms = HAL_GetTick();
  gnss_received = 1U;
  latest_fix_valid = ((pvt->fix_ok != 0U) && (pvt->fix_type >= 2U) &&
                      coordinate_valid(pvt->latitude_deg_1e7, pvt->longitude_deg_1e7)) ? 1U : 0U;
  latest_latitude = pvt->latitude_deg_1e7;
  latest_longitude = pvt->longitude_deg_1e7;
}

void uav_link_process(void)
{
  uint8_t packet[LORA_MAX_PACKET_SIZE];
  uint8_t packet_length = 0U;
  int16_t rssi_dbm = 0;
  int16_t snr_x100 = 0;
  lora_rx_status_t rx_status;

  process_waypoint_log();

  if (link_state == LINK_TX_DELAY)
  {
    if ((int32_t)(HAL_GetTick() - pending_tx_at_ms) < 0)
    {
      return;
    }
    if (lora_start_tx(pending_tx_packet, pending_tx_length) != HAL_OK)
    {
      (void)uart1_printf("[LORA] DELAYED TX START ERROR type=0x%02X seq=%u -> RX\r\n",
                         tx_message_type, tx_sequence);
      lora_rx_set();
      link_state = LINK_RX;
      return;
    }
    (void)uart1_printf("[LORA TX] START type=0x%02X seq=%u after guard\r\n",
                       tx_message_type, tx_sequence);
    link_state = LINK_TX;
    return;
  }

  if (link_state == LINK_TX)
  {
    lora_tx_status_t status = lora_tx_process();
    if (status == LORA_TX_DONE)
    {
      (void)uart1_printf("[LORA] TX DONE type=0x%02X seq=%u -> RX Continuous\r\n",
                         tx_message_type, tx_sequence);
      link_state = LINK_RX;
    }
    else if ((status == LORA_TX_TIMEOUT) || (status == LORA_TX_ERROR))
    {
      (void)uart1_printf("[LORA] TX ERROR type=0x%02X seq=%u status=%u -> RX\r\n",
                         tx_message_type, tx_sequence, status);
      link_state = LINK_RX;
    }
    return;
  }

  rx_status = lora_receive_packet(packet, sizeof(packet), &packet_length,
                                  &rssi_dbm, &snr_x100);
  if (rx_status == LORA_RX_PACKET)
  {
    handle_packet(packet, packet_length, rssi_dbm, snr_x100);
  }
  else if (rx_status == LORA_RX_CRC_ERROR)
  {
    (void)uart1_printf("[LORA] CRC ERROR RSSI=%d dBm\r\n", rssi_dbm);
    log_rx_debug_throttled("CRC_ERROR");
  }
  else if (rx_status == LORA_RX_INVALID_LENGTH)
  {
    (void)uart1_printf("[LORA] INVALID LENGTH (FIFO/capacity)\r\n");
    log_rx_debug_throttled("INVALID_LENGTH");
  }
  else if (rx_status == LORA_RX_ERROR)
  {
    (void)uart1_printf("[LORA] SPI/RX ERROR -> RX Continuous\r\n");
    log_rx_debug_throttled("SPI_RX_ERROR");
  }
}

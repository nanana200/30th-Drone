#include "gnss.h"

#include "uart_bridge.h"

#include <string.h>

extern UART_HandleTypeDef huart2;

#define GNSS_LINE_BUFFER_SIZE 128U
#define GNSS_RESET_PULSE_MS     5U
#define GNSS_DEFAULT_BAUDRATE   9600U
#define GNSS_DRONE_BAUDRATE     115200U
#define GNSS_UART_TIMEOUT_MS    100U
#define GNSS_CONFIG_SETTLE_MS   150U

#define GNSS_UBX_SYNC_1         0xB5U
#define GNSS_UBX_SYNC_2         0x62U
#define GNSS_UBX_CLASS_CFG      0x06U
#define GNSS_UBX_ID_VALSET      0x8AU
#define GNSS_UBX_CLASS_NAV      0x01U
#define GNSS_UBX_ID_NAV_PVT     0x07U
#define GNSS_NAV_PVT_LENGTH     92U
#define GNSS_VALSET_VERSION     0x01U
#define GNSS_VALSET_LAYER_RAM   0x01U
#define GNSS_VALSET_LAYER_BBR   0x02U
#define GNSS_VALSET_MAX_PAYLOAD 96U
#define GNSS_UBX_FRAME_OVERHEAD 8U

typedef struct
{
  uint32_t key;
  uint32_t value;
  uint8_t value_size;
} gnss_ubx_config_entry_t;

typedef enum
{
  GNSS_UBX_STATE_SYNC_1 = 0,
  GNSS_UBX_STATE_SYNC_2,
  GNSS_UBX_STATE_CLASS,
  GNSS_UBX_STATE_ID,
  GNSS_UBX_STATE_LENGTH_1,
  GNSS_UBX_STATE_LENGTH_2,
  GNSS_UBX_STATE_PAYLOAD,
  GNSS_UBX_STATE_CHECKSUM_A,
  GNSS_UBX_STATE_CHECKSUM_B
} gnss_ubx_rx_state_t;

static uint8_t gnss_rx_byte = 0U;
static char gnss_line_buffer[GNSS_LINE_BUFFER_SIZE];
static char gnss_ready_line[GNSS_LINE_BUFFER_SIZE];
static volatile uint16_t gnss_line_length = 0U;
static volatile uint16_t gnss_ready_length = 0U;
static volatile uint8_t gnss_collecting = 0U;
static volatile uint8_t gnss_sentence_ready = 0U;
static volatile uint8_t gnss_bridge_mode_active = 0U;
static volatile uint8_t gnss_pvt_ready = 0U;
static gnss_pvt_t gnss_latest_pvt;
static gnss_ubx_rx_state_t gnss_ubx_state = GNSS_UBX_STATE_SYNC_1;
static uint8_t gnss_ubx_class = 0U;
static uint8_t gnss_ubx_id = 0U;
static uint16_t gnss_ubx_length = 0U;
static uint16_t gnss_ubx_payload_index = 0U;
static uint8_t gnss_ubx_checksum_a = 0U;
static uint8_t gnss_ubx_checksum_b = 0U;
static uint8_t gnss_ubx_payload[GNSS_NAV_PVT_LENGTH];

static const gnss_ubx_config_entry_t gnss_drone_config[] = {
    {0x40520001UL, GNSS_DRONE_BAUDRATE, 4U}, /* CFG-UART1-BAUDRATE */
    {0x10740001UL, 1U, 1U},                  /* CFG-UART1OUTPROT-UBX */
    {0x10740002UL, 0U, 1U},                  /* CFG-UART1OUTPROT-NMEA */
    {0x20110021UL, 6U, 1U},                  /* CFG-NAVSPG-DYNMODEL: airborne <1g */
    {0x30210001UL, 200U, 2U},                /* CFG-RATE-MEAS: 5 Hz */
    {0x30210002UL, 1U, 2U},                  /* CFG-RATE-NAV */
    {0x20910007UL, 1U, 1U},                  /* CFG-MSGOUT-UBX_NAV_PVT-UART1 */
    {0x209100BBUL, 0U, 1U},                  /* CFG-MSGOUT-NMEA_ID_GGA-UART1 */
    {0x209100CAUL, 0U, 1U},                  /* CFG-MSGOUT-NMEA_ID_GLL-UART1 */
    {0x209100C0UL, 0U, 1U},                  /* CFG-MSGOUT-NMEA_ID_GSA-UART1 */
    {0x209100C5UL, 0U, 1U},                  /* CFG-MSGOUT-NMEA_ID_GSV-UART1 */
    {0x209100ACUL, 0U, 1U},                  /* CFG-MSGOUT-NMEA_ID_RMC-UART1 */
    {0x209100B1UL, 0U, 1U},                  /* CFG-MSGOUT-NMEA_ID_VTG-UART1 */
};

static uint32_t gnss_enter_critical(void);
static void gnss_exit_critical(uint32_t primask);

static void gnss_append_u16(uint8_t *buffer, uint16_t *index, uint16_t value)
{
  buffer[*index] = (uint8_t)(value & 0xFFU);
  (*index)++;
  buffer[*index] = (uint8_t)((value >> 8U) & 0xFFU);
  (*index)++;
}

static void gnss_append_u32(uint8_t *buffer, uint16_t *index, uint32_t value)
{
  buffer[*index] = (uint8_t)(value & 0xFFUL);
  (*index)++;
  buffer[*index] = (uint8_t)((value >> 8U) & 0xFFUL);
  (*index)++;
  buffer[*index] = (uint8_t)((value >> 16U) & 0xFFUL);
  (*index)++;
  buffer[*index] = (uint8_t)((value >> 24U) & 0xFFUL);
  (*index)++;
}

static void gnss_append_value(uint8_t *buffer, uint16_t *index, uint32_t value, uint8_t value_size)
{
  uint8_t i;

  for (i = 0U; i < value_size; i++)
  {
    buffer[*index] = (uint8_t)((value >> (8U * i)) & 0xFFUL);
    (*index)++;
  }
}

static void gnss_append_ubx_checksum(uint8_t *frame, uint16_t frame_length_without_checksum)
{
  uint8_t checksum_a = 0U;
  uint8_t checksum_b = 0U;
  uint16_t i;

  for (i = 2U; i < frame_length_without_checksum; i++)
  {
    checksum_a = (uint8_t)(checksum_a + frame[i]);
    checksum_b = (uint8_t)(checksum_b + checksum_a);
  }

  frame[frame_length_without_checksum] = checksum_a;
  frame[frame_length_without_checksum + 1U] = checksum_b;
}

static uint16_t gnss_read_u16_le(const uint8_t *buffer, uint16_t index)
{
  return (uint16_t)((uint16_t)buffer[index] | ((uint16_t)buffer[index + 1U] << 8U));
}

static uint32_t gnss_read_u32_le(const uint8_t *buffer, uint16_t index)
{
  return ((uint32_t)buffer[index] |
          ((uint32_t)buffer[index + 1U] << 8U) |
          ((uint32_t)buffer[index + 2U] << 16U) |
          ((uint32_t)buffer[index + 3U] << 24U));
}

static int32_t gnss_read_i32_le(const uint8_t *buffer, uint16_t index)
{
  return (int32_t)gnss_read_u32_le(buffer, index);
}

static void gnss_ubx_checksum_update(uint8_t data)
{
  gnss_ubx_checksum_a = (uint8_t)(gnss_ubx_checksum_a + data);
  gnss_ubx_checksum_b = (uint8_t)(gnss_ubx_checksum_b + gnss_ubx_checksum_a);
}

static void gnss_reset_ubx_state(void)
{
  gnss_ubx_state = GNSS_UBX_STATE_SYNC_1;
  gnss_ubx_class = 0U;
  gnss_ubx_id = 0U;
  gnss_ubx_length = 0U;
  gnss_ubx_payload_index = 0U;
  gnss_ubx_checksum_a = 0U;
  gnss_ubx_checksum_b = 0U;
}

static void gnss_store_nav_pvt(void)
{
  gnss_pvt_t pvt;
  uint32_t primask;

  pvt.i_tow_ms = gnss_read_u32_le(gnss_ubx_payload, 0U);
  pvt.fix_type = gnss_ubx_payload[20U];
  pvt.fix_ok = (uint8_t)(gnss_ubx_payload[21U] & 0x01U);
  pvt.satellites_used = gnss_ubx_payload[23U];
  pvt.longitude_deg_1e7 = gnss_read_i32_le(gnss_ubx_payload, 24U);
  pvt.latitude_deg_1e7 = gnss_read_i32_le(gnss_ubx_payload, 28U);
  pvt.height_mm = gnss_read_i32_le(gnss_ubx_payload, 32U);
  pvt.height_msl_mm = gnss_read_i32_le(gnss_ubx_payload, 36U);
  pvt.horizontal_accuracy_mm = gnss_read_u32_le(gnss_ubx_payload, 40U);
  pvt.vertical_accuracy_mm = gnss_read_u32_le(gnss_ubx_payload, 44U);
  pvt.north_velocity_mm_s = gnss_read_i32_le(gnss_ubx_payload, 48U);
  pvt.east_velocity_mm_s = gnss_read_i32_le(gnss_ubx_payload, 52U);
  pvt.down_velocity_mm_s = gnss_read_i32_le(gnss_ubx_payload, 56U);
  pvt.ground_speed_mm_s = gnss_read_i32_le(gnss_ubx_payload, 60U);
  pvt.heading_motion_deg_1e5 = gnss_read_i32_le(gnss_ubx_payload, 64U);
  pvt.speed_accuracy_mm_s = gnss_read_u32_le(gnss_ubx_payload, 68U);
  pvt.heading_accuracy_deg_1e5 = gnss_read_u32_le(gnss_ubx_payload, 72U);
  pvt.position_dop_0p01 = gnss_read_u16_le(gnss_ubx_payload, 76U);

  primask = gnss_enter_critical();
  gnss_latest_pvt = pvt;
  gnss_pvt_ready = 1U;
  gnss_exit_critical(primask);
}

static void gnss_handle_ubx_byte(uint8_t data)
{
  switch (gnss_ubx_state)
  {
    case GNSS_UBX_STATE_SYNC_1:
      if (data == GNSS_UBX_SYNC_1)
      {
        gnss_ubx_state = GNSS_UBX_STATE_SYNC_2;
      }
      break;

    case GNSS_UBX_STATE_SYNC_2:
      if (data == GNSS_UBX_SYNC_2)
      {
        gnss_ubx_checksum_a = 0U;
        gnss_ubx_checksum_b = 0U;
        gnss_ubx_state = GNSS_UBX_STATE_CLASS;
      }
      else
      {
        gnss_ubx_state = GNSS_UBX_STATE_SYNC_1;
      }
      break;

    case GNSS_UBX_STATE_CLASS:
      gnss_ubx_class = data;
      gnss_ubx_checksum_update(data);
      gnss_ubx_state = GNSS_UBX_STATE_ID;
      break;

    case GNSS_UBX_STATE_ID:
      gnss_ubx_id = data;
      gnss_ubx_checksum_update(data);
      gnss_ubx_state = GNSS_UBX_STATE_LENGTH_1;
      break;

    case GNSS_UBX_STATE_LENGTH_1:
      gnss_ubx_length = data;
      gnss_ubx_checksum_update(data);
      gnss_ubx_state = GNSS_UBX_STATE_LENGTH_2;
      break;

    case GNSS_UBX_STATE_LENGTH_2:
      gnss_ubx_length |= (uint16_t)((uint16_t)data << 8U);
      gnss_ubx_checksum_update(data);
      gnss_ubx_payload_index = 0U;

      if ((gnss_ubx_class == GNSS_UBX_CLASS_NAV) &&
          (gnss_ubx_id == GNSS_UBX_ID_NAV_PVT) &&
          (gnss_ubx_length == GNSS_NAV_PVT_LENGTH))
      {
        gnss_ubx_state = GNSS_UBX_STATE_PAYLOAD;
      }
      else if (gnss_ubx_length == 0U)
      {
        gnss_ubx_state = GNSS_UBX_STATE_CHECKSUM_A;
      }
      else
      {
        gnss_reset_ubx_state();
      }
      break;

    case GNSS_UBX_STATE_PAYLOAD:
      gnss_ubx_payload[gnss_ubx_payload_index] = data;
      gnss_ubx_payload_index++;
      gnss_ubx_checksum_update(data);
      if (gnss_ubx_payload_index >= gnss_ubx_length)
      {
        gnss_ubx_state = GNSS_UBX_STATE_CHECKSUM_A;
      }
      break;

    case GNSS_UBX_STATE_CHECKSUM_A:
      if (data == gnss_ubx_checksum_a)
      {
        gnss_ubx_state = GNSS_UBX_STATE_CHECKSUM_B;
      }
      else
      {
        gnss_reset_ubx_state();
      }
      break;

    case GNSS_UBX_STATE_CHECKSUM_B:
      if (data == gnss_ubx_checksum_b)
      {
        gnss_store_nav_pvt();
      }
      gnss_reset_ubx_state();
      break;

    default:
      gnss_reset_ubx_state();
      break;
  }
}

static HAL_StatusTypeDef gnss_set_uart_baudrate(uint32_t baudrate)
{
  HAL_StatusTypeDef status;

  (void)HAL_UART_Abort(&huart2);
  huart2.Init.BaudRate = baudrate;

  status = HAL_UART_Init(&huart2);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8);
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_UARTEx_DisableFifoMode(&huart2);
}

static HAL_StatusTypeDef gnss_send_drone_config(void)
{
  uint8_t frame[GNSS_VALSET_MAX_PAYLOAD + GNSS_UBX_FRAME_OVERHEAD];
  uint16_t index = 0U;
  uint16_t length_index;
  uint16_t payload_length;
  uint16_t i;

  frame[index++] = GNSS_UBX_SYNC_1;
  frame[index++] = GNSS_UBX_SYNC_2;
  frame[index++] = GNSS_UBX_CLASS_CFG;
  frame[index++] = GNSS_UBX_ID_VALSET;
  length_index = index;
  index += 2U;

  frame[index++] = GNSS_VALSET_VERSION;
  frame[index++] = (uint8_t)(GNSS_VALSET_LAYER_RAM | GNSS_VALSET_LAYER_BBR);
  frame[index++] = 0U;
  frame[index++] = 0U;

  for (i = 0U; i < (uint16_t)(sizeof(gnss_drone_config) / sizeof(gnss_drone_config[0])); i++)
  {
    gnss_append_u32(frame, &index, gnss_drone_config[i].key);
    gnss_append_value(frame, &index, gnss_drone_config[i].value, gnss_drone_config[i].value_size);
  }

  payload_length = (uint16_t)(index - GNSS_UBX_FRAME_OVERHEAD + 2U);
  gnss_append_u16(frame, &length_index, payload_length);
  gnss_append_ubx_checksum(frame, index);
  index += 2U;

  return HAL_UART_Transmit(&huart2, frame, index, GNSS_UART_TIMEOUT_MS);
}

static HAL_StatusTypeDef gnss_configure_for_drone(void)
{
  HAL_StatusTypeDef status;
  uint8_t retry;

  /* 1. STM32 USART2를 9600 bps로 초기화 */
  (void)gnss_set_uart_baudrate(GNSS_DEFAULT_BAUDRATE);

  /* 1.5. [임시 조치] 꼬인 GPS 수신기 메모리를 공장초기화 상태로 밀어버리는 Cold Start 명령 송신 */
  uint8_t cold_start_cmd[] = {0xB5U, 0x62U, 0x06U, 0x04U, 0x04U, 0x00U, 0xFFU, 0xFFU, 0x02U, 0x00U, 0x0EU, 0x61U};
  (void)HAL_UART_Transmit(&huart2, cold_start_cmd, sizeof(cold_start_cmd), GNSS_UART_TIMEOUT_MS);
  HAL_Delay(2000U); /* GPS 모듈이 메모리를 완벽히 포맷하고 재부팅할 때까지 2초간 대기 */

  /* 2. Overrun 등 기존의 하드웨어 에러 플래그와 RX 레지스터 클리어 */
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  volatile uint32_t dummy = huart2.Instance->RDR;
  (void)dummy;

  /* 3. 9600 bps로 설정 프레임을 여러 번 전송하여 모듈이 일부 유실하더라도 무조건 수신 보장 */
  for (retry = 0U; retry < 4U; retry++)
  {
    (void)gnss_send_drone_config();
    HAL_Delay(100U);
  }

  /* 4. STM32 USART2를 115200 bps로 변경 */
  status = gnss_set_uart_baudrate(GNSS_DRONE_BAUDRATE);
  if (status != HAL_OK)
  {
    return status;
  }

  /* 5. 115200 bps에서도 에러 플래그 및 레지스터 정리 */
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  dummy = huart2.Instance->RDR;
  (void)dummy;

  /* 6. 115200 bps로 설정을 여러 번 재전송하여 설정을 확실히 적용 */
  for (retry = 0U; retry < 3U; retry++)
  {
    status = gnss_send_drone_config();
    HAL_Delay(100U);
  }

  return status;
}

static uint32_t gnss_enter_critical(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();

  return primask;
}

static void gnss_exit_critical(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static HAL_StatusTypeDef gnss_start_receive_it(void)
{
  return HAL_UART_Receive_IT(&huart2, &gnss_rx_byte, 1U);
}

static void gnss_reset_line_state(void)
{
  gnss_line_length = 0U;
  gnss_collecting = 0U;
}

HAL_StatusTypeDef gnss_init(void)
{
  HAL_StatusTypeDef status;

  /* 하드웨어 리셋 수행 및 부팅 완료 충분히 대기 (2초) */
  gnss_reset();
  HAL_Delay(2000U);

  gnss_reset_line_state();
  gnss_reset_ubx_state();
  gnss_sentence_ready = 0U;
  gnss_ready_length = 0U;
  gnss_pvt_ready = 0U;
  gnss_bridge_mode_active = 0U;

  status = gnss_configure_for_drone();
  if (status != HAL_OK)
  {
    return status;
  }

  /* 인터럽트 시작하기 직전에 다시 한 번 에러 플래그와 RX 레지스터 완벽히 비우기 */
  __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
  volatile uint32_t dummy = huart2.Instance->RDR;
  (void)dummy;

  return gnss_start_receive_it();
}

void gnss_reset(void)
{
  /* u-blox 가이드 기준 RESET_N은 최소 100ms 동안 LOW로 유지해야 깔끔한 리셋 보장 */
  HAL_GPIO_WritePin(UART2_RESET_GPIO_Port, UART2_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(100U);
  HAL_GPIO_WritePin(UART2_RESET_GPIO_Port, UART2_RESET_Pin, GPIO_PIN_SET);
}

bool gnss_read_line(char *buffer, size_t buffer_size)
{
  uint16_t ready_length;
  uint32_t primask;

  if ((buffer == NULL) || (buffer_size == 0U))
  {
    return false;
  }

  if (gnss_bridge_mode_active != 0U)
  {
    return false;
  }

  primask = gnss_enter_critical();

  if (gnss_sentence_ready == 0U)
  {
    gnss_exit_critical(primask);
    return false;
  }

  ready_length = gnss_ready_length;
  if (ready_length >= buffer_size)
  {
    ready_length = (uint16_t)(buffer_size - 1U);
  }

  memcpy(buffer, gnss_ready_line, ready_length);
  buffer[ready_length] = '\0';
  gnss_sentence_ready = 0U;

  gnss_exit_critical(primask);

  return true;
}

bool gnss_read_pvt(gnss_pvt_t *pvt)
{
  uint32_t primask;

  if (pvt == NULL)
  {
    return false;
  }

  if (gnss_bridge_mode_active != 0U)
  {
    return false;
  }

  primask = gnss_enter_critical();

  if (gnss_pvt_ready == 0U)
  {
    gnss_exit_critical(primask);
    return false;
  }

  *pvt = gnss_latest_pvt;
  gnss_pvt_ready = 0U;

  gnss_exit_critical(primask);

  return true;
}

void gnss_set_bridge_mode(uint8_t active)
{
  gnss_bridge_mode_active = active;
  gnss_reset_line_state();
  gnss_reset_ubx_state();
  gnss_sentence_ready = 0U;
  gnss_ready_length = 0U;
  gnss_pvt_ready = 0U;

  (void)HAL_UART_AbortReceive(&huart2);
  (void)gnss_start_receive_it();
}

void gnss_handle_uart_error(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->Instance != USART2))
  {
    return;
  }

  gnss_reset_line_state();
  gnss_reset_ubx_state();
  gnss_sentence_ready = 0U;
  gnss_pvt_ready = 0U;
  (void)HAL_UART_AbortReceive(huart);
  (void)gnss_start_receive_it();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  if ((huart == NULL) || (huart->Instance != USART2))
  {
    return;
  }

  if (gnss_bridge_mode_active != 0U)
  {
    (void)uart_bridge_enqueue_gnss_byte(gnss_rx_byte);
    (void)gnss_start_receive_it();
    return;
  }

  gnss_handle_ubx_byte(gnss_rx_byte);

  if (gnss_rx_byte == '$')
  {
    gnss_line_buffer[0] = '$';
    gnss_line_length = 1U;
    gnss_collecting = 1U;
  }
  else if (gnss_collecting != 0U)
  {
    if (gnss_rx_byte == '\r')
    {
      (void)gnss_start_receive_it();
      return;
    }

    if (gnss_rx_byte == '\n')
    {
      if (gnss_line_length > 0U)
      {
        primask = gnss_enter_critical();
        memcpy(gnss_ready_line, gnss_line_buffer, gnss_line_length);
        gnss_ready_line[gnss_line_length] = '\0';
        gnss_ready_length = gnss_line_length;
        gnss_sentence_ready = 1U;
        gnss_exit_critical(primask);
      }

      gnss_reset_line_state();
      (void)gnss_start_receive_it();
      return;
    }

    if (gnss_line_length < (GNSS_LINE_BUFFER_SIZE - 1U))
    {
      gnss_line_buffer[gnss_line_length] = (char)gnss_rx_byte;
      gnss_line_length++;
    }
    else
    {
      gnss_reset_line_state();
    }
  }

  (void)gnss_start_receive_it();
}

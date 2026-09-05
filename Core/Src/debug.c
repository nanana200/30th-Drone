#include "debug.h"
#include "gnss.h"
#include "motor.h"
#include "oled.h"
#include "main.h"
#include "sensor.h"
#include "uart_bridge.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim3;
extern uint32_t user_step_throttle_compare;
extern float battery_voltage;
extern int8_t battery_percent;

#define UART1_DMA_TX_QUEUE_LENGTH 8U
#define UART1_DMA_TX_BUFFER_SIZE  160U
#define UART1_DMA_CACHE_LINE_SIZE 32U
#define UART1_DMA_RX_BUFFER_SIZE  16U
#define UART1_TRIGGER_HOLD_MS      1000U
#define UART6_DMA_TX_QUEUE_LENGTH 8U
#define UART6_DMA_TX_BUFFER_SIZE  80U
#define UART6_DMA_RX_BUFFER_SIZE  16U

typedef struct
{
  uint8_t data[UART1_DMA_TX_BUFFER_SIZE];
  uint16_t length;
  uint8_t reserved[30];
} uart1_dma_tx_entry_t;

typedef struct
{
  uint8_t data[UART6_DMA_TX_BUFFER_SIZE];
  uint16_t length;
  uint8_t reserved[14];
} uart6_dma_tx_entry_t;

static uart1_dma_tx_entry_t uart1_dma_tx_queue[UART1_DMA_TX_QUEUE_LENGTH]
  __attribute__((aligned(UART1_DMA_CACHE_LINE_SIZE), section(".dma_buffer")));
static uart6_dma_tx_entry_t uart6_dma_tx_queue[UART6_DMA_TX_QUEUE_LENGTH]
  __attribute__((aligned(UART1_DMA_CACHE_LINE_SIZE), section(".dma_buffer")));
static uint8_t uart1_dma_rx_buffer[UART1_DMA_RX_BUFFER_SIZE]
  __attribute__((aligned(UART1_DMA_CACHE_LINE_SIZE), section(".dma_buffer")));
static uint8_t uart6_dma_rx_buffer[UART6_DMA_RX_BUFFER_SIZE]
  __attribute__((aligned(UART1_DMA_CACHE_LINE_SIZE), section(".dma_buffer")));
static volatile uint8_t uart1_dma_tx_busy = 0U;
static volatile uint8_t uart6_dma_tx_busy = 0U;
static volatile uint8_t uart1_dma_tx_head = 0U;
static volatile uint8_t uart1_dma_tx_tail = 0U;
static volatile uint8_t uart1_dma_tx_count = 0U;
static volatile uint8_t uart6_dma_tx_head = 0U;
static volatile uint8_t uart6_dma_tx_tail = 0U;
static volatile uint8_t uart6_dma_tx_count = 0U;
volatile float uart1_rx_float_value = 0.0f;
volatile float uart6_rx_float_value = 0.0f;
volatile uint8_t debug_uart_update_flag = 0U;
volatile uint8_t debug_uart6_update_flag = 0U;
volatile uint8_t debug_oled_update_flag = 0U;
volatile uint8_t debug_oled_tick_divider = 0U;
volatile uint32_t main_flag = 0U;
static volatile uint8_t uart1_trigger_active = 0U;
static volatile uint32_t uart1_trigger_start_tick = 0U;
static volatile uint8_t uart6_start_received = 0U;
static volatile uint8_t debug_bridge_mode_active = 0U;

static HAL_StatusTypeDef UART6_DMATxEnqueue(const uint8_t *data, uint16_t length);

static void UART1_DMARxStart(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1,
                                   uart1_dma_rx_buffer,
                                   UART1_DMA_RX_BUFFER_SIZE) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
}

static void UART6_DMARxStart(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6,
                                   uart6_dma_rx_buffer,
                                   UART6_DMA_RX_BUFFER_SIZE) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
  }
}

static void UART6_DMATxRespond(uint16_t length)
{
  static const uint8_t started_text[] = {'S', 'T', 'A', 'R', 'T', 'E', 'D'};
  static const uint8_t retry_text[] = {'R', 'E', 'T', 'R', 'Y'};
  const uint8_t *tx_data;
  uint16_t tx_length;
  char rx_text[UART6_DMA_RX_BUFFER_SIZE];
  char *parse_end;
  float rx_value;

  while ((length > 0U) &&
         ((uart6_dma_rx_buffer[length - 1U] == '\r') || (uart6_dma_rx_buffer[length - 1U] == '\n')))
  {
    length--;
  }

  if (uart6_start_received == 0U)
  {
    if ((length == 5U) &&
        (uart6_dma_rx_buffer[0] == 'S') &&
        (uart6_dma_rx_buffer[1] == 'T') &&
        (uart6_dma_rx_buffer[2] == 'A') &&
        (uart6_dma_rx_buffer[3] == 'R') &&
        (uart6_dma_rx_buffer[4] == 'T'))
    {
      uart6_start_received = 1U;
      tx_data = started_text;
      tx_length = 7U;
    }
    else
    {
      tx_data = retry_text;
      tx_length = 5U;
    }
  }
  else
  {
    if (length >= UART6_DMA_RX_BUFFER_SIZE)
    {
      length = UART6_DMA_RX_BUFFER_SIZE - 1U;
    }

    memcpy(rx_text, uart6_dma_rx_buffer, length);
    rx_text[length] = '\0';
    rx_value = strtof(rx_text, &parse_end);

    if ((parse_end == rx_text) || (*parse_end != '\0') || (rx_value < 1000.0f) || (rx_value > 2000.0f))
    {
      return;
    }
    else
    {
      uart6_rx_float_value = rx_value;
      motor_set_throttle((uint32_t)rx_value);
      tx_data = uart6_dma_rx_buffer;
      tx_length = length;
    }
  }

  (void)UART6_DMATxEnqueue(tx_data, tx_length);
}

static void UART1_DMARxStoreFloat(uint16_t length)
{
  char rx_text[UART1_DMA_RX_BUFFER_SIZE];
  char *parse_end;
  long trigger_index;
  float target_roll_rate_dps;

  if ((length == 0U) || (length >= UART1_DMA_RX_BUFFER_SIZE))
  {
    return;
  }

  memcpy(rx_text, uart1_dma_rx_buffer, length);
  rx_text[length] = '\0';

  parse_end = rx_text;
  while (*parse_end == ' ')
  {
    parse_end++;
  }

  if (*parse_end != 'T')
  {
    return;
  }

  parse_end++;

  while (*parse_end == ' ')
  {
    parse_end++;
  }

  if (*parse_end != ',')
  {
    return;
  }

  parse_end++;

  while (*parse_end == ' ')
  {
    parse_end++;
  }

  trigger_index = strtol(parse_end, &parse_end, 10);
  if ((*parse_end != '\0') && (*parse_end != ' ') && (*parse_end != '\r') && (*parse_end != '\n'))
  {
    return;
  }

  target_roll_rate_dps = 0.0f;

  if (trigger_index == 1)
  {
    target_roll_rate_dps = 5.0f;
  }
  else if (trigger_index == 2)
  {
    target_roll_rate_dps = 10.0f;
  }
  else if (trigger_index == 3)
  {
    target_roll_rate_dps = 15.0f;
  }
  else if (trigger_index != 0)
  {
    return;
  }

  uart1_rx_float_value = target_roll_rate_dps;
  motor_set_rate_targets(target_roll_rate_dps, 0.0f, 0.0f);

  if (target_roll_rate_dps > 0.0f)
  {
    uart1_trigger_active = 1U;
    uart1_trigger_start_tick = HAL_GetTick();
  }
  else
  {
    uart1_trigger_active = 0U;
  }

  debug_oled_update_flag = 1U;
}

static int DebugAbs(int value)
{
  return (value < 0) ? -value : value;
}

static int DebugSignScaledTenths(float value)
{
  return (value < 0.0f) ? -DebugAbs((int)(value * 10.0f)) : DebugAbs((int)(value * 10.0f));
}

static int DebugIntegerPartFromScaled(int scaled_value, int scale)
{
  if ((scaled_value < 0) && (DebugAbs(scaled_value) < scale))
  {
    return -0;
  }

  return scaled_value / scale;
}

static uint32_t UART1_DMATxEnterCritical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void UART1_DMATxExitCritical(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

// static void UART1_DMATxCleanCache(const void *address, uint32_t length)
// {
//   //uintptr_t aligned_address = ((uintptr_t)address) & ~(uintptr_t)(UART1_DMA_CACHE_LINE_SIZE - 1U);
//   //uint32_t aligned_length = (uint32_t)(((uintptr_t)address + length + (UART1_DMA_CACHE_LINE_SIZE - 1U)) - aligned_address);

//   //SCB_CleanDCache_by_Addr((uint32_t *)aligned_address, (int32_t)aligned_length);
// }

static void UART1_DMATxStartNext(void)
{
  uint32_t primask;
  uint8_t queue_tail;
  HAL_StatusTypeDef status;

  primask = UART1_DMATxEnterCritical();

  if ((uart1_dma_tx_busy != 0U) || (uart1_dma_tx_count == 0U))
  {
    UART1_DMATxExitCritical(primask);
    return;
  }

  queue_tail = uart1_dma_tx_tail;
  uart1_dma_tx_busy = 1U;
  UART1_DMATxExitCritical(primask);

  //UART1_DMATxCleanCache(uart1_dma_tx_queue[queue_tail].data, UART1_DMA_TX_BUFFER_SIZE);

  status = HAL_UART_Transmit_DMA(&huart1,
                                 uart1_dma_tx_queue[queue_tail].data,
                                 uart1_dma_tx_queue[queue_tail].length);

  if (status != HAL_OK)
  {
    HAL_UART_Abort(&huart1);

    primask = UART1_DMATxEnterCritical();
    uart1_dma_tx_busy = 0U;
    UART1_DMATxExitCritical(primask);
  }
}

static HAL_StatusTypeDef UART1_DMATxEnqueue(const uint8_t *data, uint16_t length)
{
  uint32_t primask;
  uint8_t queue_head;

  if ((data == NULL) || (length == 0U) || (length > UART1_DMA_TX_BUFFER_SIZE))
  {
    return HAL_ERROR;
  }

  primask = UART1_DMATxEnterCritical();

  if (uart1_dma_tx_count >= UART1_DMA_TX_QUEUE_LENGTH)
  {
    UART1_DMATxExitCritical(primask);
    return HAL_BUSY;
  }

  queue_head = uart1_dma_tx_head;
  memcpy(uart1_dma_tx_queue[queue_head].data, data, length);
  uart1_dma_tx_queue[queue_head].length = length;
  uart1_dma_tx_head = (uint8_t)((queue_head + 1U) % UART1_DMA_TX_QUEUE_LENGTH);
  uart1_dma_tx_count++;

  UART1_DMATxExitCritical(primask);

  UART1_DMATxStartNext();

  return HAL_OK;
}

static void UART6_DMATxStartNext(void)
{
  uint32_t primask;
  uint8_t queue_tail;
  HAL_StatusTypeDef status;

  primask = UART1_DMATxEnterCritical();

  if ((uart6_dma_tx_busy != 0U) || (uart6_dma_tx_count == 0U))
  {
    UART1_DMATxExitCritical(primask);
    return;
  }

  queue_tail = uart6_dma_tx_tail;
  uart6_dma_tx_busy = 1U;
  UART1_DMATxExitCritical(primask);

  status = HAL_UART_Transmit_DMA(&huart6,
                                 uart6_dma_tx_queue[queue_tail].data,
                                 uart6_dma_tx_queue[queue_tail].length);

  if (status != HAL_OK)
  {
    HAL_UART_Abort(&huart6);

    primask = UART1_DMATxEnterCritical();
    uart6_dma_tx_busy = 0U;
    UART1_DMATxExitCritical(primask);
  }
}

static HAL_StatusTypeDef UART6_DMATxEnqueue(const uint8_t *data, uint16_t length)
{
  uint32_t primask;
  uint8_t queue_head;

  if ((data == NULL) || (length == 0U) || (length > UART6_DMA_TX_BUFFER_SIZE))
  {
    return HAL_ERROR;
  }

  primask = UART1_DMATxEnterCritical();

  if (uart6_dma_tx_count >= UART6_DMA_TX_QUEUE_LENGTH)
  {
    UART1_DMATxExitCritical(primask);
    return HAL_BUSY;
  }

  queue_head = uart6_dma_tx_head;
  memcpy(uart6_dma_tx_queue[queue_head].data, data, length);
  uart6_dma_tx_queue[queue_head].length = length;
  uart6_dma_tx_head = (uint8_t)((queue_head + 1U) % UART6_DMA_TX_QUEUE_LENGTH);
  uart6_dma_tx_count++;

  UART1_DMATxExitCritical(primask);

  UART6_DMATxStartNext();

  return HAL_OK;
}

static void Debug_SendUart6Telemetry(void)
{
  char tx_buffer[UART6_DMA_TX_BUFFER_SIZE];
  int roll_rate = (int)(sensor_gyro_x_dps * 100.0f);
  int pitch_rate = (int)(sensor_gyro_y_dps * 100.0f);
  int yaw_rate = (int)(sensor_gyro_z_dps * 100.0f);
  int roll_angle = (int)(sensor_roll_deg * 100.0f);
  int pitch_angle = (int)(sensor_pitch_deg * 100.0f);
  int length;

  length = snprintf(tx_buffer, sizeof(tx_buffer), "%c%d.%02d, %c%d.%02d, %c%d.%02d, %c%d.%02d, %c%d.%02d\r\n",
                    (roll_rate < 0) ? '-' : ' ', DebugAbs(roll_rate) / 100, DebugAbs(roll_rate) % 100,
                    (pitch_rate < 0) ? '-' : ' ', DebugAbs(pitch_rate) / 100, DebugAbs(pitch_rate) % 100,
                    (yaw_rate < 0) ? '-' : ' ', DebugAbs(yaw_rate) / 100, DebugAbs(yaw_rate) % 100,
                    (roll_angle < 0) ? '-' : ' ', DebugAbs(roll_angle) / 100, DebugAbs(roll_angle) % 100,
                    (pitch_angle < 0) ? '-' : ' ', DebugAbs(pitch_angle) / 100, DebugAbs(pitch_angle) % 100);

  if (length > 0)
  {
    if (length >= (int)sizeof(tx_buffer))
    {
      length = (int)sizeof(tx_buffer) - 1;
    }
    (void)UART6_DMATxEnqueue((uint8_t *)tx_buffer, (uint16_t)length);
  }
}

static void Debug_SendUartSnapshot(void)
{
  int gx_tenths = DebugSignScaledTenths(sensor_gyro_x_dps);
  int gy_tenths = DebugSignScaledTenths(sensor_gyro_y_dps);
  int gz_tenths = DebugSignScaledTenths(sensor_gyro_z_dps);
  int roll_tenths = DebugSignScaledTenths(sensor_roll_deg);
  int pitch_tenths = DebugSignScaledTenths(sensor_pitch_deg);
  int yaw_tenths = DebugSignScaledTenths(sensor_yaw_deg);
  float display_voltage = battery_voltage + 0.05f;
  uint8_t v_int = (uint8_t)display_voltage;
  uint8_t v_dec = (uint8_t)((display_voltage - v_int) * 10.0f);

  (void)uart1_printf("GX %c%d.%01d  GY %c%d.%01d  GZ %c%d.%01d  "
                     "BAT %d.%dV %d%%  R %c%d.%01d  P %c%d.%01d  Y %c%d.%01d\r\n",
                     (gx_tenths < 0) ? '-' : ' ', DebugIntegerPartFromScaled(gx_tenths, 10), DebugAbs(gx_tenths) % 10,
                     (gy_tenths < 0) ? '-' : ' ', DebugIntegerPartFromScaled(gy_tenths, 10), DebugAbs(gy_tenths) % 10,
                     (gz_tenths < 0) ? '-' : ' ', DebugIntegerPartFromScaled(gz_tenths, 10), DebugAbs(gz_tenths) % 10,
                     v_int, v_dec, battery_percent,
                     (roll_tenths < 0) ? '-' : ' ', DebugIntegerPartFromScaled(roll_tenths, 10), DebugAbs(roll_tenths) % 10,
                     (pitch_tenths < 0) ? '-' : ' ', DebugIntegerPartFromScaled(pitch_tenths, 10), DebugAbs(pitch_tenths) % 10,
                     (yaw_tenths < 0) ? '-' : ' ', DebugIntegerPartFromScaled(yaw_tenths, 10), DebugAbs(yaw_tenths) % 10);
}

static void Debug_UpdateOledSnapshot(void)
{
  int gx_tenths = DebugSignScaledTenths(sensor_gyro_x_dps);
  int gy_tenths = DebugSignScaledTenths(sensor_gyro_y_dps);
  int gz_tenths = DebugSignScaledTenths(sensor_gyro_z_dps);
  int roll_tenths = DebugSignScaledTenths(sensor_roll_deg);
  int pitch_tenths = DebugSignScaledTenths(sensor_pitch_deg);
  int yaw_tenths = DebugSignScaledTenths(sensor_yaw_deg);
  char gx_sign = (gx_tenths < 0) ? '-' : ' ';
  char gy_sign = (gy_tenths < 0) ? '-' : ' ';
  char gz_sign = (gz_tenths < 0) ? '-' : ' ';
  char roll_sign = (roll_tenths < 0) ? '-' : ' ';
  char pitch_sign = (pitch_tenths < 0) ? '-' : ' ';
  char yaw_sign = (yaw_tenths < 0) ? '-' : ' ';

  float display_voltage = battery_voltage + 0.05f; // 소수점 첫째자리 반올림
  uint8_t v_int = (uint8_t)display_voltage;
  uint8_t v_dec = (uint8_t)((display_voltage - v_int) * 10.0f);

  OLED_Clear();
  OLED_Printf(0, 0, "GX %c%d.%01d", gx_sign, DebugIntegerPartFromScaled(gx_tenths, 10), DebugAbs(gx_tenths) % 10);
  OLED_Printf(1, 0, "GY %c%d.%01d", gy_sign, DebugIntegerPartFromScaled(gy_tenths, 10), DebugAbs(gy_tenths) % 10);
  OLED_Printf(2, 0, "GZ %c%d.%01d", gz_sign, DebugIntegerPartFromScaled(gz_tenths, 10), DebugAbs(gz_tenths) % 10);
  OLED_Printf(3, 0, "BAT %d.%dV %d%%", v_int, v_dec, battery_percent);
  OLED_Printf(4, 0, "                ");
  OLED_Printf(5, 0, "                ");
  OLED_Printf(6, 0, "R %c%d.%01d P %c%d.%01d", roll_sign, DebugIntegerPartFromScaled(roll_tenths, 10), DebugAbs(roll_tenths) % 10,
                                          pitch_sign, DebugIntegerPartFromScaled(pitch_tenths, 10), DebugAbs(pitch_tenths) % 10);
  OLED_Printf(7, 0, "Y %c%d.%01d", yaw_sign, DebugIntegerPartFromScaled(yaw_tenths, 10), DebugAbs(yaw_tenths) % 10);
  OLED_Update();
}

void debug_init(void)
{
  debug_uart_update_flag = 0U;
  debug_uart6_update_flag = 0U;
  debug_oled_update_flag = 0U;
  debug_oled_tick_divider = 0U;
  main_flag = 0U;
  uart1_rx_float_value = 0.0f;
  uart1_trigger_active = 0U;
  uart1_trigger_start_tick = 0U;
  uart6_rx_float_value = 0.0f;
  uart6_start_received = 0U;
  debug_bridge_mode_active = 0U;
  uart1_dma_tx_busy = 0U;
  uart1_dma_tx_head = 0U;
  uart1_dma_tx_tail = 0U;
  uart1_dma_tx_count = 0U;
  uart6_dma_tx_busy = 0U;
  uart6_dma_tx_head = 0U;
  uart6_dma_tx_tail = 0U;
  uart6_dma_tx_count = 0U;
  UART1_DMARxStart();
  UART6_DMARxStart();
}

void debug_set_bridge_mode(uint8_t active)
{
  uint32_t primask = UART1_DMATxEnterCritical();

  debug_bridge_mode_active = active;
  uart1_dma_tx_busy = 0U;
  uart1_dma_tx_head = 0U;
  uart1_dma_tx_tail = 0U;
  uart1_dma_tx_count = 0U;

  UART1_DMATxExitCritical(primask);

  (void)HAL_UART_AbortTransmit(&huart1);
  (void)HAL_UART_AbortReceive(&huart1);
  UART1_DMARxStart();
}

void debug_process(void)
{
  if ((uart1_trigger_active != 0U) &&
      ((HAL_GetTick() - uart1_trigger_start_tick) >= UART1_TRIGGER_HOLD_MS))
  {
    uart1_trigger_active = 0U;
    uart1_rx_float_value = 0.0f;
    motor_set_rate_targets(0.0f, 0.0f, 0.0f);
    debug_oled_update_flag = 1U;
  }

  if (debug_uart_update_flag != 0U)
  {
    debug_uart_update_flag = 0U;
    if (debug_bridge_mode_active == 0U)
    {
      Debug_SendUartSnapshot();
    }
  }

  if (debug_oled_update_flag != 0U)
  {
    debug_oled_update_flag = 0U;
    // OLED_Clear();
    // OLED_Printf(3, 0, "MOTOR = %d", user_step_throttle_compare);
    // OLED_Update();
    Debug_UpdateOledSnapshot();
  }

  if (debug_uart6_update_flag != 0U)
  {
    debug_uart6_update_flag = 0U;
    Debug_SendUart6Telemetry();
  }
}

int uart1_printf(const char *format, ...)
{
  uint8_t tx_buffer[UART1_DMA_TX_BUFFER_SIZE];
  int length;
  va_list args;

  if (format == NULL)
  {
    return -1;
  }

  if (debug_bridge_mode_active != 0U)
  {
    return 0;
  }

  va_start(args, format);
  length = vsnprintf((char *)tx_buffer, sizeof(tx_buffer), format, args);
  va_end(args);

  if (length <= 0)
  {
    return length;
  }

  if (length >= (int)sizeof(tx_buffer))
  {
    length = (int)sizeof(tx_buffer) - 1;
  }

  if (UART1_DMATxEnqueue(tx_buffer, (uint16_t)length) != HAL_OK)
  {
    return -1;
  }

  return length;
}

HAL_StatusTypeDef debug_uart1_write_raw(const uint8_t *data, uint16_t length)
{
  if (debug_bridge_mode_active == 0U)
  {
    return HAL_BUSY;
  }

  return UART1_DMATxEnqueue(data, length);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART6)
  {
    primask = UART1_DMATxEnterCritical();

    if (uart6_dma_tx_count > 0U)
    {
      uart6_dma_tx_tail = (uint8_t)((uart6_dma_tx_tail + 1U) % UART6_DMA_TX_QUEUE_LENGTH);
      uart6_dma_tx_count--;
    }

    uart6_dma_tx_busy = 0U;
    UART1_DMATxExitCritical(primask);

    UART6_DMATxStartNext();
    return;
  }

  if (huart->Instance != USART1)
  {
    return;
  }

  primask = UART1_DMATxEnterCritical();

  if (uart1_dma_tx_count > 0U)
  {
    uart1_dma_tx_tail = (uint8_t)((uart1_dma_tx_tail + 1U) % UART1_DMA_TX_QUEUE_LENGTH);
    uart1_dma_tx_count--;
  }

  uart1_dma_tx_busy = 0U;
  UART1_DMATxExitCritical(primask);

  UART1_DMATxStartNext();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uint32_t primask;

  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART2)
  {
    gnss_handle_uart_error(huart);
    return;
  }

  if (huart->Instance == USART6)
  {
    primask = UART1_DMATxEnterCritical();
    uart6_dma_tx_busy = 0U;
    UART1_DMATxExitCritical(primask);

    (void)HAL_UART_AbortTransmit(huart);
    UART6_DMARxStart();
    return;
  }

  if (huart->Instance != USART1)
  {
    return;
  }

  primask = UART1_DMATxEnterCritical();

  uart1_dma_tx_busy = 0U;
  UART1_DMATxExitCritical(primask);

  (void)HAL_UART_AbortTransmit(huart);

  UART1_DMARxStart();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART6)
  {
    UART6_DMATxRespond(Size);
    UART6_DMARxStart();
    return;
  }

  if (huart->Instance != USART1)
  {
    return;
  }

  if (debug_bridge_mode_active != 0U)
  {
    (void)uart_bridge_enqueue_pc_data(uart1_dma_rx_buffer, Size);
    UART1_DMARxStart();
    return;
  }

  UART1_DMARxStoreFloat(Size);
  UART1_DMARxStart();
}

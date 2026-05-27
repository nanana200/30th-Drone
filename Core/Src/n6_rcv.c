#include "n6_rcv.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *n6_uart = NULL;

static uint8_t n6_rx_byte = 0;

static char n6_line_buf[N6_RCV_LINE_BUF_SIZE];
static volatile uint16_t n6_line_idx = 0;

static char n6_parse_buf[N6_RCV_LINE_BUF_SIZE];
static volatile uint8_t n6_line_ready = 0;

/* Latest N6 detection state */
static volatile uint8_t n6_detected = 0;
static volatile uint8_t n6_person_count = 0;
static volatile uint32_t n6_last_rx_tick = 0;

/* Internal functions */
static void N6_RCV_ParseLine(char *line);
static void N6_RCV_StartReceiveIT(void);
static void N6_RCV_PushByte(uint8_t byte);

// 맨 처음 n6관련 초기화 함수
void N6_RCV_Init(UART_HandleTypeDef *huart)
{
    n6_uart = huart; // N6 UART handle. Current board uses UART7 with PE8 as swapped RX.

    n6_rx_byte = 0;
    n6_line_idx = 0;
    n6_line_ready = 0;

    memset(n6_line_buf, 0, sizeof(n6_line_buf));
    memset(n6_parse_buf, 0, sizeof(n6_parse_buf));

    n6_detected = 0;
    n6_person_count = 0;
    n6_last_rx_tick = HAL_GetTick();

    N6_RCV_StartReceiveIT(); // n6에서 신호 들어올때 어케 할지 초기화 세팅함
}

static void N6_RCV_StartReceiveIT(void)
{
    if (n6_uart != NULL)
    {
        /* N6는 드론 보드로 감지 결과를 보내기만 하므로 RX 하나만 쓴다.
         * PE8 = UART7 swapped RX, using 1byte interrupt so main loop is not blocked.
         */
        (void)HAL_UART_Receive_IT(n6_uart, &n6_rx_byte, 1);
    }
}

// uart 수신 인터럽트가 발생할 때 이 함수가 실행됨
static void N6_RCV_PushByte(uint8_t byte)
{
    char ch = (char)byte; // n6에서 보낸 문자를 1byte로 저장

    if (ch == '\n')   // 문장이 끝났다면
    {
        n6_line_buf[n6_line_idx] = '\0';

        if (n6_line_ready == 0)
        {
            strncpy(n6_parse_buf, n6_line_buf, N6_RCV_LINE_BUF_SIZE - 1);
            n6_parse_buf[N6_RCV_LINE_BUF_SIZE - 1] = '\0';
            n6_line_ready = 1;
        }

        n6_line_idx = 0;
        memset(n6_line_buf, 0, sizeof(n6_line_buf));
    }
    else if (ch == '\r')
    {
        // \r은 버림
    }
    else
    {
        if (n6_line_idx < (N6_RCV_LINE_BUF_SIZE - 1))
        {
            n6_line_buf[n6_line_idx++] = ch;
        }
        else
        {
            n6_line_idx = 0;
            memset(n6_line_buf, 0, sizeof(n6_line_buf));
        }
    }
}

void N6_RCV_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (n6_uart == NULL)
    {
        return;
    }

    if (huart != n6_uart)
    {
        return;
    }

    N6_RCV_PushByte(n6_rx_byte);
    N6_RCV_StartReceiveIT();
}

void N6_RCV_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)huart;
    (void)size;
    /* 예전 DMA ReceiveToIdle 방식용 hook.
     * Current drone integration uses UART7 1byte interrupt RX, so nothing is handled here.
     */
}

void N6_RCV_Process(void)
{
    if (n6_line_ready)
    {
        char local_line[N6_RCV_LINE_BUF_SIZE];

        __disable_irq();
        strncpy(local_line, n6_parse_buf, N6_RCV_LINE_BUF_SIZE - 1);
        local_line[N6_RCV_LINE_BUF_SIZE - 1] = '\0';
        n6_line_ready = 0;
        __enable_irq();

        N6_RCV_ParseLine(local_line);
    }
}

static void N6_RCV_ParseLine(char *line) // n6에서 받은 문자를 실제로 해석 시작
{
    unsigned int detected = 0;
    unsigned int count = 0;

    if (line == NULL)
    {
        return;
    }

    // line 문자열이 "@DET,숫자,숫자" 형식이면 detected, count로 해석
    if (sscanf(line, "@DET,%u,%u", &detected, &count) == 2)
    {
        if (detected > 1)
        {
            detected = 1;
        }

        if (detected == 0)
        {
            count = 0;
        }

        if (count > 255)
        {
            count = 255;
        }

        n6_detected = (uint8_t)detected;
        n6_person_count = (uint8_t)count;
        n6_last_rx_tick = HAL_GetTick();
    }
}

// 일정 시간 동안 N6 감지 데이터가 안 들어오면 자동으로 0으로 초기화
void N6_RCV_TimeoutCheck(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - n6_last_rx_tick) > N6_DET_TIMEOUT_MS)
    {
        if ((n6_detected != 0) || (n6_person_count != 0))
        {
            n6_detected = 0;
            n6_person_count = 0;
        }
    }
}

uint8_t N6_RCV_GetDetected(void)
{
    return n6_detected;
}

uint8_t N6_RCV_GetPersonCount(void)
{
    return n6_person_count;
}

uint32_t N6_RCV_GetLastRxTick(void)
{
    return n6_last_rx_tick;
}

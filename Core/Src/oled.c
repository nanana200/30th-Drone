#include "oled.h"
#include "debug.h"
#include "oled_font.h"
#include "sensor.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c3;

#define OLED_ADDR 0x78U
#define OLED_PAGE_COUNT 8U
#define OLED_TEXT_COLUMNS 16U
#define OLED_COMMAND_TIMEOUT_MS 20U

typedef enum
{
    OLED_DMA_IDLE = 0,
    OLED_DMA_CMD,
    OLED_DMA_DATA
} oled_dma_stage_t;

static uint8_t draw_buffer[1024];
static uint8_t frame_buffer[1024];
static uint8_t dma_tx_buffer[160] __attribute__((aligned(32), section(".dma_buffer")));
static char oled_text_rows[OLED_PAGE_COUNT][OLED_TEXT_COLUMNS + 1U];
static uint8_t oled_available = 1U;
static volatile uint8_t oled_dma_busy = 0U;
volatile uint8_t oled_update_pending = 0U;
volatile uint8_t oled_dma_page = 0U;
volatile oled_dma_stage_t oled_dma_stage = OLED_DMA_IDLE;

static uint32_t OLED_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void OLED_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void OLED_Cmd(uint8_t cmd);
static void OLED_ClearTextRow(uint8_t row, uint8_t col);
static void OLED_MirrorToUart(void);
static void OLED_StartDmaFrame(void);
static void OLED_StartPageCommand(uint8_t page);
static void OLED_StartPageData(uint8_t page);

uint8_t OLED_IsBusy(void)
{
    return oled_dma_busy;
}

static void OLED_Cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00U, cmd};

    if (oled_available == 0U)
    {
        return;
    }

    if (HAL_I2C_Master_Transmit(&hi2c3, OLED_ADDR, data, 2U, OLED_COMMAND_TIMEOUT_MS) != HAL_OK)
    {
        oled_available = 0U;
    }
}

void OLED_Init(void)
{
    oled_available = 1U;
    HAL_Delay(100);

    OLED_Cmd(0xAE);
    OLED_Cmd(0x20);
    OLED_Cmd(0x02);
    OLED_Cmd(0xB0);
    OLED_Cmd(0xC8);
    OLED_Cmd(0x00);
    OLED_Cmd(0x10);
    OLED_Cmd(0x40);
    OLED_Cmd(0x81);
    OLED_Cmd(0x7F);
    OLED_Cmd(0xA1);
    OLED_Cmd(0xA6);
    OLED_Cmd(0xA8);
    OLED_Cmd(0x3F);
    OLED_Cmd(0xD3);
    OLED_Cmd(0x00);
    OLED_Cmd(0xD5);
    OLED_Cmd(0x80);
    OLED_Cmd(0xD9);
    OLED_Cmd(0xF1);
    OLED_Cmd(0xDA);
    OLED_Cmd(0x12);
    OLED_Cmd(0xDB);
    OLED_Cmd(0x40);
    OLED_Cmd(0x8D);
    OLED_Cmd(0x14);
    OLED_Cmd(0xAF);

    OLED_Clear();
    OLED_Update();

    while (OLED_IsBusy() != 0U)
    {
    }
}

void OLED_Clear(void)
{
    uint8_t row;

    memset(draw_buffer, 0, sizeof(draw_buffer));
    for (row = 0U; row < OLED_PAGE_COUNT; row++)
    {
        memset(oled_text_rows[row], ' ', OLED_TEXT_COLUMNS);
        oled_text_rows[row][OLED_TEXT_COLUMNS] = '\0';
    }
}

void OLED_Update(void)
{
    uint32_t primask;

    OLED_MirrorToUart();
    primask = OLED_EnterCritical();

    if (oled_available == 0U)
    {
        OLED_ExitCritical(primask);
        return;
    }

    if (oled_dma_busy != 0U)
    {
        oled_update_pending = 1U;
        OLED_ExitCritical(primask);
        return;
    }

    OLED_ExitCritical(primask);

    OLED_StartDmaFrame();
}

void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    if (color != 0U)
    {
        draw_buffer[x + (uint16_t)(y / 8U) * OLED_WIDTH] |= (uint8_t)(1U << (y % 8U));
    }
    else
    {
        draw_buffer[x + (uint16_t)(y / 8U) * OLED_WIDTH] &= (uint8_t)~(1U << (y % 8U));
    }
}

static void OLED_ClearTextRow(uint8_t row, uint8_t col)
{
    uint8_t x = (uint8_t)(col * 7U);

    if ((row >= OLED_PAGE_COUNT) || (x >= OLED_WIDTH))
    {
        return;
    }

    memset(&draw_buffer[(uint16_t)row * OLED_WIDTH + x], 0, OLED_WIDTH - x);
    if (col < OLED_TEXT_COLUMNS)
    {
        memset(&oled_text_rows[row][col], ' ', OLED_TEXT_COLUMNS - col);
    }
}

void OLED_Print(uint8_t row, uint8_t col, const char *str)
{
    uint8_t x = (uint8_t)(col * 7U);
    uint8_t y = (uint8_t)(row * 8U);
    uint8_t text_col = col;

    if ((str == NULL) || (row >= OLED_PAGE_COUNT))
    {
        return;
    }

    OLED_ClearTextRow(row, col);

    while (*str != '\0')
    {
        char c = *str;
        int i;

        if ((c < 32) || (c > 127))
        {
            c = '?';
        }

        if (text_col < OLED_TEXT_COLUMNS)
        {
            oled_text_rows[row][text_col] = c;
            text_col++;
        }

        for (i = 0; i < 7; i++)
        {
            uint8_t line = font5x7[(uint8_t)c - 32U][i];
            int j;

            for (j = 0; j < 8; j++)
            {
                if ((line & (uint8_t)(1U << (7 - j))) != 0U)
                {
                    OLED_DrawPixel((uint8_t)(x + j), (uint8_t)(y + i), 1U);
                }
            }
        }

        x = (uint8_t)(x + 8U);
        str++;
    }
}

void OLED_Printf(uint8_t row, uint8_t col, const char *fmt, ...)
{
    char buf[64];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    OLED_Print(row, col, buf);
}

static void OLED_MirrorToUart(void)
{
    char tx_buffer[160];
    size_t offset = 0U;
    uint8_t row;

    offset += (size_t)snprintf(tx_buffer, sizeof(tx_buffer), "[OLED]\r\n");
    for (row = 0U; row < OLED_PAGE_COUNT; row++)
    {
        if ((offset + OLED_TEXT_COLUMNS + 2U) >= sizeof(tx_buffer))
        {
            break;
        }

        memcpy(&tx_buffer[offset], oled_text_rows[row], OLED_TEXT_COLUMNS);
        offset += OLED_TEXT_COLUMNS;
        tx_buffer[offset++] = '\r';
        tx_buffer[offset++] = '\n';
    }
    tx_buffer[offset] = '\0';

    (void)uart1_printf("%s", tx_buffer);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C3)
    {
        return;
    }

    if (oled_dma_stage == OLED_DMA_CMD)
    {
        OLED_StartPageData(oled_dma_page);
        return;
    }

    if (oled_dma_stage == OLED_DMA_DATA)
    {
        uint32_t primask;

        oled_dma_page++;

        if (oled_dma_page < OLED_PAGE_COUNT)
        {
            OLED_StartPageCommand(oled_dma_page);
            return;
        }

        oled_dma_stage = OLED_DMA_IDLE;
        primask = OLED_EnterCritical();
        oled_dma_busy = 0U;

        if (oled_update_pending != 0U)
        {
            oled_update_pending = 0U;
            OLED_ExitCritical(primask);
            OLED_StartDmaFrame();
            return;
        }

        OLED_ExitCritical(primask);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C3)
    {
        return;
    }

    oled_dma_stage = OLED_DMA_IDLE;
    oled_dma_busy = 0U;
    oled_available = 0U;
}

static void OLED_StartDmaFrame(void)
{
    uint32_t primask = OLED_EnterCritical();

    memcpy(frame_buffer, draw_buffer, sizeof(frame_buffer));
    oled_update_pending = 0U;
    oled_dma_busy = 1U;
    oled_dma_page = 0U;
    OLED_ExitCritical(primask);
    OLED_StartPageCommand(0U);
}

static void OLED_StartPageCommand(uint8_t page)
{
    dma_tx_buffer[0] = 0x00U;
    dma_tx_buffer[1] = (uint8_t)(0xB0U + page);
    dma_tx_buffer[2] = 0x00U;
    dma_tx_buffer[3] = 0x10U;
    //SCB_CleanDCache_by_Addr((uint32_t *)dma_tx_buffer, 32);
    oled_dma_stage = OLED_DMA_CMD;

    if (HAL_I2C_Master_Transmit_DMA(&hi2c3, OLED_ADDR, dma_tx_buffer, 4U) != HAL_OK)
    {
        oled_dma_stage = OLED_DMA_IDLE;
        oled_dma_busy = 0U;
        oled_available = 0U;
    }
}

static void OLED_StartPageData(uint8_t page)
{
    dma_tx_buffer[0] = 0x40U;
    memcpy(&dma_tx_buffer[1], &frame_buffer[page * OLED_WIDTH], OLED_WIDTH);
    //SCB_CleanDCache_by_Addr((uint32_t *)dma_tx_buffer, 160);
    oled_dma_stage = OLED_DMA_DATA;

    if (HAL_I2C_Master_Transmit_DMA(&hi2c3, OLED_ADDR, dma_tx_buffer, 129U) != HAL_OK)
    {
        oled_dma_stage = OLED_DMA_IDLE;
        oled_dma_busy = 0U;
        oled_available = 0U;
    }
}

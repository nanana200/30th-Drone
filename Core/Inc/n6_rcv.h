#ifndef INC_N6_RCV_H_
#define INC_N6_RCV_H_

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define N6_RCV_LINE_BUF_SIZE     64
#define N6_DET_TIMEOUT_MS        3000

void N6_RCV_Init(UART_HandleTypeDef *huart);
void N6_RCV_RxCpltCallback(UART_HandleTypeDef *huart);
void N6_RCV_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void N6_RCV_Process(void);
void N6_RCV_TimeoutCheck(void);

uint8_t N6_RCV_GetDetected(void);
uint8_t N6_RCV_GetPersonCount(void);
uint32_t N6_RCV_GetLastRxTick(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_N6_RCV_H_ */

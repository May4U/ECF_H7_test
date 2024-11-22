#ifndef __REAL_MAIN_HPP
#define __REAL_MAIN_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* __REAL_MAIN_HPP */
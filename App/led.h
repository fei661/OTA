#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"

/* ==================== LED 编号定义 ====================
 *  LED0 → PF9   （正点原子探索者，低电平点亮）
 *  LED1 → PF10  （正点原子探索者，低电平点亮）
 * ==================================================== */
#define LED0        0
#define LED1        1

void LED_Init(void);
void LED_On(uint8_t n);
void LED_Off(uint8_t n);
void LED_Toggle(uint8_t n);
void LED_Set(uint8_t n, uint8_t On);

#endif
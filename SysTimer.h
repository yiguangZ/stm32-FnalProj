/*
 * ECE 153B
 *
 * Name(s):
 * Section:
 * Project
 */

#ifndef __STM32L476G_NUCLEO_SYSTICK_H
#define __STM32L476G_NUCLEO_SYSTICK_H
#include <stdbool.h>
#include "stm32l476xx.h"

void SysTick_Init(void);
void SysTick_Handler(void);
void delay(uint32_t ms);
bool isManual(void);
void startManual(void);
#endif

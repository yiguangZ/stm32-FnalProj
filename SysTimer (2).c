/*
 * ECE 153B
 *
 * Name(s):
 * Section:
 * Project
 */

#include "SysTimer.h"
#include "motor.h"
#include <stdbool.h>
static uint32_t volatile step;
static uint32_t volatile motor_step;
static uint32_t volatile manual_step;
static bool volatile manual = false;

void SysTick_Init(void) {
	// SysTick Control & Status Register
	SysTick->CTRL = 0; // Disable SysTick IRQ and SysTick Counter

	SysTick->LOAD = 79999;
	SysTick->VAL = 0;
	
	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	// 0 = counting down to zero does not assert the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; 
	// Select clock source
	// If CLKSOURCE = 0, the external clock is used. The frequency of SysTick clock is the frequency of the AHB clock divided by 8.
	// If CLKSOURCE = 1, the processor clock is used.
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;		
	
	// Enable SysTick IRQ and SysTick Timer
	// Configure and Enable SysTick interrupt in NVIC
	NVIC_EnableIRQ(SysTick_IRQn);
	NVIC_SetPriority(SysTick_IRQn, 1); // Set Priority to 1
}

void SysTick_Handler(void) {
	step++;

	motor_step++;

	if (manual) {
		manual_step++;
	}

	if (motor_step >= 5) {
		rotate();
		motor_step = 0;
	}

	if (manual_step >= 3000) {
		manual_step = 0;
		manual = false;
	}
}

bool isManual(void) {
	return manual;
}

void startManual(void) {
	manual = true;
	manual_step = 0;
}

void delay(uint32_t ms) {
	// Reset the counter
	step = 0;

	// Reset the value in the SysTick Current Value Register
	SysTick->VAL = 0;

	// Set the load value
	SysTick->LOAD = 79999; // 1 ms

	// Enable the counter
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

	// Wait until the counter reaches the desired value
	while (step < ms);

	// Disable the counter
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

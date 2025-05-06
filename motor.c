/*
 * ECE 153B
 *
 * Name(s):
 * Section:
 * Project
 */

#include "stm32l476xx.h"
#include "motor.h"

static const uint32_t MASK = 0x00000360; // GPIO pins 5, 6, 8, 9
static const uint32_t HalfStep[8] = {
    0x00000220, // 1001 (Pins 5, 9)
    0x00000020, // 1000 (Pin 5)
    0x00000120, // 1010 (Pins 5, 8)
    0x00000100, // 0010 (Pin 8)
    0x00000140, // 0110 (Pins 6, 8)
    0x00000040, // 0100 (Pin 6)
    0x00000240, // 0101 (Pins 6, 9)
    0x00000200  // 0001 (Pin 9)
};
static volatile int8_t dire = 0;
static volatile uint8_t step = 0;

void Motor_Init(void) {	
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	GPIOC->MODER &= ~(3UL<<10);
	GPIOC->MODER |= 1UL<<10;
	GPIOC->MODER &= ~(3UL<<12);
	GPIOC->MODER |= 1UL<<12;
	GPIOC->MODER &= ~(3UL<<16);
	GPIOC->MODER |= 1UL<<16;
	GPIOC->MODER &= ~(3UL<<18);
	GPIOC->MODER |= 1UL<<18;
	GPIOC->OSPEEDR &= ~(3UL<<10);
	GPIOC->OSPEEDR |= 2UL<<10;
	GPIOC->OSPEEDR &= ~(3UL<<12);
	GPIOC->OSPEEDR |= 2UL<<12;
	GPIOC->OSPEEDR &= ~(3UL<<16);
	GPIOC->OSPEEDR |= 2UL<<16;
	GPIOC->OSPEEDR &= ~(3UL<<18);
	GPIOC->OSPEEDR |= 2UL<<18;
	GPIOC->OTYPER &= ~(1UL<<5);
	GPIOC->OTYPER &= ~(1UL<<6);
	GPIOC->OTYPER &= ~(1UL<<8);	
	GPIOC->OTYPER &= ~(1UL<<9);
	GPIOC->PUPDR &= ~(3UL<<10);
	GPIOC->PUPDR &= ~(3UL<<12);
	GPIOC->PUPDR &= ~(3UL<<16);
	GPIOC->PUPDR &= ~(3UL<<18);
}

void rotate(void) {
    step += dire; // Update step index based on direction
    if (step < 0) {
        step = 7; // Wrap around to the last step
    } else if (step > 7) {
        step = 0; // Wrap around to the first step
    }

    GPIOC->ODR &= ~MASK;          // Clear motor bits
    GPIOC->ODR |= HalfStep[step]; // Set motor bits for current step
}
void setDire(int8_t direction) {
	//TODO
	dire = direction;

}



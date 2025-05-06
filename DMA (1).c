/*
 * ECE 153B
 *
 * Name(s):
 * Section:
 * Project
 */
 
#include "DMA.h"
#include "SysTimer.h"

void DMA_Init_UARTx(DMA_Channel_TypeDef * tx, USART_TypeDef * uart) {
    // Enable clock for DMA1
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Wait until DMA1 clock is ready
    while (!(RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN));

    // Disable the DMA channel before config
    tx->CCR &= ~DMA_CCR_EN;

    // Set the peripheral address to USART data register
    tx->CPAR = (uint32_t) &(uart->TDR);

    // Configure the direction as memory-to-peripheral
    tx->CCR |= DMA_CCR_DIR;

    // Disable memory-to-memory mode
    tx->CCR &= ~DMA_CCR_MEM2MEM;

    // Configure memory size to 8 bits
    tx->CCR &= ~DMA_CCR_MSIZE;

    // Configure peripheral size to 8 bits
    tx->CCR &= ~DMA_CCR_PSIZE;

    // Enable memory increment
    tx->CCR |= DMA_CCR_MINC;

    // Disable peripheral increment
    tx->CCR &= ~DMA_CCR_PINC;

    // Set the prio to high
    tx->CCR |= DMA_CCR_PL_1;

    // Da circular mode
    tx->CCR &= ~DMA_CCR_CIRC;

    // Enable transfer complete interrupt
    tx->CCR |= DMA_CCR_TCIE;

    // Disable half-transfer interrupt
    tx->CCR &= ~DMA_CCR_HTIE;

    // Disable transfer error interrupt
    tx->CCR &= ~DMA_CCR_TEIE;

    // Select the DMA request mapping for USART (DMA1 Channel 4 and 7)
    DMA1_CSELR->CSELR &= ~DMA_CSELR_C4S;
    DMA1_CSELR->CSELR &= ~DMA_CSELR_C7S;
    DMA1_CSELR->CSELR |= 0x02002000;

    // Enable the DMA interrupt in NVIC for Channel 4 and Channel 7
    NVIC_EnableIRQ(DMA1_Channel4_IRQn); // For USART1
    NVIC_EnableIRQ(DMA1_Channel7_IRQn); // For USART2
}

void DMA1_Channel7_IRQHandler(void) {   
    // Check Transfer Complete interrupt flag
    NVIC_ClearPendingIRQ(DMA1_Channel7_IRQn);
    // Clear the Transfer Complete interrupt flag
    DMA1->IFCR |= DMA_IFCR_CTCIF7;    
    // Clear global DMA interrupt flag 
    DMA1->IFCR |= DMA_IFCR_CGIF7;
    // Disable the DMA channel
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;
}
void DMA1_Channel4_IRQHandler(void) {   
    // Check Transfer Complete interrupt flag
    NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);
    // Clear the Transfer Complete interrupt flag
    DMA1->IFCR |= DMA_IFCR_CTCIF4;    
    // Clear global DMA interrupt flag 
    DMA1->IFCR |= DMA_IFCR_CGIF4;
    // Disable the DMA channel
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
}

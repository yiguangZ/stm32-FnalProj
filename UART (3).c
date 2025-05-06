/*
 * ECE 153B
 *
 * Name(s):
 * Section:
 * Project
 */


#include "UART.h"
#include "DMA.h"

static volatile DMA_Channel_TypeDef * tx;
static volatile char inputs[IO_SIZE];
static volatile uint8_t data_t_0[IO_SIZE];
static volatile uint8_t data_t_1[IO_SIZE];
static volatile uint8_t input_size = 0;
static volatile uint8_t pending_size = 0;
static volatile uint8_t * active = data_t_0;
static volatile uint8_t * pending = data_t_1;

#define SEL_0 1
#define BUF_0_EMPTY 2
#define BUF_1_EMPTY 4
#define BUF_0_PENDING 8
#define BUF_1_PENDING 16

void transfer_data(char ch);
void on_complete_transfer(void);


void UART1_Init(void) {
	UART1_GPIO_Init();
	USART_Init(USART1);

	// Enable usart1 Clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  	RCC->CCIPR &= ~(RCC_CCIPR_USART1SEL); // Clear USART1 clock source selection
   	RCC->CCIPR |= RCC_CCIPR_USART1SEL_0;  // Set system clock as source for USART1}
	// Enable DMA for USART1
	DMA_Init_UARTx(DMA1_Channel4, USART1);

	// Set DMA channel to USART1
	tx = DMA1_Channel4;

	// Set DMA memory address
	tx->CMAR = (volatile uint32_t) active;

	// Enable NVIC interrupt
	NVIC_EnableIRQ(USART1_IRQn);
}

void UART2_Init(void) {
	USART_Init(USART2);
	NVIC_EnableIRQ(USART2_IRQn);

	// Enable USART2 Clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

	RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL); //cleart USART2 clock source selection
	RCC->CCIPR |= RCC_CCIPR_USART2SEL_0; //set system clock as source for USART2
	// Enable DMA for USART2
	DMA_Init_UARTx(DMA1_Channel7, USART2);


	// Set DMA channel to USART2
	tx = DMA1_Channel7;

	// Set DMA memory 
	tx->CMAR = (volatile uint32_t) active;

	// Enable NVIC interrupt
	NVIC_EnableIRQ(USART2_IRQn);

}


void UART1_GPIO_Init(void) {
	 RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;  // Enable GPIO clock for Port B

    // Set PB6 and PB7 to alternate function mode for USART1 (AF7)
    GPIOB->MODER &= ~(3UL << (12)); // Clear PB6 mode
    GPIOB->MODER |= (2UL << (12));  // Set PB6 to AF mode
    GPIOB->MODER &= ~(3UL << (14)); // Clear PB7 mode
    GPIOB->MODER |= (2UL << (14));  // Set PB7 to AF mode

    // Set alternate function for PB6 and PB7 to USART1 (AF7)
    GPIOB->AFR[0] |= (7UL << (24)); // Set PB6 AF to AF7
    GPIOB->AFR[0] |= (7UL << (28)); // Set PB7 AF to AF7

   //Set I/O output speed value as very high speed for PB6 PB7
	GPIOB->OSPEEDR &= ~(3UL<<12);
	GPIOB->OSPEEDR |= 3UL<<12;
	GPIOB->OSPEEDR &= ~(3UL<<14);
	GPIOB->OSPEEDR |= 3UL<<14;
	// GPIO Pull up
	GPIOB->PUPDR &= ~(3UL<<12);
	GPIOB->PUPDR &= ~(3UL<<14);
	GPIOB->PUPDR |= (1UL<<12);
	GPIOB->PUPDR &= (1UL<<14);
	//GPIO Push pull
	GPIOB->OTYPER &= ~(1UL<<6);
	GPIOB->OTYPER &= ~(1UL<<7);
}

void UART2_GPIO_Init(void) {
	// Enable GPIO Clock for Port A
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	 // Set PA2 and PA3 to alternate function mode (AF7 for USART2)
    GPIOA->MODER &= ~(3UL << (4));  // Clear PA2 mode
    GPIOA->MODER |= (2UL << (4));   // Set PA2 to AF mode
    GPIOA->MODER &= ~(3UL << (6));  // Clear PA3 mode
    GPIOA->MODER |= (2UL << (6));   // Set PA3 to AF mode

    // Set alternate function for PA2 and PA3 to USART2 (AF7)
    GPIOA->AFR[0] |= (7UL << 8);  // Set PA2 AF to AF7
    GPIOA->AFR[0] |= (7UL << 12);  // Set PA3 AF to AF7
	//Set I/O output speed value as very high speed for PA2 PA3
	GPIOA->OSPEEDR &= ~(3UL<<4);
	GPIOA->OSPEEDR |= 3UL<<4;
	GPIOA->OSPEEDR &= ~(3UL<<6);
	GPIOA->OSPEEDR |= 3UL<<6;
	// GPIO Pull up
	GPIOA->PUPDR &= ~(3UL<<4);
	GPIOA->PUPDR &= ~(3UL<<6);
	GPIOA->PUPDR |= (1UL<<4);
	GPIOA->PUPDR &= (1UL<<6);
	//GPIO Push pull
	GPIOA->OTYPER &= ~(1UL<<2);
	GPIOA->OTYPER &= ~(1UL<<3);

}
void USART_Init(USART_TypeDef* USARTx) {
	
  USARTx->CR1 &= ~USART_CR1_UE;
  USARTx->CR1 &= ~USART_CR1_M;   // Set word length to 8 bits
  USARTx->CR1 &= ~USART_CR1_OVER8; // Set oversampling mode to 16
  USARTx->CR2 &= ~USART_CR2_STOP;  // Set number of stop bits to 1
  // Calculate and set baud rate to 9600
  uint32_t USARTDIV = 80000000 / 9600;
  USARTx->BRR = USARTDIV;

	// Enable DMA mode for transmission
	USARTx->CR3 |= USART_CR3_DMAT;

    USARTx->CR1 |= USART_CR1_TE | USART_CR1_RE;  // Enable transmitter and receiver

	// Enable the transmission complete interrupt
	USARTx->CR1 |= USART_CR1_TCIE;

	// Enable the receive dat
	USARTx->CR1 |= USART_CR1_RXNEIE;

	// Disable the idle interrupt
	USARTx->CR1 &= ~USART_CR1_IDLEIE;

	// Disable the transmit data register empty
	USARTx->CR1 &= ~USART_CR1_TXEIE;

	// Enable USART
	USARTx->CR1 |= USART_CR1_UE;
}

/**
 * This function accepts a string that should be sent through UART
*/
void UART_print(char* data) {
	//Check DMA status. If DMA is ready, send data
	//If DMA is not ready, put the data aside
	if (tx->CCR & DMA_CCR_EN) {
		for (int i = 0; i < IO_SIZE; i++) {
			if (data[i] == '\0') {
				break;
			}
			pending[i] = data[i];
			pending_size++;
		}
	} else {
        //DMA ready
		uint8_t size = 0;
		for (int i = 0; i < IO_SIZE; i++) {
			if (data[i] == '\0') {
				break;
			}
			active[i] = data[i];
			size++;
		}
		tx->CCR &= ~DMA_CCR_EN;
		tx->CNDTR = size;
		tx->CMAR = (volatile uint32_t) active;
		tx->CCR |= DMA_CCR_EN;
	}
}

/**
 * This function should be invoked when a character is accepted through UART
*/
void transfer_data(char ch) {
	// Append character to input buffer.
	// If the character is end-of-line, invoke UART_onInput
	if (ch == '\n') {
		inputs[input_size] = '\0';
		UART_onInput(inputs, input_size);
		input_size = 0;
	}
	 else {
		inputs[input_size] = ch;
		input_size++;
	}

}

/**
 * This function should be invoked when DMA transaction is completed
*/
void on_complete_transfer(void) {
	// If there are pending data to send, switch active and pending buffer, and send data
	if (pending_size > 0) {
		volatile uint8_t * swap = active;
		active = pending;
		pending = swap;
		tx->CCR &= ~DMA_CCR_EN;
		tx->CNDTR = pending_size;
		tx->CMAR = (volatile uint32_t) active;
		tx->CCR |= DMA_CCR_EN;
		pending_size = 0;
	}
}

void USART1_IRQHandler(void) {
    // When receive a character, invoke transfer_data
    // When complete sending data, invoke on_complete_transfer
    tx = DMA1_Channel4;

    // Check if the interrupt was triggered by a transmission complete event
    if (USART1->ISR & USART_ISR_TC) {
        // Clear the transmission complete flag
        USART1->ICR |= USART_ICR_TCCF;

        // If the DMA is finished, invoke on_complete_transfer
        if (!(tx->CCR & DMA_CCR_EN)) {
            on_complete_transfer();
        }
    } 
    // Check if the interrupt was triggered by a received character
    else if (USART1->ISR & USART_ISR_RXNE) {
        transfer_data(USART1->RDR);
    }
}

void USART2_IRQHandler(void) {
    // When receive a character, invoke transfer_data
    // When complete sending data, invoke on_complete_transfer
    tx = DMA1_Channel7;

    // Check if the interrupt was triggered by a transmission complete event
    if (USART2->ISR & USART_ISR_TC) {
        // Clear the transmission complete flag
        USART2->ICR |= USART_ICR_TCCF;

        // If the DMA is finished, invoke on_complete_transfer
        if (!(tx->CCR & DMA_CCR_EN)) {
            on_complete_transfer();
        }
    }
    // Check if the interrupt was triggered by a received character
    else if (USART2->ISR & USART_ISR_RXNE) {
        transfer_data(USART2->RDR);
    }
}

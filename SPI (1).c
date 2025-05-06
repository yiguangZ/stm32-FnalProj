#include "SPI.h"
#include "SysTimer.h"

void SPI1_GPIO_Init(void) {
    // Enable GPIO clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // Set PA4 to Alternate Function, Very High Speed, Push-Pull, No Pull-Up/Down
    GPIOA->MODER &= ~(3UL << 8);
    GPIOA->MODER |= (2UL << 8);
    GPIOA->AFR[0] |= (5UL << 16);
    GPIOA->OSPEEDR |= (3UL << 8);
    GPIOA->OTYPER &= ~(1UL << 4);
    GPIOA->PUPDR &= ~(3UL << 8);

    // Set PB3 (SPI1 SCK), PB4 (SPI1 MISO), PB5 (SPI1 MOSI) to Alternate Function, Very High Speed, Push-Pull, No Pull-Up/Down
    GPIOB->MODER &= ~(3UL << 6);
    GPIOB->MODER |= (2UL << 6);
    GPIOB->AFR[0] |= (5UL << 12);
    GPIOB->OSPEEDR |= (3UL << 6);
    GPIOB->OTYPER &= ~(1UL << 3);
    GPIOB->PUPDR &= ~(3UL << 6);

    GPIOB->MODER &= ~(3UL << 8);
    GPIOB->MODER |= (2UL << 8);
    GPIOB->AFR[0] |= (5UL << 16);
    GPIOB->OSPEEDR |= (3UL << 8);
    GPIOB->OTYPER &= ~(1UL << 4);
    GPIOB->PUPDR &= ~(3UL << 8);

    GPIOB->MODER &= ~(3UL << 10);
    GPIOB->MODER |= (2UL << 10);
    GPIOB->AFR[0] |= (5UL << 20);
    GPIOB->OSPEEDR |= (3UL << 10);
    GPIOB->OTYPER &= ~(1UL << 5);
    GPIOB->PUPDR &= ~(3UL << 10);
}

void SPI1_Init(void){
	// Enable SPI clock and Reset SPI
	 RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	 RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
   RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
	// Disable SPI
	SPI1->CR1 &= ~SPI_CR1_SPE;

	// Configure for Full Duplex Communication
  SPI1->CR1 &= ~SPI_CR1_RXONLY;
	// Configure for 2-line Unidirectional Data Mode
 	SPI1->CR1 &= ~SPI_CR1_BIDIMODE;
	// Disable Output in Bidirectional Mode
  SPI1->CR1 &= ~SPI_CR1_BIDIOE;

	// Set Frame Format: MSB First, 16-bit, Motorola Mode
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;      // MSB first
  SPI1->CR2 |= SPI_CR2_DS;           // 16-bit data frame format
  SPI1->CR2 &= ~SPI_CR2_FRF;  //motorola mode

	// Configure Clock. Read DataSheet for required value
	SPI1->CR1 |= SPI_CR1_CPOL;           // CPOL = 1
  SPI1->CR1 |= SPI_CR1_CPHA;           // CPHA = 1
	

  // Set baud rate to 16
  SPI1->CR1 |= SPI_CR1_BR;
  SPI1->CR1 &= ~SPI_CR1_BR_2;

	// Disable Hardware CRC Calculation
	SPI1->CR1 &= ~SPI_CR1_CRCEN;

	// Set as Master
	SPI1->CR1 |= SPI_CR1_MSTR;
	// Disable Software Slave Management
	SPI1->CR1 &= ~SPI_CR1_SSM; 

	// Enable NSS Pulse Management
  SPI1->CR2 |= SPI_CR2_NSSP;
	

  // Enable output for SPI_GPIO_Init
  SPI1->CR2 |= SPI_CR2_SSOE;

	// Set FIFO Reception Threshold to 1/2
  SPI1->CR2 &= ~(SPI_CR2_FRXTH); 
	// Enable SPI
	SPI1->CR1 |= SPI_CR1_SPE; 
}

uint16_t SPI_Transfer_Data(uint16_t write_data) {
    while (!(SPI1->SR & SPI_SR_TXE));  // Wait until TXE ready
    SPI1->DR = write_data;
    while (SPI1->SR & SPI_SR_BSY);
    while (!(SPI1->SR & SPI_SR_RXNE));  // Wait for RXNE
    return SPI1->DR;  // Read data
}
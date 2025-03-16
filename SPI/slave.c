#include "stm32l0xx.h"

#define NSS_HIGH GPIOA->BSRR = GPIO_BSRR_BS_0
#define NSS_LOW GPIOA->BSRR = GPIO_BSRR_BR_0

void init_SPI1(void);
static inline uint8_t SPI1_read(void);
static inline void SPI1_write(uint16_t data);

uint32_t spi_errors = 0;

void init_SPI1(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;      // Enable SPI1 clock
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;       // Enable GPIOA clock
    
    // Configure SPI1 Pins (PA5, PA6, PA7 for SCK, MISO, MOSI)
    GPIOA->MODER &= ~(GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0); 
    GPIOA->MODER |= (GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);  // Set SCK, MISO, and MOSI as Alternate Function
    GPIOA->AFR[0] |= (5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL6_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos);  // Set alternate function for SPI1

    // Configure Chip Select (PA0)
    GPIOA->MODER &= ~GPIO_MODER_MODE0;        // Clear mode for PA0
    GPIOA->MODER |= GPIO_MODER_MODE0_0;       // Set PA0 as output (CS)

    // SPI1 settings (Slave Mode)
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1;  // Master mode, software slave management, baud rate (divided by 4)
    SPI1->CR1 &= ~SPI_CR1_MSTR; // Set SPI to Slave mode
    SPI1->CR1 |= SPI_CR1_SPE;  // Enable SPI1
}

static inline uint8_t SPI1_read(void) {
    while (!(SPI1->SR & SPI_SR_RXNE));  // Wait until RXNE (Receive buffer not empty)
    return SPI1->DR;                    // Read data from SPI data register
}

static inline void SPI1_write(uint16_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));  // Wait until TXE (Transmit buffer empty)
    SPI1->DR = data;                   // Write data to SPI data register
}

int main(void) {
    init_SPI1();
    
    while (1) {
        if (SPI1->SR & SPI_SR_RXNE) {  // Check if data is received
            uint8_t received_data = SPI1_read();  // Read received data
            SPI1_write(received_data + 1);  // Send incremented value back to master
        }
    }
}

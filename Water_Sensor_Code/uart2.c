#include "uart2.h"
#include "stm32l0xx.h"

#define UART_BAUD_RATE 9600

void init_USART2(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;      // Enable GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // Enable USART2 clock

    // Set PA9 as USART2_TX and PA10 as USART2_RX
    GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
    GPIOA->MODER |= (GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);  // Alternate function mode
    GPIOA->AFR[1] |= (0x4 << (4 * 1)) | (0x4 << (4 * 2));       // AF4 for PA9 and PA10

    USART2->BRR = (SystemCoreClock / UART_BAUD_RATE);   // Set baud rate
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;  // Enable USART, TX, and RX
}

void UART_send(uint8_t data) {
    while (!(USART2->ISR & USART_ISR_TXE)) {
        // Wait until TX is empty
    }
    USART2->TDR = data;
}
void UART_send_string(char *str) {
    while (*str) {
        UART_send(*str++);
    }
}
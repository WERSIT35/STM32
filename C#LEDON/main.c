#include "stm32l0xx.h"
#include "delay.h"

#define LED_PIN GPIOB->BSRR = GPIO_BSRR_BS_3
#define LED_PORT GPIOB->BSRR = GPIO_BSRR_BR_3

void SystemClock_Config(void);
void UART2_Init(void);
void GPIO_Init(void);
void USART2_IRQHandler(void);

volatile char received_command = 0;

int main(void) {
    SystemInit();
    SystemClock_Config();
    GPIO_Init();
    UART2_Init();

    while (1) {
        if (received_command == '1') {
            GPIOB->BSRR = GPIO_BSRR_BS_3; // Turn LED ON
            received_command = 0;
        } else if (received_command == '0') {
            GPIOB->BSRR = GPIO_BSRR_BR_3; // Turn LED OFF
            received_command = 0;
        }
    }
}

void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSION; // Enable HSI clock
    while (!(RCC->CR & RCC_CR_HSIRDY)); // Wait for HSI to be ready

    RCC->CFGR = 0; // Use HSI as the system clock
    RCC->CFGR |= RCC_CFGR_SW_HSI;
    while (!(RCC->CFGR & RCC_CFGR_SWS_HSI)); // Wait for HSI to be used as the system clock

    SystemCoreClockUpdate();
}

void GPIO_Init(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN; // Enable GPIOA clock

    // Configure LED pin (PA5) as output
    GPIOB->MODER &= ~GPIO_MODER_MODE3_1; // Output mode
    GPIOB->MODER |= GPIO_MODER_MODE3_0;

    // Set LED to OFF initially
    GPIOB->BSRR = GPIO_BSRR_BR_3;

    // Configure PA9 (TX) and PA10 (RX) for UART
    GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
    GPIOA->MODER |= (GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1); // Alternate function mode

    GPIOA->AFR[1] |= (4 << GPIO_AFRH_AFSEL9_Pos); // AF4 for PA9
    GPIOA->AFR[1] |= (4 << GPIO_AFRH_AFSEL10_Pos); // AF4 for PA10
}

void UART2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 clock

    USART2->BRR = SystemCoreClock / 9600; // Set baud rate to 9600
    USART2->CR1 |= USART_CR1_RE | USART_CR1_TE; // Enable RX and TX
    USART2->CR1 |= USART_CR1_RXNEIE; // Enable RX interrupt
    USART2->CR1 |= USART_CR1_UE; // Enable USART2

    NVIC_EnableIRQ(USART2_IRQn); // Enable USART2 interrupt in NVIC
}

void USART2_IRQHandler(void) {
    if (USART2->ISR & USART_ISR_RXNE) { // Check if data is received
        received_command = (char)USART2->RDR; // Read the received data
    }
}

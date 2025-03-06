#include "stm32l0xx.h"
#include "delay.h"

// LED Macros
#define LED4_ON   GPIOB->BSRR = GPIO_BSRR_BS_4   // LED4 on PB4
#define LED4_OFF  GPIOB->BSRR = GPIO_BSRR_BR_4  

#define LED5_ON   GPIOB->BSRR = GPIO_BSRR_BS_5   // LED5 on PB6
#define LED5_OFF  GPIOB->BSRR = GPIO_BSRR_BR_5  

#define LED9_ON   GPIOB->BSRR = GPIO_BSRR_BS_1   // LED9 on PB9
#define LED9_OFF  GPIOB->BSRR = GPIO_BSRR_BR_1  

#define LED11_ON  GPIOB->BSRR = GPIO_BSRR_BS_6  // LED11 on PB11
#define LED11_OFF GPIOB->BSRR = GPIO_BSRR_BR_6  

#define LED12_ON  GPIOB->BSRR = GPIO_BSRR_BS_7  // LED12 on PB12
#define LED12_OFF GPIOB->BSRR = GPIO_BSRR_BR_7  

// Button Pins (Inputs)
#define VT_PIN   (1U << 0)  // PA0 - VT Button Pin
#define D0_PIN   (1U << 1)  // PA1 - D0 Button Pin
#define D1_PIN   (1U << 3)  // PA3 - D1 Button Pin
#define D2_PIN   (1U << 4)  // PA4 - D2 Button Pin

void init_USART2(void);
void UART_send(uint8_t data);
void init_GPIO(void);
void check_button_signals(void);
void UART_send_string(char *str);
void delay_ms(uint32_t ms);  // Adding delay for debouncing

void SysTick_Handler(void) {
    delay_tick();
}

int main() {
    SystemInit();
    SysTick_Config(SystemCoreClock / 1000); // Set SysTick to 1ms

    init_USART2();
    init_GPIO();

    while (1) {
        check_button_signals();
        delay(100);  // Small delay for stability and debouncing
    }
}

void init_USART2(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;      // Enable GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   // Enable USART2 clock

    // Set PA9 as USART2_TX and PA10 as USART2_RX
    GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
    GPIOA->MODER |= (GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);  // Alternate function mode
    GPIOA->AFR[1] |= (0x4 << (4 * 1)) | (0x4 << (4 * 2));       // AF4 for PA9 and PA10

    USART2->BRR = (SystemCoreClock / 9600);   // Set baud rate (9600)
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;  // Enable USART, TX, and RX
}

void UART_send(uint8_t data) {
    while (!(USART2->ISR & USART_ISR_TXE)) { } // Wait until TX is empty
    USART2->TDR = data;
}

void UART_send_string(char *str) {
    while (*str) {
        UART_send(*str++);
    }
}

void init_GPIO(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;  // Enable GPIOA and GPIOB clocks

    // Configure Button Pins (PA0 - PA4 as inputs with pull-down resistors)
    GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4);
    GPIOA->PUPDR |= (GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD1_1 | GPIO_PUPDR_PUPD3_1 | GPIO_PUPDR_PUPD4_1);  // Enable pull-down resistors

    // Configure LED Pins (PB4, PB5, PB6, PB9, PB11, PB12 as outputs)
    GPIOB->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE1 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOB->MODER |= (GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0);  // Output mode
}

void check_button_signals(void) {
    // Check if VT (PA0) button is pressed
    if (GPIOA->IDR & VT_PIN) {
        UART_send_string("VT Button Pressed!\r\n");
        LED11_ON;  // LED11 on PB11
        LED4_ON;   // LED4 on PB4
        LED5_OFF;  // Ensure LED5 is off
        LED9_OFF;  // Ensure LED9 is off
        LED12_OFF; // Ensure LED12 is off
    } else {
        LED11_OFF; // Turn off LED11
        LED4_OFF;  // Turn off LED4
    }

    // Check if D0 (PA1) button is pressed
    if (GPIOA->IDR & D0_PIN) {
        UART_send_string("D0 Button Pressed!\r\n");
        LED9_ON;   // LED9 on PB9
        LED4_ON;   // LED4 on PB4
        LED11_OFF; // Ensure LED11 is off
        LED5_OFF;  // Ensure LED5 is off
        LED12_OFF; // Ensure LED12 is off
    } else {
        LED9_OFF;  // Turn off LED9
        LED4_OFF;  // Turn off LED4
    }

    // Check if D1 (PA3) button is pressed
    if (GPIOA->IDR & D1_PIN) {
        UART_send_string("D1 Button Pressed!\r\n");
        LED12_ON;  // LED12 on PB12
        LED4_ON;   // LED4 on PB4
        LED9_OFF;  // Ensure LED9 is off
        LED5_OFF;  // Ensure LED5 is off
        LED11_OFF; // Ensure LED11 is off
    } else {
        LED12_OFF; // Turn off LED12
        LED4_OFF;  // Turn off LED4
    }

    // Check if D2 (PA4) button is pressed
    if (GPIOA->IDR & D2_PIN) {
        UART_send_string("D2 Button Pressed!\r\n");
        LED5_ON;   // LED5 on PB6
        LED4_ON;   // LED4 on PB4
        LED9_OFF;  // Ensure LED9 is off
        LED12_OFF; // Ensure LED12 is off
        LED11_OFF; // Ensure LED11 is off
    } else {
        LED5_OFF;  // Turn off LED5
        LED4_OFF;  // Turn off LED4
    }
}

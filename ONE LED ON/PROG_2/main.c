#include "stm32l0xx.h"
#include "delay.h"

#define LED_ON GPIOA->BSRR = GPIO_BSRR_BS_1;
#define LED_OFF GPIOA->BSRR = GPIO_BSRR_BR_1;

void init_GPIO(void);

int main() {
    SystemInit(); // Initialize system clock
    init_GPIO();  // Initialize GPIO for LED

    while(1) {
        LED_ON;      // Turn on the LED
        delay(500);
        LED_OFF;     // Turn off the LED
        delay(500); // Delay loop
    }
}

void init_GPIO(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;  // Enable GPIOA clock

    GPIOA->MODER &= ~GPIO_MODER_MODE1;    // Clear mode bits for PA1
    GPIOA->MODER |= GPIO_MODER_MODE1_0;   // Set PA1 to output mode
}

#include "stm32l0xx.h"
#include "delay.h"
#include "uart2.h"

#define LED1_ON GPIOB->BSRR = GPIO_BSRR_BS_1;
#define LED1_OFF GPIOB->BSRR = GPIO_BSRR_BR_1;

#define WATER_SENSOR_PIN (1U << 0) // PA0 as Water Sensor Input Pin

void init_ADC(void);
uint16_t ADC_read(uint8_t ch);
void init_GPIO(void);
uint16_t ADC_data = 0;

void SysTick_Handler(void) {
    delay_tick();
}

int main() {
    SystemInit();
    SysTick_Config(SystemCoreClock / 1000); // Set SysTick to 1ms

    init_ADC();
    init_GPIO();
		init_USART2();  
	
    while(1) {
        uint8_t WaterStatus = (GPIOA->IDR & WATER_SENSOR_PIN) ? 0 : 1; // Invert logic

        if (WaterStatus) {
            LED1_ON;
            UART_send_string("Water Detected!\r\n");
        } else {
            LED1_OFF;
            UART_send_string("No Water.\r\n");
        }
        delay(1000);
    }
}



void init_ADC(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN; // Enable GPIOA clock for ADC
    GPIOA->MODER |= GPIO_MODER_MODE0;  // Set PA0 to analog mode for ADC

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  // Enable ADC1 clock

    ADC1->CFGR2 |= ADC_CFGR2_CKMODE;    // Set ADC clock mode
    ADC1->SMPR |= ADC_SMPR_SMP;         // Set sample time
    
    if ((ADC1->CR & ADC_CR_ADEN) != 0) {
        ADC1->CR |= ADC_CR_ADDIS;  // Disable ADC if already enabled
    }
    ADC1->CR |= ADC_CR_ADCAL; // Start ADC calibration
    while ((ADC1->ISR & ADC_ISR_EOCAL) == 0); // Wait for calibration to finish
    ADC1->ISR |= ADC_ISR_EOCAL;  // Clear calibration flag

    ADC1->CR |= ADC_CR_ADEN;  // Enable ADC
}

void init_GPIO(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;  // Enable GPIOA and GPIOB clocks

    // Configure PA0 as input (Water Sensor)
    GPIOA->MODER &= ~GPIO_MODER_MODE0;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;

    // Configure PB1 as output (LED1)
    GPIOB->MODER &= ~GPIO_MODER_MODE1;  // Clear bits (reset state)
    GPIOB->MODER |= GPIO_MODER_MODE1_0; // Set PB1 as output mode
}
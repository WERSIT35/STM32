#include "stm32l0xx.h"
#include "delay.h"

#define LED1_ON 	GPIOB->BSRR = GPIO_BSRR_BS_3
#define LED1_OFF	GPIOB->BSRR = GPIO_BSRR_BR_3

#define WATER_SENSOR_PIN (1U << 0)
#define UART_BAUD_RATE 9600

void init_ADC(void);
uint16_t ADC_read(uint8_t ch);
void ADC_select_channel(uint8_t ch);
void init_leds(void);
void init_USART2(void);
void UART_send(uint8_t data);

uint16_t ADC_data = 0;

void SysTick_Handler(void) {
    delay_tick();
}

int main() {
    SystemInit();
    SysTick_Config(SystemCoreClock / 1000);

    init_ADC();
    init_leds();
    init_USART2();

    GPIOA->MODER &= ~GPIO_MODER_MODE0;
    
    while (1) {
        uint8_t waterStatus = (GPIOA->IDR & WATER_SENSOR_PIN) ? 1 : 0;
        UART_send(waterStatus);
        
        if (waterStatus) {
            LED1_ON;
        } else {
            LED1_OFF;
        }

        delay(0);
    }
}

void init_USART2(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->MODER &= ~GPIO_MODER_MODE9;
    GPIOA->MODER |= GPIO_MODER_MODE9_1;

    GPIOA->MODER &= ~GPIO_MODER_MODE10;
    GPIOA->MODER |= GPIO_MODER_MODE10_1;

    GPIOA->AFR[1] |= (0x1 << (4 * 1));
    GPIOA->AFR[1] |= (0x1 << (4 * 2));

    USART2->BRR = (SystemCoreClock / UART_BAUD_RATE);

    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void UART_send(uint8_t data) {
    while (!(USART2->ISR & USART_ISR_TXE)) {
        // Wait for TX
    }
    USART2->TDR = data;
}

void init_ADC(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    GPIOA->MODER |= GPIO_MODER_MODE0;
    
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC1->CFGR2 |= ADC_CFGR2_CKMODE;
    ADC1->SMPR |= ADC_SMPR_SMP;    
    
    if ((ADC1->CR & ADC_CR_ADEN) != 0)
    {
        ADC1->CR |= ADC_CR_ADDIS; 
    }
    ADC1->CR |= ADC_CR_ADCAL; 
    while ((ADC1->ISR & ADC_ISR_EOCAL) == 0);
    ADC1->ISR |= ADC_ISR_EOCAL;

    ADC1->CR |= ADC_CR_ADEN;    
}

void init_leds(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
    GPIOB->MODER &= ~GPIO_MODER_MODE3_1;
    GPIOB->MODER |= GPIO_MODER_MODE3_0;

    LED1_OFF;
}

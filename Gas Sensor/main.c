#include "stm32l0xx.h"
#include "delay.h"

#define LED1_ON GPIOB -> BSRR = GPIO_BSRR_BS_1;
#define LED1_OFF GPIOB -> BSRR = GPIO_BSRR_BR_1;

#define GAS_SENSOR_PIN (1U << 0) //PA0 as Gas sensor Input Pin

#define UART_BAUD_RATE 9600

void init_ADC(void);
//Analog to Digital Converter

uint16_t ADC_read(uint8_t ch);//PA0 analog signal Digital data 1 / 0

void init_USART2(void);

void UART_send(uint8_t data);

void init_GPIO(void);

void UART_send_string(char *str);

uint16_t ADC_data = 0;

void SysTick_Handler(void) {
    delay_tick();
}

int main() {
    SystemInit();
    SysTick_Config(SystemCoreClock / 1000); // Set SysTick to 1ms

    init_ADC();
	
    init_USART2();
	
		init_GPIO();
    while(1){
			
        uint8_t GasStatus = (GPIOA->IDR & GAS_SENSOR_PIN) ? 1 : 0;
				
				if (GasStatus) {
					LED1_ON;
          UART_send_string("Gas Detected!\r\n");
        } else {
					LED1_OFF;
					UART_send_string("No Gas.\r\n");   
        }
				delay(1000);
    }
}
    

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

    // Configure PA0 as input (Gas Sensor)
    GPIOA->MODER &= ~GPIO_MODER_MODE0;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;

    // Configure PB1 as output (LED1)
    GPIOB->MODER &= ~GPIO_MODER_MODE1;  // Clear bits (reset state)
    GPIOB->MODER |= GPIO_MODER_MODE1_0; // Set PB1 as output mode
}
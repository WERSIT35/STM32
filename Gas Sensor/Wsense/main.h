// main.h
#ifndef MAIN_H
#define MAIN_H

#include "stm32l0xx.h"   // Device-specific header
#include "delay.h"       // Include custom delay functions

#define WATER_SENSOR_PORT GPIOA
#define WATER_SENSOR_PIN GPIO_PIN_0

void init_water_sensor(void);

#endif // MAIN_H

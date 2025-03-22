#include "delay.h"

uint32_t delay_time = 0;

inline void delay_tick(void){
    if(delay_time)
        --delay_time;
}

void delay(uint32_t ms){
    delay_time = ms;
    while(delay_time){};
}

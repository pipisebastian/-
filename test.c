#include "stm32f10x.h"

void delay(void)
{
    int i = 0;
    for (int i = 0; i < 10000000; i++)
    {
    }
}



int main(){
    *((volatile unsigned int *)0x40021018) |= 0x70; // clock enable port C, D, E

    *((volatile unsigned int *)0x40011000) &= 0xFF0000FF; // init port C    
    *((volatile unsigned int *)0x40011000) |= 0x00888800; // port C input mode

    *((volatile unsigned int *)0x40011404) &= 0xFFFF0FFF; // init port D - 스위치!!! (11 PIN reset)
    *((volatile unsigned int *)0x40011404) |= 0x00008000; // port D input mode

    *((volatile unsigned int *)0x40011800) &= 0xFFFFF00F; // init port E
    *((volatile unsigned int *)0x40011800) |= 0x00000330; // port E 1 output
  
  while(1){
    *((volatile unsigned int *)0x40011810) |= 0x00000002;
    }
}


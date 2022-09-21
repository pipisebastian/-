#include "stm32f10x.h"

void delay(void)
{
    int i = 0;
    for (int i = 0; i < 10000000; i++)
    {
    }
}

int main()
{
    // PC2(조이스틱 down) , PC5(조이스틱 up), PD11(스위치)
    *((volatile unsigned int *)0x40021018) |= 0x70; // clock enable port C, D

    *((volatile unsigned int *)0x40011000) &= 0xFF0000FF; // init port C
    *((volatile unsigned int *)0x40011000) |= 0x00888800; // port C input mode

    *((volatile unsigned int *)0x40011404) &= 0xFFFF0FFF; // init port D - 스위치!!! (11 PIN reset)
    *((volatile unsigned int *)0x40011404) |= 0x00008000; // port D input mode

    *((volatile unsigned int *)0x40011800) &= 0xFFFFF00F; // init port E -  (1,2 PIN reset)
    *((volatile unsigned int *)0x40011800) |= 0x00000330; // port E output mode

    while (1)
    {
        delay();
        printf("1");
        *((volatile unsigned int *)0x40011810) |= 0x00060000;
        delay();
        printf("2");
        *((volatile unsigned int *)0x40011810) |= 0x00060000;
        *((volatile unsigned int *)0x40011810) |= 0x00000002;
        delay();
        printf("3");
        *((volatile unsigned int *)0x40011810) |= 0x00060000;
        *((volatile unsigned int *)0x40011810) |= 0x00000004; 
    }
}

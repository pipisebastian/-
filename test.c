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
    *((volatile unsigned int *)0x40021018) |= 0x30; // clock enable port C, D

    *((volatile unsigned int *)0x40011000) &= 0xFF0000FF; // init port C
    *((volatile unsigned int *)0x40011000) |= 0x00888800; // port C input mode

    *((volatile unsigned int *)0x40011404) &= 0xFFFF0FFF; // init port D - 스위치!!! (11 PIN reset)
    *((volatile unsigned int *)0x40011404) |= 0x00008000; // port D input mode

    *((volatile unsigned int *)0x40011400) &= 0xFFFFF00F; // init port D - 스위치!!! (1,2 PIN reset)
    *((volatile unsigned int *)0x40011400) |= 0x00000330; // port D output mode

    while (1)
    {

        if (0x1 & ~(*((volatile unsigned int *)0x4001100C) >> 11)) // 스위치 누름 => pd11
        {                                                          //모터 정지                                                       // Down 2 bit
            *((volatile unsigned int *)0x40011410) |= 0x00000000;  // set PD1,2 -> 0, 0
        }
        else if (0x1 & ~(*((volatile unsigned int *)0x40011008) >> 2)) //DOWN -> 반대방향ㅇ
        {                                                             
            *((volatile unsigned int *)0x40011410) |= 0x00000002;  // set PD1,2 -> 0, 1
        }
        else if (0x1 & ~(*((volatile unsigned int *)0x40011008) >> 5)) //모터 시계
        {                                                              
            *((volatile unsigned int *)0x40011410) |= 0x00000004; // set PD1,2 -> 1, 0
        }
    }
}

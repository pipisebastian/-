

int main(){
    *((volatile unsigned int *)0x40021018) |= 0x30; // clock enable port C, D

    *((volatile unsigned int *)0x40011000) &= 0xFF0000FF; // init port C    
    *((volatile unsigned int *)0x40011000) |= 0x00888800; // port C input mode

    *((volatile unsigned int *)0x40011404) &= 0xFFFF0FFF; // init port D - 스위치!!! (11 PIN reset)
    *((volatile unsigned int *)0x40011404) |= 0x00008000; // port D input mode

    *((volatile unsigned int *)0x40011800) &= 0xFFFFFF0F; // init port E
    *((volatile unsigned int *)0x40011800) |= 0x00000030; // port E 1 output
  
  while(1){
    
    if( 0x1 & ~(*((volatile unsigned int *)0x40011008) >> 2) ){ // Down 2 bit
      printf("down");
      *((volatile unsigned int *)0x40011810) |= 0x00020000; // reset LED1, LED2 off

    } else if( 0x1 & ~(*((volatile unsigned int *)0x40011008) >> 3) ) { // Left 3 bit
      printf("left");
      *((volatile unsigned int *)0x40011810) |= 0x00020000; // reset LED3, LED4 off

    } else if( 0x1 & ~(*((volatile unsigned int *)0x40011008) >> 4) ) { // Right 4 bit
      printf("right");
      *((volatile unsigned int *)0x40011810) |= 0x00000002; // set LED3, LED4 on

    } else if( 0x1 & ~(*((volatile unsigned int *)0x40011008) >> 5) ) { // Up 5 bit
      printf("up");
      *((volatile unsigned int *)0x40011810) |= 0x00000002; // set LED1, LED2 on
    } else {
        printf("?");
    }
    }
}

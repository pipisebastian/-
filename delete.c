int main(){
  *((volatile unsigned int *)0x40021018) |= 0x30; // clock enable port C, D

  *((volatile unsigned int *)0x40011000) &= 0x00000000; // init port C
  *((volatile unsigned int *)0x40011000) |= 0x00888800; // port C input mode

  *((volatile unsigned int *)0x40011400) &= 0x00000000; // init port D
  *((volatile unsigned int *)0x40011400) |= 0x30033300; // port D output mode

  while(1){
    if( 0x1 & (*((volatile unsigned int *)0x40011008) >> 2) ){ // Down 2 bit

      *((volatile unsigned int *)0x40011410) |= 0x000c0000; // reset LED1, LED2 off

    } else if( 0x1 & (*((volatile unsigned int *)0x40011008) >> 3) ) { // Left 3 bit

      *((volatile unsigned int *)0x40011410) |= 0x00900000; // reset LED3, LED4 off

    } else if( 0x1 & (*((volatile unsigned int *)0x40011008) >> 4) ) { // Right 4 bit

      *((volatile unsigned int *)0x40011410) |= 0x00000090; // set LED3, LED4 on

    } else if( 0x1 & (*((volatile unsigned int *)0x40011008) >> 5) ) { // Up 5 bit

      *((volatile unsigned int *)0x40011410) |= 0x000c000c; // set LED1, LED2 on
      
    }
  }
}

# stm32f7_getInfo
Get chip info and core registers for STM32F7xxx and output to UART one<br>
Output should look something like this:<br>
Connected to UART One<br>
stm32f76xxx/stm32f77xxx  Revision A<br>
ARM Cortex M7 r1p0<br>
Single and double precision FPU<br>
2048K Flash, 512K RAM (128K DTCM, 368K SRAM1, 16K SRAM2)<br>
LQFP176, LQFP208 or TFBGA216<br>
Device Unique ID: 002B0036 33355111 36303934<br>
Wafer 17 of Lot Q534906<br>
Location on wafer: X:54, Y:43<br>
HAL Version: 1.2.0 <br>
BSP Version: 2.0.0 <br>
<br>
Oscillators and clocks<br>
----------------------<br>
            HSE Speed: 25000000<br>
            LSE Speed: 32768<br>
            HSI Speed: 16000000<br>
        SYSCLK Source: PLLCLK<br>
           PLL Source: HSE<br>
                PLL M: /25<br>
                PLL N: *400<br>
                PLL P: /2<br>
                PLL Q: /4<br>
                PLL R: /2<br>
       AHBCLK Divider: /1<br>
      APB1CLK Divider: /4<br>
      APB2CLK Divider: /2<br>
        Flash latency: 6<br>
    System core clock: 200000000<br>
<br>
Core registers<br>
--------------<br>
           r0: 0x0000000A<br>
           r1: 0x2000033B<br>
           r2: 0x00000000<br>
           r3: 0x12000000<br>
           r4: 0x00000000<br>
           r5: 0x00000000<br>
           r6: 0x00000000<br>
           r7: 0x2007F0EC<br>
           r8: 0x00000000<br>
           r9: 0x00000000<br>
          r10: 0x00000000<br>
          r11: 0x00000000<br>
          r12: 0x2007F100<br>
stack pointer: 0x2007F0C0<br>
link register: 0x08002F0B<br>
<br>
Developed and tested on STM32F769I-Discovery<br>

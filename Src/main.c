/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32f7xx_ll_rcc.h"
#include "stm32f769i_discovery.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
RCC_OscInitTypeDef RCC_OscInitStruct;
RCC_ClkInitTypeDef RCC_ClkInitStruct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache-------------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache-------------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED2);
  BSP_LED_On(LED2);

  char uart1Data[24] = "Connected to UART One\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)&uart1Data,sizeof(uart1Data), 0xFFFF);

	uint8_t len = 0;
	//uint32_t latency;
	char devidString[512] = {0};
	uint32_t idcode = DBGMCU->IDCODE & 0xFFF;
	switch (idcode)
	{
	case 0x449: len += sprintf(devidString+len, "stm32f74xxx/stm32f75xxx"); break;
	case 0x451: len += sprintf(devidString+len, "stm32f76xxx/stm32f77xxx"); break;
	case 0x452: len += sprintf(devidString+len, "stm32f72xxx/stm32f73xxx"); break;
	default: len += sprintf(devidString+len, "unknown device\r\n");
	}

	uint32_t revCode = (DBGMCU->IDCODE & DBGMCU_IDCODE_REV_ID_Msk) >> DBGMCU_IDCODE_REV_ID_Pos;

	switch (revCode)
	{
	case 0x1000: len += sprintf(devidString+len, "  Revision A\r\n"); break;
	case 0x1001: len += sprintf(devidString+len, "  Revision Z\r\n"); break;
	}
	printf("%s", devidString);


	len = 0;
	uint32_t cpuid = SCB->CPUID;
	uint32_t var, pat;
	pat = (cpuid & 0x0000000F);
	var = (cpuid & 0x00F00000) >> 20;
	uint32_t mvfr0;
	char cpuString[24] = {0};

	//len += sprintf(cpuString, "CPU ID: 0x");
	//len += sprintf(cpuString+len, "%lX\r\n", cpuid);

	if ((cpuid & 0xFF000000) == 0x41000000) // ARM
		{
			len += sprintf(cpuString+len, "ARM ");
			switch((cpuid & 0x0000FFF0) >> 4)
				{
					case 0xC20 : len += sprintf(cpuString+len, "Cortex M0 r%ldp%ld\r\n", var, pat); break;
					case 0xC60 : len += sprintf(cpuString+len, "Cortex M0+ r%ldp%ld\r\n", var, pat); break;
					case 0xC21 : len += sprintf(cpuString+len, "Cortex M1 r%ldp%ld\r\n", var, pat); break;
					case 0xC23 : len += sprintf(cpuString+len, "Cortex M3 r%ldp%ld\r\n", var, pat); break;
					case 0xC24 : len += sprintf(cpuString+len, "Cortex M4 r%ldp%ld\r\n", var, pat); break;
					case 0xC27 : len += sprintf(cpuString+len, "Cortex M7 r%ldp%ld\r\n", var, pat); break;

					default : len += sprintf(cpuString+len, "Unknown CORE\r\n");
				}
		}

    //mvfr0 = *(volatile uint32_t *)0xE000EF40;
	//mvfr0 = SCB->MVFR;

    mvfr0 = SCB->MVFR0;
   if (mvfr0 > 0){
   /*
   printf("FPCCR: %08lX\r\n", *(volatile uint32_t *)0xE000EF34); // 0xC0000000
   printf("FPCAR: %08lX\r\n", *(volatile uint32_t *)0xE000EF38);
   printf("FPDSCR: %08lX\r\n", *(volatile uint32_t *)0xE000EF3C);
   printf("MVFR0: %08lX\r\n", *(volatile uint32_t *)0xE000EF40); // 0x10110021 single or 10110221 double
   printf("MVFR1: %08lX\r\n", *(volatile uint32_t *)0xE000EF44); // 0x11000011 or 12000011
   printf("MVFR2: %08lX\r\n", *(volatile uint32_t *)0xE000EF48); // 0x00000040
	 */
   switch(mvfr0)
   	 {
   	 case 0x10110021 : len += sprintf(cpuString+len, "Single precision FPU\r\n"); break;
   	 case 0x10110221 : len += sprintf(cpuString+len, "Single and double precision FPU\r\n"); break;
   	 //default : len += sprintf(cpuString+len, "No FPU\r\n");
   	 default: ;
   	 }
   }
	printf("%s",cpuString);

	len = 0;
	char memString[512] = {0};
	switch (idcode)
	{
	case 0x449: len += sprintf(memString+len, "%dK Flash, 320K RAM (64K DTCM, 240K SRAM1, 16K SRAM2)\r\n", *((unsigned short *)FLASHSIZE_BASE)); break;
	case 0x451: len += sprintf(memString+len, "%dK Flash, 512K RAM (128K DTCM, 368K SRAM1, 16K SRAM2)\r\n", *((unsigned short *)FLASHSIZE_BASE)); break;
	case 0x452: len += sprintf(memString+len, "%dK Flash, 256K RAM (64K DTCM, 176K SRAM1, 16K SRAM2)\r\n", *((unsigned short *)FLASHSIZE_BASE)); break;
	default: len += sprintf(memString+len, "unknown device\r\n");
	}
	printf("%s", memString);


	len = 0;
	char pkgString[128] = {0};
	uint32_t pkg = *((uint32_t *)PACKAGE_BASE) & 0x0700;
	switch (pkg)
	{
	case 0x00000100: len += sprintf(pkgString+len, "LQFP100\r\n"); break;
	case 0x00000200: len += sprintf(pkgString+len, "LQFP144 or WLCSP143\r\n"); break;
	case 0x00000300: len += sprintf(pkgString+len, "WLCSP180, LQFP176 or UFBGA176\r\n"); break;
	case 0x00000400: len += sprintf(pkgString+len, "LQFP176, LQFP208 or TFBGA216\r\n"); break;
	case 0x00000500: len += sprintf(pkgString+len, "LQFP176, LQFP208 or TFBGA216\r\n"); break;
	case 0x00000600: len += sprintf(pkgString+len, "LQFP176, LQFP208 or TFBGA216\r\n"); break;
	case 0x00000700: len += sprintf(pkgString+len, "LQFP176, LQFP208 or TFBGA216\r\n"); break;
	}
	printf("%s", pkgString);


	len = 0;
	char idString[2048] = {0};
	uint32_t word0 = *(uint32_t *)(UID_BASE);
	uint32_t word1 = *(uint32_t *)(UID_BASE + 0x04);
	uint32_t word2 = *(uint32_t *)(UID_BASE + 0x08);
	uint64_t lotNum;
	char lotNumString[32] = {0};
	lotNum = ((uint64_t)word2 << 24 ) | (word1 >> 8);
	len += sprintf(idString, "Device Unique ID: ");
	len += sprintf(idString+len, "%08lX %08lX %08lX\r\n", word0, word1, word2);
	uint32_t x = word0 & 0xFFFF;
	uint32_t y = (word0 & 0xFFFF0000) >> 16;
	uint32_t wafer = word1 & 0xFF;
	sprintf(lotNumString, "%s", (char *)&lotNum);
	len += sprintf(idString+len, "Wafer %ld of Lot %s\r\n", wafer, lotNumString);
	len += sprintf(idString+len, "Location on wafer: X:%ld, Y:%ld\r\n", x,y);
	printf("%s", idString);


	len = 0;
	char myString[512] = {0};
	len += sprintf(myString+len, "HAL Version: ");
	uint32_t halVersion = HAL_GetHalVersion();
	len += sprintf(myString+len, "%d.", (uint8_t)(halVersion >> 24) & 0xff);
	len += sprintf(myString+len, "%d.", (uint8_t)(halVersion >> 16) & 0xff);
	len += sprintf(myString+len, "%d ", (uint8_t)(halVersion >> 8) & 0xff);
	if ((uint8_t)(halVersion >> 0) & 0xff)
	  {
		len += sprintf(myString+len, "rc%d\r\n", (uint8_t)(halVersion >> 0) & 0xff);
	  }
	else
	  {
		len += sprintf(myString+len, "\r\n");
	  }

	len += sprintf(myString+len, "BSP Version: ");
	uint32_t bspVersion = BSP_GetVersion();
	len += sprintf(myString+len, "%d.", (uint8_t)(bspVersion >> 24) & 0xff);
	len += sprintf(myString+len, "%d.", (uint8_t)(bspVersion >> 16) & 0xff);
	len += sprintf(myString+len, "%d ", (uint8_t)(bspVersion >> 8) & 0xff);
	if ((uint8_t)(bspVersion >> 0) & 0xff)
	  {
		len += sprintf(myString+len, "rc%d\r\n", (uint8_t)(bspVersion >> 0) & 0xff);
	  }
	else
	  {
		len += sprintf(myString+len, "\r\n");
	  }
	printf("%s", myString);


	printf("\r\n");

	printf("Oscillators and clocks\r\n");
	printf("----------------------\r\n");
	HAL_RCC_GetOscConfig(&RCC_OscInitStruct);
	uint32_t latency;
	HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &latency);

	//printf("      Oscillator type: %08lX\r\n", RCC_OscInitStruct.OscillatorType);

	if (RCC_OscInitStruct.HSEState > 0) {
	//printf("            HSE State: %08lX\r\n", RCC_OscInitStruct.HSEState);
	printf("            HSE Speed: %ld\r\n", HSE_VALUE);
	  } else {
	//printf("            HSE State: Off\r\n");
	}

	if (RCC_OscInitStruct.LSEState > 0) {
	//printf("            LSE State: %08lX\r\n", RCC_OscInitStruct.LSEState);
	printf("            LSE Speed: %ld\r\n", LSE_VALUE);
	  } else {
  //printf("            LSE State: Off\r\n");
	}

	if (RCC_OscInitStruct.HSIState > 0) {
	//printf("            HSI State: On\r\n");
	//printf("HSI Calibration value: %08lX\r\n", RCC_OscInitStruct.HSICalibrationValue);
	printf("            HSI Speed: %ld\r\n", HSI_VALUE);
	  } else {
  //printf("            HSI State: Off\r\n");
	}

	if (RCC_OscInitStruct.LSIState > 0) {
	//printf("            LSI State: On\r\n");
	printf("            LSI Speed: %ld\r\n", LSI_VALUE);
	  } else {
	//printf("            LSI State: Off\r\n");
	}

	printf("        SYSCLK Source: ");
	switch (RCC_ClkInitStruct.SYSCLKSource)
	{
		case RCC_SYSCLKSOURCE_HSI : printf("HSI\r\n"); break;
		case RCC_SYSCLKSOURCE_HSE : printf("HSE\r\n"); break;
		case RCC_SYSCLKSOURCE_PLLCLK : printf("PLLCLK\r\n"); break;
		default : printf("??\r\n");
	}


	if (RCC_OscInitStruct.PLL.PLLState > 0) {
	  printf("           PLL Source: ");
	  switch (RCC_OscInitStruct.PLL.PLLSource)
		{
			//case RCC_PLLSOURCE_NONE  : printf("No PLL\r\n"); break;
			case RCC_PLLSOURCE_HSI : printf("HSI\r\n"); break;
			case RCC_PLLSOURCE_HSE : printf("HSE\r\n"); break;
		}
	//printf("       PLL Multiplier: %08lX\r\n", RCC_OscInitStruct.PLL.PLLMUL);
	  printf("                PLL M: /%ld\r\n", RCC_OscInitStruct.PLL.PLLM);
	  printf("                PLL N: *%ld\r\n", RCC_OscInitStruct.PLL.PLLN);
	  printf("                PLL P: ");
	  switch (RCC_OscInitStruct.PLL.PLLP)
	    {
	  	  case RCC_PLLP_DIV2 : printf("/2\r\n"); break;
	  	  case RCC_PLLP_DIV4 : printf("/4\r\n"); break;
	  	  case RCC_PLLP_DIV6 : printf("/6\r\n"); break;
	  	  case RCC_PLLP_DIV8 : printf("/8\r\n"); break;
	    }

	  printf("                PLL Q: /%ld\r\n", RCC_OscInitStruct.PLL.PLLQ);
	  //printf("                PLL Q: /%ld\r\n", ((RCC->PLLCFGR) & RCC_PLLCFGR_PLLQ) >> RCC_PLLCFGR_PLLQ_Pos);
	  //printf("rcc->cfgr: %lx\r\n", ((RCC->PLLCFGR) & RCC_PLLCFGR_PLLR) >> 28 );
	  printf("                PLL R: /%ld\r\n", RCC_OscInitStruct.PLL.PLLR);
	  //printf("                PLL R: /%ld\r\n", ((RCC->PLLCFGR) & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos);
	  //printf("                PLL R: /%ld\r\n", ((RCC->PLLCFGR) & 0x70000000) >> 28);

	}

	//printf("           Clock Type: %08lX\r\n", RCC_ClkInitStruct.ClockType);
	//printf("\r\n");

	printf("       AHBCLK Divider: ");
	switch (RCC_ClkInitStruct.AHBCLKDivider)
	{
	case RCC_SYSCLK_DIV1 : printf("/1\r\n"); break;
	case RCC_SYSCLK_DIV2 : printf("/2\r\n"); break;
	case RCC_SYSCLK_DIV4 : printf("/4\r\n"); break;
	case RCC_SYSCLK_DIV8 : printf("/8\r\n"); break;
	case RCC_SYSCLK_DIV16 : printf("/16\r\n"); break;
	case RCC_SYSCLK_DIV64 : printf("/64\r\n"); break;
	case RCC_SYSCLK_DIV128 : printf("/128\r\n"); break;
	case RCC_SYSCLK_DIV256 : printf("/256\r\n"); break;
	case RCC_SYSCLK_DIV512 : printf("/512\r\n"); break;
	}

	printf("      APB1CLK Divider: ");
	switch (RCC_ClkInitStruct.APB1CLKDivider)
	{
	case RCC_CFGR_PPRE1_DIV1 : printf("/1\r\n"); break;
	case RCC_CFGR_PPRE1_DIV2 : printf("/2\r\n"); break;
	case RCC_CFGR_PPRE1_DIV4 : printf("/4\r\n"); break;
	case RCC_CFGR_PPRE1_DIV8 : printf("/8\r\n"); break;
	case RCC_CFGR_PPRE1_DIV16 : printf("/16\r\n"); break;
	}


	printf("      APB2CLK Divider: ");
	switch (RCC_ClkInitStruct.APB2CLKDivider)
	{
	case RCC_CFGR_PPRE1_DIV1 : printf("/1\r\n"); break;
	case RCC_CFGR_PPRE1_DIV2 : printf("/2\r\n"); break;
	case RCC_CFGR_PPRE1_DIV4 : printf("/4\r\n"); break;
	case RCC_CFGR_PPRE1_DIV8 : printf("/8\r\n"); break;
	case RCC_CFGR_PPRE1_DIV16 : printf("/16\r\n"); break;
	}

	printf("        Flash latency: %lX\r\n", latency);

	len = 0;
	char sysclkString[29] = {0};
	len += sprintf(sysclkString, "    System core clock: ");
	len += sprintf(sysclkString+len, "%lu\r\n", HAL_RCC_GetSysClockFreq());
	printf("%s", sysclkString);

	printf("\r\n");
	printf("Core registers\r\n");
	printf("--------------\r\n");
	register int r0 asm ("r0");
	printf("           r0: 0x%08X\r\n", r0);
	register int r1 asm ("r1");
	printf("           r1: 0x%08X\r\n", r1);
	register int r2 asm ("r2");
	printf("           r2: 0x%08X\r\n", r2);
	register int r3 asm ("r3");
	printf("           r3: 0x%08X\r\n", r3);
	register int r4 asm ("r4");
	printf("           r4: 0x%08X\r\n", r4);
	register int r5 asm ("r5");
	printf("           r5: 0x%08X\r\n", r5);
	register int r6 asm ("r6");
	printf("           r6: 0x%08X\r\n", r6);
	register int r7 asm ("r7");
	printf("           r7: 0x%08X\r\n", r7);
	register int r8 asm ("r8");
	printf("           r8: 0x%08X\r\n", r8);
	register int r9 asm ("r9");
	printf("           r9: 0x%08X\r\n", r9);
	register int r10 asm ("r10");
	printf("          r10: 0x%08X\r\n", r10);
	register int r11 asm ("r11");
	printf("          r11: 0x%08X\r\n", r11);
	register int r12 asm ("r12");
	printf("          r12: 0x%08X\r\n", r12);
	register int sp asm ("sp");
	printf("stack pointer: 0x%08X\r\n", sp);
	register int lr asm ("lr");
	printf("link register: 0x%08X\r\n", lr);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins
     PE4   ------> SAI1_FS_A
     PE3   ------> SAI1_SD_B
     PE2   ------> QUADSPI_BK1_IO2
     PG14   ------> ETH_TXD1
     PE1   ------> FMC_NBL1
     PE0   ------> FMC_NBL0
     PB8   ------> I2C1_SCL
     PB5   ------> USB_OTG_HS_ULPI_D7
     PB4   ------> SDMMC2_D3
     PB3   ------> SDMMC2_D2
     PD7   ------> SDMMC2_CMD
     PC12   ------> UART5_TX
     PA15   ------> CEC
     PE5   ------> SAI1_SCK_A
     PE6   ------> SAI1_SD_A
     PG13   ------> ETH_TXD0
     PB9   ------> I2C1_SDA
     PB7   ------> I2C4_SDA
     PB6   ------> QUADSPI_BK1_NCS
     PG15   ------> FMC_SDNCAS
     PG11   ------> ETH_TX_EN
     PD6   ------> SDMMC2_CK
     PD0   ------> FMC_D2_DA2
     PC11   ------> S_DATAIN5DFSDM1
     PC10   ------> QUADSPI_BK1_IO1
     PA12   ------> SPI2_SCK
     PI4   ------> FMC_NBL2
     PG12   ------> SPDIFRX_IN1
     PG10   ------> SDMMC2_D1
     PD3   ------> S_CKOUTDFSDM1
     PD1   ------> FMC_D3_DA3
     PI3   ------> FMC_D27
     PI2   ------> FMC_D26
     PA11   ------> SPI2_NSS
     PF0   ------> FMC_A0
     PI5   ------> FMC_NBL3
     PI7   ------> FMC_D29
     PI10   ------> FMC_D31
     PI6   ------> FMC_D28
     PG9   ------> SDMMC2_D0
     PD2   ------> UART5_RX
     PH15   ------> FMC_D23
     PI1   ------> FMC_D25
     PF1   ------> FMC_A1
     PI9   ------> FMC_D30
     PH13   ------> FMC_D21
     PH14   ------> FMC_D22
     PI0   ------> FMC_D24
     PI11   ------> USB_OTG_HS_ULPI_DIR
     PC9   ------> QUADSPI_BK1_IO0
     PA8   ------> RCC_MCO_1
     PF2   ------> FMC_A2
     PC8   ------> S_TIM3_CH3
     PC7   ------> USART6_RX
     PF3   ------> FMC_A3
     PH4   ------> USB_OTG_HS_ULPI_NXT
     PG8   ------> FMC_SDCLK
     PC6   ------> USART6_TX
     PF4   ------> FMC_A4
     PH5   ------> FMC_SDNWE
     PH3   ------> FMC_SDNE0
     PG7   ------> SAI1_MCLK_A
     PF7   ------> S_TIM11_CH1
     PF6   ------> S_TIM10_CH1
     PF5   ------> FMC_A5
     PH2   ------> FMC_SDCKE0
     PD15   ------> FMC_D1_DA1
     PB13   ------> USB_OTG_HS_ULPI_D6
     PD10   ------> FMC_D15_DA15
     PF10   ------> ADC3_IN8
     PF9   ------> ADC3_IN7
     PF8   ------> ADC3_IN6
     PC3   ------> S_DATAIN1DFSDM1
     PD14   ------> FMC_D0_DA0
     PB12   ------> USB_OTG_HS_ULPI_D5
     PD9   ------> FMC_D14_DA14
     PD8   ------> FMC_D13_DA13
     PC0   ------> USB_OTG_HS_ULPI_STP
     PC1   ------> ETH_MDC
     PC2   ------> ADCx_IN12
     PB2   ------> QUADSPI_CLK
     PF12   ------> FMC_A6
     PG1   ------> FMC_A11
     PF15   ------> FMC_A9
     PD12   ------> I2C4_SCL
     PD13   ------> QUADSPI_BK1_IO3
     PG2   ------> FMC_A12
     PH12   ------> FMC_D20
     PA1   ------> ETH_REF_CLK
     PA4   ------> ADCx_IN4
     PC4   ------> ETH_RXD0
     PF13   ------> FMC_A7
     PG0   ------> FMC_A10
     PE8   ------> FMC_D5_DA5
     PD11   ------> SAI2_SD_A
     PG5   ------> FMC_A15_BA1
     PG4   ------> FMC_A14_BA0
     PH9   ------> FMC_D17
     PH11   ------> FMC_D19
     PA2   ------> ETH_MDIO
     PA6   ------> ADCx_IN6
     PA5   ------> USB_OTG_HS_ULPI_CK
     PC5   ------> ETH_RXD1
     PF14   ------> FMC_A8
     PJ2   ------> DSIHOST_TE
     PF11   ------> FMC_SDNRAS
     PE9   ------> FMC_D6_DA6
     PE11   ------> FMC_D8_DA8
     PE14   ------> FMC_D11_DA11
     PB10   ------> USB_OTG_HS_ULPI_D3
     PH6   ------> S_TIM12_CH1
     PH8   ------> FMC_D16
     PH10   ------> FMC_D18
     PA3   ------> USB_OTG_HS_ULPI_D0
     PA7   ------> ETH_CRS_DV
     PB1   ------> USB_OTG_HS_ULPI_D2
     PB0   ------> USB_OTG_HS_ULPI_D1
     PE7   ------> FMC_D4_DA4
     PE10   ------> FMC_D7_DA7
     PE12   ------> FMC_D9_DA9
     PE15   ------> FMC_D12_DA12
     PE13   ------> FMC_D10_DA10
     PB11   ------> USB_OTG_HS_ULPI_D4
     PB14   ------> SPI2_MISO
     PB15   ------> SPI2_MOSI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pins : SAI1_FSA_Pin SAI1_SDB_Pin SAI1_SCKA_Pin SAI1_SDA_Pin */
  GPIO_InitStruct.Pin = SAI1_FSA_Pin|SAI1_SDB_Pin|SAI1_SCKA_Pin|SAI1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D2_Pin */
  GPIO_InitStruct.Pin = QSPI_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin|RMII_TXD0_Pin|RMII_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_NBL1_Pin FMC_NBL0_Pin FMC_D5_Pin FMC_D6_Pin 
                           FMC_D8_Pin FMC_D11_Pin FMC_D4_Pin FMC_D7_Pin 
                           FMC_D9_Pin FMC_D12_Pin FMC_D10_Pin */
  GPIO_InitStruct.Pin = FMC_NBL1_Pin|FMC_NBL0_Pin|FMC_D5_Pin|FMC_D6_Pin 
                          |FMC_D8_Pin|FMC_D11_Pin|FMC_D4_Pin|FMC_D7_Pin 
                          |FMC_D9_Pin|FMC_D12_Pin|FMC_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_SCL_D15_Pin ARDUINO_SDA_D14_Pin */
  GPIO_InitStruct.Pin = ARDUINO_SCL_D15_Pin|ARDUINO_SDA_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin 
                           ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
  GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin 
                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : uSD_D3_Pin uSD_D2_Pin */
  GPIO_InitStruct.Pin = uSD_D3_Pin|uSD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_SDMMC2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : uSD_CMD_Pin uSD_CLK_Pin */
  GPIO_InitStruct.Pin = uSD_CMD_Pin|uSD_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_SDMMC2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_RX_Pin */
  GPIO_InitStruct.Pin = WIFI_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(WIFI_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CEC_Pin */
  GPIO_InitStruct.Pin = CEC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_CEC;
  HAL_GPIO_Init(CEC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AUDIO_SDA_Pin */
  GPIO_InitStruct.Pin = AUDIO_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_I2C4;
  HAL_GPIO_Init(AUDIO_SDA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_NCS_Pin */
  GPIO_InitStruct.Pin = QSPI_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_SDNCAS_Pin FMC_SDCLK_Pin FMC_A11_Pin FMC_A12_Pin 
                           FMC_A10_Pin FMC_BA1_Pin FMC_BA0_Pin */
  GPIO_InitStruct.Pin = FMC_SDNCAS_Pin|FMC_SDCLK_Pin|FMC_A11_Pin|FMC_A12_Pin 
                          |FMC_A10_Pin|FMC_BA1_Pin|FMC_BA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LD_USER1_Pin Audio_INT_Pin WIFI_RST_Pin DSI_RESET_Pin 
                           ARD_D8_Pin LD_USER2_Pin ARD_D7_Pin ARD_D4_Pin 
                           ARD_D2_Pin */
  GPIO_InitStruct.Pin = LD_USER1_Pin|Audio_INT_Pin|WIFI_RST_Pin|DSI_RESET_Pin 
                          |ARD_D8_Pin|LD_USER2_Pin|ARD_D7_Pin|ARD_D4_Pin 
                          |ARD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_D2_Pin FMC_D3_Pin FMC_D1_Pin FMC_D15_Pin 
                           FMC_D0_Pin FMC_D14_Pin FMC_D13_Pin */
  GPIO_InitStruct.Pin = FMC_D2_Pin|FMC_D3_Pin|FMC_D1_Pin|FMC_D15_Pin 
                          |FMC_D0_Pin|FMC_D14_Pin|FMC_D13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DFSDM_DATIN5_Pin DFSDM_DATIN1_Pin */
  GPIO_InitStruct.Pin = DFSDM_DATIN5_Pin|DFSDM_DATIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_DFSDM1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : QSPI_D1_Pin QSPI_D0_Pin */
  GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D13_SCK_Pin */
  GPIO_InitStruct.Pin = ARD_D13_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(ARD_D13_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NC4_Pin NC5_Pin uSD_Detect_Pin LCD_BL_CTRL_Pin */
  GPIO_InitStruct.Pin = NC4_Pin|NC5_Pin|uSD_Detect_Pin|LCD_BL_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_NBL2_Pin D27_Pin D26_Pin FMC_NBL3_Pin 
                           D29_Pin D31_Pin D28_Pin D25_Pin 
                           D30_Pin D24_Pin */
  GPIO_InitStruct.Pin = FMC_NBL2_Pin|D27_Pin|D26_Pin|FMC_NBL3_Pin 
                          |D29_Pin|D31_Pin|D28_Pin|D25_Pin 
                          |D30_Pin|D24_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : NC3_Pin NC2_Pin NC1_Pin NC8_Pin 
                           NC7_Pin */
  GPIO_InitStruct.Pin = NC3_Pin|NC2_Pin|NC1_Pin|NC8_Pin 
                          |NC7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_RX_Pin */
  GPIO_InitStruct.Pin = SPDIF_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_SPDIFRX;
  HAL_GPIO_Init(SPDIF_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : uSD_D1_Pin uSD_D0_Pin */
  GPIO_InitStruct.Pin = uSD_D1_Pin|uSD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_SDMMC2;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_RXER_Pin OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = RMII_RXER_Pin|OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : DFSDM_CKOUT_Pin */
  GPIO_InitStruct.Pin = DFSDM_CKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_DFSDM1;
  HAL_GPIO_Init(DFSDM_CKOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_NSS_Pin */
  GPIO_InitStruct.Pin = SPI2_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(SPI2_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FMC_A0_Pin FMC_A1_Pin FMC_A2_Pin FMC_A3_Pin 
                           FMC_A4_Pin FMC_A5_Pin FMC_A6_Pin FMC_A9_Pin 
                           FMC_A7_Pin FMC_A8_Pin FMC_SDNRAS_Pin */
  GPIO_InitStruct.Pin = FMC_A0_Pin|FMC_A1_Pin|FMC_A2_Pin|FMC_A3_Pin 
                          |FMC_A4_Pin|FMC_A5_Pin|FMC_A6_Pin|FMC_A9_Pin 
                          |FMC_A7_Pin|FMC_A8_Pin|FMC_SDNRAS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_TX_Pin */
  GPIO_InitStruct.Pin = WIFI_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(WIFI_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D23_Pin D21_Pin D22_Pin FMC_SDNME_Pin 
                           FMC_SDNE0_Pin FMC_SDCKE0_Pin D20_Pin FMC_D_7_Pin 
                           FMC_D19_Pin FMC_D16_Pin FMC_D18_Pin */
  GPIO_InitStruct.Pin = D23_Pin|D21_Pin|D22_Pin|FMC_SDNME_Pin 
                          |FMC_SDNE0_Pin|FMC_SDCKE0_Pin|D20_Pin|FMC_D_7_Pin 
                          |FMC_D19_Pin|FMC_D16_Pin|FMC_D18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_DIR_Pin */
  GPIO_InitStruct.Pin = ULPI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CEC_CLK_Pin */
  GPIO_InitStruct.Pin = CEC_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(CEC_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_INT_Pin */
  GPIO_InitStruct.Pin = LCD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D5_PWM_Pin */
  GPIO_InitStruct.Pin = ARD_D5_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARD_D5_PWM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D0_RX_Pin ARDUINO_TX_D1_Pin */
  GPIO_InitStruct.Pin = ARD_D0_RX_Pin|ARDUINO_TX_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_NXT_Pin */
  GPIO_InitStruct.Pin = ULPI_NXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SAI1_MCLKA_Pin */
  GPIO_InitStruct.Pin = SAI1_MCLKA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
  HAL_GPIO_Init(SAI1_MCLKA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EXT_SDA_Pin EXT_SCL_Pin */
  GPIO_InitStruct.Pin = EXT_SDA_Pin|EXT_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_PWM_Pin */
  GPIO_InitStruct.Pin = ARD_D6_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
  HAL_GPIO_Init(ARD_D6_PWM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_PWM_Pin */
  GPIO_InitStruct.Pin = ARD_D3_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
  HAL_GPIO_Init(ARD_D3_PWM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_A1_Pin ARDUINO_A2_Pin ARDUINO_A3_Pin */
  GPIO_InitStruct.Pin = ARDUINO_A1_Pin|ARDUINO_A2_Pin|ARDUINO_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : ULPI_STP_Pin */
  GPIO_InitStruct.Pin = ULPI_STP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(ULPI_STP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_A2_Pin */
  GPIO_InitStruct.Pin = ARD_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_A2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : AUDIO_SCL_Pin */
  GPIO_InitStruct.Pin = AUDIO_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
  HAL_GPIO_Init(AUDIO_SCL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : QSPI_D3_Pin */
  GPIO_InitStruct.Pin = QSPI_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(QSPI_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : B_USER_Pin */
  GPIO_InitStruct.Pin = B_USER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B_USER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPDIF_TX_Pin */
  GPIO_InitStruct.Pin = SPDIF_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
  HAL_GPIO_Init(SPDIF_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_RST_Pin */
  GPIO_InitStruct.Pin = EXT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXT_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
  GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DSIHOST_TE_Pin */
  GPIO_InitStruct.Pin = DSIHOST_TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DSI;
  HAL_GPIO_Init(DSIHOST_TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARDUINO_PWM_D6_Pin */
  GPIO_InitStruct.Pin = ARDUINO_PWM_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
  HAL_GPIO_Init(ARDUINO_PWM_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
  GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
     /**
       * @brief  Retargets the C library printf function to the USART.
       * @param  None
       * @retval None
       */
     PUTCHAR_PROTOTYPE
     {
       /* Place your implementation of fputc here */
       /* e.g. write a character to the USART2 and Loop until the end of transmission */
       HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

       return ch;
     }

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

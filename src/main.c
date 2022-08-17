/*==============================================================================
 * Name        : main.c
 * Author      : Martin Doff-Sotta (martin.doff-sotta@eng.ox.ac.uk) 
 * Description : A blinky example for the stm32h743
 * Note        : Tested on stm32h743vit6 (version V) development board from DevEBox 
 -------------------------------------------------------------------------------
 * The MIT License (MIT)
 * Copyright (c) 2021 Martin Doff-Sotta
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
===============================================================================*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Override the 'write' clib method to implement 'printf' over UART.
int _write(int handle, char* data, int size) {
  int count = size;
  
  // invariant: data[0:i] have been added to TDR register  
  while(count--) {
      while(!(UART5->ISR & USART_ISR_TXE_TXFNF)) {}; // wait for empty transmit register
      UART5->TDR = *data++; 
  
  }
  return size;
}

typedef union {
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void toggle_LED(void);
static void LED_Init(void);
static void UART_Init(void);
void UART_send_blocking(uint8_t*);
void UART_rcv_blocking(uint8_t*);
void delay(int comp); 

/**
  * The application entry point.
  */
int main(void)
{
  
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  LED_Init();
  UART_Init();
  GPIOA->ODR     |=  GPIOA1;    // LED in OFF state
  
  /* Infinite loop */
  FLOATUNION_t rcv;
  FLOATUNION_t snd;
  float TAS = 0.0;
  float ref_TAS = 80.0;
  float k_p = 500.0; 
  float k_i = 30.0;
  float u = 0.0;
  float err = 0.0;
  float sum_err = 0.0;
  float d = 0.1; 
  uint8_t ch = '\0'; 
  while (1)
  {

    	//Reception
    	for (int i=0; i<4; i++)
    	{
            UART_rcv_blocking(&rcv.bytes[i]);
    	}
    	                   
    	// Controller (PID)
    	TAS = rcv.number;
    	err = ref_TAS-TAS;
    	sum_err += err*d; 
    	u = k_p*err + k_i*sum_err;
    	
    	// Transmission
    	snd.number = u; 
    	ch = 'A';
        UART_send_blocking(&ch);
    	for (int i=0; i<4; i++)
    	{
            UART_send_blocking(&snd.bytes[i]);
    	}
    	ch = '\n'; 
        UART_send_blocking(&ch);
        
      
  }

}

void toggle_LED(void)
{
	GPIOA->ODR &= ~GPIOA1;  // pull down (clear) => ON
    delay(30000000);
    GPIOA->ODR |=  GPIOA1;  // pull up (set) => OFF
    delay(30000000);
}
/**
  * Configure peripherals for LED blink
  */
static void LED_Init(void)
{
  
  //Initialize all configured peripherals
  RCC_AHB4ENR |= RCC_AHB4ENR_GPIOAEN;

  // Set GPIO #PA1
  GPIOA->MODER   |=  (1 << 2);  // GPIO port mode register (general purpose output mode) 
  GPIOA->MODER   &= ~(1 << 3);  
  GPIOA->OTYPER  |=  (0 << 1);  // GPIO port output type register (Output push-pull)
  GPIOA->OSPEEDR |=  (0 << 2);  // GPIO port output speed register (Low speed)
  GPIOA->OSPEEDR |=  (0 << 3);
  GPIOA->PUPDR   |=  (0 << 2);  // GPIO port pull-up/pull-down register (No pull-up/-down)
  GPIOA->PUPDR   |=  (0 << 3);
}

/**
  * Configure peripherals for UART
  * PB12: UART5_RX, PB13: UART5_TX
  */
static void UART_Init(void)
{
  
  // Enable peripheral clocks: GPIOB, UART5.
  RCC_AHB4ENR  |= RCC_AHB4ENR_GPIOBEN;
  RCC_APB1LENR |= RCC_APB1LENR_UART5EN;

  // Set PB13 for TX UART5 
  GPIOB->MODER   &=  ~(0x3UL << 26U);
  GPIOB->MODER   |=   (0x2UL << 26U); // MODER13[1:0] <- 0x10 for alternate function mode
  GPIOB->OTYPER  &=  ~(0x1UL << 13U);
  GPIOB->OTYPER  |=   (0x0UL << 13U); // OT13 <- 0x0 for output push-pull
  GPIOB->OSPEEDR &=  ~(0x3UL << 26U);
  GPIOB->OSPEEDR |=   (0x3UL << 26U); // OSPEEDER13[1:0] <- 0x11 for very high speed
  GPIOB->PUPDR   &=  ~(0x3UL << 26U);
  GPIOB->PUPDR   |=   (0x0UL << 26U); // PUPDR13[1:0] <- 0x00 for no pull-up/pull_down
  GPIOB->AFR[1]  &=  ~(0xFUL << 20U);
  GPIOB->AFR[1]  |=   (0xEUL << 20U); // AFR13[3:0] <- 0x1110 to set PB13 as AFR14 (UART5)
  
  // Set PB12 for RX UART5 
  GPIOB->MODER   &=  ~(0x3UL << 24U);
  GPIOB->MODER   |=   (0x2UL << 24U); // MODER12[1:0] <- 0x10 for alternate function mode
  GPIOB->OTYPER  &=  ~(0x1UL << 12U);
  GPIOB->OTYPER  |=   (0x0UL << 12U); // OT12 <- 0x0 for output push-pull
  GPIOB->OSPEEDR &=  ~(0x3UL << 24U);
  GPIOB->OSPEEDR |=   (0x3UL << 24U); // OSPEEDER12[1:0] <- 0x11 for very high speed
  GPIOB->PUPDR   &=  ~(0x3UL << 24U);
  GPIOB->PUPDR   |=   (0x0UL << 24U); // PUPDR12[1:0] <- 0x00 for no pull-up/pull_down
  GPIOB->AFR[1]  &=  ~(0xFUL << 16U);
  GPIOB->AFR[1]  |=   (0xEUL << 16U); // AFR12[3:0] <- 0x1110 to set PB12 as AFR14 (UART5)
  
  // Set baudrate (oversampling by 16)
  uint16_t uartdiv = 120000000 / 38400;
  UART5->BRR = uartdiv;
  
  
  // Enable the USART peripheral for transmission.
  UART5->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE ); 

}

/**
  * System Clock Configuration
  */
void SystemClock_Config(void)
{
    uint32_t __attribute((unused)) tmpreg ; 

    /**  1) Boost the voltage scaling level to VOS0 to reach system maximum frequency **/
	
    // Supply configuration update enable
    MODIFY_REG (PWR->CR3, (PWR_CR3_SCUEN | PWR_CR3_LDOEN | PWR_CR3_BYPASS),  PWR_CR3_LDOEN);
    for(int i=0; i<1500000;i++){__asm__("nop");}
  
    // Configure the Voltage Scaling 1 in order to modify ODEN bit 
    MODIFY_REG(PWR->D3CR, PWR_D3CR_VOS, (0x2UL << 14U));
    // Delay after setting the voltage scaling 
    tmpreg = READ_BIT(PWR->D3CR, PWR_D3CR_VOS);
    // Enable the PWR overdrive
    SET_BIT(SYSCFG->PWRCR, SYSCFG_PWRCR_ODEN);
    // Delay after setting the syscfg boost setting 
    tmpreg = READ_BIT(SYSCFG->PWRCR, SYSCFG_PWRCR_ODEN);

    // Wait for VOS to be ready
    while( (PWR->D3CR & PWR_D3CR_VOSRDY) != PWR_D3CR_VOSRDY) {}

	/** 2) Oscillator initialisation **/

	//Enable HSE
	RCC->CR |= RCC_CR_HSEON;
	// Wait till HSE is ready
	while((RCC->CR & RCC_CR_HSERDY) == 0);

	// Switch (disconnect)
	RCC->CFGR |= 0x2UL;                  // Swich to HSE temporarly
	while((RCC->CFGR & RCC_CFGR_SWS) != (0x00000010UL));
	RCC->CR   &= ~1;				 // Disable HSI
	RCC->CR   &= ~(0x1UL << 24U);	// Disable PLL
	while((RCC->CR & RCC_CR_PLL1RDY) != 0); // wait for PPL to be disabled

    // Config PLL
	//RCC -> PLLCKSELR |= RCC_PLLCKSELR_PLLSRC_HSE; //RCC -> PLLCKSELR |= (0x05UL << 4U);
	//MODIFY_REG(RCC->PLLCKSELR, (RCC_PLLCKSELR_PLLSRC ) , (RCC_PLLSOURCE_HSE) );
	RCC -> PLLCKSELR &= ~(0b111111UL << 4U); // reset bit
	RCC -> PLLCKSELR |= (0x05UL << 4U);
	RCC -> PLLCKSELR |= RCC_PLLCKSELR_PLLSRC_HSE;
	//MODIFY_REG(RCC->PLLCKSELR, ( RCC_PLLCKSELR_DIVM1) , ( (5) <<4U) );

	// DIVN = 192, DIVP = 2, DIVQ = 2, DIVR = 2.
	RCC -> PLL1DIVR  |= (0xBFUL << 0U);
	RCC -> PLL1DIVR  |= (0x01UL << 9U);
	RCC -> PLL1DIVR  |= (0x01UL << 16U);
	RCC -> PLL1DIVR  |= (0x01UL << 24U);

	// Disable PLLFRACN
	RCC->PLLCFGR &= ~(0x1UL << 0U);

	//  Configure PLL  PLL1FRACN //__HAL_RCC_PLLFRACN_CONFIG(RCC_OscInitStruct->PLL.PLLFRACN);
	RCC -> PLL1FRACR = 0;

	//Select PLL1 input reference frequency range: VCI //__HAL_RCC_PLL_VCIRANGE(RCC_OscInitStruct->PLL.PLLRGE) ;
	//RCC->PLLCFGR |= RCC_PLLCFGR_PLL1RGE_3;
	RCC->PLLCFGR |= (0x2UL << 2U);

	// Select PLL1 output frequency range : VCO //__HAL_RCC_PLL_VCORANGE(RCC_OscInitStruct->PLL.PLLVCOSEL) ;
	//RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL1VCOSEL;
	RCC->PLLCFGR |= (0x0UL << 1U);

	// Enable PLL System Clock output. // __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVP);//Bit 16 DIVP1EN: PLL1 DIVP divider output enable
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVP1EN;

	// Enable PLL1Q Clock output. //__HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVQ);
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVQ1EN;

	// Enable PLL1R  Clock output. // __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVR);
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVR1EN;

	// Enable PLL1FRACN . //__HAL_RCC_PLLFRACN_ENABLE();
	RCC->PLLCFGR |= RCC_PLLCFGR_PLL1FRACEN;

	// Enable the main PLL. //__HAL_RCC_PLL_ENABLE();
	RCC->CR |= RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLL1RDY) == 0);

	/** 3) Clock initialisation **/

	//HPRE[3:0]: D1 domain AHB prescaler //1000: rcc_hclk3 = sys_d1cpre_ck / 2
	RCC -> D1CFGR |= (0x08UL << 0U);


	//D1CPRE[3:0]: D1 domain Core prescaler //0xxx: sys_ck not divided (default after reset)
	RCC -> D1CFGR |= (0x0UL << 8U);

	//SW[2:0]: System clock switch//011: PLL1 selected as system clock (pll1_p_ck)
	RCC->CFGR |= (0b011 << 0U);
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL1);

	//D1PPRE[2:0]: D1 domain APB3 prescaler//100: rcc_pclk3 = rcc_hclk3 / 2
	RCC->D1CFGR   |= (0b100 << 4U);


	//D2PPRE1[2:0]: D2 domain APB1 prescaler//100: rcc_pclk1 = rcc_hclk1 / 2
	RCC -> D2CFGR |=  (0b100 << 4U);

	//D2PPRE2[2:0]: D2 domain APB2 prescaler//100: rcc_pclk2 = rcc_hclk1 / 2
	RCC -> D2CFGR |=  (0b100 << 8U);


	//D3PPRE[2:0]: D3 domain APB4 prescaler//100: rcc_pclk4 = rcc_hclk4 / 2
	RCC -> D3CFGR |=  (0b100 << 4U);

	//Update global variables
	const  uint8_t D1CorePrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
	SystemD2Clock = (480000000 >> ((D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_HPRE)>> RCC_D1CFGR_HPRE_Pos]) & 0x1FU));
	SystemCoreClock = 480000000;
}

/**
  * Simulate a time delay 
  */
void delay(int comp)
{
for(int i=0; i < comp; i++){__asm__("nop");}
}

/**
  * Send in blocking mode
  */
void UART_send_blocking(uint8_t* byte)
{
    while(!(UART5->ISR & USART_ISR_TXE_TXFNF)){}; // wait for empty transmit register
    UART5->TDR = *byte;
}

/**
  * Receiving in blockin mode 
  */
void UART_rcv_blocking(uint8_t* byte)
{
    while(!(UART5->ISR & USART_ISR_RXNE_RXFNE)){}; // wait for non empty read register
    *byte = UART5->RDR;

}
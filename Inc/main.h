/*
 * main.h
 *This header file contains type definitions, macro definitions, and inline functions
 * for setting up system clocks, configuring delays using the SysTick timer, and various
 * utility functions for the STM32F4 microcontroller.
 *
 *
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include <stdio.h>
#include "math.h"
#include "inttypes.h"
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"
#include "stdbool.h"
#include "stdint.h"
#include "system_stm32f4xx.h"

// Structure for managing interrupt settings
typedef struct Interrupts
{
	bool Enable;// Flag to enable or disable the interrupt
	uint32_t Interrup_Flags;// Flags for different interrupt settings
}Interrupts;

#define __weak   __attribute__((weak))

#define SPI_Debug_Flag 0// Debug flag for SPI, used to enable or disable SPI debugging

extern uint32_t APB1CLK_SPEED;  // External variable to store the APB1 peripheral bus clock speed
extern uint32_t APB2CLK_SPEED;  // External variable to store the APB2 peripheral bus clock speed

/**
 * @brief Get the clock speed of the APB1 peripheral bus
 *
 * This function calculates the clock speed of the APB1 peripheral bus by using the
 * system core clock and the APB1 prescaler value from the RCC_CFGR register.
 *
 * @return int32_t Clock speed in Hz
 */
__STATIC_INLINE int32_t SystemAPB1_Clock_Speed(void)
{
	return (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos]);
}

/**
 * @brief Get the clock speed of the APB2 peripheral bus
 *
 * This function calculates the clock speed of the APB2 peripheral bus by using the
 * system core clock and the APB2 prescaler value from the RCC_CFGR register.
 *
 * @return int32_t Clock speed in Hz
 */

__STATIC_INLINE int32_t SystemAPB2_Clock_Speed(void)
{
	return (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2)>> RCC_CFGR_PPRE2_Pos]);
}
/**
 * @brief Set up the MCU system clock
 *
 * This function configures the PLL to set up the system clock. It includes steps to enable
 * the HSE (High-Speed External oscillator), configure the PLL settings, and set the system
 * and peripheral clock dividers. It also configures the SysTick timer to generate a
 * periodic interrupt.
 */

__STATIC_INLINE void MCU_Clock_Setup(void)
{
//	uint8_t pll_m = 4;
//	uint8_t pll_n = 168; //192
//	uint8_t pll_p = 0;
//	uint8_t pll_q = 7;

	uint8_t pll_m = 8;   // PLL division factor for the VCO input clock
	uint16_t pll_n = 336; //192  // PLL multiplication factor for the VCO output clock
	uint8_t pll_p = 0;// PLL division factor for the system clock
	uint8_t pll_q = 7; // PLL division factor for the I2S clock

	RCC->PLLCFGR = 0x00000000;// Reset PLL configuration register
	RCC -> CR |= RCC_CR_HSEON;// Enable HSE
	while(!(RCC -> CR & RCC_CR_HSERDY)){}// Wait until HSE is ready
	RCC -> APB1ENR |= RCC_APB1ENR_PWREN;// Enable power interface clock
	PWR ->CR |= PWR_CR_VOS;    // Set voltage scaling
	FLASH -> ACR |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;// Configure Flash memory
	// Configure PLL
	RCC->PLLCFGR |= (pll_q << 24) | (pll_p << 16) | (pll_n << 6) | (pll_m << 0);
	RCC ->PLLCFGR |= 1 << 22;// Enable PLL
	RCC -> CFGR |= RCC_CFGR_HPRE_DIV1;// Set AHB prescaler
	RCC -> CFGR |= RCC_CFGR_PPRE1_DIV4;// Set APB1 prescaler
	RCC -> CFGR |= RCC_CFGR_PPRE2_DIV2;// Set APB2 prescaler





	RCC -> CR |= RCC_CR_PLLON; // Enable PLL



	while(!(RCC->CR & RCC_CR_PLLRDY)){}// Wait until PLL is ready
	RCC -> CFGR |= RCC_CFGR_SW_PLL;// Switch to PLL as system clock source
	while((RCC -> CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);// Wait until PLL is used as system clock source
	SystemCoreClockUpdate(); // Update SystemCoreClock variable
	SysTick_Config(SystemCoreClock/168); // Configure SysTick for 1 ms time base
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;// Enable system configuration controller clock
}
/**
 * @brief Initialize I2S clock
 *
 * This function configures the PLLI2S settings to provide the clock for the I2S peripheral.
 * It involves setting up PLLI2S multiplication and division factors and enabling the PLLI2S.
 *
 * @return int Status of the initialization (0 for success)
 */


__STATIC_INLINE int I2S_Clock_Init()
{
//	int plli2s_m = 25; //25 25 4
//	int plli2s_n = 344; //344 192 50
//	int plli2s_r = 2; //2 5 2
//	RCC -> PLLI2SCFGR = (plli2s_m << 0) | (plli2s_n << 6) | (plli2s_r << 28);
//	RCC -> CR |= RCC_CR_PLLI2SON;
//	while(!(RCC -> CR & RCC_CR_PLLI2SRDY));

	uint32_t RCC_PLLI2SCFGR = 0;
	uint32_t plli2s_n = 384;// PLLI2S multiplication factor
	uint32_t plli2s_r = 5; // PLLI2S division factor
	RCC_PLLI2SCFGR = plli2s_n << 6;// Set PLLI2S N value
	RCC_PLLI2SCFGR |= plli2s_r << 28;// Apply configuration to PLLI2S
	RCC -> PLLI2SCFGR = RCC_PLLI2SCFGR;// Enable PLLI2S
	RCC -> CR |= RCC_CR_PLLI2SON;// Apply configuration to PLLI2S
	while(!(RCC -> CR & RCC_CR_PLLI2SRDY));// Wait until PLLI2S is ready

	return (0UL);
}
/**
 * @brief Configure the SysTick timer for delay generation
 *
 * This function sets up the SysTick timer for delay generation. It initializes the
 * SysTick timer with a maximum value and enables it.
 *
 * @return uint32_t Status of the configuration (0 for success)
 */


__STATIC_INLINE uint32_t Delay_Config(void)
{

	SysTick->CTRL = 0;// Disable SysTick timer
	SysTick->LOAD = 0x00FFFFFF;// Set maximum reload value
	SysTick->VAL = 0;// Clear current value
	SysTick->CTRL = 5;// Enable SysTick timer and its interrupt

	return (0UL);  // Function successful                                                   /* Function successful */
}
/**
 * @brief De-initialize the SysTick timer
 *
 * This function disables the SysTick timer interrupt.
 */

__STATIC_INLINE void Delay_DeInit(void)
{
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

/**
 * @brief Generate a delay in microseconds
 *
 * This function generates a delay of the specified duration in microseconds using
 * the SysTick timer. The delay is achieved by adjusting the SysTick LOAD register
 * and waiting for the timer to count down.
 *
 * @param us Delay duration in microseconds
 * @return uint32_t Status of the delay (0 for success)
 */
__STATIC_INLINE uint32_t Delay_us(float us)
{

	SysTick->LOAD = 167 * us;// Set delay value for microseconds
	SysTick->VAL = 0;// Clear current value
	while((SysTick->CTRL & 0x00010000) == 0); // Wait until delay is complete
	return (0UL);  /* Function successful */
}
/**
 * @brief Generate a delay in milliseconds
 *
 * This function generates a delay of the specified duration in milliseconds using
 * the SysTick timer. The delay is achieved by adjusting the SysTick LOAD register
 * and waiting for the timer to count down.
 *
 * @param ms Delay duration in milliseconds
 * @return uint32_t Status of the delay (0 for success)
 */

__STATIC_INLINE uint32_t Delay_ms(float ms)
{
	unsigned long x =0x29040 * (ms);
	SysTick->LOAD =  x ;
	SysTick->VAL = 0;
	SysTick->CTRL |= 1;
	while((SysTick->CTRL & 0x00010000) == 0);
	return (0UL);                                                     /* Function successful */
}
/**
 * @brief Generate a delay in seconds
 *
 * This function generates a delay of the specified duration in seconds by converting
 * the delay into milliseconds and calling the Delay_ms function repeatedly.
 *
 * @param s Delay duration in seconds
 * @return uint32_t Status of the delay (0 for success)
 */


__STATIC_INLINE uint32_t Delay_s(unsigned long s)
{
	s = s * 1000;
	for (; s>0; s--)
	{
		Delay_ms(1);
	}
	return (0UL);
}


__STATIC_INLINE float Time_Stamp_Start(void)
{
	float temp = 0;
	SysTick->CTRL = 0;// Disable SysTick timer
	SysTick->LOAD = 0xFFFFFFFF;// Set maximum reload value
	SysTick->VAL = 0;// Clear current value
	SysTick->CTRL = 0x5;// Enable SysTick timer and its interrupt
	while(SysTick->VAL != 0);// Wait until timer counts down
	temp = (float)(SysTick->VAL / (SystemCoreClock));// Capture time stamp value
	return temp;
}
/**
 * @brief End a time stamp measurement using SysTick
 *
 * This function ends a time stamp measurement using the SysTick timer. It captures
 * the final value of the SysTick counter and returns it as a float.
 *
 * @return float Time stamp end value
 */

__STATIC_INLINE float Time_Stamp_End(void)
{
	float temp = 0;
	temp = (float)(SysTick->VAL / (SystemCoreClock)); // Capture time stamp value
	return temp;
}
/**
 * @brief Separate a floating-point number into integral and fractional parts
 *
 * This function separates a floating-point number into its integral and fractional
 * parts. The results are stored in the provided pointers.
 *
 * @param number The input number
 * @param fractionalPart Pointer to store the fractional part
 * @param integralPart Pointer to store the integral part
 */
__STATIC_INLINE void separateFractionAndIntegral(double number, double *fractionalPart, double *integralPart) {
    *integralPart = (double)((int64_t)number);// Extract integral part
    *fractionalPart = number - *integralPart;// Calculate fractional part
}

#endif /* MAIN_H_ */

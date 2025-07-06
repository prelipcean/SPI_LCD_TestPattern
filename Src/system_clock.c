/*
 * system_clock.c
 *
 * Legend:
 * PLL = phase-locked loop
 * VCO = voltage controlled oscillator
 *
 */

#include "sytem_clock.h"

#define OVERDRIVE

/*
 * RCC PLL configuration register (RCC_PLLCFGR)
 * Address offset: 0x04
 * Reset value: 0x2400 3010
 * Access: no wait state, word, half-word and byte access.
 * This register is used to configure the PLL clock outputs according to the formulas:
 * f(VCO clock) = f(PLL clock input) × (PLLN / PLLM)
 *
 */

/*
 * The maximum frequency of the AHB domain is 180 MHz.
 * The maximum allowed frequency of the high-speed APB2 domain is 90 MHz.
 * The maximum allowed frequency of the low-speed APB1 domain is 45 MHz
 *
 */
void SystemClock_Setup(void)
{
	RCC_TypeDef 	*pRCC 	= (RCC_TypeDef *) RCC_BASE; // point to base address of RCC
	FLASH_TypeDef 	*pFlash = FLASH;
	PWR_TypeDef 	*pPWR 	= PWR;

	// Note: pay attention to datasheet cautions when setting the values and
	// the bits to be set in a register to have a desired value

	/* Program flash wait states */
	REG_SET_VAL(pFlash->ACR, 0x05U, 0x0FU, FLASH_ACR_LATENCY_Pos);

#ifdef OVERDRIVE
	/* Over drive settings */
	// Enable clock for PWR register access
	REG_SET_BIT(pRCC->APB1ENR, RCC_APB1ENR_PWREN_Pos);
	// VOS = 0b11 Scale 1 mode
	REG_SET_VAL(pPWR->CR, 0x03U, 0x03, PWR_CR_VOS_Pos);
	// Activate over drive mode
	REG_SET_BIT(pPWR->CR, PWR_CR_ODEN_Pos);
	// Wait for over drive ready
	while(! REG_READ_BIT(pPWR->CSR, PWR_CSR_ODRDY_Pos));
	// Over drive switch enable
	REG_SET_BIT(pPWR->CR, PWR_CR_ODSWEN_Pos);
#endif

	/* Configure main PLL parameters */
	// Set up PLLCLK (SYSCLK) to 180Mhz
	// Set PLLM bits 0 to 5 to value 8
	REG_SET_VAL(pRCC->PLLCFGR, 0x08U, 0x3FU, /*0U*/ RCC_PLLCFGR_PLLM_Pos);

	// Set PLLN bits 6 to 14 to value 180
	REG_SET_VAL(pRCC->PLLCFGR, 0xB4U, 0x1FFU, /*6U*/ RCC_PLLCFGR_PLLN_Pos);

	// Set PLLP bits 16 to 17 to value 00 which means 2 according to datasheet
	REG_SET_VAL(pRCC->PLLCFGR, 0x00U, 0x03U, /*16U*/ RCC_PLLCFGR_PLLP_Pos);

	/* Configure PLLSAI parameters */
	// We don't have LCD controller on our board to configure this PLL for LCD-TFT clock (pixel clock)

	/* Configure AHB and APBx prescalers */
	// Set AHB (HCLK) clock to 180MHz (SYSCLK / 1)
	REG_SET_VAL(pRCC->CFGR, 0x00U, 0x0FU, /*4U*/ RCC_CFGR_HPRE_Pos);

	// Set APB1 clock to 45MHz (HCLK / 4)
	REG_SET_VAL(pRCC->CFGR, 0x05U, 0x07U, /*10U*/ RCC_CFGR_PPRE1_Pos);

	// Set APB2 clock to 90 MHz (HCLK / 2)
	REG_SET_VAL(pRCC->CFGR, 0x04U, 0x07U, /*13U*/ RCC_CFGR_PPRE2_Pos);

	/* Turn on main PLL */
	REG_SET_BIT(pRCC->CR, RCC_CR_PLLON_Pos);

	/* Wait until PLLCLK ready bit is set */
	while(!REG_READ_BIT(pRCC->CR, RCC_CR_PLLRDY_Pos));

	/* Switch PLLCLK as SYSCLK */
	REG_SET_VAL(pRCC->CFGR, 0x02U, 0x03U, RCC_CFGR_SW_Pos);

	/* Wait for switch status */
	while(!(REG_READ_VAL(pRCC->CFGR, 0x03U, RCC_CFGR_SW_Pos) == 0x02U));

	/* Turn on PLLSAI */
	// Not needed in our case
	/* Wait until PLLSAICLK ready bit is set */
	// Not needed in our case
}


/*
 * Clock-out capability
 * Two microcontroller clock output (MCO) pins are available:
 * MCO1
 * You can output four different clock sources onto the MCO1 pin (PA8) using the
 * configurable prescaler (from 1 to 5):
 * – HSI clock
 * – LSE clock
 * – HSE clock
 * – PLL clock
 * The desired clock source is selected using the MCO1PRE[2:0] and MCO1[1:0] bits in
 * the RCC clock configuration register (RCC_CFGR).
 *
 * */
void SystemClock_Output(void)
{
	RCC_TypeDef 	*pRCC 	= (RCC_TypeDef *) RCC_BASE; // point to base address of RCC
    GPIO_TypeDef	*pGPIOA	= (GPIO_TypeDef *) GPIOA_BASE;

	// MCO1: Microcontroller clock output 1 => 0b11 PLL clock selected
	REG_SET_VAL(pRCC->CFGR, 0x03U, 0x03U, RCC_CFGR_MCO1_Pos);

	// MCO1PRE: MCO1 prescaler
	// 110: division by 4, for 168Mhz we should have 42MHz and for 180MHz we should have 45MHz
	REG_SET_VAL(pRCC->CFGR, 0x06U, 0x07U, RCC_CFGR_MCO1PRE_Pos);

	// Enable peripheral clock for GPIOA
	// GPIOAEN: IO port A clock enable => 1: IO port A clock enabled
	REG_SET_VAL(pRCC->AHB1ENR, 0x01U, 0x01U, RCC_AHB1ENR_GPIOAEN_Pos);

	// Configure mode for GPIOA pin 8 to alternate function mode (0b10)
	REG_SET_VAL(pGPIOA->MODER, 0x02U, 0x03U, GPIO_MODER_MODE8_Pos);

	// Configure speed. 0b11: High speed
	REG_SET_VAL(pGPIOA->OSPEEDR, 0x03U, 0x03U, GPIO_OSPEEDR_OSPEED8_Pos);

	//GPIO Alternate Function High Register (PA8 till PA15 are here the other are in Low Register)
	//Sets pin PA8 to mode 0 (0b0000 AF0)
	REG_SET_VAL(pGPIOA->AFR[1], 0x00U, 0x0FU, GPIO_AFRH_AFSEL8_Pos);
}

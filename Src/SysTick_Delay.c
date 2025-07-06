/*
 * SysTick_Delay.c
 *
 */

#include "main.h"
#include "reg_util.h"
#include "SysTick_Delay.h"

void SystTick_Init(void)
{
  SysTick_Type *pSYSTICK = SysTick;

  REG_WRITE(pSYSTICK->CTRL, 0x00U);
  REG_WRITE(pSYSTICK->LOAD, 0x00FFFFFFU);
  REG_WRITE(pSYSTICK->VAL, 0x00U);

  // CLKSOURCE: Clock source selection
  // 0b0: AHB/8
  //REG_SET_VAL(pSYSTICK->CTRL, 0x00U, 0x01, SysTick_CTRL_CLKSOURCE_Pos);
  REG_CLR_BIT(pSYSTICK->CTRL, SysTick_CTRL_CLKSOURCE_Pos);
  REG_SET_BIT(pSYSTICK->CTRL, SysTick_CTRL_ENABLE_Pos);
}

void DelayOneMs(void)
{
  SysTick_Type *pSYSTICK = SysTick;

  // 180 MHz system clock
  // 22.5 MHz AHB/8
  REG_WRITE(pSYSTICK->LOAD, (22500 - 1));
  REG_WRITE(pSYSTICK->VAL, 0x00U); // initialize value for the load

  while (!REG_READ_BIT(pSYSTICK->CTRL, SysTick_CTRL_COUNTFLAG_Pos));
}

void Delay_ms(uint32_t t)
{
  for (; t > 0; t--)
  {
    DelayOneMs();
  }
}

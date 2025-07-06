/*
 * bsp_lcd.c
 *
 * 1.8" Serial SPI 128x160 Color TFT LCD Module Display
 * Driver ST7735
 * 4-wire SPI interface
 * 18-bit color (262, 144)
 * Compatible with 3.3 V or 5 V systems
 *
 */

#include <string.h>
#include "bsp_lcd.h"
#include "ST7735.h"
#include "GFX_FUNCTIONS.h"

// Local functions
static void LCD_Pin_Init(void);
static void LCD_SPI_Init(void);
static void LCD_SPI_Enable(void);
static void LCD_Reset(void);
static void LCD_Config(void);

void BSP_LCD_Init(void)
{
  LCD_Pin_Init();
  LCD_SPI_Init();
  LCD_SPI_Enable();
  LCD_Reset();
  LCD_Config();
}

static void LCD_Pin_Init(void)
{
  RCC_TypeDef 	*pRCC 	= (RCC_TypeDef *) RCC_BASE; // point to base address of RCC
  GPIO_TypeDef	*pGPIOA	= (GPIO_TypeDef *) GPIOA_BASE;
  GPIO_TypeDef	*pGPIOB	= (GPIO_TypeDef *) GPIOB_BASE;
  GPIO_TypeDef	*pGPIOC	= (GPIO_TypeDef *) GPIOC_BASE;

  // Set OUTPUT pins

  // Enable peripheral clock for GPIOA, GPIOB and GPIOC
  // 0b1: IO port clock enabled
  REG_SET_VAL(pRCC->AHB1ENR, 0x01U, 0x01U, RCC_AHB1ENR_GPIOAEN_Pos);
  REG_SET_VAL(pRCC->AHB1ENR, 0x01U, 0x01U, RCC_AHB1ENR_GPIOBEN_Pos);
  REG_SET_VAL(pRCC->AHB1ENR, 0x01U, 0x01U, RCC_AHB1ENR_GPIOCEN_Pos);

  // Set pins as output for output pins PA9, PB6, PC7
  // 0b01: General purpose output mode
  REG_SET_VAL(pGPIOA->MODER, 0x01U, 0x03U, GPIO_MODER_MODE9_Pos);
  REG_SET_VAL(pGPIOB->MODER, 0x01U, 0x03U, GPIO_MODER_MODE6_Pos);
  REG_SET_VAL(pGPIOC->MODER, 0x01U, 0x03U, GPIO_MODER_MODE7_Pos);

  // Set pins to push-pull type
  // 0b0: Output push-pull
  REG_SET_VAL(pGPIOA->OTYPER, 0x00U, 0x01U, GPIO_OTYPER_OT9_Pos);
  REG_SET_VAL(pGPIOB->OTYPER, 0x00U, 0x01U, GPIO_OTYPER_OT6_Pos);
  REG_SET_VAL(pGPIOC->OTYPER, 0x00U, 0x01U, GPIO_OTYPER_OT7_Pos);

  // Set speed for pins
  // 0b10: High speed
  REG_SET_VAL(pGPIOA->OSPEEDR, 0x02U, 0x03U, GPIO_OSPEEDR_OSPEED9_Pos);
  REG_SET_VAL(pGPIOB->OSPEEDR, 0x02U, 0x03U, GPIO_OSPEEDR_OSPEED6_Pos);
  REG_SET_VAL(pGPIOC->OSPEEDR, 0x02U, 0x03U, GPIO_OSPEEDR_OSPEED7_Pos);

  // Set no pull up or pull down for pins
  // 0b00: No pull up, no pull down
  REG_SET_VAL(pGPIOA->PUPDR, 0x00U, 0x03U, GPIO_PUPDR_PUPD9_Pos);
  REG_SET_VAL(pGPIOB->PUPDR, 0x00U, 0x03U, GPIO_PUPDR_PUPD6_Pos);
  REG_SET_VAL(pGPIOC->PUPDR, 0x00U, 0x03U, GPIO_PUPDR_PUPD7_Pos);

  // Set SPI pins

  // Configure mode for GPIOA pin 5 and 7 to alternate function mode (0b10)
  REG_SET_VAL(pGPIOA->MODER, 0x02U, 0x03U, GPIO_MODER_MODE5_Pos);
  REG_SET_VAL(pGPIOA->MODER, 0x02U, 0x03U, GPIO_MODER_MODE7_Pos);

  // Set pins to push-pull type
  // 0b0: Output push-pull
  REG_SET_VAL(pGPIOA->OTYPER, 0x00U, 0x01U, GPIO_OTYPER_OT5_Pos);
  REG_SET_VAL(pGPIOA->OTYPER, 0x00U, 0x01U, GPIO_OTYPER_OT7_Pos);

  // Set speed for pins
  // 0b10: Fast speed
  REG_SET_VAL(pGPIOA->OSPEEDR, 0x02U, 0x03U, GPIO_OSPEEDR_OSPEED5_Pos);
  REG_SET_VAL(pGPIOA->OSPEEDR, 0x02U, 0x03U, GPIO_OSPEEDR_OSPEED7_Pos);

  //GPIO Alternate Function Low Register (PA8 till PA15 in High Register)
  //Sets pin PA5 and PA7 to mode 5 (0b0101 AF5)
  REG_SET_VAL(pGPIOA->AFR[0], 0x05U, 0x0FU, GPIO_AFRL_AFSEL5_Pos);
  REG_SET_VAL(pGPIOA->AFR[0], 0x05U, 0x0FU, GPIO_AFRL_AFSEL7_Pos);

  // set CSX (PB6) to HIGH
  REG_SET_BIT(pGPIOB->ODR, GPIO_ODR_OD6_Pos);
  // set RESX (PC7) to HIGH
  REG_SET_BIT(pGPIOC->ODR, GPIO_ODR_OD7_Pos);
  // set D/CX (PA9) to HIGH
  REG_SET_BIT(pGPIOA->ODR, GPIO_ODR_OD9_Pos);
}

static void LCD_SPI_Init(void)
{
  /* We are controller and we control an external peripheral over SPI
   * SPI mode: Half-duplex Controller
   * Data format: 8 bit, msb first
   * CPOL (clock polarity) and CPHA (clock phase), check external peripheral datasheet
   * SPI Clock: check external peripheral datasheet (<6MHz always)
   * Chip select: handled by the software
   * */

  //SPI_TypeDef *pSPI1 = (SPI_TypeDef *) SPI1_BASE;
  SPI_TypeDef *pSPI1 	= LCD_SPI; // same as above
  // SPI1 is located on APB2 bus, see Table STM32F446xx register boundary addresses
  RCC_TypeDef *pRCC 	= RCC;

  // Enable SPI1 peripheral clock
  REG_SET_BIT(pRCC->APB2ENR, RCC_APB2ENR_SPI1EN_Pos);

  // MSTR: Master selection
  // Set SPI1 to Master (Controller mode), 0b1: Master configuration
  REG_SET_BIT(pSPI1->CR1, SPI_CR1_MSTR_Pos);

  // BIDIMODE: Bidirectional data mode enable
  // 0b1: 1-line bidirectional data mode selected
  REG_SET_BIT(pSPI1->CR1, SPI_CR1_BIDIMODE_Pos);

  // BIDIOE: Output enable in bidirectional mode
  // 0b1: Output enabled (transmit-only mode)
  REG_SET_BIT(pSPI1->CR1, SPI_CR1_BIDIOE_Pos);

  // DFF: Data frame format
  // 0b0: 8-bit data frame format is selected for transmission/reception
  REG_CLR_BIT(pSPI1->CR1, SPI_CR1_DFF_Pos);

  // We enable SSM because we want to control the slave (peripheral) from software and don't let the hardware control it
  // SSM: Software slave management
  // 0b1: Software slave management enabled
  REG_SET_BIT(pSPI1->CR1, SPI_CR1_SSM_Pos);
  // SSI: Internal slave select
  REG_SET_BIT(pSPI1->CR1, SPI_CR1_SSI_Pos);

  // LSBFIRST: Frame format
  // 0b0: MSB transmitted first
  REG_CLR_BIT(pSPI1->CR1, SPI_CR1_LSBFIRST_Pos);

  // SPI clock = 90 MHz / 8 = 11.25 MHz
  // 0b010: fPCLK/8
  REG_SET_VAL(pSPI1->CR1, 0x02U, 0x07U, SPI_CR1_BR_Pos);

  // CPOL: Clock polarity
  // 0b0: CK to 0 when idle
  REG_CLR_BIT(pSPI1->CR1, SPI_CR1_CPOL_Pos);

  // CPHA: Clock phase
  // 0b0: The first clock transition is the first data capture edge
  REG_CLR_BIT(pSPI1->CR1, SPI_CR1_CPHA_Pos);

  // FRF: Frame format
  // 0b0: SPI Motorola mode
  REG_CLR_BIT(pSPI1->CR2, SPI_CR2_FRF_Pos);
}

static void LCD_SPI_Enable(void)
{
  SPI_TypeDef *pSPI1 	= LCD_SPI;

  // SPE: SPI enable
  // 0b1: Peripheral enabled
  REG_SET_BIT(pSPI1->CR1, SPI_CR1_SPE_Pos);
}

static void LCD_Reset(void)
{
  LCD_RESX_HIGH();
  Delay_ms(100);
  LCD_RESX_LOW();
  Delay_ms(200);
  LCD_RESX_HIGH();
  Delay_ms(100);
}

static void LCD_Config(void)
{
  ST7735_Init(0);
  fillScreen(BLACK);
}

void LCD_Write_Cmd(uint8_t cmd)
{
  SPI_TypeDef *pSPI1 	= LCD_SPI;

  // Assert CS (make it low)
  LCD_CSX_LOW();

  LCD_DCX_LOW(); //DCX = 0 , for command

  // See first is Tx buffer is empty, if not wait for it to be empty
  while(!REG_READ_BIT(pSPI1->SR, SPI_SR_TXE_Pos));

  // Now we can transmit
  REG_WRITE(pSPI1->DR, cmd);
  while(!REG_READ_BIT(pSPI1->SR,SPI_SR_TXE_Pos)); // see datasheet why we do this here
  // Before deaserting lines wait for busy flag clear
  while(REG_READ_BIT(pSPI1->SR, SPI_SR_BSY_Pos));

  LCD_DCX_HIGH();
  LCD_CSX_HIGH();
}

void LCD_Write_Data(uint8_t *buffer, uint32_t len)
{
  SPI_TypeDef *pSPI1 	= LCD_SPI;

  for(uint32_t i = 0 ; i < len ;i++)
  {
    // Assert CS (make it low)
    LCD_CSX_LOW();
    // DSX is already HIGH

    // See first is Tx buffer is empty, if not wait for it to be empty
    while(!REG_READ_BIT(pSPI1->SR, SPI_SR_TXE_Pos));

    // Now we can transmit
    REG_WRITE(pSPI1->DR, buffer[i]);
    while(!REG_READ_BIT(pSPI1->SR,SPI_SR_TXE_Pos));// see datasheet why we do this here
    // Before deaserting lines wait for busy flag clear
    while(REG_READ_BIT(pSPI1->SR,SPI_SR_BSY_Pos));
    LCD_CSX_HIGH();
  }
}

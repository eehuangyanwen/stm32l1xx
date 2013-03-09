/**
  ******************************************************************************
  * @file    stm32l1xx_gpio.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-December-2010
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the GPIO peripheral:           
  *           - Initialization and Configuration
  *              ≥ı ºªØº∞≈‰÷√GPIO
  *           - GPIO Read and Write
  *                ∂¡–¥GPIO 
  *           - GPIO Alternate functions configuration
  *              GPIO »Œ“‚π¶ƒ‹≈‰÷√
  *             ±÷”…Ë÷√ºƒ¥Ê∆˜RCC->AHBENR£®AHB ±÷” πƒ‹£¨”√”⁄ø™∆ÙGPIO ±÷”£©
  *ºƒ¥Ê∆˜ΩÈ…‹£∫≈‰÷√ºƒ¥Ê∆˜: 1°¢GPIOx_MODER(ƒ£ Ωºƒ¥Ê∆˜£©£®”√”⁄—°‘Òin/out/analog/AF)
  *                        2°¢GPIOx_OTYPER£® ‰≥ˆ¿‡–Õºƒ¥Ê∆˜)(”√”⁄—°‘Ò ‰≥ˆµƒ¿‡–Õpush-pull/open drain )
  *                        3°¢GPIOx_OSPEEDR£® ‰≥ˆÀŸ∂»ºƒ¥Ê∆˜)£®—°‘Ò ‰≥ˆÀŸ∂»0.4/2/10/40MHz)
  *                        4°¢GPIOx_PUPDR£®…œœ¬¿≠ºƒ¥Ê∆˜£©
  *             ˝æ›ºƒ¥Ê∆˜£∫1°¢GPIOx_IDR£® ‰»Î ˝æ›ºƒ¥Ê∆˜£©£®÷ª∂¡£©
  *                        2°¢GPIOx_ODR£® ‰≥ˆ ˝æ›ºƒ¥Ê∆˜£©£®”…GPIOx_BSRR register…Ë÷√£©
  *                        3°¢GPIOx_BSRR( ‰≥ˆºƒ¥Ê∆˜µƒ÷√∏¥Œªºƒ¥Ê∆˜)
  * 
  *  @verbatim
  *
  *          ===================================================================
  *                                 How to use this driver
  *          ===================================================================       
  *           1. Enable the GPIO AHB clock using RCC_AHBPeriphClockCmd()
  *           1°¢ πƒ‹GPIO AHB ±÷”Õ®π˝ RCC_AHBPeriphClockCmd()÷˜“™≈‰÷√GPIO->AHBENR‘⁄stm32lxx_rcc.c£©÷– 

  *           2. Configure the GPIO pin(s) using GPIO_Init()
              2°¢≈‰÷√GPIO PINS Õ®π˝GPIO_Init()÷˜“™≈‰÷√…œ√Êµƒ∏˜∏ˆºƒ¥Ê∆˜
  *              Four possible configuration are available for each pin:
                  √ø∏ˆpin”–4∏ˆ≈‰÷√∑Ω Ω
  *                - Input: Floating, Pull-up, Pull-down.
                   ‰»Î:–¸ø’£¨…œ¿≠£¨œ¬¿≠
  *                - Output: Push-Pull (Pull-up, Pull-down or no Pull)
                    ‰≥ˆ£∫Õ∆ÕÏ£®∞¸¿®…œ¿≠°¢œ¬¿≠°¢≤ª¿≠£©
  *                          Open Drain (Pull-up, Pull-down or no Pull).
                          ø™¬©£®∞¸¿®…œ¿≠°¢œ¬¿≠°¢≤ª¿≠£©
  *                  In output mode, the speed is configurable: Very Low, Low,
  *                  Medium or High.
                     ‰»Î ‰≥ˆƒ£ ΩÀŸ∂»ø…≈‰÷√Œ™0.4/2/10/40MHz ∂‘”¶”⁄Very Low, Low,
  *                  Medium or High.
  *                - Alternate Function: Push-Pull (Pull-up, Pull-down or no Pull)
  *                                      Open Drain (Pull-up, Pull-down or no Pull).
                     »Œ“‚π¶ƒ‹ƒ£ Ω£∫Õ∆ÕÏ£®∞¸¿®…œ¿≠°¢œ¬¿≠°¢≤ª¿≠£©
                                   ø™¬©£®∞¸¿®…œ¿≠°¢œ¬¿≠°¢≤ª¿≠£©
  *                - Analog: required mode when a pin is to be used as ADC channel,
  *                          DAC output or comparator input.
                     ƒ£ƒ‚–≈∫≈£∫”√”⁄ADC/DAC£¨ªÚ’ﬂ±»Ωœ∆˜µƒ ‰»Î
  * 
  *          3- Peripherals alternate function:
             3- Õ‚…Ë»Œ“‚π¶ƒ‹
  *              - For ADC, DAC and comparators, configure the desired pin in 
  *                analog mode using GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AN
                 - ∂‘”⁄ADC£¨DACº∞±»Ωœ∆˜£¨≈‰÷√π‹Ω≈≥…ƒ£ƒ‚ƒ£ Ω£¨Õ®π˝
                   GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AN
  *              - For other peripherals (TIM, USART...):
                 - ∂‘”⁄∆‰À˚Õ‚…Ë£®∂® ±∆˜°¢¥Æø⁄...)
  *                 - Connect the pin to the desired peripherals' Alternate 
  *                   Function (AF) using GPIO_PinAFConfig() function
                    -¡¨Ω”π‹Ω≈µΩƒø±ÍAF ”√ GPIO_PinAFConfig() 
  *                 - Configure the desired pin in alternate function mode using
  *                   GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
                     -≈‰÷√π‹Ω≈≥…AFmode”√GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
  *                 - Select the type, pull-up/pull-down and output speed via 
  *                   GPIO_PuPd, GPIO_OType and GPIO_Speed members
                    -—°‘Ò¿‡–Õ£®…œ¿≠œ¬¿≠°¢ ‰≥ˆÀŸ∂»£©Õ®π˝GPIO_PuPd, GPIO_OType and GPIO_Speed 
  *                 - Call GPIO_Init() function
                    - µ˜”√GPIO_Init()
  *        
  *          4. To get the level of a pin configured in input mode use GPIO_ReadInputDataBit()
             4°¢∂¡»°pinµÁ∆ΩÕ®π˝≈‰÷√ ‰»Îƒ£ Ω£® π”√GPIO_ReadInputDataBit()£©
  *          
  *          5. To set/reset the level of a pin configured in output mode use
  *             GPIO_SetBits()/GPIO_ResetBits()
             5°¢÷√∏¥Œªpin “™”√GPIO_SetBits()/GPIO_ResetBits()¿¥≈‰÷√ ‰≥ˆƒ£ Ω
  *               
  *          6. During and just after reset, the alternate functions are not 
  *             active and the GPIO pins are configured in input floating mode
  *             (except JTAG pins).
                ∏¥Œª∆⁄º‰ªÚ÷Æ∫Û£¨AF Œ¥±ªº§ªÓ£¨GPIO±ª≈‰÷√≥…–¸ø’ƒ£ Ω£®≥˝JTAGÕ‚£©
  *
  *          7. The LSE oscillator pins OSC32_IN and OSC32_OUT can be used as 
  *             general-purpose (PC14 and PC15, respectively) when the LSE
  *             oscillator is off. The LSE has priority over the GPIO function.
             7°¢LSE æß’Òπ‹Ω≈OSC32_IN°¢OSC32_OUTø…◊˜Œ™Õ®”√ƒ£ Ω£®∂‘”¶µƒ «PC14°¢PC15£©µ±LSE æß’Òπÿ±’µƒ ±∫Ú°£
                LSE ”≈œ»”⁄GPIO π¶ƒ‹
  *
  *          8. The HSE oscillator pins OSC_IN/OSC_OUT can be used as 
  *             general-purpose PH0 and PH1, respectively, when the HSE 
  *             oscillator is off. The HSE has priority over the GPIO function.
             8°¢HSE æß’Òπ‹Ω≈OSC_IN°¢OSC_OUTø…◊˜Œ™Õ®”√ƒ£ Ω£®∂‘”¶µƒ «PH0°¢PH1£©µ±HSE æß’Òπÿ±’µƒ ±∫Ú°£
                HSE ”≈œ»”⁄GPIO π¶ƒ‹
  *             
  *  @endverbatim        
  *
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_gpio.h"
#include "stm32l1xx_rcc.h"

/** @addtogroup STM32L1xx_StdPeriph_Driver
  * @{
  */

/** @defgroup GPIO 
  * @brief GPIO driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup GPIO_Private_Functions
  * @{
  */

/** @defgroup GPIO_Group1 Initialization and Configuration
 *  @brief   Initialization and Configuration
 *
@verbatim   
 ===============================================================================
                        Initialization and Configuration
 ===============================================================================  

@endverbatim
  * @{
  */

/**
  * @brief  Deinitializes the GPIOx peripheral registers to their default reset 
  *         values.
            ≥ı ºªØGPIOxºƒ¥Ê∆˜µΩ»± °÷µ,Õ®π˝≈‰÷√RCC_AHBRSTR
  *         By default, The GPIO pins are configured in input floating mode
  *         (except JTAG pins).
            »± °œ¬£¨GPIO ≈‰÷√≥… ‰»Î–¸ø’ƒ£ Ω£®≥˝¡ÀJTAG)
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
    ≤Œ ˝    GPIOx:xø…“‘Œ™(A, B, C, D, E or H)
  * @retval None
	 √ª”–∑µªÿ÷µ	   
  */
void GPIO_DeInit(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  /*’‚∏ˆ∂œ—‘∫Í”√”⁄ºÏ≤È∫Ø ˝≤Œ ˝,µ±≤Œ ˝¥ÌŒÛµ˜”√assert_failed£®‘⁄main.c)”…”√ªß…Ë÷√£¨±®∏Ê¥ÌŒÛµƒ‘¥Œƒº˛º∞–– ˝£¨’˝»∑≤ª∑µªÿ*/
  /*IS_GPIO_ALL_PERIPH()”√”⁄ºÏ≤‚≤Œ ˝ «∑Ò∫œ∏Ò£¨ «1£¨∑Ò0*/
  if(GPIOx == GPIOA)
  {
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOA, DISABLE);  
	/*÷ÿ∆ÙGPIOx,Õ®π˝≈‰÷√RCC_AHBRSTR*/
  }
  else if(GPIOx == GPIOB)
  {
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOB, DISABLE);
  }
  else if(GPIOx == GPIOC)
  {
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOC, DISABLE);
  }
  else if(GPIOx == GPIOD)
  {
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOD, DISABLE);
  }
  else if(GPIOx == GPIOE)
  {
      RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOE, ENABLE);
      RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOE, DISABLE);
  }
  else
  {
    if(GPIOx == GPIOH)
    {
      RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOH, ENABLE);
      RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOH, DISABLE);
    }
  }
}

/**
  * @brief  Initializes the GPIOx peripheral according to the specified 
  *         parameters in the GPIO_InitStruct.
            ≥ı ºªØGPIOx Õ‚…ËÕ®π˝≤Œ ˝GPIOx(…Ë÷√mode\speed\PuPd\otype)
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
    ≤Œ ˝    GPIOx(A, B, C, D, E or H)
  * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure that 
  *         contains the configuration information for the specified GPIO
  *         peripheral.
  *         GPIO_Pin: selects the pin to be configured: GPIO_Pin_0 -> GPIO_Pin_15
  *         GPIO_Mode: selects the mode of the pin: 
  *                      - Input mode: GPIO_Mode_IN
  *                      - Output mode: GPIO_Mode_OUT
  *                      - Alternate Function mode: GPIO_Mode_AF
  *                      - Analog mode: GPIO_Mode_AN
  *         GPIO_Speed: selects the speed of the pin if configured in Output:
  *                      - Very Low: GPIO_Speed_400KHz
  *                      - Low: GPIO_Speed_2MHz
  *                      - Medium: GPIO_Speed_10MHz
  *                      - High: GPIO_Speed_40MHz
  *         GPIO_OType: selects the Output type (if the selected mode is output):
  *                      - Push-pull: GPIO_OType_PP
  *                      - Open Drain: GPIO_OType_OD
  *         GPIO_PuPd: configures the Pull-up/Pull-down resistor on the pin:
  *                      - pull-up: GPIO_PuPd_UP
  *                      - pull-down: GPIO_PuPd_DOWN
  *                      - Neither pull-up nor Pull-down: GPIO_PuPd_NOPULL
  ≤Œ ˝Œ™1°¢GPIO_TypeDef* GPIOx£®ø…ÃÓGPIOx)”√”⁄≈‰÷√…œ ˆµƒºƒ¥Ê∆˜µƒ
        2°¢GPIO_InitTypeDef* GPIO_InitStruct ”√”⁄ÃÓ–¥”√ªß–Ë“™µƒ¿‡–Õ°¢ÀŸ∂»°¢ ‰≥ˆƒ£ Ω°¢…œ¿≠œ¬¿≠µ»£©
  * @retval None
  */
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct)
{
  uint32_t pinpos = 0x00, pos = 0x00 , currentpin = 0x00;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_InitStruct->GPIO_Pin));
  assert_param(IS_GPIO_MODE(GPIO_InitStruct->GPIO_Mode));
  assert_param(IS_GPIO_PUPD(GPIO_InitStruct->GPIO_PuPd));

  /* -------------------------Configure the port pins---------------- */
  /*-- GPIO Mode Configuration --*/
  for (pinpos = 0x00; pinpos < 0x10; pinpos++)
  {
    pos = ((uint32_t)0x01) << pinpos;

    /* Get the port pins position */
    currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;

    if (currentpin == pos)
    {
      GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (pinpos * 2));/*“ª∏ˆpin’º2Œª£¨“Ú¥À±ÿ–Î“∆∂Øpinpos * 2,œ»«Âø’*/

      GPIOx->MODER |= (((uint32_t)GPIO_InitStruct->GPIO_Mode) << (pinpos * 2));/*“ª∏ˆpin’º2Œª£¨“Ú¥À±ÿ–Î“∆∂Øpinpos * 2,‘Ÿ∏≥÷µ*/

      if ((GPIO_InitStruct->GPIO_Mode == GPIO_Mode_OUT) || (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_AF))
      {  /*»Ù—°‘Ò ‰≥ˆƒ£ ΩªÚ’ﬂAFƒ£ Ω,≈‰÷√ÀŸ∂»°¢ ‰≥ˆ¿‡–Õ°¢*/
        /* Check Speed mode parameters */
        assert_param(IS_GPIO_SPEED(GPIO_InitStruct->GPIO_Speed));

        /* Speed mode configuration */
        GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));/*“ª∏ˆpin’º2Œª£¨“Ú¥À±ÿ–Î“∆∂Øpinpos * 2,œ»«Âø’*/
        GPIOx->OSPEEDR |= ((uint32_t)(GPIO_InitStruct->GPIO_Speed) << (pinpos * 2));/*“ª∏ˆpin’º2Œª£¨“Ú¥À±ÿ–Î“∆∂Øpinpos * 2,‘Ÿ∏≥÷µ*/

        /*Check Output mode parameters */
        assert_param(IS_GPIO_OTYPE(GPIO_InitStruct->GPIO_OType));

        /* Output mode configuration */
        GPIOx->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pinpos)) ;/*“ª∏ˆpin’º1Œª£¨“Ú¥À±ÿ–Î“∆∂Øpinpos ,œ»«Âø’*/
        GPIOx->OTYPER |= (uint16_t)(((uint16_t)GPIO_InitStruct->GPIO_OType) << ((uint16_t)pinpos));/*“ª∏ˆpin’º1Œª£¨“Ú¥À±ÿ–Î“∆∂Øpinpos ,‘Ÿ∏≥÷µ*/
      }
       /*…œœ¬¿≠*/
      /* Pull-up Pull down resistor configuration */
      GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pinpos * 2));/*“ª∏ˆpin’º2Œª£¨“Ú¥À±ÿ–Î“∆∂Øpinpos * 2,œ»«Âø’*/
      GPIOx->PUPDR |= (((uint32_t)GPIO_InitStruct->GPIO_PuPd) << (pinpos * 2));/*“ª∏ˆpin’º2Œª£¨“Ú¥À±ÿ–Î“∆∂Øpinpos*2 ,‘Ÿ∏≥÷µ*/
      }
    }
  }
}

/**
  * @brief  Fills each GPIO_InitStruct member with its default value.
		    ≥ı ºªØGPIO_InitStructµΩ∆‰»± °≈‰÷√
  * @param  GPIO_InitStruct : pointer to a GPIO_InitTypeDef structure which will 
  *         be initialized.
  * @retval None
  */
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct)
{
  /* Reset GPIO init structure parameters values */
  GPIO_InitStruct->GPIO_Pin  = GPIO_Pin_All;
  GPIO_InitStruct->GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct->GPIO_Speed = GPIO_Speed_400KHz;
  GPIO_InitStruct->GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct->GPIO_PuPd = GPIO_PuPd_NOPULL;
}

/**
  * @brief  Locks GPIO Pins configuration registers.
  *         The locked registers are GPIOx_MODER, GPIOx_OTYPER, GPIOx_OSPEEDR,
  *         GPIOx_PUPDR, GPIOx_AFRL and GPIOx_AFRH.
  *         The configuration of the locked GPIO pins can no longer be modified
  *         until the next reset.
  *         À¯¥Êπ‹Ω≈≈‰÷√£¨£®GPIOx_MODER, GPIOx_OTYPER, GPIOx_OSPEEDR,
  *         GPIOx_PUPDR, GPIOx_AFRL and GPIOx_AFRH.£©£¨‘⁄œ¬¥Œ∏¥Œª«∞≤ªƒ‹‘Ÿ∏¸∏ƒ°£
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
  * ≤Œ ˝GPIO_TypeDef* GPIOx(A, B, C, D, E or H)
  * @param  GPIO_Pin: specifies the port bit to be written.
    ≤Œ ˝uint16_t GPIO_Pin£®GPIO_Pin_x£©0-15
  *   This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
  * @retval None
  */
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  uint32_t tmp = 0x00010000;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  tmp |= GPIO_Pin;
  /* Set LCKK bit */
  GPIOx->LCKR = tmp;
  /* Reset LCKK bit */
  GPIOx->LCKR =  GPIO_Pin;
  /* Set LCKK bit */
  GPIOx->LCKR = tmp;
  /* Read LCKK bit*/
  tmp = GPIOx->LCKR;
  /* Read LCKK bit*/
  tmp = GPIOx->LCKR;
}

/**
  * @}
  */

/** @defgroup GPIO_Group2 GPIO Read and Write
 *  @brief   GPIO Read and Write
		     GPIO ∂¡–¥
 *
@verbatim   
 ===============================================================================
                              GPIO Read and Write
 ===============================================================================  

@endverbatim
  * @{
  */

/**
  * @brief  Reads the specified input port pin.
		    ∂¡π‹Ω≈ ‰»Îºƒ¥Ê∆˜
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
     ≤Œ ˝   GPIOx(A, B, C, D, E or H)  
  * @param  GPIO_Pin: specifies the port bit to read.
     ≤Œ ˝   GPIO_Pin GPIO_Pin_x(0..15)
  *   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval The input port pin value.
     ∑µªÿ π‹Ω≈÷µ£®uint8_t£© 
  */
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  uint8_t bitstatus = 0x00;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin));

  if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)Bit_RESET)
  {
    bitstatus = (uint8_t)Bit_SET;
  }
  else
  {
    bitstatus = (uint8_t)Bit_RESET;
  }
  return bitstatus;
}

/**
  * @brief  Reads the specified GPIO input data port.
		 ∂¡∂Àø⁄GPIOx ‰»Îºƒ¥Ê∆˜µƒ÷µ(’˚∏ˆ∂Àø⁄)
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
     ≤Œ ˝   GPIOx(A, B, C, D, E or H)
  * @retval GPIO input data port value.
     ∑µªÿ∂Àø⁄÷µ(uint16_t)
  */
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  return ((uint16_t)GPIOx->IDR);
}

/**
  * @brief  Reads the specified output data port bit.
		 ∂¡»° ‰≥ˆºƒ¥Ê∆˜ƒ≥Œªµƒ÷µ
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
		 ≤Œ ˝   GPIOx(A, B, C, D, E or H)
  * @param  GPIO_Pin: Specifies the port bit to read.
		 ≤Œ ˝   GPIO_Pin GPIO_Pin_x(0..15) 
  *   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval The output port pin value.
		  ∑µªÿ ‰≥ˆ∂Àø⁄µƒƒ≥Œªµƒ÷µuint8_t
  */
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  uint8_t bitstatus = 0x00;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin));
  
  if ((GPIOx->ODR & GPIO_Pin) != (uint32_t)Bit_RESET)
  {
    bitstatus = (uint8_t)Bit_SET;
  }
  else
  {
    bitstatus = (uint8_t)Bit_RESET;
  }
  return bitstatus;
}

/**
  * @brief  Reads the specified GPIO output data port.
	∂¡∂Àø⁄GPIOx ‰≥ˆºƒ¥Ê∆˜µƒ÷µ(’˚∏ˆ∂Àø⁄)	 
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
	≤Œ ˝   GPIOx(A, B, C, D, E or H)	 
  * @retval GPIO output data port value.
	∑µªÿ∂Àø⁄÷µ(uint16_t)	  
  */
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  return ((uint16_t)GPIOx->ODR);
}

/**
  * @brief  Sets the selected data port bits.
		    ÷√ƒ≥∏ˆπ‹Ω≈÷µŒ™1
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
     ≤Œ ˝   GPIOx(A, B, C, D, E or H)
  * @param  GPIO_Pin: specifies the port bits to be written.
  *   This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
     ≤Œ ˝   GPIO_Pin GPIO_Pin_x(0..15)

  * @note  This functions uses GPIOx_BSRR register to allow atomic read/modify 
  *        accesses. In this way, there is no risk of an IRQ occurring between
  *        the read and the modify access.
    ◊¢“‚÷√ŒªŒ™◊‘∂ØÕÍ≥…£¨√ª”–±ª÷–∂œµƒ∑Áœ’
  * @retval None
  */
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  GPIOx->BSRRL = GPIO_Pin;
}

/**
  * @brief  Clears the selected data port bits.
		    ÷√ƒ≥∏ˆπ‹Ω≈÷µŒ™0
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
	≤Œ ˝   GPIOx(A, B, C, D, E or H)	 
  * @param  GPIO_Pin: specifies the port bits to be written.
  *   This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
    ≤Œ ˝   GPIO_Pin GPIO_Pin_x(0..15)
  * @note  This functions uses GPIOx_BSRR register to allow atomic read/modify 
  *        accesses. In this way, there is no risk of an IRQ occurring between
  *        the read and the modify access.
    ◊¢“‚∏¥ŒªŒ™◊‘∂ØÕÍ≥…£¨√ª”–±ª÷–∂œµƒ∑Áœ’
  * @retval None
  */
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  GPIOx->BSRRH = GPIO_Pin;
}

/**
  * @brief  Sets or clears the selected data port bit.
     –¥∂Àø⁄ƒ≥Œª£®…œ√Ê¡Ω∏ˆ∫Ø ˝µƒ∫œÃÂ£©
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bit to be written.
  *   This parameter can be one of GPIO_Pin_x where x can be (0..15).
  * @param  BitVal: specifies the value to be written to the selected bit.
  *   This parameter can be one of the BitAction enum values:
  *     @arg Bit_RESET: to clear the port pin
  *     @arg Bit_SET: to set the port pin
  * @retval None
  */
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin));
  assert_param(IS_GPIO_BIT_ACTION(BitVal));
  
  if (BitVal != Bit_RESET)
  {
    GPIOx->BSRRL = GPIO_Pin;
  }
  else
  {
    GPIOx->BSRRH = GPIO_Pin ;
  }
}

/**
  * @brief  Writes data to the specified GPIO data port.
		 Ω´ ˝æ›–¥µΩƒ≥∏ˆ∂Àø⁄
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
  * @param  PortVal: specifies the value to be written to the port output data 
  *                  register.
  * @retval None
  */
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  GPIOx->ODR = PortVal;
}

/**
  * @}
  */

/** @defgroup GPIO_Group3 GPIO Alternate functions configuration functions
 *  @brief   GPIO Alternate functions configuration functions
    »Œ“‚π¶ƒ‹≈‰÷√
 *
@verbatim   
 ===============================================================================
               GPIO Alternate functions configuration functions
 ===============================================================================  

@endverbatim
  * @{
  */

/**
  * @brief  Changes the mapping of the specified pin.
	∏ƒ±‰π‹Ω≈”≥…‰
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
	 ≤Œ ˝   GPIOx(A, B, C, D, E or H)
  * @param  GPIO_PinSource: specifies the pin for the Alternate function.
  *   This parameter can be GPIO_PinSourcex where x can be (0..15).
    ≤Œ ˝GPIO_PinSourcex£®0-15£©÷∆∂®ƒ≥∏ˆΩ≈
  * @param  GPIO_AFSelection: selects the pin to used as Alternat function.
    ≤Œ ˝  GPIO_AFSelection —°‘Òπ¶ƒ‹
  *   This parameter can be one of the following values:
  *     @arg GPIO_AF_RTC_50Hz: RTC 50/60 Hz synchronization
  *     @arg GPIO_AF_MCO: Microcontroller clock output
  *     @arg GPIO_AF_RTC_AF1: Time stamp, Tamper, Alarm A out, Alarm B out,
  *                           512 Hz clock output (with an LSE oscillator of 32.768 kHz)
  *     @arg GPIO_AF_WKUP: wakeup
  *     @arg GPIO_AF_SWJ: SWJ (SW and JTAG)
  *     @arg GPIO_AF_TRACE
  *     @arg GPIO_AF_TIM2
  *     @arg GPIO_AF_TIM3
  *     @arg GPIO_AF_TIM4
  *     @arg GPIO_AF_TIM9
  *     @arg GPIO_AF_TIM10
  *     @arg GPIO_AF_TIM11
  *     @arg GPIO_AF_I2C1
  *     @arg GPIO_AF_I2C2
  *     @arg GPIO_AF_SPI1
  *     @arg GPIO_AF_SPI2
  *     @arg GPIO_AF_USART1
  *     @arg GPIO_AF_USART2
  *     @arg GPIO_AF_USART3
  *     @arg GPIO_AF_USB
  *     @arg GPIO_AF_LCD
  *     @arg GPIO_AF_RI
  *     @arg GPIO_AF_EVENTOUT: Cortex-M3 EVENTOUT signal
  * @note: The pin should already been configured in Alternate Function mode(AF)
  *        using GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
    ◊¢“‚π‹Ω≈±ÿ–Îœ»∂®“ÂŒ™AFƒ£ Ω£®GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF£©
  * @note: Please refer to the Alternate function mapping table in the device 
  *        datasheet for the detailed mapping of the system and peripheralsí
  *        alternate function I/O pins.
    ◊¢“‚«Î≤Œøº ˝æ› ÷≤·  
  * @note: EVENTOUT is not mapped on PH0, PH1 and PH2.  
  * @retval None
  */
void GPIO_PinAFConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_PinSource, uint8_t GPIO_AF)
{
  uint32_t temp = 0x00;
  uint32_t temp_2 = 0x00;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN_SOURCE(GPIO_PinSource));
  assert_param(IS_GPIO_AF(GPIO_AF));
  
  temp = ((uint32_t)(GPIO_AF) << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4)) ;
  GPIOx->AFR[GPIO_PinSource >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4)) ;/*œ»«Â¡„*/
  temp_2 = GPIOx->AFR[GPIO_PinSource >> 0x03] | temp;
  GPIOx->AFR[GPIO_PinSource >> 0x03] = temp_2;/*‘Ÿ∏≥÷µ£¨πÿº¸‘⁄”⁄µ⁄ÀƒŒª¿¥»∑∂® «AFR[1]ªπ «AFT[0]*/
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

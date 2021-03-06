/**
  ******************************************************************************
  * @file    stm32l1xx_gpio.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-December-2010
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the GPIO peripheral:           
  *           - Initialization and Configuration
  *              初始化及配置GPIO
  *           - GPIO Read and Write
  *                读写GPIO 
  *           - GPIO Alternate functions configuration
  *              GPIO 任意功能配置
  *            时钟设置寄存器RCC->AHBENR（AHB时钟使能，用于开启GPIO时钟）
  *寄存器介绍：配置寄存器: 1、GPIOx_MODER(模式寄存器）（用于选择in/out/analog/AF)
  *                        2、GPIOx_OTYPER（输出类型寄存器)(用于选择输出的类型push-pull/open drain )
  *                        3、GPIOx_OSPEEDR（输出速度寄存器)（选择输出速度0.4/2/10/40MHz)
  *                        4、GPIOx_PUPDR（上下拉寄存器）
  *            数据寄存器：1、GPIOx_IDR（输入数据寄存器）（只读）
  *                        2、GPIOx_ODR（输出数据寄存器）（由GPIOx_BSRR register设置）
  *                        3、GPIOx_BSRR(输出寄存器的置复位寄存器)
  * 
  *  @verbatim
  *
  *          ===================================================================
  *                                 How to use this driver
  *          ===================================================================       
  *           1. Enable the GPIO AHB clock using RCC_AHBPeriphClockCmd()
  *           1、使能GPIO AHB时钟通过 RCC_AHBPeriphClockCmd()主要配置GPIO->AHBENR在stm32lxx_rcc.c）中 

  *           2. Configure the GPIO pin(s) using GPIO_Init()
              2、配置GPIO PINS 通过GPIO_Init()主要配置上面的各个寄存器
  *              Four possible configuration are available for each pin:
                  每个pin有4个配置方式
  *                - Input: Floating, Pull-up, Pull-down.
                  输入:悬空，上拉，下拉
  *                - Output: Push-Pull (Pull-up, Pull-down or no Pull)
                   输出：推挽（包括上拉、下拉、不拉）
  *                          Open Drain (Pull-up, Pull-down or no Pull).
                          开漏（包括上拉、下拉、不拉）
  *                  In output mode, the speed is configurable: Very Low, Low,
  *                  Medium or High.
                    输入输出模式速度可配置为0.4/2/10/40MHz 对应于Very Low, Low,
  *                  Medium or High.
  *                - Alternate Function: Push-Pull (Pull-up, Pull-down or no Pull)
  *                                      Open Drain (Pull-up, Pull-down or no Pull).
                     任意功能模式：推挽（包括上拉、下拉、不拉）
                                   开漏（包括上拉、下拉、不拉）
  *                - Analog: required mode when a pin is to be used as ADC channel,
  *                          DAC output or comparator input.
                     模拟信号：用于ADC/DAC，或者比较器的输入
  * 
  *          3- Peripherals alternate function:
             3- 外设任意功能
  *              - For ADC, DAC and comparators, configure the desired pin in 
  *                analog mode using GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AN
                 - 对于ADC，DAC及比较器，配置管脚成模拟模式，通过
                   GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AN
  *              - For other peripherals (TIM, USART...):
                 - 对于其他外设（定时器、串口...)
  *                 - Connect the pin to the desired peripherals' Alternate 
  *                   Function (AF) using GPIO_PinAFConfig() function
                    -连接管脚到目标AF 用 GPIO_PinAFConfig() 
  *                 - Configure the desired pin in alternate function mode using
  *                   GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
                     -配置管脚成AFmode用GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
  *                 - Select the type, pull-up/pull-down and output speed via 
  *                   GPIO_PuPd, GPIO_OType and GPIO_Speed members
                    -选择类型（上拉下拉、输出速度）通过GPIO_PuPd, GPIO_OType and GPIO_Speed 
  *                 - Call GPIO_Init() function
                    - 调用GPIO_Init()
  *        
  *          4. To get the level of a pin configured in input mode use GPIO_ReadInputDataBit()
             4、读取pin电平通过配置输入模式（使用GPIO_ReadInputDataBit()）
  *          
  *          5. To set/reset the level of a pin configured in output mode use
  *             GPIO_SetBits()/GPIO_ResetBits()
             5、置复位pin 要用GPIO_SetBits()/GPIO_ResetBits()来配置输出模式
  *               
  *          6. During and just after reset, the alternate functions are not 
  *             active and the GPIO pins are configured in input floating mode
  *             (except JTAG pins).
                复位期间或之后，AF 未被激活，GPIO被配置成悬空模式（除JTAG外）
  *
  *          7. The LSE oscillator pins OSC32_IN and OSC32_OUT can be used as 
  *             general-purpose (PC14 and PC15, respectively) when the LSE
  *             oscillator is off. The LSE has priority over the GPIO function.
             7、LSE 晶振管脚OSC32_IN、OSC32_OUT可作为通用模式（对应的是PC14、PC15）当LSE 晶振关闭的时候。
                LSE 优先于GPIO 功能
  *
  *          8. The HSE oscillator pins OSC_IN/OSC_OUT can be used as 
  *             general-purpose PH0 and PH1, respectively, when the HSE 
  *             oscillator is off. The HSE has priority over the GPIO function.
             8、HSE 晶振管脚OSC_IN、OSC_OUT可作为通用模式（对应的是PH0、PH1）当HSE 晶振关闭的时候。
                HSE 优先于GPIO 功能
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
            初始化GPIOx寄存器到缺省值,通过配置RCC_AHBRSTR
  *         By default, The GPIO pins are configured in input floating mode
  *         (except JTAG pins).
            缺省下，GPIO 配置成输入悬空模式（除了JTAG)
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
    参数    GPIOx:x可以为(A, B, C, D, E or H)
  * @retval None
	 没有返回值	   
  */
void GPIO_DeInit(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  /*这个断言宏用于检查函数参数,当参数错误调用assert_failed（在main.c)由用户设置，报告错误的源文件及行数，正确不返回*/
  /*IS_GPIO_ALL_PERIPH()用于检测参数是否合格，是1，否0*/
  if(GPIOx == GPIOA)
  {
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOA, DISABLE);  
	/*重启GPIOx,通过配置RCC_AHBRSTR*/
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
            初始化GPIOx 外设通过参数GPIOx(设置mode\speed\PuPd\otype)
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
    参数    GPIOx(A, B, C, D, E or H)
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
  参数为1、GPIO_TypeDef* GPIOx（可填GPIOx)用于配置上述的寄存器的
        2、GPIO_InitTypeDef* GPIO_InitStruct 用于填写用户需要的类型、速度、输出模式、上拉下拉等）
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
      GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (pinpos * 2));/*一个pin占2位，因此必须移动pinpos * 2,先清空*/

      GPIOx->MODER |= (((uint32_t)GPIO_InitStruct->GPIO_Mode) << (pinpos * 2));/*一个pin占2位，因此必须移动pinpos * 2,再赋值*/

      if ((GPIO_InitStruct->GPIO_Mode == GPIO_Mode_OUT) || (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_AF))
      {  /*若选择输出模式或者AF模式,配置速度、输出类型、*/
        /* Check Speed mode parameters */
        assert_param(IS_GPIO_SPEED(GPIO_InitStruct->GPIO_Speed));

        /* Speed mode configuration */
        GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));/*一个pin占2位，因此必须移动pinpos * 2,先清空*/
        GPIOx->OSPEEDR |= ((uint32_t)(GPIO_InitStruct->GPIO_Speed) << (pinpos * 2));/*一个pin占2位，因此必须移动pinpos * 2,再赋值*/

        /*Check Output mode parameters */
        assert_param(IS_GPIO_OTYPE(GPIO_InitStruct->GPIO_OType));

        /* Output mode configuration */
        GPIOx->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pinpos)) ;/*一个pin占1位，因此必须移动pinpos ,先清空*/
        GPIOx->OTYPER |= (uint16_t)(((uint16_t)GPIO_InitStruct->GPIO_OType) << ((uint16_t)pinpos));/*一个pin占1位，因此必须移动pinpos ,再赋值*/
      }
       /*上下拉*/
      /* Pull-up Pull down resistor configuration */
      GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pinpos * 2));/*一个pin占2位，因此必须移动pinpos * 2,先清空*/
      GPIOx->PUPDR |= (((uint32_t)GPIO_InitStruct->GPIO_PuPd) << (pinpos * 2));/*一个pin占2位，因此必须移动pinpos*2 ,再赋值*/
      }
    }
  }
}

/**
  * @brief  Fills each GPIO_InitStruct member with its default value.
		    初始化GPIO_InitStruct到其缺省配置
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
  *         锁存管脚配置，（GPIOx_MODER, GPIOx_OTYPER, GPIOx_OSPEEDR,
  *         GPIOx_PUPDR, GPIOx_AFRL and GPIOx_AFRH.），在下次复位前不能再更改。
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
  * 参数GPIO_TypeDef* GPIOx(A, B, C, D, E or H)
  * @param  GPIO_Pin: specifies the port bit to be written.
    参数uint16_t GPIO_Pin（GPIO_Pin_x）0-15
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
		     GPIO 读写
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
		    读管脚输入寄存器
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
     参数   GPIOx(A, B, C, D, E or H)  
  * @param  GPIO_Pin: specifies the port bit to read.
     参数   GPIO_Pin GPIO_Pin_x(0..15)
  *   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval The input port pin value.
     返回 管脚值（uint8_t） 
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
		 读端口GPIOx输入寄存器的值(整个端口)
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
     参数   GPIOx(A, B, C, D, E or H)
  * @retval GPIO input data port value.
     返回端口值(uint16_t)
  */
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  return ((uint16_t)GPIOx->IDR);
}

/**
  * @brief  Reads the specified output data port bit.
		 读取输出寄存器某位的值
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
		 参数   GPIOx(A, B, C, D, E or H)
  * @param  GPIO_Pin: Specifies the port bit to read.
		 参数   GPIO_Pin GPIO_Pin_x(0..15) 
  *   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval The output port pin value.
		  返回输出端口的某位的值uint8_t
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
	读端口GPIOx输出寄存器的值(整个端口)	 
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
	参数   GPIOx(A, B, C, D, E or H)	 
  * @retval GPIO output data port value.
	返回端口值(uint16_t)	  
  */
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  return ((uint16_t)GPIOx->ODR);
}

/**
  * @brief  Sets the selected data port bits.
		    置某个管脚值为1
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
     参数   GPIOx(A, B, C, D, E or H)
  * @param  GPIO_Pin: specifies the port bits to be written.
  *   This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
     参数   GPIO_Pin GPIO_Pin_x(0..15)

  * @note  This functions uses GPIOx_BSRR register to allow atomic read/modify 
  *        accesses. In this way, there is no risk of an IRQ occurring between
  *        the read and the modify access.
    注意置位为自动完成，没有被中断的风险
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
		    置某个管脚值为0
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
	参数   GPIOx(A, B, C, D, E or H)	 
  * @param  GPIO_Pin: specifies the port bits to be written.
  *   This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
    参数   GPIO_Pin GPIO_Pin_x(0..15)
  * @note  This functions uses GPIOx_BSRR register to allow atomic read/modify 
  *        accesses. In this way, there is no risk of an IRQ occurring between
  *        the read and the modify access.
    注意复位为自动完成，没有被中断的风险
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
     写端口某位（上面两个函数的合体）
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
		 将数据写到某个端口
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
    任意功能配置
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
	改变管脚映射
  * @param  GPIOx: where x can be (A, B, C, D, E or H) to select the GPIO peripheral.
	 参数   GPIOx(A, B, C, D, E or H)
  * @param  GPIO_PinSource: specifies the pin for the Alternate function.
  *   This parameter can be GPIO_PinSourcex where x can be (0..15).
    参数GPIO_PinSourcex（0-15）制定某个脚
  * @param  GPIO_AFSelection: selects the pin to used as Alternat function.
    参数  GPIO_AFSelection 选择功能
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
    注意管脚必须先定义为AF模式（GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF）
  * @note: Please refer to the Alternate function mapping table in the device 
  *        datasheet for the detailed mapping of the system and peripherals�
  *        alternate function I/O pins.
    注意请参考数据手册  
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
  GPIOx->AFR[GPIO_PinSource >> 0x03] &= ~((uint32_t)0xF << ((uint32_t)((uint32_t)GPIO_PinSource & (uint32_t)0x07) * 4)) ;/*先清零*/
  temp_2 = GPIOx->AFR[GPIO_PinSource >> 0x03] | temp;
  GPIOx->AFR[GPIO_PinSource >> 0x03] = temp_2;/*再赋值，关键在于第四位来确定是AFR[1]还是AFT[0]*/
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

;******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
;* File Name          : startup_stm32l1xx_md.s
;* Author             : MCD Application Team
;* Version            : V1.0.0
;* Date               : 31-December-2010
;* Description        : STM32L1xx Ultra Low Power Medium-density Devices vector 
;*                      table for MDK-ARM toolchain.
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Set the initial PC == Reset_Handler
;*                      - Set the vector table entries with the exceptions ISR address
;*                      - Branches to __main in the C library (which eventually
;*                        calls main()).
;*                      After Reset the CortexM3 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;* <<< Use Configuration Wizard in Context Menu >>>   
;*******************************************************************************
; THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
; WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
; AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
; INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
; CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
; INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
;*******************************************************************************

; Amount of memory (in bytes) allocated for Stack
; Tailor this value to your application needs
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3 ;开辟stack段
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200 ;设定堆的大小  在SRAM中开辟，可读写

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3 ;定义一个名为HEAP的数据段，不初始化，以2^3字节对齐
__heap_base
Heap_Mem        SPACE   Heap_Size ;分配heap_size大小的内存单元，并初始化为0
__heap_limit

                PRESERVE8          ;指示当前数据栈8字节对齐
                THUMB              ;适用thumb指令集 见ARM体系结构与编程一书


; Vector Table Mapped to Address 0 at Reset   在flash中开辟 只读
                AREA    RESET, DATA, READONLY ;定义一个名为RESET的数据段，属性为只读
                EXPORT  __Vectors             ;声明__Vectors能被其他文件引用
                EXPORT  __Vectors_End         ;声明__Vectors_End能被其他文件引用
                EXPORT  __Vectors_Size        ;声明__Vectors_Size能被其他文件引用

__Vectors       DCD     __initial_sp              ; Top of Stack  分配一段字内存单元（以字对齐），存放名为__initial_sp的栈顶地址
                DCD     Reset_Handler             ; Reset Handler 分配一段字内存单元（以字对齐），存放标号着Reset handler函数的地址，中断触发后，会被送到PC中去，而后跳转到Reset handler（）
                DCD     NMI_Handler               ; NMI Handler   这些伪指令用途一样，存放中断处理程序的入口地址，这样就可跳转到下面那个对应的地方，最后才跳转到stm32lxx_it.c中的中断服务程序。利于模块化。与单片机有不同。
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     WWDG_IRQHandler           ; Window Watchdog
                DCD     PVD_IRQHandler            ; PVD through EXTI Line detect
                DCD     TAMPER_STAMP_IRQHandler   ; Tamper and Time Stamp
                DCD     RTC_WKUP_IRQHandler       ; RTC Wakeup
                DCD     FLASH_IRQHandler          ; FLASH
                DCD     RCC_IRQHandler            ; RCC
                DCD     EXTI0_IRQHandler          ; EXTI Line 0
                DCD     EXTI1_IRQHandler          ; EXTI Line 1
                DCD     EXTI2_IRQHandler          ; EXTI Line 2
                DCD     EXTI3_IRQHandler          ; EXTI Line 3
                DCD     EXTI4_IRQHandler          ; EXTI Line 4
                DCD     DMA1_Channel1_IRQHandler  ; DMA1 Channel 1
                DCD     DMA1_Channel2_IRQHandler  ; DMA1 Channel 2
                DCD     DMA1_Channel3_IRQHandler  ; DMA1 Channel 3
                DCD     DMA1_Channel4_IRQHandler  ; DMA1 Channel 4
                DCD     DMA1_Channel5_IRQHandler  ; DMA1 Channel 5
                DCD     DMA1_Channel6_IRQHandler  ; DMA1 Channel 6
                DCD     DMA1_Channel7_IRQHandler  ; DMA1 Channel 7
                DCD     ADC1_IRQHandler           ; ADC1
                DCD     USB_HP_IRQHandler         ; USB High Priority
                DCD     USB_LP_IRQHandler         ; USB Low  Priority
                DCD     DAC_IRQHandler            ; DAC
                DCD     COMP_IRQHandler           ; COMP through EXTI Line
                DCD     EXTI9_5_IRQHandler        ; EXTI Line 9..5
                DCD     LCD_IRQHandler            ; LCD
                DCD     TIM9_IRQHandler           ; TIM9
                DCD     TIM10_IRQHandler          ; TIM10
                DCD     TIM11_IRQHandler          ; TIM11
                DCD     TIM2_IRQHandler           ; TIM2
                DCD     TIM3_IRQHandler           ; TIM3
                DCD     TIM4_IRQHandler           ; TIM4
                DCD     I2C1_EV_IRQHandler        ; I2C1 Event
                DCD     I2C1_ER_IRQHandler        ; I2C1 Error
                DCD     I2C2_EV_IRQHandler        ; I2C2 Event
                DCD     I2C2_ER_IRQHandler        ; I2C2 Error
                DCD     SPI1_IRQHandler           ; SPI1
                DCD     SPI2_IRQHandler           ; SPI2
                DCD     USART1_IRQHandler         ; USART1
                DCD     USART2_IRQHandler         ; USART2
                DCD     USART3_IRQHandler         ; USART3
                DCD     EXTI15_10_IRQHandler      ; EXTI Line 15..10
                DCD     RTC_Alarm_IRQHandler      ; RTC Alarm through EXTI Line
                DCD     USB_FS_WKUP_IRQHandler    ; USB FS Wakeup from suspend
                DCD     TIM6_IRQHandler           ; TIM6
                DCD     TIM7_IRQHandler           ; TIM7
__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors    ;向量表大小

                AREA    |.text|, CODE, READONLY   ;定义一个代码段

; Reset handler routine  Reset_Handler程序段 
Reset_Handler    PROC
                 EXPORT  Reset_Handler             [WEAK] ;声明Reset_Handler能被其他文件引用
        IMPORT  __main                                    ;声明__main由其他文件定义
        IMPORT  SystemInit                                ;声明 SystemInit由其他文件定义
                 LDR     R0, =SystemInit                  ;将SystemInit代表的地址读取到R0
                 BLX     R0                               ;跳转到systeminit，并保存PC值到LR寄存器
                 LDR     R0, =__main                      ;从systeminit中返回后将__main的地址读取到R0
                 BX      R0                               ;跳转到__main(这里的__main是库初始化程序入口完成堆栈、文件等与系统相关的初始化后跳转到用户的main函数），并保存PC值到LR寄存器
                 ENDP

; Dummy Exception Handlers (infinite loops which can be modified)
;由于weak的存在，编译器会首先跳转到其他文件的NMI_Handler（其他名称亦如是），也就是跳转到stm32l1xx_it.c中的同名函数，当然，为了保险起见，倘若用户在stm32l1xx_it.c没有该函数，就执行这个文件的，也就是执行死循环了。
NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
                B       .                          ;无限死循环  ·代表当前PC所在
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler          [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler          [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler           [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler           [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler             [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler            [WEAK]
                B       .                         
                ENDP
;由于weak的存在，编译器会首先跳转到其他文件的xxxxxx（其他名称亦如是），也就是跳转到stm32l1xx_it.c中的同名函数，当然，为了保险起见，倘若用户在stm32l1xx_it.c没有该函数，就执行这个文件的xxxxxx，也就是执行死循环了。
Default_Handler PROC

                EXPORT  WWDG_IRQHandler            [WEAK]
                EXPORT  PVD_IRQHandler             [WEAK]
                EXPORT  TAMPER_STAMP_IRQHandler    [WEAK]
                EXPORT  RTC_WKUP_IRQHandler        [WEAK]
                EXPORT  FLASH_IRQHandler           [WEAK]
                EXPORT  RCC_IRQHandler             [WEAK]
                EXPORT  EXTI0_IRQHandler           [WEAK]
                EXPORT  EXTI1_IRQHandler           [WEAK]
                EXPORT  EXTI2_IRQHandler           [WEAK]
                EXPORT  EXTI3_IRQHandler           [WEAK]
                EXPORT  EXTI4_IRQHandler           [WEAK]
                EXPORT  DMA1_Channel1_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel2_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel3_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel4_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel5_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel6_IRQHandler   [WEAK]
                EXPORT  DMA1_Channel7_IRQHandler   [WEAK]
                EXPORT  ADC1_IRQHandler            [WEAK]
                EXPORT  USB_HP_IRQHandler          [WEAK]
                EXPORT  USB_LP_IRQHandler          [WEAK]
                EXPORT  DAC_IRQHandler             [WEAK]
                EXPORT  COMP_IRQHandler            [WEAK]
                EXPORT  EXTI9_5_IRQHandler         [WEAK]
                EXPORT  LCD_IRQHandler             [WEAK]
                EXPORT  TIM9_IRQHandler            [WEAK]
                EXPORT  TIM10_IRQHandler           [WEAK]
                EXPORT  TIM11_IRQHandler           [WEAK]
                EXPORT  TIM2_IRQHandler            [WEAK]
                EXPORT  TIM3_IRQHandler            [WEAK]
                EXPORT  TIM4_IRQHandler            [WEAK]
                EXPORT  I2C1_EV_IRQHandler         [WEAK]
                EXPORT  I2C1_ER_IRQHandler         [WEAK]
                EXPORT  I2C2_EV_IRQHandler         [WEAK]
                EXPORT  I2C2_ER_IRQHandler         [WEAK]
                EXPORT  SPI1_IRQHandler            [WEAK]
                EXPORT  SPI2_IRQHandler            [WEAK]
                EXPORT  USART1_IRQHandler          [WEAK]
                EXPORT  USART2_IRQHandler          [WEAK]
                EXPORT  USART3_IRQHandler          [WEAK]
                EXPORT  EXTI15_10_IRQHandler       [WEAK]
                EXPORT  RTC_Alarm_IRQHandler       [WEAK]
                EXPORT  USB_FS_WKUP_IRQHandler     [WEAK]
                EXPORT  TIM6_IRQHandler            [WEAK]
                EXPORT  TIM7_IRQHandler            [WEAK]

WWDG_IRQHandler
PVD_IRQHandler
TAMPER_STAMP_IRQHandler
RTC_WKUP_IRQHandler
FLASH_IRQHandler
RCC_IRQHandler
EXTI0_IRQHandler
EXTI1_IRQHandler
EXTI2_IRQHandler
EXTI3_IRQHandler
EXTI4_IRQHandler
DMA1_Channel1_IRQHandler
DMA1_Channel2_IRQHandler
DMA1_Channel3_IRQHandler
DMA1_Channel4_IRQHandler
DMA1_Channel5_IRQHandler
DMA1_Channel6_IRQHandler
DMA1_Channel7_IRQHandler
ADC1_IRQHandler
USB_HP_IRQHandler
USB_LP_IRQHandler
DAC_IRQHandler
COMP_IRQHandler
EXTI9_5_IRQHandler
LCD_IRQHandler
TIM9_IRQHandler
TIM10_IRQHandler
TIM11_IRQHandler
TIM2_IRQHandler
TIM3_IRQHandler
TIM4_IRQHandler
I2C1_EV_IRQHandler
I2C1_ER_IRQHandler
I2C2_EV_IRQHandler
I2C2_ER_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
USART1_IRQHandler
USART2_IRQHandler
USART3_IRQHandler
EXTI15_10_IRQHandler
RTC_Alarm_IRQHandler
USB_FS_WKUP_IRQHandler
TIM6_IRQHandler
TIM7_IRQHandler

                B       .

                ENDP

                ALIGN

;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                 IF      :DEF:__MICROLIB           
                
                 EXPORT  __initial_sp ;栈顶指针
                 EXPORT  __heap_base  ;堆基址
                 EXPORT  __heap_limit ;堆的限制
                
                 ELSE
                
                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap
                 
__user_initial_stackheap                  ;库的初始化子程序___main 会调用这个函数来初始化堆栈的

                 LDR     R0, =  Heap_Mem  ;保存堆的起始地址
                 LDR     R1, =(Stack_Mem + Stack_Size);保存栈的结束地址
                 LDR     R2, = (Heap_Mem +  Heap_Size);保存堆的结束地址
                 LDR     R3, = Stack_Mem              ;保存栈的起始地址
                 BX      LR                           ;跳回到调用前的位置

                 ALIGN

                 ENDIF

                 END

;******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE*****


;********************************************************************************
;********************************************************************************
;假设是从flash段启动的，地址0x80000000,则向量表被放置在0x80000000开始的一段连续区域内
;0x80000000放置的是栈顶地址
;复位触发中断，读取0x80000004的地址（Reset_Handler），跳转到Reset_Handler，见上面Reset_Handler的注释
;倘若是某中断被触发，自然获取该中断处存放的地址如BusFault_Handler  ，跳转到BusFault_Handler  ，这里值得思考的是export BusFault_Handler  weak伪指令的存在意义
;export label weak 为伪指令，在编译时就起作用，与代码位置无关。这个指令使得其他文件的label会先于这个文件的label被引用
;也就是说先跳转到stm32l1xx_it.c中的BusFault_Handler，执行该处的代码，倘若用户并没有在stm32l1xx_it.c处写有该函数，没紧要，那就执行这个文件下的
;BusFault_Handler,这里是无限死循环（也可自行修改），保证了程序的可靠性
;其他的中断服务函数也是这样的道理，先找到stm32l1xx_it.c（或者其他文件）的对应函数标号，若没有，执行本文件下的（这里是无限死循环，也可自行修改），保证了程序的可靠性
;**********************************************************************************
;**********************************************************************************
;**********************************************************************************
;注意这里与单片机的略有不同，单片机必须在中断向量表处放置跳转指令
;如org 0x00
; ljmp start
;  org 0x03
;  ljmp int0
;········
;·········
; int0: ***
;       reti
;这样才能实现中断服务程序
;而这里中断向量表放的是中断服务程序的地址
;然后硬件设计送这个地址到PC，然后实现跳转
;*****************************************************************************************
;*****************************************************************************************
;*****************************************************************************************

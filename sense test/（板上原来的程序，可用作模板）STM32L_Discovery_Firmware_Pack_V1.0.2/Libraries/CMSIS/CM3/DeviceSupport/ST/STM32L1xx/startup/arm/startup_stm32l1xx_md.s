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

                AREA    STACK, NOINIT, READWRITE, ALIGN=3 ;����stack��
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200 ;�趨�ѵĴ�С  ��SRAM�п��٣��ɶ�д

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3 ;����һ����ΪHEAP�����ݶΣ�����ʼ������2^3�ֽڶ���
__heap_base
Heap_Mem        SPACE   Heap_Size ;����heap_size��С���ڴ浥Ԫ������ʼ��Ϊ0
__heap_limit

                PRESERVE8          ;ָʾ��ǰ����ջ8�ֽڶ���
                THUMB              ;����thumbָ� ��ARM��ϵ�ṹ����һ��


; Vector Table Mapped to Address 0 at Reset   ��flash�п��� ֻ��
                AREA    RESET, DATA, READONLY ;����һ����ΪRESET�����ݶΣ�����Ϊֻ��
                EXPORT  __Vectors             ;����__Vectors�ܱ������ļ�����
                EXPORT  __Vectors_End         ;����__Vectors_End�ܱ������ļ�����
                EXPORT  __Vectors_Size        ;����__Vectors_Size�ܱ������ļ�����

__Vectors       DCD     __initial_sp              ; Top of Stack  ����һ�����ڴ浥Ԫ�����ֶ��룩�������Ϊ__initial_sp��ջ����ַ
                DCD     Reset_Handler             ; Reset Handler ����һ�����ڴ浥Ԫ�����ֶ��룩����ű����Reset handler�����ĵ�ַ���жϴ����󣬻ᱻ�͵�PC��ȥ��������ת��Reset handler����
                DCD     NMI_Handler               ; NMI Handler   ��Щαָ����;һ��������жϴ���������ڵ�ַ�������Ϳ���ת�������Ǹ���Ӧ�ĵط���������ת��stm32lxx_it.c�е��жϷ����������ģ�黯���뵥Ƭ���в�ͬ��
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

__Vectors_Size  EQU  __Vectors_End - __Vectors    ;�������С

                AREA    |.text|, CODE, READONLY   ;����һ�������

; Reset handler routine  Reset_Handler����� 
Reset_Handler    PROC
                 EXPORT  Reset_Handler             [WEAK] ;����Reset_Handler�ܱ������ļ�����
        IMPORT  __main                                    ;����__main�������ļ�����
        IMPORT  SystemInit                                ;���� SystemInit�������ļ�����
                 LDR     R0, =SystemInit                  ;��SystemInit����ĵ�ַ��ȡ��R0
                 BLX     R0                               ;��ת��systeminit��������PCֵ��LR�Ĵ���
                 LDR     R0, =__main                      ;��systeminit�з��غ�__main�ĵ�ַ��ȡ��R0
                 BX      R0                               ;��ת��__main(�����__main�ǿ��ʼ�����������ɶ�ջ���ļ�����ϵͳ��صĳ�ʼ������ת���û���main��������������PCֵ��LR�Ĵ���
                 ENDP

; Dummy Exception Handlers (infinite loops which can be modified)
;����weak�Ĵ��ڣ���������������ת�������ļ���NMI_Handler���������������ǣ���Ҳ������ת��stm32l1xx_it.c�е�ͬ����������Ȼ��Ϊ�˱�������������û���stm32l1xx_it.cû�иú�������ִ������ļ��ģ�Ҳ����ִ����ѭ���ˡ�
NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
                B       .                          ;������ѭ��  ������ǰPC����
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
;����weak�Ĵ��ڣ���������������ת�������ļ���xxxxxx���������������ǣ���Ҳ������ת��stm32l1xx_it.c�е�ͬ����������Ȼ��Ϊ�˱�������������û���stm32l1xx_it.cû�иú�������ִ������ļ���xxxxxx��Ҳ����ִ����ѭ���ˡ�
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
                
                 EXPORT  __initial_sp ;ջ��ָ��
                 EXPORT  __heap_base  ;�ѻ�ַ
                 EXPORT  __heap_limit ;�ѵ�����
                
                 ELSE
                
                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap
                 
__user_initial_stackheap                  ;��ĳ�ʼ���ӳ���___main ����������������ʼ����ջ��

                 LDR     R0, =  Heap_Mem  ;����ѵ���ʼ��ַ
                 LDR     R1, =(Stack_Mem + Stack_Size);����ջ�Ľ�����ַ
                 LDR     R2, = (Heap_Mem +  Heap_Size);����ѵĽ�����ַ
                 LDR     R3, = Stack_Mem              ;����ջ����ʼ��ַ
                 BX      LR                           ;���ص�����ǰ��λ��

                 ALIGN

                 ENDIF

                 END

;******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE*****


;********************************************************************************
;********************************************************************************
;�����Ǵ�flash�������ģ���ַ0x80000000,��������������0x80000000��ʼ��һ������������
;0x80000000���õ���ջ����ַ
;��λ�����жϣ���ȡ0x80000004�ĵ�ַ��Reset_Handler������ת��Reset_Handler��������Reset_Handler��ע��
;������ĳ�жϱ���������Ȼ��ȡ���жϴ���ŵĵ�ַ��BusFault_Handler  ����ת��BusFault_Handler  ������ֵ��˼������export BusFault_Handler  weakαָ��Ĵ�������
;export label weak Ϊαָ��ڱ���ʱ�������ã������λ���޹ء����ָ��ʹ�������ļ���label����������ļ���label������
;Ҳ����˵����ת��stm32l1xx_it.c�е�BusFault_Handler��ִ�иô��Ĵ��룬�����û���û����stm32l1xx_it.c��д�иú�����û��Ҫ���Ǿ�ִ������ļ��µ�
;BusFault_Handler,������������ѭ����Ҳ�������޸ģ�����֤�˳���Ŀɿ���
;�������жϷ�����Ҳ�������ĵ������ҵ�stm32l1xx_it.c�����������ļ����Ķ�Ӧ������ţ���û�У�ִ�б��ļ��µģ�������������ѭ����Ҳ�������޸ģ�����֤�˳���Ŀɿ���
;**********************************************************************************
;**********************************************************************************
;**********************************************************************************
;ע�������뵥Ƭ�������в�ͬ����Ƭ���������ж�������������תָ��
;��org 0x00
; ljmp start
;  org 0x03
;  ljmp int0
;����������������
;������������������
; int0: ***
;       reti
;��������ʵ���жϷ������
;�������ж�������ŵ����жϷ������ĵ�ַ
;Ȼ��Ӳ������������ַ��PC��Ȼ��ʵ����ת
;*****************************************************************************************
;*****************************************************************************************
;*****************************************************************************************

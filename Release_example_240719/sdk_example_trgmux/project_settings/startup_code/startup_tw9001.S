/* ---------------------------------------------------------------------------------------*/
/*  @file:    startup_tw9001.s                                                           */
/*  @purpose: GNU Compiler Collection Startup File                                        */
/*            TW9001                                                                     */
/*  @version: 1.0                                                                         */
/*  @date:    2023-1-12                                                                   */
/*  @build:   b230112                                                                     */
/* ---------------------------------------------------------------------------------------*/

/*****************************************************************************/
/* Version: GNU Compiler Collection                                          */
/*****************************************************************************/
    .syntax unified
    .arch armv6-m

    .section .isr_vector, "a"
    .align 2
    .globl __isr_vector
__isr_vector:
    .long   __StackTop                       /* Top of Stack */
    .long   Reset_Handler                    /* Reset Handler */
    .long   NMI_Handler                      /* Non Maskable Interrupt */
    .long   HardFault_Handler                /* Cortex-M0 SV Hard Fault Interrupt */
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   SVC_Handler                      /* Cortex-M0 SV Call Interrupt */
    .long   0
    .long   0
    .long   PendSV_Handler                   /* Cortex-M0 Pend SV Interrupt */
    .long   SysTick_Handler                  /* Cortex-M0 System Tick Interrupt */


    .long   DMA0_IRQHandler                  /* DMA channel 0 transfer complete */
    .long   DMA1_IRQHandler                  /* DMA channel 1 transfer complete */
    .long   DMA2_IRQHandler                  /* DMA channel 2 transfer complete */
    .long   DMA3_IRQHandler                  /* DMA channel 3 transfer complete */
    .long   0
    .long   0
    .long   0
    .long   0
    .long   LPTMR0_IRQHandler                /* LPTIMER interrupt request */
    .long   PORT_IRQHandler                  /* Port A, B, C, D and E pin detect interrupt */
    .long   0
    .long   0
    .long   FTM0_Ch0_7_IRQHandler            /* FTM0 Channel 0 to 7 interrupt */
    .long   FTM0_Fault_IRQHandler            /* FTM0 Fault interrupt */
    .long   FTM0_Ovf_Reload_IRQHandler       /* FTM0 Counter overflow and Reload interrupt */
    .long   FTM1_Ch0_7_IRQHandler            /* FTM1 Channel 0 to 7 interrupt */
    .long   FTM1_Fault_IRQHandler            /* FTM1 Fault interrupt */
    .long   FTM1_Ovf_Reload_IRQHandler       /* FTM1 Counter overflow and Reload interrupt */
    .long   FTFC_IRQHandler                  /* FTFC Read collision and Double bit fault detect */
    .long   HSSPI_IRQHandler                 /* HSSPI Interrupt */
    .long   LPI2C1_IRQHandler                /* LPI2C1 Interrupt */
    .long   0
    .long   WDOG_IRQHandler                  /* WDOG interrupt request out before wdg reset out */
    .long   RCM_IRQHandler                   /* RCM Asynchronous Interrupt */
    .long   LPI2C0_IRQHandler                /* LPI2C0 Interrupt */
    .long   PUF_IRQHandler                   /* PUF Interrupt */
    .long   LPSPI0_IRQHandler                /* LPSPI0 Interrupt */
    .long   LPSPI1_IRQHandler                /* LPSPI1 Interrupt */
    .long   FSUSB_IRQHandler                 /* FSUSB Interrupt */
    .long   LPSPI2_IRQHandler                /* LPSPI2 Interrupt */
    .long   LPUART1_IRQHandler               /* LPUART1 Transmit / Receive  Interrupt */
    .long   LPUART0_IRQHandler               /* LPUART0 Transmit / Receive Interrupt */

    .size    __isr_vector, . - __isr_vector

    .text
    .thumb

/* Reset Handler */

    .thumb_func
    .align 2
    .globl   Reset_Handler
    .weak    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    cpsid   i               /* Mask interrupts */

    /* Init the rest of the registers */
    ldr     r1,=0
    ldr     r2,=0
    ldr     r3,=0
    ldr     r4,=0
    ldr     r5,=0
    ldr     r6,=0
    ldr     r7,=0
    mov     r8,r7
    mov     r9,r7
    mov     r10,r7
    mov     r11,r7
    mov     r12,r7

    /* Init ECC RAM */
    ldr r1, =__RAM_START
    ldr r2, =__RAM_END

    subs    r2, r1
    subs    r2, #1
    ble .LC5

    movs    r0, 0
    movs    r3, #4
.LC4:
    str r0, [r1]
    add	r1, r1, r3
    subs r2, 4
    bge .LC4
.LC5:

    /* Initialize the stack pointer */
    ldr     r0,=__StackTop
    mov     r13,r0

#ifndef __NO_SYSTEM_INIT
    /* Call the system init routine */
    ldr     r0,=SystemInit
    blx     r0
#endif

    /* Init .data and .bss sections */
    ldr     r0,=init_data_bss
    blx     r0
    cpsie   i               /* Unmask interrupts */
    bl      main
JumpToSelf:
    b       JumpToSelf

    .pool
    .size Reset_Handler, . - Reset_Handler

    .align  1
    .thumb_func
    .weak DefaultISR
    .type DefaultISR, %function
DefaultISR:
    b       DefaultISR
    .size DefaultISR, . - DefaultISR

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro def_irq_handler  handler_name
    .weak \handler_name
    .set  \handler_name, DefaultISR
    .endm

/* Exception Handlers */
    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    DMA0_IRQHandler
    def_irq_handler    DMA1_IRQHandler
    def_irq_handler    DMA2_IRQHandler
    def_irq_handler    DMA3_IRQHandler
    def_irq_handler    LPTMR0_IRQHandler
    def_irq_handler    PORT_IRQHandler
    def_irq_handler    FTM0_Ch0_7_IRQHandler
    def_irq_handler    FTM0_Fault_IRQHandler
    def_irq_handler    FTM0_Ovf_Reload_IRQHandler
    def_irq_handler    FTM1_Ch0_7_IRQHandler
    def_irq_handler    FTM1_Fault_IRQHandler
    def_irq_handler    FTM1_Ovf_Reload_IRQHandler
    def_irq_handler    FTFC_IRQHandler
    def_irq_handler    HSSPI_IRQHandler
    def_irq_handler    LPI2C1_IRQHandler
    def_irq_handler    WDOG_IRQHandler
    def_irq_handler    RCM_IRQHandler
    def_irq_handler    LPI2C0_IRQHandler
    def_irq_handler    PUF_IRQHandler
    def_irq_handler    LPSPI0_IRQHandler
    def_irq_handler    LPSPI1_IRQHandler
    def_irq_handler    FSUSB_IRQHandler
    def_irq_handler    LPSPI2_IRQHandler
    def_irq_handler    LPUART1_IRQHandler
    def_irq_handler    LPUART0_IRQHandler

    .end

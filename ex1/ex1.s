        .syntax unified
	
	      .include "efm32gg.s"

	/////////////////////////////////////////////////////////////////////////////
	//
  // Exception vector table
  // This table contains addresses for all exception handlers
	//
	/////////////////////////////////////////////////////////////////////////////
	
        .section .vectors
	
	      .long   stack_top               /* Top of Stack                 */
	      .long   _reset                  /* Reset Handler                */
	      .long   dummy_handler           /* NMI Handler                  */
	      .long   dummy_handler           /* Hard Fault Handler           */
	      .long   dummy_handler           /* MPU Fault Handler            */
	      .long   dummy_handler           /* Bus Fault Handler            */
	      .long   dummy_handler           /* Usage Fault Handler          */
	      .long   dummy_handler           /* Reserved                     */
	      .long   dummy_handler           /* Reserved                     */
	      .long   dummy_handler           /* Reserved                     */
	      .long   dummy_handler           /* Reserved                     */
	      .long   dummy_handler           /* SVCall Handler               */
	      .long   dummy_handler           /* Debug Monitor Handler        */
	      .long   dummy_handler           /* Reserved                     */
	      .long   dummy_handler           /* PendSV Handler               */
	      .long   dummy_handler           /* SysTick Handler              */

	      /* External Interrupts */
	      .long   dummy_handler
	      .long   gpio_handler            /* GPIO even handler */
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   gpio_handler            /* GPIO odd handler */
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler
	      .long   dummy_handler

	      .section .text

	/////////////////////////////////////////////////////////////////////////////
	//
	// Reset handler
  // The CPU will start executing here after a reset
	//
	/////////////////////////////////////////////////////////////////////////////

	      .globl  _reset
	      .type   _reset, %function
        .thumb_func
_reset:

    bl setup_clk
    bl setup_leds
    bl setup_buttons  
    bl setup_sleep_mode 
    bl enable_energysaveing    


    

    //turn off LFACLK/LFBCLK
    ldr r0, =CMU_BASE
    mov r1, #0
    str r1, [r0, #CMU_LFCLKSEL]
    

    //branch to polling function if that is desired
    //instead of interrupts
    //b button_polling
    b setup_interupt
    bx lr
	/////////////////////////////////////////////////////////////////////////////
	//
  // GPIO handler
  // The CPU will jump here when there is a GPIO interrupt
	//
	/////////////////////////////////////////////////////////////////////////////
	
        .thumb_func

setup_clk:

    // set GPIO CLOCK
    mov r0, #1
    lsl r0, r0, #CMU_HFPERCLKEN0_GPIO
    ldr r1, =CMU_BASE
    ldr r2, [r1, #CMU_HFPERCLKEN0]
    orr r2, r2, r0
    str r2, [r1, #CMU_HFPERCLKEN0]
    
    bx lr
    .thumb_func

setup_leds:
    // GPIO output
    // drive strength to low
    ldr r0, =GPIO_PA_BASE
    mov r1, #0x1 
    str r1, [r0, #GPIO_CTRL]
    
    // set to output
    mov r1, #0x55555555
    str r1, [r0, #GPIO_MODEH]
    
    bx lr
    .thumb_func

setup_buttons:

    // GPIO output
    // set to input
    ldr r0, =GPIO_PC_BASE
    mov r1, #0x33333333
    str r1, [r0, #GPIO_MODEL]
    // enable internal pull-up
    mov r1, 0xff
    str r1, [r0, #GPIO_DOUT]

    bx lr
    .thumb_func

setup_sleep_mode:
    //turn on sleep mode
    ldr r0, =SCR
    mov r1, #6
    str r1, [r0]
    
    bx lr
    .thumb_func

enable_energysaveing:

    //turn off ramblock 1-3:
    ldr r0, =EMU_BASE
    mov r1, #7
    str r1, [r0, #EMU_MEMCTRL]
    
    //energymode 3:
    mov r1, #0
    str r1, [r0, #EMU_CTRL]

    bx lr
    .thumb_func

setup_interupt:

    // setup interrupts
    ldr r0, =GPIO_BASE
    mov r1, #0x22222222
    str r1, [r0, #GPIO_EXTIPSELL]
    mov r1, #0xff
    str r1, [r0, #GPIO_EXTIRISE]    //enable rising edge
    str r1, [r0, #GPIO_EXTIFALL]     //enable falling edge
    str r1, [r0, #GPIO_IEN]         //enable interruts
    ldr r1, =#0x802
    ldr r0, =ISER0
    str r1, [r0]                    //enable interrupt handling

    bx lr
    .thumb_func

led_handle:
    
    //read input, process it and write to output
    ldr r3, [r2, #GPIO_DIN]     // get input
    lsl r3, r3, #8              // left shift to get right pins
    eor r3, r3, #0
    str r3, [r1, #GPIO_DOUT]    // write back to leds
    
    b gpio_handler
    .thumb_func

gpio_handler:
    ldr r0, =GPIO_BASE
    ldr r1, =GPIO_PA_BASE
    ldr r2, =GPIO_PC_BASE
    //clear interrupt flag
    ldr r3, [r0, #GPIO_IF]
    str r3, [r0, #GPIO_IFC]
    
//    b led_handle // seems you can't branch from the gpio interupt handler
    //read input, process it and write to output
    ldr r3, [r2, #GPIO_DIN]     // get input
    lsl r3, r3, #8              // left shift to get right pins
    eor r3, r3, #0
    str r3, [r1, #GPIO_DOUT]    // write back to leds
    
    bx lr
	
	/////////////////////////////////////////////////////////////////////////////

    .thumb_func

//Call this function and remove interupt enabled(This is a while loop)
button_polling:
    //Polling function for later comparison
    ldr r1, =GPIO_PA_BASE
    ldr r2, =GPIO_PC_BASE
    
    //read input, process it and write to output
    ldr r3, [r2, #GPIO_DIN]     // get input
    lsl r3, r3, #8              // left shift to get right pins
    eor r3, r3, #0
    str r3, [r1, #GPIO_DOUT]    // write back to leds
    b button_polling
    
    
        .thumb_func
dummy_handler:  
    bx lr  // do nothing(in case of unexpected interupt)

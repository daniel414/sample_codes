# ChipWon mcu SDK template
<p>
	this repository is sdk template for tw9001. <br \>
</p>
<p hidden>
SDK lib version : sdklib_231205_1930 <br \>
</p>

Overview
========
The LED Blinky demo application provides a sanity check for the new SDK build environments and board bring up. The LED Blinky demo uses the lptmr interrupt to realize the function of timing delay. The example takes turns to shine the LED. The purpose of this demo is to provide a simple project for debugging and further development.

Toolchain supported
===================
- GCC ARM Embedded  10.3.1

Hardware requirements
=====================
- FPGA board
- Personal Computer
- Daplink Device or Jlink

Board settings
==============
No special settings are required.

Prepare the demo
1.  Connect daplink device between the PC host and the FPGA platform.
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.

Running the demo
================
When the demo runs successfully, you will find the led0 is blinking.

Add macro definition
====================
1. SPLL_96MHZ: select SPLL 96MHZ (default: SPLL 120MHZ)
2. CUSTOM_DMA_IRQ: Select customized DMA IRQ HANDLER (default: DMA IRQ HANDLER Template)
3. CUSTOM_DMA_ERR_IRQ: Select customized DMA ERR IRQ HANDLER (default: DMA ERR IRQ HANDLER Template)
4. CUSTOM_LPUART_IRQ: Select customized LPUART IRQ HANDLER (default: LPUART IRQ HANDLER Template)
5. CUSTOM_SYSICK_IRQ: Select customized SYSTICK IRQ HANDLER (default: SYSTICK IRQ HANDLER Template)




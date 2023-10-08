Overview
========
The sensorhub application demonstrates the use of multi-sensors in the KSDK. This demo simulates a use scenario of sensorhub on the 8ULP board: 
In general, Linux suspend(A-core) and M33(M-core) enters deep sleep mode, the screen is off. When we lift the board(simulate wrist lifting), 
the gyro sensor LSM6DSO will send an interrupt signal to M33, M33 will wake up and get the sensor data and display them on the screen, after 
3 seconds, M33 will go to deep sleep and the screen will be off again.​


Toolchain supported
===================
- IAR embedded Workbench  9.30.1
- GCC ARM Embedded  10.3.1

Hardware requirements
=====================
- Micro USB cable
- MIMX8ULP-EVK/EVK9 board
- J-Link Debug Probe
- 5V power supply
- Personal Computer

Board settings
==============
Please refer to the community blog: https://community.nxp.com/t5/blogs/blogworkflowpage/blog-id/blog/article-id/494 to do hardware rework.

Build the code
==============
Put the folder into SDK_2_13_1_EVK-MIMX8ULP\boards\evkmimx8ulp\demo_apps and build with IAR or armgcc.

Prepare the Demo
================
1.  Connect 5V power supply and J-Link Debug Probe to the board, switch SW10 to power on the board.
2.  Connect a micro USB cable between the host PC and the J17 USB port on the target board.
3.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  Download the program to the target board.
5.  Press the reset button on your board to begin running the demo.

Running the demo
================
Type any key to start sensor hub demo(except enter key).

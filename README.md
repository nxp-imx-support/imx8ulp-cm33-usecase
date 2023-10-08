# Cortex-M33 Use Cases on i.MX8ULP

This repository contains Cortex-M33 Use Cases for i.MX8ULP.


## License
 - License BSD-3-Clause
 - License MIT

## Hardware and software setup
 - NXP MCIMX8ULP-EVK
 - BSP Linux 5.15.71
 - MCUXpresso  SDK 2.13.0_RC2

## Build instructions

### MCUXpresso SDK build
Please refer to the _Getting Started with MCUXpresso SDK for EVK-MIMX8ULP and EVK9-MIMX8ULP.pdf_ included in the official SDK package for SDK build and deployment instructions.
The Cortex-M33 application needed for this application note should be copied directly into the SDK's _sensorhub_example_ folder:
```
cd <SDK path>/boards/evkmimx8ulp/demo_apps/
cp -r <path to imx8ulp_sensorhub folder>/imx8ulp_sensorhub/ .
```
Build the application using IAR or using the armgcc _build_debug.sh_ script:
```
cd <SDK path>/boards/evkmimx8ulp/demo_apps/imx8ulp_sensorhub/armgcc/
./build_debug.sh
```

## Prepare the Demo
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


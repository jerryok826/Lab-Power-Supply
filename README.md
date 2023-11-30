# Lab Power Supply
![Robot_Front](https://github.com/jerryok826/Lab-Power-Supply/blob/main/Pictures/lps_bat_lps_load.jpeg)

## Project Description
I always thought most laborary power supplies take up to much bench space. This design is just three inches by six inches and runs off a wall wart. It can also be powered from a lithium battery pack. The specs are 0 to 15 volts with a maximum output current of 1.5 amps. It has two volt presets and two output controls. One for voltage the other for current limiting. There is one additional button for output enable. It can can also be controlled remotely via USB. 

The control processor is a stm32f103c8t6, with Arm Cortex-M3 MCU with 64 Kbytes of Flash memory and a 72 MHz CPU. The display is a 256x128 OLED display. It is driven by uGui software.
 
### Project Status
The project is basicly comlete. Working on improvements.
![Robot_Front](https://github.com/jerryok826/Lab-Power-Supply/blob/main/Pictures/lps_bat_lps_load.jpeg)

## Design Files
### Electrical Design Files
This project was designed in Altium Designer 20

#### Datasheets
- [DRV8838 Motor Driver](https://www.ti.com/lit/ds/symlink/drv8838.pdf?HQS=TI-null-null-digikeymode-df-pf-null-wwe&ts=1599069814852)

### Motor Driver

The DRV8838 offers a simple two-pin PHASE/ENABLE control interface, which this board makes available for each motor as DIR and PWM, respectively. The DIR pin determines the motor direction (low drives the motor forward, high drives it in reverse) and the PWM pin can be supplied with a PWM signal to control the motor speed. The DIR and PWM control inputs are pulled low through weak internal pull-down resistors (approximately 100 kΩ). When the PWM pin is low, the motor outputs are both shorted to ground, which results in dynamic braking of a connected motor.

The two drivers’ SLEEP pins (labeled SLP) are connected together by default and can be driven low to put the drivers into a low-power sleep mode and turn off the motor outputs, which is useful if you want to let the motors coast. The SLEEP pins are pulled high through 10 kΩ pull-up resistors on the board so that the drivers are awake by default. In most applications, these pins can be left disconnected; if you want independent control of SLEEP on each side, you can cut the jumper labeled SLP L = R. The two SLEEP pins should not be driven separately without cutting this jumper.

The following simplified truth table shows how each driver operates:

(https://github.com/Elipsit/STM32-ROMI/blob/master/Pics/DRV8838_Truth_Table.png)

### PCBA
This project was manufactured by JLCPCB as an excuse for me to test their PCBA service.
*They didn't sponcer me I just wanted to try it out*


## Requited Parts

- [Romi Chassis Kit](https://www.pololu.com/product/3506)

- [Romi Encoder Pair Kit, 12 CPR, 3.5-18V](https://www.pololu.com/product/3542)

- [Motor Driver and Power Distribution Board for Romi Chassis](https://www.pololu.com/product/3543)

- [QTR-1A Reflectance Sensor (2-Pack)](https://www.pololu.com/product/2458)

- [0.96 Inch OLED](https://www.amazon.com/UCTRONICS-SSD1306-Self-Luminous-Display-Raspberry/dp/B072Q2X2LL/ref=sr_1_3?dchild=1&keywords=oled+0.96&qid=1598137389&sr=8-3)

- [HC-SRO4](https://www.amazon.com/Smraza-Ultrasonic-Distance-Mounting-Duemilanove/dp/B01JG09DCK/ref=sr_1_6?dchild=1&keywords=sonar+arduino&qid=1598137419&sr=8-6)

### 3D Print Files
- [Sonar Servo Mount](https://www.thingiverse.com/thing:1423)

# Bring Up
The Uart2 channel can be connected to the CH340C; USB to serial converter, or an ESP8266 running [esp-link](https://github.com/jeelabs/esp-link) as a telnet connection


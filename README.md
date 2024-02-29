# Lab Power Supply
![Robot_Front](https://github.com/jerryok826/Lab-Power-Supply/blob/main/Pictures/lps_bat_lps_load.jpeg)

![Robot_Front](https://github.com/jerryok826/Lab-Power-Supply/blob/main/Pictures/lab_power_supply_2.jpeg)

## Project Description
I always thought most laborary power supplies(LPS) take up to much bench space. This design is just 2.5 inches by 5.5 inches and runs off a wall wart or can also be powered from a lithium battery pack. The units specs are 0 to 15 volts output with a maximum output current of 1.5 amps. It has two output voltage presets with defaults are 5 and 3.3 volts. The presets are adjustable. It has two rotary encoders, one for voltage control and the other for current limiting. Pressing a encoder button will change it adjustement range. There is one additional button for output enable. The LPS can also be controlled remotely via USB using ModBus. There is also an realtime analog current bar graph on the oled display.

The control processor is a stm32f103c8t6, an Arm Cortex-M3 MCU with 64 Kbytes of Flash memory and a 72 MHz CPU. The display is a 256x128 OLED display. It is driven by uGui software. The unit is composed of two baords. The top board is the control and display board. The board below that is the regulator board. The regulator board is composed of a low volatge drop linear regulator. It is feed from a highly efficient switching regulator. Because combined efficiency not much of a heat sink is required. To reduce the noise generated by the switcher to a minimum. The switcher has large capacitors on it's input and 10KHz loop pass filter on its feed to the linear regulator. The switcher output is always keep about two volts above the linear's output.

### Project Status
The project is basicly comlete. Working on converting to Kicad and stm ide cube. and some other improvements.

## Design Files
### Electrical Design Files
This project was designed in Altium Designer 20 and an updated version in Kicad 7.X The control software is C code runing on FreeRTOS. 



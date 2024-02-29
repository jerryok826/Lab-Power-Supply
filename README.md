# Lab Power Supply
![Robot_Front](https://github.com/jerryok826/Lab-Power-Supply/blob/main/Pictures/lps_bat_lps_load.jpeg)

![Robot_Front](https://github.com/jerryok826/Lab-Power-Supply/blob/main/Pictures/lab_power_supply_2.jpeg)


## Project Description
I always thought most laborary power supplies take up to much bench space. This design is just 2.5 inches by 5.5 inches and runs off a wall wart or it can also be powered from a lithium battery pack. The specs are 0 to 15 volts output with a maximum output current of 1.5 amps. It has two volt presets and two output controls. One for voltage the other for current limiting. There is one additional button for output enable. It can can also be controlled remotely via USB using ModBus. There is also an realtime analog current bar graph on the oled display.

The control processor is a stm32f103c8t6, with Arm Cortex-M3 MCU with 64 Kbytes of Flash memory and a 72 MHz CPU. The display is a 256x128 OLED display. It is driven by uGui software.
 
### Project Status
The project is basicly comlete. Working on improvements.
![Robot_Front](https://github.com/jerryok826/Lab-Power-Supply/blob/main/Pictures/lps_bat_lps_load.jpeg)

## Design Files
### Electrical Design Files
This project was designed in Altium Designer 20 and an updated version in Kicad 7.X



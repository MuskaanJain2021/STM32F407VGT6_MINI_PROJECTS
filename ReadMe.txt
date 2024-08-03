STM32F407 LED Blinking Patterns with SysTick
This repository contains the source code for the LED blinking patterns project on the STM32F407 microcontroller. The project demonstrates various LED blinking patterns using the onboard LEDs of the STM32F407 Discovery board, with timing controlled by the SysTick timer.

Table of Contents
Introduction
Features
Hardware Requirements
Software Requirements
Project Structure
Setup and Usage
License


Introduction
This project showcases different LED blinking patterns on an STM32F407 Discovery board. The LEDs are controlled through GPIO pins, and the timing is managed using the SysTick timer. The project includes individual LED blinking, combination patterns, multi-LED patterns, and sequential LED patterns.

Features
Individual LED Blinking: Green, Red, and Orange LEDs blink on and off every 1 second.
Combination LED Blinking:
Green + Orange LEDs blink together every 1 second.
Green + Red LEDs blink together every 1 second.
Red + Orange LEDs blink together every 1 second.
Multi-LED Pattern: All three LEDs (Green, Red, Orange) blink together every 2 seconds.
Sequential LED Pattern: LEDs turn on and off sequentially (Green → Red → Orange) with 2-second intervals between turning off one LED and turning on the next.
Hardware Requirements

STM32F407 Discovery Board

Software Requirements

STM32CubeIDE
STM32CubeMX (optional for further configuration)

Project Structure


.
├── Core
│   ├── Inc
│   │   ├── main.h
│   │   └── gpio.h
│   ├── Src
│   │   ├── main.c
│   │   └── gpio.c
├── Drivers
│   ├── CMSIS
│   │   ├── Device
│   │   └── Include
│   ├── STM32F4xx_HAL_Driver
│   │   ├── Inc
│   │   └── Src
├── LICENSE
└── README.md
Setup and Usage
Clone the Repository:

git clone https://github.com/username/stm32f407-led-blinking-patterns.git
cd stm32f407-led-blinking-patterns
Open the Project:
Open STM32CubeIDE and import the project by selecting the stm32f407-led-blinking-patterns folder.

Build the Project:
Build the project by clicking on the build icon or pressing Ctrl+B.

Flash the Project to the Board:
Connect your STM32F407 Discovery board to your computer via USB. Flash the firmware to the board by clicking on the debug icon or pressing F11.

Observe the LED Patterns:
The onboard LEDs should start blinking according to the defined patterns.

License
This project is licensed under the MIT License - see the LICENSE file for details.


# EEE 192 Mobile Robot

Firmware for a Bluetooth-controlled mobile robot that can drive manually or run by itself, either **following a line** or **following walls through a maze**. It runs on a Microchip **PIC32CM5164LS00048** (Curiosity Nano) and was made for the EEE 192 course project at UP Diliman.

## What it does

| Mode | What the robot does |
|---|---|
| **Manual** | Drive forward, reverse, left and right from a phone over Bluetooth. A turn can take an angle, for example `A090` turns left about 90 degrees. |
| **Line tracing** | Reads a row of 7 analog line sensors and steers to stay on the line. If it loses the line, it sweeps left and right to look for it. |
| **Wall following** | Reads three ultrasonic sensors (front, left, right). It drives forward, and when the front is blocked it turns right if it can, otherwise left, otherwise turns around. |
| **Safe mode (SOS)** | If the line is lost for about 1 second, the robot stops all motors, switches off the automatic modes, turns on the buzzer and LED, and sends an SOS message to the phone. |
| **Telemetry** | Sends status messages, ultrasonic distances and raw line-sensor readings back to the phone. |

## Bluetooth commands

Send these characters from a Bluetooth serial app on the phone (9600 baud). Upper and lower case both work.

| Command | Action |
|---|---|
| `W` | Forward |
| `S` | Reverse |
| `A` or `A090` | Turn left (optionally by an angle, `A` plus three digits, up to 360) |
| `D` or `D090` | Turn right (same angle format) |
| `X` or space | Stop and leave any automatic mode |
| `L` | Start line tracing |
| `U` | Start wall following |
| `T` | Report ultrasonic distances: left, right, front (cm) |
| `P` | Report raw and on/off line-sensor readings |
| `C` | Ultrasonic calibration helper (place an object 10 cm from the right sensor) |
| `R` | Clear SOS and turn the buzzer off |

## Hardware

| Part | Notes |
|---|---|
| Microcontroller | PIC32CM5164LS00048 on a Curiosity Nano board, running at 24 MHz |
| Chassis | 4-wheel robot chassis with a left motor and a right motor |
| Motor driver | Dual H-bridge, with a PWM pin and two direction pins per motor |
| Line sensors | Analog reflectance sensors read by the ADC (2 left, 3 middle, 2 right) |
| Distance sensors | 3 x HC-SR04 ultrasonic (front, left, right) |
| Wireless | Bluetooth serial module (HC-05) on the USART |
| Alert | Buzzer and LED on one pin |

### Pin map (from the code)

| Function | Pins |
|---|---|
| Left motor | PWM PA04, direction PA05 / PA12 |
| Right motor | PWM PA13, direction PA20 / PA14 |
| Buzzer and LED (SOS) | PA16 |
| Line sensors (ADC) | PA02, PA03 (left), PA06, PA07, PA08 (middle), PA09, PA10 (right) |
| Ultrasonic, right | Trigger PA00, echo PA01 |
| Ultrasonic, left | Trigger PA19, echo PA07 |
| Ultrasonic, front | Trigger PA18, echo PB03 |
| Bluetooth USART | SERCOM3 on PB08 (TX) and PB09 (RX) |

## Files

| File | What it contains |
|---|---|
| `main.c` | Main program: command handling, line tracing, wall following and SOS logic |
| `init.c`, `init.h` | Clock setup (24 MHz), SysTick timer, event flags |
| `gpio.c` | Motor control and PWM, ADC (line sensors), ultrasonic sensors |
| `usart.c`, `usart.h` | Bluetooth serial communication (interrupt driven) |
| `include.h` | Internal function declarations shared between the files above |

In the original MPLAB project, `main.c` includes `platform/init.h` and `platform/usart.h`, so `init.*`, `gpio.c`, `usart.*` and `include.h` sit in a `platform` folder next to `main.c`.

## Known issues

- **Pin PA07 is used twice.** `platform_ADC_init()` sets PA07 up as an analog input (ADC channel 7, one of the middle line sensors). Later, `platform_ultrasonic_init()` reconfigures the same pin as a plain digital input for the left ultrasonic echo. That second setup switches off the analog function, so the middle line-sensor reading on channel 7 may be unreliable. To fix it, move the left echo to a pin that nothing else uses, or move that line sensor to another ADC pin.

## How it works

- A SysTick interrupt runs every 156.25 us. It drives the software motor PWM, times out serial reception and keeps a 10 ms application tick.
- The main loop runs forever and does a few small jobs each pass: check the turn timer, run line tracing or wall following, send any pending message and read any new command.
- Turns are done by time: the motors run for a set duration that depends on the angle, then stop.

## Building

The code uses the Microchip `xc.h` headers, so build it with **MPLAB X IDE** and the **XC32 compiler** for the PIC32CM LS00 family. Create a project for the PIC32CM5164LS00048, add these files, and program the Curiosity Nano.

## Credits

Written by Tristan Josh E. Sibug & Ria Shane H. Santiago for EEE 192. The platform code (clock, SysTick, USART and GPIO handlers) started from the EEE 158 AY2025-26 Sem 1 handlers by Alberto De Villa and was modified for this project.

# FPGA Tron Light Cycle Game

An FPGA-based implementation of the classic Tron Light Cycle game, written in embedded C and running on a RISC-V (Nios V) processor. The game renders directly to VGA hardware, supports human vs. AI gameplay, and runs on both the **Terasic DE10-Lite** and **DE1-SoC** FPGA boards, as well as in CPUlator for development and debugging. 

<img width="480" height="360" alt="VGAPixelBuffer" src="https://github.com/user-attachments/assets/43d5b489-2c15-49a6-b4a2-a1a529afdcd7" />

Figure 1: *VGA Pixel Buffer (0x08000000) on CPULator*

## Features
* Two-player Tron Light Cycle gameplay: human player vs. autonomous robot player
* AI component: looks one and two pixels ahead and attempts to turn left/right to avoid collisions
* VGA rendering: pixel-level framebuffer access, white border, obstacles, and coloured light trails
* Memory-mapped hardware control: VGA buffer access, JTAG UART, output, 7-segment display score tracking
* Score system: first player to reach 9 wins and winning player's colour fills the screen

## Controls
| Input              | Description                         |
| :---               | :----:                              |
| Push buttons [1:0] | [0] to turn right, [1] to turn left |
| Switches [9:0]     | controls the game's speed           |

*Active-low on-release input for push buttons

## Running on FPGA Hardware (DE10-Lite / DE1-SoC)

### Dependencies
* Intel® Quartus® Prime Lite Edition
* Nios V Command Shell
* FPGA board connected via USB (requires Altera USB Blaster Driver)
* VGA connector from FPGA to monitor

### Step 1: Load the Nios V System
Open Nios V Command Shell and run:
```
make DE10-Lite
make GDB_SERVER
```
Alternatively, you may use `make DE1-SoC`
```
make TERMINAL
make COMPILE
make RUN
```

### Step 2: Start the JTAG UART Terminal
```
make TERMINAL
```
This window displays all output sent from the Nios V program (e.g.,`start`,`done`).

### Step 3: Compile and Run the Game
```
make COMPILE
make RUN
```

## Running on CPUlator (No Hardware Required)
CPUlator is a browser-based emulator that features the DE1-SoC RISC-V system.

### Step 1: Configure the Emulator
Visit: [CPUlator](http://cpulator.01xz.net/) and
* select RISC-V RV32 under Architecture
* select RISC-V RV32 DE1-SoC under System
* select Go
* in the Editor window, change Language from RV32 to C
* select File, and navigate to Open...
* select the downloaded `vga.c` or paste its full contents into the editor

### Step 2: Run
* Click Compile and Load
* Click Continue
* VGA output appears in the emulator window (VGA pixel buffer 08000000)

## Switching Between DE10-Lite and DE1-SoC
Platform selection is controlled by a single macro in address_map_niosv.h:
```
#define DE10LITE 1 // change to 0 for CPUlator or DE1-SoC, 1 for DE10-Lite
```
Change this value before compiling to match your target platform. Note that if you are using CPULator, you must use the DE1-SoC platform.

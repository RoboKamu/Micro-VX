> ⚙️ Project status: Work in progress – hardware fully assembled, firmware under active development.

<h1 align="center"> Micro-VX: Mini Audio Visualizer </h1>

<div align="center">
<i> Small portable STM32L4 based audio analyzer that visualizes live waveform and FFT data from MEMS microphone. This project started as an experiment to design my first PCB and explore the STM32L4 low-power MCU family. </i>
</div>

---

> This repository includes an extra project on the same board. The accelerometer visualization. That project was for expirementing with the module for future use. The main purpose for the board is for it to be used with the microphone, however a PCB error ment it had to be modified after assembly.
>

> Demo for expirimental Accelerometer Visualization: ![Github Video Link](https://github.com/user-attachments/assets/0a82b6ed-7fab-43ed-bdb7-8c6c1341423e) 
>
> Demo for Audio Analyzer visualization: (WIP)


<p align="center">
  <img src="Docs/Images/formal-micro-vx.jpg" alt="Micro-VX board and OLED module" width="500"/>
  <img src="Docs/Images/holding-micro-vx.jpg" alt="Close-up of the assembled STM32L4 Micro-VX board" width="500"/>
</p>

---

### 1. Project structure

#### 1.1 Hardware

All Kicad source files for Schematic and PCB files are in the hardware folder. Also including Gerber files and footprint placement (assembly) for direct manufacturing, in this case JLCPCB. 

There are 2 versions of the PCB, with one being modified after assembly.

* **Accelerometer visualization:** 
Original PCB according to the schematic. Utilizing the Accelerometer and I2C pin headers for OLED display.

* **Audio analyzer:**
Modified after assembly (refer to hardware folder). The PCB used for the audio analysis with microphone. Also utilizes the I2C pin headers for OLED display. 

#### 1.2 Firmware

In the firmware folder you will see 2 projects: 

* **The STM32L4-LIS2DW12-Visual project in the STM32L4-Accelerometer-Visual folder:**
Finished visualizer for the cheap accelerometer I added to the PCB. The program just shows the X, Y and Z axis as well as temperature. Another detail is to measure VDDA, this is very useful for sampling with ADC on battery powered devices. Ensuring accurate scaling. 

* **STM32L4-FW (WIP):**
The main project is this audio analyzer. Depending on the mechanical switch state it can either be showing continous audio wave or the FFT representation of it. Based on the VDDA measurement so the device works both on USB power and battery.  

### 2. Hardware Architecture

Nothing new to note here. Refer to the hardware folder for more specific information. 

The STM32L4 was chosen because it is from the low power series and also includes FPU and DSP. Not much flash memory is needed here so the 32 KB flash size was chosen. 

### 3. Firmware Architecture

#### 3.1 STM32L4-FW (WIP) 
(WIP)


#### 3.2 STM32L4-LIS2DW12-Visual project 
Timer 2 triggers ADC injected group conversion, the callback function then stores the raw ADC value to a variable and sets a data ready flag

Timer 15 triggers normal ADC conversion, the callback function then stores the raw ADC value to a variable and sets a data ready flag

VDDA conversion just uses the last non zero value to calculate the voltage based on the formula from the datasheet. 

For the accelerometer the Data-Ready interrupt pin is connected to EXTI pin on microcontroller, when state changes on the pin the microcontroller notes this with a flag to later use in the superloop. 


TODO:
- [x] Design and assemble PCB
- [x] Write a simple driver for LIS2DW12 Accelerometer and display on oled
- [x] Read real battery voltage level with VREF channel
- [x] Read RAW ADC data on Microphone
- [ ] Visualize audio wave
- [ ] Visualize FFT of audio wave


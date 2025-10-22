# Smart Knee Sleeve for Osteoarthritis Monitoring

## Overview
This repository contains the firmware and code developed for the **Smart Knee Sleeve**, a wearable device designed to monitor **knee flexion** and **gait parameters** in individuals with **osteoarthritis (OA)**.  
The project was completed as part of an **MEng group project at King’s College London (KCL)**, aiming to provide clinicians and patients with a **low-cost, portable, and easy-to-use** solution for assessing knee motion outside clinical environments.

## Project Background
Osteoarthritis is a leading cause of disability worldwide, often resulting in reduced joint mobility and pain. Quantitative assessment of knee movement is vital for rehabilitation and long-term management but is typically confined to laboratory settings.  
The Smart Knee Sleeve addresses this limitation by embedding **flex sensors** and an **inertial measurement unit (IMU)** into a comfortable elastic sleeve, managed by an **ESP32 microcontroller** that performs real-time data acquisition and logging.

## Features
- **Real-time monitoring** of knee flexion angle  
- **Integrated flex sensor and IMU** for high-resolution motion tracking  
- **ESP32-based system** enabling fast sampling and wireless connectivity options  
- **Automatic data logging** to SD card  
- **Lightweight, unobtrusive sleeve design** with minimal impact on natural movement  

## Repository Contents
- `Knee_Flexion_Angle_Code_for_Arthropodic_Knee_Microcontroller.ino`  
  Firmware for the ESP32-based knee sleeve, used to capture and log knee flexion angle data.  
- `Shoe_Gait_Data_Code_for_Arthropodic_Shoe_Microcontroller.ino`  
  Firmware for a companion shoe module used to complement gait analysis.  
- *(Optional)* Documentation such as circuit diagrams, calibration data, and analysis scripts may be added later.

## Hardware Requirements
- **Microcontroller:** ESP32 development board  
- **Sensors:**  
  - Flex sensor (positioned over the knee joint)  
  - IMU (e.g., MPU6050, BNO055) for angular velocity and acceleration  
- **Storage:** SD card module for onboard data logging  
- **Power:** Rechargeable Li-Po battery (3.7 V)  
- **Wearable base:** Elastic knee sleeve with integrated wiring and sensor channels  

## Setup Instructions
1. Clone this repository:  
   ```bash
   git clone https://github.com/kentonleung/Knee-Sleeve.git
   ```
2. Open the desired `.ino` file in **Arduino IDE**.  
3. Under **Tools → Board**, select **ESP32 Dev Module** (or the specific ESP32 variant you are using).  
4. Connect the sensors and SD module following your circuit diagram.  
5. Install the required libraries (e.g., `Wire.h`, `Adafruit_Sensor`, `SD.h`, IMU-specific drivers).  
6. Upload the code to the ESP32.  
7. Power on the device — it will begin recording automatically.  
8. Remove the SD card after testing to retrieve and analyze data.

## Usage
1. Wear the sleeve so that the flex sensor aligns with the knee joint.  
2. Begin movement (walking, sitting, squatting, etc.) — the ESP32 records knee angle and motion data.  
3. Data logs can be processed in MATLAB, Python, or Excel for analysis and visualization.  
4. Wireless features of the ESP32 can later be used for live data transmission or mobile integration.

## Applications
- Osteoarthritis monitoring and rehabilitation tracking  
- Physiotherapy and post-surgical recovery assessment  
- Biomechanical gait analysis  
- At-home motion tracking and telehealth support  

## Project Status
- **Stage:** Functional prototype validated with preliminary motion trials  
- **Next steps:**  
  - Enable Bluetooth Low Energy for real-time data streaming  
  - Implement improved calibration algorithms  
  - Develop clinician-facing visualization software  

## Team
Developed by **MEng students at King’s College London** as part of the **Applied Medical Engineering Project**.  
The team included students from **Biomedical**, **Mechanical**, and **Electrical Engineering** disciplines.

## License
This project is released under the MIT License.  
```
MIT License  
© 2025 Kenton Leung and Contributors
```

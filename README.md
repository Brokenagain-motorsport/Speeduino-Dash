## ESP Dash (LVGL) — Speeduino Dashboard + WiFi Portal + SD Logging

A fast, touchscreen-friendly ESP32 dashboard for Speeduino-style data, built on **LVGL + TFT_eSPI** for a 480×320 3.5" display.  
Includes **RPM meter + tiles**, **SD CSV logging**, a **WiFi AP configuration portal**, warning alarms, and a shift-light overlay.

### Highlights
- **Two dashboard layouts**
  - **Meter view**: analog-style RPM meter with color zones + big RPM readout
    <img width="681" height="418" alt="image" src="https://github.com/user-attachments/assets/5db0e0aa-0c6f-448c-b4cc-0b16b356360a" />
  - **Bar view**: horizontal RPM bar with tick labels + tiles grid
    <img width="674" height="415" alt="image" src="https://github.com/user-attachments/assets/6eabca2d-9d4a-4360-9192-3203a5ac14cf" />

- **SD logging (CSV)**
  - One-press REC start/stop
  - Auto-increment log index (`/log_00001.csv`, etc.)
  - Download logs via the WiFi portal (including “latest log” shortcut)
    
- **WiFi configuration portal (AP mode)**
  - Connect to `ESP_DASH` and configure warnings / logging / view mode
  - When a device connects, the dashboard switches to a **static portal screen** for maximum stability
    <img width="677" height="443" alt="image" src="https://github.com/user-attachments/assets/b5acbc02-7dc8-4ca6-8795-3e34961e1e53" />

- **Warnings + shift light**
  - Per-sensor min/max thresholds with enable toggles
  - Full-screen flashing shift overlay at configurable RPM
 <img width="673" height="440" alt="image" src="https://github.com/user-attachments/assets/d56ed6c4-18d1-4d3a-85db-8e6a2253042c" />

Check the files section for demo videos and actual screenshots

  ### Current Bugs
  -web portal is a bit flakey and requires more development


  ###HARDWARE
  Freenove FNK103, ESP32 CYD 3.5 Inch Touch Display WiFi BT Dual-core 32-bit 240 MHz Microcontroller
  <img width="605" height="333" alt="image" src="https://github.com/user-attachments/assets/5062cff9-566b-4062-8729-468183eb2774" />

  <img width="580" height="564" alt="image" src="https://github.com/user-attachments/assets/498c2bbd-7d10-400a-8d4f-0c369cf87719" />


## Installation

  ### Install
   Arduino IDE 2.3.7
   
  ### Download 
  files from from GitHub 
   - esp_dash_v1.ino 
   - lv_conf.h 
   - esp_dash_v1.cpp 
   - TFT_eSPI_Setups 
   - User_Setup.h 
   - User_Setup_Select.h 

 ### Pre-Requisites 
 install the following Libraries
   - TFT_eSPI by Bodmer v2.5.43
   - copy User_Setup.h and User_Setup_Select.h into the TFT_eSPI library folder overwriting the existing
   - copy TFT_eSPI_Setups folder into the Arduino Libraries space
   - <img width="261" height="110" alt="image" src="https://github.com/user-attachments/assets/e077f77a-ef32-4370-bf04-f4724dc25f19" />
   
   - lvgl by Kisvegabor v8.4.0

 install the following board type
   - esp32 by Espressif Systems v3.3.5
   - set board to ESP32 Dev Module
   - PSRAM = Enabled
   - Partition Scheme = Huge APP (3MB No OTA/1MB SPIFFS)
   
### Prepare the firmware
run esp_dash_v1.ino and allow it to create a folder then move lv_conf.h and esp_dash_v1.cpp into the same folder.

you should now be able to flash your display




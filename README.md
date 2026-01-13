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

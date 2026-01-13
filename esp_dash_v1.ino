/*
  esp_dash_v1_lvgl v2.1.0
  - LVGL UI that matches original (pre-LVGL) layout & functionality
  - 480x320 (3.5") TFT_eSPI + XPT2046 touch via TFT_eSPI getTouch()
  - Speeduino 'n' frame polling/decoding (same indexes as original)
  - Tiles: AFR / VBAT / IAT / CLT (left) and TPS / ADV / WARMUP / LAUNCH (right)
  - Center tach ring + big RPM number (RING view) and optional BAR view
  - Settings screen: warning thresholds, shift light, view mode, save/back
  - SD logging + REC toggle
*/

#include <Arduino.h>

// Arduino sketch must provide setup()/loop(), we delegate to cpp
void dashSetup();
void dashLoop();
void wifiSetup();
void wifiLoop();

void setup() { 
dashSetup();
wifiSetup();
}
void loop()  { 
dashLoop();  
wifiLoop();
}

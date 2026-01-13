// esp_dash_v1_lvgl_v2.6.16_fixed.cpp
// - Freenove FNK103 3.5" stability fix: prevent WiFi-pause related crashes / blank UI
// - Adds lvReady guard so wifiLoop won't touch LVGL until LVGL/UI are built
// - Keeps LVGL running (throttled) while "paused" so display stays valid
// - Keeps backlight ON when WiFi client connects (no "blank screen" symptom)
//
// Your .ino should call: dashSetup(); wifiSetup(); in setup()
// and: dashLoop(); wifiLoop(); in loop()

#include <Arduino.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#include <TFT_eSPI.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <lvgl.h>

// WiFi web portal (AP mode)
#include <WiFi.h>
#include <WebServer.h>

// ============================= VERSION =============================
static const char* FW_VERSION = "v2.6.16";

// ============================= OPTIONAL FEATURES =============================
#define USE_TOUCH 1
#define USE_SD    1
#define USE_WIFI  1

// "Pause" means: throttle LVGL + stop heavy tile updates while portal is in use.
// It should NOT blank the backlight or stop LVGL completely.
static volatile bool uiPaused = false;
static volatile bool lvReady  = false;   // only true after LVGL + UI objects exist
static volatile bool portalMode = false; // NEW: when true, LVGL/UI fully stopped and TFT shows a static portal screen
static uint8_t wifiStaCount = 0;

// ============================= WIFI AP CONFIG =============================
// ============================= UI PAUSE WHILE WIFI CLIENT CONNECTED =============================
// When a phone/laptop connects to the ESP_DASH AP to use the web portal, the UI rendering can
// be throttled to free CPU for better web-server stability.
// This reduces LVGL + TFT load during portal use.
#define PAUSE_UI_WHEN_WIFI_CLIENT 1
#define PAUSE_UI_BACKLIGHT_OFF   0   // KEEP SCREEN VISIBLE (avoid "blank screen" symptom)

static uint32_t lastWifiClientCheckMs = 0;
static const uint32_t WIFI_CLIENT_CHECK_MS = 250;

#if USE_WIFI

// Forward declarations for WebServer handlers (needed for Arduino build order)
static void handleRoot();
static void handleSave();
static void handleDownload();
static void handleDownloadLatest();
static void handleReboot();
// Forward declarations needed by LVGL callbacks
static void refresh_settings_list();
static void flashSavedMsg(const char* msg);
static void showToast(const char* title, const char* msg);


static const char* WIFI_AP_SSID = "ESP_DASH";
static const char* WIFI_AP_PASS = "12345678"; // >=8 chars
static const uint8_t WIFI_AP_CH = 6;
static const bool WIFI_AP_HIDDEN = false;
static const uint8_t WIFI_AP_MAX_CONN = 2;
static WebServer server(80);
#endif

// ============================= SPLASH =============================
static const uint32_t SPLASH_DELAY_MS = 4000;

// ============================= UART select =============================
#define USE_UART0 1
#if USE_UART0
  #define ECU_SERIAL Serial
#else
  #define ECU_SERIAL Serial2
  static const int ECU_RX_PIN = 16;
  static const int ECU_TX_PIN = 17;
#endif
static const uint32_t ECU_BAUD = 115200;

// ============================= Freenove SD pins =============================
#if USE_SD
static const int SD_VSPI_SS   = 5;
static const int SD_VSPI_SCK  = 18;
static const int SD_VSPI_MISO = 19;
static const int SD_VSPI_MOSI = 23;
static SPIClass sdSpi(VSPI);
#endif

// ============================= Timing Constants =============================
static const uint32_t POLL_MS = 100;
static const uint32_t LINK_STALE_MS = 700;
static const uint32_t UI_UPDATE_MS = 60;
static const uint32_t STATUS_UPDATE_MS = 250;
static const uint32_t LOG_INTERVAL_MS = 100;
static const uint32_t LOG_FLUSH_MS = 1000;
static const uint32_t SHIFT_FLASH_MS = 180; // blink speed

// When portal is connected, run LVGL timer handler at a reduced rate (~25 Hz)
static const uint32_t LVGL_PAUSED_MS = 40;

// ============================= Screen =============================
static TFT_eSPI tft;
static const int SCREEN_W = 480;
static const int SCREEN_H = 320;
static const int STATUS_H = 16;

// ============================= Theme =============================
static const uint16_t C_BG      = TFT_BLACK;
static const uint16_t C_PANEL   = 0x2104;
static const uint16_t C_OUTLINE = 0x52AA;
static const uint16_t C_TEXT    = TFT_WHITE;
static const uint16_t C_MUTED   = 0xAD55;
static const uint16_t C_YELL    = 0xFFE0;
static const uint16_t C_AMBER   = 0xFD20;
static const uint16_t C_RED     = 0xF800;
static const uint16_t C_GREEN   = 0x07E0;
static const uint16_t C_BLUEG   = 0x3186;

// ============================= Helpers =============================
static inline uint16_t u16le(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static inline bool bitSetU8(uint8_t v, uint8_t b) { return (v >> b) & 1U; }
static inline float clampf(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }

static inline lv_color_t lvcol(uint16_t rgb565) {
  uint8_t r = ((rgb565 >> 11) & 0x1F) * 255 / 31;
  uint8_t g = ((rgb565 >> 5)  & 0x3F) * 255 / 63;
  uint8_t b = ((rgb565 >> 0)  & 0x1F) * 255 / 31;
  return lv_color_make(r, g, b);
}
static inline int clampi(int v, int a, int b){ return v < a ? a : (v > b ? b : v); }

// ============================= Types =============================
struct EcuData {
  int rpm = 0;
  int iatC = 0;
  int cltC = 0;
  float vbat = 0.0f;
  float afr = 0.0f;
  int tps = 0;
  int advance = 0;
  bool warmup = false;
  bool launch = false;
  uint32_t lastUpdateMs = 0;
};

struct PrevData {
  int rpm = INT32_MIN;
  int iatC = INT32_MIN;
  int cltC = INT32_MIN;
  int vbat10 = INT32_MIN;
  int afrScaled = INT32_MIN;
  int tps = INT32_MIN;
  int advance = INT32_MIN;
  int warmup = INT32_MIN;
  int launch = INT32_MIN;
};

// -------------------- AFR format --------------------
enum AfrFormat { AFR_U16_x100 = 0, AFR_U16_x10 = 1, AFR_U8_div10 = 2 };

// -------------------- Settings (persisted) --------------------
static Preferences prefs;
static AfrFormat setting_afrFmt = AFR_U8_div10;
static uint32_t setting_logIndex = 1;
static bool setting_logEnabled = true;

// Thresholds
struct WarnCfg { bool enabled; float minV; float maxV; };

enum WarnId { W_AFR=0, W_VBAT, W_IAT, W_CLT, W_TPS, W_ADV, W_COUNT };
static WarnCfg warnCfg[W_COUNT];

// shift light
static int setting_shiftRpm = 6500;
static bool setting_shiftEnabled = true;

// view mode
enum ViewMode { VIEW_RING = 0, VIEW_BAR = 1 };
static uint8_t setting_viewMode = VIEW_RING;

// -------------------- SD logging --------------------
#if USE_SD
static bool sdOk = false;
static bool recording = false;
static File logFile;
static uint32_t lastLogMs = 0;
static volatile bool portalBusy = false; // prevents SD read while handler running
#endif

// -------------------- State --------------------
static EcuData ecu;
static PrevData prev;

// -------------------- Shift overlay --------------------
static bool shiftActive = false;
static uint32_t shiftBlinkT0 = 0;
static bool shiftBlinkOn = false;

// ============================= Speeduino 'n' frame reader =============================
static const uint8_t CMD_N = 'n';
static const int MAX_PAYLOAD = 200;
static uint8_t payload[MAX_PAYLOAD];
static uint8_t rxLen  = 0;
static int rxCount = 0;

enum RxState { WAIT_N, WAIT_TYPE, WAIT_LEN, READ_PAYLOAD };
static RxState rxState = WAIT_N;

// -------------------- Polling & Link --------------------
static uint32_t lastPoll = 0;
static uint32_t rxBytes = 0;
static uint32_t lastRxMs = 0;
static bool linkValid = false;

// ============================= Data mapping =============================
static const int TEMP_OFFSET = 40;
static const int IDX_IAT     = 6;
static const int IDX_CLT     = 7;
static const int IDX_VBAT10  = 9;
static const int IDX_RPM_L   = 14;
static const int IDX_ADVANCE = 23;
static const int IDX_TPS     = 24;
static const int IDX_ENGINE  = 2;
static const int IDX_SPARKBF = 31;
static const int AFR_INDEX   = 10;

// ============================= Tach config =============================
static const int RPM_MAX = 8000;
static const int RPM_YELLOW = 5500;
static const int RPM_REDLINE = 7000;

// ============================= Touch calibration =============================
#if USE_TOUCH
static uint16_t touchCalData[5] = { 303, 3458, 350, 3304, 7 };
#endif

// ============================= LVGL plumbing =============================
static lv_disp_draw_buf_t draw_buf;
static lv_color_t* buf1 = nullptr;
static lv_color_t* buf2 = nullptr;

static void my_flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t*)&color_p->full, w * h, true);
  tft.endWrite();
  lv_disp_flush_ready(disp);
}

#if USE_TOUCH
static void my_touch_read(lv_indev_drv_t* indev_driver, lv_indev_data_t* data) {
  static uint16_t last_x = 0, last_y = 0;
  uint16_t x, y;
  bool pressed = tft.getTouch(&x, &y);

  if (pressed) {
    last_x = x;
    last_y = y;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
  data->point.x = last_x;
  data->point.y = last_y;
}
#endif

// ============================= UI Objects =============================
// Pages
static lv_obj_t* scr_dash = nullptr;
static lv_obj_t* scr_settings = nullptr;
static lv_obj_t* scr_shift = nullptr;

// ============================= UI PAUSE (portal mode) =============================
static lv_obj_t* lbl_webmode = nullptr;

// NOTE: This must not call lv_scr_load() or otherwise "switch screens" while paused.
// That can hang if you stop calling lv_timer_handler() afterwards.
// Also must not touch LVGL objects before lvReady is true.
static void setUiPaused(bool pause) {
  if (uiPaused == pause) return;
  uiPaused = pause;

#ifdef TFT_BL
#if PAUSE_UI_BACKLIGHT_OFF
  if (pause) {
    digitalWrite(TFT_BL, !TFT_BACKLIGHT_ON);
  } else {
    digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
  }
#endif
#endif
}


// Status labels
static lv_obj_t* lbl_link = nullptr;
static lv_obj_t* lbl_rx = nullptr;
static lv_obj_t* lbl_age = nullptr;
static lv_obj_t* lbl_sd = nullptr;
static lv_obj_t* lbl_rec = nullptr;
static lv_obj_t* lbl_ver = nullptr;

// Dash buttons
static lv_obj_t* btn_rec = nullptr;
static lv_obj_t* btn_set = nullptr;


// REC button visual state (turn red while SD logging is active)
static void setRecButtonActive(bool on) {
  if (!btn_rec) return;
  lv_obj_set_style_bg_color(btn_rec, lvcol(on ? C_RED : C_PANEL), 0);
  lv_obj_set_style_border_color(btn_rec, lvcol(C_OUTLINE), 0);
  lv_obj_t* lab = lv_obj_get_child(btn_rec, 0);
  if (lab) lv_obj_set_style_text_color(lab, lv_color_white(), 0);
}

// Ring view objects (traditional gauge)
static lv_obj_t* meter_rpm = nullptr;
static lv_meter_scale_t* meter_scale_rpm = nullptr;
static lv_meter_indicator_t* meter_arc_green = nullptr;
static lv_meter_indicator_t* meter_arc_yellow = nullptr;
static lv_meter_indicator_t* meter_arc_red = nullptr;
static lv_meter_indicator_t* meter_needle = nullptr;
static lv_obj_t* lbl_rpm = nullptr;

// Bar view objects

static lv_obj_t* cont_bar = nullptr;
static lv_obj_t* bar_rpm = nullptr;
static lv_obj_t* lbl_rpm_bar = nullptr;

// Tiles
struct TileUI {
  lv_obj_t* cont;
  lv_obj_t* lbl_name;
  lv_obj_t* lbl_unit;
  lv_obj_t* lbl_value;
  lv_obj_t* bar;
  uint16_t normalBar565;
  const char* name;
  const char* unit;
};

static TileUI ui_afr, ui_vbat, ui_iat, ui_clt, ui_tps, ui_adv, ui_warm, ui_launch;
static TileUI* tiles_all[8] = { &ui_afr,&ui_vbat,&ui_iat,&ui_clt,&ui_tps,&ui_adv,&ui_warm,&ui_launch };

// Shift overlay label
static lv_obj_t* lbl_shift = nullptr;

// Settings objects
static lv_obj_t* btn_back = nullptr;
static lv_obj_t* btn_save = nullptr;
static lv_obj_t* btn_clear = nullptr; // DEFAULT button
static lv_obj_t* mbox_default = nullptr;
static lv_obj_t* list_settings = nullptr;
static lv_obj_t* btn_minmax = nullptr;
static lv_obj_t* btn_minus = nullptr;
static lv_obj_t* btn_plus = nullptr;
static lv_obj_t* lbl_saved = nullptr;
static lv_obj_t* lbl_help = nullptr;

// Settings state
static int settingsRow = 0;
static bool editMin = true;

// ======== Settings row UI tracking (for switches/value labels) ========
static const int SETTINGS_ROW_COUNT = (W_COUNT + 3); // warnings + SHIFT + VIEW + LOGGING
static lv_obj_t* settings_rows[SETTINGS_ROW_COUNT] = {0};
static lv_obj_t* settings_val_lbl[SETTINGS_ROW_COUNT] = {0};
static lv_obj_t* settings_sw[SETTINGS_ROW_COUNT] = {0};

// ============================= Settings persistence =============================
static void defaultsWarn() {
  warnCfg[W_AFR]  = { true, 10.0f, 16.5f };
  warnCfg[W_VBAT] = { true, 11.5f, 15.2f };
  warnCfg[W_IAT]  = { true, -10.0f, 60.0f };
  warnCfg[W_CLT]  = { true,  0.0f, 105.0f };
  warnCfg[W_TPS]  = { false, 0.0f, 100.0f };
  warnCfg[W_ADV]  = { false, -10.0f, 50.0f };
}

static void loadSettings() {
  defaultsWarn();
  prefs.begin("espdash", true);

  setting_logEnabled   = prefs.getBool("logEn", true);
  setting_afrFmt       = (AfrFormat)prefs.getUChar("afrFmt", (uint8_t)AFR_U8_div10);
  setting_logIndex     = prefs.getUInt("logIdx", 1);

  setting_shiftEnabled = prefs.getBool("shEn", true);
  setting_shiftRpm     = (int)prefs.getInt("shRpm", 6500);

  setting_viewMode     = prefs.getUChar("view", (uint8_t)VIEW_RING);

  for (int i = 0; i < W_COUNT; i++) {
    char kE[8], kN[8], kX[8];
    snprintf(kE, sizeof(kE), "w%de", i);
    snprintf(kN, sizeof(kN), "w%dn", i);
    snprintf(kX, sizeof(kX), "w%dx", i);
    warnCfg[i].enabled = prefs.getBool(kE, warnCfg[i].enabled);
    warnCfg[i].minV    = prefs.getFloat(kN, warnCfg[i].minV);
    warnCfg[i].maxV    = prefs.getFloat(kX, warnCfg[i].maxV);
  }
  prefs.end();
}

static void saveSettings() {
  prefs.begin("espdash", false);

  prefs.putBool("logEn", setting_logEnabled);
  prefs.putUChar("afrFmt", (uint8_t)setting_afrFmt);
  prefs.putUInt("logIdx", setting_logIndex);

  prefs.putBool("shEn", setting_shiftEnabled);
  prefs.putInt("shRpm", (int32_t)setting_shiftRpm);

  prefs.putUChar("view", setting_viewMode);

  for (int i = 0; i < W_COUNT; i++) {
    char kE[8], kN[8], kX[8];
    snprintf(kE, sizeof(kE), "w%de", i);
    snprintf(kN, sizeof(kN), "w%dn", i);
    snprintf(kX, sizeof(kX), "w%dx", i);
    prefs.putBool(kE, warnCfg[i].enabled);
    prefs.putFloat(kN, warnCfg[i].minV);
    prefs.putFloat(kX, warnCfg[i].maxV);
  }
  prefs.end();
}


// Reset warnings to defaults and persist
static void resetWarningsToDefaults() {
  defaultsWarn();
  // Persist only warning keys (keep other settings unchanged)
  prefs.begin("espdash", false);
  for (int i = 0; i < W_COUNT; i++) {
    char kE[8], kN[8], kX[8];
    snprintf(kE, sizeof(kE), "w%de", i);
    snprintf(kN, sizeof(kN), "w%dn", i);
    snprintf(kX, sizeof(kX), "w%dx", i);
    prefs.putBool(kE, warnCfg[i].enabled);
    prefs.putFloat(kN, warnCfg[i].minV);
    prefs.putFloat(kX, warnCfg[i].maxV);
  }
  prefs.end();
}


// Default confirmation popup helpers
typedef struct {
  lv_obj_t* mbox;
  bool apply;
} DefaultActionCtx;

static void default_action_timer(lv_timer_t* t) {
  DefaultActionCtx* ctx = (DefaultActionCtx*)t->user_data;
  if (ctx) {
    if (ctx->apply) {
      resetWarningsToDefaults();   // writes hard-coded defaults to Preferences
      refresh_settings_list();     // redraw settings UI to match prefs
      flashSavedMsg("DEFAULT");
    }
    if (ctx->mbox) lv_obj_del_async(ctx->mbox);
    // Release any stuck touch/press state (common after msgbox clicks)
    lv_indev_t* indev = lv_indev_get_act();
    if (indev) lv_indev_reset(indev, NULL);
    free(ctx);
  }
  lv_timer_del(t);
}

static void default_confirm_cb(lv_event_t* e) {
  lv_obj_t* mbox = (lv_obj_t*)lv_event_get_user_data(e);
  const char* txt = lv_msgbox_get_active_btn_text(mbox);
  if (!txt) return;

  bool apply = (strcmp(txt, "Yes") == 0);

  DefaultActionCtx* ctx = (DefaultActionCtx*)malloc(sizeof(DefaultActionCtx));
  if (!ctx) { if (mbox) lv_obj_del_async(mbox); return; }
  ctx->mbox = mbox;
  ctx->apply = apply;

  lv_timer_t* t = lv_timer_create(default_action_timer, 1, ctx);
  lv_timer_set_repeat_count(t, 1);
}
static void mbox_default_deleted_cb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_DELETE) return;
  mbox_default = nullptr;
}

static void showDefaultConfirm() {
  if (mbox_default) return;
  static const char* btns[] = {"No", "Yes", ""};

  // Create on the current screen so it stays in the same LVGL context
  mbox_default = lv_msgbox_create(lv_scr_act(), "Defaults", "Reset warning alarms to default values?", btns, false);
  lv_obj_center(mbox_default);
  lv_obj_add_event_cb(mbox_default, mbox_default_deleted_cb, LV_EVENT_DELETE, nullptr);

  // Attach callback to btnmatrix and pass msgbox as user_data
  lv_obj_t* btnm = lv_msgbox_get_btns(mbox_default);
  if (btnm) {
    lv_obj_add_event_cb(btnm, default_confirm_cb, LV_EVENT_VALUE_CHANGED, mbox_default);
  }
}

static bool warnCheckFloat(WarnId id, float v) {
  if (!warnCfg[id].enabled) return false;
  return (v < warnCfg[id].minV) || (v > warnCfg[id].maxV);
}
static bool warnCheckInt(WarnId id, int v) {
  if (!warnCfg[id].enabled) return false;
  return ((float)v < warnCfg[id].minV) || ((float)v > warnCfg[id].maxV);
}

// ============================= SD logging =============================
#if USE_SD
static String makeLogFilename() {
  char buf[32];
  snprintf(buf, sizeof(buf), "/log_%05lu.csv", (unsigned long)setting_logIndex);
  return String(buf);
}

static void stopRecording() {
  if (logFile) { logFile.flush(); logFile.close(); }
  recording = false;
  setRecButtonActive(false);
}

static const char* startRecording() {
  if (!setting_logEnabled) return "Logging disabled";
  if (!sdOk) return "SD card not detected";
  if (recording) return "Already recording";
  if (portalBusy) return "Portal busy";

  String fn = makeLogFilename();
  logFile = SD.open(fn.c_str(), FILE_WRITE);
  if (!logFile) return "Failed to open log file";

  logFile.println("ms,rpm,iatC,cltC,vbat,afr,tps,advance,warmup,launch");
  recording = true;
  setRecButtonActive(true);

  setting_logIndex++;
  saveSettings();
  lastLogMs = 0;
  return nullptr;
}

static void logIfRecording() {
  if (!recording || !sdOk || !logFile) return;
  if (portalBusy) return;
  if (millis() - lastLogMs < LOG_INTERVAL_MS) return;
  lastLogMs = millis();

  logFile.print(millis()); logFile.print(",");
  logFile.print(ecu.rpm);  logFile.print(",");
  logFile.print(ecu.iatC); logFile.print(",");
  logFile.print(ecu.cltC); logFile.print(",");
  logFile.print(ecu.vbat, 2); logFile.print(",");
  logFile.print(ecu.afr, 2);  logFile.print(",");
  logFile.print(ecu.tps);     logFile.print(",");
  logFile.print(ecu.advance); logFile.print(",");
  logFile.print((int)ecu.warmup); logFile.print(",");
  logFile.println((int)ecu.launch);

  static uint32_t lastFlush = 0;
  if (millis() - lastFlush > LOG_FLUSH_MS) { logFile.flush(); lastFlush = millis(); }
}
#endif

// ============================= Splash =============================
static void showBasicSplash() {
  tft.fillScreen(C_BG);
  tft.setTextColor(TFT_WHITE, C_BG);
  tft.setTextSize(2);
  const char* txt = "@BROKENAGAIN_MINI";
  int tw = tft.textWidth(txt);
  tft.setCursor((SCREEN_W - tw)/2, (SCREEN_H/2) - 14);
  tft.print(txt);

  tft.setTextSize(1);
  String v = String("esp_dash_") + FW_VERSION;
  int tw2 = tft.textWidth(v);
  tft.setCursor((SCREEN_W - tw2)/2, (SCREEN_H/2) + 14);
  tft.print(v);
}

static void showSplashThenStartSerial() {
  showBasicSplash();
  uint32_t t0 = millis();
  while (millis() - t0 < SPLASH_DELAY_MS) delay(10);

#if USE_UART0
  ECU_SERIAL.begin(ECU_BAUD);
#else
  ECU_SERIAL.begin(ECU_BAUD, SERIAL_8N1, ECU_RX_PIN, ECU_TX_PIN);
#endif
  while (ECU_SERIAL.available()) ECU_SERIAL.read();
  rxState = WAIT_N;
  rxCount = 0;
  rxLen = 0;
  linkValid = false;
  lastRxMs = 0;
}

// ============================= Decode =============================
static float decodeAfr(const uint8_t* p, int len) {
  if (setting_afrFmt == AFR_U16_x100) {
    if (AFR_INDEX + 1 >= len) return 0;
    uint16_t v = u16le(&p[AFR_INDEX]);
    return (float)v / 100.0f;
  } else if (setting_afrFmt == AFR_U16_x10) {
    if (AFR_INDEX + 1 >= len) return 0;
    uint16_t v = u16le(&p[AFR_INDEX]);
    return (float)v / 10.0f;
  } else {
    if (AFR_INDEX >= len) return 0;
    return (float)p[AFR_INDEX] / 10.0f;
  }
}

static void decodePayload(const uint8_t* p, int len) {
  if (len < 40) return;

  ecu.iatC = (int)p[IDX_IAT] - TEMP_OFFSET;
  ecu.cltC = (int)p[IDX_CLT] - TEMP_OFFSET;
  ecu.rpm  = (int)u16le(&p[IDX_RPM_L]);
  ecu.vbat = ((float)p[IDX_VBAT10]) / 10.0f;
  ecu.afr  = decodeAfr(p, len);
  ecu.advance = (int)p[IDX_ADVANCE];
  ecu.tps = ((int)p[IDX_TPS] + 1) / 2;

  uint8_t eng = p[IDX_ENGINE];
  ecu.warmup = bitSetU8(eng, 3);

  uint8_t sp = p[IDX_SPARKBF];
  ecu.launch = bitSetU8(sp, 0) || bitSetU8(sp, 1);

  ecu.lastUpdateMs = millis();
  linkValid = true;
}

static void onRxByte(uint8_t b) {
  rxBytes++;
  lastRxMs = millis();

  switch (rxState) {
    case WAIT_N:    if (b == CMD_N) rxState = WAIT_TYPE; break;
    case WAIT_TYPE: rxState = WAIT_LEN; break;
    case WAIT_LEN:
      rxLen = b;
      rxCount = 0;
      if (rxLen == 0 || rxLen > MAX_PAYLOAD) rxState = WAIT_N;
      else rxState = READ_PAYLOAD;
      break;
    case READ_PAYLOAD:
      payload[rxCount++] = b;
      if (rxCount >= rxLen) { decodePayload(payload, rxLen); rxState = WAIT_N; }
      break;
  }
}

static void pollSpeeduino() { if (rxState == WAIT_N) ECU_SERIAL.write(CMD_N); }

// ============================= UI: tiles =============================
static void style_tile_container(lv_obj_t* cont, bool warn=false) {
  lv_obj_set_style_radius(cont, 10, 0);
  lv_obj_set_style_bg_color(cont, warn ? lvcol(C_RED) : lvcol(C_PANEL), 0);
  lv_obj_set_style_border_color(cont, lvcol(C_OUTLINE), 0);
  lv_obj_set_style_border_width(cont, 2, 0);
  lv_obj_set_style_pad_all(cont, 8, 0);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
}

static TileUI make_tile(lv_obj_t* parent, int x, int y, const char* name, const char* unit, uint16_t bar565) {
  TileUI t{};
  t.normalBar565 = bar565;
  t.name = name;
  t.unit = unit;

  t.cont = lv_obj_create(parent);
  lv_obj_set_pos(t.cont, x, y);
  lv_obj_set_size(t.cont, 120, 70);
  style_tile_container(t.cont, false);

  t.lbl_name = lv_label_create(t.cont);
  lv_label_set_text(t.lbl_name, name);
  lv_obj_set_style_text_color(t.lbl_name, lvcol(C_MUTED), 0);
  lv_obj_set_style_text_font(t.lbl_name, &lv_font_montserrat_12, 0);
  lv_obj_align(t.lbl_name, LV_ALIGN_TOP_LEFT, 2, -2);

  t.lbl_unit = lv_label_create(t.cont);
  lv_label_set_text(t.lbl_unit, unit);
  lv_obj_set_style_text_color(t.lbl_unit, lvcol(C_MUTED), 0);
  lv_obj_set_style_text_font(t.lbl_unit, &lv_font_montserrat_12, 0);
  lv_obj_align(t.lbl_unit, LV_ALIGN_LEFT_MID, 2, 0);

  t.lbl_value = lv_label_create(t.cont);
  lv_label_set_text(t.lbl_value, "---");
  lv_obj_set_style_text_color(t.lbl_value, lvcol(C_TEXT), 0);
  lv_obj_set_style_text_font(t.lbl_value, &lv_font_montserrat_22, 0);
  lv_obj_align(t.lbl_value, LV_ALIGN_TOP_RIGHT, 2, 14);

  t.bar = lv_bar_create(t.cont);
  lv_obj_set_size(t.bar, 100, 10);
  lv_obj_align(t.bar, LV_ALIGN_BOTTOM_MID, 0, -2);
  lv_bar_set_range(t.bar, 0, 1000);
  lv_bar_set_value(t.bar, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(t.bar, lvcol(C_BLUEG), LV_PART_MAIN);
  lv_obj_set_style_bg_color(t.bar, lvcol(bar565), LV_PART_INDICATOR);

  return t;
}

static void set_tile_value(TileUI& t, const char* value, int bar_0_1000, bool warn=false, bool on=false) {
  style_tile_container(t.cont, warn);
  lv_label_set_text(t.lbl_value, value);

  if (warn) {
    lv_obj_set_style_text_color(t.lbl_value, lv_color_black(), 0);
    lv_obj_set_style_text_color(t.lbl_unit, lv_color_black(), 0);
    lv_obj_set_style_text_color(t.lbl_name, lv_color_black(), 0);
    lv_obj_set_style_bg_color(t.bar, lvcol(C_RED), LV_PART_MAIN);
    lv_obj_set_style_bg_color(t.bar, lvcol(C_RED), LV_PART_INDICATOR);
  } else {
    lv_obj_set_style_text_color(t.lbl_value, lvcol(C_TEXT), 0);
    lv_obj_set_style_text_color(t.lbl_unit, lvcol(C_MUTED), 0);
    lv_obj_set_style_text_color(t.lbl_name, lvcol(C_MUTED), 0);
    lv_obj_set_style_bg_color(t.bar, lvcol(C_BLUEG), LV_PART_MAIN);
    lv_obj_set_style_bg_color(t.bar, lvcol(t.normalBar565), LV_PART_INDICATOR);
  }

  lv_bar_set_value(t.bar, bar_0_1000, LV_ANIM_OFF);
  if (on) lv_bar_set_value(t.bar, 1000, LV_ANIM_OFF);
}

static void set_tile_blank(TileUI& t) {
  style_tile_container(t.cont, false);
  lv_label_set_text(t.lbl_value, "---");
  lv_bar_set_value(t.bar, 0, LV_ANIM_OFF);
}

// ============================= UI layout switching =============================
static void layout_tiles_ring();
static void layout_tiles_bar();

static void apply_view_layout() {
  if (setting_viewMode == VIEW_RING) {
    for (int i=0;i<8;i++){
      lv_obj_set_size(tiles_all[i]->cont, 105, 70);
      lv_obj_set_width(tiles_all[i]->bar, 100);
    }
    layout_tiles_ring();
  } else {
    layout_tiles_bar();
  }
}

static void layout_tiles_ring() {
  const int TILE_W = 105, TILE_H = 70, GAP_Y = 6;
  const int L_X = 4;
  const int R_X = SCREEN_W - 8 - TILE_W;
  const int TOP_Y = STATUS_H + 6;

  lv_obj_set_pos(ui_afr.cont,   L_X, TOP_Y + (TILE_H+GAP_Y)*0);
  lv_obj_set_pos(ui_vbat.cont,  L_X, TOP_Y + (TILE_H+GAP_Y)*1);
  lv_obj_set_pos(ui_iat.cont,   L_X, TOP_Y + (TILE_H+GAP_Y)*2);
  lv_obj_set_pos(ui_clt.cont,   L_X, TOP_Y + (TILE_H+GAP_Y)*3);

  lv_obj_set_pos(ui_tps.cont,   R_X, TOP_Y + (TILE_H+GAP_Y)*0);
  lv_obj_set_pos(ui_adv.cont,   R_X, TOP_Y + (TILE_H+GAP_Y)*1);
  lv_obj_set_pos(ui_warm.cont,  R_X, TOP_Y + (TILE_H+GAP_Y)*2);
  lv_obj_set_pos(ui_launch.cont,R_X, TOP_Y + (TILE_H+GAP_Y)*3);
  if (cont_bar) { lv_obj_add_flag(cont_bar, LV_OBJ_FLAG_HIDDEN); }
  if (meter_rpm) { lv_obj_clear_flag(meter_rpm, LV_OBJ_FLAG_HIDDEN); }
  if (lbl_rpm)   { lv_obj_clear_flag(lbl_rpm, LV_OBJ_FLAG_HIDDEN); }
}

static void layout_tiles_bar() {
  const int tileW = 114;
  const int tileH = 70;
  const int gapX  = 6;
  const int gapY  = 8;

  const int cols = 4;
  const int totalW = cols * tileW + (cols - 1) * gapX;
  const int startX = (SCREEN_W - totalW) / 2;

  const int row1Y = STATUS_H + 110;
  const int row2Y = row1Y + tileH + gapY;

  lv_obj_set_pos(ui_afr.cont,   startX + (tileW+gapX)*0, row1Y);
  lv_obj_set_pos(ui_vbat.cont,  startX + (tileW+gapX)*1, row1Y);
  lv_obj_set_pos(ui_tps.cont,   startX + (tileW+gapX)*2, row1Y);
  lv_obj_set_pos(ui_adv.cont,   startX + (tileW+gapX)*3, row1Y);

  lv_obj_set_pos(ui_iat.cont,   startX + (tileW+gapX)*0, row2Y);
  lv_obj_set_pos(ui_clt.cont,   startX + (tileW+gapX)*1, row2Y);
  lv_obj_set_pos(ui_warm.cont,  startX + (tileW+gapX)*2, row2Y);
  lv_obj_set_pos(ui_launch.cont,startX + (tileW+gapX)*3, row2Y);

  for (int i=0;i<8;i++){
    lv_obj_set_size(tiles_all[i]->cont, tileW, tileH);
    lv_obj_set_width(tiles_all[i]->bar, tileW - 20);
  }
  if (cont_bar) { lv_obj_clear_flag(cont_bar, LV_OBJ_FLAG_HIDDEN); }
  if (meter_rpm) { lv_obj_add_flag(meter_rpm, LV_OBJ_FLAG_HIDDEN); }
  if (lbl_rpm)   { lv_obj_add_flag(lbl_rpm, LV_OBJ_FLAG_HIDDEN); }
}

// ============================= UI: status bar =============================
static void build_status_bar(lv_obj_t* parent) {
  lv_obj_t* bar = lv_obj_create(parent);
  lv_obj_set_pos(bar, 0, 0);
  lv_obj_set_size(bar, SCREEN_W, STATUS_H);
  lv_obj_set_style_bg_color(bar, lv_color_make(0, 80, 0), 0);
  lv_obj_set_style_border_width(bar, 0, 0);
  lv_obj_set_style_radius(bar, 0, 0);
  lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);

  lbl_link = lv_label_create(bar);
  lv_label_set_text(lbl_link, "LINK: STALE");
  lv_obj_set_style_text_font(lbl_link, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lbl_link, lv_color_white(), 0);
  lv_obj_align(lbl_link, LV_ALIGN_LEFT_MID, 6, 0);

  lbl_rx = lv_label_create(bar);
  lv_label_set_text(lbl_rx, "RX:0");
  lv_obj_set_style_text_font(lbl_rx, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lbl_rx, lv_color_white(), 0);
  lv_obj_align(lbl_rx, LV_ALIGN_LEFT_MID, 120, 0);

  lbl_age = lv_label_create(bar);
  lv_label_set_text(lbl_age, "Age:0ms");
  lv_obj_set_style_text_font(lbl_age, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lbl_age, lv_color_white(), 0);
  lv_obj_align(lbl_age, LV_ALIGN_LEFT_MID, 220, 0);

  lbl_sd = lv_label_create(bar);
  lv_label_set_text(lbl_sd, "SD:--");
  lv_obj_set_style_text_font(lbl_sd, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lbl_sd, lv_color_white(), 0);
  lv_obj_align(lbl_sd, LV_ALIGN_LEFT_MID, 340, 0);

  lbl_rec = lv_label_create(bar);
  lv_label_set_text(lbl_rec, "   ");
  lv_obj_set_style_text_font(lbl_rec, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lbl_rec, lv_color_white(), 0);
  lv_obj_align(lbl_rec, LV_ALIGN_LEFT_MID, 400, 0);

  lbl_ver = lv_label_create(bar);
  lv_label_set_text(lbl_ver, FW_VERSION);
  lv_obj_set_style_text_font(lbl_ver, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lbl_ver, lv_color_white(), 0);
  lv_obj_align(lbl_ver, LV_ALIGN_RIGHT_MID, -6, 0);
}

// ============================= Bar view (rpm bar) =============================
static void add_rpm_scale(lv_obj_t* parent) {
  const int tickCount = 5;  // 0, 2000, 4000, 6000, 8000
  const int barWidth = SCREEN_W - 24;
  const int barStartX = 2 + 12;

  for (int i = 0; i < tickCount; i++) {
    int rpmVal = i * 2000;
    float ratio = (float)rpmVal / RPM_MAX;
    int xPos = barStartX + (int)(ratio * barWidth);

    lv_obj_t* tick = lv_obj_create(parent);
    lv_obj_set_size(tick, 2, 6);
    lv_obj_set_pos(tick, xPos - 1, 28);
    lv_obj_set_style_bg_color(tick, lvcol(C_MUTED), 0);
    lv_obj_set_style_border_width(tick, 0, 0);
    lv_obj_clear_flag(tick, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t* label = lv_label_create(parent);
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", rpmVal);
    lv_label_set_text(label, buf);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(label, lvcol(C_MUTED), 0);
    lv_obj_align_to(label, tick, LV_ALIGN_OUT_BOTTOM_MID, 0, 2);
  }
}

static void build_bar_view(lv_obj_t* parent) {
  cont_bar = lv_obj_create(parent);
  lv_obj_set_pos(cont_bar, 0, STATUS_H);
  lv_obj_set_size(cont_bar, SCREEN_W, 88);
  lv_obj_set_style_bg_opa(cont_bar, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(cont_bar, 0, 0);
  lv_obj_clear_flag(cont_bar, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* panel = lv_obj_create(cont_bar);
  lv_obj_set_pos(panel, 2, 6);
  lv_obj_set_size(panel, SCREEN_W - 35, 40);
  lv_obj_set_style_radius(panel, 8, 0);
  lv_obj_set_style_bg_color(panel, lvcol(C_PANEL), 0);
  lv_obj_set_style_border_color(panel, lvcol(C_OUTLINE), 0);
  lv_obj_set_style_border_width(panel, 2, 0);
  lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

  bar_rpm = lv_bar_create(panel);
  lv_obj_set_size(bar_rpm, SCREEN_W - 24, 14);
  lv_obj_align(bar_rpm, LV_ALIGN_CENTER, 0, -2);
  lv_bar_set_range(bar_rpm, 0, RPM_MAX);
  lv_bar_set_value(bar_rpm, 0, LV_ANIM_OFF);
  lv_obj_set_style_radius(bar_rpm, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(bar_rpm, lvcol(C_PANEL), LV_PART_MAIN);
  lv_obj_set_style_bg_color(bar_rpm, lvcol(C_GREEN), LV_PART_INDICATOR);
  add_rpm_scale(cont_bar);

  lbl_rpm_bar = lv_label_create(cont_bar);
  lv_label_set_text(lbl_rpm_bar, "0 RPM");
  lv_obj_set_style_text_font(lbl_rpm_bar, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(lbl_rpm_bar, lvcol(C_TEXT), 0);
  lv_obj_align(lbl_rpm_bar, LV_ALIGN_TOP_MID, 0, 48);
}

static void update_bar_rpm(int rpm) {
  rpm = clampi(rpm, 0, RPM_MAX);
  if (!bar_rpm) return;

  lv_bar_set_value(bar_rpm, rpm, LV_ANIM_OFF);

  if (rpm >= RPM_REDLINE) lv_obj_set_style_bg_color(bar_rpm, lvcol(C_RED), LV_PART_INDICATOR);
  else if (rpm >= RPM_YELLOW) lv_obj_set_style_bg_color(bar_rpm, lvcol(C_YELL), LV_PART_INDICATOR);
  else lv_obj_set_style_bg_color(bar_rpm, lvcol(C_GREEN), LV_PART_INDICATOR);

  static char buf[20];
  snprintf(buf, sizeof(buf), "%d RPM", rpm);
  lv_label_set_text(lbl_rpm_bar, buf);
}

// ============================= Saved indicator =============================
static uint32_t savedUntilMs = 0;
static void flashSavedMsg(const char* msg) {
  savedUntilMs = millis() + 2000;
  if (lbl_saved) {
    lv_label_set_text(lbl_saved, msg);
    lv_obj_clear_flag(lbl_saved, LV_OBJ_FLAG_HIDDEN);
  }
}
static void flashSaved() { flashSavedMsg("SAVED"); }


// ============================= Toast / popup =============================
// Lightweight message box that auto-closes (used for REC/SD logging errors).
static lv_obj_t* mbox_toast = nullptr;

static void toast_deleted_cb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_DELETE) return;
  mbox_toast = nullptr;
}

static void toast_timer_cb(lv_timer_t* t) {
  if (mbox_toast) lv_obj_del_async(mbox_toast);
  lv_timer_del(t);
}

static void showToast(const char* title, const char* msg) {
  if (!lvReady) return;
  // If we're in portalMode, LVGL is stopped; don't create popups.
  if (portalMode) return;

  if (mbox_toast) { lv_obj_del_async(mbox_toast); mbox_toast = nullptr; }

  static const char* btns[] = {"OK", ""};
  mbox_toast = lv_msgbox_create(lv_scr_act(), title ? title : "", msg ? msg : "", btns, true);
  lv_obj_center(mbox_toast);
  lv_obj_add_event_cb(mbox_toast, toast_deleted_cb, LV_EVENT_DELETE, nullptr);

  lv_timer_t* t = lv_timer_create(toast_timer_cb, 1800, nullptr);
  lv_timer_set_repeat_count(t, 1);
}

// ============================= UI events =============================
static void refresh_settings_list();

static float warnStep(int row) {
  switch (row) {
    case W_AFR:  return 0.1f;
    case W_VBAT: return 0.1f;
    default:     return 1.0f;
  }
}

static const char* warnName(int id) {
  switch(id){
    case W_AFR: return "AFR";
    case W_VBAT:return "VBAT";
    case W_IAT: return "IAT";
    case W_CLT: return "CLT";
    case W_TPS: return "TPS";
    case W_ADV: return "ADV";
    default: return "?";
  }
}

static void format_warn_range(int i, char* out, size_t outSz) {
  const bool oneDec = (i == W_AFR || i == W_VBAT);
  const char* which = editMin ? "MIN" : "MAX";
  if (oneDec) {
    snprintf(out, outSz, "%s %.1f..%.1f", which, warnCfg[i].minV, warnCfg[i].maxV);
  } else {
    snprintf(out, outSz, "%s %.0f..%.0f", which, warnCfg[i].minV, warnCfg[i].maxV);
  }
}

static void settings_apply_highlight() {
  for (int i = 0; i < SETTINGS_ROW_COUNT; i++) {
    lv_obj_t* r = settings_rows[i];
    if (!r) continue;
    lv_obj_set_style_bg_opa(r, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_outline_width(r, 0, LV_PART_MAIN);

    lv_obj_t* title = lv_obj_get_child(r, 0);
    if (title) lv_obj_set_style_text_color(title, lvcol(C_TEXT), 0);
    lv_obj_t* val = settings_val_lbl[i];
    if (val) lv_obj_set_style_text_color(val, lvcol(C_MUTED), 0);
  }

  settingsRow = clampi(settingsRow, 0, SETTINGS_ROW_COUNT-1);
  lv_obj_t* rowObj = settings_rows[settingsRow];
  if (!rowObj) return;

  lv_obj_set_style_bg_color(rowObj, lv_color_make(0, 140, 0), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rowObj, LV_OPA_30, LV_PART_MAIN);
  lv_obj_set_style_outline_color(rowObj, lv_color_make(0, 255, 0), LV_PART_MAIN);
  lv_obj_set_style_outline_width(rowObj, 2, LV_PART_MAIN);
  lv_obj_set_style_outline_pad(rowObj, 2, LV_PART_MAIN);

  lv_obj_t* title = lv_obj_get_child(rowObj, 0);
  if (title) lv_obj_set_style_text_color(title, lv_color_white(), 0);
  lv_obj_t* val = settings_val_lbl[settingsRow];
  if (val) lv_obj_set_style_text_color(val, lv_color_white(), 0);

  lv_obj_scroll_to_view(rowObj, LV_ANIM_OFF);
}

static void btn_event_cb(lv_event_t* e) {
  lv_obj_t* obj = lv_event_get_target(e);

  if (obj == btn_rec) {
#if USE_SD
    if (!recording) {
      const char* err = startRecording();
      if (err) showToast("SD LOG", err);
    } else {
      stopRecording();
    }
#endif
  } else if (obj == btn_set) {
    refresh_settings_list();
    lv_scr_load(scr_settings);
  } else if (obj == btn_back) {
    lv_scr_load(scr_dash);
  } else if (obj == btn_save) {
    saveSettings();
    flashSaved();
  } else if (obj == btn_clear) {
    showDefaultConfirm();
  } else if (obj == btn_minmax) {
    editMin = !editMin;
    lv_label_set_text(lv_obj_get_child(btn_minmax, 0), editMin ? "MIN" : "MAX");
    refresh_settings_list();
  } else if (obj == btn_minus || obj == btn_plus) {
    float step = warnStep(settingsRow);
    float dir = (obj == btn_plus) ? 1.0f : -1.0f;

    if (settingsRow < W_COUNT) {
      if (editMin) warnCfg[settingsRow].minV += dir * step;
      else         warnCfg[settingsRow].maxV += dir * step;
      if (warnCfg[settingsRow].minV > warnCfg[settingsRow].maxV) {
        float mid = 0.5f*(warnCfg[settingsRow].minV + warnCfg[settingsRow].maxV);
        warnCfg[settingsRow].minV = warnCfg[settingsRow].maxV = mid;
      }
    } else if (settingsRow == W_COUNT) {
      setting_shiftRpm = clampi(setting_shiftRpm + (int)(dir*100), 0, RPM_MAX);
    } else if (settingsRow == W_COUNT + 1) {
      setting_viewMode = (setting_viewMode == VIEW_RING) ? VIEW_BAR : VIEW_RING;
      apply_view_layout();
    } else {
      // LOGGING row: +/- do nothing
    }

    refresh_settings_list();
  }
}

static void settings_row_click_cb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  int idx = (int)(intptr_t)lv_event_get_user_data(e);
  settingsRow = clampi(idx, 0, SETTINGS_ROW_COUNT-1);
  settings_apply_highlight();
}

static void settings_switch_cb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
  int idx = (int)(intptr_t)lv_event_get_user_data(e);
  settingsRow = clampi(idx, 0, SETTINGS_ROW_COUNT-1);

  lv_obj_t* sw = lv_event_get_target(e);
  const bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);

  if (idx < W_COUNT) {
    warnCfg[idx].enabled = on;
  } else if (idx == W_COUNT) {
    setting_shiftEnabled = on;
  } else if (idx == W_COUNT + 1) {
    setting_viewMode = on ? VIEW_BAR : VIEW_RING;
    apply_view_layout();
  } else {
    setting_logEnabled = on;
  }

  refresh_settings_list();
  settings_apply_highlight();
}

// ============================= UI: build DASH =============================
static void build_dash() {
  scr_dash = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr_dash, lvcol(C_BG), 0);
  lv_obj_clear_flag(scr_dash, LV_OBJ_FLAG_SCROLLABLE);

  build_status_bar(scr_dash);

  // Web mode banner (hidden by default)
  lbl_webmode = lv_label_create(scr_dash);
  lv_label_set_text(lbl_webmode, "WEB CONFIG MODE");
  lv_obj_set_style_text_font(lbl_webmode, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(lbl_webmode, lvcol(C_GREEN), 0);
  lv_obj_align(lbl_webmode, LV_ALIGN_TOP_MID, 0, STATUS_H + 4);
  lv_obj_add_flag(lbl_webmode, LV_OBJ_FLAG_HIDDEN);

  // Traditional RPM gauge (meter + needle)
const int cx = 240, cy = 150;
const int r  = 122;

meter_rpm = lv_meter_create(scr_dash);
lv_obj_set_size(meter_rpm, r*2, r*2);
lv_obj_set_pos(meter_rpm, cx - r, cy - r);

// Transparent background, no border
lv_obj_set_style_bg_opa(meter_rpm, LV_OPA_TRANSP, 0);
lv_obj_set_style_border_width(meter_rpm, 0, 0);
lv_obj_set_style_pad_all(meter_rpm, 0, 0);
lv_obj_clear_flag(meter_rpm, LV_OBJ_FLAG_SCROLLABLE);
lv_obj_clear_flag(meter_rpm, LV_OBJ_FLAG_CLICKABLE);

meter_scale_rpm = lv_meter_add_scale(meter_rpm);

// 240-degree sweep starting at 150deg (matches old ring 150..390)
lv_meter_set_scale_range(meter_rpm, meter_scale_rpm, 0, RPM_MAX, 240, 150);

// Ticks
lv_meter_set_scale_ticks(meter_rpm, meter_scale_rpm, 41, 2, 10, lvcol(C_MUTED));
lv_meter_set_scale_major_ticks(meter_rpm, meter_scale_rpm, 8, 4, 15, lvcol(C_TEXT), 12);

// Colored arcs: green -> yellow -> red
meter_arc_green  = lv_meter_add_arc(meter_rpm, meter_scale_rpm, 14, lvcol(C_BLUEG), 0);
lv_meter_set_indicator_start_value(meter_rpm, meter_arc_green, 0);
lv_meter_set_indicator_end_value  (meter_rpm, meter_arc_green, RPM_YELLOW);

meter_arc_yellow = lv_meter_add_arc(meter_rpm, meter_scale_rpm, 14, lvcol(C_YELL), 0);
lv_meter_set_indicator_start_value(meter_rpm, meter_arc_yellow, RPM_YELLOW);
lv_meter_set_indicator_end_value  (meter_rpm, meter_arc_yellow, RPM_REDLINE);

meter_arc_red    = lv_meter_add_arc(meter_rpm, meter_scale_rpm, 14, lvcol(C_RED), 0);
lv_meter_set_indicator_start_value(meter_rpm, meter_arc_red, RPM_REDLINE);
lv_meter_set_indicator_end_value  (meter_rpm, meter_arc_red, RPM_MAX);

// Needle
meter_needle = lv_meter_add_needle_line(meter_rpm, meter_scale_rpm, 4, lvcol(C_RED), -10);
lv_meter_set_indicator_value(meter_rpm, meter_needle, 0);

lbl_rpm = lv_label_create(scr_dash);
  lv_label_set_text(lbl_rpm, "0");
  lv_obj_set_style_text_font(lbl_rpm, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(lbl_rpm, lvcol(C_TEXT), 0);
  lv_obj_align(lbl_rpm, LV_ALIGN_CENTER, 0, 52);

  ui_afr    = make_tile(scr_dash, 0,0, "AFR",  "",   C_YELL);
  ui_vbat   = make_tile(scr_dash, 0,0, "VBAT", "V",  C_GREEN);
  ui_iat    = make_tile(scr_dash, 0,0, "IAT",  "C",  C_AMBER);
  ui_clt    = make_tile(scr_dash, 0,0, "CLT",  "C",  C_AMBER);
  ui_tps    = make_tile(scr_dash, 0,0, "TPS",  "%",  C_GREEN);
  ui_adv    = make_tile(scr_dash, 0,0, "ADV",  "deg",C_YELL);
  ui_warm   = make_tile(scr_dash, 0,0, "WARMUP","",  C_AMBER);
  ui_launch = make_tile(scr_dash, 0,0, "LAUNCH","",  C_RED);

  build_bar_view(scr_dash);

  btn_rec = lv_btn_create(scr_dash);
  lv_obj_set_size(btn_rec, 90, 32);
  lv_obj_set_pos(btn_rec, 140, SCREEN_H - 40);
  lv_obj_set_style_radius(btn_rec, 8, 0);
  lv_obj_set_style_bg_color(btn_rec, lvcol(C_PANEL), 0);
  lv_obj_set_style_border_color(btn_rec, lvcol(C_OUTLINE), 0);
  lv_obj_set_style_border_width(btn_rec, 2, 0);
  lv_obj_add_event_cb(btn_rec, btn_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lr = lv_label_create(btn_rec);
  lv_label_set_text(lr, "REC");
  lv_obj_set_style_text_color(lr, lv_color_white(), 0);
  lv_obj_center(lr);
  setRecButtonActive(recording);


  btn_set = lv_btn_create(scr_dash);
  lv_obj_set_size(btn_set, 90, 32);
  lv_obj_set_pos(btn_set, SCREEN_W - 230, SCREEN_H - 40);
  lv_obj_set_style_radius(btn_set, 8, 0);
  lv_obj_set_style_bg_color(btn_set, lvcol(C_PANEL), 0);
  lv_obj_set_style_border_color(btn_set, lvcol(C_OUTLINE), 0);
  lv_obj_set_style_border_width(btn_set, 2, 0);
  lv_obj_add_event_cb(btn_set, btn_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t* ls = lv_label_create(btn_set);
  lv_label_set_text(ls, "SET");
  lv_obj_set_style_text_color(ls, lv_color_white(), 0);
  lv_obj_center(ls);

  scr_shift = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr_shift, lvcol(C_RED), 0);
  lv_obj_clear_flag(scr_shift, LV_OBJ_FLAG_SCROLLABLE);
  lbl_shift = lv_label_create(scr_shift);
  lv_label_set_text(lbl_shift, "SHIFT");
  lv_obj_set_style_text_font(lbl_shift, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(lbl_shift, lv_color_black(), 0);
  lv_obj_center(lbl_shift);

  apply_view_layout();
}

// ============================= Settings screen =============================
static void build_settings() {
  scr_settings = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr_settings, lvcol(C_BG), 0);
  lv_obj_clear_flag(scr_settings, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* top = lv_obj_create(scr_settings);
  lv_obj_set_pos(top, 0, 0);
  lv_obj_set_size(top, SCREEN_W, STATUS_H);
  lv_obj_set_style_bg_color(top, lvcol(C_PANEL), 0);
  lv_obj_set_style_border_width(top, 0, 0);
  lv_obj_set_style_radius(top, 0, 0);
  lv_obj_clear_flag(top, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* title = lv_label_create(top);
  lv_label_set_text(title, "SETTINGS");
  lv_obj_set_style_text_color(title, lvcol(C_TEXT), 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_12, 0);
  lv_obj_align(title, LV_ALIGN_LEFT_MID, 6, 0);

  btn_back = lv_btn_create(scr_settings);
  lv_obj_set_size(btn_back, 90, 32);
  lv_obj_set_pos(btn_back, 8, SCREEN_H - 40);
  lv_obj_set_style_radius(btn_back, 8, 0);
  lv_obj_set_style_bg_color(btn_back, lvcol(C_PANEL), 0);
  lv_obj_set_style_border_color(btn_back, lvcol(C_OUTLINE), 0);
  lv_obj_set_style_border_width(btn_back, 2, 0);
  lv_obj_add_event_cb(btn_back, btn_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lb = lv_label_create(btn_back);
  lv_label_set_text(lb, "BACK");
  lv_obj_set_style_text_color(lb, lv_color_white(), 0);
  lv_obj_center(lb);

  btn_save = lv_btn_create(scr_settings);
  lv_obj_set_size(btn_save, 90, 32);
  lv_obj_set_pos(btn_save, 110, SCREEN_H - 40);
  lv_obj_set_style_radius(btn_save, 8, 0);
  lv_obj_set_style_bg_color(btn_save, lvcol(C_PANEL), 0);
  lv_obj_set_style_border_color(btn_save, lvcol(C_OUTLINE), 0);
  lv_obj_set_style_border_width(btn_save, 2, 0);
  lv_obj_add_event_cb(btn_save, btn_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t* ls = lv_label_create(btn_save);
  lv_label_set_text(ls, "SAVE");
  lv_obj_set_style_text_color(ls, lv_color_white(), 0);
  lv_obj_center(ls);

  btn_clear = lv_btn_create(scr_settings);
  lv_obj_set_size(btn_clear, 90, 32);
  lv_obj_set_pos(btn_clear, 212, SCREEN_H - 40);
  lv_obj_set_style_radius(btn_clear, 8, 0);
  lv_obj_set_style_bg_color(btn_clear, lvcol(C_PANEL), 0);
  lv_obj_set_style_border_color(btn_clear, lvcol(C_OUTLINE), 0);
  lv_obj_set_style_border_width(btn_clear, 2, 0);
  lv_obj_add_event_cb(btn_clear, btn_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lc = lv_label_create(btn_clear);
  lv_label_set_text(lc, "DEFAULT");
  lv_obj_set_style_text_color(lc, lv_color_white(), 0);
  lv_obj_center(lc);


  btn_minus = lv_btn_create(scr_settings);
  lv_obj_set_size(btn_minus, 48, 32);
  lv_obj_set_pos(btn_minus, 310, SCREEN_H - 40);
  lv_obj_set_style_radius(btn_minus, 8, 0);
  lv_obj_set_style_bg_color(btn_minus, lvcol(C_PANEL), 0);
  lv_obj_set_style_border_color(btn_minus, lvcol(C_OUTLINE), 0);
  lv_obj_set_style_border_width(btn_minus, 2, 0);
  lv_obj_add_event_cb(btn_minus, btn_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lm = lv_label_create(btn_minus);
  lv_label_set_text(lm, "-");
  lv_obj_set_style_text_color(lm, lv_color_white(), 0);
  lv_obj_center(lm);

  btn_plus = lv_btn_create(scr_settings);
  lv_obj_set_size(btn_plus, 48, 32);
  lv_obj_set_pos(btn_plus, 364, SCREEN_H - 40);
  lv_obj_set_style_radius(btn_plus, 8, 0);
  lv_obj_set_style_bg_color(btn_plus, lvcol(C_PANEL), 0);
  lv_obj_set_style_border_color(btn_plus, lvcol(C_OUTLINE), 0);
  lv_obj_set_style_border_width(btn_plus, 2, 0);
  lv_obj_add_event_cb(btn_plus, btn_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lp = lv_label_create(btn_plus);
  lv_label_set_text(lp, "+");
  lv_obj_set_style_text_color(lp, lv_color_white(), 0);
  lv_obj_center(lp);

  lbl_saved = lv_label_create(scr_settings);
  lv_label_set_text(lbl_saved, "SAVED");
  lv_obj_set_style_text_font(lbl_saved, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl_saved, lvcol(C_GREEN), 0);
  lv_obj_align(lbl_saved, LV_ALIGN_TOP_RIGHT, -12, STATUS_H + 10);
  lv_obj_add_flag(lbl_saved, LV_OBJ_FLAG_HIDDEN);

  btn_minmax = lv_btn_create(scr_settings);
  lv_obj_set_size(btn_minmax, 56, 32);
  lv_obj_set_pos(btn_minmax, 416, SCREEN_H - 40);
  lv_obj_set_style_radius(btn_minmax, 8, 0);
  lv_obj_set_style_bg_color(btn_minmax, lvcol(C_PANEL), 0);
  lv_obj_set_style_border_color(btn_minmax, lvcol(C_OUTLINE), 0);
  lv_obj_set_style_border_width(btn_minmax, 2, 0);
  lv_obj_add_event_cb(btn_minmax, btn_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t* lmm = lv_label_create(btn_minmax);
  lv_label_set_text(lmm, "MIN");
  lv_obj_set_style_text_color(lmm, lv_color_white(), 0);
  lv_obj_center(lmm);

  lbl_help = lv_label_create(scr_settings);
  lv_label_set_text(lbl_help, "Swipe list to scroll. Tap row to select. Use MIN/MAX + +/- to edit. DEFAULT resets warnings.");
  lv_obj_set_style_text_font(lbl_help, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lbl_help, lvcol(C_MUTED), 0);
  lv_obj_set_pos(lbl_help, 12, STATUS_H + 6);

  list_settings = lv_obj_create(scr_settings);
  lv_obj_set_pos(list_settings, 12, STATUS_H + 26);
  lv_obj_set_size(list_settings, SCREEN_W - 24, SCREEN_H - (STATUS_H + 26) - 50);
  lv_obj_set_style_bg_color(list_settings, lvcol(C_BG), 0);
  lv_obj_set_style_border_color(list_settings, lvcol(C_OUTLINE), 0);
  lv_obj_set_style_border_width(list_settings, 1, 0);
  lv_obj_set_style_pad_row(list_settings, 6, 0);
  lv_obj_set_style_pad_all(list_settings, 6, 0);
  lv_obj_set_flex_flow(list_settings, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(list_settings, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_set_scroll_dir(list_settings, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(list_settings, LV_SCROLLBAR_MODE_AUTO);
}

// Create one row: [Title] [Value text] [Switch]
static lv_obj_t* create_settings_row(lv_obj_t* parent, int idx, const char* titleTxt) {
  lv_obj_t* row = lv_obj_create(parent);
  lv_obj_set_width(row, lv_pct(100));
  lv_obj_set_height(row, 44);
  lv_obj_set_style_radius(row, 10, 0);
  lv_obj_set_style_border_color(row, lvcol(C_OUTLINE), 0);
  lv_obj_set_style_border_width(row, 1, 0);
  lv_obj_set_style_bg_color(row, lvcol(C_PANEL), 0);
  lv_obj_set_style_bg_opa(row, LV_OPA_30, 0);
  lv_obj_set_style_pad_all(row, 8, 0);
  lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_add_event_cb(row, settings_row_click_cb, LV_EVENT_CLICKED, (void*)(intptr_t)idx);

  lv_obj_t* title = lv_label_create(row);
  lv_label_set_text(title, titleTxt);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(title, lvcol(C_TEXT), 0);
  lv_obj_align(title, LV_ALIGN_LEFT_MID, 0, 0);

  lv_obj_t* sw = lv_switch_create(row);
  lv_obj_align(sw, LV_ALIGN_RIGHT_MID, 0, 0);
  lv_obj_add_event_cb(sw, settings_switch_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)idx);

  lv_obj_t* val = lv_label_create(row);
  lv_obj_set_style_text_font(val, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(val, lvcol(C_MUTED), 0);
  lv_label_set_long_mode(val, LV_LABEL_LONG_DOT);
  lv_obj_set_width(val, 230);
  lv_obj_align_to(val, sw, LV_ALIGN_OUT_LEFT_MID, -10, 0);

  settings_rows[idx] = row;
  settings_sw[idx] = sw;
  settings_val_lbl[idx] = val;
  return row;
}

static void refresh_settings_list() {
  if (!list_settings) return;

  for (int i=0;i<SETTINGS_ROW_COUNT;i++){
    settings_rows[i] = nullptr;
    settings_sw[i] = nullptr;
    settings_val_lbl[i] = nullptr;
  }

  lv_obj_clean(list_settings);

  for (int i=0;i<W_COUNT;i++){
    char titleBuf[24];
    snprintf(titleBuf, sizeof(titleBuf), "%s WARN", warnName(i));

    create_settings_row(list_settings, i, titleBuf);

    char valBuf[48];
    format_warn_range(i, valBuf, sizeof(valBuf));
    lv_label_set_text(settings_val_lbl[i], valBuf);

    if (warnCfg[i].enabled) lv_obj_add_state(settings_sw[i], LV_STATE_CHECKED);
    else                   lv_obj_clear_state(settings_sw[i], LV_STATE_CHECKED);
  }

  {
    const int idx = W_COUNT;
    create_settings_row(list_settings, idx, "SHIFT");
    char v[32];
    snprintf(v, sizeof(v), "%d rpm", setting_shiftRpm);
    lv_label_set_text(settings_val_lbl[idx], v);

    if (setting_shiftEnabled) lv_obj_add_state(settings_sw[idx], LV_STATE_CHECKED);
    else                      lv_obj_clear_state(settings_sw[idx], LV_STATE_CHECKED);
  }

  {
    const int idx = W_COUNT + 1;
    create_settings_row(list_settings, idx, "VIEW");
    lv_label_set_text(settings_val_lbl[idx], (setting_viewMode==VIEW_RING) ? "RING" : "BAR");

    if (setting_viewMode==VIEW_BAR) lv_obj_add_state(settings_sw[idx], LV_STATE_CHECKED);
    else                            lv_obj_clear_state(settings_sw[idx], LV_STATE_CHECKED);
  }

  {
    const int idx = W_COUNT + 2;
    create_settings_row(list_settings, idx, "LOGGING");
    lv_label_set_text(settings_val_lbl[idx], "SD log");

    if (setting_logEnabled) lv_obj_add_state(settings_sw[idx], LV_STATE_CHECKED);
    else                    lv_obj_clear_state(settings_sw[idx], LV_STATE_CHECKED);
  }

  settingsRow = clampi(settingsRow, 0, SETTINGS_ROW_COUNT-1);
  settings_apply_highlight();
}

// ============================= UI update =============================
static void update_status_bar(bool stale) {
  lv_obj_t* bar = lv_obj_get_parent(lbl_link);
  if (stale) {
    lv_obj_set_style_bg_color(bar, lv_color_make(120, 0, 0), 0);
    lv_label_set_text(lbl_link, "LINK: STALE");
  } else {
    lv_obj_set_style_bg_color(bar, lv_color_make(0, 80, 0), 0);
    lv_label_set_text(lbl_link, "LINK: OK");
  }

  static char b1[24], b2[24];
  snprintf(b1, sizeof(b1), "RX:%lu", (unsigned long)rxBytes);
  snprintf(b2, sizeof(b2), "Age:%lums", (unsigned long)(millis() - lastRxMs));
  lv_label_set_text(lbl_rx, b1);
  lv_label_set_text(lbl_age, b2);

#if USE_SD
  lv_label_set_text(lbl_sd, sdOk ? "SD:OK" : "SD:NO");
  lv_label_set_text(lbl_rec, recording ? "REC" : "   ");
#else
  lv_label_set_text(lbl_sd, "SD:--");
#endif
}

static void update_dash_values() {
  bool stale = (millis() - lastRxMs) > LINK_STALE_MS;
  if (stale) linkValid = false;

  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > STATUS_UPDATE_MS) {
    update_status_bar(stale);
    lastStatus = millis();
  }

  if (setting_shiftEnabled && linkValid && ecu.rpm >= setting_shiftRpm) {
    if (!shiftActive) {
      shiftActive = true;
      shiftBlinkT0 = millis();
      shiftBlinkOn = true;
      lv_scr_load(scr_shift);
    }
    uint32_t now = millis();
    if (now - shiftBlinkT0 >= SHIFT_FLASH_MS) {
      shiftBlinkT0 = now;
      shiftBlinkOn = !shiftBlinkOn;
      if (shiftBlinkOn) lv_obj_set_style_bg_color(scr_shift, lvcol(C_RED), 0);
      else              lv_obj_set_style_bg_color(scr_shift, lvcol(C_BG), 0);
    }
    return;
  } else {
    if (shiftActive) {
      shiftActive = false;
      lv_scr_load(scr_dash);
    }
  }

  if (!linkValid) {
    for (int i=0;i<8;i++) set_tile_blank(*tiles_all[i]);
    lv_label_set_text(lbl_rpm, "0");
    if (meter_rpm && meter_needle) lv_meter_set_indicator_value(meter_rpm, meter_needle, 0);
    update_bar_rpm(0);
    prev = PrevData();
    prev.rpm = 0;
    return;
  }

  if (ecu.rpm != prev.rpm) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%d", ecu.rpm);
    lv_label_set_text(lbl_rpm, buf);
    if (meter_rpm && meter_needle) lv_meter_set_indicator_value(meter_rpm, meter_needle, ecu.rpm);
    update_bar_rpm(ecu.rpm);
    prev.rpm = ecu.rpm;
  }

  int afr100 = (int)lroundf(ecu.afr * 100.0f);
  if (afr100 != prev.afrScaled) {
    bool warn = warnCheckFloat(W_AFR, ecu.afr);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.2f", ecu.afr);
    int bar = (int)(clampf((ecu.afr - 9.0f) / (20.0f - 9.0f), 0, 1) * 1000);
    set_tile_value(ui_afr, buf, bar, warn);
    prev.afrScaled = afr100;
  }

  int vbat10 = (int)lroundf(ecu.vbat * 10.0f);
  if (vbat10 != prev.vbat10) {
    bool warn = warnCheckFloat(W_VBAT, ecu.vbat);
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", ecu.vbat);
    int bar = (int)(clampf((ecu.vbat - 10.0f) / (15.5f - 10.0f), 0, 1) * 1000);
    set_tile_value(ui_vbat, buf, bar, warn);
    prev.vbat10 = vbat10;
  }

  if (ecu.iatC != prev.iatC) {
    bool warn = warnCheckInt(W_IAT, ecu.iatC);
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", ecu.iatC);
    int bar = (int)(clampf(((float)ecu.iatC - -20.0f) / (80.0f - -20.0f), 0, 1) * 1000);
    set_tile_value(ui_iat, buf, bar, warn);
    prev.iatC = ecu.iatC;
  }

  if (ecu.cltC != prev.cltC) {
    bool warn = warnCheckInt(W_CLT, ecu.cltC);
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", ecu.cltC);
    int bar = (int)(clampf(((float)ecu.cltC - 0.0f) / (120.0f - 0.0f), 0, 1) * 1000);
    set_tile_value(ui_clt, buf, bar, warn);
    prev.cltC = ecu.cltC;
  }

  if (ecu.tps != prev.tps) {
    bool warn = warnCheckInt(W_TPS, ecu.tps);
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", ecu.tps);
    int bar = (int)(clampf((float)ecu.tps / 100.0f, 0, 1) * 1000);
    set_tile_value(ui_tps, buf, bar, warn);
    prev.tps = ecu.tps;
  }

  if (ecu.advance != prev.advance) {
    bool warn = warnCheckInt(W_ADV, ecu.advance);
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", ecu.advance);
    int bar = (int)(clampf(((float)ecu.advance - -10.0f) / (50.0f - -10.0f), 0, 1) * 1000);
    set_tile_value(ui_adv, buf, bar, warn);
    prev.advance = ecu.advance;
  }

  if ((int)ecu.warmup != prev.warmup) {
    set_tile_value(ui_warm, ecu.warmup ? "ACTIVE" : "----", 0, false, ecu.warmup);
    prev.warmup = (int)ecu.warmup;
  }

  if ((int)ecu.launch != prev.launch) {
    set_tile_value(ui_launch, ecu.launch ? "ACTIVE" : "----", 0, false, ecu.launch);
    prev.launch = (int)ecu.launch;
  }
}

// ============================= LVGL tick helper =============================
static uint32_t lastTick = 0;
static void lvglTick() {
  uint32_t now = millis();
  uint32_t diff = now - lastTick;
  lastTick = now;
  lv_tick_inc(diff);
}

// ============================= PORTAL SCREEN (TFT direct) =============================
// When a WiFi station connects, we STOP LVGL completely and draw a simple TFT screen.
// This avoids LVGL + webserver contention and prevents "web pages not loading" freezes.
static void drawPortalScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.setTextSize(2);
  tft.setCursor(18, 18);
  tft.print("WEB CONFIGURATION MODE");

  tft.setTextSize(1);
  tft.setCursor(18, 58);
  tft.print("Connect to WiFi AP:");

  tft.setTextSize(2);
  tft.setCursor(18, 78);
  tft.print(WIFI_AP_SSID);

  tft.setTextSize(1);
  tft.setCursor(18, 112);
  tft.print("Password: ");
  tft.print(WIFI_AP_PASS);

  // IP address
#if USE_WIFI
  IPAddress ip = WiFi.softAPIP();
  tft.setCursor(18, 136);
  tft.print("Open in browser: http://");
  tft.print(ip[0]); tft.print(".");
  tft.print(ip[1]); tft.print(".");
  tft.print(ip[2]); tft.print(".");
  tft.print(ip[3]);
#else
  tft.setCursor(18, 136);
  tft.print("WiFi disabled in build");
#endif

  tft.setCursor(18, 160);
  tft.print("Tip: When connected, dashboard UI is stopped.");
  tft.setCursor(18, 174);
  tft.print("Disconnect from AP to return to dashboard.");

  tft.setCursor(18, 208);
  tft.print("FW: ");
  tft.print(FW_VERSION);
}

// Portal mode transition handler (safe to call from wifiLoop)
static void setPortalMode(bool on) {
  if (portalMode == on) return;
  portalMode = on;

  if (on) {
    // Stop ECU serial completely to reduce interrupt/CPU load while serving web pages.
    // Note: if USE_UART0, this also stops USB serial debug (acceptable in portal mode).
    ECU_SERIAL.end();
    delay(20);
    // Stop SD recording so SD browsing/download doesn't fight SPI or file handle
#if USE_SD
    if (recording) stopRecording();
#endif
    setUiPaused(true);

    // Draw the portal screen once (no LVGL needed)
    drawPortalScreen();
  } else {
    // Restart ECU serial
#if USE_UART0
    ECU_SERIAL.begin(ECU_BAUD);
#else
    ECU_SERIAL.begin(ECU_BAUD, SERIAL_8N1, ECU_RX_PIN, ECU_TX_PIN);
#endif
    while (ECU_SERIAL.available()) ECU_SERIAL.read();
    rxState = WAIT_N;
    rxCount = 0;
    rxLen = 0;
    linkValid = false;
    lastRxMs = millis();

    setUiPaused(false);

    // Force LVGL to repaint on next dashLoop iteration
    if (lvReady && scr_dash) {
      lv_scr_load(scr_dash);
      lv_obj_invalidate(scr_dash);
    }
  }
}

// ============================================================================
// ============================= WIFI WEB PORTAL (LITE) ========================
// ============================================================================

#if USE_WIFI

// ---- tiny helpers ----
static bool endsWithCsv(const char* name) {
  if (!name) return false;
  size_t n = strlen(name);
  if (n < 4) return false;
  return (name[n-4] == '.' && (name[n-3] | 0x20) == 'c' && (name[n-2] | 0x20) == 's' && (name[n-1] | 0x20) == 'v');
}

static void sendHtmlHeaderLite(const char* title) {
  server.sendContent(F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"));
  server.sendContent(F("<title>"));
  server.sendContent(title);
  server.sendContent(F("</title>"));
  // Minimal CSS (small + fast)
  server.sendContent(F("<style>"
                       "body{font-family:Arial;margin:12px;background:#111;color:#eee;max-width:820px}"
                       "a{color:#7af}small{color:#aaa}"
                       ".card{border:1px solid #333;border-radius:12px;padding:12px;margin:12px 0;background:#1b1b1b}"
                       "label{display:block;margin:8px 0 4px}"
                       "input,select,button{width:100%;padding:10px;border-radius:10px;border:1px solid #333;background:#222;color:#eee;font-size:16px}"
                       "button{cursor:pointer}"
                       ".row{display:flex;gap:10px;flex-wrap:wrap}"
                       ".row>*{flex:1;min-width:160px}"
                       "table{width:100%;border-collapse:collapse}"
                       "td,th{padding:6px;border-bottom:1px solid #333;text-align:left}"
                       ".ok{color:#3f3}.bad{color:#f66}"
                       "</style></head><body>"));
  server.sendContent(F("<header><h2>ESP Dash <small>"));
  server.sendContent(FW_VERSION);
  server.sendContent(F("</small></h2></header>"));
}

static void sendHtmlFooterLite() {
  server.sendContent(F("</body></html>"));
}

// Root: live + GENERAL settings only (small + stable)
static void handleRoot() {
#if USE_SD
  portalBusy = true;
  if (recording && logFile) logFile.flush();
#endif

  const bool saved = server.hasArg("saved") && server.arg("saved") == "1";

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", "");
  sendHtmlHeaderLite("ESP Dash");

  if (saved) {
    server.sendContent(F("<div class='card ok'><b>Saved!</b> Settings written to ESP32.</div>"));
  }
  // Config-only portal: no live data rendered to reduce CPU/heap.

  server.sendContent(F("<div class='card'>"
                       "<b>Portal</b><br>"
                       "<a href='/'>Home</a> &nbsp;|&nbsp; "
                       "<a href='/warn'>Warnings</a> &nbsp;|&nbsp; "
                       "<a href='/logs'>SD Logs</a> &nbsp;|&nbsp; "
                       "<a href='/reboot' onclick=\"return confirm('Reboot ESP32?')\">Reboot</a>"
                       "</div>"));

  // Logs (no directory listing: stable + low RAM)
  server.sendContent(F("<div class='card'><h3>Logs</h3>"
                       "<p>Download a log by number (matches <code>log_00001.csv</code>).</p>"
                       "<form method='GET' action='/download'>"
                       "<div class='row'>"
                       "<div><label>Log #</label><input name='i' type='number' min='1' step='1' value='1'></div>"
                       "<div style='align-self:end'><button type='submit'>Download</button></div>"
                       "</div></form>"
                       "<p><a href='/downloadLatest'>Download latest log</a></p>"
                       "</div>"));

  // General settings form (small)
  server.sendContent(F("<div class='card'><form method='POST' action='/save'>"
                       "<h3>General</h3><div class='row'>"));

  server.sendContent(F("<div><label>View</label><select name='view'>"));
  server.sendContent(setting_viewMode==0 ? F("<option value='0' selected>Ring</option><option value='1'>Bar</option>")
                                         : F("<option value='0'>Ring</option><option value='1' selected>Bar</option>"));
  server.sendContent(F("</select></div>"));

  server.sendContent(F("<div><label>Logging</label><select name='logEn'>"));
  server.sendContent(!setting_logEnabled ? F("<option value='0' selected>Off</option><option value='1'>On</option>")
                                         : F("<option value='0'>Off</option><option value='1' selected>On</option>"));
  server.sendContent(F("</select></div>"));

  server.sendContent(F("<div><label>Shift Enable</label><select name='shiftEn'>"));
  server.sendContent(!setting_shiftEnabled ? F("<option value='0' selected>Off</option><option value='1'>On</option>")
                                           : F("<option value='0'>Off</option><option value='1' selected>On</option>"));
  server.sendContent(F("</select></div>"));

  char rpmBuf[96];
  snprintf(rpmBuf, sizeof(rpmBuf),
           "<div><label>Shift RPM</label><input name='shiftRpm' type='number' min='0' max='%d' value='%d'></div>",
           RPM_MAX, setting_shiftRpm);
  server.sendContent(rpmBuf);

  server.sendContent(F("</div><p><button type='submit'>Save General</button></p></form></div>"));

  server.sendContent(F("<div class='card'><b>Tip</b><br>"
                       "When a device is connected, the dashboard + serial polling are paused for maximum portal stability."
                       "</div>"));

  sendHtmlFooterLite();
  server.sendContent(""); // end chunked response

#if USE_SD
  portalBusy = false;
#endif
}

// Warnings page: separate to reduce load on /
static void handleWarn() {
#if USE_SD
  portalBusy = true;
  if (recording && logFile) logFile.flush();
#endif

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", "");
  sendHtmlHeaderLite("Warnings");

  server.sendContent(F("<div class='card'>"
                       "<a href='/'>Home</a> &nbsp;|&nbsp; "
                       "<a href='/warn'>Warnings</a> &nbsp;|&nbsp; "
                       "<a href='/logs'>SD Logs</a>"
                       "</div>"));

  // Use /save handler; include required general fields as hidden so /save stays simple.
  char hid[160];
  snprintf(hid, sizeof(hid),
           "<form method='POST' action='/save'>"
           "<input type='hidden' name='view' value='%d'>"
           "<input type='hidden' name='logEn' value='%d'>"
           "<input type='hidden' name='shiftEn' value='%d'>"
           "<input type='hidden' name='shiftRpm' value='%d'>",
           (int)setting_viewMode, (int)(setting_logEnabled?1:0), (int)(setting_shiftEnabled?1:0), (int)setting_shiftRpm);
  server.sendContent(F("<div class='card'><h3>Warnings</h3>"));
  server.sendContent(hid);

  server.sendContent(F("<table><tr><th>Item</th><th>Enable</th><th>Min</th><th>Max</th></tr>"));

  for (int i=0;i<W_COUNT;i++) {
    const char* name = warnName(i);

    // Pre-format min/max as strings (small)
    char minStr[16], maxStr[16];
    if (i==W_AFR || i==W_VBAT) {
      snprintf(minStr, sizeof(minStr), "%.1f", warnCfg[i].minV);
      snprintf(maxStr, sizeof(maxStr), "%.1f", warnCfg[i].maxV);
    } else {
      snprintf(minStr, sizeof(minStr), "%.0f", warnCfg[i].minV);
      snprintf(maxStr, sizeof(maxStr), "%.0f", warnCfg[i].maxV);
    }

    char row[320];
    snprintf(row, sizeof(row),
             "<tr><td>%s</td>"
             "<td><select name='w%de'>"
               "<option value='0'%s>Off</option>"
               "<option value='1'%s>On</option>"
             "</select></td>"
             "<td><input name='w%dmin' type='number' step='0.1' value='%s'></td>"
             "<td><input name='w%dmax' type='number' step='0.1' value='%s'></td>"
             "</tr>",
             name, i,
             (!warnCfg[i].enabled ? " selected" : ""),
             ( warnCfg[i].enabled ? " selected" : ""),
             i, minStr,
             i, maxStr);
    server.sendContent(row);
    yield();
  }

  server.sendContent(F("</table><p><button type='submit'>Save Warnings</button></p></form></div>"));

  sendHtmlFooterLite();
  server.sendContent("");

#if USE_SD
  portalBusy = false;
#endif
}

// Logs page: separate, avoid Strings while listing
static void handleLogs() {
#if USE_SD
  portalBusy = true;
  if (recording && logFile) logFile.flush();
#endif

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", "");
  sendHtmlHeaderLite("SD Logs");

  server.sendContent(F("<div class='card'>"
                       "<a href='/'>Home</a> &nbsp;|&nbsp; "
                       "<a href='/warn'>Warnings</a> &nbsp;|&nbsp; "
                       "<a href='/logs'>SD Logs</a>"
                       "</div>"));

  server.sendContent(F("<div class='card'><h3>SD Logs</h3>"));

#if USE_SD
  if (!sdOk) {
    server.sendContent(F("<p><b class='bad'>SD not detected.</b></p>"));
  } else if (recording) {
    server.sendContent(F("<p class='bad'><b>Recording is ON</b>. Stop REC to browse/download logs.</p>"));
    server.sendContent(F("<p><a href='/rec'>Toggle REC</a></p>"));
  } else {
    File root = SD.open("/");
    if (!root) {
      server.sendContent(F("<p class='bad'>Unable to open SD root.</p>"));
    } else {
      server.sendContent(F("<ul>"));
      File f = root.openNextFile();
      while (f) {
        const char* n = f.name();
        if (!f.isDirectory() && endsWithCsv(n)) {
          char li[260];
          // n may already have leading '/', but SD library varies; ensure URL uses raw name
          const char* nUrl = (n && n[0] == '/') ? (n + 1) : n;
          snprintf(li, sizeof(li),
                   "<li><a href='/download?f=%s'>%s</a> (%lu bytes)</li>",
                   nUrl ? nUrl : "", nUrl ? nUrl : "", (unsigned long)f.size());
          server.sendContent(li);
        }
        f.close();
        f = root.openNextFile();
        yield();
      }
      root.close();
      server.sendContent(F("</ul>"));
    }
    server.sendContent(F("<p><a href='/rec'>Toggle REC</a></p>"));
  }
#else
  server.sendContent(F("<p>SD support disabled in build.</p>"));
#endif

  server.sendContent(F("</div>"));

  sendHtmlFooterLite();
  server.sendContent("");

#if USE_SD
  portalBusy = false;
#endif
}

// Save handler (unchanged behavior, but used by both / and /warn)
static void handleSave() {
#if USE_SD
  portalBusy = true;
#endif

  if (!server.hasArg("view") || !server.hasArg("logEn") || !server.hasArg("shiftEn") || !server.hasArg("shiftRpm")) {
#if USE_SD
    portalBusy = false;
#endif
    server.send(400, "text/plain", "Missing required fields");
    return;
  }

  setting_viewMode = (uint8_t)server.arg("view").toInt();
  setting_logEnabled = server.arg("logEn").toInt() == 1;
  setting_shiftEnabled = server.arg("shiftEn").toInt() == 1;
  setting_shiftRpm = clampi(server.arg("shiftRpm").toInt(), 0, RPM_MAX);

  // Warnings are optional in this POST (root page doesn't send them)
  for (int i=0;i<W_COUNT;i++) {
    String ke = "w" + String(i) + "e";
    String kmin = "w" + String(i) + "min";
    String kmax = "w" + String(i) + "max";
    if (!server.hasArg(ke) || !server.hasArg(kmin) || !server.hasArg(kmax)) continue;

    warnCfg[i].enabled = server.arg(ke).toInt() == 1;
    warnCfg[i].minV = server.arg(kmin).toFloat();
    warnCfg[i].maxV = server.arg(kmax).toFloat();
    if (warnCfg[i].minV > warnCfg[i].maxV) {
      float mid = 0.5f * (warnCfg[i].minV + warnCfg[i].maxV);
      warnCfg[i].minV = warnCfg[i].maxV = mid;
    }
    yield();
  }

  saveSettings();
  apply_view_layout();
  refresh_settings_list();
  flashSaved();

#if USE_SD
  portalBusy = false;
#endif
  server.sendHeader("Location", "/?saved=1");
  server.send(303);
}


static void handleDownloadLatest() {
#if USE_SD
  portalBusy = true;

  if(!sdOk){ portalBusy = false; server.send(500, "text/plain", "SD not ready"); return; }
  if (recording) { portalBusy = false; server.send(409, "text/plain", "Stop REC before download"); return; }

  // Latest completed log is (setting_logIndex - 1) because we increment index on startRecording()
  uint32_t idx = (setting_logIndex > 0) ? (setting_logIndex - 1) : 0;
  if (idx == 0) { portalBusy = false; server.send(404, "text/plain", "No logs yet"); return; }

  char fn[32];
  snprintf(fn, sizeof(fn), "/log_%05lu.csv", (unsigned long)idx);

  if(!SD.exists(fn)){ portalBusy = false; server.send(404, "text/plain", "Not found"); return; }

  File f = SD.open(fn, FILE_READ);
  if(!f){ portalBusy = false; server.send(404, "text/plain", "Not found"); return; }

  server.sendHeader("Content-Disposition", String("attachment; filename=\"") + String(fn+1) + "\"");
  server.streamFile(f, "text/csv");
  f.close();
  portalBusy = false;
#else
  server.send(500, "text/plain", "SD disabled");
#endif
}

static void handleDownload() {
#if USE_SD
  portalBusy = true;

  if(!sdOk){ portalBusy = false; server.send(500, "text/plain", "SD not ready"); return; }
  if (recording) { portalBusy = false; server.send(409, "text/plain", "Stop REC before download"); return; }

  String fn;
  if (server.hasArg("i")) {
    long idx = server.arg("i").toInt();
    if (idx < 1) { portalBusy = false; server.send(400, "text/plain", "Bad i"); return; }
    char b[32];
    snprintf(b, sizeof(b), "/log_%05ld.csv", idx);
    fn = String(b);
  } else {
    fn = server.arg("f");
    if(fn.length()==0){ portalBusy = false; server.send(400, "text/plain", "Missing f"); return; }
    if(!fn.startsWith("/")) fn = "/"+fn;
  }
  if(!fn.startsWith("/")) fn = "/"+fn;
  if(fn.indexOf("..") >= 0){ portalBusy = false; server.send(400, "text/plain", "Bad path"); return; }
  if(!SD.exists(fn.c_str())){ portalBusy = false; server.send(404, "text/plain", "Not found"); return; }

  File f = SD.open(fn.c_str(), FILE_READ);
  if(!f){ portalBusy = false; server.send(404, "text/plain", "Not found"); return; }

  server.sendHeader("Content-Disposition", "attachment; filename=\""+fn.substring(1)+"\"");
  server.streamFile(f, "text/csv");
  f.close();
  portalBusy = false;
#else
  server.send(500, "text/plain", "SD disabled");
#endif
}

static void handleRec() {
#if USE_SD
  portalBusy = true;
  if(!sdOk){ portalBusy = false; server.send(500, "text/plain", "SD not ready"); return; }

  if (!recording) (void)startRecording();
  else stopRecording();

  portalBusy = false;
  server.sendHeader("Location","/logs");
  server.send(303);
#else
  server.send(500, "text/plain", "SD disabled");
#endif
}

static void handleReboot() {
  server.send(200, "text/plain", "Rebooting...");
  delay(200);
  ESP.restart();
}

static void wifiSetupInternal() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CH, WIFI_AP_HIDDEN, WIFI_AP_MAX_CONN);
  delay(100);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/warn", HTTP_GET, handleWarn);
  server.on("/logs", HTTP_GET, handleLogs);

  server.on("/save", HTTP_POST, handleSave);
  server.on("/download", HTTP_GET, handleDownload);
  server.on("/downloadLatest", HTTP_GET, handleDownloadLatest);
  server.on("/rec", HTTP_GET, handleRec);
  server.on("/reboot", HTTP_GET, handleReboot);

  server.onNotFound([](){ server.send(404, "text/plain", "Not found"); });
  server.begin();
}

#endif // USE_WIFI

// ============================================================================
// ============================= PUBLIC API ===================================
// ============================================================================

void wifiSetup(){
#if USE_WIFI
  wifiSetupInternal();
#endif
}

void wifiLoop(){
#if USE_WIFI
  server.handleClient();

#if PAUSE_UI_WHEN_WIFI_CLIENT
  const uint32_t now = millis();
  if (now - lastWifiClientCheckMs >= WIFI_CLIENT_CHECK_MS) {
    lastWifiClientCheckMs = now;
    int n = WiFi.softAPgetStationNum();

    // Only touch LVGL (setUiPaused) after dashSetup has completed LVGL init + UI build
    if (lvReady) setPortalMode(n > 0);
  }
#endif

#endif
}

// ============================= Public API: dashSetup/dashLoop =============================
void dashSetup() {
  Serial.begin(115200);
  delay(100);
  Serial.print("Speeduino Dashboard LVGL ");
  Serial.println(FW_VERSION);

  loadSettings();

  tft.begin();
  tft.setRotation(1);

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
#endif

#if USE_TOUCH
  tft.setTouch(touchCalData);
#endif

#if USE_SD
  sdSpi.begin(SD_VSPI_SCK, SD_VSPI_MISO, SD_VSPI_MOSI, SD_VSPI_SS);
  sdOk = SD.begin(SD_VSPI_SS, sdSpi);
#endif

  showSplashThenStartSerial();

  lv_init();

  const uint32_t buf_pixels = SCREEN_W * 40; // 40 lines
  buf1 = (lv_color_t*)heap_caps_malloc(buf_pixels * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  buf2 = (lv_color_t*)heap_caps_malloc(buf_pixels * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!buf1 || !buf2) {
    if (buf1) free(buf1);
    if (buf2) free(buf2);
    buf1 = (lv_color_t*)malloc(buf_pixels * sizeof(lv_color_t));
    buf2 = (lv_color_t*)malloc(buf_pixels * sizeof(lv_color_t));
  }

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buf_pixels);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_W;
  disp_drv.ver_res = SCREEN_H;
  disp_drv.flush_cb = my_flush_cb;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

#if USE_TOUCH
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touch_read;
  lv_indev_drv_register(&indev_drv);
#endif

  build_dash();
  build_settings();
  refresh_settings_list();
  lv_scr_load(scr_dash);

  lastTick = millis();
  lastPoll = millis();

  // IMPORTANT: mark LVGL/UI ready only after everything is built and screen is loaded
  lvReady = true;
}

void dashLoop() {
  // If a WiFi station is connected we are in portalMode.
  // For maximum web stability we pause ALL ECU serial reads/polling and LVGL.
  if (portalMode) { delay(2); return; }

  while (ECU_SERIAL.available()) onRxByte((uint8_t)ECU_SERIAL.read());

  if (!portalMode && (millis() - lastPoll >= POLL_MS)) {
    pollSpeeduino();
    lastPoll = millis();
  }

  // If a WiFi station is connected, we are in portalMode:
// - LVGL/UI is stopped completely
// - TFT shows a static "WEB CONFIGURATION MODE" screen
if (portalMode) {
  // Portal mode: prioritize web server + WiFi stack. Avoid extra work.
  delay(2);
#if USE_SD
  // Still allow logging if user left it on (but we stop recording on entry to portal mode for stability)
#endif
  return;
}

// Normal mode: LVGL + dashboard updates
lvglTick();
lv_timer_handler();

// dashboard value updates
static uint32_t lastUi = 0;
if (!uiPaused && (millis() - lastUi > UI_UPDATE_MS)) {
  if (lv_scr_act() == scr_dash || lv_scr_act() == scr_shift) update_dash_values();
  lastUi = millis();
}

#if USE_SD
logIfRecording();
#endif

if (lbl_saved && savedUntilMs != 0) {
  if (millis() > savedUntilMs) {
    lv_obj_add_flag(lbl_saved, LV_OBJ_FLAG_HIDDEN);
    savedUntilMs = 0;
  }
}
}



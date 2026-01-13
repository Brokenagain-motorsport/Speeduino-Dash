#ifndef LV_CONF_H
#define LV_CONF_H

/*====================
 * General settings
 *====================*/
#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0

/* Use Arduino millis() for tick */
#define LV_TICK_CUSTOM 0
#define LV_TICK_CUSTOM_INCLUDE "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())

/*====================
 * Memory settings
 *====================*/
#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (64U * 1024U)

/*====================
 * Feature configuration
 *====================*/
#define LV_USE_LOG 0

/*====================
 * Fonts
 *====================*/
 
#define LV_FONT_MONTSERRAT_10 1
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_22 1
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_32 1
#define LV_FONT_MONTSERRAT_40 1
#define LV_FONT_MONTSERRAT_48 1

#define LV_FONT_DEFAULT &lv_font_montserrat_16

/*====================
 * Draw engine
 *====================*/
#define LV_USE_DRAW_SW 1
#define LV_DRAW_SW_COMPLEX 1

/*====================
 * Core widgets needed by the dash
 *====================*/
#define LV_USE_ARC 1
#define LV_USE_BAR 1
#define LV_USE_LABEL 1
#define LV_USE_LINE 1

/*====================
 * Extra widgets (TURN OFF!)
 *====================*/
#define LV_USE_EXTRA 0

/*====================
 * Themes
 *====================*/
#define LV_USE_THEME_DEFAULT 1
#define LV_THEME_DEFAULT_DARK 1
#define LV_THEME_DEFAULT_GROW 1
#define LV_THEME_DEFAULT_TRANSITION_TIME 80

/*====================
 * Misc
 *====================*/
#define LV_USE_USER_DATA 1
#define LV_USE_ASSERT_NULL 1
#define LV_USE_ASSERT_MALLOC 1

#endif /*LV_CONF_H*/

/*
 * OLEDHelper.cpp
 * Copyright (C) 2019-2022 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../system/SoC.h"

#if defined(USE_OLED)

#include <Wire.h>

#include "OLED.h"

#include "RF.h"
#include "LED.h"
#include "GNSS.h"
#include "Baro.h"
#include "Battery.h"
#include "../TrafficHelper.h"
#include "../system/Time.h"

enum
{
  OLED_PAGE_RADIO,
  OLED_PAGE_OTHER,
#if !defined(EXCLUDE_OLED_BARO_PAGE)
  OLED_PAGE_BARO,
#endif /* EXCLUDE_OLED_BARO_PAGE */
#if !defined(EXCLUDE_OLED_TRAFFIC_PAGE)
  OLED_PAGE_TRAFFIC,
#endif /* EXCLUDE_OLED_TRAFFIC_PAGE */
  OLED_PAGE_COUNT
};

#if !defined(EXCLUDE_OLED_049)
enum
{
  OLED_049_PAGE_ID,
  OLED_049_PAGE_PROTOCOL,
  OLED_049_PAGE_RX,
  OLED_049_PAGE_SATS_TX,
  OLED_049_PAGE_ACFTS,
  OLED_049_PAGE_UPTIME,
  OLED_049_PAGE_VOLTAGE,
  OLED_049_PAGE_COUNT
};
#endif /* EXCLUDE_OLED_049 */

U8X8_OLED_I2C_BUS_TYPE u8x8_i2c(U8X8_PIN_NONE);

U8X8_OLED_I2C_BUS_TYPE *u8x8 = NULL;

static bool OLED_display_titles = false;
static uint32_t prev_tx_packets_counter = (uint32_t) -1;
static uint32_t prev_rx_packets_counter = (uint32_t) -1;
extern uint32_t tx_packets_counter, rx_packets_counter;

static uint32_t prev_acrfts_counter = (uint32_t) -1;
static uint32_t prev_sats_counter   = (uint32_t) -1;
static uint32_t prev_uptime_minutes = (uint32_t) -1;
static int32_t  prev_voltage        = (uint32_t) -1;
static int8_t   prev_fix            = (uint8_t)  -1;

#if !defined(EXCLUDE_OLED_BARO_PAGE)
static int32_t  prev_altitude       = (int32_t)   -10000;
static int32_t  prev_temperature    = (int32_t)   -100;
static uint32_t prev_pressure       = (uint32_t)  -1;
static int32_t  prev_cdr            = (int32_t)   -10000; /* climb/descent rate */
#endif /* EXCLUDE_OLED_BARO_PAGE */

unsigned long OLEDTimeMarker = 0;

const char *ISO3166_CC[] = {
  [RF_BAND_AUTO] = "--",
  [RF_BAND_EU]   = "EU",
  [RF_BAND_US]   = "US",
  [RF_BAND_AU]   = "AU",
  [RF_BAND_NZ]   = "NZ",
  [RF_BAND_RU]   = "RU",
  [RF_BAND_CN]   = "CN",
  [RF_BAND_UK]   = "UK",
  [RF_BAND_IN]   = "IN",
  [RF_BAND_IL]   = "IL",
  [RF_BAND_KR]   = "KR"
};

const char SoftRF_text1[]  = "SoftRF";
const char SoftRF_text2[]  = "and";
const char SoftRF_text3[]  = "LilyGO";
const char ID_text[]       = "ID";
const char PROTOCOL_text[] = "PROTOCOL";
const char RX_text[]       = "RX";
const char TX_text[]       = "TX";
const char ACFTS_text[]    = "ACFTS";
const char SATS_text[]     = "SATS";
const char FIX_text[]      = "FIX";
const char UPTIME_text[]   = "UPTIME";
const char BAT_text[]      = "BAT";

#if !defined(EXCLUDE_OLED_BARO_PAGE)
const char ALT_text[]      = "ALT M";
const char TEMP_text[]     = "TEMP C";
const char PRES_text[]     = "PRES MB";
const char CDR_text[]      = "CDR FPM";
#endif /* EXCLUDE_OLED_BARO_PAGE */

static const uint8_t Dot_Tile[] = { 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00 };

static uint8_t OLED_current_page = OLED_PAGE_RADIO;
static uint8_t page_count        = OLED_PAGE_COUNT;

byte OLED_setup() {

  byte rval = DISPLAY_NONE;
  bool oled_probe = false;

#if defined(plat_oled_probe_func)
  oled_probe = plat_oled_probe_func();
#else
  /* SSD1306 I2C OLED probing */
  Wire.begin();
  Wire.beginTransmission(SSD1306_OLED_I2C_ADDR);
  oled_probe = (Wire.endTransmission() == 0);
#endif /* plat_oled_probe_func */
  if (oled_probe)
  {
    u8x8 = &u8x8_i2c;
    rval = (hw_info.model == SOFTRF_MODEL_MINI     ? DISPLAY_OLED_HELTEC :
            hw_info.model == SOFTRF_MODEL_BRACELET ? DISPLAY_OLED_0_49   :
            DISPLAY_OLED_TTGO);
  }

  if (u8x8) {
    u8x8->begin();
    u8x8->setFont(u8x8_font_chroma48medium8_r);

    switch (rval)
    {
#if !defined(EXCLUDE_OLED_049)
    case DISPLAY_OLED_0_49:

      u8x8->setContrast(255);

      u8x8->draw2x2Glyph(4,  4, SoftRF_text3[0]);
      u8x8->draw2x2Glyph(6,  4, SoftRF_text3[1]);
      u8x8->draw2x2Glyph(8,  4, SoftRF_text3[2]);
      u8x8->draw2x2Glyph(10, 4, SoftRF_text3[3]);
      u8x8->draw2x2Glyph(6,  6, SoftRF_text3[4]);
      u8x8->draw2x2Glyph(8,  6, SoftRF_text3[5]);

      delay(2000);

      u8x8->clear();
      u8x8->draw2x2String( 5, 5, SoftRF_text2);

      delay(2000);

      u8x8->clear();
      u8x8->draw2x2Glyph(4,  4, SoftRF_text1[0]);
      u8x8->draw2x2Glyph(6,  4, SoftRF_text1[1]);
      u8x8->draw2x2Glyph(8,  4, SoftRF_text1[2]);
      u8x8->draw2x2Glyph(10, 4, SoftRF_text1[3]);
      u8x8->draw2x2Glyph(6,  6, SoftRF_text1[4]);
      u8x8->draw2x2Glyph(8,  6, SoftRF_text1[5]);

      OLED_current_page = OLED_049_PAGE_SATS_TX;
      page_count        = OLED_049_PAGE_COUNT;
      break;
#endif /* EXCLUDE_OLED_049 */
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    default:
      uint8_t shift_y = (hw_info.model == SOFTRF_MODEL_DONGLE ? 1 : 0);

      u8x8->draw2x2String( 2, 2 - shift_y, SoftRF_text1);

      if (hw_info.model == SOFTRF_MODEL_DONGLE) {
        u8x8->drawString   ( 6, 3, SoftRF_text2);
        u8x8->draw2x2String( 2, 4, SoftRF_text3);
      }

      u8x8->drawString   ( 3, 6 + shift_y, SOFTRF_FIRMWARE_VERSION);
      u8x8->drawString   (11, 6 + shift_y, ISO3166_CC[settings->band]);

      break;
    }
  }

  OLEDTimeMarker = millis();

  return rval;
}

static void OLED_radio()
{
  char buf[16];
  uint32_t disp_value;

  if (!OLED_display_titles) {

    u8x8->clear();

    u8x8->drawString(1, 1, ID_text);

    snprintf (buf, sizeof(buf), "%06X", ThisAircraft.addr);
    u8x8->draw2x2String(0, 2, buf);

    u8x8->drawString(8, 1, PROTOCOL_text);

    u8x8->draw2x2Glyph(14, 2, Protocol_ID[ThisAircraft.protocol][0]);

    u8x8->drawString(1, 5, RX_text);

    u8x8->drawString(9, 5, TX_text);

    if (settings->power_save & POWER_SAVE_NORECEIVE &&
        (hw_info.rf == RF_IC_SX1276 || hw_info.rf == RF_IC_SX1262)) {
      u8x8->draw2x2String(0, 6, "OFF");
      prev_rx_packets_counter = rx_packets_counter;
    } else {
      prev_rx_packets_counter = (uint32_t) -1;
    }

    if (settings->mode        == SOFTRF_MODE_RECEIVER ||
        settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ||
        settings->txpower     == RF_TX_POWER_OFF) {
      u8x8->draw2x2String(8, 6, "OFF");
      prev_tx_packets_counter = tx_packets_counter;
    } else {
      prev_tx_packets_counter = (uint32_t) -1;
    }

    OLED_display_titles = true;
  }

  if (rx_packets_counter != prev_rx_packets_counter) {
    disp_value = rx_packets_counter % 1000;
    itoa(disp_value, buf, 10);

    if (disp_value < 10) {
      strcat_P(buf,PSTR("  "));
    } else {
      if (disp_value < 100) {
        strcat_P(buf,PSTR(" "));
      };
    }

    u8x8->draw2x2String(0, 6, buf);
    prev_rx_packets_counter = rx_packets_counter;
  }
  if (tx_packets_counter != prev_tx_packets_counter) {
    disp_value = tx_packets_counter % 1000;
    itoa(disp_value, buf, 10);

    if (disp_value < 10) {
      strcat_P(buf,PSTR("  "));
    } else {
      if (disp_value < 100) {
        strcat_P(buf,PSTR(" "));
      };
    }

    u8x8->draw2x2String(8, 6, buf);
    prev_tx_packets_counter = tx_packets_counter;
  }
}

static void OLED_other()
{
  char buf[16];
  uint32_t disp_value;

  if (!OLED_display_titles) {

    u8x8->clear();

    u8x8->drawString( 1, 1, ACFTS_text);

    u8x8->drawString( 7, 1, SATS_text);

    u8x8->drawString(12, 1, FIX_text);

    u8x8->drawString( 1, 5, UPTIME_text);

    u8x8->drawString(12, 5, BAT_text);

    u8x8->drawTile  (4, 6, 1, (uint8_t *) Dot_Tile);
    u8x8->drawTile  (4, 7, 1, (uint8_t *) Dot_Tile);

    u8x8->drawGlyph (13, 7, '.');

    prev_acrfts_counter = (uint32_t) -1;
    prev_sats_counter   = (uint32_t) -1;
    prev_fix            = (uint8_t)  -1;
    prev_uptime_minutes = (uint32_t) -1;
    prev_voltage        = (uint32_t) -1;

    OLED_display_titles = true;
  }

  uint32_t acrfts_counter = Traffic_Count();
  uint32_t sats_counter   = gnss.satellites.value();
  uint8_t  fix            = (uint8_t) isValidGNSSFix();
  uint32_t uptime_minutes = UpTime.minutes;
  int32_t  voltage        = Battery_voltage() > BATTERY_THRESHOLD_INVALID ?
                              (int) (Battery_voltage() * 10.0) : 0;

  if (prev_acrfts_counter != acrfts_counter) {
    disp_value = acrfts_counter > 99 ? 99 : acrfts_counter;
    itoa(disp_value, buf, 10);

    if (disp_value < 10) {
      strcat_P(buf,PSTR(" "));
    }

    u8x8->draw2x2String(1, 2, buf);
    prev_acrfts_counter = acrfts_counter;
  }

  if (prev_sats_counter != sats_counter) {
    disp_value = sats_counter > 99 ? 99 : sats_counter;
    itoa(disp_value, buf, 10);

    if (disp_value < 10) {
      strcat_P(buf,PSTR(" "));
    }

    u8x8->draw2x2String(7, 2, buf);
    prev_sats_counter = sats_counter;
  }

  if (prev_fix != fix) {
    u8x8->draw2x2Glyph(12, 2, fix > 0 ? '+' : '-');
//  u8x8->draw2x2Glyph(12, 2, '0' + fix);
    prev_fix = fix;
  }

  if (prev_uptime_minutes != uptime_minutes) {
    disp_value = UpTime.hours; /* 0-23 */
    if (disp_value < 10) {
      buf[0] = '0';
      itoa(disp_value, buf+1, 10);
    } else {
      itoa(disp_value, buf, 10);
    }

    u8x8->draw2x2String(0, 6, buf);

    disp_value = uptime_minutes;
    if (disp_value < 10) {
      buf[0] = '0';
      itoa(disp_value, buf+1, 10);
    } else {
      itoa(disp_value, buf, 10);
    }

    u8x8->draw2x2String(5, 6, buf);

    prev_uptime_minutes = uptime_minutes;
  }

  if (prev_voltage != voltage) {
    if (voltage) {
      disp_value = voltage / 10;
      disp_value = disp_value > 9 ? 9 : disp_value;
      u8x8->draw2x2Glyph(11, 6, '0' + disp_value);

      disp_value = voltage % 10;

      u8x8->draw2x2Glyph(14, 6, '0' + disp_value);
    } else {
      u8x8->draw2x2Glyph(11, 6, 'N');
      u8x8->draw2x2Glyph(14, 6, 'A');
    }
    prev_voltage = voltage;
  }
}

#if !defined(EXCLUDE_OLED_BARO_PAGE)
static void OLED_baro()
{
  char buf[16];

  if (!OLED_display_titles) {

    u8x8->clear();

    u8x8->drawString( 2, 1, ALT_text);

    u8x8->drawString( 10, 1, TEMP_text);

    u8x8->drawString( 1, 5, PRES_text);

    u8x8->drawString( 9, 5, CDR_text);

    prev_altitude     = (int32_t)   -10000;
    prev_temperature  = prev_altitude;
    prev_pressure     = (uint32_t)  -1;
    prev_cdr          = prev_altitude;

    OLED_display_titles = true;
  }

  int32_t altitude    = Baro_altitude();        /* metres */
  int32_t temperature = Baro_temperature();     /* Celcius */
  uint32_t pressure   = Baro_pressure() / 100;  /* mbar */
  int32_t cdr         = ThisAircraft.vs;        /* feet per minute */

  if (prev_altitude != altitude) {
    snprintf(buf, sizeof(buf), "%4d", altitude);
    u8x8->draw2x2String(0, 2, buf);
    prev_altitude = altitude;
  }

  if (prev_temperature != temperature) {
    snprintf(buf, sizeof(buf), "%3d", temperature);
    u8x8->draw2x2String(10, 2, buf);
    prev_temperature = temperature;
  }

  if (prev_pressure != pressure) {
    snprintf(buf, sizeof(buf), "%4d", pressure);
    u8x8->draw2x2String(0, 6, buf);
    prev_pressure = pressure;
  }

  if (prev_cdr != cdr) {
    int disp_value = constrain(cdr, -999, 999);
    snprintf(buf, sizeof(buf), "%3d", abs(disp_value));
    u8x8->drawGlyph    ( 9, 6, disp_value < 0 ? '_' : ' ');
    u8x8->draw2x2String(10, 6, buf);
    prev_cdr = cdr;
  }
}
#endif /* EXCLUDE_OLED_BARO_PAGE */

#if !defined(EXCLUDE_OLED_049)

void OLED_049_func()
{
  char buf[16];
  uint32_t disp_value;
  uint32_t acrfts_counter;
  uint32_t sats_counter;
  uint32_t uptime_minutes;
  int32_t  voltage;

  switch (OLED_current_page)
  {
  case OLED_049_PAGE_ID:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString(5, 4, ID_text);
      snprintf (buf, sizeof(buf), "%06X", ThisAircraft.addr);
      u8x8->draw2x2Glyph ( 8, 4, buf[0]);
      u8x8->draw2x2Glyph (10, 4, buf[1]);
      u8x8->draw2x2String( 4, 6, buf+2);

      OLED_display_titles = true;
    }

    break;

  case OLED_049_PAGE_PROTOCOL:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString(4, 4, PROTOCOL_text);
      u8x8->draw2x2String(5, 6, Protocol_ID[ThisAircraft.protocol]);

      OLED_display_titles = true;
    }

    break;

  case OLED_049_PAGE_RX:
    if (!OLED_display_titles) {
      u8x8->clear();

      u8x8->drawString(5, 4, RX_text);

      if (settings->power_save & POWER_SAVE_NORECEIVE &&
          (hw_info.rf == RF_IC_SX1276 || hw_info.rf == RF_IC_SX1262)) {
        u8x8->draw2x2String(5, 6, "OFF");
        prev_rx_packets_counter = rx_packets_counter;
      } else {
        prev_rx_packets_counter = (uint32_t) -1;
      }

      OLED_display_titles = true;
    }

    if (rx_packets_counter != prev_rx_packets_counter) {
      disp_value = rx_packets_counter % 1000;
      itoa(disp_value, buf, 10);

      if (disp_value < 10) {
        strcat_P(buf,PSTR("  "));
      } else {
        if (disp_value < 100) {
          strcat_P(buf,PSTR(" "));
        };
      }

      u8x8->draw2x2String(5, 6, buf);
      prev_rx_packets_counter = rx_packets_counter;
    }

    break;

  case OLED_049_PAGE_SATS_TX:
    if (!OLED_display_titles) {
      u8x8->clear();

      u8x8->drawString( 4, 4, SATS_text);
      prev_sats_counter   = (uint32_t) -1;

      u8x8->drawString(10, 4, TX_text);

      if (settings->mode        == SOFTRF_MODE_RECEIVER ||
          settings->rf_protocol == RF_PROTOCOL_ADSB_UAT ||
          settings->txpower     == RF_TX_POWER_OFF) {
        u8x8->draw2x2String(8, 6, "NA");
        prev_tx_packets_counter = tx_packets_counter;
      } else {
        prev_tx_packets_counter = (uint32_t) -1;
      }

      OLED_display_titles = true;
    }

    sats_counter   = gnss.satellites.value();

    if (prev_sats_counter != sats_counter) {
      disp_value = sats_counter > 9 ? 9 : sats_counter;

      u8x8->draw2x2Glyph(4, 6, '0' + disp_value);
      prev_sats_counter = sats_counter;
    }

    if (tx_packets_counter != prev_tx_packets_counter) {
      disp_value = tx_packets_counter % 100;
      itoa(disp_value, buf, 10);

      if (disp_value < 10) {
        strcat_P(buf,PSTR(" "));
      } else {
      }

      u8x8->draw2x2String(8, 6, buf);
      prev_tx_packets_counter = tx_packets_counter;
    }

    break;

  case OLED_049_PAGE_ACFTS:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString( 5, 4, ACFTS_text);
      prev_acrfts_counter = (uint32_t) -1;

      OLED_display_titles = true;
    }

    acrfts_counter = Traffic_Count();

    if (prev_acrfts_counter != acrfts_counter) {
      disp_value = acrfts_counter > 99 ? 99 : acrfts_counter;
      itoa(disp_value, buf, 10);

      if (disp_value < 10) {
        strcat_P(buf,PSTR(" "));
      }

      u8x8->draw2x2String(5, 6, buf);
      prev_acrfts_counter = acrfts_counter;
    }

    break;

  case OLED_049_PAGE_UPTIME:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString( 5, 4, UPTIME_text);
      u8x8->drawTile  (7, 6, 1, (uint8_t *) Dot_Tile);
      u8x8->drawTile  (7, 7, 1, (uint8_t *) Dot_Tile);
      prev_uptime_minutes = (uint32_t) -1;

      OLED_display_titles = true;
    }

    uptime_minutes = UpTime.minutes;

    if (prev_uptime_minutes != uptime_minutes) {
      disp_value = UpTime.hours % 10; /* 0-9, 0-9, 0-3 */
      itoa(disp_value, buf, 10);

      u8x8->draw2x2String(5, 6, buf);

      disp_value = uptime_minutes;
      if (disp_value < 10) {
        buf[0] = '0';
        itoa(disp_value, buf+1, 10);
      } else {
        itoa(disp_value, buf, 10);
      }

      u8x8->draw2x2String(8, 6, buf);

      prev_uptime_minutes = uptime_minutes;
    }

    break;

  case OLED_049_PAGE_VOLTAGE:
    if (!OLED_display_titles) {
      u8x8->clear();
      u8x8->drawString(5, 4, BAT_text);
      u8x8->drawGlyph (7, 7, '.');
      prev_voltage        = (uint32_t) -1;

      OLED_display_titles = true;
    }

    voltage = Battery_voltage() > BATTERY_THRESHOLD_INVALID ?
              (int) (Battery_voltage() * 10.0) : 0;

    if (prev_voltage != voltage) {
      if (voltage) {
        disp_value = voltage / 10;
        disp_value = disp_value > 9 ? 9 : disp_value;
        u8x8->draw2x2Glyph(5, 6, '0' + disp_value);

        disp_value = voltage % 10;

        u8x8->draw2x2Glyph(8, 6, '0' + disp_value);
      } else {
        u8x8->draw2x2Glyph(5, 6, 'N');
        u8x8->draw2x2Glyph(8, 6, 'A');
      }
      prev_voltage = voltage;
    }

    break;

  default:
    break;
  }
}

#endif /* EXCLUDE_OLED_049 */

#if !defined(EXCLUDE_OLED_TRAFFIC_PAGE)
static void OLED_traffic()
{
  const int SECTOR_ROTATE_ANGLE = 15;
  const int NUM_SECTORS = 12;
  const unsigned char WIDTH = 128;
  const unsigned char HEIGHT = 64;
  const unsigned char padding = 16;
  // angular indicators for 12 sectors
  static const unsigned char indicatorPositions[12][2]={
       {padding + 39, 6}, // 0 - 29
       {padding + 51, 13}, // 30-59
       {padding + 58,  25}, // 60 - 89
       {padding + 58, 39}, // 90 - 119
       {padding + 51,  51}, // 120 - 149
       {padding + 39,  58}, // 150 - 179
       {padding + 25,  58}, // 180 - 209
       {padding + 13,  51}, // 210 - 239
       {padding + 6,  39}, // 240 . 269
       {padding + 6, 25}, // 270 - 299
       {padding + 13, 13}, // 300 - 329
       {padding + 25,  6}}; // 330 - 359

  static const unsigned char verticalIndicatorPositionOffset = padding + 78;
  static const unsigned char verticalIndicatorPositions[4][2]={
      {verticalIndicatorPositionOffset,13}, // > 14 deg above (tan > 0.2493)
      {verticalIndicatorPositionOffset,26}, // >  7 deg (tan > 0.1228)
      {verticalIndicatorPositionOffset,39}, // >  7 deg
      {verticalIndicatorPositionOffset,52}  // > 14 deg below 
  };

  static const unsigned char nearestIndicatorSize = 7; // indicate nearest planes even if no alert is present
  static const unsigned char alertLevelIndicatorSize[4] = {3,7,9,11};
 
  unsigned char sectorAlertLevels[12] = {0};
  float sectorDistances[12] = {100000,100000,100000,100000,100000,100000,100000,100000,100000,100000,100000,100000};
  float sectorVerticalSeparations[12] = {100000,100000,100000,100000,100000,100000,100000,100000,100000,100000,100000,100000};
  unsigned char verticalAlertLevels[4] = {0};
  float verticalDistances[4] ={100000,100000,100000,100000}; // this is not the vertical separation but the 2D distance relevant for the vertical indicator
  float verticalIndicatorVerticalSeparations[4] ={100000,100000,100000,100000};

  /* simulate traffic */
  /*
  unsigned int tmp=0;
  for (int i=0;i<12;i++){
    tmp = random(1000);
    if (tmp < 800){
      sectorAlertLevels[i] = 0;
    }
    else if (tmp < 980){
      sectorAlertLevels[i] = 1;
    }
    else if (tmp<990){
      sectorAlertLevels[i] = 2;
    }
    else{
      sectorAlertLevels[i] = 3;
    }
    tmp = random(4);
    if (verticalAlertLevels[tmp] < sectorAlertLevels[i]) {
      verticalAlertLevels[tmp] = sectorAlertLevels[i];
    }
  }
  */

  #ifdef OLED_TRAFFIC_SIMULATE_TRAFFIC
  static unsigned int simulateTrafficCounter = 0;
  const int SIMULATOR_UPDATE_STEP = 8;
  const int distances[6] = {1400, 1200, 1000, 800, 0, 0};
  const int verticalSeparations[6] = {400, 0, -100, -400, -100, 100} ;
  #include <math.h>
  if (simulateTrafficCounter % SIMULATOR_UPDATE_STEP == 0){
    Container[0].addr = 1;
    Container[0].timestamp = now();
    Container[0].bearing = (simulateTrafficCounter / SIMULATOR_UPDATE_STEP * 30) % 360;
    Container[0].distance = distances[(simulateTrafficCounter / (SIMULATOR_UPDATE_STEP * NUM_SECTORS )) % 6]; //1400 - (simulateTrafficCounter / (SIMULATOR_UPDATE_STEP * NUM_SECTORS ) * 200) % 1400;
    //if (Container[0].distance < 0){
    //  Container[0].distance = 1400;
    //}
    Container[0].altitude = ThisAircraft.altitude + verticalSeparations[(simulateTrafficCounter / (SIMULATOR_UPDATE_STEP * NUM_SECTORS )) % 6]; //400 - (simulateTrafficCounter / (SIMULATOR_UPDATE_STEP * NUM_SECTORS ) * 250) % 900;
    Container[0].alarm_level = (simulateTrafficCounter / (SIMULATOR_UPDATE_STEP * NUM_SECTORS )) % 4;
    Serial.print("SIM: Aircraft at ");
    Serial.print(Container[0].bearing);
    Serial.print(" deg, "); 
    Serial.print(Container[0].distance); 
    Serial.print( " m, "); 
    Serial.print(Container[0].altitude-ThisAircraft.altitude);
    Serial.print(" m above, ");
    Serial.print(round(atan2(Container[0].altitude-ThisAircraft.altitude,Container[0].distance)*180/3.1415f));
    Serial.print(" deg (elev), alarm level: ");
    Serial.println(Container[0].alarm_level);
    if (simulateTrafficCounter / (SIMULATOR_UPDATE_STEP * NUM_SECTORS ) > 5){
      Container[1].addr = 2;
      Container[1].timestamp = now();
      Container[1].bearing = ((int)Container[0].bearing + 180) % 360;
      Container[1].distance = distances[0];
      Container[1].altitude = Container[0].altitude-ThisAircraft.altitude == 0 ? ThisAircraft.altitude - 100 :  ThisAircraft.altitude - (Container[0].altitude - ThisAircraft.altitude);
      Container[1].alarm_level = 0;
        Serial.print("     2nd aircraft at ");
        Serial.print(Container[1].bearing);
        Serial.print(" deg, "); 
        Serial.print(Container[1].distance); 
        Serial.print( " m, "); 
        Serial.print(Container[1].altitude-ThisAircraft.altitude);
        Serial.print(" m above, ");
        Serial.print(round(atan2(Container[1].altitude-ThisAircraft.altitude,Container[1].distance)*180/3.1415f));
        Serial.print(" deg (elev), alarm level: ");
        Serial.println(Container[0].alarm_level);
      Container[2].addr = 3;
      Container[2].timestamp = now();
      Container[2].bearing = Container[0].bearing;
      Container[2].distance = distances[0];
      Container[2].altitude = Container[0].altitude;
      Container[2].alarm_level = 0;
        Serial.print("     3rd aircraft");
        Serial.print(Container[2].bearing);
        Serial.print(" deg, "); 
        Serial.print(Container[2].distance); 
        Serial.print( " m, "); 
        Serial.print(Container[2].altitude-ThisAircraft.altitude);
        Serial.print(" m above, ");
        Serial.print(round(atan2(Container[2].altitude-ThisAircraft.altitude,Container[2].distance)*180/3.1415f));
        Serial.print(" deg (elev), alarm level: ");
        Serial.print(Container[2].alarm_level);
        Serial.println(" (must not disable overall alarm state)");
    }
  }
  simulateTrafficCounter++;
  #endif // OLED_TRAFFIC_SIMULATE_TRAFFIC

  const int sectorAngleSpan = 360/NUM_SECTORS;
  float tangent = 0;
  float verticalSeparation = 10000.0;
  int distance = 0;
  int bearing = 0;
  unsigned char sectorIndex, closestNeighbourSectorIndex, verticalIndicatorIndex = 0;
  unsigned char shouldBlink = 0;
  for (int i=0; i < MAX_TRACKING_OBJECTS; i++) {
    if (Container[i].addr && (now() - Container[i].timestamp) <= LED_EXPIRATION_TIME) {
        bearing  = (int) Container[i].bearing;
        distance = (int) Container[i].distance; // 2d distance
        verticalSeparation = Container[i].altitude - ThisAircraft.altitude;

        if (settings->pointer == DIRECTION_TRACK_UP) {
          bearing = (360 + bearing - (int)ThisAircraft.course) % 360;
        }
        
        
        sectorIndex = (bearing / sectorAngleSpan) % NUM_SECTORS;
        closestNeighbourSectorIndex = ((bearing + sectorAngleSpan / 2) / sectorAngleSpan ) % NUM_SECTORS;
        if (closestNeighbourSectorIndex == sectorIndex){
          closestNeighbourSectorIndex = (closestNeighbourSectorIndex + (NUM_SECTORS - 1) ) % NUM_SECTORS;
        }

        if (Container[i].distance < 1){
          tangent = verticalSeparation >=0 ? 1 : -1; // prevent division by zero for overflight.
        }
        else{
          tangent = verticalSeparation / Container[i].distance;
        }
        verticalIndicatorIndex = 1; // default to +7 deg indicator

        if (tangent > 0.2493){ // > +14 deg
          verticalIndicatorIndex = 0;
        }else if (tangent > 0){ // > 0 deg
          verticalIndicatorIndex = 1;
        }else if (tangent < -0.2493){ // < -14 deg
          verticalIndicatorIndex = 3; 
        }
        else if (tangent < 0){ // < 0 deg
          verticalIndicatorIndex = 2;
        }
      
      sectorAlertLevels[sectorIndex] = sectorAlertLevels[sectorIndex] < Container[i].alarm_level ? Container[i].alarm_level : sectorAlertLevels[sectorIndex];
      if (Container[i].alarm_level == ALARM_LEVEL_URGENT){
        shouldBlink = 1;
        // Original flarm makes three segments blink alert level URGENT, where the plane is in the middle sector
        // raise alert level for next sector
        sectorAlertLevels[(sectorIndex + 1) % NUM_SECTORS] = sectorAlertLevels[(sectorIndex + 1) % NUM_SECTORS] < 2 ? 2 : sectorAlertLevels[(sectorIndex + 1) % NUM_SECTORS];
        // raise alert level for previous sector
        sectorAlertLevels[(sectorIndex + (NUM_SECTORS - 1)) % NUM_SECTORS] = sectorAlertLevels[(sectorIndex + (NUM_SECTORS - 1)) % NUM_SECTORS] < 2 ? 2 : sectorAlertLevels[(sectorIndex + (NUM_SECTORS - 1)) % NUM_SECTORS];
      }
      else if (Container[i].alarm_level == ALARM_LEVEL_IMPORTANT){
        shouldBlink = 1;
        // Original flarm makes two segments blink for alert level MEDIUM
        // raise alert level for closest neighbor sector
        sectorAlertLevels[closestNeighbourSectorIndex] = sectorAlertLevels[closestNeighbourSectorIndex] < 2 ? 2 : sectorAlertLevels[closestNeighbourSectorIndex];
      }
      else if (Container[i].alarm_level == ALARM_LEVEL_LOW){
        shouldBlink = 1;
      }
      
      sectorDistances[sectorIndex] = sectorDistances[sectorIndex] > Container[i].distance ? Container[i].distance : sectorDistances[sectorIndex];
      sectorVerticalSeparations[sectorIndex] = sectorVerticalSeparations[sectorIndex] > verticalSeparation ? verticalSeparation : sectorVerticalSeparations[sectorIndex];

      verticalAlertLevels[verticalIndicatorIndex] = verticalAlertLevels[verticalIndicatorIndex] < Container[i].alarm_level ? Container[i].alarm_level : verticalAlertLevels[verticalIndicatorIndex];
      verticalDistances[verticalIndicatorIndex] = verticalDistances[verticalIndicatorIndex] > Container[i].distance ? Container[i].distance : verticalDistances[verticalIndicatorIndex];
      verticalIndicatorVerticalSeparations[verticalIndicatorIndex] = verticalIndicatorVerticalSeparations[verticalIndicatorIndex] > verticalSeparation ? verticalSeparation : verticalIndicatorVerticalSeparations[verticalIndicatorIndex];
    }
  }

   /* XBM bitmap */
  unsigned char bitmap[1024] = {0};

  /* helper variables for pixel to bitmap bit conversion */
  int pixelNumber = 0;
  int byteNumber = 0;
  int bitNumber = 0;

  unsigned char indicatorSize=0;

  int x,y=0;

  static unsigned char blinkState = 0;

  // iterate over angular indicators
  for (int i = 0; i<12; i++){
    if (sectorAlertLevels[i] < 1 || blinkState % 2 == 0){
      indicatorSize = alertLevelIndicatorSize[sectorAlertLevels[i]];
      if (sectorAlertLevels[i] == 0 && sectorDistances[i] < OLED_TRAFFIC_DISTANCE_INFO && sectorVerticalSeparations[i] < OLED_TRAFFIC_VERTICAL_SEPARATION_INFO){
        indicatorSize = nearestIndicatorSize;
      }
      for (int relativeX = -indicatorSize/2; relativeX<=indicatorSize/2; relativeX++){
        for (int relativeY = -indicatorSize/2; relativeY<=indicatorSize/2; relativeY++){
          /* setPixel(int x, int y) */
          /* setPixel(indicatorPositions[i][0]+x,indicatorPositions[i][1]+y); */
         
         /* tiles */
          x = indicatorPositions[i][0] + relativeX;
          y = indicatorPositions[i][1] + relativeY;
          //pixelNumber = y * WIDTH + x;
          byteNumber = WIDTH * (y / 8) + x;   //x + WIDTH/8 * (y % 8);
          bitNumber = (y % 8);

          /* XBM */
          /*
          x=indicatorPositions[i][0]+relativeX;
          y=indicatorPositions[i][1]+relativeY;
          pixelNumber = y*WIDTH + x;
          byteNumber = pixelNumber/8; 
          bitNumber = pixelNumber % 8;
          */
          bitmap[byteNumber] |= 1 << bitNumber;
          /* end setPixel */
        }
      }
    }
  }

  // iterate over vertical indicators
  for (int i = 0; i < 4; i++){
    if (verticalAlertLevels[i] < 1 || blinkState % 2 == 0){
      indicatorSize = alertLevelIndicatorSize[verticalAlertLevels[i]];
      if (verticalAlertLevels[i] == 0 && verticalDistances[i] < OLED_TRAFFIC_DISTANCE_INFO && verticalIndicatorVerticalSeparations[i] < OLED_TRAFFIC_VERTICAL_SEPARATION_INFO){
        indicatorSize = nearestIndicatorSize;
      }
      for (int relativeX = -indicatorSize/2; relativeX<=indicatorSize/2; relativeX++){
        for (int relativeY = -indicatorSize/2; relativeY<=indicatorSize/2; relativeY++){
         /* tiles */
          x = verticalIndicatorPositions[i][0] + relativeX;
          y = verticalIndicatorPositions[i][1] + relativeY;
          //pixelNumber = y * WIDTH + x;
          byteNumber = WIDTH * (y / 8) + x;   //x + WIDTH/8 * (y % 8);
          bitNumber = (y % 8);
          bitmap[byteNumber] |= 1 << bitNumber;
          /* end setPixel */
        }
      }
    }
  }

  for (int i = 0; i<HEIGHT/8; i++){
    u8x8->drawTile(0,i, 16, bitmap+i*128);
  }

  blinkState = shouldBlink ? blinkState + 1 : 0;
  
}
#endif /* EXCLUDE_OLED_TRAFFIC_PAGE */

void OLED_loop()
{
  if (u8x8) {
    if (isTimeToOLED()) {
#if !defined(EXCLUDE_OLED_049)
      if (hw_info.display == DISPLAY_OLED_0_49) {
        OLED_049_func();
      } else
#endif /* EXCLUDE_OLED_049 */
        switch (OLED_current_page)
        {
        case OLED_PAGE_OTHER:
          OLED_other();
          break;
#if !defined(EXCLUDE_OLED_BARO_PAGE)
        case OLED_PAGE_BARO:
          OLED_baro();
          break;
#endif /* EXCLUDE_OLED_BARO_PAGE */
#if !defined(EXCLUDE_OLED_TRAFFIC_PAGE)
        case OLED_PAGE_TRAFFIC:
          OLED_traffic();
          break;
#endif /* EXCLUDE_OLED_TRAFFIC_PAGE */
        case OLED_PAGE_RADIO:
        default:
          OLED_radio();
          break;
        }

      OLEDTimeMarker = millis();
    }
  }
}

void OLED_fini(int reason)
{
  if (u8x8) {
    u8x8->clear();
    switch (hw_info.display)
    {
#if !defined(EXCLUDE_OLED_049)
    case DISPLAY_OLED_0_49:
      u8x8->draw2x2String(5, 5, reason == SOFTRF_SHUTDOWN_LOWBAT ?
                                "BAT" : "OFF");
      delay(2000);
      u8x8->noDisplay();
      break;
#endif /* EXCLUDE_OLED_049 */
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    default:
      u8x8->draw2x2String(1, 3, reason == SOFTRF_SHUTDOWN_LOWBAT ?
                                "LOW BAT" : "  OFF  ");
      break;
    }
  }
}

void OLED_info1()
{
  if (u8x8) {

    u8x8->clear();

    switch (hw_info.display)
    {
#if !defined(EXCLUDE_OLED_049)
    case DISPLAY_OLED_0_49:
      {
        u8x8->draw2x2Glyph(  4, 4, 'R');
        u8x8->draw2x2Glyph(  6, 4, hw_info.rf      != RF_IC_NONE       ? '+' : '-');
        u8x8->draw2x2Glyph(  8, 4, 'G');
        u8x8->draw2x2Glyph( 10, 4, hw_info.gnss    != GNSS_MODULE_NONE ? '+' : '-');
        u8x8->draw2x2Glyph(  4, 6, 'O');
        u8x8->draw2x2Glyph(  6, 6, hw_info.display != DISPLAY_NONE     ? '+' : '-');
        u8x8->draw2x2Glyph(  8, 6, 'I');
        u8x8->draw2x2Glyph( 10, 6, hw_info.imu     != IMU_NONE         ? '+' : '-');

        delay(3000);

        const char buf[] = SOFTRF_FIRMWARE_VERSION;
        int ndx = strlen(buf) - 3;
        ndx = ndx < 0 ? 0 : ndx;
        u8x8->clear();
        u8x8->drawString  (4, 4, "VERSION");
        u8x8->draw2x2Glyph(5, 6, toupper(buf[ndx++]));
        u8x8->draw2x2Glyph(7, 6, toupper(buf[ndx++]));
        u8x8->draw2x2Glyph(9, 6, toupper(buf[ndx]));

        delay(2000);

        u8x8->clear();
        u8x8->drawString   (4, 4, "REGION");
        u8x8->draw2x2String(6, 6, ISO3166_CC[settings->band]);
      }
      break;
#endif /* EXCLUDE_OLED_049 */
    case DISPLAY_OLED_TTGO:
    case DISPLAY_OLED_HELTEC:
    default:

      u8x8->draw2x2String(0, 0, "RADIO");
      u8x8->draw2x2String(14, 0, hw_info.rf   != RF_IC_NONE       ? "+" : "-");
      u8x8->draw2x2String(0, 2, "GNSS");
      u8x8->draw2x2String(14, 2, hw_info.gnss != GNSS_MODULE_NONE ? "+" : "-");
      u8x8->draw2x2String(0, 4, "OLED");
      u8x8->draw2x2String(14, 4, hw_info.display != DISPLAY_NONE  ? "+" : "-");
      u8x8->draw2x2String(0, 6, "BARO");
      u8x8->draw2x2String(14, 6, hw_info.baro != BARO_MODULE_NONE ? "+" : "-");

      break;
    }

    delay(3000);
  }
}

void OLED_Next_Page()
{
  if (u8x8) {
    OLED_current_page = (OLED_current_page + 1) % page_count;

#if !defined(EXCLUDE_OLED_BARO_PAGE)
    if (hw_info.display   != DISPLAY_OLED_0_49 &&
        OLED_current_page == OLED_PAGE_BARO    &&
        hw_info.baro      == BARO_MODULE_NONE) {
      OLED_current_page = (OLED_current_page + 1) % page_count;
    }
#endif /* EXCLUDE_OLED_BARO_PAGE */

#if !defined(EXCLUDE_OLED_049)
    if (hw_info.display   == DISPLAY_OLED_0_49      &&
        OLED_current_page == OLED_049_PAGE_ACFTS    &&
        settings->power_save & POWER_SAVE_NORECEIVE &&
        (hw_info.rf == RF_IC_SX1276 || hw_info.rf == RF_IC_SX1262)) {
      OLED_current_page = (OLED_current_page + 1) % page_count;
    }
#endif /* EXCLUDE_OLED_049 */

    OLED_display_titles = false;
  }
}

#endif /* USE_OLED */

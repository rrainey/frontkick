/*
 * This file is part of the Kick distribution (https://github.com/rrainey/frontkick
 * Copyright (c) 2022 Riley Rainey
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * --------------------------------------------------------------------
 * 
 * Frontkick Wrist Display application
 * Currently designed for a LilyGo V2.2 2.9" e-paper display.
 *
 * ESP32 processor w/Wifi & BT
 * SD card slot
 *
 * Graphics based on LilyGo sample code originally written by Lewis He
 */

#include <BluetoothSerial.h>
#include <SD.h>
#include "LogParser.h"
#include "LogPlayback.h"
#include "Geodesy.h"
#include "AppConfig.h"


unsigned long g_ulNextGraphicsUpdate_ms = 0;
unsigned long g_ulTotalUpdateTime_ms = 0;
unsigned long g_ulPassCount = 0;
unsigned long g_ulNextStatsUpdate_ms = 0;

#define PERF_UPDATE_INTERVAL_MS 30000

#define GRAPHICS_UPDATE_INTERVAL_MS 300

using namespace Geodesy;

enum AppMode { 
  appModeStarting,
  appModeWaitingForConnection,
  appModeNormal,
  appModeLogging,
  appModePlayback 
  };

AppMode g_eAppMode = appModeStarting;

bool setupSDCard(void);
#if defined(_HAS_SDCARD_) && !defined(_USE_SHARED_SPI_BUS_)
SPIClass SDSPI(VSPI);
#endif
bool rlst = false;

char * generateLogname(char *gname, int *pnIndex);

/*
 * From the LilyGo Template code; select exactly one board type to define
 * then include "boards.h"
 */
// #define LILYGO_T5_V213
#define LILYGO_T5_V22
// #define LILYGO_T5_V24
// #define LILYGO_T5_V28
// #define LILYGO_T5_V266
// #define LILYGO_EPD_DISPLAY_102
// #define LILYGO_EPD_DISPLAY_154

#include "boards.h"
#include <GxEPD.h>

#if defined(LILYGO_T5_V102) || defined(LILYGO_EPD_DISPLAY_102)
#include <GxGDGDEW0102T4/GxGDGDEW0102T4.h> //1.02" b/w
#elif defined(LILYGO_T5_V266)
#include <GxDEPG0266BN/GxDEPG0266BN.h>    // 2.66" b/w   form DKE GROUP
#elif defined(LILYGO_T5_V213)
#include <GxDEPG0213BN/GxDEPG0213BN.h>    // 2.13" b/w  form DKE GROUP
#else
// #include <GxGDGDEW0102T4/GxGDGDEW0102T4.h> //1.02" b/w
// #include <GxGDEW0154Z04/GxGDEW0154Z04.h>  // 1.54" b/w/r 200x200
// #include <GxGDEW0154Z17/GxGDEW0154Z17.h>  // 1.54" b/w/r 152x152
// #include <GxGDEH0154D67/GxGDEH0154D67.h>  // 1.54" b/w
// #include <GxDEPG0150BN/GxDEPG0150BN.h>    // 1.51" b/w   form DKE GROUP
// #include <GxDEPG0266BN/GxDEPG0266BN.h>    // 2.66" b/w   form DKE GROUP
// #include <GxDEPG0290R/GxDEPG0290R.h>      // 2.9" b/w/r  form DKE GROUP
// #include <GxDEPG0290B/GxDEPG0290B.h>      // 2.9" b/w    form DKE GROUP
#include "GxDEPG0290B.h"                     // 2.9" b/w    form DKE GROUP
// #include <GxGDEW029Z10/GxGDEW029Z10.h>    // 2.9" b/w/r  form GoodDisplay
// #include <GxGDEW0213Z16/GxGDEW0213Z16.h>  // 2.13" b/w/r form GoodDisplay
// #include <GxGDE0213B1/GxGDE0213B1.h>      // 2.13" b/w  old panel , form GoodDisplay
// #include <GxGDEH0213B72/GxGDEH0213B72.h>  // 2.13" b/w  old panel , form GoodDisplay
// #include <GxGDEH0213B73/GxGDEH0213B73.h>  // 2.13" b/w  old panel , form GoodDisplay
// #include <GxGDEM0213B74/GxGDEM0213B74.h>  // 2.13" b/w  form GoodDisplay 4-color
// #include <GxGDEW0213M21/GxGDEW0213M21.h>  // 2.13"  b/w Ultra wide temperature , form GoodDisplay
// #include <GxDEPG0213BN/GxDEPG0213BN.h>    // 2.13" b/w  form DKE GROUP
// #include <GxGDEW027W3/GxGDEW027W3.h>      // 2.7" b/w   form GoodDisplay
// #include <GxGDEW027C44/GxGDEW027C44.h>    // 2.7" b/w/r form GoodDisplay
// #include <GxGDEH029A1/GxGDEH029A1.h>      // 2.9" b/w   form GoodDisplay
// #include <GxDEPG0750BN/GxDEPG0750BN.h>    // 7.5" b/w   form DKE GROUP
#endif

#include "Comfortaa_Medium48pt7b.h"
#include "Comfortaa_Medium23pt7b.h"
#include "Comfortaa_Medium15pt7b.h"

#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>
#include <WiFi.h>

LogPlayback * g_pPlayback;
LogParser g_logParser;
AppConfig *g_pAppConfig = NULL;
SDFile g_playbackFile;

GxIO_Class io(SPI,  EPD_CS, EPD_DC,  EPD_RSET);
GxEPD_Class display(io, EPD_RSET, EPD_BUSY);

#define EPD_WIDTH  296
#define EPD_HEIGHT 128

#define SLOPE_AREA_X 257
#define TICKS_RIGHT_X 282
#define SLOPE_AREA_WIDTH (EPD_WIDTH-SLOPE_AREA_X)
#define SLOPE_AREA_HEIGHT EPD_HEIGHT

#define ALT_AREA_WIDTH 180
#define ALT_AREA_HEIGHT 75

#define TICK_BIG_WIDTH (TICKS_RIGHT_X-SLOPE_AREA_X)
#define TICK_SMALL_WIDTH (TICKS_RIGHT_X-265)
#define TICK_HEIGHT 5

#define PIXELS_PER_POINT(x) (((-x) * 15)/100)
#define SLOPE_ZERO_Y 64

#define TARGET_GUIDE_SIZE 85
#define TARGET_GUIDE_D    12    // offset of range readout opposite of direction pointer
#define TARGET_GUIDE_P1    (10)
#define TARGET_GUIDE_P2    (42)

#define TARGET_GUIDE_X  (SLOPE_AREA_X - TARGET_GUIDE_SIZE)
#define TARGET_GUIDE_Y (EPD_HEIGHT - TARGET_GUIDE_SIZE)

GFXcanvas1 canvas1(ALT_AREA_WIDTH,ALT_AREA_HEIGHT);
GFXcanvas1 canvas2(SLOPE_AREA_WIDTH, SLOPE_AREA_HEIGHT);
GFXcanvas1 canvas4(TARGET_GUIDE_SIZE, TARGET_GUIDE_SIZE);

typedef struct _SimplePoint {
  int16_t  x, y;
} SimplePoint;

/*
 * This represents the caret pointing the relative direction to
 * the landing target.
 */
SimplePoint gTargetTriangle[] = {
  { 0, - TARGET_GUIDE_P2 },
  { 8, - TARGET_GUIDE_P2 + 20 },
  { -8, - TARGET_GUIDE_P2 + 20 }
};

typedef struct _SimpleBox {
  int16_t  x, y;
  uint16_t w, h;
} SimpleBox;

/*
 * "Glide Slope" ticks in screen coordinates
 */
SimpleBox ticks[] = {
  { 265, 18-2, TICK_SMALL_WIDTH, TICK_HEIGHT },
  { 265, 34-2, TICK_SMALL_WIDTH, TICK_HEIGHT },
  { 265, 49-2, TICK_SMALL_WIDTH, TICK_HEIGHT },
  { 257, 64-2, TICK_BIG_WIDTH, TICK_HEIGHT },
  { 265, 79-2, TICK_SMALL_WIDTH, TICK_HEIGHT },
  { 265, 94-2, TICK_SMALL_WIDTH, TICK_HEIGHT },
  { 265, 109-2, TICK_SMALL_WIDTH, TICK_HEIGHT }
};

SimpleBox dirtyBoxes[32];
int dbTop = 0;

uint8_t address1[6] = {0xC7, 0x99, 0xC9, 0xF9, 0xAE, 0x32};
uint8_t address2[6] = {0xDA, 0x7A, 0xC7, 0x99, 0xC9, 0xF9};
char * pszNname = "DA7AC799C9F9AE32";
char *pszPin = "1234"; //<- standard pin would be provided by default
bool connected = false;

BluetoothSerial SerialBT;

/*
 * if(connected) {
Serial.println("Connected Succesfully!");
} else {
while(!SerialBT.connected(10000)) {
Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app.");
}
}

// disconnect() may take upto 10 secs max
if (SerialBT.disconnect()) {
Serial.println("Disconnected Succesfully!");
}
// this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
SerialBT.connect();
}
 */
File logFile;
char logpath[32];

/**
 * Callback to process data received from the sensor pack
 */
void OnBluetoothDataReceived (const uint8_t *pBuffer, size_t nSize) {

  /*
   * Analyze received data that might affect the display
   */
  g_logParser.ProcessIncomingStreamBytes( (const char *) pBuffer, nSize);

  /*
   * Starting a new log file?
   */
  if ( g_eAppMode == appModeNormal ) {

    g_eAppMode = appModeLogging;

    generateLogname( logpath, &g_pAppConfig->m_nLastJumpNumber );
    logFile = SD.open( logpath, FILE_WRITE );

    //if (! saveConfigurationToFile("/config.txt") ) {
    //  Serial.println(F("Unable to save configuration file"));
    //}
  }

  /*
   * Store data
   */
  logFile.write( pBuffer, nSize );

  if (strstr( (const char *) pBuffer, "$PCLOS") != NULL) {
    logFile.close();
    g_eAppMode = appModeNormal;
  }
}

/**
 * Callback to handle data from logfile playback
 */
void OnPlaybackDataReceived(const uint8_t *pBuffer, size_t nSize) 
{
  OnBluetoothDataReceived( pBuffer, nSize );
}

void partialUpdate()
{
#ifdef notdef
  int i;
  for (i=0; i<dbTop; ++i) {
    SimpleBox *p = &dirtyBoxes[i];
    //display.updateWindow(p->x, p->y, p->w, p->h, true);
    display.drawBitmap(canvas.getBuffer(), p->x, p->y, p->w, p->h, GxEPD_BLACK);
    display.updateWindow(p->x, p->y, p->w, p->h, false);
  }
#endif
  display.drawBitmap(canvas1.getBuffer(), 0, 0, ALT_AREA_WIDTH, ALT_AREA_HEIGHT, GxEPD_WHITE, GxEPD_BLACK);
  display.updateWindow( 0, 0, ALT_AREA_WIDTH, ALT_AREA_HEIGHT, true);
  //display.drawBitmap(canvas2.getBuffer(), SLOPE_AREA_X, 0, SLOPE_AREA_WIDTH, SLOPE_AREA_HEIGHT, GxEPD_WHITE, GxEPD_BLACK);
  //display.updateWindow( SLOPE_AREA_X, 0, SLOPE_AREA_WIDTH-1, SLOPE_AREA_HEIGHT, true);
  //display.drawBitmap(canvas4.getBuffer(), TARGET_GUIDE_X, TARGET_GUIDE_Y, TARGET_GUIDE_SIZE, TARGET_GUIDE_SIZE, GxEPD_WHITE, GxEPD_BLACK);
  //display.updateWindow( TARGET_GUIDE_X, TARGET_GUIDE_Y, TARGET_GUIDE_SIZE, TARGET_GUIDE_SIZE, true);
}

void prepAltitudeDisplay(int nAlt, char * pThousands, char *pHundreds)
{
  int n = nAlt / 1000;
  if (n == 0) {
    strcpy(pThousands, "  ");
  }
  else {
    sprintf(pThousands, "%2d", n);
  }
  sprintf(pHundreds, "%03.3d", ((abs(nAlt) % 1000) / 10) * 10);
}

void markBoxDirty(int16_t x, int16_t y, uint16_t w, uint16_t h)
{
  SimpleBox *p = &dirtyBoxes[dbTop++];
  p->x = x;
  p->y = y;
  p->w = w;
  p->h = h;
}

void clearDirtyBoxes()
{
  /*
  int i;
  for (i=0; i<dbTop; ++i) {
    SimpleBox *p = &dirtyBoxes[i];
    canvas.fillRect(p->x, p->y, p->w, p->h, GxEPD_WHITE);
  }
  */
  canvas1.fillScreen(GxEPD_WHITE);
  //canvas2.fillScreen(GxEPD_WHITE);
  //canvas4.fillScreen(GxEPD_WHITE);
  dbTop = 0;
}

bool bFillCycle = true;

void drawSlope(int nDot_points)
{
  int i;
  int nTicks = sizeof(ticks)/sizeof(SimpleBox);
  int nPoint_y;
  boolean bOffScale = false;
  SimpleBox *p;

  nPoint_y = PIXELS_PER_POINT(nDot_points) + SLOPE_ZERO_Y;
  if (nPoint_y < 0) {
    nPoint_y = 0;
    bOffScale = true;
  }
  else if (nPoint_y >= 128) {
    nPoint_y = 127;
    bOffScale = true;
  }
  
  for(i=0; i<nTicks; ++i) {
    p = &ticks[i];
    canvas2.fillRect(p->x - SLOPE_AREA_X, p->y, p->w, p->h, GxEPD_BLACK);
  }

  uint16_t x0, x1, x2;
  uint16_t y0, y1, y2;

  x0 = TICKS_RIGHT_X - SLOPE_AREA_X;
  y0 = nPoint_y;
  x1 = x0 + 12;
  y1 = nPoint_y - 9;
  x2 = x0 + 12;
  y2 = nPoint_y + 9;

  if (g_logParser.m_bGlideSlopeValid) {
    if (bOffScale) { 
      canvas2.drawTriangle(x0, y0, x1, y1, x2, y2, GxEPD_BLACK);
    }
    else {
      canvas2.fillTriangle(x0, y0, x1, y1, x2, y2, GxEPD_BLACK);
    }
}

  bFillCycle = ! bFillCycle;
}

#define DEGtoRAD(x) ((x) * 3.141592653f / 180.0f )

void drawTargetGuide(float fHeading_rad, float fDist)
{
  float cosH, sinH;
  int16_t  x_b, y_b;
  int16_t  x, y;
  int16_t  x1, y1;
  int16_t  x2, y2;
  uint16_t w, h;
  char szDist[6];
  int16_t h_baseline;
  unsigned long ulUpdateStartTime_ms;

  sinH = sin( fHeading_rad );
  cosH = cos( fHeading_rad );

  // Units are kilometers (may be configurable in the future)
  sprintf( szDist, "%3.2f", fDist );

  strcpy(szDist, "9.9");

  x = 0;
  y = 25;
  canvas4.setCursor(x,y);
  canvas4.getTextBounds(szDist, x, y, &x1, &y1, &w, &h);

  h_baseline = 25 - y1;

  // center of cicle
  x = TARGET_GUIDE_SIZE / 2;
  y = x;

  // top, left corner of text on canvas 4
  x_b = x - TARGET_GUIDE_D * sinH - w / 2;
  y_b = y + TARGET_GUIDE_D * cosH + h_baseline / 2;

  canvas4.drawCircle(x, y, TARGET_GUIDE_P2, GxEPD_BLACK);
  canvas4.drawCircle(x, y, TARGET_GUIDE_P2-1, GxEPD_BLACK);
  canvas4.drawCircle(x, y, TARGET_GUIDE_P2-2, GxEPD_BLACK);
  //canvas4.drawLine(TARGET_GUIDE_P1 * sinH + x, - TARGET_GUIDE_P1 * cosH + y, 
  //                 TARGET_GUIDE_P2 * sinH + x, - TARGET_GUIDE_P2 * cosH + y, GxEPD_BLACK);

  if (g_logParser.m_bLocationValid && g_logParser.m_bTargetValid) {
    canvas4.fillTriangle(
      gTargetTriangle[0].x * cosH - gTargetTriangle[0].y * sinH + x, gTargetTriangle[0].x * sinH + gTargetTriangle[0].y * cosH + y,
      gTargetTriangle[1].x * cosH - gTargetTriangle[1].y * sinH + x, gTargetTriangle[1].x * sinH + gTargetTriangle[1].y * cosH + y,
      gTargetTriangle[2].x * cosH - gTargetTriangle[2].y * sinH + x, gTargetTriangle[2].x * sinH + gTargetTriangle[2].y * cosH + y,
      GxEPD_BLACK);
    canvas4.setCursor(x_b, y_b);
    canvas4.print(szDist);
  }
}

void updateDisplay()
{
  unsigned long ulTime_ms = millis();

  if ( ulTime_ms > g_ulNextGraphicsUpdate_ms ) {

    char thousands[3];
    char hundreds[4];
    int nAlt_ft = METERStoFEET(g_logParser.m_dBarometricAltitudeAGL_m);

    prepAltitudeDisplay(nAlt_ft, thousands, hundreds);

    int16_t  x, y;
    int16_t  x1, y1;
    uint16_t w, h;

    clearDirtyBoxes();

    display.setTextColor(GxEPD_BLACK);

    x = 0;
    y = ALT_AREA_HEIGHT;
    canvas1.setCursor(x, y);
    canvas1.setFont(&Comfortaa_Medium48pt7b);
    canvas1.getTextBounds(thousands, x, y, &x1, &y1, &w, &h);
    markBoxDirty(x,y,w,h);
    canvas1.print (thousands);

    canvas1.setFont(&Comfortaa_Medium23pt7b);
    x = x + w + 5;
    y = 36;
    canvas1.setCursor(x,y);
    canvas1.getTextBounds(hundreds, x, y, &x1, &y1, &w, &h);
    markBoxDirty(x,y,w,h);
    canvas1.print (hundreds);

    drawSlope( (int) (g_logParser.m_fGlideSlope_dots * 100.0f) );

    drawTargetGuide(g_logParser.m_fTargetBearing_rad, 
      (g_logParser.m_fTargetRange_m/1000.0f));

/*
  x = 0;
  y = 118;
  canvas.setFont(&Comfortaa_Medium15pt7b);
  canvas.setCursor(x, y);
  canvas.getTextBounds("1000", x, y, &x1, &y1, &w, &h);
  markBoxDirty(x,y,w,h);
  canvas.print ("1000");

  x = 190;
  y = 114;
  canvas.setFont(&Comfortaa_Medium15pt7b);
  canvas.setCursor(x, y);
  canvas.getTextBounds("1.2", x, y, &x1, &y1, &w, &h);
  markBoxDirty(x,y,w,h);
  canvas.print ("1.2");

  x = 190;
  y = 81;
  canvas.drawCircle( x-90/2, y-90/2, 90/2, GxEPD_BLACK );
  markBoxDirty(x-90/2, y-90/2,90,90);
*/

    partialUpdate();

    g_ulNextGraphicsUpdate_ms = ulTime_ms + GRAPHICS_UPDATE_INTERVAL_MS;

    g_ulTotalUpdateTime_ms += millis() - ulTime_ms;
    g_ulPassCount++;

  }
}

void setup(void)
{
    Serial.begin(115200);
    delay(2000);
    Serial.println();
    Serial.println(F("reboot"));

#if defined(LILYGO_EPD_DISPLAY_102)
    pinMode(EPD_POWER_ENABLE, OUTPUT);
    digitalWrite(EPD_POWER_ENABLE, HIGH);
#endif /*LILYGO_EPD_DISPLAY_102*/
#if defined(LILYGO_T5_V102)
    pinMode(POWER_ENABLE, OUTPUT);
    digitalWrite(POWER_ENABLE, HIGH);
#endif /*LILYGO_T5_V102*/

    SPI.begin(EPD_SCLK, EPD_MISO, EPD_MOSI);

    rlst = setupSDCard();

    /*
     * Load application configuration settings from SD card
     */
    g_pAppConfig = new AppConfig();;

    g_pAppConfig->m_nLastJumpNumber = 0;
    g_pAppConfig->m_nWiFiNetworkCount = 0;
    strcpy(g_pAppConfig->m_szJumperName, "None");
    strcpy(g_pAppConfig->m_szReplayLogfile, "");
    strcpy(g_pAppConfig->m_sensorPackName, "");
    g_pAppConfig->m_dTargetLatitude_rad = 0.0;
    g_pAppConfig->m_dTargetLongitude_rad = 0.0;
    g_pAppConfig->m_dTargetAltitude_m = 0;

    if (g_pAppConfig->loadConfiguration("/config.txt") ) {
      Serial.println(F("Configuration settings loaded from SD card"));
    }
    else {
      Serial.println(F("Configuration file not found; using defaults"));
    }

    //g_pAppConfig->saveConfigurationToStream(Serial);

    if (strlen(g_pAppConfig->m_sensorPackName) == 0) {
      strcpy(g_pAppConfig->m_sensorPackName, pszNname);
    }

    Serial.println(F("initializing display"));

    display.init();
    display.setRotation(3);
    display.setTextColor(GxEPD_BLACK);
    display.fillScreen(GxEPD_WHITE);

    display.update();

    canvas1.fillScreen(GxEPD_WHITE);
    canvas1.setTextColor(GxEPD_BLACK);

    canvas4.fillScreen(GxEPD_WHITE);
    canvas4.setTextColor(GxEPD_BLACK);
    canvas4.setFont(&Comfortaa_Medium15pt7b);

#ifdef notdef
    /*
     * Bluetooth connect as master
     */
    SerialBT.begin("Wrist", true);
    SerialBT.setPin( pszPin );
    Serial.println(F("Bluetooth master mode, make sure Frontkick module is on"));

    /*
     * Register callback to process received data
     */
    SerialBT.onData( OnBluetoothDataReceived );
#endif

    g_eAppMode = appModeWaitingForConnection;

    /**
     * Temporary: test logfile playback
     */
    g_playbackFile = SD.open("/FF0226.TXT");
    if (! g_playbackFile) {
      Serial.println(F("Failed to open log playback file"));
      while (1) delay(10);
    }
    g_pPlayback = new LogPlayback(g_playbackFile, OnPlaybackDataReceived);

    g_ulNextStatsUpdate_ms = millis() + PERF_UPDATE_INTERVAL_MS;

    g_ulTotalUpdateTime_ms = 0;
    g_ulPassCount = 0;
}

void loop()
{
  //Serial.println(F("loop()"));
  //delay(10);

  /*
   * Connect to the sensor pack
   */

#ifdef notdef
  if (!connected) {

    Serial.println(F("Attempting connection"));
    
    connected = SerialBT.connect( g_pAppConfig->m_sensorPackName );
    if (connected) {
      Serial.println(F("Connected succesfully"));
    } 
    else {
      while(!SerialBT.connected(10000)) {
        Serial.println(F("Failed to connect. Make sure remote device is available and in range, then restart app."));
      }
    }
    connected = true;
    g_eAppMode = appModeNormal;
  }
  #endif
  /*
   * Actual display update every 300ms
   */
  updateDisplay();

  /*
   * Playback in progress?  Handle movement forward in time
   */
  if (g_pPlayback) {
    if (! g_pPlayback->TimeStep() ) {
      delete g_pPlayback;
      g_pPlayback = NULL;
    }
  }

  /*
   * Display performance metrics
   */
  unsigned long ulTime_ms = millis();
  if (ulTime_ms > g_ulNextStatsUpdate_ms) {
    char szBuffer[128];
    g_ulNextStatsUpdate_ms = ulTime_ms + PERF_UPDATE_INTERVAL_MS;
    sprintf(szBuffer, "Avg display update time: %d ms", (int) (g_ulTotalUpdateTime_ms / g_ulPassCount) );
    g_ulTotalUpdateTime_ms = 0;
    g_ulPassCount = 0;
    Serial.println(szBuffer);
  }
}

/**
 * Initialize SD Card access
 */
bool setupSDCard(void)
{
#if defined(_HAS_SDCARD_) && !defined(_USE_SHARED_SPI_BUS_)
    SDSPI.begin(SDCARD_SCLK, SDCARD_MISO, SDCARD_MOSI);
    return SD.begin(SDCARD_CS, SDSPI);
#elif defined(_HAS_SDCARD_)
    return SD.begin(SDCARD_CS);
#endif
    return false;
}

/// @brief Generate a unique filename based on existing filenames already present on the SD card
/// @param gname buffer to hold generated name
/// @param pnIndex integer index to start the search at
/// @return buffer containing generated filename (gname)
char * generateLogname(char *gname, int *pnIndex)
{
  char * result = NULL;
  int i = *pnIndex;
  for (; true; i++) {
    sprintf (gname, "log%05d.txt", i);

    if (!SD.exists(gname)) {
        result = gname;
        *pnIndex = i;
        break;
    }
  }

  return result;
}

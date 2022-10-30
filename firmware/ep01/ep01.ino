/*
 * A simple test of swapping in partial updating from the GxEPD2 on the LilyGo T5 V2.2 board
 *
 * Based on code from https://create.arduino.cc/projecthub/galoebn/e-paper-display-using-partial-updates-a8af20
 */
#define ENABLE_GxEPD2_GFX 0

#include <GxEPD2_BW.h> // including both doesn't use more code or ram
#include <GxEPD2_3C.h> // including both doesn't use more code or ram
#include <U8g2_for_Adafruit_GFX.h>

/*
 * LilyGo pin definitions
 */
#define EPD_MOSI                (23)
#define EPD_MISO                (2)
#define EPD_SCLK                (18)
#define EPD_CS                  (5)
#define EPD_BUSY                (4)
#define EPD_RSET                (12)
#define EPD_DC                  (19)
#define SDCARD_CS               (13)

//if you have another microcontroller or another e-ink display module you have to change the following line
GxEPD2_BW<GxEPD2_290_T94_V2, GxEPD2_290_T94_V2::HEIGHT> display(GxEPD2_290_T94_V2(/*CS=5*/ 5, /*DC=*/ 19, /*RST=*/ 12, /*BUSY=*/ 4)); 

U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;

#include "Comfortaa_Medium18pt7b.h"

void setup()
{
  display.init();
  display.setTextColor(GxEPD_BLACK);
  display.firstPage();
  display.setRotation(1);

  u8g2Fonts.begin(display);
  delay(1000);

  uint16_t bg = GxEPD_WHITE;
  uint16_t fg = GxEPD_BLACK;
  u8g2Fonts.setForegroundColor(fg);
  u8g2Fonts.setBackgroundColor(bg);

  do {
    display.fillScreen(GxEPD_WHITE);

    u8g2Fonts.setFont(u8g2_font_fub20_tr);
    u8g2Fonts.setCursor(20, 80);
    u8g2Fonts.print("Millis: ");
  }
  while (display.nextPage());
}


void loop() {
  display.setPartialWindow(115, 50, 200, 40);
  display.firstPage();

  do {
    display.fillScreen(GxEPD_WHITE);
    u8g2Fonts.setCursor(120, 80);
    u8g2Fonts.print(millis());
  }
  while(display.nextPage());

}

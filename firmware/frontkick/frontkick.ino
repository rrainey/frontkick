/* 
 * This file is part of the Frontkick distribution (https://github.com/rrainey/frontkick).
 * Copyright (c) 2022 Riley Rainey.
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
 */

#define SERIAL_BUFFER_SIZE 512

#include <Wire.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "LoggerCore.h"

// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

class FrontLogger : public LoggerCore {
  public: 
    FrontLogger() {};

    /**
     * write() is the only method which must be overridden - this
     * directs all logstream output to the appropriate destination -
     * the Bluetooth UART stream, in this case.
     */
    size_t write(const uint8_t *buffer, size_t size) {
      bleuart.write( buffer, size );
      return size;
    }
};

FrontLogger sensors;

char szUniqueName[64];

char * getUniqueName() {
  strncpy(szUniqueName, "Frontkick sensor pack ", sizeof(szUniqueName));
  const char *p = getMcuUniqueID();
  strncat(szUniqueName, p+12, 8);
  return szUniqueName;
}

void setup()
{
  unsigned long ulStart_ms = millis();

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

#if 1
  // Blocking wait for connection when debug mode is enabled via IDE
  while ( !Serial && ((millis() - ulStart_ms) < 10000)) { delay(5); yield(); };
#endif
  
  Serial.println("Frontkick booting");
  Serial.println("-----------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName( getUniqueName() );
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Stark Industries");
  bledis.setModel("Frontkick Sensors");
  bledis.begin();

  bleuart.begin();

  blebas.begin();
  blebas.write(100);

  /*
   * Configure GNSS module
   */
  sensors.Initialize();

  // Set up and start advertising
  startAdv();
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void loop()
{

  /*
   * Invoke sensor logging functions which, in turn, will call write()
   * to send data into the Bluetooth UART stream.
   */
  sensors.Loop();

  // Forward from BLEUART to HW Serial
  while ( bleuart.available() )
  {
    uint8_t ch;
    ch = (uint8_t) bleuart.read();
    Serial.write(ch);
  }
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  sensors.SetStreamConnectionState(true);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  sensors.SetStreamConnectionState(false);
}

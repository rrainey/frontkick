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
#include "AppConfig.h"

#include <ArduinoJson.h>
#include <SD.h>
#include "Geodesy.h"
#include "LogParser.h"

using namespace Geodesy;

extern LogParser g_logParser;

/**
 * Load application configuration from SD card
 */
bool AppConfig::loadConfiguration(const char *filename) {

  bool bResult = true;
  
  File file = SD.open(filename);

  // Use arduinojson.org/assistant to compute the capacity.
  DynamicJsonDocument doc(4096);

  DeserializationError err = deserializeJson(doc, file);

  if (err != DeserializationError::Ok) {
    Serial.print(F("Can't load configuration file:"));
    Serial.println(err.c_str());
    return false;
  }

  m_nLastJumpNumber = doc["lastJump"] | 0;
  strlcpy(m_szJumperName,   
          doc["jumperName"] | "Jumper Name",
          sizeof(m_szJumperName));
  
  strlcpy(m_szReplayLogfile,   
          doc["replayLogfile"] | "",
          sizeof(m_szReplayLogfile));
  
  m_dTargetLatitude_rad = DEGtoRAD( doc["target"]["lat"] | 0.0) ;
  m_dTargetLongitude_rad = DEGtoRAD( doc["target"]["lon"] | 0.0) ;
  m_dTargetAltitude_m = doc["target"]["alt_m"] | 0.0 ;

  GeodeticPosition gpTarget;
  gpTarget.m_latitude_rad = m_dTargetLatitude_rad;
  gpTarget.m_longitude_rad = m_dTargetLongitude_rad;
  gpTarget.m_altitude_meters = m_dTargetAltitude_m;

  g_logParser.SetLandingTarget( gpTarget );

  strlcpy(m_szCanopyName,
          doc["canopyPerformance"]["name"]  | "",
          sizeof(m_szCanopyName));
  m_fWingLoading = doc["canopyPerformance"]["wingLoading"] | 0.0f;
  m_fForwardDrive_mps = doc["canopyPerformance"]["forwardDrive_mps"] | 10.0f;
  m_fDescentRate_mps = doc["canopyPerformance"]["descentRate_mps"] | 5.5f;

  JsonArray wifi = doc["wifi"];
  m_nWiFiNetworkCount = wifi.size();
  m_pWifiNetwork = (WiFiNetwork*) malloc(sizeof(WiFiNetwork) * m_nWiFiNetworkCount);
  /*
   * copy network info
   */

  int i;
  for(i=0; i<m_nWiFiNetworkCount; ++i) {
    const char * pSsid = doc["wifi"][i]["ssid"].as<const char*>();
    const char * pPassword = doc["wifi"][i]["password"].as<const char*>();

    strcpy( m_pWifiNetwork[i].ssidName, "" );
    strcpy( m_pWifiNetwork[i].ssidPassword, "" );

    if (pSsid) {
      strlcpy(m_pWifiNetwork[i].ssidName,   
          pSsid,
          sizeof(m_pWifiNetwork[i].ssidName));
    }
    if (pPassword) {
      strlcpy(m_pWifiNetwork[i].ssidPassword,   
          pPassword,
          sizeof(m_pWifiNetwork[i].ssidPassword));
    }
  }

  file.close();

  return bResult;
}

bool AppConfig::saveConfigurationToFile(const char * pszFilename)
{
  File file = SD.open(pszFilename, FILE_WRITE);
  bool bResult= saveConfigurationToStream(file);
  file.close();
  return bResult;
}

bool AppConfig::saveConfigurationToStream(Stream &s)
{
  DynamicJsonDocument doc(4096);
  int i;

  JsonObject root = doc.to<JsonObject>();

  root["jumperName"] = m_szJumperName;
  root["replayLogfile"] = m_szReplayLogfile;
  root["lastJump"] = m_nLastJumpNumber;

  if (m_dTargetLatitude_rad != 0.0) {
    JsonObject tobject = doc.createNestedObject("target");
    tobject["lat"] = RADtoDEG(m_dTargetLatitude_rad);
    tobject["lon"] = RADtoDEG(m_dTargetLongitude_rad);
    tobject["alt_m"] = RADtoDEG(m_dTargetAltitude_m);
    //root["target"] = tobject;
  }

  JsonObject cobject = doc.createNestedObject("canopyPerformance");
  cobject["name"] = m_szCanopyName;
  cobject["forwardDrive_mps"] = m_fForwardDrive_mps;
  cobject["descentRate_mps"] = m_fDescentRate_mps;
  cobject["wingLoading"] = m_fWingLoading;
  //root["canopyPerformance"] = cobject;

  JsonArray wifiArray = doc.createNestedArray("wifi");
  for(i=0; i<m_nWiFiNetworkCount; ++i) {
    JsonObject object = wifiArray.createNestedObject();
    object["ssid"] = m_pWifiNetwork[i].ssidName;
    object["password"] = m_pWifiNetwork[i].ssidPassword;
    //wifiArray.add(object);
  }
  //root["wifi"] = wifiArray;
  serializeJson(doc, s);
}

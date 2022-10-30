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
#ifndef APPCONFIG_H
#define APPCONFIG_H

#include <Arduino.h>

struct WiFiNetwork {
  char ssidName[32];
  char ssidPassword[32];
};

/**
 * Application Settings: stored on the SD Card
 */
class AppConfig {
public:
    char m_szJumperName[64];
    char m_szReplayLogfile[64];
    int  m_nLastJumpNumber;
    char m_sensorPackName[32];
    /*
    * WGS-84 coordinates of landing target
    */
    double m_dTargetLatitude_rad;
    double m_dTargetLongitude_rad;
    double m_dTargetAltitude_m;

    /*
    * Normal Canopy performance characteristics for this jumper and main canopy.
    * TODO: estimate change in performance based on different wing loading
    *       conditions.
    */
    char m_szCanopyName[64];
    float m_fWingLoading;
    float m_fForwardDrive_mps;
    float m_fDescentRate_mps;

    /*
    * Wifi Networks the app is authorized to use.
    * (Wifi not yet used)
    */
    int  m_nWiFiNetworkCount;
    struct WiFiNetwork *m_pWifiNetwork;

    bool loadConfiguration(const char *filename);
    bool saveConfigurationToStream(Stream &s);
    bool saveConfigurationToFile(const char *pszFilename);

};

#endif
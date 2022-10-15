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
 */
#include "LogParser.h"

LogParser * g_pLogParser = NULL;

static void handlePCLOSE() {

}

static void handleIMU() {
  g_pLogParser->HandleIMU();
}

void LogParser::HandleIMU() {
  unsigned long ulTimestamp;
  Vector3d vAccel_mps2;
  Vector3d vRates_rad_per_sec;

  m_parser.getArg(0, ulTimestamp);
  m_parser.getArg(1, vAccel_mps2.x);
  m_parser.getArg(2, vAccel_mps2.y);
  m_parser.getArg(3, vAccel_mps2.z);
  m_parser.getArg(4, vRates_rad_per_sec.x);
  m_parser.getArg(5, vRates_rad_per_sec.y);
  m_parser.getArg(6, vRates_rad_per_sec.z);

}

static void handleENV() {
  g_pLogParser->HandleENV();
}

void LogParser::HandleENV() {
  unsigned long ulTimestamp;
  float fPressure_hPa;
  float fAltitude_ftMSL;
  float fBattery_volts;
  double dBarometricAltitudeMSL_m;

  m_parser.getArg(0, ulTimestamp);
  m_parser.getArg(1, fPressure_hPa);
  m_parser.getArg(2, fAltitude_ftMSL);
  m_parser.getArg(3, fBattery_volts);

  dBarometricAltitudeMSL_m = FEETtoMETERS(fAltitude_ftMSL);

  m_fRateOfDescent_mps = (m_dBarometricAltitudeMSL_m - dBarometricAltitudeMSL_m) / (ulTimestamp - m_ulLastEnvReceived_ms);
  
  m_ulLastEnvReceived_ms = ulTimestamp;
  m_dBarometricAltitudeMSL_m = dBarometricAltitudeMSL_m;

  // TODO: enhance $PVER record to insert presumed ground level MSL altitude
  //m_dBarometricAltitudeAGL_m = m_dBarometricAltitudeMSL_m + m_gcTarget.z;

  m_dBarometricAltitudeAGL_m = m_dBarometricAltitudeMSL_m;
}

static void handleGGA() {
  g_pLogParser->HandleGGA();
}

void LogParser::HandleGGA() {
    char szLatitude[16];
    char szLongitude[16];
    char szNS[4];
    char szEW[4];
    char szFixTime[16];
    float fAltitude_m;
    float fHDOP;
    int nFixType;
    int nSatCount;

    m_ulTimeOfThisPositionReport_ms = millis();

    m_parser.getArg(0, szFixTime);
    m_parser.getArg(1, szLatitude);
    m_parser.getArg(2, szNS);
    m_parser.getArg(3, szLongitude);
    m_parser.getArg(4, szEW);
    m_parser.getArg(5, nFixType);
    m_parser.getArg(6, nSatCount);
    m_parser.getArg(7, fHDOP);
    m_parser.getArg(8, fAltitude_m);
    //parser.getArg(12, ageDGPS);

    // HDOP requirement here corrsponds to about 2.5 meter or less uncertainty
    m_bLocationValid = ((nFixType > 0) && (nSatCount > 3) && (fHDOP < 1.0f));

    if ( m_bLocationValid ) {
        // parse NMEA fields for Lat/Lon values
        m_gpCurrentGNSSPostion.m_latitude_rad = 
            DEGtoRAD((((szLatitude[0] - '0') * 10 + (szLatitude[1] - '0')) + 
            strtod(&szLatitude[2], NULL) / 60.0));
        m_gpCurrentGNSSPostion.m_longitude_rad =
            DEGtoRAD((((szLongitude[0] - '0') * 100 + (szLongitude[1] - '0') * 10 + (szLongitude[2] - '0')) + 
            strtod(&szLongitude[3], NULL) / 60.0));
        if (szNS[0] == 'S') {
            m_gpCurrentGNSSPostion.m_latitude_rad = - m_gpCurrentGNSSPostion.m_latitude_rad;
        }
        if (szEW[0] == 'W') {
            m_gpCurrentGNSSPostion.m_longitude_rad = - m_gpCurrentGNSSPostion.m_longitude_rad;
        }

        m_gpCurrentGNSSPostion.m_altitude_meters = fAltitude_m;

        // recalculate range and bearing to target, glideslope

        m_gcCurrentGNSSPostion = (GeocentricCoordinates) m_gpCurrentGNSSPostion;

        m_vNEDCurrentPosition_m = m_mTargetGCtoNED * (m_gcCurrentGNSSPostion - m_gcTarget);

        m_fTargetRange_m = m_vNEDCurrentPosition_m.Magnitude();

        m_fTargetBearing_rad = atan2(m_vNEDCurrentPosition_m.y, m_vNEDCurrentPosition_m.x);
        if (m_fTargetBearing_rad < 0.0) {
          m_fTargetBearing_rad = 2.0 * M_PI + m_fTargetBearing_rad;
        }

    }
}

static void handleVTG() {
    g_pLogParser->HandleVTG();
}

void LogParser::HandleVTG() {
    float fTrack_degTrue;
    float fGroundspeed_kph;
    char szT[2];
    char szK[2];
    m_parser.getArg(0, fTrack_degTrue);
    m_parser.getArg(1, szT);
    m_parser.getArg(6, fGroundspeed_kph);
    m_parser.getArg(7, szK);

    m_dGroundTrackCourse_rad = DEGtoRAD(fTrack_degTrue);
    m_dCurrentGroundspeed_mps = fGroundspeed_kph * (1000.0f / 3600.0f);

    // TODO: recalculate range and bearing to target, glideslope

}

LogParser::LogParser() {
    g_pLogParser = this;
    
    m_parser.addHandler("PCLOS", handlePCLOSE);
    m_parser.addHandler("PIMU", handleIMU);
    m_parser.addHandler("PENV", handleENV);
    m_parser.addHandler("GNGGA", handleGGA);
    m_parser.addHandler("GNVTG", handleVTG);
    m_parser.setHandleCRC(false);

    unsigned long m_ulLastReceived_ms = 0;

    m_fRateOfDescent_mps = 0.0;

    m_dBarometricAltitudeMSL_m = 0;
    m_fTargetRange_m = 0;
    m_fTargetBearing_rad = 0.0;  // North = 0.0; East = pi/2
    m_fGlideSlope_dots = 0.0;

    m_gpCurrentGNSSPostion.m_latitude_rad = 0;
    m_gpCurrentGNSSPostion.m_longitude_rad = 0;
    m_gpCurrentGNSSPostion.m_altitude_meters = 0;

    m_bLocationValid = false;
    m_bTargetValid = false;
    m_bGlideSlopeValid = false;

    m_nSatsVisible = 0;

    m_fMaxAcceleration_mps2 = 0;
    m_fCurrentAcceleration_mps2 = 0;

    m_fSurfaceWindSpeed_mps = 0;
    m_fSurfaceWindDirection_rad = 0; // Direction wind is "from"

    /*
     * Idealized canopy performance based on estimated (or past) history of
     * canopy type, wing loading, atmospheric conditions, target altitude.
     */
    m_fIdealizedRateOfDescent_mps = 0;
    m_fIdealizedForwardDrive_mps = 0;
}
        
LogParser::~LogParser() {
}

void LogParser::ProcessIncomingStreamBytes( const char * pBuffer, int nSize) {
    const char * p = pBuffer;

    m_ulLastReceived_ms = millis();
    
    while(nSize-- > 0) {
        m_parser << *p++;
    }
}

void LogParser::SetLandingTarget( GeodeticPosition &gpLocation) {
    m_gcTarget = (GeocentricCoordinates) gpLocation;
    m_mTargetGCtoNED = gpLocation.ToNEDTransform();
    m_bTargetValid = true;
}

void LogParser::SetSurfaceWind( float fDirectionFrom_rad, float fWindVelocity_mps) {
  m_fSurfaceWindSpeed_mps = fWindVelocity_mps;
  m_fSurfaceWindDirection_rad = fDirectionFrom_rad;

  // AS = GS + WS
  m_vNEDSurfaceWind_mps.x = - fWindVelocity_mps * cos(fDirectionFrom_rad);
  m_vNEDSurfaceWind_mps.y = - fWindVelocity_mps * sin(fDirectionFrom_rad);
}

void LogParser::UpdateTargeting() {
  /*
   * Glideslope computation
   *
   * This code is acting as an experimental placeholder for now. The indications
   * it generates should not be relied upon for actual guidance or navigation.
   * 
   * Here's a summary of the algorithm as it stands today:
   * 
   * 1) Convert GNSS-reported altitude to AGL altitude using target Z-value as the reference ground level.
   * 2) Estimate sink rate from time history of GNSS-reported altitude.
   * 3) Update Time-to-Go (TGo) until landing based on AGL altitude and estimated sink rate values.
   * 4) Use current VTG-reported horizontal velocity to compute distance travelled (X_m) in TGo seconds
   * 5) Subtract current horizontal distance to target from X_m. A positive value indicates "landing long"
   * 6) Compute number of dots to offset the indexer: positive deflection "above" the zero index and this
   *    corresponds to landing long; each dot represents a 30m error on landing.
   * 
   * This technique will yield wildly inaccurate guidance on the downwind leg. One approach to addresing that
   * will be to use a different estimate for groundspeed on the downwind leg -- for example, we might use
   * estimated canopy forward drive and average winds to compute targeting during the downwind and base legs.
   * This remains to be implemented.
   */

  if ( m_bLocationValid && m_bTargetValid ) {

    double dAltitude_mAGL = - m_vNEDCurrentPosition_m.z;
    // Unfiltered, for now; moreover, we'll currently just use the time of arrival of each report at
    // this processor as the time measure to compute the rate of descent.  
    // TODO: it might be more accurate to compute this as the sensor information is first processed.
    double dDescentRate_mps = (dAltitude_mAGL - m_dLastAltitude_mAGL) /
      ( 1000.0 * (m_ulTimeOfThisPositionReport_ms - m_ulTimeOfLastPositionReport_ms));
    
    m_ulTimeOfLastPositionReport_ms = m_ulTimeOfThisPositionReport_ms;
    m_dLastAltitude_mAGL = dAltitude_mAGL;

    fTGo_sec = dAltitude_mAGL / dDescentRate_mps;

    m_bGlideSlopeValid = false;

    if ( fTGo_sec > 0.0 ) {

      double dX_m = m_dCurrentGroundspeed_mps * fTGo_sec;

      double dTargetDist_m = sqrt( m_vNEDCurrentPosition_m.x * m_vNEDCurrentPosition_m.x +
                                   m_vNEDCurrentPosition_m.y * m_vNEDCurrentPosition_m.y );

      m_fGlideSlope_dots = (dX_m - dTargetDist_m) / METERSperDOT;

      m_bGlideSlopeValid = true;
    }
  }
  else {
    m_bGlideSlopeValid = false;
  }

  /*
   * Altitude Display
   */

}
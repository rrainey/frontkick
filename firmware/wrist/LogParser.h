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
#ifndef LOG_PARSER_H
#define LOG_PARSER_H

#include <Arduino.h>
#include "NMEAParserEx.h"
#include "Geodesy.h"

using namespace Geodesy;

#define METERSperDOT    30.0  // landing distance error for each "dot"

/**
 * Maintain state of displayable information by sniffing the log stream received from
 * a sensor pack.
 * 
 * This implementation relies on the Arduino NMEAParser class to parse the incoming packets.
 * NMEAParser requires us to use s set of static functions separate from the class to handle
 * each NMEA/Log record type. Straightforward if not elegant.
 *
 * $GL--- - GLONASS
 * $GP--- - GPS
 * $GA--- - Galileo
 * $GN--- - combined (preferred)

 * $PVER
 * $PIMU
 * $PENV
 * $PTH
 * $PCLOS
 *
 * $G*GGA - used
 * $G*VTG - used
 * $G*RMC
 * $G*GLL
 * $GNGSA
 * $G*GSV
 * $G*TXT
 */
class LogParser {
    public:
        LogParser();
        virtual ~LogParser();

        /**
         * Examine the incoming log data stream updating relevant state as we do.
         */
        void ProcessIncomingStreamBytes( const char * pBuffer, int nSize);

        /**
         * Set the desired landing location
         */
        void SetLandingTarget( GeodeticPosition &gpLocation);

        /**
         * Set avarage surface winds
         */
        void SetSurfaceWind( float fDirectionFrom_rad, float fWindVelocity_mps);

        /// @brief Perform housekeeping related to updating targeting information
        void UpdateTargeting();

        /// @brief Device "millis()" timestamp of last received Bluetooth packet
        unsigned long m_ulLastReceived_ms;
        /// @brief GNSS-based straight-line (not across ground) range to landing target
        float m_fTargetRange_m;
        /// @brief Bearing from current position to target, relative to current ground travel path
        float m_fTargetBearing_rad;
        /// @brief Current compulted glide slope reading (negative readings below index mark)
        float m_fGlideSlope_dots;

        // Information from PENV records: barometric altitude
        unsigned long m_ulLastEnvReceived_ms;
        /// @brief rate of descent derived from baromentric pressure changes
        float m_fRateOfDescent_mps;
        /// @brief current MSL barometric alititude (based on standard artmosphere conditions)
        double m_dBarometricAltitudeMSL_m;
        /// @brief current AGL barometric altitude
        double m_dBarometricAltitudeAGL_m;

        /// @brief true when we have a usable GNSS location fix
        bool m_bLocationValid;
        /// @brief true once we have a velid target landing location
        bool m_bTargetValid;
        /// @brief true iff we have all precondiction met to compute a glide slope indicator
        bool m_bGlideSlopeValid;

        /// @brief Current reported GNSS WGS-84 position
        GeodeticPosition m_gpCurrentGNSSPostion;

        /// @brief Current reported position in ECF Coordinates (meters)
        GeocentricCoordinates m_gcCurrentGNSSPostion;

        double m_dGroundTrackCourse_rad;
        double m_dCurrentGroundspeed_mps;
        unsigned long m_ulTimeOfThisPositionReport_ms;
        unsigned long m_ulTimeOfLastPositionReport_ms;
        double m_dLastAltitude_mAGL;

        /// @brief Current reported position in Target NED frame (meters)
        Vector3d m_vNEDCurrentPosition_m;

        /// @brief WGS-84 landing target location
        GeocentricCoordinates m_gcTarget;

        /// @brief Transform matrix go rotate Geocentric corrdinates into Target NED frame
        Matrix4d m_mTargetGCtoNED;

        /// @brief Current number of visible satellites reported by GNSS
        int m_nSatsVisible;

        float m_fMaxAcceleration_mps2;
        float m_fCurrentAcceleration_mps2;

        /* 
         * Estimated "average" surface wind conditions
         */
        float m_fSurfaceWindSpeed_mps;
        float m_fSurfaceWindDirection_rad; // Direction wind is "from"; North = 0
        /// @brief NED vector representing surface wind field (subtract this from NED velocity to get apparent airspeed)
        Vector3d m_vNEDSurfaceWind_mps;

        /*
         * Idealized canopy performance based on estimated (or past) history of
         * canopy type, wing loading, (TODO: atmospheric conditions, target altitude)
         */
        float m_fIdealizedRateOfDescent_mps;
        float m_fIdealizedForwardDrive_mps;

        /// @brief estimated time-to-go before landing (upper bound)
        float fTGo_sec;

        NMEAParserEx<10> m_parser;

    public:
        void HandleGGA();
        void HandleVTG();
        void HandleIMU();
        void HandleENV();

};

#endif

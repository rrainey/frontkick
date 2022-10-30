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
#ifndef LOGGERCORE_H
#define LOGGERCORE_H

#define SERIAL_BUFFER_SIZE 512

#include <Arduino.h>
//#include <Adafruit_DPS310.h>
//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
#include <Zanshin_BME680.h>
// Library here: // see https://github.com/bolderflight/bmi088-arduino
#include "./BMI088.h"

#include <Wire.h>
//Library here: http://librarymanager/All#SparkFun_u-blox_GNSS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
//http://librarymanager/All#MicroNMEA
#include <MicroNMEA.h> 
#include <Print.h>

#ifndef M_PI
const double M_PI   = 3.14159265358979323846;
const double M_PI_2 = 1.57079632679489661923;
const double M_PI_4 = 0.78539816339744830962;
#endif

#if !defined (DEGtoRAD)
#define DEGtoRAD(theta) ((theta) * M_PI / 180.0)
#define RADtoDEG(theta) ((theta) * 180.0 / M_PI)
#endif

#define APP_STRING  "Frontkick, version 0.57"
#define LOG_VERSION 1
#define NMEA_APP_STRING "$PVER,\"Frontkick, version 0.57\",57"

/*
 * Last built with Arduino IDE version 2.0.1; also builds with arduino-cli
 * 
 * Change History affecting sensor output and log file contents
 *
 * Version 57:
 *    Add estimated DZ field elevation, expressed as both air pressure (hPa) and MSL altitude (feet) to $PVER
 *    Add NMEA-style checksums to all sentences
 *
 * Version 56:
 *    Add $PCLOS directive to direct the logging function to close a completed log file
 * 
 * Version 55:
 *    Disable NMEA satellite status sentences when in higher GNSS position polling rates
 *    
 * Version 54:
 *    Switch to 5 Hz GNSS message rate when in freefall till landing.
 * 
 * Version 53:
 *    u-blox Dynamic Platform Mode now set to Airborne 2g 
 *    (see pg. 21 of https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf)
 *    Logging GNSS fix rate raised from 2 to 4 fixes per second.
 *    millis() times appearing in a log file are now relative to the start of logging for the file
 *    Boot initialization now only waits for 20 seconds for a USB connection (was 30 seconds)
 */

/*
 * I2C peripheral addresses for the V2-SAM and V3-SAM PCBs as well as Frontkick
 */
#define GPS_I2C_ADDR          0x42
#define DPS310_I2C_ADDR       0x76  // only present on dropkick
#define MPU6050_I2C_ADDR      0x69  // only present on dropkick
#define BMI680_I2C_ADDR       0x76  // only present on frontkick
#define BMI088_ACCEL_I2C_ADDR 0x18  // only present on frontkick
#define BMI088_GYRO_I2C_ADDR  0x68  // only present on frontkick

/*
 * Operating mode
 */
#define OPS_FLIGHT        0  // normal mode; altimeter used to detect motion
#define OPS_STATIC_TEST   1  // for testing; time based simulation of vertical motion (preferred test mode)
#define OPS_GROUND_TEST   2  // for testing; uses GPS horizontal movement as an analogue to altitude changes

/*
 * Set operating mode for this build of the firmware here. If using arduino-cli to compile, we can
 * define OPS_MODE on the command line to get a build suitable for integration testing.
 */
#ifndef OPS_MODE
#define OPS_MODE OPS_FLIGHT 
#endif

#define TEST_SPEED_THRESHOLD_KTS     6.0
#define OPS_HDOT_THRESHOLD_FPM       300
#define OPS_HDOT_LAND_THRESHOLD_FPM  100
#define OPS_HDOT_JUMPING_FPM         -800

/*
 * Minutes to milliseconds
 */
#define MINtoMS(x) ((x) * 60 * 1000)

/*
 * Set up timers
 * 
 * Timer 1: used in landing state machine (OFF initially)
 * 
 * Timer 2: periodic check of battery state
 * 
 * Timer 3: blink controller for RED LED  (OFF initially)
 * 
 * Timer 4: IMU sensor logging interval timer  (OFF initially)
 * 
 * Timer 5: Periodic SD-card log file flushing  (OFF initially)
 */

#define TIMER1_INTERVAL_MS 30000

#define TIMER2_INTERVAL_MS 30000

#define TIMER3_ON_INTERVAL_MS     750
#define TIMER3_OFF_INTERVAL_1_MS  750 // off interval when signaling battery low
#define TIMER3_OFF_INTERVAL_2_MS  (3000 - TIMER3_ON_INTERVAL_MS) // off interval for flight mode

#define TIMER4_INTERVAL_MS 5         // 100Hz BMI088 data rate used here; poll FIFOs at 200Hz

#define TIMER5_INTERVAL_MS 10000

// Altimeter subsystem
#define NUM_H_SAMPLES 5
#define SEALEVELPRESSURE_HPA (1013.25)

#define STATE_WAIT       0
#define STATE_IN_FLIGHT  1
#define STATE_JUMPING    2
#define STATE_LANDED_1   3
#define STATE_LANDED_2   4

#define BLINK_STATE_OFF     0
#define BLINK_STATE_LOGGING 1
#define BLINK_STATE_BATTERY 2

/*
 * LiPoly battery is rated at 3.7V
 */
#define LOWBATT_THRESHOLD 3.55

/*
 * The LoggerCore includes a simple subsystem used to estimate the field elevation at the dropzone.
 * It does that by periodically taking an altitude reading when otherwise idle. (The sample period is
 * associated with the battery monitor timer, which runs every 30 seconds).  When a flight up to altitude
 * starts, the subsytem uses a reading from a few minutes back and reports that as the field elevation.
 */
#define NUM_ALT_SAMPLES 10

class LoggerCore : public Print {
  public:
    LoggerCore();
    virtual ~LoggerCore() {};
    
  protected:

    void updateStateMachine();

    void updateHDot(float H_feet);

    /*
     * Data gathering and reporting is controlled via a state machine
     */
    int m_nAppState;

    /*
      * Records last millis() time when timers were updated in
      * the main loop.
      */
    uint32_t m_lastTime_ms = 0;

    SFE_UBLOX_GNSS m_GNSS;

    // Bosch BMI680 Gas Sensor (used for air pressure/termperature)
    // see https://github.com/Zanduino/BME680
    BME680_Class m_bme680;

    // Bosch BLI088 IMU sensors
    //Bmi088Accel * m_pbmi088Accel;
    //Bmi088Gyro  * m_pbmi088Gyro;

    BMI088 m_bmi088;

    /*
      * I2C connection to the MPU-6050 IMU
      */
    //Adafruit_MPU6050 m_mpu;

    //Adafruit_DPS310 m_dps;

    bool m_bTimer1Active;
    int32_t m_timer1_ms;
    bool m_bTimer2Active;
    int32_t m_timer2_ms;
    bool m_bTimer3Active;
    int32_t m_timer3_ms;
    bool m_bTimer4Active;
    int32_t m_timer4_ms;
    bool m_bTimer5Active ;
    int32_t m_timer5_ms;

    int m_nHSample[NUM_H_SAMPLES];
    int m_nHDotSample[NUM_H_SAMPLES];
    int m_nNextHSample = 0;

    bool m_bIMUPresent;

    int m_redLEDState;

    bool m_bBatteryAlarm;

    float m_fMeasuredBattery_volts;

    int m_nCurAltitudeSample;
    float m_fMeasuredPressure_hPa[NUM_ALT_SAMPLES];
    int m_nMeaasuredAltitude_ftMSL[NUM_ALT_SAMPLES];

    int m_nBlinkState;

    /*
    * Estimated MSL altitude, based on standard day pressure @ sea level
    */
    int m_nH_feet;

    /*
    * Estimated rate of climb (fpm)
    */
    int m_nHDot_fpm;
    /*
    * Estimated ground elevation, ft
    * 
    * Computed while in WAIT state.
    */
    int m_nHGround_feet;

    boolean m_bFirstPressureSample;

    uint32_t m_ulLastHSampleMillis;
    uint32_t m_ulLogfileOriginMillis;
    bool m_bStreamConnected;
    bool m_bLogActive;

  public:
    /*
     * Mark the start of a new jump log
     */
    void StartLogStream();

    /// @brief Compute NMEA-style checksum and append it to the string
    void AddNMEAChecksum(char *pszSentence, unsigned char *pszResult);

    void FlushLogStream() {};
    /*
     * Mark the end of a jump log
     */
    void EndLogStream();

    /// @brief Get logging status
    /// @return returns true any time we are logging
    bool IsLogActive(void) {
      return m_bLogActive;
    }
    bool IsStreamConnected() { return m_bStreamConnected; };
    void SetStreamConnectionState( bool bValue ) { m_bStreamConnected = bValue; }
    void StartLogFileFlushing() {};

    /// @brief Sample air pressure, altitude, and log it if logging is active
    void SampleAndLogAltitude();

    uint32_t GetLogfileOriginMillis() const { return m_ulLogfileOriginMillis; }
    void SetLogfileOriginMillis(uint32_t value) { m_ulLogfileOriginMillis = value; }    

    void Initialize();
    void Loop();
    void Shutdown();

    void initializeIMUSampling();

    void shutdownIMUSampling();

    /*
     * Intended to be overriden in whatever class derives from LoggerCore
     */
    size_t write(const uint8_t *buffer, size_t size) { return 0; };
    /*
     * Artifact of Print.h/cpp - not used here.
     */
    size_t write(uint8_t i) { return 0; }

    int computAvgHDot();

    void SampleIMU();

    void SetBlinkState(int nState);

};

#endif

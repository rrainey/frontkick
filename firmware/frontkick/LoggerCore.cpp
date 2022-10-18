#include "LoggerCore.h"

#define VBATPIN       PIN_VBAT
#define RED_LED       LED_RED
//#define GREEN_SD_LED   8

#define VBAT_MV_PER_LSB   (0.73242188F)
#ifdef NRF52840_XXAA
#define VBAT_DIVIDER      (0.5F)          // 150K + 150K voltage divider on VBAT
#define VBAT_DIVIDER_COMP (2.0F)          // Compensation factor for the VBAT divider
#else
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider
#endif

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// For debugging
static bool g_bPrintNMEA = false;

static LoggerCore * g_pLoggerCoreSingleton = NULL;

float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {
  /*!
  @brief     This converts a pressure measurement into a height in meters
  @details   The corrected sea-level pressure can be passed into the function if it is known,
             otherwise the standard atmospheric pressure of 1013.25hPa is used (see
             https://en.wikipedia.org/wiki/Atmospheric_pressure) for details.
  @param[in] press    Pressure reading from BME680
  @param[in] seaLevel Sea-Level pressure in millibars
  @return    floating point altitude in meters.
  */
  static float Altitude;
  Altitude =
      44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));
  return (Altitude);
} 

/**
 * see https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/nrf52-adc
 */
static float readVBAT(void) {

  float raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBATPIN);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * REAL_VBAT_MV_PER_LSB;
}

LoggerCore::LoggerCore() {

  m_lastTime_ms = 0;
  m_bIMUPresent = false;

  m_redLEDState = LOW;
  m_nBlinkState = BLINK_STATE_OFF;

  m_bTimer1Active = false;
  m_timer1_ms = 0;
  m_bTimer2Active = true;
  m_timer2_ms = TIMER2_INTERVAL_MS;
  m_bTimer3Active = false;
  m_timer3_ms = 0;
  m_bTimer4Active = false;
  m_timer4_ms = 0;
  m_bTimer5Active = false;
  m_timer5_ms = 0;

  m_nCurAltitudeSample = 0;

  m_nH_feet = 0;
  m_nHDot_fpm = 0;
  m_nHGround_feet = 0;
  m_bFirstPressureSample = true;

  m_bStreamConnected = false;

  m_bLogActive = false;

  m_ulLastHSampleMillis = 0;
  m_ulLogfileOriginMillis = 0;

  g_pLoggerCoreSingleton = this;

  g_bPrintNMEA = false;

}

void LoggerCore::Initialize() {
  int status;

  pinMode(RED_LED, OUTPUT);
  m_redLEDState = LOW;
  digitalWrite(RED_LED, m_redLEDState);

  m_bBatteryAlarm = false;
  m_nBlinkState = BLINK_STATE_OFF;

  if (m_bmi088.isConnection()) {
    /*
    m_bmi088.resetAcc();
    delay(5);
    m_bmi088.resetGyro();
    delay(5);
    */
    m_bmi088.initialize();
    m_bmi088.setAccOutputDataRate(ODR_50);
  }
  else {
    Serial.println("BMI088 is not connected");
  }

  Serial.println("IMU Initialized");

  //pNMEA = incomingNMEA;

  if (m_GNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
  }

  Serial.println(F("GNSS Detected"));

  // Disable unused output channels
  
  m_GNSS.setUART1Output(0);
  m_GNSS.setUART2Output(0);

  //m_GNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
  m_GNSS.setI2COutput(COM_TYPE_NMEA);
  m_GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  // Idle reporting will be at 0.5 Hz
  m_GNSS.setMeasurementRate(2000);
  m_GNSS.setNavigationRate(1);

  if (m_GNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g) == false) 
  {
    Serial.println(F("*** Warning: setDynamicModel failed"));
  }
  else
  {
    Serial.println(F("GNSS Dynamic Platform Model set to AIRBORNE2g"));
  }

  //This will pipe all NMEA sentences to the serial port so we can see them
  //m_GNSS.setNMEAOutputPort(Serial);

  m_GNSS.setNavigationFrequency(1);

  m_bIMUPresent = true;


  while (!m_bme680.begin(I2C_FAST_MODE)) {
    Serial.print(F("-  Unable to find m_bme680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  // of loop until device is located
  Serial.print(F("- Setting 16x oversampling for all sensors\n"));
  delay(200);
  m_bme680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  m_bme680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  m_bme680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  Serial.print(F("- Setting IIR filter to a value of 4 samples\n"));
  m_bme680.setIIRFilter(IIR4);  // Use enumerated type values
  Serial.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));
  m_bme680.setGas(320, 150);  // 320�c for 150 milliseconds

}

void LoggerCore::EndLogStream() {
  char szBuffer[128];
  unsigned char ucChecksum;

  sprintf( szBuffer, "$PCLOS" );
  AddNMEAChecksum( szBuffer, &ucChecksum );
  println( szBuffer );

  m_bLogActive = false;
};

void LoggerCore::StartLogStream() { 
      /*
      print( NMEA_APP_STRING );
      m_bLogActive = true;

      // use altitude sample from two minutes back
      int nSample = m_nCurAltitudeSample - 4;
      if (nSample < 0) {
        nSample += NUM_ALT_SAMPLES;
      }
      print( "," );
      print( m_fMeasuredPressure_hPa[nSample] );
      print( "," );
      println( m_nMeaasuredAltitude_ftMSL[nSample] );
      */

      m_bLogActive = true;

      // use altitude sample from two minutes back
      int nSample = m_nCurAltitudeSample - 4;
      if (nSample < 0) {
        nSample += NUM_ALT_SAMPLES;
      }

      char szBuffer[128];
      unsigned char ucChecksum;

      sprintf( szBuffer, "%s,%.3f,%d", NMEA_APP_STRING, 
               m_fMeasuredPressure_hPa[nSample], m_nMeaasuredAltitude_ftMSL[nSample] );
      AddNMEAChecksum( szBuffer, &ucChecksum );
      println( szBuffer );
    };

void LoggerCore::Loop() {

  uint32_t curTime_ms = millis();

  uint32_t deltaTime_ms = curTime_ms - m_lastTime_ms;

  if ( deltaTime_ms > 0 ) {

    /*
     * Update active timer countdowns
     */
    if (m_bTimer1Active) {
      m_timer1_ms -= deltaTime_ms;
    }
  
    if (m_bTimer2Active) {
      m_timer2_ms -= deltaTime_ms;
    }
  
    if (m_bTimer3Active) {
      m_timer3_ms -= deltaTime_ms;
    }
  
    if (m_bTimer4Active) {
      m_timer4_ms -= deltaTime_ms;
    }

    if (m_bTimer5Active) {
      m_timer5_ms -= deltaTime_ms;
    }
    
    m_lastTime_ms = curTime_ms;

  }

  m_GNSS.checkUblox();


  /*
   * Processing tasks below are outside of the
   * GPS NMEA processing loop.
   */

  /*
  if (OPS_MODE == OPS_GROUND_TEST) {
    //updateTestStateMachine();
  }
  else {
    updateFlightStateMachine();
  }
  */

  updateStateMachine();

  SampleAndLogAltitude();

  /*
   * RED LED Blink Logic
   * 
   * The RED LED will blink using different patterns to indicate
   * one of three states: Constant off, ON/OFF at 1.5Hz to indicates a low battery.
   * A 3-second blink is used to indicate flight mode.
   */
  if (m_bTimer3Active && m_timer3_ms <= 0) {
    
    m_redLEDState = (m_redLEDState == HIGH) ? LOW : HIGH;
    digitalWrite(RED_LED, m_redLEDState);

    if ( m_redLEDState == HIGH ) {
      m_timer3_ms = TIMER3_ON_INTERVAL_MS;
    }
    else {
      m_timer3_ms = m_bBatteryAlarm ? TIMER3_OFF_INTERVAL_1_MS : TIMER3_OFF_INTERVAL_2_MS;
    }
    
  }

  /*
   * Every 30 seconds, measure the battery state.
   * Blink red LED if low.
   */
  if (m_bTimer2Active && m_timer2_ms <= 0) {
    
    m_timer2_ms = TIMER2_INTERVAL_MS;
    
    m_fMeasuredBattery_volts = readVBAT();

    if (m_nCurAltitudeSample++ == NUM_ALT_SAMPLES) {
      m_nCurAltitudeSample = 0;
    }

    /*
     *  Maintain a history of altitude samples. This is used to have an estimate of the
     *  MSL altitude corresponding to ground level.  The Bosch data sheet for the BME088
     *  indicates that the altitude margin for error is +/-3.2 ft (+/-1 meter).
     */
    int32_t  temp, humidity, pressure, gas; 

    m_bme680.getSensorData( temp, humidity, pressure, gas );
    // Reported values do not take into account how high off the ground the unit is
    // positioned when sampling (likely at least chest height)
    m_fMeasuredPressure_hPa[m_nCurAltitudeSample] = pressure;
    m_nMeaasuredAltitude_ftMSL[m_nCurAltitudeSample] = (int) (altitude(pressure) * 3.28084f);

    if ( m_fMeasuredBattery_volts <= LOWBATT_THRESHOLD ) {
      m_bBatteryAlarm = true;
      SetBlinkState ( BLINK_STATE_BATTERY );
    }
    else {
      if ( m_bBatteryAlarm ) {
        SetBlinkState( (m_nAppState != STATE_WAIT) ? BLINK_STATE_LOGGING : BLINK_STATE_OFF );
      }
      m_bBatteryAlarm = false;
    }
  }

  /*
   * Log BME sensor information
   */
  if (m_bTimer4Active && m_timer4_ms <= 0) {
 
    SampleIMU();

    m_timer4_ms = TIMER4_INTERVAL_MS;
  }

}

void Shutdown() {

}

void LoggerCore::updateStateMachine() {

  /**
   * State machine appropriate for flight
   */
  switch ( m_nAppState ) {

  case STATE_WAIT:
    if (m_nHDot_fpm > OPS_HDOT_THRESHOLD_FPM) {

      Serial.println("Switching to STATE_IN_FLIGHT");
      
      // open log file
      //generateLogname( logpath );
      //logFile = SD.open( logpath, FILE_WRITE );

      StartLogStream();

      // Activate altitude / battery sensor logging
      m_bTimer4Active = true;
      m_timer4_ms = TIMER4_INTERVAL_MS;

      // Activate periodic log file flushing
      StartLogFileFlushing();

      // Activate "in flight" LED blinking
      SetBlinkState ( BLINK_STATE_LOGGING );

      // Set "time 0" for log file.
      m_ulLogfileOriginMillis = millis();
      
      m_nAppState = STATE_IN_FLIGHT;
    }
    break;

  case STATE_IN_FLIGHT:
    {
      if (m_nHDot_fpm <= OPS_HDOT_JUMPING_FPM) {
        Serial.println("Switching to STATE_JUMPING");
        m_nAppState = STATE_JUMPING;

        // set nav update rate to 4Hz
        m_GNSS.setMeasurementRate(250);
        m_GNSS.setNavigationRate(1);

        m_GNSS.disableNMEAMessage( UBX_NMEA_GSA, COM_PORT_I2C );
        m_GNSS.disableNMEAMessage( UBX_NMEA_GSV, COM_PORT_I2C );
      }
    }
    break;

  case STATE_JUMPING:
    {
      if (labs(m_nHDot_fpm) <= OPS_HDOT_LAND_THRESHOLD_FPM) {
        Serial.println("Switching to STATE_LANDED_1");
        m_nAppState = STATE_LANDED_1;
        m_timer1_ms = TIMER1_INTERVAL_MS;
        m_bTimer1Active = true;
      }
    }
    break;

  case STATE_LANDED_1:
    {
      if (m_nHDot_fpm <= OPS_HDOT_JUMPING_FPM) {
        Serial.println("Switching to STATE_JUMPING");
        m_nAppState = STATE_JUMPING;
        m_bTimer1Active = false;
      }
      else if (labs(m_nHDot_fpm) >= OPS_HDOT_THRESHOLD_FPM) {
        Serial.println("Switching to STATE_IN_FLIGHT");
        m_nAppState = STATE_IN_FLIGHT;
        m_bTimer1Active = false;
      }
      else if (m_bTimer1Active && m_timer1_ms <= 0) {

        // Back to 0.5Hz update rate
        m_GNSS.setMeasurementRate(2000);
        m_GNSS.setNavigationRate(1);

        m_GNSS.enableNMEAMessage( UBX_NMEA_GSA, COM_PORT_I2C );
        m_GNSS.enableNMEAMessage (UBX_NMEA_GSV, COM_PORT_I2C );
        
        m_bTimer4Active = false;
        Serial.println("Switching to STATE_WAIT");
        SetBlinkState ( BLINK_STATE_OFF );
        m_nAppState = STATE_WAIT;
        m_bTimer1Active = false;

        FlushLogStream();
        EndLogStream();
      }
    }
    break;
  }
}

char incomingNMEA[256];
char *pNMEA = incomingNMEA;
bool bStartOfNMEA = true;

/*
  * Records last millis() time for start of NMEA sentence arrival. 
  * Useful to sync millis() time with GPS clock.
  */
uint32_t lastNMEATime_ms = 0;

void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  /*
   * New sentence arriving? record the time
   */
  if (bStartOfNMEA) {
    lastNMEATime_ms = millis() - g_pLoggerCoreSingleton->GetLogfileOriginMillis();
    bStartOfNMEA = false;
  }
  *pNMEA++ = incoming;

  nmea.process(incoming);

  if (incoming == '\n') {

    *pNMEA++ = '\0';
    
    if (g_pLoggerCoreSingleton->IsLogActive()) {
      
      g_pLoggerCoreSingleton->print( incomingNMEA );

      /*
       * Include time hack for important GNSS messages.  This is
       * designed to allow us to correlate GPS time and millis() time in the output stream.
       */
      if (strncmp( incomingNMEA+3, "GGA", 3) == 0 || strncmp( incomingNMEA+3, "GLL", 3) == 0) {
        char szBuffer[128];
        unsigned char ucChecksum;

        sprintf( szBuffer, "$PTH,%u", millis() - g_pLoggerCoreSingleton->GetLogfileOriginMillis() );
        g_pLoggerCoreSingleton->AddNMEAChecksum( szBuffer, &ucChecksum );
        g_pLoggerCoreSingleton->println( szBuffer );
      }
      
      g_pLoggerCoreSingleton->FlushLogStream();
    }
  
    if ( g_bPrintNMEA ) {
      Serial.print( incomingNMEA );
    }

    pNMEA = incomingNMEA;
    bStartOfNMEA = true;
  }
}

void LoggerCore::SampleIMU() {

  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  int16_t nTemp = 0;

  if (m_bIMUPresent) {

    m_bmi088.getAcceleration(&ax, &ay, &az);
    m_bmi088.getGyroscope(&gx, &gy, &gz);
    nTemp = m_bmi088.getTemperature();

#ifdef notdef
    Serial.print(g.gyro.x); // rad per sec
    Serial.print(",");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print(g.gyro.z);
    Serial.println();
#endif
  
    if (IsLogActive()) {
      /*
      print("$PIMU,");
      print(millis() - m_ulLogfileOriginMillis);
      
      print(",");
      print(ax);  // m/s^2
      print(",");
      print(ay);
      print(",");
      print(az);
      
      print(",");
      print(gx); // rad per sec
      print(",");
      print(gy);
      print(",");
      print(gz);
      println();
      */

      char szBuffer[128];
      unsigned char ucChecksum;

      sprintf( szBuffer, "$PIMU,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
               millis() - m_ulLogfileOriginMillis, ax, ay, az, gx, gy, gz );
      AddNMEAChecksum( szBuffer, &ucChecksum );
      println( szBuffer );
    }
  }
}

void LoggerCore::SampleAndLogAltitude()
{
  double dAlt_ft;
  float fPressure_hPa;
  int32_t  temp, humidity, pressure, gas; 

  m_bme680.getSensorData(temp, humidity, pressure, gas);

  fPressure_hPa = pressure;
    
  //sensors_event_t temp_event, pressure_event;

  //if (dps.temperatureAvailable()) {
  //  dps_temp->getEvent(&temp_event);
    
    /*
    Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");
    Serial.println();
    */
    
  //}

  /*
   * Note: sampling pressure in dps also samples temperature.
   */
  //if (dps.pressureAvailable()) {

    //dps_pressure->getEvent(&pressure_event);

    //dPressure_hPa = pressure_event.pressure;

    if (OPS_MODE != OPS_STATIC_TEST) {

      //m_dAlt_ft = dps.readAltitude() * 3.28084;

      dAlt_ft = altitude(pressure) * 3.28084;

    }
    else {
      
      /*
       * Simulate interpolated altitude based on this schedule:
       * 
       * Time (min)     Alt(ft)
       *     0             600
       *     2             600
       *     12           6500
       *     13           6500
       *     14           3500
       *     17            600
       *     19            600 
       *     
       *     Values clamped at finish to last value.
       */

      struct _vals {
        float time_ms;
        int alt_ft;
      };

    struct _vals *p, *prev;
    
    /*
     * time and altitude readings for a idealized hop-n-pop
     */
    static struct _vals table[7] = {
      { MINtoMS(0),   600 },
      { MINtoMS(2),   600 },
      { MINtoMS(12), 6500 },
      { MINtoMS(13), 6500 },
      { MINtoMS(13.5), 3500 },
      { MINtoMS(16.5),  600 },
      { MINtoMS(19),    600 }
    };

    static int tableSize = sizeof(table)/sizeof(struct _vals);

     int t = millis();

     if (t >= table[tableSize-1].time_ms || t <= table[0].time_ms ) {
      dAlt_ft = 600.0;
     }
     else {
        int i;
        p = &table[0];
        for (i=1; i<tableSize-1; ++i) {
          prev = p;
          p = &table[i];
  
          if (t < p->time_ms) {
            dAlt_ft =  prev->alt_ft + (t - prev->time_ms) * (p->alt_ft - prev->alt_ft) / (p->time_ms - prev->time_ms);
            break;
          }
        }
     }

      fPressure_hPa = 1000.0;

      //g_atm.SetConditions( m_dAlt_ft, 0.0 );

      //pressure_event.pressure = 1000.0; //TODO hPA pressure
    }

    /*
     * Update based on estimated altitude
     */

    updateHDot(dAlt_ft);

    /*
     * Output a record
     */
    if (m_nAppState != STATE_WAIT) {
      /*
      print("$PENV,");
      print(millis() - m_ulLogfileOriginMillis);
      print(",");
      print(fPressure_hPa);
      print(",");
      print( dAlt_ft );
      print(",");
      println(m_fMeasuredBattery_volts);
      */

      char szBuffer[128];
      unsigned char ucChecksum;

      sprintf( szBuffer, "$PENV,%u,%.3f,%.1g,%.3f",
               millis() - m_ulLogfileOriginMillis, fPressure_hPa, dAlt_ft, m_fMeasuredBattery_volts );
      AddNMEAChecksum( szBuffer, &ucChecksum );
      println( szBuffer );
    
    }
    else {
      // When we're in WAIT mode, we can use the altitude
      // to set ground altitude.
      m_nHGround_feet = dAlt_ft;
    }
  //}
}


/**
 * Currently unused.
 */
int LoggerCore::computAvgHDot() {
  int i;
  int sum;
  for(i=0; i<NUM_H_SAMPLES; ++i) {
    sum += m_nHDotSample[i];
  }
  return sum / 5;
}

/**
 * Use pressure altitude samples to estimate rate of climb.
 * 
 * Rate of climb is re-estimated every 10 seconds.
 */
void LoggerCore::updateHDot(float H_feet) {

  uint32_t ulMillis = millis();
  int nLastHSample_feet;
  int nInterval_ms =  ulMillis - m_ulLastHSampleMillis;

  /* update HDot every ten seconds */
  if (nInterval_ms > 10000) {
    if (m_bFirstPressureSample == false) {
      if (m_nNextHSample == 0) {
        nLastHSample_feet = m_nHSample[NUM_H_SAMPLES-1];
      }
      else {
        nLastHSample_feet = m_nHSample[m_nNextHSample-1];
      }
      m_nHSample[m_nNextHSample] = H_feet;
      m_nHDotSample[m_nNextHSample] = (((long) H_feet - nLastHSample_feet) * 60000L) / nInterval_ms;
      m_nHDot_fpm = m_nHDotSample[m_nNextHSample];
    }
    else {
      m_bFirstPressureSample = false;
      m_nHSample[m_nNextHSample] = H_feet;
      m_nHDotSample[m_nNextHSample] = 0;
      m_nHDot_fpm = 0;
    }

    m_ulLastHSampleMillis = ulMillis;
    if (++m_nNextHSample >= NUM_H_SAMPLES) {
      m_nNextHSample = 0;
    }
  }
}

/**
 * Control RED (blinking) LED
 * This LED is locted to the left of the USB connector on
 * the Adalogger
 */
void LoggerCore::SetBlinkState( int newState ) {

  switch ( m_nBlinkState ) {

  case BLINK_STATE_OFF:
    // Was off, now on?
    if (newState != BLINK_STATE_OFF ) {
      m_bTimer3Active = true;
      m_timer3_ms = TIMER3_ON_INTERVAL_MS;
      m_redLEDState = HIGH;
    }
    break;
    
  case BLINK_STATE_LOGGING:
    if (newState == BLINK_STATE_BATTERY) {
      m_bTimer3Active = true;
      m_timer3_ms = TIMER3_ON_INTERVAL_MS;
      m_redLEDState = HIGH;
    }
    else if (newState == BLINK_STATE_OFF) {
        m_bTimer3Active = false;
        m_redLEDState = LOW;
     }
     break;
     
  case BLINK_STATE_BATTERY:
    if (newState == BLINK_STATE_OFF) {
      m_bTimer3Active = false;
      m_redLEDState = LOW;
    }
    else {
      // update state, but let blinking logic handle the transition
    }
    break;
  }

  // Update state and LED
  m_nBlinkState = newState;
  digitalWrite( RED_LED, m_redLEDState );
}

void LoggerCore::AddNMEAChecksum(char *pszSentence, unsigned char *pucResult) {
  unsigned char ucChecksum = 0;
  char * p = pszSentence + 1; // skip leading '$'
  while (*p) {
    unsigned char ucByte = (unsigned char) *p++;
    ucChecksum ^= ucByte;
  }
  // append NMEA representation of checksum to the senntence '*XX'
  sprintf(p, "*%02x", (int) ucChecksum);
  *pucResult = ucChecksum;
}


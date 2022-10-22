
#include <Wire.h>
#include "BMI088.h"

BMI088 m_bmi088;

unsigned long g_ulStart_ms;
unsigned long g_ulTotalBytes = 0;
unsigned long g_ulTotalGyroBytes = 0;
unsigned long g_ulTotalLoops = 0;
unsigned long bucket[7] = {0,};

void setup()
{
  g_ulStart_ms = millis();

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

#if 1
  // Blocking wait for connection when debug mode is enabled via IDE
  while ( !Serial && ((millis() - g_ulStart_ms) < 10000)) { delay(5); yield(); };
#endif
  
  Serial.println("SE01 booting");
  Serial.println("------------\n");

  if (m_bmi088.isConnection()) {

    uint16_t n;

    /*
     *-----------------------------------------------------
     *
     * Start SELF TESTS
     * 
     */

    // Powerdown is required to perform an I2C soft reset
    m_bmi088.setAccPoweMode(ACC_SUSPEND);
    m_bmi088.setGyroPoweMode(GYRO_DEEP_SUSPEND);

    // Soft reset
    delay(5);
    m_bmi088.resetAcc();
    delay(5);
    m_bmi088.resetGyro();
    delay(60);

    // power back up
    m_bmi088.setAccPoweMode(ACC_ACTIVE);
    m_bmi088.setGyroPoweMode(GYRO_NORMAL);
    delay(100);

    bool bResult = m_bmi088.runSelfTests();

    Serial.print( "Self-tests " );
    Serial.println( bResult ? "passed" : "failed" );

    /*
     *
     * End SELF TESTS
     *
     *-----------------------------------------------------
     */

    // Powerdown is required to perform an I2C soft reset
    m_bmi088.setAccPoweMode(ACC_SUSPEND);
    m_bmi088.setGyroPoweMode(GYRO_DEEP_SUSPEND);

    // Soft reset
    delay(5);
    m_bmi088.resetAcc();
    delay(5);
    m_bmi088.resetGyro();
    delay(60);

    // power back up
    m_bmi088.setAccPoweMode(ACC_ACTIVE);
    m_bmi088.setGyroPoweMode(GYRO_NORMAL);
    delay(60);

    // and initialize the chip
    m_bmi088.initialize( ODR_100, ODR_100_BW_32);

    Serial.println("Starting test -  120 seconds");
    Serial.println("----------------------------");

    // Switch to FIFO sampling for both ACC and GYRO
    m_bmi088.setAccFifoMode(ACC_FIFO_MODE_FIFO, true, true, false);
    m_bmi088.setGyroFifoMode(0, GYRO_FIFO_MODE_FIFO);

    m_bmi088.setAccScaleRange( RANGE_6G );
    m_bmi088.setGyroScaleRange( RANGE_250 );

    // let BMI088 settle into new operating modes (from manual 4.6.1 - > 50ms)
    delay(55);

  }
  else {
    Serial.println("BMI088 is not connected");
  }

  g_ulStart_ms = millis();
}

float fx, fy, fz;

void loop()
{
  
  int n;
  int m, m1;
  unsigned char ucBuf[128], *p, prefix;
  char szMessage[100];

  if ((millis() - g_ulStart_ms) > 120000) {
    Serial.print("Complete ");
    Serial.print(g_ulTotalBytes);
    Serial.println(" processed");

    Serial.print(g_ulTotalLoops);
    Serial.println(" loops");

    Serial.print(bucket[0]);
    Serial.println(" accelerometer samples");

    Serial.print(bucket[1]);
    Serial.println(" skips");

    Serial.print(bucket[2]);
    Serial.println(" times");

    Serial.print(bucket[3]);
    Serial.println(" config");

    Serial.print(bucket[4]);
    Serial.println(" drops");

    Serial.print(bucket[6]);
    Serial.println(" invalid accelerometer FIFO frame headers");

    Serial.print(bucket[5]);
    Serial.println(" gyro samples");

    sprintf(szMessage, "last accelerometer %.3f %.3f %.3f (mg)", fx, fy, fz);
    Serial.println( szMessage );

    while ( 1 ) { delay(100); yield(); };
  }

  /*
   * Read ACC FIFO
   */

  n = m_bmi088.getAccFifoLength();

  Wire.beginTransmission(BMI088_ACC_ADDRESS);
  Wire.write(BMI088_ACC_FIFO_DATA);
  Wire.endTransmission();

  Wire.requestFrom(BMI088_ACC_ADDRESS, n);
  m = m1 = Wire.available();
  p = ucBuf;
  while (m1-- > 0) {
    *p++ = Wire.read();
  }

  g_ulTotalBytes += m;
  g_ulTotalLoops++;
  p = ucBuf;

  while (m-- > 0) {

    prefix = *p++;

    switch(prefix & BMI088_FIFO_HEADER_MASK) {
    case BMI088_FIFO_HEADER_ACC_FRAME:
      {
        uint16_t ax = 0, ay = 0, az = 0;
        float value;

        ax = *p | (*(p+1) << 8);
        ay = *(p+2) | (*(p+3) << 8);
        az = *(p+4) | (*(p+5) << 8);

        value = (int16_t)ax;
        fx = 6000.0f * value / 32768.0f;

        value = (int16_t)ay;
        fy = 6000.0f * value / 32768.0f;

        value = (int16_t)az;
        fz = 6000.0f * value / 32768.0f;

        p += 6;
        m -= 6;
        bucket[0]++;
      }
      break;
    case BMI088_FIFO_HEADER_SKIP_FRAME:
      p += 1;
      m --;
      bucket[1]++;
      break;
    case BMI088_FIFO_HEADER_TIME_FRAME:
      p += 3;
      m -= 3;
      bucket[2]++;
      break;
    case BMI088_FIFO_HEADER_CONFIG_FRAME:
      p += 1;
      m --;
      bucket[3]++;
      break;
    case BMI088_FIFO_HEADER_DROP_FRAME:
      p += 1;
      m --;
      bucket[4]++;
      break;
    default:
      bucket[6]++;
      //sprintf(szMessage, "unrecognized prefix: 0x%02X", prefix & BMI088_FIFO_HEADER_MASK);
      //Serial.println(szMessage);
      break;
    }

    if (m<0) {
      Serial.println("buffer not message aligned");
    }

    /*
     * Now reaad Gyro FIFO
     */

    uint8_t ucGyroFrames;
    bool bOverflow = m_bmi088.getGyroFifoStatus(&ucGyroFrames);
    int nBytes = ucGyroFrames * 6;

    Wire.beginTransmission(BMI088_GYRO_ADDRESS);
    Wire.write(BMI088_GYRO_FIFO_DATA);
    Wire.endTransmission();

    Wire.requestFrom(BMI088_GYRO_ADDRESS, nBytes);
    m = m1 = Wire.available();
    p = ucBuf;
    while (m1-- > 0) {
      *p++ = Wire.read();
    }

    if (m != nBytes) {
      Serial.println("didn\'t get full last frame in Gyro buffer");
    }

    bucket[5] += (m/6);
    g_ulTotalGyroBytes += m;

    delay(3);
  }
  
}

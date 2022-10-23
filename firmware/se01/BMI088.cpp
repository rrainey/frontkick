/*
    A library for Grove - 6-Axis Accelerometer&Gyroscope（BMI088）

    This is a modified version of this library, specofically for the Frontkick
    project.

    The original Seeed Project is located at 
    https://github.com/Seeed-Studio/Grove_6Axis_Accelerometer_And_Gyroscope_BMI088

    Copyright (c) 2018 seeed technology co., ltd.
    Author      : Wayen Weng
    Create Time : June 2018
    Change Log  :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "BMI088.h"

BMI088::BMI088(void) {
    devAddrAcc = BMI088_ACC_ADDRESS;
    devAddrGyro = BMI088_GYRO_ADDRESS;
}

/// @brief Execute the standard IC self test sequence; See BST-BMI0088-DS000-18 section 4.6; requires at least 160ms to run.
///        Note: this does not function properly yet: always fails.
/// @return returns true if self tests pass
bool BMI088::runSelfTests(void) {

    float x, y, z;
    float px, py, pz;

    initialize();

    // Begin Accelerometer self test
    setAccScaleRange( RANGE_24G );

    write8(ACC, BMI088_ACC_CONF, (uint8_t)0xA7);
    delay ( 5 );

    write8(ACC, BMI088_ACC_SELF_TEST, (uint8_t)0x0D);
    delay ( 100 );

    getAcceleration(&px, &py, &pz);
    delay(50);
    getAcceleration(&px, &py, &pz);

    write8(ACC, BMI088_ACC_SELF_TEST, (uint8_t)0x09);
    delay ( 100 );

    getAcceleration(&x, &y, &z);
    delay(50);
    getAcceleration(&x, &y, &z);

    write8(ACC, BMI088_ACC_SELF_TEST, (uint8_t)0x00);
    delay ( 100 );

    // Begin Gyro Self Test
    write8(GYRO, BMI088_GYRO_SELF_TEST, (uint8_t)0x01);
    delay ( 5 );

    uint8_t status = 0;

    while ((status & (1<<1)) == 0 ) {
        status = read8(GYRO, BMI088_GYRO_SELF_TEST);
        delay(5);
    }

    char szBuffer[128];
    sprintf(szBuffer, "%.3f %.3f %.3f   %.3f %.3f %.3f  %d", px, py, pz, x, y, z, (status & (1<<2)));
    Serial.println(szBuffer);

    return ((status & (1<<2)) == 0) && ((px-x) > 1000.0f) && ((py-y) > 1000.0f) && ((pz-z) > 0.500f);
}

void BMI088::initialize(acc_odr_type_t acc_odr, gyro_odr_type_t gyro_odr) {
    int i;
    for (i=0; i<=BUCKET_ACC_UNK_FRAMES; ++i) {   
        m_ulFIFOBuckets[i] = 0;
    }

    delay(2);
    setAccPoweMode(ACC_ACTIVE);
    delay(1);
    setAccScaleRange(RANGE_6G);
    setAccOutputDataRate(acc_odr);
    
    setGyroScaleRange(RANGE_2000);
    setGyroOutputDataRate(gyro_odr);
    setGyroPoweMode(GYRO_NORMAL);
}

bool BMI088::isConnection(void) {
    return ((getAccID() == 0x1E) && (getGyroID() == 0x0F));
}

void BMI088::resetAcc(void) {
    write8(ACC, BMI088_ACC_SOFT_RESET, 0xB6);
}

void BMI088::resetGyro(void) {
    write8(GYRO, BMI088_GYRO_SOFT_RESET, 0xB6);
}

uint8_t BMI088::getAccID(void) {
    return read8(ACC, BMI088_GYRO_CHIP_ID);
}

uint8_t BMI088::getGyroID(void) {
    return read8(GYRO, BMI088_GYRO_CHIP_ID);
}

void BMI088::setAccPoweMode(acc_power_type_t mode) {
    if (mode == ACC_ACTIVE) {
        write8(ACC, BMI088_ACC_PWR_CTRL, 0x04);
        write8(ACC, BMI088_ACC_PWR_CONF, 0x00);
    } else if (mode == ACC_SUSPEND) {
        write8(ACC, BMI088_ACC_PWR_CONF, 0x03);
        write8(ACC, BMI088_ACC_PWR_CTRL, 0x00);
    }
}

void BMI088::setGyroPoweMode(gyro_power_type_t mode) {
    if (mode == GYRO_NORMAL) {
        write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_NORMAL);
    } else if (mode == GYRO_SUSPEND) {
        write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_SUSPEND);
    } else if (mode == GYRO_DEEP_SUSPEND) {
        write8(GYRO, BMI088_GYRO_LPM_1, (uint8_t)GYRO_DEEP_SUSPEND);
    }
}

void BMI088::setAccScaleRange(acc_scale_type_t range) {
    if (range == RANGE_3G) {
        accRange = 3000;
    } else if (range == RANGE_6G) {
        accRange = 6000;
    } else if (range == RANGE_12G) {
        accRange = 12000;
    } else if (range == RANGE_24G) {
        accRange = 24000;
    }

    write8(ACC, BMI088_ACC_RANGE, (uint8_t)range);
}

void BMI088::setAccOutputDataRate(acc_odr_type_t odr) {
    uint8_t data = 0;

    data = read8(ACC, BMI088_ACC_CONF);
    data = data & 0xf0;
    data = data | (uint8_t)odr;

    write8(ACC, BMI088_ACC_CONF, data);
}

void BMI088::setGyroScaleRange(gyro_scale_type_t range) {
    if (range == RANGE_2000) {
        gyroRange = 2000;
    } else if (range == RANGE_1000) {
        gyroRange = 1000;
    } else if (range == RANGE_500) {
        gyroRange = 500;
    } else if (range == RANGE_250) {
        gyroRange = 250;
    } else if (range == RANGE_125) {
        gyroRange = 125;
    }

    write8(GYRO, BMI088_GYRO_RANGE, (uint8_t)range);
}

void BMI088::setGyroOutputDataRate(gyro_odr_type_t odr) {
    write8(GYRO, BMI088_GYRO_BAND_WIDTH, (uint8_t)odr);
}

void BMI088::getAcceleration(float* x, float* y, float* z) {
    uint8_t buf[6] = {0};
    uint16_t ax = 0, ay = 0, az = 0;
    float value = 0;

    read(ACC, BMI088_ACC_X_LSB, buf, 6);

    ax = buf[0] | (buf[1] << 8);
    ay = buf[2] | (buf[3] << 8);
    az = buf[4] | (buf[5] << 8);

    value = (int16_t)ax;
    *x = accRange * value / 32768;

    value = (int16_t)ay;
    *y = accRange * value / 32768;

    value = (int16_t)az;
    *z = accRange * value / 32768;
}

float BMI088::getAccelerationX(void) {
    uint16_t ax = 0;
    float value = 0;

    ax = read16(ACC, BMI088_ACC_X_LSB);

    value = (int16_t)ax;
    value = accRange * value / 32768;

    return value;
}

float BMI088::getAccelerationY(void) {
    uint16_t ay = 0;
    float value = 0;

    ay = read16(ACC, BMI088_ACC_Y_LSB);

    value = (int16_t)ay;
    value = accRange * value / 32768;

    return value;
}

float BMI088::getAccelerationZ(void) {
    uint16_t az = 0;
    float value = 0;

    az = read16(ACC, BMI088_ACC_Z_LSB);

    value = (int16_t)az;
    value = accRange * value / 32768;

    return value;
}

void BMI088::getGyroscope(float* x, float* y, float* z) {
    uint8_t buf[6] = {0};
    uint16_t gx = 0, gy = 0, gz = 0;
    float value = 0;

    read(GYRO, BMI088_GYRO_RATE_X_LSB, buf, 6);

    gx = buf[0] | (buf[1] << 8);
    gy = buf[2] | (buf[3] << 8);
    gz = buf[4] | (buf[5] << 8);

    value = (int16_t)gx;
    *x = gyroRange * value / 32768;

    value = (int16_t)gy;
    *y = gyroRange * value / 32768;

    value = (int16_t)gz;
    *z = gyroRange * value / 32768;
}

float BMI088::getGyroscopeX(void) {
    uint16_t gx = 0;
    float value = 0;

    gx = read16(GYRO, BMI088_GYRO_RATE_X_LSB);

    value = (int16_t)gx;
    value = gyroRange * value / 32768;

    return value;
}

float BMI088::getGyroscopeY(void) {
    uint16_t gy = 0;
    float value = 0;

    gy = read16(GYRO, BMI088_GYRO_RATE_Y_LSB);

    value = (int16_t)gy;
    value = gyroRange * value / 32768;

    return value;
}

float BMI088::getGyroscopeZ(void) {
    uint16_t gz = 0;
    float value = 0;

    gz = read16(GYRO, BMI088_GYRO_RATE_Z_LSB);

    value = (int16_t)gz;
    value = gyroRange * value / 32768;

    return value;
}

float BMI088::getTemperature(void) {
    uint16_t uSample;
    int16_t nData;

    // See BMI Datasheet section 5.3.7

    uSample = read16Be(ACC, BMI088_ACC_TEMP_MSB);
    uSample = uSample >> 5;

    if (uSample > 1023) {
        nData = uSample - 2048;
    }
    else {
        nData = uSample;
    }

    return (float) nData / 8.0f + 23.0f;
}

void BMI088::write8(device_type_t dev, uint8_t reg, uint8_t val) {
    uint8_t addr = 0;

    if (dev) {
        addr = devAddrGyro;
    } else {
        addr = devAddrAcc;
    }

    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t BMI088::read8(device_type_t dev, uint8_t reg) {
    uint8_t addr = 0, data = 0;

    if (dev) {
        addr = devAddrGyro;
    } else {
        addr = devAddrAcc;
    }

    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(addr, 1);
    while (Wire.available()) {
        data = Wire.read();
    }

    return data;
}

/*
 * Little-endian 16-bit read
 */
uint16_t BMI088::read16(device_type_t dev, uint8_t reg) {
    uint8_t addr = 0;
    uint16_t msb = 0, lsb = 0;

    if (dev) {
        addr = devAddrGyro;
    } else {
        addr = devAddrAcc;
    }

    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(addr, 2);
    while (Wire.available()) {
        lsb = Wire.read();
        msb = Wire.read();
    }

    return (lsb | (msb << 8));
}

/*
 * Big-endian 16-bit read
 */
uint16_t BMI088::read16Be(device_type_t dev, uint8_t reg) {
    uint8_t addr = 0;
    uint16_t msb = 0, lsb = 0;

    if (dev) {
        addr = devAddrGyro;
    } else {
        addr = devAddrAcc;
    }

    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(addr, 2);
    while (Wire.available()) {
        msb = Wire.read();
        lsb = Wire.read();
    }

    return (lsb | (msb << 8));
}

uint32_t BMI088::read24(device_type_t dev, uint8_t reg) {
    uint8_t addr = 0;
    uint32_t hsb = 0, msb = 0, lsb = 0;

    if (dev) {
        addr = devAddrGyro;
    } else {
        addr = devAddrAcc;
    }

    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(addr, 3);
    while (Wire.available()) {
        lsb = Wire.read();
        msb = Wire.read();
        hsb = Wire.read();
    }

    return (lsb | (msb << 8) | (hsb << 16));
}

void BMI088::read(device_type_t dev, uint8_t reg, uint8_t* buf, uint16_t len) {
    uint8_t addr = 0;

    if (dev) {
        addr = devAddrGyro;
    } else {
        addr = devAddrAcc;
    }

    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(addr, len);
    while (Wire.available()) {
        for (uint16_t i = 0; i < len; i ++) {
            buf[i] = Wire.read();
        }
    }
}

/// @brief Get accelerometer intterup status; reading resets the status bit if set
/// @param  
/// @return returns true if interrupt is pending
bool BMI088::getAccInterruptState(void) {
    uint8_t uiStatus = read8(ACC, BMI088_ACC_INT_STAT_1);
    return (uiStatus & (1 << 7)) != 0;
}

/// @brief Get availability status of accelerometer data
/// @param  
/// @return returns true if data is ready for reading
bool BMI088::isAccDataReady(void) {
    uint8_t uiStatus = read8(ACC, BMI088_ACC_STATUS);
    return (uiStatus & (1 << 7)) != 0;
}

bool BMI088::isGyroDataReady(void) {
    uint8_t ucStatus = read8(GYRO, BMI088_GYRO_INT_STAT_1);
    return (ucStatus & (1 << 7)) != 0;
}

void BMI088::setGyroFifoMode(int nWatermarkLevel, gyro_fifo_mode_type_t eMode) {
    write8(GYRO, BMI088_GYRO_FIFO_CONFIG_1, eMode);
    write8(GYRO, BMI088_GYRO_FIFO_CONFIG_0, nWatermarkLevel);
}

bool BMI088::getGyroFifoStatus(uint8_t *pucVal) {
    uint8_t ucStatus = read8(GYRO, BMI088_GYRO_FIFO_STATUS);
    *pucVal = ucStatus & 0x7F;
    return (ucStatus & (1 << 7)) != 0;
}

uint16_t BMI088::getAccFifoLength(void) {
    uint16_t val = read16(ACC, BMI088_ACC_FIFO_LEN_0);
    return val & 0x3FFF;
}

void BMI088::setAccFifoMode(acc_fifo_mode_type_t eMode, bool bAcc_en, bool bInt1_en, bool bInt2_en)
{
    write8(ACC, BMI088_ACC_FIFO_CONFIG_0, (uint8_t) eMode);
    write8(ACC, BMI088_ACC_FIFO_CONFIG_1, (uint8_t) (
        1<<4 |  
        (bAcc_en ? (1<<6) : 0) | 
        (bInt1_en ? (1<<3) : 0) | 
        (bInt2_en ? (1<<2) : 0)
        ));
}

int BMI088::updateAccQueue(void) {

  int n;
  int m, m1;
  unsigned char ucBuf[128], *p, prefix;
  //char szMessage[100];

  n = getAccFifoLength();

  Wire.beginTransmission(BMI088_ACC_ADDRESS);
  Wire.write(BMI088_ACC_FIFO_DATA);
  Wire.endTransmission();

  Wire.requestFrom(BMI088_ACC_ADDRESS, n);
  m = m1 = Wire.available();
  p = ucBuf;
  while (m1-- > 0) {
    *p++ = Wire.read();
  }

  p = ucBuf;

  while (m-- > 0) {

    prefix = *p++;

    switch(prefix & BMI088_FIFO_HEADER_MASK) {
    case BMI088_FIFO_HEADER_ACC_FRAME:
      {
        BMISample sample;

        sample.x = *p | (*(p+1) << 8);
        sample.y = *(p+2) | (*(p+3) << 8);
        sample.z = *(p+4) | (*(p+5) << 8);

        m_accQueue.pushBack( sample );

        p += 6;
        m -= 6;
        m_ulFIFOBuckets[BUCKET_ACC_SAMPLE_FRAMES ]++;
      }
      break;
    case BMI088_FIFO_HEADER_SKIP_FRAME:
      p += 1;
      m --;
      m_ulFIFOBuckets[BUCKET_ACC_SKIP_FRAMES]++;
      break;
    case BMI088_FIFO_HEADER_TIME_FRAME:
      p += 3;
      m -= 3;
      m_ulFIFOBuckets[BUCKET_ACC_TIME_FRAMES]++;
      break;
    case BMI088_FIFO_HEADER_CONFIG_FRAME:
      p += 1;
      m --;
      m_ulFIFOBuckets[BUCKET_ACC_CONFIG_FRAMES]++;
      break;
    case BMI088_FIFO_HEADER_DROP_FRAME:
      p += 1;
      m --;
      m_ulFIFOBuckets[BUCKET_GYRO_DROP_FRAMES]++;
      break;
    default:
      m_ulFIFOBuckets[BUCKET_ACC_UNK_FRAMES]++;
      //sprintf(szMessage, "unrecognized prefix: 0x%02X", prefix & BMI088_FIFO_HEADER_MASK);
      //Serial.println(szMessage);
      break;
    }

    if (m<0) {
      Serial.println("buffer not message aligned");
    }
    
  }
  return m_accQueue.size();
}

int BMI088::updateGyroQueue(void) {

    int n;
    int m, m1;
    unsigned char ucBuf[128], *p, prefix;
    BMISample sample;

    /*
     * reaad Gyro FIFO
     */

    uint8_t ucGyroFrames;
    bool bOverflow = getGyroFifoStatus(&ucGyroFrames);
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

    int nFrames = m/6;
    p = ucBuf;
    for (m1=0; m1<nFrames; m1++) {

        sample.x = *p | (*(p+1) << 8);
        sample.y = *(p+2) | (*(p+3) << 8);
        sample.z = *(p+4) | (*(p+5) << 8);

        m_gyroQueue.pushBack( sample );

        p+=6;
    }

    m_ulFIFOBuckets[BUCKET_GYRO_SAMPLE_FRAMES] += nFrames;

    return m_gyroQueue.size();
}

bool BMI088::popAccelerationSampleFromQueue(float* px, float* py, float* pz) {

    BMISample *pSample;

    pSample = m_accQueue.popFront();

    if (pSample) {
        *px = accRange * pSample->x / 32768.0f;

        *py = accRange * pSample->y / 32768.0f;

        *pz = accRange * pSample->z / 32768.0f;
    }

    return !(pSample==NULL);
}

bool BMI088::popGyroscopeSampleFromQueue(float* px, float* py, float* pz) {

    BMISample *pSample;

    pSample = m_gyroQueue.popFront();

    if (pSample) {
        *px = gyroRange * pSample->x / 32768.0f;

        *py = gyroRange * pSample->y / 32768.0f;

        *pz = gyroRange * pSample->z / 32768.0f;
    }

    return !(pSample==NULL);
}

BMISampleQueue::BMISampleQueue() {
    clear();
}

void BMISampleQueue::clear() {
    m_nFront = m_nBack = 0;
}


bool BMISampleQueue::isEmpty() const { 
    return m_nFront == m_nBack;
}

bool BMISampleQueue::isFull() const { 
    return size() == SAMPLE_QUEUE_SIZE;
}

int BMISampleQueue::size() const {
    int n;
    if (m_nBack >= m_nFront) {
        n = m_nBack - m_nFront;
    }
    else {
        n = (SAMPLE_QUEUE_SIZE - m_nFront) + m_nBack;
    }
    return n;
}

void BMISampleQueue::pushBack( BMISample &s) {
    if (size() < SAMPLE_QUEUE_SIZE) {
        m_q[m_nBack++] = s;
        if (m_nBack == SAMPLE_QUEUE_SIZE) {
            m_nBack = 0;
        }
    }
}

BMISample * BMISampleQueue::popFront() {
    BMISample *pResult = NULL;
    if (! isEmpty()) {
        pResult = &m_q[m_nFront++];
        if (m_nFront == SAMPLE_QUEUE_SIZE) {
            m_nFront = 0;
        }
    }
    return pResult;
}
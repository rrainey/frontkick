/*
    This is a modified version of this library, specifically for the Frontkick
    project.

    The original Seeed Project is located at 
    https://github.com/Seeed-Studio/Grove_6Axis_Accelerometer_And_Gyroscope_BMI088

    A library for Grove - 6-Axis Accelerometer&Gyroscope（BMI088)

    Copyright (c) 2018 seeed technology co., ltd.
    Author      : Wayen Weng
    Create Time : June 2018
    Change Log  :
            2022-10 Riley Rainey - Added FIFO support along with a few bug fixes (notably getTemperature())

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

    References:
    https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-mis-an005.pdf
*/

#ifndef __BOSCH_BMI088_H__
#define __BOSCH_BMI088_H__

#define SERIAL_BUFFER_SIZE 512

#include <Arduino.h>

#include <Wire.h>

// mg to meters per second (unsure of the value Bosch uses for calibration)
#define MGtoMPS(x) ((x)*9.80665f)

#define BMI088_ACC_ADDRESS          0x18  

#define BMI088_ACC_CHIP_ID          0x00 // Default value 0x1E
#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03

#define BMI088_ACC_X_LSB            0x12
#define BMI088_ACC_X_MSB            0x13
#define BMI088_ACC_Y_LSB            0x14
#define BMI088_ACC_Y_MSB            0x15
#define BMI088_ACC_Z_LSB            0x16
#define BMI088_ACC_Z_MSB            0x17

#define BMI088_ACC_SENSOR_TIME_0    0x18
#define BMI088_ACC_SENSOR_TIME_1    0x19
#define BMI088_ACC_SENSOR_TIME_2    0x1A

#define BMI088_ACC_INT_STAT_1       0x1D

#define BMI088_ACC_TEMP_MSB         0x22
#define BMI088_ACC_TEMP_LSB         0x23

#define BMI088_ACC_FIFO_LEN_0       0x24
#define BMI088_ACC_FIFO_LEN_1       0x25
#define BMI088_ACC_FIFO_DATA        0x26

#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41

#define BMI088_ACC_FIFO_CONFIG_0    0x48
#define BMI088_ACC_FIFO_CONFIG_1    0x49

#define BMI088_ACC_INT1_IO_CTRL     0x53
#define BMI088_ACC_INT2_IO_CTRL     0x54
#define BMI088_ACC_INT_MAP_DATA     0x58

#define BMI088_ACC_SELF_TEST        0x6D

#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRL         0x7D

#define BMI088_ACC_SOFT_RESET       0x7E

#define BMI088_GYRO_ADDRESS             0x68

#define BMI088_GYRO_CHIP_ID             0x00 // Default value 0x0F

#define BMI088_GYRO_RATE_X_LSB          0x02
#define BMI088_GYRO_RATE_X_MSB          0x03
#define BMI088_GYRO_RATE_Y_LSB          0x04
#define BMI088_GYRO_RATE_Y_MSB          0x05
#define BMI088_GYRO_RATE_Z_LSB          0x06
#define BMI088_GYRO_RATE_Z_MSB          0x07

#define BMI088_GYRO_INT_STAT_1          0x0A

#define BMI088_GYRO_FIFO_STATUS         0x0E

#define BMI088_GYRO_RANGE               0x0F
#define BMI088_GYRO_BAND_WIDTH          0x10

#define BMI088_GYRO_LPM_1               0x11

#define BMI088_GYRO_SOFT_RESET          0x14

#define BMI088_GYRO_INT_CTRL            0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF   0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP    0x18

#define BMI088_GYRO_SELF_TEST           0x3C

#define BMI088_GYRO_FIFO_CONFIG_0       0x3D
#define BMI088_GYRO_FIFO_CONFIG_1       0x3E

#define BMI088_GYRO_FIFO_DATA           0x3F // six byte groups; no autoincrement: X_LSB X_MSB Y_LSB Y_MSB Z_LSB Z_MSB

/*
 * Definitions related to Accelerometer FIFO processing
 * See BMI088 datasheet section 4.9
 */
#define BMI088_FIFO_HEADER_MASK          (0xFC)
#define BMI088_FIFO_HEADER_ACC_FRAME     (1<<7 | 1<<2)  // + 6 data bytes
#define BMI088_FIFO_HEADER_SKIP_FRAME    (1<<6)         // + 1 payload byte
#define BMI088_FIFO_HEADER_TIME_FRAME    (1<<6 | 1<<2)  // + 3 payload bytes
#define BMI088_FIFO_HEADER_CONFIG_FRAME  (1<<6 | 1<<3)  // + 1 payload byte
#define BMI088_FIFO_HEADER_DROP_FRAME    (1<<6 | 1<<4)  // + 1 payload byte

enum device_type_t { // device type
    ACC = 0x00, //
    GYRO = 0x01, //
};

enum acc_scale_type_t { // measurement rage
    RANGE_3G = 0x00, //
    RANGE_6G = 0x01, //
    RANGE_12G = 0x02, //
    RANGE_24G = 0x03, //
};

enum acc_odr_type_t { // output data rate
    ODR_12 = 0x05, //
    ODR_25 = 0x06, //
    ODR_50 = 0x07, //
    ODR_100 = 0x08, //
    ODR_200 = 0x09, //
    ODR_400 = 0x0A, //
    ODR_800 = 0x0B, //
    ODR_1600 = 0x0C, //
};

enum acc_power_type_t { // power mode
    ACC_ACTIVE = 0x00, //
    ACC_SUSPEND = 0x03, //
};

enum gyro_scale_type_t { // measurement rage
    RANGE_2000 = 0x00, //
    RANGE_1000 = 0x01, //
    RANGE_500 = 0x02, //
    RANGE_250 = 0x03, //
    RANGE_125 = 0x04, //
};

enum gyro_odr_type_t { // output data rate
    ODR_2000_BW_532 = 0x00, //
    ODR_2000_BW_230 = 0x01, //
    ODR_1000_BW_116 = 0x02, //
    ODR_400_BW_47 = 0x03, //
    ODR_200_BW_23 = 0x04, //
    ODR_100_BW_12 = 0x05, //
    ODR_200_BW_64 = 0x06, //
    ODR_100_BW_32 = 0x07, //
};

enum gyro_power_type_t { // power mode
    GYRO_NORMAL = 0x00, //
    GYRO_SUSPEND = 0x80, //
    GYRO_DEEP_SUSPEND = 0x20, //
};

enum gyro_fifo_mode_type_t { // device type
    GYRO_FIFO_MODE_FIFO = 0x40,
    GYRO_FIFO_MODE_STREAM = 0x80
};

enum acc_fifo_mode_type_t {
    ACC_FIFO_MODE_STREAM = 0x2,
    ACC_FIFO_MODE_FIFO = 0x3,
};

/// @brief sample data from acc/gyro FIFOs; queued in 16-bit integer form rather than 32-bit float to conserve storage
typedef struct _BMISample {
    int16_t x;
    int16_t y;
    int16_t z;
} BMISample;

///@brief size of processed FIFO sample queue; must be large enough to take all entries from the FIFO expected in one loop()
///       so, the size required will be dependent on both the BMI sample rate and the rate that loop() is invoked.
#ifndef SAMPLE_QUEUE_SIZE
#define SAMPLE_QUEUE_SIZE   50
#endif

/// @brief An intermediate queue for storing samples from the BMI FIFO in a more structured manner.
class BMISampleQueue {
    public:
        BMISampleQueue();

        /// @brief return true if there are no entries in the queue
        bool isEmpty() const;

        /// @brief returns true if there is no remaining space in the queue
        bool isFull() const;

        /// @brief number of elements in the queue
        unsigned short size() const;

        /// @brief remove all entries from the queue
        void clear();

        /// @brief add an element to the back of the queue, space permitting; no action if the queue was already full
        /// @param s element to push at the back of the queue
        void pushBack( BMISample &s);

        /// @brief pops the front element off the queue
        /// @return a pointer to the entry; should be copied immediately to avoid losing the data; 
        ///         returns NULL if no entry available
        BMISample * popFront();

    protected:
        BMISample m_q[SAMPLE_QUEUE_SIZE];
        unsigned short m_nFront;
        unsigned short m_nBack;
};

class BMI088 {
  public:

    BMI088(void);

    bool isConnection(void);

    bool runSelfTests(void);

    void initialize(acc_odr_type_t acc_odr = ODR_100, gyro_odr_type_t gyro_odr = ODR_2000_BW_532);

    void setAccPoweMode(acc_power_type_t mode);
    void setGyroPoweMode(gyro_power_type_t mode);

    void setAccScaleRange(acc_scale_type_t range);
    void setAccOutputDataRate(acc_odr_type_t odr);

    void setGyroScaleRange(gyro_scale_type_t range);
    void setGyroOutputDataRate(gyro_odr_type_t odr);

    void getAcceleration(float* x, float* y, float* z);

    /// @brief Pop next Accelerometer sample from queue if available; scale to milli-gee
    /// @param px 
    /// @param py 
    /// @param pz 
    /// @return return false if no sample was available in the Accelerometer queue
    bool popAccelerationSampleFromQueue(float* px, float* py, float* pz);

    float getAccelerationX(void);
    float getAccelerationY(void);
    float getAccelerationZ(void);

    void getGyroscope(float* x, float* y, float* z);

    /// @brief Pop next Gyro sample from queue if available; scale to deg/sec
    /// @param px 
    /// @param py 
    /// @param pz 
    /// @return return false if no sample was available in the Gyro queue
    bool popGyroscopeSampleFromQueue(float* px, float* py, float* pz);

    float getGyroscopeX(void);
    float getGyroscopeY(void);
    float getGyroscopeZ(void);

    /// @brief returns current IC temperature reading; resolution 0.125 degC
    /// @return returns temprature (degrees Celcius)
    float getTemperature(void);

    uint8_t getAccID(void);
    uint8_t getGyroID(void);

    void resetAcc(void);
    void resetGyro(void);

    bool getAccInterruptState(void);
    bool isAccDataReady(void);
    bool isGyroDataReady(void);

    void setGyroFifoMode(int nWatermarkLevel, gyro_fifo_mode_type_t eMode);

    void setAccFifoMode(acc_fifo_mode_type_t eMode, bool bAcc_en, bool bInt1_en, bool bInt2_en);

    /// @brief examine the Gyro FIFO state and return the number of elements available for reading
    /// @param puVal - number of entries left to be read from FIFO
    /// @return state of FIFO buffer overflow bit
    bool getGyroFifoStatus(uint8_t *puVal);

    uint16_t getAccFifoLength(void);

    /// @brief pull all data in Accelerometer FIFO, creating corresponding queue entries
    /// @return returns number of entries in Accelerometer queue after completion
    int updateAccQueue(void);

    /// @brief pull all data in Gyro FIFO, creating corresponding queue entries
    /// @return returns number of entries in Accelerometer queue after completion
    int updateGyroQueue(void);

  private:

    void write8(device_type_t dev, uint8_t reg, uint8_t val);
    uint8_t read8(device_type_t dev, uint8_t reg);
    uint16_t read16(device_type_t dev, uint8_t reg);
    uint16_t read16Be(device_type_t dev, uint8_t reg);
    uint32_t read24(device_type_t dev, uint8_t reg);
    void read(device_type_t dev, uint8_t reg, uint8_t* buf, uint16_t len);

    float accRange;
    float gyroRange;
    uint8_t devAddrAcc;
    uint8_t devAddrGyro;

    BMISampleQueue m_accQueue;
    BMISampleQueue m_gyroQueue;

#define BUCKET_ACC_SAMPLE_FRAMES  0
#define BUCKET_ACC_SKIP_FRAMES    1
#define BUCKET_ACC_TIME_FRAMES    2
#define BUCKET_ACC_CONFIG_FRAMES  3
#define BUCKET_GYRO_DROP_FRAMES   4
#define BUCKET_GYRO_SAMPLE_FRAMES 5
#define BUCKET_ACC_UNK_FRAMES     6 /// unrecognized frame header bytes (skipped)

  public:
    /// @brief maintain statistics on data we pull from the two BMI FIFOs
    unsigned long m_ulFIFOBuckets[7];

};

//extern BMI088 bmi088;

#endif

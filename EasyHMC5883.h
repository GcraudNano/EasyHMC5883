#ifndef __EasyHMC5883__
#define __EasyHMC5883__

#if defined(ARDUINO)
#   include <Arduino.h>
#   include <Wire.h>
#elif defined(__MBED__)
#   include <i2c_api.h>
#endif

class HMC5883Class {
    static const int     i2cAddress;
    static const int     modeRegister;  /* select  "Mode Register"              */
    static const uint8_t i2cMode;       /* set the "Continuous-MeasurementMode" */
    static const int     configReg;     /* select  "Configuration Register A"   */
    static const uint8_t sensorRate;    /* set 75Hz                             */
    static const int     outRegister;   /* select "Data Output XMSB Register"   */
    union Data {
        uint8_t buf[6];
        struct {
            int16_t x;
            int16_t z;
            int16_t y;
        };
    };
#ifdef __MBED__
    i2c_t i2c;
#endif
    uint16_t xOffset;
    uint16_t yOffset;
    float angleOffset;
    void configure();
    void readRawDatas(uint8_t *buf);
    void i2c_device_read(int address, uint8_t *buf, int length);
    void i2c_device_write(int address, const uint8_t *buf, int length);
    
public:
    HMC5883Class();
#if defined(ARDUINO)
    void begin();
#elif defined(__MBED__)
    void beginOnPins(PinName sda, PinName scl);
#endif
    float read();
    void calibrateForMilliSecs(int ms);
    void calibrateForFunction(bool (*delegate)());
    void setOffset();
};

extern HMC5883Class HMC5883;

#endif

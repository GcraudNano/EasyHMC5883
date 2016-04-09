#include "EasyHMC5883.h"
#include <math.h>

#ifdef __MBED__
#   include <Timer.h>
#   define degrees(rad) ((rad) * 180 / M_PI)
#endif
#define LAMBDA(arglist, arrow, retType, body)	\
({                                              \
    class Lambda                                \
    {                                           \
    public:                                     \
        static retType func arglist body        \
    };                                          \
    Lambda::func;                               \
})

const int     HMC5883Class::i2cAddress      = 0x1E;
const int     HMC5883Class::modeRegister    = 2;
const uint8_t HMC5883Class::i2cMode         = 0;
const int     HMC5883Class::configReg       = 0x00;
const uint8_t HMC5883Class::sensorRate      = 0b00111000;
const int     HMC5883Class::outRegister     = 3;

HMC5883Class::HMC5883Class() : xOffset(0), yOffset(0), angleOffset(0)
{
}

void HMC5883Class::configure()
{
    i2c_device_write(modeRegister, &i2cMode, 1);
    i2c_device_write(configReg, &sensorRate, 1);
}

void HMC5883Class::readRawDatas(uint8_t *buf)
{
    i2c_device_read(outRegister, buf, 6);
}

#if defined(ARDUINO)
void HMC5883Class::begin()
{
    Wire.begin();
}
#elif defined(__MBED__)
void HMC5883Class::beginOnPins(PinName sda, PinName scl)
{
    i2c_init(&i2c, sda, scl);
}
#endif

float HMC5883Class::read()
{
    Data data;
    readRawDatas(data.buf);
    return degrees(atan2f(data.y - yOffset, data.x - xOffset)) - angleOffset;
}

void HMC5883Class::calibrateForMilliSecs(int ms)
{
#if defined(ARDUINO)
    static unsigned long limit;
    limit = millis() + ms;
#elif defined(__MBED__)
    static int limit;
    static mbed::Timer timer;
    limit = ms;
    timer.reset();
    timer.start();
#endif
    calibrateForFunction(LAMBDA((), ->, bool, {
#if defined(ARDUINO)
        return millis() < limit;
#elif defined(__MBED__)
        return timer.read_ms() < limit;
#endif
    }));
}

void HMC5883Class::calibrateForFunction(bool (*delegate)())
{
    int16_t xMin = 0, xMax = 0, yMin = 0, yMax = 0;
    Data data;
    while (delegate()) {
        readRawDatas(data.buf);
        if (data.x < xMin) {
            xMin = data.x;
        } else if (xMax < data.x) {
            xMax = data.x;
        }
        if (data.y < yMin) {
            yMin = data.y;
        } else if (yMax < data.y) {
            yMax = data.y;
        }
    }
    xOffset = (xMin + xMax) / 2;
    yOffset = (yMin + yMax) / 2;
}

void HMC5883Class::setOffset()
{
    angleOffset = 0;
    angleOffset = read();
}

void HMC5883Class::i2c_device_read(int address, uint8_t *buf, int length)
{
#if defined(ARDUINO)
    Wire.beginTransmission(i2cAddress);
    Wire.write(address);
    Wire.endTransmission();
    Wire.requestFrom(i2cAddress, length);
    do {
        *buf = Wire.read();
        ++buf;
    } while (--length);
#elif defined(__MBED__)
    i2c_write(&i2c, i2cAddress, reinterpret_cast<char *>(&address), 1, false);
    i2c_read(&i2c, i2cAddress, reinterpret_cast<char *>(buf), length, true);
#endif
}

void HMC5883Class::i2c_device_write(int address, const uint8_t *buf, int length)
{
#if defined(ARDUINO)
    Wire.beginTransmission(i2cAddress);
    Wire.write(address);
    do {
        Wire.write(*buf);
        ++buf;
    } while (--length);
    Wire.endTransmission();
#elif defined(__MBED__)
    char datas[length + 1];
    datas[0] = address;
    std::memcpy(&datas[1], buf, length);
    i2c_write(&i2c, i2cAddress, datas, length, true);
#endif
}

HMC5883Class HMC5883;

/*!
 * @file Adafruit_BME280.cpp
 *
 * @mainpage Adafruit BME280 humidity, temperature & pressure sensor
 *
 * @section intro_sec Introduction
 *
 *  Driver for the BME280 humidity, temperature & pressure sensor
 *
 * These sensors use I2C or SPI to communicate, 2 or 4 pins are required
 * to interface.
 *
 * Designed specifically to work with the Adafruit BME280 Breakout
 * ----> http://www.adafruit.com/products/2652
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 * See the LICENSE file for details.
 *
 */

#include "Adafruit_BME280.h"
#include "Arduino.h"

Adafruit_BME280::Adafruit_BME280() {
}

bool Adafruit_BME280::begin(TwoWire* theWire, uint8_t addr) {
    i2c_dev = new Adafruit_I2CDevice(addr, theWire);
    if (!i2c_dev->begin())
        return false;
    return init();
}

bool Adafruit_BME280::init() {
    // check if sensor, i.e. the chip ID is correct
    _sensorID = _read8(BME280_REGISTER_CHIPID);
    if (_sensorID != 0x60)
        return false;

    // reset the device using soft-reset
    // this makes sure the IIR is off, etc.
    _write8(BME280_REGISTER_SOFTRESET, 0xB6);

    return true;
}

void Adafruit_BME280::setSampling(sensor_mode mode,
                                  sensor_sampling tempSampling,
                                  sensor_sampling pressSampling,
                                  sensor_sampling humSampling,
                                  sensor_filter filter,
                                  standby_duration duration) {
    _measReg.mode = mode;
    _measReg.osrs_t = tempSampling;
    _measReg.osrs_p = pressSampling;

    _humReg.osrs_h = humSampling;
    _configReg.filter = filter;
    _configReg.t_sb = duration;
    _configReg.spi3w_en = 0;

    // making sure sensor is in sleep mode before setting configuration
    // as it otherwise may be ignored
    _write8(BME280_REGISTER_CONTROL, MODE_SLEEP);

    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see
    // DS 5.4.3)
    _write8(BME280_REGISTER_CONTROLHUMID, _humReg.get());
    _write8(BME280_REGISTER_CONFIG, _configReg.get());
    _write8(BME280_REGISTER_CONTROL, _measReg.get());
}

bool Adafruit_BME280::takeForcedMeasurement() {
    // If we are in forced mode, the BME sensor goes back to sleep after each
    // measurement and we need to set it to forced mode once at this point, so
    // it will take the next measurement and then return to sleep again.
    // In normal mode simply does new measurements periodically.
    if (_measReg.mode == MODE_FORCED) {
        // set to forced mode, i.e. "take next measurement"
        _write8(BME280_REGISTER_CONTROL, _measReg.get());
        return true;
    }
    return false;
}

float Adafruit_BME280::readTemperature(void) {
    int32_t var1, var2;

    int32_t adc_T = _read24(BME280_REGISTER_TEMPDATA);
    if (adc_T == 0x800000)  // value in case temp measurement was disabled
        return NAN;
    adc_T >>= 4;

    var1 = (int32_t)((adc_T / 8) - ((int32_t)_bme280_calib.dig_T1 * 2));
    var1 = (var1 * ((int32_t)_bme280_calib.dig_T2)) / 2048;
    var2 = (int32_t)((adc_T / 16) - ((int32_t)_bme280_calib.dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)_bme280_calib.dig_T3)) / 16384;

    t_fine = var1 + var2 + t_fine_adjust;

    int32_t T = (t_fine * 5 + 128) / 256;

    return (float)T / 100;
}

float Adafruit_BME280::readPressure(void) {
    int64_t var1, var2, var3, var4;

    readTemperature();  // must be done first to get t_fine

    int32_t adc_P = _read24(BME280_REGISTER_PRESSUREDATA);
    if (adc_P == 0x800000)  // value in case pressure measurement was disabled
        return NAN;
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_bme280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)_bme280_calib.dig_P5) * 131072);
    var2 = var2 + (((int64_t)_bme280_calib.dig_P4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)_bme280_calib.dig_P3) / 256) +
           ((var1 * ((int64_t)_bme280_calib.dig_P2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t)_bme280_calib.dig_P1) / 8589934592;

    if (var1 == 0)
        return 0;  // avoid exception caused by division by zero

    var4 = 1048576 - adc_P;
    var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
    var1 = (((int64_t)_bme280_calib.dig_P9) * (var4 / 8192) * (var4 / 8192)) /
           33554432;
    var2 = (((int64_t)_bme280_calib.dig_P8) * var4) / 524288;
    var4 =
        ((var4 + var1 + var2) / 256) + (((int64_t)_bme280_calib.dig_P7) * 16);

    float P = var4 / 256.0;

    return P;
}

float Adafruit_BME280::readHumidity(void) {
    int32_t var1, var2, var3, var4, var5;

    readTemperature();  // must be done first to get t_fine

    int32_t adc_H = _read16(BME280_REGISTER_HUMIDDATA);
    if (adc_H == 0x8000)  // value in case humidity measurement was disabled
        return NAN;

    var1 = t_fine - ((int32_t)76800);
    var2 = (int32_t)(adc_H * 16384);
    var3 = (int32_t)(((int32_t)_bme280_calib.dig_H4) * 1048576);
    var4 = ((int32_t)_bme280_calib.dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)_bme280_calib.dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)_bme280_calib.dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)_bme280_calib.dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)_bme280_calib.dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    uint32_t H = (uint32_t)(var5 / 4096);

    return (float)H / 1024.0;
}

void Adafruit_BME280::readCoefficients(void) {
    _bme280_calib.dig_T1 = _read16_LE(BME280_REGISTER_DIG_T1);
    _bme280_calib.dig_T2 = _readS16_LE(BME280_REGISTER_DIG_T2);
    _bme280_calib.dig_T3 = _readS16_LE(BME280_REGISTER_DIG_T3);

    _bme280_calib.dig_P1 = _read16_LE(BME280_REGISTER_DIG_P1);
    _bme280_calib.dig_P2 = _readS16_LE(BME280_REGISTER_DIG_P2);
    _bme280_calib.dig_P3 = _readS16_LE(BME280_REGISTER_DIG_P3);
    _bme280_calib.dig_P4 = _readS16_LE(BME280_REGISTER_DIG_P4);
    _bme280_calib.dig_P5 = _readS16_LE(BME280_REGISTER_DIG_P5);
    _bme280_calib.dig_P6 = _readS16_LE(BME280_REGISTER_DIG_P6);
    _bme280_calib.dig_P7 = _readS16_LE(BME280_REGISTER_DIG_P7);
    _bme280_calib.dig_P8 = _readS16_LE(BME280_REGISTER_DIG_P8);
    _bme280_calib.dig_P9 = _readS16_LE(BME280_REGISTER_DIG_P9);

    _bme280_calib.dig_H1 = _read8(BME280_REGISTER_DIG_H1);
    _bme280_calib.dig_H2 = _readS16_LE(BME280_REGISTER_DIG_H2);
    _bme280_calib.dig_H3 = _read8(BME280_REGISTER_DIG_H3);
    _bme280_calib.dig_H4 = ((int8_t)_read8(BME280_REGISTER_DIG_H4) << 4) |
                           (_read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
    _bme280_calib.dig_H5 = ((int8_t)_read8(BME280_REGISTER_DIG_H5 + 1) << 4) |
                           (_read8(BME280_REGISTER_DIG_H5) >> 4);
    _bme280_calib.dig_H6 = (int8_t)_read8(BME280_REGISTER_DIG_H6);
}

bool Adafruit_BME280::isReadingCalibration(void) {
    uint8_t const rStatus = _read8(BME280_REGISTER_STATUS);

    return (rStatus & (1 << 0)) != 0;
}

void Adafruit_BME280::_write8(byte reg, byte value) {
    byte buffer[2];

    buffer[0] = reg;
    buffer[1] = value;

    i2c_dev->write(buffer, 2);
}

uint8_t Adafruit_BME280::_read8(byte reg) {
    uint8_t buffer[1];

    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 1);

    return buffer[0];
}

uint16_t Adafruit_BME280::_read16(byte reg) {
    uint8_t buffer[2];

    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 2);

    return uint16_t(buffer[0]) << 8 | uint16_t(buffer[1]);
}

uint16_t Adafruit_BME280::_read16_LE(byte reg) {
    uint16_t temp = _read16(reg);
    return (temp >> 8) | (temp << 8);
}

int16_t Adafruit_BME280::_readS16(byte reg) {
    return (int16_t)_read16(reg);
}

int16_t Adafruit_BME280::_readS16_LE(byte reg) {
    return (int16_t)_read16_LE(reg);
}

uint32_t Adafruit_BME280::_read24(byte reg) {
    uint8_t buffer[3];

    buffer[0] = uint8_t(reg);
    i2c_dev->write_then_read(buffer, 1, buffer, 3);

    return uint32_t(buffer[0]) << 16 | uint32_t(buffer[1]) << 8 |
           uint32_t(buffer[2]);
}

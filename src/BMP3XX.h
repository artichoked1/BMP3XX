#ifndef BMP3XX_H
#define BMP3XX_H

#include <Arduino.h>
#include <Wire.h>

// Sensor's I2C address
#define BMP3XX_ADDRESS 0x77

// Control registers
#define REG_RESET 0x7E
#define REG_CONFIG 0x00
#define REG_PWR_CTRL 0x1B
#define REG_OSR 0x1C
#define REG_ODR 0x1D
#define REG_STATUS 0x03
#define REG_PRESSURE 0x04
#define REG_TEMPERATURE 0x07
#define REG_ERROR 0x02

// NVM calibration registers
#define NVM_PAR_T1_0 0x31
#define NVM_PAR_T1_1 0x32
#define NVM_PAR_T2_0 0x33
#define NVM_PAR_T2_1 0x34
#define NVM_PAR_T3 0x35
#define NVM_PAR_P1_0 0x36
#define NVM_PAR_P1_1 0x37
#define NVM_PAR_P2_0 0x38
#define NVM_PAR_P2_1 0x39
#define NVM_PAR_P3 0x3A
#define NVM_PAR_P4 0x3B
#define NVM_PAR_P5_0 0x3C
#define NVM_PAR_P5_1 0x3D
#define NVM_PAR_P6_0 0x3E
#define NVM_PAR_P6_1 0x3F
#define NVM_PAR_P7 0x40
#define NVM_PAR_P8 0x41
#define NVM_PAR_P9_0 0x42
#define NVM_PAR_P9_1 0x43
#define NVM_PAR_P10 0x44
#define NVM_PAR_P11 0x45

// BMP3XX settings
#define BMP3XX_NO_OVERSAMPLING 0x00
#define BMP3XX_OVERSAMPLING_2X 0x01
#define BMP3XX_OVERSAMPLING_4X 0x02
#define BMP3XX_OVERSAMPLING_8X 0x03
#define BMP3XX_OVERSAMPLING_16X 0x04
#define BMP3XX_OVERSAMPLING_32X 0x05

#define BMP3XX_IIR_FILTER_OFF 0x00
#define BMP3XX_IIR_FILTER_COEFF_1 0x01
#define BMP3XX_IIR_FILTER_COEFF_3 0x02
#define BMP3XX_IIR_FILTER_COEFF_7 0x03
#define BMP3XX_IIR_FILTER_COEFF_15 0x04
#define BMP3XX_IIR_FILTER_COEFF_31 0x05
#define BMP3XX_IIR_FILTER_COEFF_63 0x06
#define BMP3XX_IIR_FILTER_COEFF_127 0x07

#define BMP3XX_ODR_200HZ 0x00
#define BMP3XX_ODR_100HZ 0x01
#define BMP3XX_ODR_50HZ 0x02
#define BMP3XX_ODR_25HZ 0x03
#define BMP3XX_ODR_25_2HZ 0x04
#define BMP3XX_ODR_25_4HZ 0x05
#define BMP3XX_ODR_25_8HZ 0x06
#define BMP3XX_ODR_25_16HZ 0x07
#define BMP3XX_ODR_25_32HZ 0x08
#define BMP3XX_ODR_25_64HZ 0x09
#define BMP3XX_ODR_25_128HZ 0x0A
#define BMP3XX_ODR_25_256HZ 0x0B
#define BMP3XX_ODR_25_512HZ 0x0C
#define BMP3XX_ODR_25_1024HZ 0x0D
#define BMP3XX_ODR_25_2048HZ 0x0E
#define BMP3XX_ODR_25_4096HZ 0x0F
#define BMP_390_ODR_25_8192HZ 0x10
#define BMP3XX_ODR_25_16384HZ 0x11

#define MAX_TIMEOUT_MS 100

// Power control setting - put sensor in forced mode and enable temperature and pressure measurement
#define FORCED_TEMP_PRESS_MODE 0b00100011

struct BMP3XX_Calibration {
    double par_p11, par_p10, par_p9, par_p8, par_p7, par_p6;
    double par_p5, par_p4, par_p3, par_p2, par_p1;
    double par_t3, par_t2, par_t1, t_lin;
};

struct BMP3XX_Settings {
    uint8_t press_os, temp_os, iir_filter, odr;
};

class BMP3XX {
    public:
        BMP3XX();
        bool begin();
        void reset();
        void configure(uint8_t press_os, uint8_t temp_os, uint8_t iir_filter, uint8_t odr);
        void readCoefficients();
        double readTemperature();
        double readPressure();

    private:
        BMP3XX_Settings settings;
        BMP3XX_Calibration coeffs;
        void configureSensor();
        void writeRegister(uint8_t reg, uint8_t value);
        uint8_t readRegister(uint8_t reg);
        uint32_t read24bit(uint8_t reg);
        double compensateTemp(uint32_t uncompTemp);
        double compensatePressure(uint32_t uncompPressure);
};

#endif
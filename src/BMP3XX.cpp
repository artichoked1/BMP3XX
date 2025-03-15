/*
 *
 * This file is part of the BMP3XX library.
 * Copyright (c) 2025 Nikolai Patrick
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "BMP3XX.h"

BMP3XX::BMP3XX() {}

/**
 * Initialize communication.
 *
 * Inits the sensor and reads the calibration coefficients.
 *
 * @return True if the sensor was successfully initialized.
 *
 */
bool BMP3XX::begin()
{
  Wire.begin();
  reset();
  if (readRegister(REG_ERROR) != 0)
  {
    return false;
  }
  configureSensor();
  readCoefficients();
  return true;
}

/**
 * Perform a soft reset.
 *
 * Writes the reset command to the sensor.
 *
 */
void BMP3XX::reset()
{
  writeRegister(REG_RESET, 0xB6);
  delay(10);
}

/**
 * Configure the sensor.
 *
 * @param press_os Pressure oversampling setting.
 * @param temp_os Temperature oversampling setting.
 * @param iir_filter IIR filter coefficient.
 * @param odr Output data rate.
 *
 */
void BMP3XX::configure(uint8_t press_os, uint8_t temp_os, uint8_t iir_filter, uint8_t odr)
{
  settings.press_os = press_os;
  settings.temp_os = temp_os;
  settings.iir_filter = iir_filter;
  settings.odr = odr;
}

/**
 * Configure the sensor.
 *
 * Writes the oversampling, IIR filter and output data rate settings to the sensor.
 *
 */
void BMP3XX::configureSensor()
{
  writeRegister(REG_OSR, (settings.press_os << 3) | settings.temp_os);
  writeRegister(REG_CONFIG, settings.iir_filter);
  writeRegister(REG_ODR, settings.odr);
}

/**
 * Write a value to a register.
 *
 * @param reg Register address.
 * @param value Value to write.
 *
 */
void BMP3XX::writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(BMP3XX_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

/**
 * Read a register value.
 *
 * @param reg Register address.
 * @return Register value.
 *
 */
uint8_t BMP3XX::readRegister(uint8_t reg)
{
  Wire.beginTransmission(BMP3XX_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BMP3XX_ADDRESS, 1);
  return Wire.read();
}

/**
 * Read a 24-bit value from a register.
 *
 * @param reg Register address.
 * @return 24-bit value.
 *
 */
uint32_t BMP3XX::read24bit(uint8_t reg)
{
  Wire.beginTransmission(BMP3XX_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP3XX_ADDRESS, 3);
  while (Wire.available() < 3)
    ;
  uint32_t xlsb = Wire.read();
  uint32_t lsb = Wire.read();
  uint32_t msb = Wire.read();
  return (msb << 16) | (lsb << 8) | xlsb;
}


/**
 * Read calibration coefficients.
 *
 * Reads the calibration coefficients from the sensor and stores them in the calibration struct.
 *
 */
void BMP3XX::readCoefficients()
{
  uint16_t par_t1 = (uint16_t)readRegister(NVM_PAR_T1_0) | ((uint16_t)readRegister(NVM_PAR_T1_1) << 8);
  int16_t par_t2 = (int16_t)readRegister(NVM_PAR_T2_0) | ((int16_t)readRegister(NVM_PAR_T2_1) << 8);
  int8_t par_t3 = (int8_t)readRegister(NVM_PAR_T3);

  int8_t par_p11 = (int8_t)readRegister(NVM_PAR_P11);
  int8_t par_p10 = (int8_t)readRegister(NVM_PAR_P10);
  int16_t par_p9 = (int16_t)readRegister(NVM_PAR_P9_0) | ((int16_t)readRegister(NVM_PAR_P9_1) << 8);
  int8_t par_p8 = (int8_t)readRegister(NVM_PAR_P8);
  int8_t par_p7 = (int8_t)readRegister(NVM_PAR_P7);
  uint16_t par_p6 = (uint16_t)readRegister(NVM_PAR_P6_0) | ((uint16_t)readRegister(NVM_PAR_P6_1) << 8);
  uint16_t par_p5 = (uint16_t)readRegister(NVM_PAR_P5_0) | ((uint16_t)readRegister(NVM_PAR_P5_1) << 8);
  int8_t par_p4 = (int8_t)readRegister(NVM_PAR_P4);
  int8_t par_p3 = (int8_t)readRegister(NVM_PAR_P3);
  int16_t par_p2 = (int16_t)readRegister(NVM_PAR_P2_0) | ((int16_t)readRegister(NVM_PAR_P2_1) << 8);
  int16_t par_p1 = (int16_t)readRegister(NVM_PAR_P1_0) | ((int16_t)readRegister(NVM_PAR_P1_1) << 8);

  coeffs.par_t1 = par_t1 / 0.00390625f;        // precomputed 2^(-8) = 0.00390625
  coeffs.par_t2 = par_t2 / 1073741824.0f;      // precomputed 2^30 = 1073741824
  coeffs.par_t3 = par_t3 / 281474976710656.0f; // precomputed 2^48 = 281474976710656

  coeffs.par_p11 = par_p11 / 36893488147419103232.0f; // precomputed 2^65 = 36893488147419103232.0
  coeffs.par_p10 = par_p10 / 281474976710656.0f;      // precomputed 2^48 = 281474976710656.0
  coeffs.par_p9 = par_p9 / 281474976710656.0f;        // precomputed 2^48 = 281474976710656.0
  coeffs.par_p8 = par_p8 / 32768.0f;                  // precomputed 2^15 = 32768
  coeffs.par_p7 = par_p7 / 256.0f;                    // precomputed 2^8 = 256
  coeffs.par_p6 = par_p6 / 64.0f;                     // precomputed 2^6 = 64
  coeffs.par_p5 = par_p5 / 0.125f;                    // precomputed 2^(-3) = 0.125
  coeffs.par_p4 = par_p4 / 137438953472.0f;           // precomputed 2^37 = 137438953472
  coeffs.par_p3 = par_p3 / 4294967296.0f;             // precomputed 2^32 = 4294967296
  coeffs.par_p2 = (par_p2 - 16384) / 536870912.0f;    // precomputed 2^14 = 16384, 2^29 = 536870912
  coeffs.par_p1 = (par_p1 - 16384) / 1048576.0f;      // precomputed 2^14 = 16384, 2^20 = 1048576
}

/**
 * Read temperature.
 *
 * Reads the temperature from the sensor and returns the compensated value.
 *
 * @return Compensated temperature value in deg C.
 *
 */
double BMP3XX::readTemperature()
{
  writeRegister(REG_PWR_CTRL, FORCED_TEMP_PRESS_MODE);

  uint8_t status;
  uint32_t startTime = millis();
  do
  {
    status = readRegister(REG_STATUS);
    if (millis() - startTime > MAX_TIMEOUT_MS)
    {
      Serial.println("ERROR: Measurement timeout!");
      return -999;
    }
    delay(5);
  } while (!(status & 0x60));

  uint32_t rawTemp = read24bit(REG_TEMPERATURE);
  return compensateTemp(rawTemp);
}

/**
 * Read pressure.
 *
 * Reads the pressure from the sensor and returns the compensated value.
 *
 * @return Compensated pressure value in Pa.
 *
 */
double BMP3XX::readPressure()
{
  writeRegister(REG_PWR_CTRL, FORCED_TEMP_PRESS_MODE);

  uint8_t status;
  uint32_t startTime = millis();
  do
  {
    status = readRegister(REG_STATUS);
    if (millis() - startTime > MAX_TIMEOUT_MS)
    {
      Serial.println("ERROR: Measurement timeout!");
      return -999;
    }
    delay(5);
  } while (!(status & 0x60));

  uint32_t rawPressure = read24bit(REG_PRESSURE);
  return compensatePressure(rawPressure);
}

/**
 * Do the maths to get temp in deg C from the raw ADC value.
 *
 * @param uncompTemp Uncompensated temperature value.
 * @return Compensated temperature value.
 *
 */
double BMP3XX::compensateTemp(uint32_t uncompTemp)
{
  double partialData1 = (double)(uncompTemp - coeffs.par_t1);
  double partialData2 = partialData1 * coeffs.par_t2;
  coeffs.t_lin = partialData2 + (partialData1 * partialData1) * coeffs.par_t3;
  return coeffs.t_lin;
}

/**
 * Do the maths to get pressure in Pa from the raw ADC value.
 *
 * @param uncompPressure Uncompensated pressure value.
 * @return Compensated pressure value.
 *
 */
double BMP3XX::compensatePressure(uint32_t uncompPressure)
{
  double partialOut1 = coeffs.par_p5 + coeffs.par_p6 * coeffs.t_lin +
                       coeffs.par_p7 * (coeffs.t_lin * coeffs.t_lin) +
                       coeffs.par_p8 * (coeffs.t_lin * coeffs.t_lin * coeffs.t_lin);

  double partialOut2 = (double)uncompPressure *
                       (coeffs.par_p1 + coeffs.par_p2 * coeffs.t_lin +
                        coeffs.par_p3 * (coeffs.t_lin * coeffs.t_lin) +
                        coeffs.par_p4 * (coeffs.t_lin * coeffs.t_lin * coeffs.t_lin));

  double partialData3 = pow((double)uncompPressure, 2) * (coeffs.par_p9 + coeffs.par_p10 * coeffs.t_lin);
  double partialData4 = partialData3 + pow((double)uncompPressure, 3) * coeffs.par_p11;

  return partialOut1 + partialOut2 + partialData4;
}

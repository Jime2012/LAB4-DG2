///////////////////////////////////////////////////////////////////////////
////                                                                   ////
////                               BMP280.c                            ////
////                                                                   ////
////               Driver for mikroC PRO for PIC compiler              ////
////                                                                   ////
//// Driver for Bosch BMP280 sensor. This sensor can read temperature  ////
//// and pressure.                                                     ////
//// This driver only supports I2C mode, it doesn't support SPI mode.  ////
////                                                                   ////
///////////////////////////////////////////////////////////////////////////
////                                                                   ////
////                     https://simple-circuit.com/                   ////
////                                                                   ////
///////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include "I2C.h"
#include "BMP280.h"

#if !defined BMP280_I2C1  &&  !defined BMP280_I2C2
#define  BMP280_I2C1
#endif

#if defined BMP280_I2C1  &&  defined BMP280_I2C2
#undef  BMP280_I2C2
#endif

#ifdef  BMP280_I2C1
#define BMP280_Start    I2C_Master_Start
#define BMP280_Write    I2C_Master_Write
#define BMP280_Read     I2C_Master_Read
#define BMP280_Stop     I2C_Master_Stop
#endif

#ifdef  BMP280_I2C2
#define BMP280_Start    I2C2_Start
#define BMP280_Write    I2C2_Wr
#define BMP280_Read     I2C2_Rd
#define BMP280_Stop     I2C2_Stop
#endif

// writes 1 byte '_data' to register 'reg_addr'
void BMP280_Write8(uint8_t reg_addr, uint8_t _data)
{
  BMP280_Start();
  BMP280_Write(BMP280_I2C_ADDRESS);
  BMP280_Write(reg_addr);
  BMP280_Write(_data);
  BMP280_Stop();
}

// reads 8 bits from register 'reg_addr'
uint8_t BMP280_Read8(uint8_t reg_addr)
{
  uint8_t ret;

  BMP280_Start();
  BMP280_Write(BMP280_I2C_ADDRESS);
  BMP280_Write(reg_addr);
  BMP280_Start();
  BMP280_Write(BMP280_I2C_ADDRESS | 1);
  ret = BMP280_Read(0);
  BMP280_Stop();

  return ret;
}

// reads 16 bits from register 'reg_addr'
uint16_t BMP280_Read16(uint8_t reg_addr)
{
  union
  {
    uint8_t  b[2];
    uint16_t w;
  } ret;

  BMP280_Start();
  BMP280_Write(BMP280_I2C_ADDRESS);
  BMP280_Write(reg_addr);
  BMP280_Start();
  BMP280_Write(BMP280_I2C_ADDRESS | 1);
  ret.b[0] = BMP280_Read(1);
  ret.b[1] = BMP280_Read(0);
  BMP280_Stop();

  return(ret.w);
}

// BMP280 sensor configuration function
void BMP280_Configure(BMP280_mode mode, BMP280_sampling T_sampling,
                      BMP280_sampling P_sampling, BMP280_filter filter, standby_time standby)
{
  uint8_t  _ctrl_meas, _config;

  _config = ((standby << 5) | (filter << 2)) & 0xFC;
  _ctrl_meas = (T_sampling << 5) | (P_sampling << 2) | mode;

  BMP280_Write8(BMP280_REG_CONFIG,  _config);
  BMP280_Write8(BMP280_REG_CONTROL, _ctrl_meas);
}

// initializes the BMP280 sensor, returns 1 if OK and 0 if error
uint8_t BMP280_begin(BMP280_mode mode,
                  BMP280_sampling T_sampling,
                  BMP280_sampling P_sampling,
                  BMP280_filter filter,
                  standby_time  standby)
{
  if(BMP280_Read8(BMP280_REG_CHIPID) != BMP280_CHIP_ID)
    return 0;

  // reset the BMP280 with soft reset
  BMP280_Write8(BMP280_REG_SOFTRESET, 0xB6);
  __delay_ms(100);

  // if NVM data are being copied to image registers, wait 100 ms
  while ( (BMP280_Read8(BMP280_REG_STATUS) & 0x01) == 0x01 )
  __delay_ms(100);

  BMP280_calib.dig_T1 = BMP280_Read16(BMP280_REG_DIG_T1);
  BMP280_calib.dig_T2 = BMP280_Read16(BMP280_REG_DIG_T2);
  BMP280_calib.dig_T3 = BMP280_Read16(BMP280_REG_DIG_T3);

  BMP280_calib.dig_P1 = BMP280_Read16(BMP280_REG_DIG_P1);
  BMP280_calib.dig_P2 = BMP280_Read16(BMP280_REG_DIG_P2);
  BMP280_calib.dig_P3 = BMP280_Read16(BMP280_REG_DIG_P3);
  BMP280_calib.dig_P4 = BMP280_Read16(BMP280_REG_DIG_P4);
  BMP280_calib.dig_P5 = BMP280_Read16(BMP280_REG_DIG_P5);
  BMP280_calib.dig_P6 = BMP280_Read16(BMP280_REG_DIG_P6);
  BMP280_calib.dig_P7 = BMP280_Read16(BMP280_REG_DIG_P7);
  BMP280_calib.dig_P8 = BMP280_Read16(BMP280_REG_DIG_P8);
  BMP280_calib.dig_P9 = BMP280_Read16(BMP280_REG_DIG_P9);

  BMP280_Configure(mode, T_sampling, P_sampling, filter, standby);

  return 1;
}

// Takes a new measurement, for forced mode only!
// Returns 1 if ok and 0 if error (sensor is not in sleep mode)
uint8_t BMP280_ForcedMeasurement()
{
  uint8_t ctrl_meas_reg = BMP280_Read8(BMP280_REG_CONTROL);

  if ( (ctrl_meas_reg & 0x03) != 0x00 )
    return 0;   // sensor is not in sleep mode

  // set sensor to forced mode
  BMP280_Write8(BMP280_REG_CONTROL, ctrl_meas_reg | 1);
  // wait for conversion complete
  while (BMP280_Read8(BMP280_REG_STATUS) & 0x08)
  __delay_ms(1);

  return 1;
}

// read (updates) adc_P, adc_T and adc_H from BMP280 sensor
void BMP280_Update()
{
  union
  {
    uint8_t  b[4];
    uint32_t dw;
  } ret;
  ret.b[3] = 0x00;

  BMP280_Start();
  BMP280_Write(BMP280_I2C_ADDRESS);
  BMP280_Write(BMP280_REG_PRESS_MSB);
  BMP280_Start();
  BMP280_Write(BMP280_I2C_ADDRESS | 1);
  ret.b[2] = BMP280_Read(1);
  ret.b[1] = BMP280_Read(1);
  ret.b[0] = BMP280_Read(1);

  adc_P = (ret.dw >> 4) & 0xFFFFF;  

  ret.b[2] = BMP280_Read(1);
  ret.b[1] = BMP280_Read(1);
  ret.b[0] = BMP280_Read(0);
  BMP280_Stop();

  adc_T = (ret.dw >> 4) & 0xFFFFF;
}

// Reads temperature from BMP280 sensor.
// Temperature is stored in hundredths C (output value of "5123" equals 51.23 DegC).
// Temperature value is saved to *temp, returns 1 if OK and 0 if error.
uint8_t BMP280_readTemperature(int32_t *temp)
{
  int32_t var1, var2;

  BMP280_Update();

  // calculate temperature
  var1 = ((((adc_T / 8) - ((int32_t)BMP280_calib.dig_T1 * 2))) *
         ((int32_t)BMP280_calib.dig_T2)) / 2048;

  var2 = (((((adc_T / 16) - ((int32_t)BMP280_calib.dig_T1)) *
         ((adc_T / 16) - ((int32_t)BMP280_calib.dig_T1))) / 4096) *
         ((int32_t)BMP280_calib.dig_T3)) / 16384;

  t_fine = var1 + var2;

  *temp = (t_fine * 5 + 128) / 256;

  return 1;
}

// Reads pressure from BMP280 sensor.
// Pressure is stored in Pa (output value of "96386" equals 96386 Pa = 963.86 hPa).
// Pressure value is saved to *pres, returns 1 if OK and 0 if error.
uint8_t BMP280_readPressure(uint32_t *pres)
{
  int32_t var1, var2;
  uint32_t p;

  // calculate pressure
  var1 = (((int32_t)t_fine) / 2) - (int32_t)64000;
  var2 = (((var1/4) * (var1/4)) / 2048 ) * ((int32_t)BMP280_calib.dig_P6);

  var2 = var2 + ((var1 * ((int32_t)BMP280_calib.dig_P5)) * 2);
  var2 = (var2/4) + (((int32_t)BMP280_calib.dig_P4) * 65536);

  var1 = ((((int32_t)BMP280_calib.dig_P3 * (((var1/4) * (var1/4)) / 8192 )) / 8) +
         ((((int32_t)BMP280_calib.dig_P2) * var1)/2)) / 262144;

  var1 =((((32768 + var1)) * ((int32_t)BMP280_calib.dig_P1)) / 32768);

  if (var1 == 0)
    return 0; // avoid exception caused by division by zero

  p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 / 4096))) * 3125;

  if (p < 0x80000000)
    p = (p * 2) / ((uint32_t)var1);

  else
    p = (p / (uint32_t)var1) * 2;

  var1 = (((int32_t)BMP280_calib.dig_P9) * ((int32_t)(((p/8) * (p/8)) / 8192))) / 4096;
  var2 = (((int32_t)(p/4)) * ((int32_t)BMP280_calib.dig_P8)) / 8192;

  p = (uint32_t)((int32_t)p + ((var1 + var2 + (int32_t)BMP280_calib.dig_P7) / 16));

  *pres = p;

  return 1;
}

// end of driver code.
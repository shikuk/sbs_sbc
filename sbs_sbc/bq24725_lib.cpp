/*
   BQ24725.c
   ChibiOS HAL driver for using the BQ24725 Battery Charge Controller
   https://github.com/psas/stm32/tree/master/common/devices
*/

#define WIRELIB

#include <Arduino.h>
#include "BQ24725_lib.h"
#ifdef WIRELIB
#include <Wire.h>
#else
#include <i2cmaster.h>
#endif // WIRELIB

#define BQ24725_ADDR    0x09
#define IMON_PIN  A0

#define LOWDATA_BYTE(data) ((data) & 0xFF)
#define HIGHDATA_BYTE(data) (((data) & 0xFF00) >> 8)
#define DATA_FROM_BYTES(low, high) (((low) & 0xFF) | ((high) &0xFF) << 8)

/* REGISTER ADDRESS REGISTER NAME    READ/WRITE 	DESCRIPTION 					POR STATE
  0x12H 				ChargeOption()   Read or Write  Charger Options Control 		0xF902H
  0x14H 				ChargeCurrent()  Read or Write  7-Bit Charge Current Setting 	0x0000H
  0x15H 				ChargeVoltage()  Read or Write  11-Bit Charge Voltage Setting 	0x0000H
  0x3FH 				InputCurrent()   Read or Write  6-Bit Input Current Setting 	0x1000H
  0XFEH 				ManufacturerID() Read Only 		Manufacturer ID 				0x0040H
  0xFFH 				DeviceID()       Read Only 		Device ID 						0x000BH
*/
typedef enum {
  DEVICE_ID       = 0xFF,
  MANUFACTURE_ID  = 0xFE,
  CHARGE_CURRENT  = 0x14,
  CHARGE_VOLTAGE  = 0x15,
  INPUT_CURRENT   = 0x3F,
  CHARGE_OPTION   = 0x12
} BQ24725_register;

#define CHARGE_CURRENT_MASK 0x1FC0
#define CHARGE_VOLTAGE_MASK 0x7FF0
#define INPUT_CURRENT_MASK  0x1F80

//static struct BQ24725Config * CONF;
static bool initialized = false;
static uint16_t imonbuf = 0;

const BQ24725_charge_options BQ24725_charge_options_POR_default = {
  .ACOK_deglitch_time = t150ms,
  .WATCHDOG_timer = t175s,
  .BAT_depletion_threshold = FT70_97pct,
  .EMI_sw_freq_adj = dec18pct,
  .EMI_sw_freq_adj_en = sw_freq_adj_disable,
  .IFAULT_HI_threshold = l700mV,
  .LEARN_en = LEARN_disable,
  .IOUT = adapter_current,
  .ACOC_threshold = l1_66X,
  .charge_inhibit = charge_enable
};

void BQ24725_Start(void) {
  // todo: add interrupt for reload charge voltage every xx(<175) sec
#ifdef WIRELIB
  Wire.begin();
#else
  i2c_init();
#endif
  initialized = 1;

}

int SMBusGet(uint8_t register_id, uint16_t* data) {
  uint8_t low, high;
#ifdef WIRELIB
  Wire.beginTransmission(BQ24725_ADDR);
  Wire.write(register_id);
  Wire.endTransmission(false);
  Wire.requestFrom(BQ24725_ADDR, 2, true);
  low = Wire.read();
  high=  Wire.read();
 // Wire.endTransmission();
  *data = DATA_FROM_BYTES(low, high);
#else
  i2c_start((BQ24725_ADDR << 1) + I2C_WRITE);
  i2c_write(register_id);
  i2c_rep_start((BQ24725_ADDR << 1) + I2C_READ);
  low = i2c_readAck();
  high = i2c_readNak();
  i2c_stop();
#endif

  return OK;
}

int SMBusSet(uint8_t register_id, uint16_t data) {
#ifdef WIRELIB
  Wire.beginTransmission(BQ24725_ADDR);
  Wire.write(register_id);
  Wire.write(data & 0xFF);
  Wire.write((data>>8) & 0xFF);
  Wire.endTransmission();
#else
  i2c_start_wait((BQ24725_ADDR << 1) + I2C_WRITE);
  i2c_write(register_id);
  i2c_write (data & 0xFF);
  i2c_write ((data>>8) & 0xFF);
  i2c_stop();
#endif

  return OK;
}


static int BQ24725_Get(uint8_t register_id, uint16_t* data) {
  if (!initialized) return ERR;
  switch (register_id) {
    case DEVICE_ID:
    case MANUFACTURE_ID:
    case CHARGE_CURRENT:
    case CHARGE_VOLTAGE:
    case INPUT_CURRENT:
    case CHARGE_OPTION:
      return SMBusGet(register_id, data);
    default:
      // chDbgPanic(DBG_PREFIX"Unrecognized BQ24725 read register ID");
      break;
  }
  return ERR;
}

static int BQ24725_Set(uint8_t register_id, uint16_t data) {
  if (!initialized) return ERR;
  switch (register_id) {
    case CHARGE_CURRENT:
    case CHARGE_VOLTAGE:
    case INPUT_CURRENT:
    case CHARGE_OPTION:
      return SMBusSet(register_id, data);
    default:
      //chDbgPanic(DBG_PREFIX"Unrecognized BQ24725 write register ID");
      break;
  }
  return ERR;
}

int BQ24725_GetDeviceID(uint16_t* data) {
  return BQ24725_Get(DEVICE_ID, data);
}

int BQ24725_GetManufactureID(uint16_t* data) {
  return BQ24725_Get(MANUFACTURE_ID, data);
}

int BQ24725_GetChargeCurrent(uint16_t* data) {
  return BQ24725_Get(CHARGE_CURRENT, data);
}

int BQ24725_SetChargeCurrent(unsigned int mA) {
  uint16_t data = mA & CHARGE_CURRENT_MASK;
  return BQ24725_Set(CHARGE_CURRENT, data);
}

int BQ24725_GetChargeVoltage(uint16_t* data) {
  return BQ24725_Get(CHARGE_VOLTAGE, data);
}

int BQ24725_SetChargeVoltage(unsigned int mV) {
  uint16_t data = mV & CHARGE_VOLTAGE_MASK;
  return BQ24725_Set(CHARGE_VOLTAGE, data);
}

int BQ24725_GetInputCurrent(uint16_t* data) {
  return BQ24725_Get(INPUT_CURRENT, data);
}

int BQ24725_SetInputCurrent(unsigned int mA) {
  uint16_t data = mA & INPUT_CURRENT_MASK;
  return BQ24725_Set(INPUT_CURRENT, data);
}

int BQ24725_GetChargeOption(uint16_t* data) {
  return BQ24725_Get(CHARGE_OPTION, data);
}

int BQ24725_SetChargeOption(BQ24725_charge_options * option) {
  uint16_t data = form_options_data(option);
  return BQ24725_Set(CHARGE_OPTION, data);
}

int BQ24725_ACOK(void) {
  return OK; //palReadPad(CONF->ACOK.port, CONF->ACOK.pad);
}

int BQ24725_IMON(void) {
  imonbuf=analogRead (IMON_PIN); // IOUT voltage is 20 times the differential voltage across sense resistor
  return imonbuf;
}

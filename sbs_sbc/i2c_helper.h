/*
   I2C_HELPER - read-write functions
*/
#ifndef I2C_HELPER_H_
#define I2C_HELPER_H_

#define WIRELIB

#include <Arduino.h>
#ifdef WIRELIB
#include <Wire.h>
#else
#include <i2cmaster.h>
#endif // WIRELIB

uint8_t sbsread8  		(uint8_t addr, uint8_t reg, uint8_t* data);
uint8_t sbsread16		(uint8_t addr, uint8_t reg, uint8_t* data);
uint8_t sbsreadString	(uint8_t addr, uint8_t reg, uint8_t* data);
// after write read this register and put result to data
uint8_t sbswrite8 		(uint8_t addr, uint8_t reg, uint8_t* data);
uint8_t sbswrite16		(uint8_t addr, uint8_t reg, uint8_t* data);
uint8_t sbswriteString	(uint8_t addr, uint8_t reg,  uint8_t len,  uint8_t* data);
// check for reg can be readed
uint8_t Check_Reg(uint8_t address, uint8_t reg);
uint8_t readAndReportData(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data, uint8_t stopTX);
// battery special functions
uint16_t read16uManuf (uint16_t reg);
uint16_t write16uManuf (uint16_t reg);


#endif

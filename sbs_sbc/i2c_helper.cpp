#include "i2c_helper.h"
#include "Arduino.h"

#define OK	1
#define ERR	0

#define SBS_Address           (byte)0x0B
#define SBS_ManAccess         (byte)0x00

/////////////////////////////////////////////////////////////////////////////
// Functions Below

uint8_t sbsread8 (uint8_t addr, uint8_t reg, uint8_t* data) {
  data[0] = addr;
  data[1] = reg;
  data[3] = '0';
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (byte)1);
  data[2] = Wire.read();
  Wire.endTransmission();
  return OK;
}

uint8_t sbsread16 (uint8_t addr, uint8_t reg, uint8_t* data) {
  data[0] = addr;
  data[1] = reg;
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (byte)2, (byte)1);
  data[2] = Wire.read();
  data[3] = Wire.read();
  Wire.endTransmission();
  return OK;
}

// pass a pointer to a char[] that can take up to 33 chars
// will return the length of the string received
uint8_t sbsreadString(uint8_t addr, uint8_t reg, uint8_t* data) {
  uint8_t pos = 0;
  uint8_t len;

  // Read the length of the string
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (byte)1);
  len = Wire.read();    // length of the string
  len++;            // plus one to allow for the length byte on the reread
  // if len > 32 then the it will be truncated to 32 by requestFrom

  // Now that we know the length, repeat the read to get all the string data.
  // we need to write the address again and do a restart so its a valid SMBus transaction
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  len = Wire.requestFrom(addr, len);    // readRequest returns # bytes actually read

  len--;                                             // we won't move the first byte as its not part of the string
  if (len > 0)
  {
    Wire.read();
    for (pos = 0; pos < len; pos++)
      data[pos] = Wire.read();
  }
  data[pos] = '\0';  // append the zero terminator

  return len;
}



uint8_t sbswrite8	(uint8_t addr, uint8_t reg, uint8_t* data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data[0]);
  Wire.endTransmission();
  sbsread8 (addr, reg, data);
  return OK;
}

uint8_t sbswrite16	(uint8_t addr, uint8_t reg, uint8_t* data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data[1] & 0xFF);
  Wire.write(data[0] & 0xFF);
  Wire.endTransmission();
  sbsread16 (addr, reg, data);
  return OK;
}

uint8_t sbswriteString	(uint8_t addr, uint8_t reg,  uint8_t len,  uint8_t* data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++)
    Wire.write(data[i]);
  Wire.endTransmission();
  len = sbsreadString (addr, reg, data);
  return len;
}



uint8_t Check_Reg(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false))
    return 0;
  else {
    Wire.requestFrom(address, (byte)1);
    Wire.read();
    Wire.endTransmission();
    return 1;
  }
}

uint8_t readAndReportData(uint8_t addr, uint8_t reg, uint8_t len,  uint8_t* data, uint8_t stopTX) {
  uint8_t i;
  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(stopTX); // default = true
  // do not set a value of 0
  // delay is necessary for some devices such as WiiNunchuck
  delayMicroseconds(1);

  Wire.requestFrom(addr, len);  // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if (len < Wire.available()) {
    Serial.println("I2C: Too many bytes received");
  } else if (len > Wire.available()) {
    Serial.println("I2C: Too few bytes received");
  }

  data[0] = addr;
  data[1] = reg;

  for (i = 0; i < len && Wire.available(); i++) {
    data[2 + i] = Wire.read();
  }
  return i;
}

uint16_t read16uManuf(uint16_t reg) {
  uint16_t registerValue;

  Wire.beginTransmission(SBS_Address);
  Wire.write(SBS_ManAccess);
  Wire.write(reg & 0xFF);
  Wire.write(reg >> 8 & 0xFF);
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(SBS_Address);
  Wire.write(SBS_ManAccess);
  Wire.endTransmission(false);
  Wire.requestFrom(SBS_Address, (byte)2);
  registerValue = Wire.read();
  registerValue |= (Wire.read() << 8);

  return registerValue;

}

void write16uManuf(uint16_t reg) {

  Wire.beginTransmission(SBS_Address);
  Wire.write(SBS_ManAccess);
  Wire.write(reg & 0xFF);
  Wire.write(reg >> 8 & 0xFF);
  Wire.endTransmission();

}

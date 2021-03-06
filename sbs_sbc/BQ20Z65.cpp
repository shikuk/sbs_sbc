#include "BQ20Z65.h"
#include "Arduino.h"


/////////////////////////////////////////////////////////////////////////////
// Functions Below

void BQ20Z65::write(uint8_t address, uint8_t data)
{
  Wire.beginTransmission(BQ20Z65_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

void BQ20Z65::write16(uint8_t address, uint16_t data)
{
  Wire.beginTransmission(BQ20Z65_Address);
  Wire.write(address);
  Wire.write(data & 0xFF);
  Wire.write(data >> 8 & 0xFF);
  Wire.endTransmission();
}

uint8_t BQ20Z65::read(uint8_t address)
{
  uint8_t registerValue;
  Wire.beginTransmission(BQ20Z65_Address);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BQ20Z65_Address, 1);
  registerValue = Wire.read();
  Wire.endTransmission();
  return registerValue;
}

uint16_t BQ20Z65::read16u(uint8_t address)
{
  uint16_t registerValue;
  Wire.beginTransmission(BQ20Z65_Address);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom(BQ20Z65_Address, 2);
  registerValue = Wire.read();
  registerValue |= (Wire.read() << 8);

  return registerValue;
}


uint16_t BQ20Z65::read16u2(uint8_t address)
{
  uint16_t registerValue;
  Wire.beginTransmission(BQ20Z65_Address);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom(BQ20Z65_Address, 4);
  Wire.read();
  registerValue = Wire.read();
  registerValue |= (Wire.read() << 8);;

  return registerValue;
}

int16_t BQ20Z65::read16(uint8_t address)
{
  int16_t registerValue;
  Wire.beginTransmission(BQ20Z65_Address);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom(BQ20Z65_Address, 2);
  registerValue = Wire.read();
  registerValue += (Wire.read() * 256);

  return registerValue;
}

uint32_t BQ20Z65::read32u(uint8_t address)
{
  uint32_t registerValue;
  Wire.beginTransmission(BQ20Z65_Address);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom(BQ20Z65_Address, 5);
  Wire.read();
  registerValue = Wire.read();
  registerValue |= (Wire.read() << 8);
  registerValue |= ((uint32_t)Wire.read() << 16);
  registerValue |= ((uint32_t)Wire.read() << 24);

  return registerValue;
}


// pass a pointer to a char[] that can take up to 33 chars
// will return the length of the string received
uint8_t BQ20Z65::readString(uint8_t address, uint8_t* result)
{
  uint8_t pos = 0;
  uint8_t len;

  // Read the length of the string
  Wire.beginTransmission(BQ20Z65_Address);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom(BQ20Z65_Address, 1);
  len = Wire.read();    // length of the string
  len++;            // plus one to allow for the length byte on the reread
  // if len > 32 then the it will be truncated to 32 by requestFrom

  // Now that we know the length, repeat the read to get all the string data.
  // we need to write the address again and do a restart so its a valid SMBus transaction
  Wire.beginTransmission(BQ20Z65_Address);
  Wire.write(address);
  Wire.endTransmission(false);
  len = Wire.requestFrom(BQ20Z65_Address, len);    // readRequest returns # bytes actually read

  len--;                                             // we won't move the first byte as its not part of the string
  if (len > 0)
  {
    Wire.read();
    for (pos = 0; pos < len; pos++)
      result[pos] = Wire.read();
  }
  result[pos] = '\0';  // append the zero terminator

  return len;
}

/////////////////////////////////////////////////////////////////////////////
// Class Methods Below


uint16_t BQ20Z65::GetTemp()
{
  return read16u(BQ20Z65_Temp);
}

uint16_t BQ20Z65::GetVoltage()
{
  return read16u(BQ20Z65_Volt);
}

int16_t BQ20Z65::GetCurrent()
{
  return read16(BQ20Z65_Current);
}
int16_t BQ20Z65::AverageCurrent()
{
  return read16(BQ20Z65_AveCurrent);
}
uint8_t BQ20Z65::RelativeSOC()
{
  return read(BQ20Z65_RelativeSOC);
}
uint8_t BQ20Z65::AbsoluteSOC()
{
  return read(BQ20Z65_AbsoluteSOC);
}
uint16_t BQ20Z65::RemainingCapAlarm()
{
  return read16u(BQ20Z65_RemainCapAlarm);
}
uint16_t BQ20Z65::RemainingTimeAlarm()
{
  return read16u(BQ20Z65_RemainTimeAlarm);
}
int16_t BQ20Z65::AtRate()
{
  return read16(BQ20Z65_AtRate);
}
uint8_t BQ20Z65::MaxError()
{
  return read(BQ20Z65_MaxError);
}
uint16_t BQ20Z65::AtRateTimeToFull()
{
  return read16u(BQ20Z65_AtRateTimeToFull);
}
uint16_t BQ20Z65::AtRateTimeToEmpty()
{
  return read16u(BQ20Z65_AtRateTimeToEmpty);
}
uint16_t BQ20Z65::AtRateOK()
{
  return read16u(BQ20Z65_AtRateOK);
}
uint16_t BQ20Z65::RemainingBatteryCapacity()
{
  return read16u(BQ20Z65_RemCap );
}
uint16_t BQ20Z65::FullBatteryCapacity()
{
  return read16u(BQ20Z65_FullChargCap );
}
uint16_t BQ20Z65::RunTimeTillEmpty()
{
  return read16u(BQ20Z65_RunTime2Empty );
}
uint16_t BQ20Z65::AverageTimeTillEmpty()
{
  return read16u(BQ20Z65_AveTime2Empty );
}
uint16_t BQ20Z65::AverageTimeTillFull()
{
  return read16u(BQ20Z65_AveTime2Full );
}
uint16_t BQ20Z65::ChargingCurrent()
{
  return read16u(BQ20Z65_ChargCurrent );
}
uint16_t BQ20Z65::ChargingVoltage()
{
  return read16u(BQ20Z65_ChargVolt );
}
uint16_t BQ20Z65::CycleCount()
{
  return read16u(BQ20Z65_CycleCount );
}
uint16_t BQ20Z65::DesignCapacity()
{
  return read16u(BQ20Z65_DesignCapacity );
}
uint16_t BQ20Z65::DesignVoltage()
{
  return read16u(BQ20Z65_DesignVoltage );
}
uint16_t BQ20Z65::CellVoltage1()
{
  return read16u(BQ20Z65_CellVolt1 );
}
uint16_t BQ20Z65::CellVoltage2()
{
  return read16u(BQ20Z65_CellVolt2 );
}
uint16_t BQ20Z65::CellVoltage3()
{
  return read16u(BQ20Z65_CellVolt3 );
}
uint16_t BQ20Z65::CellVoltage4()
{
  return read16u(BQ20Z65_CellVolt4 );
}
uint8_t BQ20Z65::StateOfHealth()
{
  return read(BQ20Z65_StateOfHealth);
}
uint16_t BQ20Z65::BatteryStatus()
{
  return read16u(BQ20Z65_BatteryStatus);
}
uint16_t BQ20Z65::SafetyAlert()
{
  return read32u(BQ20Z65_SafetyAlert);
}
uint32_t BQ20Z65::SafetyStatus()
{
  return read32u(BQ20Z65_SafetyStatus);
}
uint16_t BQ20Z65::PFAlert()
{
  return read16u2(BQ20Z65_PFAlert);
}

uint16_t BQ20Z65::PFStatus()
{
  return read16u2(BQ20Z65_PFStatus);
}

uint16_t BQ20Z65::OperationStatus()
{
  return read16u(BQ20Z65_OperationStatus);
}

uint16_t BQ20Z65::ChargingStatus()
{
  return read16u(BQ20Z65_ChargingStatus);
}

uint16_t BQ20Z65::BattMode()
{
  return read16u(BQ20Z65_BattMode);
}

uint16_t BQ20Z65::SpecificationInfo(void) {
  return read16u(BQ20Z65_SpecificationInfo);
}

uint16_t BQ20Z65::ManufactureDate(void) {
  return read16u(BQ20Z65_ManufactureDate);
}

uint16_t BQ20Z65::SerialNumber(void) {
  return read16u(BQ20Z65_SerialNumber);
}

uint8_t BQ20Z65::ManufactureName(uint8_t* buffer) {
  return readString(BQ20Z65_ManufactureName, buffer);
}

uint8_t BQ20Z65::DeviceName(uint8_t* buffer) {
  return readString(BQ20Z65_DeviceName, buffer);
}

uint8_t BQ20Z65::DeviceChemistry(uint8_t* buffer) {
  return readString(BQ20Z65_DeviceChemistry, buffer);
}

uint8_t BQ20Z65::ManufactureData(uint8_t* buffer) {
  return readString(BQ20Z65_ManufactureData, buffer);
}
uint32_t BQ20Z65::GetUnsealKey(void)
{
  return read32u(BQ20Z65_UnSealKey);
}
uint32_t BQ20Z65::FullAccessKey(void)
{
  return read32u(BQ20Z65_FullAccessKey);
}
uint32_t BQ20Z65::PFClearKey(void)
{
  return read32u(BQ20Z65_PFKey);
}

uint32_t BQ20Z65::PFClearKey_v2(void)
{
  uint32_t registerValue=0;
  Wire.beginTransmission(BQ20Z65_Address);
  Wire.write(BQ20Z65_PFKey);
  Wire.endTransmission(false);
  Wire.requestFrom(BQ20Z65_Address, 5);
  Wire.read();
  registerValue = Wire.read();
  registerValue |= (Wire.read() << 8);
  registerValue |= (Wire.read() << 16);
  registerValue |= (Wire.read() << 24);

  return registerValue;
}




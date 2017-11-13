#include "BQ20Z45.h"
#include "Arduino.h"


/////////////////////////////////////////////////////////////////////////////
// Functions Below

void BQ20Z45::write(uint8_t address, uint8_t data)
{
  Wire.beginTransmission(BQ20Z45_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t BQ20Z45::read(uint8_t address)
{
	uint8_t registerValue;
      Wire.beginTransmission(BQ20Z45_Address);
	Wire.write(address);
	Wire.endTransmission();
	Wire.requestFrom(BQ20Z45_Address,1,true);
	registerValue = Wire.read();
	Wire.endTransmission();
        return registerValue;
}

uint16_t BQ20Z45::read16u(uint8_t address)
{
	uint16_t registerValue;
      Wire.beginTransmission(BQ20Z45_Address);
	Wire.write(address);
	Wire.endTransmission(false);
	Wire.requestFrom(BQ20Z45_Address,2,true);
	registerValue = Wire.read();
     registerValue |= (Wire.read()<<8);
	
        return registerValue;
}

uint16_t BQ20Z45::read16u2(uint8_t address)
{
	uint16_t registerValue;
      Wire.beginTransmission(BQ20Z45_Address);
	Wire.write(address);
	Wire.endTransmission(false);
	Wire.requestFrom(BQ20Z45_Address,4,true);
	Wire.read();
        registerValue = Wire.read();
        registerValue |= (Wire.read()<<8);;
	
        return registerValue;
}

int16_t BQ20Z45::read16(uint8_t address)
{
	int16_t registerValue;
        Wire.beginTransmission(BQ20Z45_Address);
	Wire.write(address);
	Wire.endTransmission(false);
	Wire.requestFrom(BQ20Z45_Address,2,true);
	registerValue = Wire.read();
        registerValue += (Wire.read()*256);

        return registerValue;
}

uint32_t BQ20Z45::read32u(uint8_t address)
{
	uint32_t registerValue;
      Wire.beginTransmission(BQ20Z45_Address);
	Wire.write(address);
	Wire.endTransmission(false);
	Wire.requestFrom(BQ20Z45_Address,5,true);
	    Wire.read();
        registerValue = Wire.read();
        registerValue |= (Wire.read()<<8);
        registerValue |= ((uint32_t)Wire.read() << 16);
        registerValue |= ((uint32_t)Wire.read() << 24);
		
        return registerValue;
}

uint8_t BQ20Z45::Check_Reg(uint8_t address, uint8_t reg)
{
   Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission()==0){
  Wire.requestFrom(address, 1, 1);
  Wire.read();
  Wire.endTransmission();
  return 1;
  }
        else return 0;
}

// pass a pointer to a char[] that can take up to 33 chars
// will return the length of the string received
int BQ20Z45::readString(uint8_t address, char* result)
{
	int pos = 0;
	int len;

        // Read the length of the string
	Wire.beginTransmission(BQ20Z45_Address);
	Wire.write(address);
	Wire.endTransmission(false);
	Wire.requestFrom(BQ20Z45_Address, 1, true);
	len = Wire.read();    // length of the string
        len++;            // plus one to allow for the length byte on the reread
                          // if len > 32 then the it will be truncated to 32 by requestFrom

        // Now that we know the length, repeat the read to get all the string data. 
        // we need to write the address again and do a restart so its a valid SMBus transaction
	Wire.beginTransmission(BQ20Z45_Address);
	Wire.write(address);
	Wire.endTransmission(false);
	len = Wire.requestFrom(BQ20Z45_Address, len, true);    // readRequest returns # bytes actually read

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


uint16_t BQ20Z45::GetTemp()
{
	return read16u(BQ20Z45_Temp);
}

uint16_t BQ20Z45::GetVoltage()
{
	return read16u(BQ20Z45_Volt);
}

int16_t BQ20Z45::GetCurrent()
{
	return read16(BQ20Z45_Current);
}
int16_t BQ20Z45::AverageCurrent()
{
	return read16(BQ20Z45_AveCurrent);
}
uint8_t BQ20Z45::RelativeSOC()
{
	return read(BQ20Z45_RelativeSOC);
}
uint8_t BQ20Z45::AbsoluteSOC()
{
	return read(BQ20Z45_AbsoluteSOC);
}
uint16_t BQ20Z45::RemainingCapAlarm()
{
	return read16u(BQ20Z45_RemainCapAlarm);
}
uint16_t BQ20Z45::RemainingTimeAlarm()
{
	return read16u(BQ20Z45_RemainTimeAlarm);
}
uint16_t BQ20Z45::AtRate()
{
	return read16(BQ20Z45_AtRate);
}
uint8_t BQ20Z45::MaxError()
{
	return read(BQ20Z45_MaxError);
}
uint16_t BQ20Z45::AtRateTimeToFull()
{
	return read16u(BQ20Z45_AtRateTimeToFull);
}
uint16_t BQ20Z45::AtRateTimeToEmpty()
{
	return read16u(BQ20Z45_AtRateTimeToEmpty);
}
uint16_t BQ20Z45::AtRateOK()
{
 	return read16u(BQ20Z45_AtRateOK);
}
uint16_t BQ20Z45::RemainingBatteryCapacity()
{
	return read16u(BQ20Z45_RemCap );
}
uint16_t BQ20Z45::FullBatteryCapacity()
{
	return read16u(BQ20Z45_FullChargCap );
}
uint16_t BQ20Z45::RunTimeTillEmpty()
{
	return read16u(BQ20Z45_RunTime2Empty );
}
uint16_t BQ20Z45::AverageTimeTillEmpty()
{
	return read16u(BQ20Z45_AveTime2Empty );	
}
uint16_t BQ20Z45::AverageTimeTillFull()
{
	return read16u(BQ20Z45_AveTime2Full );	
}
uint16_t BQ20Z45::ChargingCurrent()
{
	return read16u(BQ20Z45_ChargCurrent );	
}
uint16_t BQ20Z45::ChargingVoltage()
{
	return read16u(BQ20Z45_ChargVolt );	
}
uint16_t BQ20Z45::CycleCount()
{
	return read16u(BQ20Z45_CycleCount );	
}
uint16_t BQ20Z45::DesignCapacity()
{
	return read16u(BQ20Z45_DesignCapacity );	
}
uint16_t BQ20Z45::DesignVoltage()
{
	return read16u(BQ20Z45_DesignVoltage );	
}
uint16_t BQ20Z45::CellVoltage1()
{
	return read16u(BQ20Z45_CellVolt1 );	
}
uint16_t BQ20Z45::CellVoltage2()
{
	return read16u(BQ20Z45_CellVolt2 );	
}
uint16_t BQ20Z45::CellVoltage3()
{
	return read16u(BQ20Z45_CellVolt3 );	
}
uint16_t BQ20Z45::CellVoltage4()
{
	return read16u(BQ20Z45_CellVolt4 );	
}
uint16_t BQ20Z45::PendingEVD()
{
	return read16u(BQ20Z45_PendingEDV );	
}
uint8_t BQ20Z45::StateOfHealth()
{
	return read(BQ20Z45_StateOfHealth);
}

uint16_t BQ20Z45::BatteryStatus()
{
	return read16u(BQ20Z45_BatteryStatus);
}

uint32_t BQ20Z45::SafetyAlert()
{
	return read32u(BQ20Z45_SafetyAlert);
}

uint32_t BQ20Z45::SafetyStatus()
{
	return read32u(BQ20Z45_SafetyStatus);
}

uint16_t BQ20Z45::PFAlert()
{
	return read16u2(BQ20Z45_PFAlert);
}

uint16_t BQ20Z45::PFStatus()
{
 	return read16u2(BQ20Z45_PFStatus);
}

uint32_t BQ20Z45::OperationStatus()
{
 	return read32u(BQ20Z45_OperationStatus);
}

uint16_t BQ20Z45::ChargingStatus()
{
 	return read16u(BQ20Z45_ChargingStatus);
}

uint16_t BQ20Z45::BattMode()
{
 	return read16u(BQ20Z45_BattMode);
}

//int BQ20Z45::readString(uint8_t address, char* result) /// This isn't all thats needed here right? I don't know what should go below. HELP!

#ifndef BQ20Z45_h
#define BQ20Z45_h

#include <Wire.h>

#define BQ20Z45_Address           0x0B

#define BQ20Z45_ManAccess         0x00
#define BQ20Z45_RemainCapAlarm    0x01 // If RemainingCapacity < RemainingCapacityAlarm, [RCA] flag is AlarmWarning message to SMBUS host.
#define BQ20Z45_RemainTimeAlarm   0x02 // If AverageTimeToEmpty < RemainingTimeAlarm, [RTA] flag is set  AlarmWarning message to SMBus host.
#define BQ20Z45_BattMode          0x03 //
#define BQ20Z45_AtRate            0x04
#define BQ20Z45_AtRateTimeToFull  0x05
#define BQ20Z45_AtRateTimeToEmpty 0x06
#define BQ20Z45_AtRateOK          0x07
#define BQ20Z45_Temp              0x08
#define BQ20Z45_Volt              0x09
#define BQ20Z45_Current           0x0A
#define BQ20Z45_AveCurrent        0x0B
#define BQ20Z45_MaxError          0x0C
#define BQ20Z45_RelativeSOC       0x0D
#define BQ20Z45_AbsoluteSOC       0x0E
#define BQ20Z45_RemCap            0x0F
#define BQ20Z45_FullChargCap      0x10
#define BQ20Z45_RunTime2Empty     0x11
#define BQ20Z45_AveTime2Empty     0x12
#define BQ20Z45_AveTime2Full      0x13
#define BQ20Z45_ChargCurrent      0x14
#define BQ20Z45_ChargVolt         0x15
#define BQ20Z45_BatteryStatus     0x16
#define BQ20Z45_CycleCount        0x17
#define BQ20Z45_DesignCapacity    0x18
#define BQ20Z45_DesignVoltage     0x19
#define BQ20Z45_SpecificationInfo 0x1A
#define BQ20Z45_ManufactureDate   0x1B
#define BQ20Z45_SerialNumber      0x1C
#define BQ20Z45_ManufactureName   0x20
#define BQ20Z45_DeviceName        0x21
#define BQ20Z45_DeviceChemistry   0x22
#define BQ20Z45_ManufactureData   0x23
#define BQ20Z45_HostFETControl    0x2B
#define BQ20Z45_GPIOStauts        0x2C
#define BQ20Z45_GPIOControl       0x2D
#define BQ20Z45_VAUXVoltage       0x2E
#define BQ20Z45_SerialNumber      0x1C

#define BQ20Z45_CellVolt1         0x3F
#define BQ20Z45_CellVolt2         0x3E
#define BQ20Z45_CellVolt3         0x3D
#define BQ20Z45_CellVolt4         0x3C
#define BQ20Z45_CellVolt5         0x3B
#define BQ20Z45_CellVolt6         0x3A
#define BQ20Z45_CellVolt7         0x39
#define BQ20Z45_CellVolt8         0x38

#define BQ20Z45_ExtAveCellVoltage 0x4D
#define BQ20Z45_PendingEDV        0x4E
#define BQ20Z45_StateOfHealth     0x4F
#define BQ20Z45_SafetyAlert       0x50
#define BQ20Z45_SafetyStatus      0x51
#define BQ20Z45_PFAlert           0x52
#define BQ20Z45_PFStatus          0x53
#define BQ20Z45_OperationStatus   0x54
#define BQ20Z45_ChargingStatus    0x55
#define BQ20Z45_GaugingStatus     0x56
#define BQ20Z45_AFEStatus         0x58
#define BQ20Z45_AFEConfig         0x59
#define BQ20Z45_AFEVCx            0x5A
#define BQ20Z45_AFEData           0x5B

#define BQ20Z45_LifetimeDataBlock1 0x60
#define BQ20Z45_LifetimeDataBlock2 0x61
#define BQ20Z45_LifetimeDataBlock3 0x62
#define BQ20Z45_LifetimeDataBlock4 0x63
#define BQ20Z45_LifetimeDataBlock5 0x64
#define BQ20Z45_LifetimeDataBlock6 0x65

#define BQ20Z45_ManufacturerInfo   0x70
#define BQ20Z45_SenseResistor      0x71	
#define BQ20Z45_TempRange          0x72
#define BQ20Z45_CUVSnapshot        0x80
#define BQ20Z45_COVSnapshot        0x81

class BQ20Z45
{
	public:

	
	uint16_t GetTemp(void);
  uint16_t GetVoltage(void);
    int16_t GetCurrent(void);
	int16_t AverageCurrent(void);
	uint8_t RelativeSOC(void);	
	uint8_t AbsoluteSOC(void);
	uint16_t RemainingCapAlarm(void); 
	uint16_t RemainingTimeAlarm(void);
	uint16_t AtRate(void);
	uint16_t AtRateTimeToFull(void);
	uint16_t AtRateTimeToEmpty(void);
	uint16_t AtRateOK(void);
	uint8_t MaxError(void);
	uint16_t RemainingBatteryCapacity(void);
	uint16_t FullBatteryCapacity(void);
	uint16_t RunTimeTillEmpty(void);
	uint16_t AverageTimeTillEmpty(void);
	uint16_t AverageTimeTillFull(void);
	uint16_t ChargingCurrent(void);
	uint16_t ChargingVoltage(void);
	uint16_t CycleCount(void);
	uint16_t DesignCapacity(void);
    uint16_t DesignVoltage(void);
	uint16_t CellVoltage1(void);
	uint16_t CellVoltage2(void);
    uint16_t CellVoltage3(void);
    uint16_t CellVoltage4(void);
    uint16_t PendingEVD(void);
	uint8_t StateOfHealth(void);
	uint16_t BatteryStatus(void);
    uint32_t SafetyAlert(void);
    uint32_t SafetyStatus(void);
	uint16_t PFAlert(void); 
    uint16_t PFStatus(void);
	uint32_t OperationStatus(void);
	uint16_t ChargingStatus(void);
	int readString(uint8_t address, char* result);
	uint16_t BattMode (void);

  uint8_t Check_Reg(uint8_t address, uint8_t reg);

	
	protected:
	  
	private:
	  
	void write(uint8_t,uint8_t);
	uint8_t read(uint8_t);	
    uint16_t read16u(uint8_t);
    int16_t read16(uint8_t);
	uint32_t read32u(uint8_t);
	uint16_t read16u2(uint8_t);
    
};
#endif

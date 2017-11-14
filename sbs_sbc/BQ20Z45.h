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
#define BQ20Z45_MaxError          0x0C  // todo: HIGH is bad???
#define BQ20Z45_RelativeSOC       0x0D
#define BQ20Z45_AbsoluteSOC       0x0E
#define BQ20Z45_RemCap            0x0F
#define BQ20Z45_FullChargCap      0x10
#define BQ20Z45_RunTime2Empty     0x11  // todo: search difference with BQ20Z45_AtRateTimeToEmpty
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

/* Battery mode bitmap */
#define MODE_INTERNAL_CHARGE_CONTROLLER (1 << 0)
#define MODE_PRIMARY_BATTERY_SUPPORT    (1 << 1)
#define MODE_CONDITION_CYCLE            (1 << 7)  // 0 = Battery OK  1 = Condition cycle requested
#define MODE_CHARGE_CONTROLLER_ENABLED  (1 << 8)
#define MODE_PRIMARY_BATTERY            (1 << 9)
#define MODE_ALARM                      (1 << 13) // enable or disable AlarmWarning
#define MODE_CHARGER                    (1 << 14) // Enables or disables transmission of ChargingCurrent and ChargingVoltage Charger
#define MODE_CAPACITY                   (1 << 15) // 0 = Reports in mA or mAh (default) 1 = Reports in 10mW or 10mWh

/* Manuf Access registers */
#define Device_Type           0x0001
#define Firmware_Version      0x0002
#define Hardware_Version      0x0003
#define Manufacturer_Status   0x0006

/* Battery manuf status bitmap */
#define STATE0              (1 << 8)
#define STATE1              (1 << 9)
#define STATE2              (1 << 10) 
#define STATE3              (1 << 11)
#define PF0                 (1 << 12) // Cell imbalance permanent failure 0,0 fuse blown
#define PF1                 (1 << 13) // Safety voltage failure 1,1 FET failure
#define FET0                (1 << 14) // 0 = DSG FET is off 1 = DSG FET is on
#define FET1                (1 << 15) // 0 = CHG FET is off 1 = CHG FET is on

/* states 
* STATE3, STATE2, STATE1, STATE0 — Indicates the battery state.
* 0,0,0,0 = Wake Up
* 0,0,0,1 = Normal Discharge
* 0,0,1,1 = Pre-Charge
* 0,1,0,1 = Charge
* 0,1,1,1 = Charge Termination
* 1,0,0,1 = Permanent Failure
* 1,0,1,0 = Overcurrent
* 1,0,1,1 = Overtemperature
* 1,1,0,0 = Battery Failure
* 1,1,0,1 = Sleep
* 1,1,1,0 = Reserved
* 1,1,1,1 = Battery Pack Removed
*/ 
#define WAKE_UP       0x0
#define NORM_DISCH      0x1
#define PRECHARGE     0x3
#define CHARGE        0x5
#define CHARGETERMINATION 0x7
#define PERMANENTFAIL   0x9
#define OVERCURRENT     0xA
#define OVERTEMP      0xB
#define BATTFAIL      0xC
#define SLEEP       0xD
#define PACKREMOVED     0xF
/* FET1, FET0 — Indicates the state of the charge and discharge FETs
0,0 = Both charge and discharge FETs are on.
0,1 = CHG FET is off, DSG FET is on.
1,0 = Both charge and discharge FETs are off.
1,1 = CHG FET is on, DSG FET is off.
*/
#define BOTH_ON       0x0
#define DISCH_ON      0x1
#define BOTH_OFF      0x2
#define CHG_ON        0x3
/*PF1, PF0 — Indicates permanent failure cause when permanent failure indicated by STATE3..STATE0
0,0 = Fuse is blown if enabled via DF:Configuration:Registers(64):Permanent Fail Cfg(6)
0,1 = Cell imbalance failure
1,0 = Safety voltage failure
1,1 = FET failure 
*/
#define FUSE_BLOWN      0x0
#define CELL_IMBALANCE    0x1
#define VOLT_FAIL     0x2
#define FET_FAIL      0x3


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
  int16_t AtRate(void);
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
  uint16_t BattMode (void);
  uint16_t SpecificationInfo(void);
  uint16_t ManufactureDate(void);
  uint16_t SerialNumber(void);
  uint8_t ManufactureName(uint8_t* buffer);
  uint8_t DeviceName(uint8_t* buffer);
  uint8_t DeviceChemistry(uint8_t* buffer);
  uint8_t ManufactureData(uint8_t* buffer);

  uint8_t Check_Reg(uint8_t address, uint8_t reg);
    uint16_t read16uManuf(uint16_t reg);
  
  protected:
    
  private:
    
  void write(uint8_t,uint8_t);
  uint8_t read(uint8_t);  
    uint16_t read16u(uint8_t);
    int16_t read16(uint8_t);
  uint32_t read32u(uint8_t);
  uint16_t read16u2(uint8_t);
  void write16(uint8_t address, uint16_t data);
  uint8_t readString(uint8_t address, uint8_t* result);

};
#endif

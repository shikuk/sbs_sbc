#ifndef BQ20Z65_h
#define BQ20Z65_h

#include <Wire.h>

#define BQ20Z65_Address           0x0B

#define BQ20Z65_ManAccess         0x00
#define BQ20Z65_RemainCapAlarm    0x01 // If RemainingCapacity < RemainingCapacityAlarm, [RCA] flag is AlarmWarning message to SMBUS host.
#define BQ20Z65_RemainTimeAlarm   0x02 // If AverageTimeToEmpty < RemainingTimeAlarm, [RTA] flag is set  AlarmWarning message to SMBus host.
#define BQ20Z65_BattMode          0x03 //
#define BQ20Z65_AtRate            0x04
#define BQ20Z65_AtRateTimeToFull  0x05
#define BQ20Z65_AtRateTimeToEmpty 0x06
#define BQ20Z65_AtRateOK          0x07
#define BQ20Z65_Temp              0x08
#define BQ20Z65_Volt              0x09
#define BQ20Z65_Current           0x0A
#define BQ20Z65_AveCurrent        0x0B
#define BQ20Z65_MaxError          0x0C  // todo: HIGH is bad???
#define BQ20Z65_RelativeSOC       0x0D
#define BQ20Z65_AbsoluteSOC       0x0E
#define BQ20Z65_RemCap            0x0F
#define BQ20Z65_FullChargCap      0x10
#define BQ20Z65_RunTime2Empty     0x11  // todo: search difference with BQ20Z65_AtRateTimeToEmpty
#define BQ20Z65_AveTime2Empty     0x12
#define BQ20Z65_AveTime2Full      0x13
#define BQ20Z65_ChargCurrent      0x14
#define BQ20Z65_ChargVolt         0x15
#define BQ20Z65_BatteryStatus     0x16
#define BQ20Z65_CycleCount        0x17
#define BQ20Z65_DesignCapacity    0x18
#define BQ20Z65_DesignVoltage     0x19
#define BQ20Z65_SpecificationInfo 0x1A
#define BQ20Z65_ManufactureDate   0x1B
#define BQ20Z65_SerialNumber      0x1C
#define BQ20Z65_ManufactureName   0x20
#define BQ20Z65_DeviceName        0x21
#define BQ20Z65_DeviceChemistry   0x22
#define BQ20Z65_ManufactureData   0x23
#define BQ20Z65_Authenticate		0x2F  // Z65 !

#define BQ20Z65_CellVolt1         0x3F
#define BQ20Z65_CellVolt2         0x3E
#define BQ20Z65_CellVolt3         0x3D
#define BQ20Z65_CellVolt4         0x3C

// EXTENDED SBS COMMANDS
#define BQ20Z65_tAFEData 			0x45 // Z65 R
#define BQ20Z65_FetControl			0x46 // R/W

#define BQ20Z65_StateOfHealth     0x4F
#define BQ20Z65_SafetyAlert       0x50
#define BQ20Z65_SafetyStatus      0x51
#define BQ20Z65_PFAlert           0x52
#define BQ20Z65_PFStatus          0x53
#define BQ20Z65_OperationStatus   0x54
#define BQ20Z65_ChargingStatus    0x55
#define BQ20Z65_ResetData		    0x57 // Z65 R
#define BQ20Z65_WDResetData			0x58 // Z65 R
#define BQ20Z65_PackVoltage         0x5A // Z65 R
#define BQ20Z65_AverageVoltage      0x5B // Z65 R
#define BQ20Z65_TS1Temperature      0x5E // Z65 R   Int -400/+1200 0.1C
#define BQ20Z65_TS2Temperature      0x5F // Z65 R

#define BQ20Z65_UnSealKey			   0x60
#define BQ20Z65_FullAccessKey		 0x61
#define BQ20Z65_PFKey		 		     0x62
#define BQ20Z65_AuthenKey3			 0x63
#define BQ20Z65_AuthenKey2			 0x64
#define BQ20Z65_AuthenKey1			 0x65
#define BQ20Z65_AuthenKey0			 0x66
#define BQ20Z65_SafetyAlert2		 0x68
#define BQ20Z65_SafetyStatus2		 0x69
#define BQ20Z65_PFAlert2		 	 0x6A
#define BQ20Z65_PFStatus2			 0x6B
#define BQ20Z65_ManufBlock1			 0x6C
#define BQ20Z65_ManufBlock2			 0x6D
#define BQ20Z65_ManufBlock3			 0x6E
#define BQ20Z65_ManufBlock4			 0x6F

#define BQ20Z65_ManufacturerInfo   0x70
#define BQ20Z65_SenseResistor      0x71
#define BQ20Z65_TempRange          0x72
#define BQ20Z65_LifetimeData1          0x73
#define BQ20Z65_LifetimeData2          0x74

#define BQ20Z65_DataFlashSubClassID    0x77
#define BQ20Z65_DataFlashSubClassPage1 0x78
#define BQ20Z65_DataFlashSubClassPage2 0x79
#define BQ20Z65_DataFlashSubClassPage3 0x7A
#define BQ20Z65_DataFlashSubClassPage4 0x7B
#define BQ20Z65_DataFlashSubClassPage5 0x7C
#define BQ20Z65_DataFlashSubClassPage6 0x7D
#define BQ20Z65_DataFlashSubClassPage7 0x7E
#define BQ20Z65_DataFlashSubClassPage8 0x7F


/* 0x54 OperationStatus read-word function returns the current operation status of the bq20z90/bq20z95 NOT FOUND for Z65 */
#define PRES	(1 << 15) // 1 = PRES is low, indicating that the system is present (battery inserted).
#define FAS		(1 << 14) // 0 = Full access security mode
#define SealS		(1 << 13) // 1 = Sealed security mode
#define CSV		(1 << 12) // 1 = Data Flash checksum value has been generated
#define LDMD	(1 << 10) // Load mode for Impedance Track modeling. 0 = constant current, 1 = constant power
#define WAKE	(1 << 7)  // 1 = bq20z90/bq20z95 WAKE mode
#define DSG		(1 << 6)  // Replica of the SBS:BatteryStatus(0x16)[DSG] flag.
#define XDSG	(1 << 5)  // 1 = Discharge fault
#define XDSGI	(1 << 4)  // 1 = Discharge disabled due to a current issue
#define R_DIS	(1 << 2)  // 1 = Ra Table resistance updates are disabled
#define VOK		(1 << 1)  // 1 = Voltages are OK for a QMAX update
#define QEN		(1 << 0)  // 1 = QMAX updates are enabled 

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
/* System Data */
#define Device_Type           0x0001
#define Firmware_Version      0x0002
#define Hardware_Version      0x0003
#define DF_Checksum 		  0x0004
#define Manufacturer_Status   0x0006
#define Chemistry_ID 		  0x0008
/* System Control write only */ 
#define Shutdown 				0x0010
#define Sleep 					0x0011
#define Seal_Device 			0x0020
#define IT_Enable 				0x0021 // forces the bq20z60-R1/bq20z65-R1 to begin the Impedance Track algorithm
#define SAFE_Activation 		0x0030 // drives the SAFE pin high
#define SAFE_Clear 				0x0031 // sets the SAFE pin back to low
#define LEDs_ON 				0x0032
#define LEDs_OFF 				0x0033
#define Display_ON 				0x0034
#define Calibration_Mode 		0x0040 // Places the bq20z60-R1/bq20z65-R1 into calibration mode  (SLUA379A) 
#define Reset 					0x0041 // The bq20z60-R1/bq20z65-R1 undergoes a full reset
#define BootROM 				0x0F00 // The bq20z60-R1/bq20z65-R1 goes into BootROM mode

/* Manufacturer_Status   0x0006 Battery bitmap */
#define STATE0              (1 << 8)
#define STATE1              (1 << 9)
#define STATE2              (1 << 10)
#define STATE3              (1 << 11)
#define PF0                 (1 << 12) // Cell imbalance permanent failure 0,0 fuse blown
#define PF1                 (1 << 13) // Safety voltage failure 1,1 FET failure
#define FET0                (1 << 14) // 0 = DSG FET is off 1 = DSG FET is on
#define FET1                (1 << 15) // 0 = CHG FET is off 1 = CHG FET is on

/* states
  STATE3, STATE2, STATE1, STATE0 — Indicates the battery state.
  0,0,0,0 = Wake Up
  0,0,0,1 = Normal Discharge
  0,0,1,1 = Pre-Charge
  0,1,0,1 = Charge
  0,1,1,1 = Charge Termination
  1,0,0,1 = Permanent Failure
  1,0,1,0 = Overcurrent
  1,0,1,1 = Overtemperature
  1,1,0,0 = Battery Failure
  1,1,0,1 = Sleep
  1,1,1,0 = Reserved
  1,1,1,1 = Battery Pack Removed
*/
#define WAKE_UP    		    0x0
#define NORM_DISCH      	0x1
#define PRECHARGE     		0x3
#define CHARGE        		0x5
#define CHARGETERMINATION 	0x7
#define PERMANENTFAIL   	0x9
#define OVERCURRENT     	0xA
#define OVERTEMP      		0xB
#define BATTFAIL      		0xC
#define SLEEP       		0xD
#define PACKREMOVED     	0xF
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
#define FUSE_BLOWN      	0x0
#define CELL_IMBALANCE    	0x1
#define VOLT_FAIL     		0x2
#define FET_FAIL      		0x3


class BQ20Z65
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
    uint8_t StateOfHealth(void);
    uint16_t BatteryStatus(void);
    uint16_t SafetyAlert(void);
    uint32_t SafetyStatus(void);
    uint16_t PFAlert(void);
    uint16_t PFStatus(void);
    uint16_t OperationStatus(void);
    uint16_t ChargingStatus(void);
    uint16_t BattMode (void);
    uint16_t SpecificationInfo(void);
    uint16_t ManufactureDate(void);
    uint16_t SerialNumber(void);
    uint8_t ManufactureName(uint8_t* buffer);
    uint8_t DeviceName(uint8_t* buffer);
    uint8_t DeviceChemistry(uint8_t* buffer);
    uint8_t ManufactureData(uint8_t* buffer);
    uint32_t GetUnsealKey (void);
    uint32_t FullAccessKey (void);
    uint32_t PFClearKey (void);
    uint32_t PFClearKey_v2 (void);
    
  protected:

  private:

    void write(uint8_t, uint8_t);
    uint8_t read(uint8_t);
    uint16_t read16u(uint8_t);
    int16_t read16(uint8_t);
    uint32_t read32u(uint8_t);
    uint16_t read16u2(uint8_t);
    void write16(uint8_t address, uint16_t data);
    uint8_t readString(uint8_t address, uint8_t* result);

};
#endif

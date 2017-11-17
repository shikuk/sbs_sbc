/* states:
   Wait for ACCU
      getVolt ?
   Idle
      show Volt ?
   Charge
      set V A and check it
   DisCharge
      Disch disable, turn on load, get V A to massiv
*/
#include "BQ24725_lib.h"
#include "BQ20Z45.h"
#include <TimerOne.h>

#define MESSAGES

#define LOAD_ON_PIN 9
#define RED_K_PIN   3
#define GREEN_K_PIN 2
#define LED_PIN     13
#define Vmin        2800 // mV minimal LION cell volt
#define Vmax        4200 // mV maximum LION cell volt
#define INPUT_CURRENT 2000 // mA maximum current from DC ( 19*2 = 38 Wt) 
#define DEBUG

// Store an instance of the BQ20Z45 Sensor
BQ20Z45 bms;

uint16_t data;
String serial_in;
uint8_t printed = 0;
enum {WAIT = 0, IDLE, CHARGING, DISCHARGING};
uint8_t chg_state;
uint8_t duty;
BQ24725_charge_options* BQ24725_current_opts;
uint16_t prevMillis;
enum chemistry { LION, NiMh, ACID};

struct battstate {
  uint16_t Vpack;
  uint16_t V1;     // mV
  uint16_t V2;
  uint16_t V3;
  uint16_t V4;
  float celsius;
  float Current;
  int16_t req_current;
  uint16_t Status;
  uint16_t Opstatus;
  uint16_t Chg_status;
  uint16_t capacity;
  uint16_t chg_cap;
  uint16_t disch_cap;
  uint8_t type;
  bool mode_capacity;
  bool need_condition;
  uint16_t want_volt;
  uint16_t want_ampere;
  int16_t current;
  int16_t avg_current;

} batt_state;

void set_def_batt (void) {
  batt_state.want_volt = 10800;
  batt_state.req_current = 64;
  batt_state.capacity = 1000;
  batt_state.type = LION;
}

uint8_t StrToHex(uint8_t str[])
{
  return (uint8_t) strtol(str, 0, 16);
}

void parse_cmd (String cmd) {
  String sub;

  // SCAN I2C devices
  if (serial_in.startsWith("devs")) {
    for (uint8_t i = 0; i < 128; i++) {
      check_i2c_addr (i);
      delay (10);
    }
    Serial.println(F("That's all devices"));
  }

  // SCAN I2C device regs
  if (serial_in.startsWith("regs")) {
    sub = serial_in.substring(4);
    sub.trim();
    int addr = sub.toInt();
    if ((addr != 0) || (addr > 128)) {
      for (uint8_t i = 0; i < 255; i++) {
        if (bms.Check_Reg(addr, i )) {
          Serial.print ("exist ");
          Serial.println(i, HEX);
        }
        delay (5);
      }
      Serial.println("That's all regs");
    }
    else Serial.println("Addr no good");
  }

  // set charge process
  if (serial_in.startsWith("schg")) {
    sub = serial_in.substring(4);
    sub.trim();
    int current = sub.toInt();
    charge (batt_state.want_volt, current);
    batt_state.req_current = current;
  }

  // start discharge process
  if (serial_in.startsWith("dchg")) {
    sub = serial_in.substring(4);
    sub.trim();
    int current = sub.toInt();
    batt_state.req_current = current;
    discharge(current);
  }

  // stop all
  if (serial_in.startsWith("stop")) {
    charge (0, 0);
    chg_state = IDLE;
  }

  // set alone charge voltage
  if (serial_in.startsWith("volt"))  {
    Serial.print("Set volt: ");
    sub = serial_in.substring(4);
    sub.trim();
    int volt = sub.toInt();
    Serial.println(volt);
    BQ24725_SetChargeVoltage (volt);
    batt_state.want_volt = volt;
    delay(100);
    BQ24725_GetChargeVoltage (&data);
    Serial.print ("get VBatt=");
    Serial.println(data);
    BQ24725_GetChargeCurrent (&data);
    Serial.print ("get chgCurr=");
    Serial.println(data);
    Serial.print("Current: ");
    Serial.println(bms.GetCurrent());

    Serial.print("Charging Current: ");
    Serial.println(bms.ChargingCurrent());

    Serial.print("Charging Voltage: ");
    Serial.println(bms.ChargingVoltage());
  }

  // set alone charge current
  if (serial_in.startsWith("curr")) {
    Serial.print("Set current: ");
    sub = serial_in.substring(4);
    sub.trim();
    int current = sub.toInt();
    Serial.println(current);
    BQ24725_SetChargeCurrent (current);
    batt_state.req_current = current;
    delay(100);
    BQ24725_GetChargeVoltage (&data);
    Serial.print ("get VBatt=");
    Serial.println(data);
    BQ24725_GetChargeCurrent (&data);
    Serial.print ("get chgCurr=");
    Serial.println(data);
    chg_state = CHARGING;
  }
 
  // dump SBS data
  if (serial_in.startsWith("dump")) {
    batt_dump();
  }

  // translate i2c commands from PC
  if (serial_in.startsWith("iicc"))  {
    String sub1, sub2;
    Serial.print("i2c: ");
    sub = serial_in.substring(4);
    sub.trim();
    uint8_t cmd, addr, reg, cnt;
    uint8_t buf[2];
    uint16_t param;
    byte i2cRxData[64];
    sub1 = sub[0];
    cmd = sub1.toInt();
    buf[0] = sub[1];
    buf[1 ]= sub[2];
    addr = StrToHex (buf);
    buf[0] = sub[3];
    buf[1 ]= sub[4];
    reg = StrToHex (buf);
    buf[0] = sub[5];
    buf[1 ]= sub[6];
    cnt = StrToHex (buf);
    sub2 = sub.substring(7);
    param = sub2.toInt();

    Serial.print(cmd, HEX);   Serial.print(F(" addr "));     Serial.print(addr, HEX);   Serial.print(F(" reg "));    Serial.print(reg, HEX);  Serial.print(F(" cnt "));    Serial.println(cnt);
    bms.readAndReportData (addr | cmd, reg, cnt, i2cRxData, 0);
    Serial.print ("get ");
    for (int i = 0; i < cnt + 2; i++ ) {
//      Serial.print (" 0x");
      if (i2cRxData[i]<=0xF) Serial.print("0");
      Serial.print(i2cRxData[i], HEX);
    }
    Serial.println(" ");

  }
  serial_in = "";

}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(RED_K_PIN, OUTPUT);
  pinMode(GREEN_K_PIN, OUTPUT);
  pinMode(LOAD_ON_PIN, OUTPUT);
  digitalWrite (LOAD_ON_PIN, LOW);
  Timer1.initialize(10);  // 10 us = 100 kHz
  duty = 0;
  Timer1.pwm(LOAD_ON_PIN, duty);

  red_on();
  // put your setup code here, to run once:
  // We start the serial library to output our messages.
  Serial.begin(115200);
  BQ24725_Start();

  // Start i2c communication.
  if (check_i2c_addr (BQ24725_ADDR)) {
    BQ24725_GetManufactureID(&data);
    Serial.print("Ven: ");
    Serial.print(data, HEX);
    if (data == 0x40) {
      BQ24725_GetDeviceID(&data);
      Serial.print(" Dev: ");
      Serial.println(data, HEX);
      if (data == 0x0B)
        Serial.println("BQ24725A found");
      else if  (data == 0x08)
        Serial.println("BQ24725 found");
      else if  (data == 0x0A)
        Serial.println("BQ24707 found");
      else if  (data == 0x1D)
        Serial.println("BQ24727 found");
      else
        Serial.println("Unknown BQxxxxx found");
    }
    else Serial.println (" Not TI manuf");
    chg_state = WAIT;
    BQ24725_SetInputCurrent(INPUT_CURRENT);
  }
  else   Serial.println("No charger");


  if (check_i2c_addr (BQ20Z45_Address)) {

    Serial.print("BMS Temp: ");
    Serial.print ( bms.GetTemp() / 10.0 - 273.15);

    Serial.print("  Volt: ");
    Serial.println(bms.GetVoltage());
  }
  else   Serial.println("No SBS device");
  set_def_batt ();
  green_on();
}

void loop() {
  // get commands
  char c;
  uint16_t check;
  int16_t batt_curr;
  int16_t tmp;
  while (Serial.available()) {
    delay(3);  //delay to allow buffer to fill
    if (Serial.available() > 0) {
      c = Serial.read();  //gets one byte from serial buffer
      serial_in += c; //makes the string readString
    }
  }

  if (serial_in.length() > 0) {
    Serial.print("Command: ");
    Serial.println(serial_in); //see what was received
    parse_cmd (serial_in);
  }

  switch (chg_state) {
    case IDLE:
      BQ24725_GetChargeVoltage (&data);
      if (!printed) {
        Serial.print ("IDLE: VBatt=");
        Serial.println(data / 1000.0, 2);
        printed = 1;
      }
      leds_off();
      break;

    case CHARGING:
      green_on();
      if (millis() - prevMillis > 3600)
      {
        charge (batt_state.want_volt, batt_state.req_current);
        prevMillis = millis();
      }
      break;

    case DISCHARGING:
      batt_curr = bms.GetCurrent();
      if (batt_curr <  -(batt_state.req_current))
        disch_pwm (--duty);
      if (batt_curr > -(batt_state.req_current))
        disch_pwm (++duty);
                    Serial.print("curr ");
        Serial.print(batt_curr);
                    Serial.print(" duty ");

      Serial.println(duty);
      red_on();
      check = bms.CellVoltage1();
      if (check <= Vmin) {
        Serial.println(F("Cell_1 empty"));
        disch_pwm(0);
        chg_state = IDLE;
      }
      check = bms.CellVoltage2();
      if (check <= Vmin) {
        Serial.println(F("Cell_2 empty"));
        disch_pwm(0);
        chg_state = IDLE;
      }
      check = bms.CellVoltage3();
      if (check <= Vmin) {
        Serial.println(F("Cell_3 empty"));
        disch_pwm(0);
        chg_state = IDLE;
      }
      break;

    case WAIT:
      BQ24725_GetChargeVoltage (&data);
      if (!printed) {
        Serial.print (F("WAIT: VBatt="));
        Serial.println(data);
        printed = 1;
      }
      if (data > 3.0) {
        chg_state = IDLE;
        printed = 0;
      }
      break;
    default:
      if (!printed) {
        Serial.print (F("default state error"));
        printed = 1;
      }
      break;
  }// switch (chg_state)

  check = bms.GetVoltage();
  tmp = batt_state.Vpack - check;
  if (tmp > 10 || tmp < -10) {
    batt_state.Vpack = check;
    Serial.print(F("Volt: "));
    Serial.println(check);
  }

  check = bms.CellVoltage1();
  tmp = batt_state.V1 - check;
  if (tmp > 10 || tmp < -10) {
    batt_state.V1 = check;
    Serial.print("V1: ");
    Serial.println(check / 1000.0);
  }

  check = bms.CellVoltage2();
  tmp = batt_state.V2 - check;
  if (tmp > 10 || tmp < -10) {
    batt_state.V2 = check;
    Serial.print("V2: ");
    Serial.println(check / 1000.0);
  }

  check = bms.CellVoltage3();
  tmp = batt_state.V3 - check;
  if (tmp > 10 || tmp < -10) {
    batt_state.V3 = check;
    Serial.print("V3: ");
    Serial.println(check / 1000.0);
  }

  check = bms.GetTemp();
  if ((batt_state.celsius - check) > 10) {
    batt_state.celsius = check;
    Serial.print("Temp: ");
    Serial.println (check / 10.0 - 273.15);
  }

  check = bms.GetCurrent();
  if (abs(batt_state.Current - check) > 5) {
    batt_state.Current = check;
    Serial.print("Current: ");
    Serial.print  (check);
    Serial.print("  average: ");
    Serial.println  (bms.AverageCurrent());
  }
  // battery need delays
  delay (1000);

}


void batt_dump()
{
  uint16_t status;
  p_manufDATA();
  p_manufAccess();
  p_battStatus();
  p_AtRate();
  p_Temp();
  p_PackVolt();
  p_cellsVolt();
  p_Current();
  p_desired_V_A();


  Serial.print("Design Voltage: ");
  Serial.println(bms.DesignVoltage());

  Serial.print(F("Charged to "));
  Serial.print(bms.RelativeSOC());
  Serial.print(F("% of FullChargeCapacity or to "));
  Serial.print(bms.AbsoluteSOC());
  Serial.println(F("% of DesignCapacity"));


  Serial.print("Remaining Capacity Alarm: ");
  Serial.println(bms.RemainingCapAlarm());

  Serial.print("Remaining Time Alarm: ");
  Serial.println(bms.RemainingTimeAlarm());

  Serial.print("Remaining Battery Capacity: ");
  Serial.print(bms.RemainingBatteryCapacity());
  if (batt_state.mode_capacity)
    Serial.println (F(" mAh"));
  else Serial.println (F(" *10 mWh"));

  Serial.print("Full Battery Capacity: ");
  Serial.print(bms.FullBatteryCapacity());
  if (batt_state.mode_capacity)
    Serial.println (F(" mAh"));
  else Serial.println (F(" *10 mWh"));

  Serial.print("Cycle Count: ");
  Serial.println(bms.CycleCount());

  Serial.print("Design Capacity: ");
  Serial.print(bms.DesignCapacity());
  if (batt_state.mode_capacity)
    Serial.println (F(" mAh"));
  else Serial.println (F(" *10 mWh"));

  Serial.println();
  //delay(300); // Show new results every second.

}

/*
   Check if device at address alive
*/
int check_i2c_addr (uint8_t address) {
  Wire.beginTransmission(address);
  uint8_t error = Wire.endTransmission();

  if (error == 0)
  {
#ifdef DEBUG
    Serial.print(F("address 0x"));
    if (address < 16)
      Serial.print("0");
    Serial.println(address, HEX);
#endif
    return 1;
  }
  else if (error == 4)
  {
#ifdef DEBUG
    Serial.print(F("error at address 0x"));
    if (address < 16)
      Serial.print("0");
    Serial.println(address, HEX);
#endif
    return 4;

  }
  else return 0;
}

void leds_off (void) {
  digitalWrite (RED_K_PIN, LOW);
  digitalWrite (GREEN_K_PIN, LOW);
}

void green_on (void) {
  digitalWrite (RED_K_PIN, LOW);
  digitalWrite (GREEN_K_PIN, HIGH);
}

void red_on (void) {
  digitalWrite (RED_K_PIN, HIGH);
  digitalWrite (GREEN_K_PIN, LOW);
}

void blue_on (void) {
  digitalWrite (LED_PIN, HIGH);
}


void leds_toggle (void) {
  if (digitalRead (RED_K_PIN))
    green_on();
  else red_on();
}

void charge (uint16_t V, uint16_t A) {
  green_on ();
  disch_pwm (0);

  BQ24725_SetChargeVoltage (V);
  BQ24725_SetChargeCurrent (A);
  chg_state = CHARGING;
}

void discharge (uint16_t A) {
  charge (0, 0);
  red_on ();
  disch_pwm (1);
  chg_state = DISCHARGING;

}

void disch_pwm (uint8_t dutyCycle){
  Timer1.pwm(LOAD_ON_PIN,  ((float)dutyCycle / 100.0)  * 1023);
  delay (1);
}

void buf2serial (uint8_t* buffer, uint8_t len) {
  for  (uint8_t i = 0; i < len; i++) {
    Serial.write (buffer[i]);
  }
}

void p_BattMode (void) {
  uint16_t u = bms.BattMode();
  batt_state.mode_capacity = u & MODE_CAPACITY;
  batt_state.need_condition = u & MODE_CONDITION_CYCLE;
#ifdef MESSAGES
  if (batt_state.mode_capacity) Serial.println (F("Reports in mA or mAh[d]"));
  else Serial.println (F("Reports in mW or mWh"));
  if (batt_state.need_condition) Serial.println (F("Condition cycle requested (MaxError > CF MaxError Limit)"));
  else Serial.println (F("Condition BattMode 0x03 = OK"));
#endif
}

void p_AtRate (void) {
  if (bms.AtRateOK() > 0 ) {
    int16_t i = bms.AtRate();
    uint16_t minutes;
    uint8_t hours, mins;
    if (i < 0) {
      minutes = bms.AtRateTimeToEmpty();
      hours = floor(minutes / 60); // Получаем количество полных часов
      mins = minutes - (hours * 60); // Получаем оставшиеся минуты
      Serial.print (F("Have ")); Serial.print (hours); Serial.print (F(" : ")); Serial.print (mins);
      Serial.print (F(" to empty at rate "));
      Serial.print (abs(i));
      if (batt_state.mode_capacity)
        Serial.println (F(" mAh"));
      else Serial.println (F(" mWh"));

    }
    else {
      minutes = bms.AtRateTimeToFull();
      hours = floor(minutes / 60); // Получаем количество полных часов
      mins = minutes - (hours * 60); // Получаем оставшиеся минуты
      Serial.print (F("Need ")); Serial.print (hours); Serial.print (F(" : ")); Serial.print (mins);
      Serial.print (F(" to full charge at rate "));
      Serial.print (abs(i));
      if (batt_state.mode_capacity)
        Serial.println (F(" mAh"));
      else Serial.println (F(" mWh"));
    }

  }
  else Serial.println (F("AtRate unUsable, check later"));
}

void p_Temp (void) {
  batt_state.celsius = bms.GetTemp() / 10 - 273.15;
#ifdef MESSAGES
  Serial.print (F("Batt Temp "));
  Serial.println (batt_state.celsius);
#endif
}

void p_PackVolt (void) {
  batt_state.Vpack = bms.GetVoltage();
#ifdef MESSAGES
  Serial.print (F("Pack Voltage "));
  Serial.println (batt_state.Vpack / 1000.0, 2);
#endif
}

void p_cellsVolt (void) {
  batt_state.V1 = bms.CellVoltage1();
  batt_state.V2 = bms.CellVoltage2();
  batt_state.V3 = bms.CellVoltage3();
  batt_state.V4 = bms.CellVoltage4();
#ifdef MESSAGES
  Serial.print (F("Cells "));  Serial.print (batt_state.V1 / 1000.0, 2);
  Serial.print (F(" + "));  Serial.print (batt_state.V2 / 1000.0, 2);
  Serial.print (F(" + "));  Serial.print (batt_state.V3 / 1000.0, 2);
  Serial.print (F(" + "));  Serial.println (batt_state.V4 / 1000.0, 2);
#endif
}

void p_Current (void) {
  batt_state.current = bms.GetCurrent();
  batt_state.avg_current = bms.AverageCurrent();
#ifdef MESSAGES
  if (batt_state.current < 0) Serial.print (F("Discharge "));
  else Serial.print (F("Charge "));
  Serial.print (F("current  "));
  Serial.print (abs(batt_state.current));
  Serial.print (F(" mA with average rate "));
  Serial.println ( abs(batt_state.avg_current));
#endif
}

void p_desired_V_A (void) {
  batt_state.want_volt = bms.ChargingVoltage();
  batt_state.want_ampere = bms.ChargingCurrent();
#ifdef MESSAGES
  Serial.print (F("Wants  "));
  Serial.print (batt_state.want_volt);
  Serial.print (F(" mVolt and  "));
  Serial.print ( batt_state.want_ampere);
  Serial.println (F(" mA for charging"));
#endif
}

void p_battStatus (void) { // todo: to bitmap to header
  uint16_t status;
  Serial.print("Battery Status Flags: ");
  Serial.println(bms.BatteryStatus(), HEX);
  status = bms.BatteryStatus();
  if (status & 0x8000) Serial.println("OVER_CHARGED_ALARM");
  if (status & 0x4000) Serial.println("TERMINATE_CHARGE_ALARM");
  if (status & 0x1000) Serial.println("OVER_TEMP_ALARM");
  if (status & 0x0800) Serial.println("TERMINATE_DISCHARGE_ALARM");
  if (status & 0x0200) Serial.println("REMAINING_CAPACITY_ALARM");
  if (status & 0x0100) Serial.println("REMAINING_TIME_ALARM");
  if (status & 0x0080) Serial.println("INITIALIZED");
  if (status & 0x0040) Serial.println("DISCHARGING");
  if (status & 0x0020) Serial.println("FULLY_CHARGED");
  if (status & 0x0010) Serial.println("FULLY_DISCHARGED");
}

void p_manufDATA (void) {
  uint8_t* buffer = {"                                        "};
  uint8_t len;
  Serial.println (F("****** Battery Manuf ******"));

  len = bms.ManufactureName(buffer);
  //buffer[len]='\0';
  buf2serial (buffer, len);
  Serial.print (F(" device "));
  len = bms.DeviceName(buffer);
  buf2serial (buffer, len);
  Serial.print (F("  "));
  len = bms.DeviceChemistry(buffer);
  buf2serial (buffer, len);
  Serial.print (F(" fw/hw rev 0x"));
  len = bms.ManufactureData(buffer);
  Serial.print ((uint16_t)buffer[1] << 8 || buffer[0], HEX);
  Serial.print (F("/0x"));
  Serial.print (buffer[2], HEX);
  Serial.print (F(" S/N "));
  Serial.print (bms.SerialNumber());
  Serial.print (F(" spec ver "));
  Serial.print (bms.SpecificationInfo() & 0xF0);
  Serial.print (F("."));
  Serial.println (bms.SpecificationInfo() & 0x0F);

  uint16_t date = bms.ManufactureDate();
  Serial.print (date & 0x1F);
  Serial.print (F("-"));

  Serial.print ((date >> 5) & 0x0F);
  Serial.print (F("-"));
  Serial.println (1980 + ((date >> 9) & 0xFF));
  Serial.println (F("****** END ******"));
}

void p_manufAccess (void) {
  uint16_t data;
  data = bms.read16uManuf (0x00);
  Serial.print(F("Manufaccess address: 0x"));
  Serial.println(data, HEX);
  data = bms.read16uManuf (Device_Type);
  Serial.print(F("Device Type: 0x"));
  Serial.println(data, HEX);
  Serial.print(F("Controller IC identified by device type: "));
  if (data == 2084) {
    Serial.print(F("bq2084"));
  }
  else    if (data == 0x700) {
    Serial.print(F("bq20z70"));
  }
  else    if (data == 0x900) {
    Serial.print(F("bq20z90"));
  }
  else     if (data == 0xA90A) {
    Serial.print(F("bq8011DBT"));
  }
  else     if (data == 0x600) {
    Serial.print(F("bq8030DBT"));
  }
  Serial.print (F(" fw/hw rev "));
  Serial.print (data = bms.read16uManuf (Firmware_Version)); Serial.print (F("/")); Serial.println (data = bms.read16uManuf (Hardware_Version));
  data = bms.read16uManuf (Manufacturer_Status);
  Serial.print (F("Status: "));
  switch (data >> 8 & 0xF) {
    case  WAKE_UP: Serial.println (F("Wake Up")); break;
    case  NORM_DISCH: Serial.println (F("Normal Discharge")); break;
    case  PRECHARGE: Serial.println (F("Pre-Charge")); break;
    case  CHARGE: Serial.println (F("Charge")); break;
    case  CHARGETERMINATION: Serial.println (F("Charge Termination")); break;
    case  PERMANENTFAIL: Serial.println (F("Permanent Failure")); break;
    case  OVERCURRENT: Serial.println (F("Overcurrent")); break;
    case  OVERTEMP: Serial.println (F("Overtemperature")); break;
    case  BATTFAIL: Serial.println (F("Battery Failure")); break;
    case  SLEEP: Serial.println (F("Sleep")); break;
    case  PACKREMOVED: Serial.println (F("Battery Pack Removed")); break;
    default:
      break;
  }
  if (((data >> 8) & 0xF) == PERMANENTFAIL)  {
    Serial.print (F("*** Failure reason: "));

    switch ((data >> 12) & 0x3) {
      case  FUSE_BLOWN: Serial.println (F("Fuse is blown")); break;
      case  CELL_IMBALANCE: Serial.println (F("Cell imbalance failure")); break;
      case  VOLT_FAIL: Serial.println (F("Safety voltage failure")); break;
      case  FET_FAIL: Serial.println (F("FET failure")); break;
      default:
        break;
    }
  }
  Serial.print (F("FETs state: "));

  switch ((data >> 14) & 0x3) {
    case  BOTH_ON: Serial.println (F("Both FETs are on")); break;
    case  DISCH_ON: Serial.println (F("CHG FET is off, DSG FET is on")); break;
    case  BOTH_OFF: Serial.println (F("Both FETs are off")); break;
    case  CHG_ON: Serial.println (F("CHG FET is on, DSG FET is off")); break;
    default:
      break;
  }
}


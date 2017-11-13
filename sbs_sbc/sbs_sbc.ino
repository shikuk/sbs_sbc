#include "BQ24725_lib.h"
#include "BQ20Z45.h"
#include <TimerOne.h>

#define LOAD_ON_PIN 9   // pin to switch discharge P-ch mosfet 

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
enum {WAIT = 0, IDLE, CHARGE, DISCHARGE};
uint8_t chg_state;
uint8_t duty;
BQ24725_charge_options* BQ24725_current_opts;

enum chemistry { LION, NiMh, ACID};

struct battstate {
  int16_t Vpack;
  int16_t V1;     // mV
  int16_t V2;
  int16_t V3;
  int16_t V4;
  float Temp;     
  float Current;
  int16_t req_current;
  uint16_t Status;
  uint16_t Opstatus;
  uint16_t Chg_status;
  uint16_t capacity;
  uint16_t chg_cap;
  uint16_t disch_cap;
  uint8_t type;
} batt_state;

void set_def_batt (void) {
  batt_state.req_current=64;
  batt_state.capacity=1000;
  batt_state.type=LION;
}

void parse_cmd (String cmd) {
    String sub;

     // SCAN I2C devices
    if (serial_in.startsWith("devs")) {
      for (uint8_t i = 0; i < 128; i++) {
        check_i2c_addr (i);
        delay (50);
      }
      Serial.println("That's all devices");
    }

    // SCAN I2C device regs
    if (serial_in.startsWith("regs")) {
      sub = serial_in.substring(4);
      sub.trim();
      int addr = sub.toInt();
      if ((addr != 0) || (addr > 128)) {
      for (uint8_t i = 0; i < 255; i++) {
        if (bms.Check_Reg(addr, i )) {
          Serial.print ("exist reg ");
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
      charge (11100, current);
      batt_state.req_current = current;
    }

  // start discharge process
    if (serial_in.startsWith("dchg")) {
      discharge ();
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
      delay(100);
      BQ24725_GetChargeVoltage (&data);
      Serial.print ("get VBatt=");
      Serial.println(data);
      BQ24725_GetChargeCurrent (&data);
      Serial.print ("get chgCurr=");
      Serial.println(data);
    }
    // dump SBS data
    if (serial_in.startsWith("dump")) {
      batt_dump();
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
  if (check_i2c_addr (BQ24725_ADDR)) {  // todo: this stucks if bq24725 not powered
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
    Serial.print ( bms.GetTemp()/10.0 - 273.15);

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
  float check;
  int16_t batt_curr;
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
      
    case CHARGE:
      BQ24725_GetChargeVoltage (&data);
      Serial.print (F("CHG V "));
      Serial.print (data / 1000.0, 3);
      batt_curr = bms.GetCurrent();
      Serial.print (F(" A "));
      Serial.print (batt_curr);
      Serial.print (F(" req "));
      Serial.print (batt_state.req_current);
      if ((batt_state.req_current - batt_curr) > 64) {
        //BQ24725_SetChargeVoltage (bms.GetVoltage() + 64);
        charge (bms.GetVoltage() + 64, batt_state.req_current);
        Serial.println (F(" need more"));
      }
      if ((batt_state.req_current - batt_curr) < -64) {
       // BQ24725_SetChargeVoltage (bms.GetVoltage() - 64);
               charge (bms.GetVoltage() - 64, batt_state.req_current);

        Serial.println (F(" need less"));
      }
      green_on();
      break;
      
    case DISCHARGE:
      batt_curr = bms.GetCurrent();
      if (batt_curr <  -(batt_state.capacity>>2))
        disch_pwm (--duty);
      if (batt_curr > -(batt_state.capacity>>2))
        disch_pwm (++duty);
        Serial.println(duty);
        red_on();
          check = bms.CellVoltage1();
          if (check<=Vmin) {
            Serial.println(F("Cell_1 empty"));
            disch_pwm(0);
            chg_state = IDLE;
            }
          check = bms.CellVoltage2();
          if (check<=Vmin) {
            Serial.println(F("Cell_2 empty"));
            disch_pwm(0);
            chg_state = IDLE;
            }
                      check = bms.CellVoltage3();
          if (check<=Vmin) {
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
  if (abs(batt_state.Vpack - check) > 0.015) {
    batt_state.Vpack = check;
    Serial.print("Volt: ");
    Serial.println(check);
  }

  check = bms.CellVoltage1();
  if (abs(batt_state.V1 - check) > 100) {
    batt_state.V1 = check;
    Serial.print("V1: ");
    Serial.println(check / 1000.0);
  }

  check = bms.CellVoltage2();
  if (abs(batt_state.V2 - check) > 100) {
    batt_state.V2 = check;
    Serial.print("V2: ");
    Serial.println(check / 1000.0);
  }

  check = bms.CellVoltage3();
  if (abs(batt_state.V3 - check) > 100) {
    batt_state.V3 = check;
    Serial.print("V3: ");
    Serial.println(check / 1000.0);
  }

  check = bms.GetTemp();
  if (batt_state.Temp != check) {
    batt_state.Temp = check;
    Serial.print("Temp: ");
    Serial.println (check / 10.0 - 273.15);
  }

  check = bms.GetCurrent();
  if (abs(batt_state.Current - check) > 0.015) {
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
  float v1 = 0, v2 = 0, v3 = 0, v4 = 0;
  int status = 0;
  Serial.print("Temp: ");
  Serial.print ((float)(bms.GetTemp() / 10.0 - 273.15));
  Serial.print(" raw ");
  Serial.print (bms.GetTemp(), DEC);

  Serial.print("  Volt: ");
  Serial.println(bms.GetVoltage());
  v1 = bms.CellVoltage1() / 1000.0;
  v2 = bms.CellVoltage2() / 1000.0;
  v3 = bms.CellVoltage3() / 1000.0;
  v4 = bms.CellVoltage4() / 1000.0;

  Serial.print("Cells Voltage: ");
  Serial.print(v1);
  Serial.print(" + ");
  Serial.print(v2);
  Serial.print(" + ");
  Serial.print(v3);
  Serial.print(" + ");
  Serial.print(v4);
  Serial.print(" = ");
  Serial.println(v1 + v2 + v3 + v4);

  Serial.print("Design Voltage: ");
  Serial.println(bms.DesignVoltage());

  Serial.print("Realtive StateOfCharge: ");
  Serial.println(bms.RelativeSOC());

  Serial.print("Absolute StateOfCharge: ");
  Serial.println(bms.AbsoluteSOC());

  Serial.print("Remaining Capacity Alarm: ");
  Serial.println(bms.RemainingCapAlarm());

  Serial.print("Remaining Time Alarm: ");
  Serial.println(bms.RemainingTimeAlarm());

  Serial.print("Remaining Battery Capacity: ");
  Serial.println(bms.RemainingBatteryCapacity());

  Serial.print("Full Battery Capacity: ");
  Serial.println(bms.FullBatteryCapacity());

  Serial.print("Current: ");
  Serial.println(bms.GetCurrent());

  Serial.print("Charging Current: ");
  Serial.println(bms.ChargingCurrent());

  Serial.print("Charging Voltage: ");
  Serial.println(bms.ChargingVoltage());

  Serial.print("Cycle Count: ");
  Serial.println(bms.CycleCount());

  Serial.print("Design Capacity: ");
  Serial.println(bms.DesignCapacity());


  Serial.print("Pending EVD: ");
  Serial.println(bms.PendingEVD());

  Serial.print("State of Health: ");
  Serial.println(bms.StateOfHealth());

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

  Serial.print("Safety Alerts: ");
  Serial.println(bms.SafetyAlert(), HEX);

  Serial.print("Safety Status: ");
  Serial.println(bms.SafetyStatus(), HEX);

  Serial.print("Permanent Failure Alert: ");
  Serial.println(bms.PFAlert(), HEX);

  Serial.print("Permanent Failure Status: ");
  Serial.println(bms.PFStatus(), HEX);

  Serial.print("Operating Status: ");
  Serial.println(bms.OperationStatus(), HEX);

  Serial.print("Charging Status: ");
  Serial.println(bms.ChargingStatus(), HEX);

  Serial.println();

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
    Serial.print("I2C device found at address 0x");
    if (address < 16)
      Serial.print("0");
    Serial.print(address, HEX);
    Serial.println("  !");
#endif
    return 1;
  }
  else if (error == 4)
  {
#ifdef DEBUG
    Serial.print("Unknown error at address 0x");
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
  chg_state = CHARGE;
}

void discharge (void) {
  charge (0, 0);
  red_on ();
  disch_pwm (1);
  chg_state = DISCHARGE;

}

void disch_pwm (uint8_t dutyCycle)
{
  Timer1.pwm(LOAD_ON_PIN,  ((float)dutyCycle / 100.0)  * 1023);
delay (1);
}

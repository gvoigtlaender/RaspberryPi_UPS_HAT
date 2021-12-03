#include <Arduino.h>
#include <config.h>

#if defined USE_I2C
#include <WireS.h>
#define I2C_SLAVE_ADDRESS                                                      \
  0x4 // the 7-bit address (remember to change this when adapting this example)

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE (16)
#endif

#define tws_delay(n) delay(n)
#define WireS_stop_check()

enum _E_I2C_Data {
  eI2C_Data_Voltage_100mV = 0,   //! 4.2V = 42d
  eI2C_Data_Battery_Percent,     //! 100% = 100d
  eI2C_Data_Temperature_DegreeC, //! 42°C = 42d
  eI2C_Data_StatusBits,          //! status bits: _E_I2C_StatusBits
  eI2C_Data_TempLow,             //! lower thr temp (switch fan off)
  eI2C_Data_TempHigh,            //! higher thr temp (switch fan on)
  eI2C_LAST
};

enum _E_I2C_StatusBits {
  eI2C_SB_Button = 0x01,
  eI2C_SB_RpiOnOut = 0x02,
  eI2C_SB_RpiOnIn = 0x04,
  eI2C_SB_RpiWait = 0x08,
  eI2C_SB_FanOn = 0x10,
  eI2C_SB_AC = 0x20,
  eI2C_SB_Battery = 0x40,
  eI2C_SB_ShutdownForPowerfail = 0x80,
};

volatile uint8_t i2c_regs[eI2C_LAST] = {0, 0, 0, 0, 34, 38};
// Tracks the current register pointer position
volatile byte reg_position = 0;
// const byte reg_size = sizeof(i2c_regs);

/**
 * This is called for each read request we receive, never put more than one byte
 * of data (with TinyWireS.send) to the send-buffer when using this callback
 */
void requestEvent() {
  TinyWireS.write(i2c_regs[reg_position % eI2C_LAST]);
#if defined DEBUG
  Serial.printf("r[%d]=0x%X\n", reg_position, i2c_regs[reg_position]);
#endif
  // Increment the reg position on each read, and loop back to zero
  reg_position++;
  // if (reg_position >= eI2C_LAST) {
  //  reg_position = 0;
  //  }
}

/**
 * The I2C data received -handler
 *
 * This needs to complete before the next incoming transaction (start, data,
 * restart/stop) on the bus does so be quick, set flags for long running tasks
 * to be called from the mainloop instead of running them directly,
 */
void receiveEvent(size_t howMany) {
  if (howMany < 1) {
    // Sanity-check
    return;
  }
  if (howMany > TWI_RX_BUFFER_SIZE) {
    // Also insane number
    return;
  }

  reg_position = TinyWireS.read();
  // Serial.printf("receiveEvent(%d) idx=%d\n", howMany, reg_position);
  howMany--;
  if (!howMany) {
    // This write was only to set the buffer for next read
    return;
  }
  while (howMany--) {
    uint8_t value = TinyWireS.read();
    i2c_regs[reg_position % eI2C_LAST] = value;
    if (reg_position == eI2C_Data_TempLow) {
      Serial.print(F("TempLow="));
      Serial.println(value);
    }
    if (reg_position == eI2C_Data_TempHigh) {
      Serial.print(F("TempHigh="));
      Serial.println(value);
    }
    reg_position++;
  }
}
#else
typedef int _E_I2C_StatusBits;
#endif

void SetI2CStatusBit(_E_I2C_StatusBits bBit, bool bOn) {
#if defined USE_I2C
  if (bOn)
    i2c_regs[eI2C_Data_StatusBits] |= bBit;
  else
    i2c_regs[eI2C_Data_StatusBits] &= ~bBit;
#endif
}

bool GetI2CStatusBit(_E_I2C_StatusBits bBit) {
#if defined USE_I2C
  return ((i2c_regs[eI2C_Data_StatusBits] & bBit) == bBit);
#else
  return false;
#endif
}

#if defined USE_DS18B20_AVR
#include <bitop.h>
#include <ds18b20/ds18b20.h>
#endif

void switch_rpi(bool bOn) {
  digitalWrite(RPI_POWER_ON_OUT, bOn);
  SetI2CStatusBit(eI2C_SB_RpiWait, GetI2CStatusBit(eI2C_SB_RpiOnOut) != bOn);
  SetI2CStatusBit(eI2C_SB_RpiOnOut, bOn);
}

void switch_fan(bool bOn) {
  digitalWrite(FAN_5V_OUT, bOn);
  SetI2CStatusBit(eI2C_SB_FanOn, bOn);
}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  Serial.printf(F("\nST\n"));

#if defined WIRE_SLAVE_ONLY
  Serial.println(F("WSO"));
#endif

  pinMode(RPI_POWER_ON_OUT, OUTPUT); // OC1B-, Arduino pin 3, ADC
  pinMode(FAN_5V_OUT, OUTPUT);       // OC1B-, Arduino pin 3, ADC

  pinMode(BTN, INPUT_PULLUP);
  pinMode(STEPUP_VOLTAGE_IN, INPUT);
  pinMode(RPI_IS_ONLINE_IN, INPUT);

#if defined USE_I2C
  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
#endif

#if defined USE_DS18B20_AVR
  // set precision
  ds18b20wsp(&PORTB, &DDRB, &PINB, BIT(PB2), NULL, 0, 0, DS18B20_RES12);
  ds18b20convert(&PORTB, &DDRB, &PINB, BIT(PB2), NULL);
#endif
  // Whatever other setup routines ?
  Serial.print(F("TempLow="));
  Serial.println(i2c_regs[eI2C_Data_TempLow]);
  Serial.print(F("TempHigh="));
  Serial.println(i2c_regs[eI2C_Data_TempHigh]);

  // switch_rpi(true);
  switch_fan(true);
}

int16_t analogIn = 0;

#define ain_min 747
#define ain_max 937
#define vout_min 3.99
#define vout_max 5.01
float map_voltage(int16_t x) {
  return (x - ain_min) * (vout_max - vout_min) / (ain_max - ain_min) + vout_min;
}

#define vin_min 3.6
#define vin_max 4.2
#define pout_min 10.0
#define pout_max 100.0
float map_percent(float x) {
  if (x < vin_min)
    return pout_min;
  if (x > vin_max)
    return pout_max;
  return (x - vin_min) * (pout_max - pout_min) / (vin_max - vin_min) + pout_min;
}
void loop_ReadVoltage() {
  analogIn = analogRead(STEPUP_VOLTAGE_IN); // Read analog voltage on pin 2 (A1)
  float dVoltage = map_voltage(analogIn);
  uint8_t Voltage100mv = (uint8_t)(dVoltage * 10.0);
  i2c_regs[eI2C_Data_Voltage_100mV] = Voltage100mv;
  uint8_t Percent = (uint8_t)map_percent(dVoltage);
  i2c_regs[eI2C_Data_Battery_Percent] = Percent;
  SetI2CStatusBit(eI2C_SB_AC, dVoltage >= 4.4);
  SetI2CStatusBit(eI2C_SB_Battery, dVoltage < 4.4);
}

void loop_ReadTemperature() {
#if defined USE_DS18B20_AVR
  // static unsigned long ulMillis = millis();
  // if (millis() >= ulMillis) {
  int16_t temp;
  ds18b20read(&PORTB, &DDRB, &PINB, BIT(PB2), NULL, &temp);
  ds18b20convert(&PORTB, &DDRB, &PINB, BIT(PB2), NULL);
  uint8_t dTemperature = (uint8_t)(temp / 16.0);
// Serial.println(sensor.getTempC());
#if defined USE_I2C
  i2c_regs[eI2C_Data_Temperature_DegreeC] = dTemperature;
#endif
  //  ulMillis += 500;
  //}
#endif
}

bool bButtonReleased = false;
bool bButtonFirstReadDone = false;
void loop_ReadButton() {
  if (digitalRead(BTN) == 0) {
    //! PRESSED
    if (!bButtonFirstReadDone) {
      bButtonFirstReadDone = true;
      SetI2CStatusBit(eI2C_SB_Button, false);
    }
    if (!GetI2CStatusBit(eI2C_SB_Button)) {
      SetI2CStatusBit(eI2C_SB_Button, true);
    }
  } else {
    //! RELEASED
    if (!bButtonFirstReadDone) {
      bButtonFirstReadDone = true;
      SetI2CStatusBit(eI2C_SB_Button, true);
    }
    if (GetI2CStatusBit(eI2C_SB_Button)) {
      bButtonReleased = true;
      SetI2CStatusBit(eI2C_SB_Button, false);
    }
  }
}

void loop_ReadIsOnline() {
  SetI2CStatusBit(eI2C_SB_RpiOnIn, digitalRead(RPI_IS_ONLINE_IN) > 0);
}

void loop_SerialLog() {
  Serial.printf("[A: %d]", analogIn);
  Serial.printf("[V: %d]", i2c_regs[eI2C_Data_Voltage_100mV]);
  Serial.printf("[P: %d]", i2c_regs[eI2C_Data_Battery_Percent]);
  Serial.printf("[B: %d]", GetI2CStatusBit(eI2C_SB_Button));
  Serial.printf("[R: %d]", GetI2CStatusBit(eI2C_SB_RpiOnOut));
  Serial.printf("[O: %d]", GetI2CStatusBit(eI2C_SB_RpiOnIn));
  Serial.printf("[W: %d]", GetI2CStatusBit(eI2C_SB_RpiWait));
  Serial.printf("[F: %d]", GetI2CStatusBit(eI2C_SB_FanOn));
  Serial.printf("[T: %d]", i2c_regs[eI2C_Data_Temperature_DegreeC]);
#if defined USE_I2C
  for (uint8_t n = 0; n < eI2C_LAST; n++) {
    Serial.printf("[%02X]", i2c_regs[n]);
  }
#endif
  Serial.println();
}

//#define dTempOn 38.0
//#define dTempOff 34.0
void loop_FanControl() {
  if (millis() < 20000)
    return;
  if (i2c_regs[eI2C_Data_Temperature_DegreeC] >= i2c_regs[eI2C_Data_TempHigh]) {
    if (!GetI2CStatusBit(eI2C_SB_FanOn))
      switch_fan(true);
  } else if (i2c_regs[eI2C_Data_Temperature_DegreeC] <=
             i2c_regs[eI2C_Data_TempLow]) {
    if (GetI2CStatusBit(eI2C_SB_FanOn))
      switch_fan(false);
  }
}

#define _RPIMODE 2
enum _E_RpiState {
  eRS_Start = 0,
  eRS_WaitForRpiOn,
  eRS_WaitForRpiOff,
  eRS_Delay,
  eRS_WaitForButtonReleased,
  eRS_WaitForPowerRestored,
  eRS_Delay2,
};
_E_RpiState eRpiState = eRS_Start;
#define nRpiOffDelay 5000
void loop_RpiControl() {
#if _RPIMODE == 1
  if (bWaitForStateChange) {
    if (bIsOnline == bRpi)
      bWaitForStateChange = false;
  } else if (bRpi) {
    if (!bIsOnline)
      switch_rpi(false);
  } else if (bButtonReleased) {
    switch_rpi(true);
  }

#else  // _RPIMODE

  static unsigned long ulMillis = millis();
  switch (eRpiState) {
  case eRS_Start:

    if (!GetI2CStatusBit(eI2C_SB_AC)) {
      SetI2CStatusBit(eI2C_SB_ShutdownForPowerfail, true);
      eRpiState = eRS_WaitForPowerRestored;
      Serial.printf(F("l_Rc, sr(0), "
                      "s=W4AC\n"));
      break;
    }
    // Serial.printf("loop_RpiControl, state=eRS_Start\n");
    Serial.printf(F("l_Rc, s=W4R1\n"));
    switch_rpi(true);
    eRpiState = eRS_WaitForRpiOn;
    break;

  case eRS_WaitForRpiOn:
    if (GetI2CStatusBit(eI2C_SB_RpiOnIn)) {
      Serial.printf(F("l_Rc, o1, s=W4R0\n"));
      eRpiState = eRS_WaitForRpiOff;
    }
    break;

  case eRS_WaitForRpiOff:
    if (!GetI2CStatusBit(eI2C_SB_RpiOnIn)) {
      Serial.printf(F("l_Rc, o0, s=D\n"));
      ulMillis = millis() + nRpiOffDelay;
      eRpiState = eRS_Delay;
    }
    break;

  case eRS_Delay:
    if (GetI2CStatusBit(eI2C_SB_RpiOnIn)) {
      Serial.printf(F("l_Rc, o1, s=W4R0\n"));
      eRpiState = eRS_WaitForRpiOff;
      break;
    }

    if (millis() <= ulMillis)
      break;

    Serial.printf(F("l_Rc, sr(0), "
                    "s=W4B0\n"));
    switch_rpi(false);

    if (GetI2CStatusBit(eI2C_SB_Battery)) {
      SetI2CStatusBit(eI2C_SB_ShutdownForPowerfail, true);
      eRpiState = eRS_WaitForPowerRestored;
      Serial.printf(F("l_Rc, sr(0), "
                      "s=W4AC\n"));
      break;
    }

    eRpiState = eRS_WaitForButtonReleased;
    break;

  case eRS_WaitForButtonReleased:
    if (bButtonReleased == false)
      break;

    Serial.printf(F("l_Rc, B0, sr(1), "
                    "s=W4R1\n"));
    switch_rpi(true);
    eRpiState = eRS_WaitForRpiOn;
    break;

  case eRS_WaitForPowerRestored:
    if (!GetI2CStatusBit(eI2C_SB_AC)) {
      break;
    }

    ulMillis = millis() + nRpiOffDelay;
    eRpiState = eRS_Delay2;
    break;

  case eRS_Delay2:
    if (!GetI2CStatusBit(eI2C_SB_AC)) {
      ulMillis = millis() + nRpiOffDelay;
      break;
    }

    if (millis() <= ulMillis)
      break;

    Serial.printf(F("l_Rc, s=W4R1\n"));
    switch_rpi(true);
    eRpiState = eRS_WaitForRpiOn;
    break;

  default:
    eRpiState = eRS_WaitForRpiOn;
    break;
  }
#endif // _RPIMODE
}

void loop() {
  // put your main code here, to run repeatedly:

  loop_ReadButton();

  static unsigned long ulMillis = millis();
  if (millis() >= ulMillis) {
    loop_FanControl();
    loop_ReadTemperature();
    loop_ReadIsOnline();
    loop_ReadVoltage();
    loop_SerialLog();
    ulMillis += 500;
  }
  loop_RpiControl();
  bButtonReleased = false;

#if defined USE_I2C
  WireS_stop_check();
#endif
}
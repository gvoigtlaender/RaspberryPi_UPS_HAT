#if !defined _CONFIG_H_
#define _CONFIG_H_

#define USE_I2C
#define USE_DS18B20_AVR
#define HW_REV 1

#if HW_REV == 1
#define STEPUP_VOLTAGE_IN PIN_A2
#define RPI_POWER_ON_OUT PIN_PA3
#define FAN_5V_OUT PIN_PA7
#define BTN PIN_PB0
#define RPI_IS_ONLINE_IN PIN_PB1
#define ONE_WIRE PIN_PB2
#elif HW_REV == 2
#define RPI_POWER_ON_OUT PIN_PA2
#define FAN_5V_OUT PIN_PA3
#define ONE_WIRE PIN_PA7
#define BTN PIN_PB0
#define RPI_IS_ONLINE_IN PIN_PB1
#define STEPUP_VOLTAGE_IN PIN_PB2
#endif

#endif // _CONFIG_H_
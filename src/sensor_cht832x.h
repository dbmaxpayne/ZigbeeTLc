
#ifndef _SENSORS_CHT832X_H_
#define _SENSORS_CHT832X_H_

#if (SENSOR_TYPE == SENSOR_CHT832X)
// Timing
#define SENSOR_POWER_TIMEOUT_ms  	5
#define SENSOR_RESET_TIMEOUT_ms		5
#define SENSOR_MEASURING_TIMEOUT_ms	60
#define SENSOR_MEASURING_TIMEOUT  (SENSOR_MEASURING_TIMEOUT_ms * CLOCK_16M_SYS_TIMER_CLK_1MS) // clk tick

//  I2C address
#define CHT832X_I2C_ADDR		0x44
#define CHT832X_I2C_ADDR_MAX	0x47

// Commands
#define CHT832X_CMD_SOFT_RESET          0x30A2
#define CHT832X_CMD_READ_TEMP_HUM 	    0x2400
#define CHT832X_CMD_READ_STAUS_REGISTER 0x3F2D


struct __attribute__((packed)) _cht832x_config_t{
	u16 checksum_status	: 1;
	u16 command_status	: 1;
	u16 reserved3		: 2;
	u16 reset_detected	: 1;
	u16 reserved2		: 8;
	u16 heater			: 1;
	u16 reserved1		: 2;
} cht832x_config_t;


typedef struct _measured_data_t {
	u16	battery_mv; // mV
	u16	average_battery_mv; // mV
	s16	temp; // in 0.01 C
	s16	humi; // in 0.01 %
	u16	battery_level; // in 0.5%
} measured_data_t;

extern u8 sensor_i2c_addr;
extern u32 sensor_id;
extern measured_data_t measured_data;

#define sensor_go_sleep()

void init_sensor(void);
int read_sensor(void);
#endif // (SENSOR_TYPE == SENSOR_CHT832X)
#endif // _SENSORS_CHT832X_H_

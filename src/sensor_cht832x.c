
#include "tl_common.h"
#if defined(SENSOR_TYPE) && SENSOR_TYPE == SENSOR_CHT832X
#include "app_cfg.h"
#include "device.h"
#include "chip_8258/timer.h"

#include "i2c_drv.h"
#include "device.h"
#include "sensors.h"

#define _SENSOR_SPEED_CODE_SEC_ _attribute_ram_code_sec_

#ifndef USE_SENSOR_ID
#warning "SET USE_SENSOR_ID!"
#define USE_SENSOR_ID	0
#endif

u8 sensor_i2c_addr;

#if USE_SENSOR_ID
u32 sensor_id;
#endif

measured_data_t measured_data;

void init_sensor(void)
 {
	int test_i2c_addr = CHT832X_I2C_ADDR << 1;
	#if USE_SENSOR_ID
		sensor_id = 0;
	#endif

	send_i2c_byte(0, 0x06); // Reset command using the general call address
	pm_wait_ms(SENSOR_POWER_TIMEOUT_ms);

	while(test_i2c_addr <= (CHT832X_I2C_ADDR_MAX << 1))
		{
			sensor_i2c_addr = scan_i2c_addr(test_i2c_addr);
			if(sensor_i2c_addr)
				{
					#if USE_SENSOR_ID
						// Not implemented - what for anyway?
					#endif

					read_sensor();
					break;
				}
			test_i2c_addr += 2;
		}
 }

__attribute__((optimize("-Os"))) int read_sensor(void) {
	u8 i = 3, buf[4];
	u16 _temp;
	u8 reg_data[6]; // 16-bit temp + 8-bit CRC + 16-bit humidity + 8-bit CRC
	battery_detect();
	if (sensor_i2c_addr != 0)
		{
			buf[0] = CHT832X_CMD_READ_TEMP_HUM >> 8;;
			buf[1] = CHT832X_CMD_READ_TEMP_HUM & 0xff;
			send_i2c_bytes(sensor_i2c_addr, buf, 2); // start measure T/H
			pm_wait_ms(SENSOR_MEASURING_TIMEOUT_ms);

			while(i--)
				{
					if (read_i2c_bytes(sensor_i2c_addr, reg_data, sizeof(reg_data)) == 0)
						{
							_temp = (reg_data[0] << 8) | reg_data[1];
							if(_temp == 0xffff)
								break;
							measured_data.temp = ((_temp * 17500) >> 16) - 4500 + g_zcl_thermostatUICfgAttrs.temp_offset; // x 0.01 C

							_temp = (reg_data[3] << 8) | reg_data[4];
							measured_data.humi = ((_temp * 10000) >> 16) + g_zcl_thermostatUICfgAttrs.humi_offset; // x 0.01 %
							if (measured_data.humi < 0)
								measured_data.humi = 0;
							else if	(measured_data.humi > 9999)
								measured_data.humi = 9999;

							//send_i2c_byte(sensor_i2c_addr, CHT832X_CMD_READ_TEMP_HUM); // start measure T/H
							// measured_data.count++;
							return 1;
						}
				}
			//send_i2c_byte(sensor_i2c_addr, CHT832X_CMD_READ_TEMP_HUM); // start measure T/H
		}
	return 0;
}

#endif // defined(SENSOR_TYPE) && SENSOR_TYPE == SENSOR_CHT8305


#include "tl_common.h"
#include "chip_8258/register.h"
//#include "app_cfg.h"
#include "i2c_drv.h"

#if USE_I2C_DRV

#define _I2C_SEG_

_I2C_SEG_
void init_i2c(void) {
	i2c_gpio_set(I2C_GROUP); // I2C_GPIO_GROUP_C0C1, I2C_GPIO_GROUP_C2C3, I2C_GPIO_GROUP_B6D7, I2C_GPIO_GROUP_A3A4
	reg_i2c_speed = (unsigned char)(CLOCK_SYS_CLOCK_HZ/(4*I2C_CLOCK)); // 100, 400, 700 kHz
    //reg_i2c_id  = slave address
    reg_i2c_mode |= FLD_I2C_MASTER_EN; //enable master mode
	reg_i2c_mode &= ~FLD_I2C_HOLD_MASTER; // Disable clock stretching for Sensor

    reg_clk_en0 |= FLD_CLK0_I2C_EN;    //enable i2c clock
    reg_spi_sp  &= ~FLD_SPI_ENABLE;   //force PADs act as I2C; i2c and spi share the hardware of IC
}

unsigned char scan_i2c_addr(unsigned char address){
	unsigned char r = irq_disable();
	if ((reg_clk_en0 & FLD_CLK0_I2C_EN)==0)
		init_i2c();
#if I2C_CLOCK != 100000
	unsigned char s = reg_i2c_speed;
	reg_i2c_speed = (u8)(CLOCK_SYS_CLOCK_HZ/(4*100000)); // 100 kHz
#endif
	reg_i2c_id = (unsigned char) address;
	reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_STOP;
	while (reg_i2c_status & FLD_I2C_CMD_BUSY);
#if I2C_CLOCK != 100000
	reg_i2c_speed = s;
#endif
	irq_restore(r);
	return ((reg_i2c_status & FLD_I2C_NAK)? 0 : address);
}

int send_i2c_byte(unsigned char i2c_addr, unsigned char cmd) {
	unsigned char r = irq_disable();
	if ((reg_clk_en0 & FLD_CLK0_I2C_EN)==0)
			init_i2c();
	reg_i2c_id = i2c_addr;
	reg_i2c_adr = cmd;
	reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_ADDR | FLD_I2C_CMD_STOP;
	while (reg_i2c_status & FLD_I2C_CMD_BUSY);
	irq_restore(r);
	return (reg_i2c_status & FLD_I2C_NAK);
}

_I2C_SEG_
int send_i2c_bytes(unsigned char i2c_addr, unsigned char * dataBuf, size_t dataLen) {
	int err = 0;
	unsigned char r = irq_disable();
	if ((reg_clk_en0 & FLD_CLK0_I2C_EN)==0)
			init_i2c();
	unsigned char * p = dataBuf;
	reg_i2c_id = i2c_addr;
	reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID;
	while (reg_i2c_status & FLD_I2C_CMD_BUSY);
	err = reg_i2c_status & FLD_I2C_NAK;
	if(!err) {
		while (dataLen--) {
			reg_i2c_do = *p++;
			reg_i2c_ctrl = FLD_I2C_CMD_DO;
			while (reg_i2c_status & FLD_I2C_CMD_BUSY);
			err = reg_i2c_status & FLD_I2C_NAK;
			if(err)
				break;
		}
	}
	reg_i2c_ctrl = FLD_I2C_CMD_STOP;
	while (reg_i2c_status & FLD_I2C_CMD_BUSY);
	irq_restore(r);
	return err;
}

_I2C_SEG_
int read_i2c_bytes(unsigned char i2c_addr, unsigned char * dataBuf, int dataLen) {
	int ret = -1;
	int size = dataLen;
	unsigned char *p = dataBuf;
	unsigned char r = irq_disable();
	if ((reg_clk_en0 & FLD_CLK0_I2C_EN)==0)
			init_i2c();
	reg_i2c_id = i2c_addr | FLD_I2C_WRITE_READ_BIT;
	reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID | FLD_I2C_CMD_READ_ID;
    while(reg_i2c_status & FLD_I2C_CMD_BUSY);
	ret = reg_i2c_status & FLD_I2C_NAK;
	if(ret == 0) {
		while(size) {
			size--;
			if(!size)
				reg_i2c_ctrl = FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID | FLD_I2C_CMD_ACK;
			else
				reg_i2c_ctrl = FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID;
			while(reg_i2c_status & FLD_I2C_CMD_BUSY);
			*p++ = reg_i2c_di;
		}
	}
	reg_i2c_ctrl = FLD_I2C_CMD_STOP;
    while(reg_i2c_status & FLD_I2C_CMD_BUSY);
    irq_restore(r);
    return ret;
}


/* Universal I2C/SMBUS read-write transaction
 * wrlen = 0..127 ! */
_I2C_SEG_
int I2CBusUtr(void * outdata, i2c_utr_t * tr, unsigned int wrlen) {
	unsigned char * pwrdata = (unsigned char *) &tr->wrdata;
	unsigned char * poutdata = (unsigned char *) outdata;
	unsigned int cntstart = wrlen - (tr->mode & 0x7f);
	unsigned int rdlen = tr->rdlen & 0x7f;

	unsigned char r = irq_disable();
	if ((reg_clk_en0 & FLD_CLK0_I2C_EN)==0)
			init_i2c();

	reg_i2c_id = *pwrdata++;
	reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID;
	while(reg_i2c_status & FLD_I2C_CMD_BUSY);

	int ret = reg_i2c_status & FLD_I2C_NAK;
	while(ret == 0 && wrlen) {
		// write data
		reg_i2c_do = *pwrdata++;
		reg_i2c_ctrl = FLD_I2C_CMD_DO;
		while(reg_i2c_status & FLD_I2C_CMD_BUSY);
		ret = reg_i2c_status & FLD_I2C_NAK;

		if(--wrlen == cntstart && ret == 0) { // + send start & id
			if(tr->mode & 0x80) {
				reg_i2c_ctrl = FLD_I2C_CMD_STOP;
			} else {
				// hw reset I2C
				reg_rst0 = FLD_RST0_I2C;
				reg_rst0 = 0;
			}
			while(reg_i2c_status & FLD_I2C_CMD_BUSY);
			reg_i2c_id = tr->wrdata[0] |  FLD_I2C_WRITE_READ_BIT;
			// start + id
			reg_i2c_ctrl = FLD_I2C_CMD_START | FLD_I2C_CMD_ID;
			while(reg_i2c_status & FLD_I2C_CMD_BUSY);
			ret = reg_i2c_status & FLD_I2C_NAK;
		}
	}
	if(ret == 0) {
		while(rdlen) {
			if(--rdlen == 0 && (tr->rdlen & 0x80) == 0)
				reg_i2c_ctrl = FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID | FLD_I2C_CMD_ACK;
			else
				reg_i2c_ctrl = FLD_I2C_CMD_DI | FLD_I2C_CMD_READ_ID;
			while(reg_i2c_status & FLD_I2C_CMD_BUSY);
			*poutdata++ = reg_i2c_di;
		}
	}
	reg_i2c_ctrl = FLD_I2C_CMD_STOP; // launch start/stop cycle
	while(reg_i2c_status & FLD_I2C_CMD_BUSY);
	irq_restore(r);
	return ret;
}

#endif // #if USE_I2C_DRV

#if USE_I2C_DRV_SW

#define _I2C_SEG_

enum {
	GPIO_INPUT_MODE,
	GPIO_OUTPUT_MODE,
};

_I2C_SEG_
void init_i2c(void)
	{
		// Already done in board definition
		// Function not required
		gpio_set_func(I2C_SDA, AS_GPIO);
		gpio_set_func(I2C_SCL, AS_GPIO);

		gpio_set_output_en(I2C_SDA, 1);
		gpio_set_input_en(I2C_SDA, 0);

		gpio_set_output_en(I2C_SCL, 1);
		gpio_set_input_en(I2C_SCL, 0);
	}

void i2c_sda_pin_mode_set(unsigned char mode, unsigned char level)
	{
		if (mode == GPIO_INPUT_MODE)
			{
				gpio_set_input_en(I2C_SDA, 1);
				gpio_set_output_en(I2C_SDA, 0);
			}
		else if(mode == GPIO_OUTPUT_MODE)
			{
				gpio_set_input_en(I2C_SDA, 0);
				gpio_set_output_en(I2C_SDA, 1);
			}
	}

void i2c_sda_pin_set(unsigned char level)
	{
		gpio_write(I2C_SDA, level);
	}

void i2c_scl_pin_set(unsigned char level)
	{
		gpio_write(I2C_SCL, level);
	}

unsigned char i2c_sda_pin_status_get(void)
	{
		return (!gpio_read(I2C_SDA))?0:1;
	}

void i2c_delay(unsigned long tim_1us)
	{
		sleep_us(tim_1us);
	}

void i2c_ack(void)
	{
		i2c_scl_pin_set(0);
		i2c_delay(I2C_CLOCK);

		i2c_sda_pin_mode_set(GPIO_OUTPUT_MODE, 0);
		i2c_sda_pin_set(0);
		i2c_delay(I2C_CLOCK);

		i2c_scl_pin_set(1);
		i2c_delay(I2C_CLOCK);
		i2c_scl_pin_set(0);
		i2c_delay(I2C_CLOCK);
	}

void i2c_noack(void)
	{
		i2c_sda_pin_mode_set(GPIO_OUTPUT_MODE, 1);
		i2c_sda_pin_set(1);

		i2c_delay(I2C_CLOCK);
		i2c_scl_pin_set(1);
		i2c_delay(I2C_CLOCK);
		i2c_scl_pin_set(0);
		i2c_delay(I2C_CLOCK);
	}

/**
 * @description: i2c wait ack
 * @param {type} none
 * @return: rev ack return true else return false
 */
unsigned char i2c_wait_ack(void)
{
	unsigned char cnt = 50;

    i2c_sda_pin_mode_set(GPIO_INPUT_MODE, 1);/* set input and release SDA */
    i2c_sda_pin_set(1);
    i2c_delay(I2C_CLOCK);

    i2c_scl_pin_set(0);       /* put down SCL ready to cheack SCA status */
    i2c_delay(I2C_CLOCK);

    i2c_scl_pin_set(1);
    i2c_delay(I2C_CLOCK);

    while (i2c_sda_pin_status_get()) { /* get ack */
        cnt--;
        if (cnt == 0) {
            i2c_scl_pin_set(0);
            return false;
        }
        i2c_delay(I2C_CLOCK);
    }

    i2c_scl_pin_set(0);
    i2c_delay(I2C_CLOCK);
    return true;
}

unsigned char scan_i2c_addr(unsigned char address)
	{
		unsigned char r = irq_disable();
		unsigned char ack = false;

		i2c_start();
		send_i2c_byte_raw(address);
		ack = i2c_wait_ack();
		i2c_stop();

		irq_restore(r);
		return (ack?address:0);
	}

void send_i2c_byte_raw(unsigned char data)
	{
		unsigned char idx = 0;

	    i2c_scl_pin_set(0);
	    i2c_sda_pin_mode_set(GPIO_OUTPUT_MODE, 1);

	    for (idx = 0; idx < 8; idx++)
	    	{
	    		i2c_delay(I2C_CLOCK/2);
	        	if (data & 0x80) i2c_sda_pin_set(1);
	        	else i2c_sda_pin_set(0);
				i2c_delay(I2C_CLOCK/2);

				i2c_scl_pin_set(1);
				i2c_delay(I2C_CLOCK);

				i2c_scl_pin_set(0);
				i2c_delay(I2C_CLOCK);

				data <<= 1;
	    	}
	}

int send_i2c_byte(unsigned char i2c_addr, unsigned char cmd)
	{
		unsigned char r = irq_disable();
		int ack = false;

		i2c_start();

		send_i2c_byte_raw(i2c_addr);
		ack = i2c_wait_ack();

		if (ack == true)
			{
				send_i2c_byte_raw(cmd);
				ack = i2c_wait_ack();
			}

		i2c_stop();

		irq_restore(r);

		return (ack?0:-1);
	}

_I2C_SEG_
int send_i2c_bytes(unsigned char i2c_addr, unsigned char * dataBuf, size_t dataLen)
	{
		int ack = 0;
		unsigned char idx;
		unsigned char r = irq_disable();

		i2c_start();

		send_i2c_byte_raw(i2c_addr);
		ack = i2c_wait_ack();

		if (ack == true)
			{
				for (idx = 0; idx < dataLen; idx++)
					{
						send_i2c_byte_raw(dataBuf[idx]);
			        	ack = i2c_wait_ack();
			        	if (ack != true) break;
					}
			}

		i2c_stop();

		irq_restore(r);
		return (ack?0:-1);
	}

void i2c_rcv_byte(unsigned char *data)
	{
		unsigned char idx;
		i2c_sda_pin_mode_set(GPIO_INPUT_MODE, 1);
		i2c_delay(25);

		for (idx = 0; idx < 8; idx++)
			{
				i2c_scl_pin_set(0);
				i2c_delay(I2C_CLOCK);

				i2c_scl_pin_set(1);
				*data = *data << 1;
				if (i2c_sda_pin_status_get()) *data |= 1;
				i2c_delay(I2C_CLOCK);
			}

		i2c_scl_pin_set(0);
	}

_I2C_SEG_
int read_i2c_bytes(unsigned char i2c_addr, unsigned char * dataBuf, int dataLen)
	{
		unsigned char r = irq_disable();
		unsigned char idx;
		unsigned char ack;

		// Increment address by one to switch chip to read mode
		i2c_addr = i2c_addr | 0x1;

		i2c_start();

		send_i2c_byte_raw(i2c_addr);
		ack = i2c_wait_ack();

		if (ack == true)
			{
				for (idx = 0; idx < dataLen; idx++)
					{
						i2c_rcv_byte(&dataBuf[idx]);

						if (idx < dataLen-1) i2c_ack();
						else i2c_noack();
					}
			}

		i2c_stop();

		irq_restore(r);
		return (ack?0:-1);
	}

void i2c_start(void)
	{
		//init_i2c();

		i2c_sda_pin_mode_set(GPIO_OUTPUT_MODE, 1);

		i2c_scl_pin_set(1);
		i2c_sda_pin_set(1);
		i2c_delay(I2C_CLOCK);

		i2c_sda_pin_set(0);
		i2c_delay(I2C_CLOCK);

		i2c_scl_pin_set(0);
		i2c_delay(I2C_CLOCK);
	}

void i2c_stop(void)
	{
		i2c_sda_pin_mode_set(GPIO_OUTPUT_MODE, 0);

		i2c_scl_pin_set(0);
		i2c_sda_pin_set(0);
		i2c_delay(I2C_CLOCK);

		i2c_scl_pin_set(1);
		i2c_delay(I2C_CLOCK);

		i2c_sda_pin_set(1);
		i2c_delay(I2C_CLOCK);
	}

#endif // #if USE_I2C_DRV_SW

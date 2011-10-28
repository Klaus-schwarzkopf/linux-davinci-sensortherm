/*
 * rw_i2c_reg.c
 *
 * mt9v126_read_reg based on MT9V113 driver from Ridgerun (mt9v113_davinci.c)
 *
 *  Created on: 20.01.2011
 *      Author: Klaus Schwarzkopf <schwarzkopf@sensortherm.de>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "rw_i2c_reg.h"


int reg_read(struct i2c_client *client, u16 reg) {
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	unsigned short val = 0;

	//printk( KERN_INFO "reg_read ");

	if (!client->adapter) {
		err = -ENODEV;
		return err;
	} else {
		// TODO: addr should be set up where else
		msg->addr = client->addr;//client->addr;
		msg->flags = 0;
		msg->len = I2C_TWO_BYTE_TRANSFER;
		msg->buf = data;
		data[0] = (reg & I2C_TXRX_DATA_MASK_UPPER) >> I2C_TXRX_DATA_SHIFT;
		data[1] = (reg & I2C_TXRX_DATA_MASK);
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			msg->flags = I2C_M_RD;
			msg->len = I2C_TWO_BYTE_TRANSFER; /* 2 byte read */
			err = i2c_transfer(client->adapter, msg, 1);
			if (err >= 0) {
				val = ((data[0] & I2C_TXRX_DATA_MASK) << I2C_TXRX_DATA_SHIFT)
						| (data[1] & I2C_TXRX_DATA_MASK);
			}else{
				printk	(KERN_INFO "\n I2C write failed");
				return err;
			}
		}else{
			printk	(KERN_INFO "\n I2C write failed");
			return err;
		}
	}

	//printk( KERN_INFO "reg=0x%04x, val=0x%04x\n", reg,(0x0000ffff & val));

	return (int) (0x0000ffff & val);
}

/*
 * mt9v126_write_reg based on MT9V113 driver from Ridgerun (mt9v113_davinci.c)
 */

int reg_write(struct i2c_client *client, u16 reg, u16 val) {
	int err = 0;
	int trycnt = 0;

	struct i2c_msg msg[1];
	unsigned char data[4];
	err = -1;

	//printk( KERN_INFO "mt9v126_reg_write reg=0x%04x, val=0x%04x\n",	reg, val );


	while ((err < 0) && (trycnt < I2C_RETRY_COUNT)) {
		trycnt++;
		if (!client->adapter) {
			err = -ENODEV;
		} else {
			// TODO: addr should be set up where else
			msg->addr = client->addr;//client->addr;
			msg->flags = 0;
			msg->len = I2C_FOUR_BYTE_TRANSFER;
			msg->buf = data;
			data[0] = (reg & I2C_TXRX_DATA_MASK_UPPER) >> I2C_TXRX_DATA_SHIFT;
			data[1] = (reg & I2C_TXRX_DATA_MASK);
			data[2] = (val & I2C_TXRX_DATA_MASK_UPPER) >> I2C_TXRX_DATA_SHIFT;
			data[3] = (val & I2C_TXRX_DATA_MASK);
			err = i2c_transfer(client->adapter, msg, 1);
		}
	}
	if (err < 0) {
		printk	(KERN_INFO "\n I2C write failed");
	}
	return err;
}


int reg_set(struct i2c_client *client, const u16 reg, const u16 data) {

	int ret;

	//printk( KERN_INFO "mt9v126_reg_set\n" );
	ret = reg_read(client, reg);
	if (ret < 0)
		return ret;
	return reg_write(client, reg, ret | data);
}

int reg_clear(struct i2c_client *client, const u16 reg, const u16 data) {
	int ret;

	//printk( KERN_INFO "mt9v126_reg_clear\n" );
	ret = reg_read(client, reg);
	if (ret < 0)
		return ret;
	return reg_write(client, reg, ret & ~data);
}



/*
 * write_regs : Initializes a list of registers
 *		if token is TOK_TERM, then entire write operation terminates
 *		if token is TOK_DELAY, then a delay of 'val' msec is introduced
 *		if token is TOK_SKIP, then the register write is skipped
 *		if token is TOK_WRITE, then the register write is performed
 *
 * reglist - list of registers to be written
 * Returns zero if successful, or non-zero otherwise.
 */
int write_regs(struct i2c_client *client,
			      const struct register_value reglist[])
{
	int err;
	int trycnt = 0;
	const struct register_value *next = reglist;

	for (; next->token != TOK_TERM; next++) {
		trycnt = 0;
		if (next->token == TOK_DELAY) {
			msleep(next->val);
			continue;
		}
		//FIXME

		if (next->token == TOK_POLL) {
			do {
				trycnt++;
				msleep(10);
				err = reg_read(client,REG_COMMAND_REGISTER);
				if (err < 0) {
					printk( KERN_INFO  "Write failed. Err[%d]\n", err);
					return err;
				}



			}while((err & 0xff) != CAMERA_ENOERR && (trycnt < I2C_RETRY_COUNT) );
			//printk( KERN_INFO  "Returncode: 0x%02x\n", err);
			continue;
		}
		if (next->token == TOK_SKIP)
			continue;

		if (next->token == TOK_WRITE)
		{

			err = reg_write(client, next->reg, next->val);
			if (err < 0) {
				printk( KERN_INFO  "Write failed. Err[%d]\n", err);
				return err;
			}
		}
	}
	return 0;
}

/*
 * rw_i2c_reg.h
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

#ifndef RW_I2C_REG_H_
#define RW_I2C_REG_H_

#include <linux/i2c.h>
#include <linux/delay.h>
/*
#define ENOERR 		0x00 	//No error - command was successful
#define ENOENT 		0x01 	//No such entity
#define EINTR 		0x02 	//Operation interrupted
#define EIO 		0x03 	//I/O failure
#define E2BIG 		0x04 	//Too big
#define EBADF 		0x05 	//Bad file/handle
#define EAGAIN 		0x06 	//Would-block, try again
#define ENOMEM 		0x07 	//Not enough memory/resource
#define EACCES 		0x08 	//Permission denied
#define EBUSY 		0x09 	//Entity busy, cannot support operation
#define EEXIST 		0x0A 	//Entity exists
#define ENODEV 		0x0B 	//Device not found
#define EINVAL 		0x0C 	//Invalid argument
#define ENOSPC 		0x0D 	//No space/resource to complete
#define ERANGE 		0x0E 	//Parameter out of range
#define ENOSYS 		0x0F 	//Operation not supported
#define EALREADY 	0x10 	//Already requested/exists
*/



#define REG_COMMAND_REGISTER			0x0040
#define VAL_DOORBELL					0x8000

#define I2C_ONE_BYTE_TRANSFER		(1)
#define I2C_TWO_BYTE_TRANSFER		(2)
#define I2C_THREE_BYTE_TRANSFER		(3)
#define I2C_FOUR_BYTE_TRANSFER		(4)
#define I2C_TXRX_DATA_MASK		(0x00FF)
#define I2C_TXRX_DATA_MASK_UPPER	(0xFF00)
#define I2C_TXRX_DATA_SHIFT		(8)

#define I2C_RETRY_COUNT                 (5)
#define LOCK_RETRY_COUNT                (5)
#define LOCK_RETRY_DELAY                (200)

/* Tokens for register write */
#define TOK_WRITE                       (0)     /* token for write operation */
#define TOK_TERM                        (1)     /* terminating token */
#define TOK_DELAY                       (2)     /* delay token for reg list */
#define TOK_SKIP                        (3)     /* token to skip a register */
#define TOK_POLL                        (4)     /* token for poll command_register::doorbell =>0x00 */
/**
 * struct register_value - Structure for TVP5146/47 register initialization values
 * @token - Token: TOK_WRITE, TOK_TERM etc..
 * @reg - Register offset
 * @val - Register Value for TOK_WRITE or delay in ms for TOK_DELAY
 */
struct register_value {
	unsigned short token;
	unsigned short reg;
	unsigned short val;
};

int reg_clear(struct i2c_client *client, const u16 reg, const u16 data) ;
int reg_set(struct i2c_client *client, const u16 reg, const u16 data) ;
int reg_write(struct i2c_client *client,  u16 reg,  u16 data) ;
int reg_read(struct i2c_client *client,  u16 reg) ;
int write_regs(struct i2c_client *client, const struct register_value reglist[]);

#endif /* RW_I2C_REG_H_ */

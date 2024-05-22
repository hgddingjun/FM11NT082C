/*************************************************************************
 * Copyright(C)2024 Duowei Inc.
 * fm11nt082c NFC IC driver.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *************************************************************************/

/*************************************************************************
 *
 *	[Create by Dingjun 2024.04.17]
 *
 *************************************************************************/
#ifndef __FM11N082C_I2C__
#define __FM11N082C_I2C__

typedef struct fm11nt082c_info {
	int nfc_3v3_gpio;
	int nfc_i2c_gpio;
	int nfc_csn_gpio;
	int nfc_irq;
	int nfc_irq_gpio;
	int i2c_clk_gpio;
	int i2c_sda_gpio;
} st_fm11nt082c_info;

#endif

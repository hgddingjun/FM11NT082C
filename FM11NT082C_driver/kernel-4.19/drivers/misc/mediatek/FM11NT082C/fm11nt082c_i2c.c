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

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>


#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif

#include "fm11nt082c_i2c.h"
#include "FM11.h"

#define LIMIT_LENGTH    (127)
#define EEPROM_START_ADDR (0x0000)
#define EEPROM_SIZE    (1024)

#define ENCRYPT_BUF_SIZE    (64)
#define SSID_BUF_SIZE     (128)
#define PASSWD_BUF_SIZE    (128)

#define HARDWARE_IIC_READ_WRITE (0)

#ifndef C_I2C_FIFO_SIZE
#define C_I2C_FIFO_SIZE		8
#endif

extern uint8_t FlagFirstFrame;	//卡片首帧标识

extern uint8_t irq_data_in;	//非接数据接收终端标识
extern uint8_t irq_rxdone;
extern uint8_t irq_txdone;


extern struct task_struct *thread_task;
/*\
typedef struct fm11nt082c_info {
	int nfc_3v3_gpio;
	int nfc_i2c_gpio;
	int nfc_csn_gpio;
	int nfc_irq_gpio;
	int i2c_clk_gpio;
	int i2c_sda_gpio;
} st_fm11nt082c_info;
\*/

/*static*/ st_fm11nt082c_info *p_nfc_info = NULL;

static char gStrEncrypt[ENCRYPT_BUF_SIZE];
static char gStrSSID[SSID_BUF_SIZE];
static char gStrPasswd[PASSWD_BUF_SIZE];

static uint8_t encrypt_length = 0; //加密类型字符串长度
static uint8_t ssid_length = 0;   //wifi热点名称字符串长度
static uint8_t password_length = 0; //wifi密码字符串长度
static uint8_t total_lengh = 0;
static uint8_t payload_length = 0;
static uint8_t wifi_head_length = 0;

/* wifi类型为 'application/vnd.wfa.wsc' fix type, fix string */
static const uint8_t wifi_type[23] = {0x61, 0x70, 0x70, 0x6C, 0x69, 0x63, 0x61, 0x74, 0x69, 0x6F, 0x6E, 0x2F, 0x76, 0x6E, 0x64, 0x2E, 0x77, 0x66, 0x61, 0x2E, 0x77, 0x73, 0x63};
static const uint8_t fixdata_1[5] = {0x10, 0x4A, 0x00, 0x01, 0x10};
static const uint8_t wifi_header[3] = {0x10, 0x0e, 0x00}; // length
static const uint8_t wifi_id[5] = {0x10, 0x26, 0x00, 0x01, 0x01};
static const uint8_t wifi_auth_type[6] = {0x10, 0x03, 0x00, 0x02, 0x00, 0x20};
static const uint8_t wifi_encrypt_type[6] = {0x10, 0x0f, 0x00, 0x02, 0x00, 0x01};
static const uint8_t wifi_mac[10] = {0x10, 0x20, 0x00, 0x06, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static const uint8_t fixdata_2[5] = {0xFF, 0xFF, 0x00, 0x01, 0x00};

#if HARDWARE_IIC_READ_WRITE
int fm11nt082c_i2c_write_add16_data32(struct i2c_client *client, u16 addr, u32 value)
{
	u8 buffer[6];
	buffer[0] = addr >> 8;
	buffer[1] = addr & 0xff;
	buffer[2] = value & 0xff;
	buffer[3] = (value >> 8) & 0xff;
	buffer[4] = (value >> 16) & 0xff;
	buffer[5] = value >> 24;
	return i2c_master_send(client, buffer, 6);
}
EXPORT_SYMBOL(fm11nt082c_i2c_write_add16_data32);


u32 fm11nt082c_i2c_read_add16_data32(struct i2c_client * client, u16 addr)
{
	struct i2c_msg msgs[2];
	u8 tx_buf[2], rx_buf[4];

	/* Write register address */
	tx_buf[0] = addr >> 8;
	tx_buf[1] = addr & 0xff;
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (char *) tx_buf;

	/* Read data from registers */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 4;
	msgs[1].buf = (char *) rx_buf;

	if (i2c_transfer(client->adapter, msgs, 2) < 2)
		return 0;

	return (rx_buf[3] << 24) | (rx_buf[2] << 16) | (rx_buf[1] << 8) |
		rx_buf[0];
}
EXPORT_SYMBOL(fm11nt082c_i2c_read_add16_data32);

int fm11nt082c_i2c_write_add16_data8(struct i2c_client *client, u16 addr, u8 value)
{
	u8 buffer[3];
	buffer[0] = addr >> 8;
	buffer[1] = addr & 0xff;
	buffer[2] = value;
	return i2c_master_send(client, buffer, 3);
}
EXPORT_SYMBOL(fm11nt082c_i2c_write_add16_data8);

u8 fm11nt082c_i2c_read_add16_data8(struct i2c_client * client, u16 addr)
{
	struct i2c_msg msgs[2];
	u8 tx_buf[2], rx_buf[1];

	/* Write register address */
	tx_buf[0] = addr >> 8;
	tx_buf[1] = addr & 0xff;
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (char *) tx_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = (char *) rx_buf;

	if (i2c_transfer(client->adapter, msgs, 2) < 2)
		return 0;

	return rx_buf[0];
}
EXPORT_SYMBOL(fm11nt082c_i2c_read_add16_data8);


int fm11nt082c_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 reg_addr = addr;
	u8 *rxbuf = data;
	u8 left = len;
	u8 retry;
	u8 offset = 0;

	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &reg_addr,
			.len = 1,
		},
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		},
	};

	if (rxbuf == NULL)
		return -1;

	while (left > 0) {
		retry = 0;
		reg_addr = addr + offset;
		msg[1].buf = &rxbuf[offset];

		if (left > C_I2C_FIFO_SIZE) {
			msg[1].len = C_I2C_FIFO_SIZE;
			left -= C_I2C_FIFO_SIZE;
			offset += C_I2C_FIFO_SIZE;
		} else {
			msg[1].len = left;
			left = 0;
		}

		while (i2c_transfer(client->adapter, &msg[0], 2) != 2) {
			retry++;

			if (retry == 20) {
				pr_err("i2c read reg=%#x length=%d failed\n",
					addr + offset, len);
				return -EIO;
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL(fm11nt082c_i2c_read_block);

int fm11nt082c_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 buffer[C_I2C_FIFO_SIZE];
	u8 *txbuf = data;
	u8 left = len;
	u8 offset = 0;
	u8 retry = 0;

	struct i2c_msg msg = {
		.addr = client->addr, .flags = 0, .buf = buffer,
	};

	if (txbuf == NULL)
		return -1;

	while (left > 0) {
		retry = 0;
		/* register address */
		buffer[0] = addr + offset;

		if (left >= C_I2C_FIFO_SIZE) {
			memcpy(&buffer[1], &txbuf[offset], C_I2C_FIFO_SIZE - 1);
			msg.len = C_I2C_FIFO_SIZE;
			left -= C_I2C_FIFO_SIZE - 1;
			offset += C_I2C_FIFO_SIZE - 1;
		} else {
			memcpy(&buffer[1], &txbuf[offset], left);
			msg.len = left + 1;
			left = 0;
		}

		while (i2c_transfer(client->adapter, &msg, 1) != 1) {
			retry++;

			if (retry == 20) {
				pr_err("i2c write reg=%#x length=%d failed\n",
					buffer[0], len);
				return -EIO;
			}

			pr_debug("i2c write addr %#x, retry %d\n", buffer[0],
				retry);
		}
	}

	return 0;
}
EXPORT_SYMBOL(fm11nt082c_i2c_write_block);
#endif/* HARDWARE_IIC_READ_WRITE */

/*\
	nfc_3v3_gpio;
	nfc_i2c_gpio;
	nfc_csn_gpio;
	nfc_irq_gpio;
	i2c_clk_gpio;
	i2c_sda_gpio;
\*/
static irqreturn_t nfc_irq_handler(int irq, void *dev_id)
{
	if(Transive_mode)//	走通道模式中断处理
	{
		uint8_t ret = 0;
		int irq_gpio;
		irq_gpio = IRQ_READ;
		pr_info("%s() irq_gpio=0x%x\n", __func__, irq_gpio);
		if( irq_gpio != RESET) //确保是否产生了EXTI Line中断
		{
			ret = FM11_ReadReg(MAIN_IRQ_REG);
			pr_info("%s() ret=0x%x\n", __func__, ret);
			if(ret & MAIN_IRQ_RX_START)      irq_data_in = 1;
			if(ret & MAIN_IRQ_RX_DONE)       irq_rxdone = 1;
			if(ret & MAIN_IRQ_TX_DONE)       irq_txdone = 1;
			if(ret & MAIN_IRQ_ACTIVE)       FlagFirstFrame = ON;
			if(ret & MAIN_IRQ_FIFO)
			{
				FM11_ReadReg(FIFO_IRQ_REG);	//读剩余中断寄存器，保证中断拉高
			}
			if(ret & MAIN_IRQ_AUX)
			{
				FM11_ReadReg(AUX_IRQ_REG);
			}
			//EXTI_ClearITPendingBit(EXTI_Line8); //清中断
		}
	}

	return IRQ_HANDLED;
}

static int fm11nt082c_init_gpio_dts(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node* node = NULL;
	node = of_find_compatible_node(NULL, NULL, "fudanmicro,fm11nt082c_nfc");
	if(node) {
		p_nfc_info->nfc_3v3_gpio = of_get_named_gpio(node, "nfc-3v3-en",0);
		ret = gpio_request(p_nfc_info->nfc_3v3_gpio, "nfc-3v3-en control pin");
		if(ret<0 && ret!=-EBUSY)
		{
			pr_err("%s,%d,-gpio_request[%d] failed: %d\n", p_nfc_info->nfc_3v3_gpio, ret);
			return ret;
		}
		if( !gpio_is_valid(p_nfc_info->nfc_3v3_gpio) )
		{
			return -ENODEV;
		}

		p_nfc_info->nfc_i2c_gpio = of_get_named_gpio(node, "nfc-i2c-en",0);
		ret = gpio_request(p_nfc_info->nfc_i2c_gpio, "nfc-i2c-en control pin");
		if(ret<0 && ret!=-EBUSY)
		{
			pr_err("%s,%d,-gpio_request[%d] failed: %d\n", p_nfc_info->nfc_i2c_gpio, ret);
			return ret;
		}
		if( !gpio_is_valid(p_nfc_info->nfc_i2c_gpio) )
		{
			return -ENODEV;
		}

		p_nfc_info->nfc_csn_gpio = of_get_named_gpio(node, "nfc-csn-en",0);
		ret = gpio_request(p_nfc_info->nfc_csn_gpio, "nfc-csn-en control pin");
		if(ret<0 && ret!=-EBUSY)
		{
			pr_err("%s,%d,-gpio_request[%d] failed: %d\n", p_nfc_info->nfc_csn_gpio, ret);
			return ret;
		}
		if( !gpio_is_valid(p_nfc_info->nfc_csn_gpio) )
		{
			return -ENODEV;
		}

		p_nfc_info->nfc_irq_gpio = of_get_named_gpio(node, "nfc-irq-pin",0);
		ret = gpio_request(p_nfc_info->nfc_irq_gpio, "nfc-irq-pin control pin");
		if(ret<0 && ret!=-EBUSY)
		{
			pr_err("%s,%d,-gpio_request[%d] failed: %d\n", p_nfc_info->nfc_irq_gpio, ret);
			return ret;
		}
		if( !gpio_is_valid(p_nfc_info->nfc_irq_gpio) )
		{
			return -ENODEV;
		}

		p_nfc_info->nfc_irq = gpio_to_irq(p_nfc_info->nfc_irq_gpio);
		if (!p_nfc_info->nfc_irq)
		{
			pr_err("p_nfc_info->nfc_irq irq_of_parse_and_map(..) fail\n");
			ret = -EINVAL;
			return ret;
		}

		pr_info("%s,%d-request handler for p_nfc_info->nfc_irq ID IRQ: %d\n",__func__, __LINE__, p_nfc_info->nfc_irq);
		ret = request_irq(p_nfc_info->nfc_irq, nfc_irq_handler, IRQ_TYPE_LEVEL_LOW, "nfc-irq", NULL);
		if (ret) {
			pr_err("p_nfc_info->nfc_irq irq thread request failed, ret=%d\n", ret);
			return ret;
		}

		enable_irq_wake(p_nfc_info->nfc_irq);

		p_nfc_info->i2c_clk_gpio = of_get_named_gpio(node, "i2c-clk-pin",0);
		ret = gpio_request(p_nfc_info->i2c_clk_gpio, "i2c-clk-pin control pin");
		if(ret<0 && ret!=-EBUSY)
		{
			pr_err("%s,%d,-gpio_request[%d] failed: %d\n", p_nfc_info->i2c_clk_gpio, ret);
			return ret;
		}
		if( !gpio_is_valid(p_nfc_info->i2c_clk_gpio) )
		{
			return -ENODEV;
		}

		p_nfc_info->i2c_sda_gpio = of_get_named_gpio(node, "i2c-sda-pin",0);
		ret = gpio_request(p_nfc_info->i2c_sda_gpio, "i2c-sda-pin control pin");
		if(ret<0 && ret!=-EBUSY)
		{
			pr_err("%s,%d,-gpio_request[%d] failed: %d\n", p_nfc_info->i2c_sda_gpio, ret);
			return ret;
		}
		if( !gpio_is_valid(p_nfc_info->i2c_sda_gpio) )
		{
			return -ENODEV;
		}
	}
	return ret;

}

static ssize_t show_dump_eeprom(struct device *dev, struct device_attribute *attr,char *buf)
{
	int i=0;
	int ret=0;
	uint8_t status;
    uint8_t eeprom[EEPROM_SIZE]={0};

	NSS_ON;//CSN管脚拉低，延时300us，给芯片上电
	udelay(250);
	status = FM11_ReadE2(eeprom, EEPROM_START_ADDR, EEPROM_SIZE);
	udelay(1000);//很重要，复位时间 1ms
	NSS_OFF;//CSN管脚拉高
	if(0 == status) {
		pr_err("FM11_ReadE2 Error!\n");
		return ret;
	}

	for(i=0; i<EEPROM_SIZE; i+=16) {
		ret += sprintf(buf + ret, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
		          eeprom[0+i],eeprom[1+i],eeprom[2+i],eeprom[3+i],eeprom[4+i],eeprom[5+i],eeprom[6+i],eeprom[7+i],
				  eeprom[8+i],eeprom[9+i],eeprom[10+i],eeprom[11+i],eeprom[12+i],eeprom[13+i],eeprom[14+i],eeprom[15+i]);
	}
	pr_info("%s,%d ret=%d\n",__func__,__LINE__,ret);
	return ret;
}


void write_ndef_user_data(void)
{
	uint8_t status;
	total_lengh = 75 + ssid_length + password_length;
	payload_length = total_lengh - 26;
	wifi_head_length = payload_length - 9;

	pr_info("FUNC:%s() LINE:%d total_lengh=%d, payload_length=%d, ssid_length=%d, password_length=%d\n",
	    __func__,__LINE__,
	    total_lengh, payload_length, ssid_length, password_length);

	ndef_file[0] = 0x03;
	ndef_file[1] = total_lengh;// 0x5D 最后计算
	ndef_file[2] = 0xD2; ndef_file[3] = 0x17; ndef_file[4] = payload_length; /*0x35+4+5+5*/
	memcpy(&ndef_file[5], wifi_type, 23); //according to wifi_type array length.
	memcpy(&ndef_file[28], fixdata_1, 5);
	memcpy(&ndef_file[33], wifi_header, 3);
	ndef_file[36] = wifi_head_length; /* 0x3A length*/
	memcpy(&ndef_file[37], wifi_id, 5);

	/*--------------wifi ssid--------------*/
	ndef_file[42] = 0x10; ndef_file[43] = 0x45; ndef_file[44] = 0x00;
	ndef_file[45] = ssid_length;
	memcpy(&ndef_file[46], gStrSSID, ssid_length);
	/*--------------wifi ssid--------------*/

	memcpy(&ndef_file[46 + ssid_length], wifi_auth_type, 6);
	memcpy(&ndef_file[46 + ssid_length + 6], wifi_encrypt_type, 6);

	/*************** wifi passwd ***************/
	ndef_file[46 + ssid_length + 6 + 6] = 0x10;
	ndef_file[46 + ssid_length + 6 + 6 + 1] = 0x27;
	ndef_file[46 + ssid_length + 6 + 6 + 1 + 1] = 0x00;
	ndef_file[46 + ssid_length + 6 + 6 + 1 + 1 + 1] = password_length;  //password length

	memcpy(&ndef_file[46 + ssid_length + 6 + 6 + 1 + 1 + 1 + 1], gStrPasswd, password_length);
	/*************** wifi passwd ***************/

	memcpy(&ndef_file[46 + ssid_length + 6 + 6 + 1 + 1 + 1 + 1 + password_length], wifi_mac, 10);
	memcpy(&ndef_file[46 + ssid_length + 6 + 6 + 1 + 1 + 1 + 1 + password_length + 10], fixdata_2, 5);

	ndef_file[46 + ssid_length + 6 + 6 + 1 + 1 + 1 + 1 + password_length + 10 + 5] = 0xFE;

	NSS_ON;//CSN管脚拉低，延时300us，给芯片上电
	udelay(250);
	status = FM11_WriteE2(FM441_NDEF_Header_EEaddress, total_lengh + 3, ndef_file);
	udelay(1000);//很重要，复位时间 1ms
	NSS_OFF;//CSN管脚拉高


	if(0 == status) {
		pr_err("write_ndef_user_data FM11_WriteE2 Error!\n");
		return;
	}
	pr_info("FUNC:%s() LINE:%d Write EEprom Successfly!\n",__func__,__LINE__);

}

static ssize_t store_dump_eeprom(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int temp = 0;
	if(1 == sscanf(buf, "%d", &temp))
	{
		if(temp==1){
			/*写EEPROM...*/
			pr_info("FUNC:%s() LINE:%d \n",__func__,__LINE__);
			write_ndef_user_data();
		}
	}
	else
	{
		pr_err("invalid format = '%s'\n", buf);
	}
	return size;
}

static DEVICE_ATTR(dumpE2P, 0664, show_dump_eeprom, store_dump_eeprom);


static ssize_t show_encrypt_eeprom(struct device *dev, struct device_attribute *attr,char *buf)
{
	pr_info("%s,%d gStrEncrypt=%s\n",__func__,__LINE__,gStrEncrypt);
	return sprintf(buf, "%s", gStrEncrypt);
}
static ssize_t store_encrypt_eeprom(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	if (size > LIMIT_LENGTH/*PAGE_SIZE - 1*/) {
        return -EINVAL;
	}

	memset(gStrEncrypt, 0x00, ENCRYPT_BUF_SIZE);
	strncpy(gStrEncrypt, buf, size);
	pr_info("%s,%d gStrEncrypt=%s, size=%d\n",__func__,__LINE__,gStrEncrypt,size);
	/*写EEPROM...*/
	encrypt_length = size;
	return size;
}
static DEVICE_ATTR(encrypt, 0664, show_encrypt_eeprom, store_encrypt_eeprom);


static ssize_t show_ssid_eeprom(struct device *dev, struct device_attribute *attr,char *buf)
{
	pr_info("%s,%d gStrSSID=%s\n",__func__,__LINE__,gStrSSID);
	return sprintf(buf, "%s", gStrSSID);
}
static ssize_t store_ssid_eeprom(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	if (size > LIMIT_LENGTH/*PAGE_SIZE - 1*/) {
		pr_err("ERROR: SSID length exceeds 255\n");
        return -EINVAL;
	}

	memset(gStrSSID, 0x00, SSID_BUF_SIZE);
	strncpy(gStrSSID, buf, size);
	ssid_length = size;
	pr_info("%s,%d gStrSSID=%s, size=%d\n",__func__,__LINE__,gStrSSID,size);
	/*写EEPROM...*/
	return size;
}
static DEVICE_ATTR(ssid, 0664, show_ssid_eeprom, store_ssid_eeprom);


static ssize_t show_password_eeprom(struct device *dev, struct device_attribute *attr,char *buf)
{
	pr_info("%s,%d gStrPasswd=%s\n",__func__,__LINE__,gStrPasswd);
	return sprintf(buf, "%s", gStrPasswd);
}
static ssize_t store_password_eeprom(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	if (size > LIMIT_LENGTH/*PAGE_SIZE - 1*/) {
		pr_err("ERROR: Password length exceeds 255\n");
        return -EINVAL;
	}

	strncpy(gStrPasswd, buf, size);
	password_length = size;
	pr_info("%s,%d gStrPasswd=%s, size=%d\n",__func__,__LINE__,gStrPasswd,size);
	return size;
}
static DEVICE_ATTR(password, 0664, show_password_eeprom, store_password_eeprom);


static int fm11nt082c_nfc_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//int i;
	int ret = 0;
	//u8 readData[8]={0};
	struct device *dev = &client->dev;

	struct platform_device *pdev = to_platform_device(dev);

	pr_info("Probing fm11nt082c nfc I2C device: %s\n", client->name);

	memset(gStrEncrypt, 0x00, sizeof(gStrEncrypt));
	memset(gStrSSID, 0x00, sizeof(gStrSSID));
	memset(gStrPasswd, 0x00, sizeof(gStrPasswd));

	p_nfc_info = devm_kzalloc(dev, sizeof(*p_nfc_info), GFP_KERNEL);
	if (!p_nfc_info)
		return -ENOMEM;

	ret = fm11nt082c_init_gpio_dts(pdev);

	gpio_direction_output(p_nfc_info->nfc_3v3_gpio, 1);
	gpio_direction_output(p_nfc_info->nfc_i2c_gpio, 1);
	//gpio_direction_output(p_nfc_info->nfc_csn_gpio, 1);
	//gpio_direction_output(p_nfc_info->nfc_irq_gpio, 1);

	ret = device_create_file(&(pdev->dev), &dev_attr_dumpE2P);
	if (ret) {
		pr_err("dev_attr_dumpE2P fail!\n");
		goto err_dumpE2P;
	}

	ret = device_create_file(&(pdev->dev), &dev_attr_encrypt);
	if (ret) {
		pr_err("dev_attr_encrypt fail!\n");
		goto err_encrypt;
	}

	ret = device_create_file(&(pdev->dev), &dev_attr_ssid);
	if (ret) {
		pr_err("dev_attr_ssid fail!\n");
		goto err_ssid;
	}

	ret = device_create_file(&(pdev->dev), &dev_attr_password);
	if (ret) {
		pr_err("dev_attr_password fail!\n");
		goto err_password;
	}

#if HARDWARE_IIC_READ_WRITE
	fm11nt082c_i2c_read_block(client, 0x00, readData, 8);
	for(i=0;i<8;i++) {
		pr_info("readData[%d]=0x%02X ", i, readData[i]);
	}
	pr_info("\n");
#endif

	FM11_Run();

	pr_info("%s() successfully! ret=%d\n",__func__,ret);

    return 0;

err_password:
	device_remove_file(&(pdev->dev), &dev_attr_password);
	return -1;

err_ssid:
	device_remove_file(&(pdev->dev), &dev_attr_ssid);
	return -1;

err_encrypt:
	device_remove_file(&(pdev->dev), &dev_attr_encrypt);
	return -1;

err_dumpE2P:
	device_remove_file(&(pdev->dev), &dev_attr_dumpE2P);
	return -1;
}

static int fm11nt082c_nfc_i2c_remove(struct i2c_client *client)
{

	if(p_nfc_info) {
		kfree(p_nfc_info);
		p_nfc_info = NULL;
	}
	pr_info("Removing fm11nt082c nfc I2C device: %s\n", client->name);

	// 停止内核线程
    if (thread_task) {
        kthread_stop(thread_task);
    }
    return 0;
}

// I2C 设备 ID 列表
static const struct i2c_device_id fm11nt082c_nfc_i2c_id[] = {
    { "fm11nt082c_i2c", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, fm11nt082c_nfc_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id fm11nt082c_nfc_i2c_of_table[] = {
	{ .compatible = "fudanmicro,fm11nt082c_nfc" },
	{ },
};
MODULE_DEVICE_TABLE(of, fm11nt082c_nfc_i2c_of_table);
#endif

// I2C 驱动程序结构体
static struct i2c_driver fm11nt082c_nfc_i2c_driver = {
    .driver = {
        .name = "fm11nt082c_driver",
        .owner = THIS_MODULE,
		#ifdef CONFIG_OF
        .of_match_table = fm11nt082c_nfc_i2c_of_table,
		#endif
    },
    .probe = fm11nt082c_nfc_i2c_probe,
    .remove = fm11nt082c_nfc_i2c_remove,
    .id_table = fm11nt082c_nfc_i2c_id,
};

// 模块初始化函数
static int __init fm11nt082c_nfc_i2c_init(void)
{
    // 注册 I2C 驱动程序
    return i2c_add_driver(&fm11nt082c_nfc_i2c_driver);
}

// 模块卸载函数
static void __exit fm11nt082c_nfc_i2c_exit(void)
{
    // 移除 I2C 驱动程序
    i2c_del_driver(&fm11nt082c_nfc_i2c_driver);
}

module_init(fm11nt082c_nfc_i2c_init);
module_exit(fm11nt082c_nfc_i2c_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("dingjun@sz-duowei.com");
MODULE_DESCRIPTION("fm11nt082c I2C Driver");

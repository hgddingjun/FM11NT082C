#include "iic.h"

void I2C_delay(void)
{
	uint8_t i=10; //
	while(i)
	{
		i--;
	}
}

uint8_t I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return FALSE;	//SDA线为低电平则总线忙,退出
	SDA_L;
	I2C_delay();
	if(SDA_read) return FALSE;	//SDA线为高电平则总线出错,退出
	SDA_L;
	I2C_delay();
	return TRUE;
}

void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
}

void I2C_Ack(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;		//ACK = 0
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

void I2C_NoAck(void)
{
	SCL_L;
	I2C_delay();
	SDA_H;		//ACK = 1
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}


uint8_t I2C_WaitAck(void)
{
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read) //ACK =0
	{
		SCL_L;
		return FALSE;
	}
	SCL_L;
	return TRUE;
}

static void I2C_SendByte(uint8_t SendByte)  //高位在前 SCL_L SDA变化
{
	uint8_t i=8;
	while(i--)
	{
		SCL_L;
		I2C_delay();
		if(SendByte&0x80)
			SDA_H;
		else
			SDA_L;
		SendByte<<=1;
		I2C_delay();
		SCL_H;
		I2C_delay();
	}
	SCL_L;
}

static uint8_t I2C_ReceiveByte(void)
{
	uint8_t i=8;
	uint8_t ReceiveByte=0;

	SDA_H;
	while(i--)
	{
		ReceiveByte<<=1;
		SCL_L;
		I2C_delay();
		SCL_H;
		I2C_delay();
		if(SDA_read)
		{
			ReceiveByte|=0x01;
		}
	}
	SCL_L;
	return ReceiveByte;
}


/**
  * @brief  Wait for EEPROM Standby state
  * @param  None
  * @retval None
  */
void sEE_WaitEEStandbyState(void)
{
	do {
		I2C_Start();
		I2C_SendByte(I2C_SLAVE_ADDRESS7);
	}while(!I2C_WaitAck());
	I2C_Stop();
}


uint8_t sEE_WritePage(uint8_t *pBuffer, uint16_t WriteAddr, uint8_t NumByteToWrite)	// FM441擦写eeprom完有中断，所以现在擦写eeprom也许注意中断屏蔽
{
	unsigned long flags;

	//__disable_irq();
	local_irq_save(flags);
	if(!I2C_Start())
	{
		//__enable_irq();
		local_irq_restore(flags);
		return FALSE;
	}

	I2C_SendByte(I2C_SLAVE_ADDRESS7 & 0xFE);	//Addr + Write 0

	if(!I2C_WaitAck())	//必须加判断，否则有可能出现擦写SAK出错，不加判断就以为成功，改写CRC8出错
	{
		I2C_Stop();
		//__enable_irq();
		local_irq_restore(flags);
		return FALSE;
	}

	I2C_SendByte((uint8_t)((WriteAddr & 0xFF00) >> 8));	//高地址先发
	I2C_WaitAck();
	I2C_SendByte((uint8_t)(WriteAddr & 0x00FF));
	I2C_WaitAck();

	while(NumByteToWrite--)
	{
		I2C_SendByte(*pBuffer);
		I2C_WaitAck();
		pBuffer++;
	}
	I2C_Stop();

	mdelay(10);	//STOP后开始擦写，FM441修改为5ms，FM365为10ms, 貌似5ms有问题还是按照10ms来
	//__enable_irq();
	local_irq_restore(flags);
	return TRUE;
}

uint8_t FM11_WriteE2(uint16_t adr,uint32_t len,uint8_t *wbuf)
{
	uint8_t offset;

	if(adr < FM11_E2_USER_ADDR || adr >= FM11_E2_MANUF_ADDR)
		return FALSE;
	if(adr % FM11_E2_BLOCK_SIZE)  //求余 字节对齐
	{
		offset = FM11_E2_BLOCK_SIZE - (adr % FM11_E2_BLOCK_SIZE);
		if(len > offset)
		{
			if(!sEE_WritePage(wbuf,adr,offset))  //先写offset，保证对齐
				return FALSE;
			adr += offset;
			wbuf += offset;
			len -= offset;
		}
		else
		{
			if(!sEE_WritePage(wbuf,adr,len))
				return FALSE;
			len = 0;
		}
	}

	while(len)
	{
		if(len >= FM11_E2_BLOCK_SIZE)
		{
			if(!sEE_WritePage(wbuf,adr,FM11_E2_BLOCK_SIZE))
				return FALSE;
			adr += FM11_E2_BLOCK_SIZE;
			wbuf += FM11_E2_BLOCK_SIZE;
			len -= FM11_E2_BLOCK_SIZE;
		}
		else
		{
			if(!sEE_WritePage(wbuf,adr,len))
				return FALSE;
			len = 0;
		}
	}
	return TRUE;
}

uint8_t FM11_ReadE2(uint8_t *pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead)
{
	unsigned long flags;
	//__disable_irq();
	local_irq_save(flags);
	if(!I2C_Start())
	{
		//__enable_irq();
		local_irq_restore(flags);
		return FALSE;
	}

	I2C_SendByte(I2C_SLAVE_ADDRESS7 & 0xFE);	//Addr + Write 0

	if(!I2C_WaitAck())
	{
		I2C_Stop();
		//__enable_irq();
		local_irq_restore(flags);
		return FALSE;
	}

	I2C_SendByte((uint8_t)((ReadAddr & 0xFF00)>>8));	//高地址先发
	I2C_WaitAck();
	I2C_SendByte((uint8_t)(ReadAddr & 0x00FF));
	I2C_WaitAck();


	I2C_Start();	//重发Start
	I2C_SendByte(I2C_SLAVE_ADDRESS7 | 0x01);	//Addr + Read 1
	I2C_WaitAck();

	while(NumByteToRead)
	{
		*pBuffer = I2C_ReceiveByte();
		if(NumByteToRead == 1)
			I2C_NoAck();
		else
			I2C_Ack();
		pBuffer++;
		NumByteToRead--;
	}
	I2C_Stop();

	//__enable_irq();
	local_irq_restore(flags);
	return TRUE;
}


uint8_t FM11_ReadReg(uint16_t addr)
{
	uint8_t pdata[10];
	uint8_t a;

	if(FM11_ReadE2(pdata, addr, 1))
	{
		a=pdata[0];
		return a;
	}
	else
		return FALSE;
}

uint8_t FM11_WriteReg(uint16_t addr, uint8_t data)
{
	unsigned long flags;
	//__disable_irq();
	local_irq_save(flags);
	if(!I2C_Start())
	{
		//__enable_irq();
		local_irq_restore(flags);
		return FALSE;
	}

	I2C_SendByte(I2C_SLAVE_ADDRESS7 & 0xFE);	//Addr + Write 0
	I2C_WaitAck();

	I2C_SendByte((uint8_t)((addr & 0xFF00) >> 8));
	I2C_WaitAck();
	I2C_SendByte((uint8_t)(addr & 0x00FF));
	I2C_WaitAck();

	I2C_SendByte(data);	//写入值
	I2C_WaitAck();

	I2C_Stop();
	//__enable_irq();
	local_irq_restore(flags);
	return TRUE;
}


//Read FM327 FIFO
uint8_t FM11_ReadFIFO(uint8_t NumByteToRead,uint8_t *pbuf)
{
	if(FM11_ReadE2(pbuf, FM327_FIFO, NumByteToRead))
		return TRUE;
	else
		return FALSE;
}

//WriteFM327 FIFO
uint8_t FM11_WriteFIFO(uint8_t *pbuf, uint8_t NumByteToWrite)
{
	unsigned long flags;
	//__disable_irq();
	local_irq_save(flags);
	if(!I2C_Start())
	{
		//__enable_irq();
		local_irq_restore(flags);
		return FALSE;
	}

	I2C_SendByte(I2C_SLAVE_ADDRESS7 & 0xFE);	//Addr + Write 0
	I2C_WaitAck();

	I2C_SendByte((uint8_t)((0xFFF0 & 0xFF00) >> 8));
	I2C_WaitAck();
	I2C_SendByte((uint8_t)(0xFFF0& 0x00FF));
	I2C_WaitAck();

	while(NumByteToWrite--)
	{
		I2C_SendByte(*pbuf);
		I2C_WaitAck();
		pbuf++;
	}
	I2C_Stop();

	//__enable_irq();
	local_irq_restore(flags);
	return TRUE;
}


void FM11_Init(void)
{
	uint8_t serial_number[9]={0x00};
	uint8_t ATQA_SAK[4]={0x00};
	uint8_t CRC8[13]={0x00};
	uint8_t CRC8_data=0x00;
	uint8_t status1,status2,status3;
	uint8_t SAK2_4[1]={0x20};    //-4 sak
	uint8_t Protocol_4[4]={0x91,0x82,0x21,0xCD};	//通道模式
	uint8_t SAK2_3[1]={0x00};
	uint8_t Protocol_Tag[4]={0x90,0x82,0x21,0xCC};	 //tag模式
	uint8_t rbuf[16]={0x00};   //读取测试buf
	//uint8_t wbuf[16]={0x1d,0x42,0x27,0xf0,0x98,0xd1,0x18,0x2b,0x7a,0x44,0x00,0x04,0x00};   //写入测试buf

	//IIC gpio初始化
	NSS_ON;
	udelay(250);

	if(Transive_mode)
	{
		FM11_WriteReg(RESET_SILENCE,0x33);	//非接复位，处于静默
		status1=FM11_WriteE2(FM441_SAK_Control_EEaddress,1,&SAK2_4[0]);
		FM11_WriteE2(FM441_Protocol_Control_EEaddress,4,Protocol_4);
		status2=FM11_ReadE2(serial_number,FM441_Serial_number_EEaddress ,9);
		status3=FM11_ReadE2(ATQA_SAK,FM441_ATQA_EEaddress,4);

		pr_info("[Transive_mode]->Function:%s Line:%d serial_number[0]=0x%02X,serial_number[1]=0x%02X,serial_number[2]=0x%02X,serial_number[3]=0x%02X,serial_number[4]=0x%02X,serial_number[5]=0x%02X,serial_number[6]=0x%02X,serial_number[7]=0x%02X,serial_number[8]=0x%02X\n", 
		                  __func__,__LINE__,
		                  serial_number[0],serial_number[1],serial_number[2],
						  serial_number[3],serial_number[4],serial_number[5],
						  serial_number[6],serial_number[7],serial_number[8]);

		if((status1!=0)&&(status2!=0)&&(status3!=0))
		{
			memcpy(CRC8,serial_number,9);
			memcpy(&CRC8[9],ATQA_SAK,4);
			CRC8_data=mycrc8(CRC8,13);
			FM11_WriteE2(FM441_CRC8_EEaddress,1,&CRC8_data);
		}
		FM11_WriteReg(RESET_SILENCE,0xCC);	//非接复位放开，退出静默
		FM11_WriteReg(FM441_RF_TXCTL_REG,0x77);	//非接切到接收状态，回到IDLE
		FM11_WriteReg(RESET_SILENCE,0x55);	//芯片复位
		mdelay(1);
	}

	else if(Tag_mode)
	{
		FM11_WriteReg(RESET_SILENCE,0x33);
		status1=FM11_WriteE2(FM441_SAK_Control_EEaddress,1,&SAK2_3[0]);
		FM11_WriteE2(FM441_Protocol_Control_EEaddress,4,Protocol_Tag);

		status2=FM11_ReadE2(serial_number,FM441_Serial_number_EEaddress ,9);
		status3=FM11_ReadE2(ATQA_SAK,FM441_ATQA_EEaddress,4);

		pr_info("[Tag_mode]->Function:%s Line:%d serial_number[0]=0x%02X,serial_number[1]=0x%02X,serial_number[2]=0x%02X,serial_number[3]=0x%02X,serial_number[4]=0x%02X,serial_number[5]=0x%02X,serial_number[6]=0x%02X,serial_number[7]=0x%02X,serial_number[8]=0x%02X\n", 
		                  __func__,__LINE__,
		                  serial_number[0],serial_number[1],serial_number[2],
						  serial_number[3],serial_number[4],serial_number[5],
						  serial_number[6],serial_number[7],serial_number[8]);

		if((status1!=0)&&(status2!=0)&&(status3!=0))
		{
			memcpy(CRC8,serial_number,9);
			memcpy(&CRC8[9],ATQA_SAK,4);
			CRC8_data=mycrc8(CRC8,13);
			FM11_WriteE2(FM441_CRC8_EEaddress,1,&CRC8_data);
		}
		FM11_WriteReg(RESET_SILENCE,0xCC);
		FM11_WriteReg(FM441_RF_TXCTL_REG,0x77);
		FM11_WriteReg(RESET_SILENCE,0x55);
		mdelay(1);
	}

	FM11_ReadE2(rbuf,FM441_Protocol_Control_EEaddress, 4);
	FM11_ReadE2(rbuf,FM441_SAK_Control_EEaddress, 1);
	NSS_OFF;//CSN管脚拉高

}

uint8_t mycrc8(uint8_t *data,uint8_t data_length) //修改SAK必须要计算CRC8
{
	int i=0;
	int j=0;
	int  Crc8 = 0xff;
	for(i=0 ; i<data_length ; i++)
	{
		Crc8 ^=data[i];
		for(j=0 ; j<8; j++)
		{
			if((Crc8 & 0x01) == 0x01) {
				Crc8 = (Crc8>>1) ^ 0xb8 ;
			} else {
				Crc8 >>= 1 ;
			}
			Crc8 &= 0xff ;
		}
	}
	return Crc8 & 0xff;
}

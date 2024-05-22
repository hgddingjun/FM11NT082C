
#ifndef _FM11_H_
#define _FM11_H_


#include "iic.h"

extern uint8_t irq_data_in;		//非接数据接收终端标识
extern uint8_t irq_rxdone;
extern uint8_t irq_txdone;
extern uint8_t FlagFirstFrame;
extern uint32_t FSDI;		//-4帧长度PCD

extern uint8_t rfLen;
extern uint8_t rfBuf[255];
extern uint32_t RFdataLen;					//非接触缓存区数据长度
extern uint8_t fm327_fifo[600];
extern uint8_t FM441_AUTH_0;//需要验证的起始页地址
extern uint8_t FM441_AUTH_Key[10];//10字节密码
extern uint8_t fm365SakConfig;
extern uint8_t ndef_file[888];


void FM11_RFDataTx(uint32_t ilen,uint8_t *ibuf);
uint32_t FM11_RFDataRx(uint8_t *rbuf);
void FM11_SetRatsCfg(uint8_t rats);
void FM11T4T(void);

#endif



#ifndef _IIC_H_
#define _IIC_H_

#include <linux/delay.h>
#include <linux/interrupt.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#include "fm11nt082c_i2c.h"

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

#define OFF 0
#define ON  1

extern st_fm11nt082c_info *p_nfc_info;

#define I2C_SLAVE_ADDRESS7	 0xAE	  //默认1010 111 如果采用硬件iic是0x57
#define sEE_PAGESIZE    16

#define SCL_H         gpio_direction_output(p_nfc_info->i2c_clk_gpio, 1)
#define SCL_L         gpio_direction_output(p_nfc_info->i2c_clk_gpio, 0)

#define SDA_H         gpio_direction_output(p_nfc_info->i2c_sda_gpio, 1)
#define SDA_L         gpio_direction_output(p_nfc_info->i2c_sda_gpio, 0)

#define SCL_read      gpio_get_value(p_nfc_info->i2c_clk_gpio)
#define SDA_read      gpio_get_value(p_nfc_info->i2c_sda_gpio)

#define NSS_OFF       gpio_direction_output(p_nfc_info->nfc_csn_gpio, 1)
#define NSS_ON        gpio_direction_output(p_nfc_info->nfc_csn_gpio, 0)  //低电平

#define IRQ_READ      gpio_get_value(p_nfc_info->nfc_irq_gpio)

/* FM327 reg for I2C */
#define FM327_FIFO                  0xFFF0
#define FIFO_FLUSH_REG              0xFFF1
#define	FIFO_WORDCNT_REG            0xFFF2
#define RF_STATUS_REG               0xFFF3
#define RF_TXEN_REG                 0xFFF4
#define RF_BAUD_REG                 0xFFF5
#define RF_RATS_REG                 0xFFF6
#define MAIN_IRQ_REG                0xFFF7
#define FIFO_IRQ_REG                0xFFF8
#define AUX_IRQ_REG                 0xFFF9
#define MAIN_IRQ_MASK_REG           0xFFFA
#define FIFO_IRQ_MASK_REG           0xFFFB
#define AUX_IRQ_MASK_REG            0xFFFC
#define NFC_CFG_REG                 0xFFFD
#define VOUT_CFG_REG                0xFFFE
#define EE_WR_CTRL_REG              0xFFFF
#define FM441_RF_TXCTL_REG          0xFFF4  //RF_TXEN_REG一样

#define  FM441_Serial_number_EEaddress            0x000
#define  FM441_CC_EEaddress                       0x00C
#define  FM441_NDEF_Header_EEaddress              0x010
#define  FM441_AUTH0_EEaddress                    0x3CF    //如果是iic改写auth0 地址不是手册上的地址，如果是非接改写的话还是E3 blcok
#define  FM441_AUTH_Key_EEaddress                 0x3A0
#define  FM441_T0_Control_EEaddress               0x3B1
#define  FM441_TB_Control_EEaddress               0x3B5
#define  FM441_CRC8_EEaddress                     0x3BB
#define  FM441_ATQA_EEaddress                     0x3BC
#define  FM441_SAK_Control_EEaddress              0x3BF
#define  FM441_Protocol_Control_EEaddress         0x390

#define MAIN_IRQ                                  0xFFF7
#define FIFO_IRQ                                  0xFFF8
#define AUX_IRQ                                   0xFFF9
#define MAIN_IRQ_MASK                             0xFFFA
#define FIFO_IRQ_MASK                             0xFFFB
#define AUX_IRQ_MASK                              0xFFFC
#define FIFO_FLUSH                                0xFFF1
#define	FIFO_WORDCNT                              0xFFF2

#define USER_CFG0                                 0xFFE0
#define USER_CFG1                                 0xFFE1
#define USER_CFG2                                 0xFFE2
#define RESET_SILENCE                             0xFFE6
#define STATUS                                    0xFFE7
#define VOUT_EN_CFG                               0xFFE9
#define VOUT_RES_CFG                              0xFFEA

#define MAIN_IRQ_RF_PWON                          0x80
#define MAIN_IRQ_ACTIVE                           0x40
#define MAIN_IRQ_RX_START                         0x20
#define MAIN_IRQ_RX_DONE                          0x10
#define MAIN_IRQ_TX_DONE                          0x08
#define MAIN_IRQ_ARBIT                            0x04
#define MAIN_IRQ_FIFO                             0x02
#define MAIN_IRQ_AUX                              0x01
#define FIFO_IRQ_WL                               0x08

#define	FM11_E2_BLOCK_SIZE                        16
#define	FM11_E2_USER_ADDR                         0x0010
#define	FM11_E2_MANUF_ADDR                        0x03FF

#define FALSE  0
#define TRUE   1


#define DW07_TAG_MODE   (1)

#if DW07_TAG_MODE
  #define Tag_mode        1   //定义走通道芯片，Tag_mode和Transive_mode 不能同时为1
  #define Transive_mode      0   //定义走tag芯片，Tag_mode和Transive_mode 不能同时为1
#else
  #define Tag_mode        0   //定义走通道芯片，Tag_mode和Transive_mode 不能同时为1
  #define Transive_mode      1   //定义走tag芯片，Tag_mode和Transive_mode 不能同时为1
#endif
#define Enable_AUTH       0   //定义非接读写EEPROM需不需要AUTH认证


void sEE_WaitEEStandbyState(void);
uint8_t FM11_ReadE2(uint8_t *pBuffer, uint16_t ReadAddr, uint16_t NumByteToRead);
uint8_t FM11_WriteE2(uint16_t adr,uint32_t len,uint8_t *wbuf);
uint8_t FM11_ReadReg(uint16_t addr);
uint8_t FM11_WriteReg(uint16_t addr, uint8_t data);
uint8_t FM11_ReadFIFO(uint8_t NumByteToRead,uint8_t *pbuf);
uint8_t FM11_WriteFIFO(uint8_t *pbuf, uint8_t NumByteToWrite);
uint8_t mycrc8(uint8_t *data,uint8_t data_length);

void FM11_Init(void);
int FM11_Run(void);

#endif

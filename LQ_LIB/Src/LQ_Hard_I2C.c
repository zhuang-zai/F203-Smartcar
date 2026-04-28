/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 【平    台】北京龙邱智能科技STC32位核心板
 【编    写】龙邱科技
 【E-mail  】chiusir@163.com
 【软件版本】V1.0 版权所有，单位使用请先联系授权
 【相关信息参考下列地址】
 【网    站】http://www.lqist.cn
 【淘宝店铺】http://longqiu.taobao.com
 --------------------------------------------------------------------------------
 【  IDE  】 keil C251 V5.60
 【Target 】 STC32G/STC8051U/AI8051U 32位模式
 【SYS CLK】 42 MHz使用内部晶振
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "LQ_Hard_I2C.h"



sbit SDA = P2^3;
sbit SCL = P2^4;

u8 IIC_Wait_time_out;
//等待响应
void Hard_IIC_Wait()
{
	while (!(I2CMSST & 0x40))
    {
        delay_ms(5);
        if(IIC_Wait_time_out++>10) break;  //50ms超时退出
    }
	I2CMSST &= ~0x40;
}

//起始信号
void Hard_IIC_Start()
{
	I2CMSCR = 0x01; //发送 START 命令
	Hard_IIC_Wait();
}

//发送数据
void Hard_IIC_SendData(unsigned char dat)
{
	I2CTXD = dat; //写数据到数据缓冲区
	I2CMSCR = 0x02; //发送 SEND 命令
	Hard_IIC_Wait();
}

//等待响应
void Hard_IIC_RecvACK()
{
	I2CMSCR = 0x03; //发送读 ACK 命令
	Hard_IIC_Wait();
}

//接收数据
unsigned char Hard_IIC_RecvData()
{
	I2CMSCR = 0x04; //发送 RECV 命令
	Hard_IIC_Wait();
	return I2CRXD;
}

//发送ACK
void Hard_IIC_SendACK()
{
	I2CMSST = 0x00; //设置 ACK 信号
	I2CMSCR = 0x05; //发送 ACK 命令
	Hard_IIC_Wait();
}

//发送NACK
void Hard_IIC_SendNAK()
{
	I2CMSST = 0x01; //设置 NAK 信号
	I2CMSCR = 0x05; //发送 ACK 命令
	Hard_IIC_Wait();
}

//IIC停止信号
void Hard_IIC_Stop()
{
	I2CMSCR = 0x06; //发送 STOP 命令
	Hard_IIC_Wait();
}

/************************************************************
  * @brief   读取陀螺仪的设备ID
  * @param   type:选择IIC引脚
                                （0：P2.4 P2.3）
                                （1：P1.5 P1.4）
                                （2：         ）
                                （3：P3.2 P3.3）
  * @see    void Hard_IIC_Init();
  *
  * @date
*************************************************************/
void Hard_IIC_Init(unsigned char type)
{
	if(type == 0)		//选择P2.4与P2.3为IIC引脚
    {
        P2_MODE_IO_PU(GPIO_Pin_3|GPIO_Pin_4);   //将I2C管脚设为准双向口
        I2C_S1=0;I2C_S0=0;      //P_SW2 = 0x80; 全局已经打开 扩展寄存器访问使能EAXSFR();
	}
    else if(type == 1)		//选择P1.5与P1.4为IIC引脚
    {
        P1_MODE_IO_PU(GPIO_Pin_4|GPIO_Pin_5);   //将I2C管脚设为准双向口
        I2C_S1=0;I2C_S0=1;
	}
    else if(type == 2)
    {
        //8051U仅三组，I2C_S[1:0]  可选值0，1，3 无2

//        I2C_S1=1;I2C_S0=0;
	}
    else if(type == 3)		//选择P3.2与P3.3为IIC引脚
    {
        P3_MODE_IO_PU(GPIO_Pin_2|GPIO_Pin_3);   //将I2C管脚设为准双向口
        I2C_S1=1;I2C_S0=1;
	}
	I2CCFG = 0xD4;  //使能 I2C 主机模式
	I2CMSST = 0x00; //清空IC2状态

}



















/*---------------------------------------------------------------------*/
/* ------------------- Web: www.STCAI.com -----------------------------*/
/*---------------------------------------------------------------------*/

#include "AI8051U_UART.h"
#include "include.h"
// 新增：包含调试模块头文件，用于 UART 数据解析
#include "debug.h"

bit B_ULinRX1_Flag;
bit B_ULinRX2_Flag;

//========================================================================
// : UART1_ISR_Handler
// : UART1жϺ.
// : none.
// : none.
// 汾: V1.0, 2020-09-23
//========================================================================
#ifdef UART1
void UART1_ISR_Handler (void) interrupt UART1_VECTOR
{
	u8 Status;

	if(RI)
    {
        char c; 
        RI = 0;
        
        c = SBUF; 

        //-------- USART LIN (ȫ) ---------------
        Status = USARTCR5;
        if(Status & 0x02)     //if LIN header is detected
        {
            B_ULinRX1_Flag = 1;
        }

        if(Status & 0xc0)     //if LIN break is detected / LIN header error is detected
        {
            COM1.RX_Cnt = 0;
        }
        USARTCR5 &= ~0xcb;    //Clear flag
        //-----------------------------------------------------

        //-------- лж ( SBUF Ϊ c) ---------
        if(COM1.RX_Cnt >= COM_RX1_Lenth) COM1.RX_Cnt = 0;
        RX1_Buffer[COM1.RX_Cnt++] = c; 
        COM1.RX_TimeOut = TimeOutSet1;
        //-----------------------------------------------------

        //++++++++  调用调试模块处理接收字节：VOFA+ JustFloat调参帧解析 + 'R'命令检测  ++++++++
        Debug_ProcessRxByte((uint8)c);
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    }

	if(TI)
	{
		TI = 0;

        #if(UART_QUEUE_MODE == 1)   //жʹöģʽ
		if(COM1.TX_send != COM1.TX_write)
		{
		 	SBUF = TX1_Buffer[COM1.TX_send];
			if(++COM1.TX_send >= COM_TX1_Lenth)		COM1.TX_send = 0;
		}
		else	COM1.B_TX_busy = 0;
        #else
        COM1.B_TX_busy = 0;     //ʹֱæ־
        #endif
	}
}
#endif

//========================================================================
// : UART2_ISR_Handler
// : UART2жϺ.
// : none.
// : none.
// 汾: V1.0, 2020-09-23
//========================================================================
#ifdef UART2
void UART2_ISR_Handler (void) interrupt UART2_VECTOR
{
	u8 Status;

	if(S2RI)
	{
		CLR_RI2();

		//--------USART LIN---------------
		Status = USART2CR5;
		if(Status & 0x02)     //if LIN header is detected
		{
			B_ULinRX2_Flag = 1;
		}

		if(Status & 0xc0)     //if LIN break is detected / LIN header error is detected
		{
			COM2.RX_Cnt = 0;
		}
		USART2CR5 &= ~0xcb;   //Clear flag
		//--------------------------------

        if(COM2.RX_Cnt >= COM_RX2_Lenth)	COM2.RX_Cnt = 0;
        RX2_Buffer[COM2.RX_Cnt++] = S2BUF;
        COM2.RX_TimeOut = TimeOutSet2;
	}

	if(S2TI)
	{
		CLR_TI2();

        #if(UART_QUEUE_MODE == 1)   //жʹöģʽ
		if(COM2.TX_send != COM2.TX_write)
		{
		 	S2BUF = TX2_Buffer[COM2.TX_send];
			if(++COM2.TX_send >= COM_TX2_Lenth)		COM2.TX_send = 0;
		}
		else	COM2.B_TX_busy = 0;
        #else
        COM2.B_TX_busy = 0;     //ʹֱæ־
        #endif
	}
}
#endif

//========================================================================
// : UART3_ISR_Handler
// : UART3жϺ.
// : none.
// : none.
// 汾: V1.0, 2020-09-23
//========================================================================
#ifdef UART3

#if (1 == ENABLE_SBUS)

#include "LQ_SBUS.h"
unsigned char sbuff;  //SBUS
unsigned char num3;
extern unsigned char SReadBuff[64];
#endif
void UART3_ISR_Handler (void) interrupt UART3_VECTOR
{
	if(S3RI)
	{
		CLR_RI3();

        if(COM3.RX_Cnt >= COM_RX3_Lenth)	COM3.RX_Cnt = 0;
#if (1 == ENABLE_SBUS)
        /* SBUS START  */
        sbuff = (u8)S3BUF;
        if(sbus_Rec_Fini_Flag == 1)       
        {
            if (num3 == 0 && sbuff == 0x0F)   
            {
                SReadBuff[num3] = sbuff;
                num3++;
            }
            else if(num3 != 0)
            {
                SReadBuff[num3] = sbuff;
                num3++;
                if (num3 > 24)              
                {
                    sbus_Rec_Fini_Flag = 0;   

                    if (SReadBuff[0] == 0x0F && SReadBuff[24] == 0x00 && num3 == 25)    
                    {
                        UpDate_SBUS();      
                    }
                    num3 = 0;               
                }

            }
        }
        /* SBUS END*/
	#endif
        COM3.RX_TimeOut = TimeOutSet3;
	}

	if(S3TI)
	{
		CLR_TI3();

        #if(UART_QUEUE_MODE == 1)   //жʹöģʽ
		if(COM3.TX_send != COM3.TX_write)
		{
		 	S3BUF = TX3_Buffer[COM3.TX_send];
			if(++COM3.TX_send >= COM_TX3_Lenth)		COM3.TX_send = 0;
		}
		else	COM3.B_TX_busy = 0;
        #else
        COM3.B_TX_busy = 0;     //ʹֱæ־
        #endif
	}
}
#endif

//========================================================================
// : UART4_ISR_Handler
// : UART4жϺ.
// : none.
// : none.
// 汾: V1.0, 2020-09-23
//========================================================================
#ifdef UART4

#if (1 == ENABLE_GPS)
#include "LQ_BDGPS.h"       
unsigned char GPS_buf;    
unsigned char GPS_ReadBuff[128];
#endif
void UART4_ISR_Handler (void) interrupt UART4_VECTOR
{
	if(S4RI)
	{
        static u8 num1;
		CLR_RI4();

        if(COM4.RX_Cnt >= COM_RX4_Lenth)	COM4.RX_Cnt = 0;
#if (1 == ENABLE_GPS)
        /* GPS_BDնstart */

        GPS_buf = (u8)S4BUF;
        if(GPS_buf == '\n' || GPS_buf == '\r')   
        {
            strcpy((char*)Save_Data.GPS_Buffer,(const char*)GPS_ReadBuff);
            Save_Data.isGetData=1;
            num1 = 0;
        }
        else
        {
            GPS_ReadBuff[num1++] = GPS_buf;
        }
        /* GPS_BDնend */
#endif
        COM4.RX_TimeOut = TimeOutSet4;
	}

	if(S4TI)
	{
		CLR_TI4();

        #if(UART_QUEUE_MODE == 1)   //жʹöģʽ
		if(COM4.TX_send != COM4.TX_write)
		{
		 	S4BUF = TX4_Buffer[COM4.TX_send];
			if(++COM4.TX_send >= COM_TX4_Lenth)		COM4.TX_send = 0;
		}
		else	COM4.B_TX_busy = 0;
        #else
        COM4.B_TX_busy = 0;     //ʹֱæ־
        #endif
	}
}
#endif

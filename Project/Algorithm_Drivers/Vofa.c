#include "VOFA.h"
#include "RoboControl.h"

extern float Shoot_Fric_First_Left_Speed   ;
extern float Shoot_Fric_First_Right_Speed  ;
extern float Shoot_Fric_First_Middle_Speed ;
extern float Shoot_Fric_Second_Left_Speed  ;
extern float Shoot_Fric_Second_Right_Speed ;
extern float Shoot_Fric_Second_Middle_Speed;

float VOFA_fdata[VOFA_TXDATA_SIZE] = {0};
uint8_t VOFA_tail[4] = {0x00, 0x00, 0x80, 0x7F};
uint8_t VOFA_message[VOFA_TXDATA_SIZE * 4 + 4] = {0};

float VOFA_RxData[VOFA_RXDATA_SIZE] = {0};


/**
 * @brief VOFA串口接收中断回调函数
 *
 * @param 串口收到的数组
 */
void VOFA_Rx_CallBack(uint8_t *VOFA_RxDataBuff)
{
	// float VOFA_RX_Float =   (VOFA_RxDataBuff[3]-0x30)*10000+
    //                         (VOFA_RxDataBuff[4]-0x30)*1000+
    //                         (VOFA_RxDataBuff[5]-0x30)*100+
    //                         (VOFA_RxDataBuff[6]-0x30)*10+
    //                         (VOFA_RxDataBuff[7]-0x30)*1+
    //                         (VOFA_RxDataBuff[9]-0x30)*0.1f+
    //                         (VOFA_RxDataBuff[10]-0x30)*0.01f+
    //                         (VOFA_RxDataBuff[11]-0x30)*0.001f;
    // if(VOFA_RxDataBuff[2] == '-') VOFA_RX_Float = -VOFA_RX_Float;
	
    
	// if		(VOFA_RxDataBuff[0]=='1' && VOFA_RxDataBuff[1]=='P') 
    // {

    // }
	// else if	(VOFA_RxDataBuff[0]=='1' && VOFA_RxDataBuff[1]=='I')
    // {

    // }
    // else if	(VOFA_RxDataBuff[0]=='1' && VOFA_RxDataBuff[1]=='D')
    // {

    // }
    // else if	(VOFA_RxDataBuff[0]=='1' && VOFA_RxDataBuff[1]=='E') 
    // {

    // }
    if(!strncmp((char *)VOFA_RxDataBuff, "VOFA", 4))
    {
        memcpy(VOFA_RxData, VOFA_RxDataBuff + 4, VOFA_RXDATA_SIZE * 4);
    }
}

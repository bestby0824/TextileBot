/**
* ��Ȩ����(C)
*
* ********
*
* @file
* @brief
* @details
* @author HWW
* @version 1.0.0
* @date xxxx-xx-xx
* @par Description:
* @par Change History:
*
* <����> | <�汾> | <����> | <����>
*
* xxxx/xx/xx | 1.0.0 | HWW | �����ļ�
*
*/
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
#ifndef _DataProcess__H_
#define _DataProcess__H_
//-------------------- include files ----------------------------------------
#include "Eeprom.h"
#include "MODBusCRC.h"
#include "StFlash.h"

//-------------------- public definitions -----------------------------------
#define OS_YEAR     ((((__DATE__ [7] - '0') * 10 + (__DATE__ [8] - '0')) * 10 + (__DATE__ [9] - '0')) * 10 + (__DATE__ [10] - '0'))
#define OS_MONTH    (__DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? 1 : 6) \
                                 : __DATE__ [2] == 'b' ? 2 \
                                 : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 3 : 4) \
                                 : __DATE__ [2] == 'y' ? 5 \
                                 : __DATE__ [2] == 'l' ? 7 \
                                 : __DATE__ [2] == 'g' ? 8 \
                                 : __DATE__ [2] == 'p' ? 9 \
                                 : __DATE__ [2] == 't' ? 10 \
                                 : __DATE__ [2] == 'v' ? 11 : 12)
#define OS_DAY      ((__DATE__ [4] == ' ' ? 0 : __DATE__ [4] - '0') * 10  + (__DATE__ [5] - '0'))
#define OS_HOUR     ((__TIME__ [0] - '0') * 10 + (__TIME__ [1] - '0'))
#define OS_MINUTE   ((__TIME__ [3] - '0') * 10 + (__TIME__ [4] - '0'))
#define OS_SECOND   ((__TIME__ [6] - '0') * 10 + (__TIME__ [7] - '0'))

#define ADDRESS_EepromParamSaving_FLAG          ADDR_FLASH_SECTOR_2    //Eeprom���������־λ
#define EepromParamSaving_ControlResult         0x1111
#define EepromParamSaving_ValveResult           0x2222
#define EepromParamSaving_SteerResult           0x3333

#define SoftVer_A                               1  //��ΧΪ0~255��AΪ��汾��һ��Ϊ�ӿڵı仯����汾�仯����ԭ�п�ܲ�����
#define SoftVer_B                               0  //��ΧΪ0~255��BΪ���ܰ汾�����ӹ�����仯B��
#define SoftVer_C                               6  //��ΧΪ0~255��CΪbug�޸ģ������C��

#define HardVerCrc(x)                           x*12/4096

//-------------------- public data ------------------------------------------
typedef struct {
//-------------------- 310-313�Ĵ������汾��ز��� --------------------------
    uint16_t Reg_Tab_Ver;                   //0:�Ĵ�����汾
    uint16_t Reg_Tab_Len;                   //1:�Ĵ�������(��������)
    uint16_t Reg_Dev_Ver;                   //2:�豸���ͱ���
    uint16_t Reg_HW_Ver;                    //3:Ӳ���汾����ʽ��V1.0
    
    uint32_t Reg_SN;                        //4-5:���к�
    uint32_t Reg_SW_Ver;                    //6-7:��ǰ����汾
    
    uint32_t Reg_SW_NewApp;                 //8-9:NewApp����汾
    uint32_t Reg_SW_Factory;                //10-11��������汾
    
    uint32_t Reg_SW_Boot;                   //12-13:boot����汾
    uint32_t Reg_Password;                  //14-15:д��������(�̻���������)

//-------------------- 320-322�Ĵ��������������ز����������޸ģ� --------------------------
    uint16_t Reg_PolePairs;                 //16:�ż�����
    uint16_t Reg_RatedCurrent;              //17:�����,��λ:0.01A
    uint16_t Reg_NoLoadSpeed;               //18:����ת��,��λ:1rpm
    uint16_t Reg_RatedVoltage;              //19:���ѹ,��λ0.01V
    
    uint16_t Reg_MotorAxleOffset;           //20:������������OFFSET,��һ���϶�����궨
    int16_t Reg_ReverseFlag;                //21:����ת��־,��һ���϶�����궨,1:��Ƕ���������ת������ͬ,-1:��Ƕ���������ת�����෴,0:δ�궨
    uint16_t Reg_ReductionRatio;            //22:������ٱ�
    uint16_t u16MotorRes3;                  //23:Ԥ��
    
    int32_t Reg_SampleMaxCur;               //24-25:AD����������,����Ӳ����·������_IQ(1)��ʾ1A---55A
    uint16_t u16MotorRes4;                  //26:Ԥ��
    uint16_t u16MotorRes5;                  //27:Ԥ��
    
//-------------------- 330-333�Ĵ��������������ز��������޸ģ� --------------------------
    uint32_t Reg_PI_POS_KP;                 //28-29:λ�û�KP
    uint32_t Reg_PI_POS_KI;                 //30-31:λ�û�KI
    
    uint32_t Reg_PI_SPD_KP;                 //32-33:�ٶȻ�KP
    uint32_t Reg_PI_SPD_KI;                 //34-35:�ٶȻ�KI
    
    uint16_t Reg_PI_ID_KP;                  //36:������(ID)KP
    uint16_t Reg_PI_ID_KI;                  //37:������(ID)KI
    uint16_t Reg_PI_IQ_KP;                  //38:������(IQ)KP
    uint16_t Reg_PI_IQ_KI;                  //39:������(IQ)KI
    
    uint16_t Reg_AngleMax;                  //40:λ�û��������ֵ
    uint16_t Reg_SpeedMax;                  //41:�ٶȻ��������ֵ(λ�û�������ֵ),��λ:RPM
    uint16_t Reg_CurrentMax;                //42:�������������ֵ(�ٶȻ�������ֵ),��λ:_IQ(1):1A
    uint16_t Reg_IdIqMax;                   //43:Id/Iq�������ֵ(������������ֵ)
    
//-------------------- 340�Ĵ������������������޸ģ� --------------------------
    uint32_t Reg_FaultCheckEnable;          //44-45:���ϼ��ʹ�ܱ�־,0:��ʹ��,1:ʹ��
    uint16_t Reg_CtrlDeadZone;              //46:ָ������
    uint16_t Reg_PosDeadZone;               //47:��λ����
    
    uint16_t Reg_PosMax;                    //48:λ������޷�
    uint16_t Reg_PosMin;                    //49:λ����С�޷�
    uint16_t Reg_TorqueMax;                 //50:ת������޷�
    uint16_t Reg_TorqueMin;                 //51:ת����С�޷�
    
    uint16_t Reg_CaliTimeOut;               //52:�궨��ʱʱ��
    uint16_t Reg_RunTimeOut;                //53:�˶���ʱʱ��
    uint16_t Reg_RC_Spdmax;                 //54:ң��ģʽ����ٶ�
    uint16_t Reg_PromEnd;                  //55:��������
    
//-------------------- 350-253�Ĵ�����������ز��� -------------------------
    uint16_t Reg_Spd_Slow;                  //�������ã�65535����8.37m/S������������0��ȣ���ǿ����ײ���գ��ǹ����������������1m/S
    uint16_t Reg_Spd_Normal;                //�������ã�65535����8.37m/S
    uint16_t Reg_Spd_Fast;                  //�������ã�65535����8.37m/S
    uint16_t Reg_Pos_Max;                   //�յ�λ�ã�1����0.1m
    
    uint16_t Reg_DogTimeOut;                 //��λ��������ʱ
    uint16_t Reg_IniSta;                     //��ʼ��״̬��
    uint16_t Reg_SocLim;                     //�����ֵ
    uint16_t ChargDistance;                  //��λ0.01m
    
    uint16_t CardDistance;                   //��λ0.01m
    uint16_t EndDistance;                    //��λ0.01m
    uint32_t Reg_SW_Ver_usr;                 //Ԥ��10
    
    int32_t Reg_Tag0;                       //��־��λ��
    int32_t Reg_Tag1;                       //��־��λ��
    
    int32_t Reg_Tag2;                       //��־��λ��
    int32_t Reg_Tag3;                       //��־��λ��
    
    int32_t Reg_Tag4;                       //��־��λ��
    int32_t Reg_Tag5;                       //��־��λ��
    
    int32_t Reg_Tag6;                       //��־��λ��
    int32_t Reg_Tag7;                       //��־��λ��
    
    int32_t Reg_Tag8;                       //��־��λ��
    int32_t Reg_Tag9;                       //��־��λ��
    
    int32_t Reg_Tag10;                       //��־��λ��
    int32_t Reg_Tag11;                       //��־��λ��
    
    int32_t Reg_Tag12;                       //��־��λ��
    int32_t Reg_Tag13;                       //��־��λ��
    
    int32_t Reg_Tag14;                       //��־��λ��
    int32_t Reg_Tag15;                       //��־��λ��
    
    int32_t Reg_Tag16;                       //��־��λ��
    int32_t Reg_Tag17;                       //��־��λ��
    
    int32_t Reg_Tag18;                       //��־��λ��
    int32_t Reg_Tag19;                       //��־��λ��
    
    int32_t Reg_Tag20;                       //��־��λ��
    int32_t Reg_Tag21;                       //��־��λ��
    
    int32_t Reg_Tag22;                       //��־��λ��
    int32_t Reg_Tag23;                       //��־��λ��
    
    int32_t Reg_Tag24;                       //��־��λ��
    int32_t Reg_Tag25;                       //��־��λ��
    
    int32_t Reg_Tag26;                       //��־��λ��
    int32_t Reg_Tag27;                       //��־��λ��
    
    int32_t Reg_Tag28;                       //��־��λ��
    int32_t Reg_Tag29;                       //��־��λ��
    
    int32_t Reg_Tag30;                       //��־��λ��
    int32_t Reg_Tag31;                       //74-75:��־��λ��
    
    uint16_t Reg_AccSpd;                     //76�����ټ��ٶȣ�65535����1m/S^2
    uint16_t Reg_DecSpd;                     //77�����ټ��ٶȣ�65535����1m/S^2
    uint16_t Reg_OverLoadLim;                //78������ʱ�䣬��λmS��2�ų�30000��������10000
//    uint16_t Reg_Res15;                     //Ԥ��15z
    
    uint16_t Reg_CRC ;                      //79:CRCУ��
} ParamStruct_Steer;
extern ParamStruct_Steer ParamHandle_Steer;

typedef enum{
    Dev_Textile_Control = 21,
    Dev_Textile_Rotate,
    Dev_Textile_Telescopic,
    Dev_Textile_Hug,
    Dev_Textile_Gland,
}Dev_Ver_List;

#define Tab_Ver         1
#define Tab_Len         sizeof( ParamStruct_Steer )
#define Dev_Ver         Dev_Textile_Gland

#define ParamDefault    { \
    Tab_Ver,/* Reg_Tab_Ver*/\
    Tab_Len,/* Reg_Tab_Len*/\
    Dev_Ver,/* Reg_Dev_Ver*/\
    0200,/* Reg_HW_Ver*/\
    0xFFFFFFFF,/* Reg_SN*/\
    001000001,/* Reg_SW_Ver*/\
    0xFFFFFFFF,/*Reg_SW_NewApp*/\
    001000001,/*Reg_SW_Factory*/\
    001000001,/*Reg_SW_Boot*/\
    202504,/* Reg_Password*/\
    \
    4,/* Reg_PolePairs*/\
    200,/* Reg_RatedCurrent*/\
    3000,/* Reg_NoLoadSpeed*/\
    2400,/* Reg_RatedVoltage*/\
    0,/* Reg_MotorAxleOffset*/\
    0,/* Reg_ReverseFlag*/\
    0,/* Reg_ReductionRatio*/\
    0,/* u16MotorRes3*/\
    _IQ(27),/* Reg_SampleMaxCur*/\
    0,/* u16MotorRes4*/\
    0,/* u16MotorRes5*/\
    \
    _IQ ( 24 ),/* Reg_PI_POS_KP*/\
    _IQ ( 0 ),/* Reg_PI_POS_KI*/\
    _IQ ( 20 ),/* Reg_PI_SPD_KP*/\
    _IQ ( 0.002 ),/* Reg_PI_SPD_KI*/\
    _IQ ( 0.085 ),/* Reg_PI_ID_KP*/\
    _IQ ( 0.075 ),/* Reg_PI_ID_KI*/\
    _IQ ( 0.085 ),/* Reg_PI_IQ_KP*/\
    _IQ ( 0.075 ),/* Reg_PI_IQ_KI*/\
    _IQ ( 0.1667 ),/* Reg_AngleMax*/\
    3000,/* Reg_SpeedMax*/\
    300,/* Reg_CurrentMax*/\
    _IQ ( 0.5 ),/* Reg_IdIqMax*/\
    \
    0xFFFFFFFF,/* Reg_FaultCheckEnable*/\
    1,/* Reg_CtrlDeadZone*/\
    1,/* Reg_PosDeadZone*/\
    \
    320,/* Reg_PosMax*/\
    8,/* Reg_PosMin*/\
    50,/* Reg_TorqueMax*/\
    30,/* Reg_TorqueMin*/\
    20000,/* Reg_CaliTimeOut*/\
    20000,/* Reg_RunTimeOut*/\
    _IQ(0.25),/* Reg_RC_Spdmax*/\
    0,/* ��������*/\
    \
    3900,/* Reg_Spd_Slow*/\
    15600,/* Reg_Spd_Normal*/\
    62600,/* Reg_Spd_Fast*/\
    1000,/* Reg_Pos_Max*/\
    6000,/* Reg_DogTimeOut*/\
    0,/* Reg_IniSta*/\
    50,/* Reg_SocLim*/\
    100,/* ChargDistance*/\
    4800,/* CardDistance*/\
    100,/* EndDistance*/\
    0,/* SWVer*/\
    _IQ(0),/*Reg_Tag0*/\
    _IQ(48),/*Reg_Tag1*/\
    _IQ(96),/*Reg_Tag2*/\
    _IQ(144),/*Reg_Tag3*/\
    _IQ(192),/*Reg_Tag4*/\
    _IQ(240),/*Reg_Tag5*/\
    _IQ(288),/*Reg_Tag6*/\
    _IQ(336),/*Reg_Tag7*/\
    _IQ(384),/*Reg_Tag8*/\
    _IQ(432),/*Reg_Tag9*/\
    _IQ(480),/*Reg_Tag10*/\
    _IQ(528),/*Reg_Tag11*/\
    _IQ(576),/*Reg_Tag12*/\
    _IQ(624),/*Reg_Tag13*/\
    _IQ(672),/*Reg_Tag14*/\
    _IQ(720),/*Reg_Tag15*/\
    _IQ(768),/*Reg_Tag16*/\
    _IQ(816),/*Reg_Tag17*/\
    _IQ(864),/*Reg_Tag18*/\
    _IQ(912),/*Reg_Tag19*/\
    _IQ(960),/*Reg_Tag20*/\
    _IQ(1008),/*Reg_Tag21*/\
    _IQ(1056),/*Reg_Tag22*/\
    _IQ(1104),/*Reg_Tag23*/\
    _IQ(1152),/*Reg_Tag24*/\
    _IQ(1200),/*Reg_Tag25*/\
    _IQ(1248),/*Reg_Tag26*/\
    _IQ(1296),/*Reg_Tag27*/\
    _IQ(1344),/*Reg_Tag28*/\
    _IQ(1392),/*Reg_Tag29*/\
    _IQ(1440),/*Reg_Tag30*/\
    _IQ(1488),/*Reg_Tag31*/\
    _IQ(0.01),/* Reg_AccSpd*/\
    _IQ(0.01),/* Reg_DecSpd*/\
    5000,/* Reg_OverLoadLim*/\
    0,/* Reg_CRC*/\
};

//-------------------- public functions -------------------------------------
void DataBaseInit(void);
int8_t Param2Eeprom ( ParamStruct_Steer Param );
int DataUpParameter ( void );

#endif // _DataProcess__H_

//-----------------------End of file------------------------------------------
/** @}*/

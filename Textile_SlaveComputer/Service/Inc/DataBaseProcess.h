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
#include "Dido.h"
//-------------------- public definitions -----------------------------------
//typedef struct {
////-------------------- 310-313�Ĵ������汾��ز��� --------------------------
//    uint16_t Reg_Tab_Ver;                   //�Ĵ�����汾
//    uint16_t Reg_Tab_Len;                   //�Ĵ�������(��������)
//    uint16_t Reg_Dev_Ver;                   //�豸���ͱ���
//    uint16_t Reg_HW_Ver;                    //Ӳ���汾����ʽ��V1.0

//    uint32_t Reg_SN;                        //���к�
//    uint32_t Reg_SW_Ver;                    //��ǰ����汾

//    uint32_t Reg_SW_NewApp;                 //NewApp����汾
//    uint32_t Reg_SW_Factory;                //��������汾

//    uint32_t Reg_SW_Boot;                   //boot����汾
//    uint32_t Reg_Password;                  //д��������(�̻���������)

////-------------------- 320-322�Ĵ��������������ز����������޸ģ� --------------------------
//    uint16_t Reg_PolePairs;                 //�ż�����
//    uint16_t Reg_RatedCurrent;              //�����,��λ:0.01A
//    uint16_t Reg_NoLoadSpeed;               //����ת��,��λ:1rpm
//    uint16_t Reg_RatedVoltage;              //���ѹ,��λ0.01V

//    uint16_t Reg_MotorAxleOffset;           //������������OFFSET,��һ���϶�����궨
//    int16_t Reg_ReverseFlag;                //����ת��־,��һ���϶�����궨,1:��Ƕ���������ת������ͬ,-1:��Ƕ���������ת�����෴,0:δ�궨
//    uint16_t u16MotorRes2;                  //Ԥ��
//    uint16_t u16MotorRes3;                  //Ԥ��

//    int32_t Reg_SampleMaxCur;               //AD����������,����Ӳ����·������_IQ(1)��ʾ1A
//    uint16_t u16MotorRes4;                  //Ԥ��
//    uint16_t u16MotorRes5;                  //Ԥ��

////-------------------- 330-333�Ĵ��������������ز��������޸ģ� --------------------------
//    uint32_t Reg_PI_POS_KP;                 //λ�û�KP
//    uint32_t Reg_PI_POS_KI;                 //λ�û�KI

//    uint32_t Reg_PI_SPD_KP;                 //�ٶȻ�KP
//    uint32_t Reg_PI_SPD_KI;                 //�ٶȻ�KI

//    uint16_t Reg_PI_ID_KP;                  //������(ID)KP
//    uint16_t Reg_PI_ID_KI;                  //������(ID)KI
//    uint16_t Reg_PI_IQ_KP;                  //������(IQ)KP
//    uint16_t Reg_PI_IQ_KI;                  //������(IQ)KI

//    uint16_t Reg_AngleMax;                  //λ�û��������ֵ
//    uint16_t Reg_SpeedMax;                  //�ٶȻ��������ֵ(λ�û�������ֵ),��λ:RPM
//    uint16_t Reg_CurrentMax;                //�������������ֵ(�ٶȻ�������ֵ),��λ:_IQ(1):1A
//    uint16_t Reg_IdIqMax;                   //Id/Iq�������ֵ(������������ֵ)

////-------------------- 340�Ĵ������������������޸ģ� --------------------------
//    uint32_t Reg_FaultCheckEnable;          //���ϼ��ʹ�ܱ�־,0:��ʹ��,1:ʹ��
//    uint16_t u16MotorRes6;                  //Ԥ��
//    uint16_t u16MotorRes7;                  //Ԥ��

////-------------------- 350-253�Ĵ�����������ز��� -------------------------
//    uint16_t Reg_Res0;                      //Ԥ��0
//    uint16_t Reg_Res1;                      //Ԥ��1
//    uint16_t Reg_Res2;                      //Ԥ��2
//    uint16_t Reg_Res3;                      //Ԥ��3

//    uint16_t Reg_Res4;                      //Ԥ��4
//    uint16_t Reg_Res5;                      //Ԥ��5
//    uint16_t Reg_Res6;                      //Ԥ��6
//    uint16_t Reg_Res7;                      //Ԥ��7

//    uint16_t Reg_Res8;                      //Ԥ��8
//    uint16_t Reg_Res9;                      //Ԥ��9
//    uint16_t Reg_Res10;                     //Ԥ��10
//    uint16_t Reg_Res11;                     //Ԥ��11

//    uint16_t Reg_Res12;                     //Ԥ��12
//    uint16_t Reg_Res13;                     //Ԥ��13
//    uint16_t Reg_Res14;                     //Ԥ��14
////    uint16_t Reg_Res15;                     //Ԥ��15

//    uint16_t Reg_CRC ;                      //CRCУ��
//} ParamStruct_Steer;
//extern ParamStruct_Steer ParamHandle_Steer;
//typedef struct {
//    uint16_t Reg_Tab_Ver ;                  //�Ĵ�����汾
//    uint16_t Reg_Tab_Len ;                  //�Ĵ�������(��������)
//    uint16_t Reg_Dev_Ver ;                  //�豸���ͱ���
//    uint16_t Reg_HW_Ver ;                   //Ӳ���汾
//    uint32_t Reg_SN ;                       //���к�
//    uint32_t Reg_SW_Ver ;                   //
//    uint32_t Reg_SW_NewApp;                 //NewApp����汾
//    uint32_t Reg_SW_Factory;                //��������汾
//    uint32_t Reg_SW_Boot;                   //boot����汾
//    uint32_t Reg_Password ;                 //
//    uint16_t Reg_Mode ;                     //����ģʽ��0:����    3:������
//    uint16_t Reg_CRC ;                      //
//} ParamStruct_Hub485;

//typedef struct {
////-------------------- 210-213�Ĵ������汾��ز��� --------------------------
//    uint16_t Reg_Tab_Ver;                   //�Ĵ�����汾
//    uint16_t Reg_Tab_Len;                   //�Ĵ�������(��������)
//    uint16_t Reg_Dev_Ver;                   //�豸���ͱ���
//    uint16_t Reg_HW_Ver;                    //Ӳ���汾����ʽ��V1.0

//    uint32_t Reg_SN;                        //���к�
//    uint32_t Reg_SW_Ver;                    //��ǰ����汾

//    uint32_t Reg_SW_NewApp;                 //NewApp����汾
//    uint32_t Reg_SW_Factory;                //��������汾

//    uint32_t Reg_SW_Boot;                   //boot����汾
//    uint32_t Reg_Password;                  //д��������(�̻���������)

////-------------------- 220-225�Ĵ�����������ز��� --------------------------
//    uint32_t Reg_Lift_SoftMax;              //�����߶�������ֵ��65535����6��
//    uint16_t Reg_Lift_SoftMin;              //�����߶������Сֵ
//    uint16_t Reg_LiftPosOffSet;             //����offset�����λ��0��Ӧ����������

//    uint32_t Reg_Lift_MechMax;              //����λ�û�е���ֵ
//    uint16_t Reg_Lift_MechMin;              //����λ�û�е��Сֵ
//    uint16_t u16LiftRes0;                   //����Ԥ��0

//    uint16_t Reg_Lift_PosAllowErr;          //����λ������
//    uint16_t Reg_Lift_Pressure;             //������ѹ�����ֵ
//    uint16_t u16LiftRes1;                   //����Ԥ��1
//    uint16_t u16LiftRes2;                   //����Ԥ��2

//    uint32_t Reg_Lift_PosKp;                //����λ�û�PID����ϵ��
//    uint16_t Reg_Lift_PosKi;                //����λ�û�PID����ϵ��
//    uint16_t Reg_Lift_PosKd;                //����λ�û�PID΢��ϵ��

//    uint16_t Reg_Lift_PosUmax;              //����λ�û�PID���ֵ�޷�
//    uint16_t Reg_Lift_PosUmin;              //����λ�û�PID��Сֵ�޷�
//    uint16_t Reg_Lift_SpdKp;                //�����ٶȻ�PID����ϵ��
//    uint16_t Reg_Lift_SpdKi;                //�����ٶȻ�PID����ϵ��

//    uint16_t Reg_Lift_SpdKd;                //�����ٶȻ�PID΢��ϵ��
//    uint16_t Reg_Lift_SpdUmax;              //�����ٶȻ�PID���ֵ�޷�
//    uint16_t Reg_Lift_SpdUmin;              //�����ٶȻ�PID��Сֵ�޷�
//    uint16_t u16LiftRes3;                   //����Ԥ��3

////-------------------- 230-233�Ĵ�����������ز��� --------------------------
//    uint16_t Reg_Claw_SoftMax;              //���ϽǶ�������ֵ��32767����360��
//    uint16_t Reg_Claw_SoftMin;              //���ϽǶ������Сֵ
//    uint16_t Reg_Claw_MechMax;              //���ϽǶȻ�е���ֵ
//    uint16_t Reg_Claw_MechMin;              //���ϽǶȻ�е��Сֵ

//    uint16_t Reg_Claw_PosAllowErr;          //����λ������
//    uint16_t Reg_Claw_PressureErr;          //������ѹ�����ֵ
//    uint16_t u16ClawRes0;                   //����Ԥ��0
//    uint16_t u16ClawRes1;                   //����Ԥ��1

//    uint32_t Reg_Claw_Kp;                    //����PID����ϵ��
//    uint16_t Reg_Claw_Ki;                   //����PID����ϵ��
//    uint16_t Reg_Claw_Kd;                   //����PID΢��ϵ��

//    uint16_t Reg_Claw_Umax;                 //����λ�û�PID���ֵ�޷�
//    uint16_t Reg_Claw_Umin;                 //����λ�û�PID��Сֵ�޷�
//    uint16_t u16ClawRes2;                   //����Ԥ��2
//    uint16_t u16ClawRes3;                   //����Ԥ��3

////-------------------- 240-243�Ĵ�����������ز��� --------------------------
//    uint16_t Reg_BowRise_SoftMax;           //��������������ֵ��65535����1��
//    uint16_t Reg_BowRise_SoftMin;           //�������������Сֵ
//    uint16_t Reg_BowRise_MechMax;           //�������Ȼ�е���ֵ
//    uint16_t Reg_BowRise_MechMin;           //�������Ȼ�е��Сֵ

//    uint16_t Reg_BowRise_PosAllowErr;       //����λ������
//    uint16_t u16BowRiseRes0;                //����Ԥ��0
//    uint16_t u16BowRiseRes1;                //����Ԥ��1
//    uint16_t u16BowRiseRes2;                //����Ԥ��2

//    uint32_t Reg_BowRise_Kp;                 //����PID����ϵ��
//    uint16_t Reg_BowRise_Ki;                //����PID����ϵ��
//    uint16_t Reg_BowRise_Kd;                //����PID΢��ϵ�� 

//    uint16_t Reg_BowRise_Umax;              //����PID�������޷�
//    uint16_t Reg_BowRise_Umin;              //����PID�����С�޷�
//    uint16_t u16BowRiseRes3;                //����Ԥ��3
//    uint16_t u16BowRiseRes4;                //����Ԥ��4

////-------------------- 250-254�Ĵ�������ת��ز��� --------------------------
//    uint32_t Reg_Rotate_SoftMax;             //��ת�Ƕ�������ֵ��32767����360��
//    uint32_t Reg_Rotate_SoftMin;             //��ת�Ƕ������Сֵ

//    uint32_t Reg_Rotate_MechMax;             //��ת�ǶȻ�е���ֵ
//    uint32_t Reg_Rotate_MechMin;             //��ת�ǶȻ�е��Сֵ

//    uint32_t Reg_RotatePosOffSet;            //��תoffset
//    uint16_t Reg_Rotate_PosAllowErr;        //��תλ������
//    uint16_t u16RotateRes0;                 //��תԤ��0

//    uint32_t Reg_Rotate_Kp;                  //��תPID����ϵ��
//    uint16_t Reg_Rotate_Ki;                 //��תPID����ϵ��
//    uint16_t Reg_Rotate_Kd;                 //��תPID΢��ϵ��

//    uint32_t Reg_Rotate_Umax;               //��תλ�û����ֵ
//    uint32_t Reg_Rotate_Umin;               //��תλ�û���Сֵ
//    uint16_t u16RotateRes1;                 //��תԤ��1
//    uint16_t u16RotateRes2;                 //��תԤ��2

////--------------------- 260�Ĵ�������ŷ���ز��� ---------------------------
//    uint32_t Reg_ValveIp_Kp;                 //2~4���Ƹ˵��λ�û�PID����ϵ��
//    uint32_t Reg_ValveIp_Ki;                 //2~4���Ƹ˵��λ�û�PID����ϵ��

////--------------------- 270�Ĵ�����������ز��� ---------------------------
//    uint32_t ValveErrEnable;               //Һѹ�����������ʹ��

////------------------- 280-283�Ĵ�����������ز��� -------------------------
//    uint16_t Reg_Res0;                      //Ԥ��0
//    uint16_t Reg_Res1;                      //Ԥ��1
//    uint16_t Reg_Res2;                      //Ԥ��2
//    uint16_t Reg_Res3;                      //Ԥ��3

//    uint16_t Reg_Res4;                      //Ԥ��4
//    uint16_t Reg_Res5;                      //Ԥ��5
//    uint16_t Reg_Res6;                      //Ԥ��6
//    uint16_t Reg_Res7;                      //Ԥ��7

//    uint16_t Reg_Res8;                      //Ԥ��8
//    uint16_t Reg_Res9;                      //Ԥ��9
//    uint16_t Reg_Res10;                     //Ԥ��10
//    uint16_t Reg_Res11;                     //Ԥ��11

//    uint16_t Reg_Res12;                      //Ԥ��12
//    uint16_t Reg_Res13;                      //Ԥ��13
//    uint16_t Reg_Res14;                      //Ԥ��14
////    uint16_t Reg_Res15;                      //Ԥ��15

////    uint16_t Reg_Res16;                      //Ԥ��16
////    uint16_t Reg_Res17;                      //Ԥ��17
////    uint16_t Reg_Res18;                      //Ԥ��18
////    uint16_t Reg_Res19;                      //Ԥ��19
////    
////    uint16_t Reg_Res20;                      //Ԥ��20
////    uint16_t Reg_Res21;                      //Ԥ��21
////    uint16_t Reg_Res22;                      //Ԥ��22
////    uint16_t Reg_Res23;                      //Ԥ��23
////    
////    uint16_t Reg_Res24;                      //Ԥ��24
////    uint16_t Reg_Res25;                      //Ԥ��25
////    uint16_t Reg_Res26;                      //Ԥ��26
////    uint16_t Reg_Res27;                      //Ԥ��27
////    
////    uint16_t Reg_Res28;                      //Ԥ��28
////    uint16_t Reg_Res29;                      //Ԥ��29
////    uint16_t Reg_Res30;                      //Ԥ��30
////    uint16_t Reg_Res31;                      //Ԥ��31

//    uint16_t Reg_CRC;                       //������У��
//} ParamStruct_Solenoid;

typedef struct {
    uint16_t Reg_Tab_Ver ;                  //�Ĵ�����汾
    uint16_t Reg_Tab_Len ;                  //�Ĵ�������(��������)
    uint16_t Reg_Dev_Ver ;                  //�豸���ͱ���
    uint16_t Reg_HW_Ver ;                   //Ӳ���汾
    uint32_t Reg_SN ;                       //���к�
    uint32_t Reg_SW_Ver ;                   //
    uint32_t Reg_SW_NewApp;                 //NewApp����汾
    uint32_t Reg_SW_Factory;                //��������汾
    uint32_t Reg_SW_Boot;                   //boot����汾
    uint32_t Reg_Password ;                 //
    uint32_t Reg_WirelessBDSN ;             //���߿��ư����к�
    uint16_t Reg_PairingFlag;               //���߿��ư���Ա�־
    uint16_t Reg_WLSpdLimit;                //���߿��ư��ٶ�����
    uint16_t Reg_PCDogTimeOut ;             //��λ��������ʱ
    uint16_t Reg_BD1DogTimeOut ;            //��ת������������ʱ
    uint16_t Reg_BD2DogTimeOut ;            //����������������ʱ
    uint16_t Reg_BD3DogTimeOut ;            //����������������ʱ
    uint16_t Reg_BD4DogTimeOut ;            //̧ѹ������������ʱ
    uint16_t Reg_VCUDogTimeOut ;            //VCU������ʱ
    uint16_t Reg_RadarDogTimeOut ;            //�״�������ʱ
    uint16_t Reg_PCWalkAccTime ;            //PC�������߼���ʱ��
    uint16_t Reg_PCWalkDecTime ;            //PC�������߼���ʱ��
    uint16_t Reg_WLWalkAccTime ;            //WL�������߼���ʱ��
    uint16_t Reg_WLWalkDecTime ;            //WL�������߼���ʱ��
    uint16_t Reg_DrawWireDogTimeOut;        //����������ʱ
    uint16_t Reg_OverHeat;                  //
    uint16_t res;         //����������ʱ
    uint16_t Reg_GasSpeedBound;             //�ߵ��ٶ����Ÿ����ֽ�ֵ��_IQ��ʾ
    uint16_t Reg_HighSpdGasMax;             //�������Ÿ�������_IQ��ʾ
    uint16_t Reg_LowSpdGasMax;              //�������Ÿ�������_IQ��ʾ
    uint16_t Reg_BreakSpeedBound;           //�ߵ��ٶ�ɲ�������ֽ�ֵ��_IQ��ʾ
    uint16_t Reg_HighSpdBreakRange;         //���ٶ���ɲ��ɲ�����룬_IQ��ʾ
    uint16_t Reg_LowSpdBreakRange;          //���ٶ���ɲ��ɲ�����룬_IQ��ʾ
    uint16_t Reg_LoadBreakAdd;              //���ض���ɲ�����Ӿ��룬_IQ��ʾ
    uint16_t Reg_RC_LiftPos;                 //ң��������������
    uint16_t Reg_RC_ClawPos;                 //ң�ر��и�������
    uint16_t Reg_RC_BowRisePos;              //ң�ظ�����������
    uint16_t Reg_RC_RotatePos;               //ң����ת��������
    uint16_t Reg_RC_Speed;                   //ң���ٶȸ�����
    uint16_t Reg_GasBias;                   //����ƫ��
    uint16_t Reg_CRC ;                      //
} ParamStruct_Control;

typedef enum{
    Dev_ForlLift_Control = 0,
    Dev_ForkLift_Solenoid,
    Dev_ForkLift_485Hub,
    
    Dev_Inspection_Control = 10,
    Dev_Inspection_Lift,
    
    Dev_Textile_Control = 21,
}Dev_Ver_List;


#define Tab_Ver         1
#define Tab_Len         37
#define Dev_Ver         Dev_Textile_Control


#define ParamDefault    { \
    Tab_Ver,/* Reg_Tab_Ver*/\
    Tab_Len,/* Reg_Tab_Len*/\
    Dev_Ver,/* Reg_Dev_Ver*/\
    0,/* Reg_HW_Ver*/\
    0,/* Reg_SN*/\
    0,/* Reg_SW_Ver*/\
    0,/*Reg_SW_NewApp*/\
    0,/*Reg_SW_Factory*/\
    0,/*Reg_SW_Boot*/\
    0,/* Reg_Password*/\
    0,/* Reg_WirelessBDSN*/\
    0,/* Reg_PairingFlag*/\
    1000,/* Reg_WLSpdLimit*/\
    1000,/* Reg_PCDogTimeOut*/\
    1000,/* Reg_BD1DogTimeOut*/\
    1000,/* Reg_BD2ogTimeOut*/\
    1000,/* Reg_BD3DogTimeOut*/\
    1000,/* Reg_BD4DogTimeOut*/\
    1000,/* Reg_VCUDogTimeOut*/\
    1000,/* Reg_RadarDogTimeOut*/\
    20,/* Reg_PCWalkAccTime*/\
    10,/* Reg_PCWalkDecTime*/\
    50,/* Reg_WLWalkAccTime*/\
    10,/* Reg_WLWalkDecTime*/\
    1000,/* Reg_TelescopicMBDRelayRunTime*/\
    52428,/* Reg_OverHeat*/\
    9363,/* Reg_MileFormula*/\
    6553,/* Reg_SpeedBoundary*/\
    52428,/* Reg_HighSpdGasMax*/\
    32767,/* Reg_LowSpdGasMax*/\
    1310,/* Reg_BreakSpeedBound*/\
    170,/* Reg_HighSpdBreakRange*/\
    300,/* Reg_LowSpdBreakRange*/\
    30,/* Reg_LoadBreakAdd*/\
    3276,/* Reg_RC_LiftPos*/\
    3276,/* Reg_RC_ClawPos*/\
    500,/* Reg_RC_BowRisePos*/\
    3276,/* Reg_RC_RotatePos*/\
    500,/* Reg_RC_Speed*/\
    19660,/* Reg_GasBias*/\
    0,/* Reg_CRC*/\
};

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
#define EepromParamSaving_SolenoidResult        0x2222
#define EepromParamSaving_SteerResult           0x3333

#define SoftVer_A                               1  //��ΧΪ0~255��AΪ��汾��һ��Ϊ�ӿڵı仯����汾�仯����ԭ�п�ܲ�����
#define SoftVer_B                               11  //��ΧΪ0~255��BΪ���ܰ汾�����ӹ�����仯B��
#define SoftVer_C                               1  //��ΧΪ0~255��CΪbug�޸ģ������C��

//-------------------- public data ------------------------------------------
extern ParamStruct_Control ParamHandle_Control;
//extern ParamStruct_Solenoid ParamHandle_Solenoid;
//extern ParamStruct_Steer ParamHandle_Steer;
//extern ParamStruct_Hub485 ParamHandle_Hub485_Fork;
//extern ParamStruct_Hub485 ParamHandle_Hub485_Body;
//-------------------- public functions -------------------------------------
void DataBaseInit(void);
void CtrlDataBaseInit ( void );
int8_t Param2Eeprom ( void );

#endif // _DataProcess__H_

//-----------------------End of file------------------------------------------
/** @}*/

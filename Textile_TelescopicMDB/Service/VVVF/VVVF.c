/**
* 版权所有(C)
*
* ********
*
* @file VVVF.c
* @brief
* @details
* @author MuNiu
* @version 1.0.0
* @date xxxx-xx-xx
* @par Description:
* @par Change History:
*
* <日期> | <版本> | <作者> | <描述>
*
* xxxx/xx/xx | 1.0.0 | MuNiu | 创建文件
*
*/
//-------------------- include files ----------------------------------------
#include "VVVF.h"
#include "SVPWM.h"
#include "TimerPWM.h"
#include "Int_Ctrl.h"
#include "DataBaseProcess.h"
#include "ReangleRS485.h"

//-------------------- local definitions ------------------------------------

//-------------------- private data -----------------------------------------

//-------------------- private functions ------------------------------------

//-------------------- public data ------------------------------------------
VVVFDef VVVFStruct = VVVFDef_DEFAULTS;
ElcAngDef ElcAngCal = ElcAngDef_DEFAULTS;
uint16_t AngleErrNum[100]={0};

//-------------------- public functions -------------------------------------
/*! \fn				void VVVF_Ctrl( void )
 *  \brief 		VVVF process
 *  \param 		none
 *  \return 	none
 */
void VVVF_Ctrl ( void )
{
    static _iq iqMotor_Ke;
    
    iqMotor_Ke = _IQdiv ( ParamHandle_Steer.Reg_RatedVoltage, 100 * ParamHandle_Steer.Reg_NoLoadSpeed );
    if ( VVVFStruct.iqMotor_Spd > VVVFStruct.iqMotor_VVVFMaxSpd ) VVVFStruct.iqMotor_Spd = VVVFStruct.iqMotor_VVVFMaxSpd;  //VVVF速度限制
    VVVFStruct.iqAngle_MaxStep = FRE_20KHZ * 60 / ( VVVFStruct.iqMotor_Spd * ParamHandle_Steer.Reg_PolePairs );
    
    if ( VVVFStruct.iqAngle_MaxStepLast != VVVFStruct.iqAngle_MaxStep )  //速度切换，先关管
    {
        VVVFStruct.iqAngle_Step = 0;
        ATIM_TIMX_CPLM_CHY_CCRY1 = 0;
        ATIM_TIMX_CPLM_CHY_CCRY2 = 0;
        ATIM_TIMX_CPLM_CHY_CCRY3 = 0;
    }
        
    if(ElcAngCal.RunDir == 0)  //正反转标志
    {
        if (( VVVFStruct.iqAngle_Step < ( VVVFStruct.iqAngle_MaxStep - 1 ) ))
        {
            VVVFStruct.iqAngle_Step += 1;
        }
        else
        {
            VVVFStruct.iqAngle_Step = 0;
        }
    }
    if(ElcAngCal.RunDir == 1)
    {
        if (( VVVFStruct.iqAngle_Step >= 1 ))
        {
            VVVFStruct.iqAngle_Step -= 1;
        }
        else
        {
            VVVFStruct.iqAngle_Step = VVVFStruct.iqAngle_MaxStep - 1;
        }
    }
    VVVFStruct.iqMotor_SpdKe_PWM = iqMotor_Ke * VVVFStruct.iqMotor_Spd / 100;
    VVVFStruct.iqAngle_MaxStepLast = VVVFStruct.iqAngle_MaxStep;
}

uint16_t Motor_Angle_Poles_Step = 0;
//-------------------- public functions -------------------------------------
/*! \fn				void MotorElcAng( void )
电机电角度计算
 */
void MotorElcAng ( uint8_t PolesNum )
{
    static uint16_t Theta1 = 0;
//将机械角度与电角度对应
    Theta =  GetReangleValue();
    Theta1 = Theta;
    Motor_Angle_Poles_Step = 65535/PolesNum;
    
    if(Theta>Motor_Angle_Poles_Step*4)
    {
        Theta = Theta - Motor_Angle_Poles_Step*4;
    }
    else if(Theta>Motor_Angle_Poles_Step*3)
    {
        Theta = Theta - Motor_Angle_Poles_Step*3;
    }
    else if(Theta>Motor_Angle_Poles_Step*2)
    {
        Theta = Theta - Motor_Angle_Poles_Step*2;
    }
    else if(Theta>Motor_Angle_Poles_Step)
    {
        Theta = Theta - Motor_Angle_Poles_Step;
    }
    else{}
        
    switch(ElcAngCal.MotorElcAngStep)
    {
        case 0:
                
        break;
        
        case 1:
              ElcAngCal.AngleAdd =  Theta1 -  ElcAngCal.LastAngle;
                if(ElcAngCal.AngleAdd > 30000)
                {
                   ElcAngCal.AngleAdd = ElcAngCal.AngleAdd - 65535;
                }
                else if(ElcAngCal.AngleAdd < - 30000)
                {
                   ElcAngCal.AngleAdd = ElcAngCal.AngleAdd + 65535;
                }

                ElcAngCal.AngleSum +=ElcAngCal.AngleAdd ;
                
                if(abs(ElcAngCal.AngleSum) > 13107)
                {
                    ElcAngCal.MotorElcAngCnt++;          //换向计数
                }

                if(ElcAngCal.MotorElcAngCnt > 0)
                {
                   ElcAngCal.MotorElcAngStep = 2; 
                }
                ElcAngCal.AngleTimeCnt++;
                
                ThetaGer = SVPWMStruct.Theta/(10*PI);

                Theta_err =  ThetaGer - Theta;
                if(Theta_err<0)
                {
                    Theta_err = Theta_err+13107;
                }
                if(ElcAngCal.AngleTimeCnt%10 == 0)
                {
                    if(ElcAngCal.i<100)
                    {
                        AngleErrNum[ElcAngCal.i] = Theta_err;  
                        ElcAngCal.i++;
                    }
                    else
                    {
                        ElcAngCal.i = 0;
                    }
                }
              ElcAngCal.LastAngle = Theta1;
        break;
        case 2:
               for(ElcAngCal.j = 0;ElcAngCal.j < 100;ElcAngCal.j++)
               {
                  ElcAngCal.Angle_ErrSum +=AngleErrNum[ElcAngCal.j]; 
               }
                ElcAngCal.Theta_ErrAvg_P = ElcAngCal.Angle_ErrSum/ElcAngCal.j;
                ElcAngCal.RunDir = 1;   //反向
                VVVFStruct.iqAngle_Step = VVVFStruct.iqAngle_MaxStep - 1;
                ElcAngCal.AngleSum = 0;
                ElcAngCal.Angle_ErrSum = 0;
                ElcAngCal.i = 0;
                ElcAngCal.MotorElcAngStep = 3; 
        break;
        
        case 3:
                ElcAngCal.AngleAdd =  Theta1 -  ElcAngCal.LastAngle;
                ElcAngCal.AngleTimeCnt++;
    
                if(ElcAngCal.AngleAdd > 30000)
                {
                   ElcAngCal.AngleAdd = ElcAngCal.AngleAdd - 65535;
                }
                else if(ElcAngCal.AngleAdd < - 30000)
                {
                   ElcAngCal.AngleAdd = ElcAngCal.AngleAdd + 65535;
                }
                ElcAngCal.AngleSum +=ElcAngCal.AngleAdd ;
                
                if(abs(ElcAngCal.AngleSum) > 13107)
                {
                    VVVFStruct.u8Enable = 0;
                    ElcAngCal.MotorElcAngStep = 4;          
                }
                ThetaGer = SVPWMStruct.Theta/(10*PI);

                Theta_err =  ThetaGer - Theta;
                if(Theta_err<0)
                {
                    Theta_err = Theta_err+13107;
                }
                if(ElcAngCal.AngleTimeCnt%10 == 0)
                {
                    if(ElcAngCal.i<100)
                    {
                        AngleErrNum[ElcAngCal.i] = Theta_err;  
                        ElcAngCal.i++;
                    }
                    else
                    {
                        ElcAngCal.i = 0;
                    }
                }
              ElcAngCal.LastAngle = Theta1;
        break;
        case 4:
               for(ElcAngCal.j = 0;ElcAngCal.j < 100;ElcAngCal.j++)
               {
                  ElcAngCal.Angle_ErrSum +=AngleErrNum[ElcAngCal.j]; 
               }
                ElcAngCal.i = 0;
                ElcAngCal.Theta_ErrAvg_N = ElcAngCal.Angle_ErrSum/ElcAngCal.j;
                ElcAngCal.MotorElcAngStep = 5; 
        break;
               
        case 5:
                ElcAngCal.Theta_Err  = (ElcAngCal.Theta_ErrAvg_N+ElcAngCal.Theta_ErrAvg_P)/2;
                ElcAngCal.MotorElcAngStep = 6; 
        break;
        case 6:
            ElcAngCal.Theta_Err = 1640;
            ElcAngCal.ElecAngle =  ((ElcAngCal.Theta_Err + Theta)*5);            //电角度 = （偏差 + 机械角度） * 极对数
            Angle_IQ0_IQ1 ( &ElcAngCal.ElecAngle );

        break;        
    }
}

//-----------------------End of file------------------------------------------
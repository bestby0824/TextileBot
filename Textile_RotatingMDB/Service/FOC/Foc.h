
#include "AngleMath.h"
#include "SVPWM.h"
#include "MotorInfo.h"
#include "MotorCmd_Ctrl.h"

typedef struct
{
    _iq Id;  // d-axis current
    _iq Iq;  // q-axis current
} IDQ_Def;


typedef struct {
    _iq  As;  		// Input: phase-a stator variable
    _iq  Bs;			// Input: phase-b stator variable
    _iq  Cs;			// Input: phase-c stator variable
    _iq  Alpha;		// Output: stationary d-axis stator variable
    _iq  Beta;		// Output: stationary q-axis stator variable
} CLARK;

typedef struct {
    _iq  Alpha;  		// Input: stationary d-axis stator variable
    _iq  Beta;	 	// Input: stationary q-axis stator variable
    _iq  Angle;		// Input: rotating angle (pu)
    _iq  Ds;			// Output: rotating d-axis stator variable
    _iq  Qs;			// Output: rotating q-axis stator variable
    _iq  Sine;
    _iq  Cosine;
} PARK;
typedef struct {
    _iq  Alpha;  		// Output: stationary d-axis stator variable
    _iq  Beta;		// Output: stationary q-axis stator variable
    _iq  Angle;		// Input: rotating angle (pu)
    _iq  Ds;			// Input: rotating d-axis stator variable
    _iq  Qs;			// Input: rotating q-axis stator variable
    _iq  Sine;		// Input: Sine term
    _iq  Cosine;		// Input: Cosine term
} IPARK;

typedef struct {
    _iq  Ref;     // Input: reference set-point
    _iq  Fbk;     // Input: feedback
    _iq  Out;     // Output: controller output
    _iq  Kp;    // Parameter: proportional loop gain
    _iq  Ki;     // Parameter: integral gain
    _iq  Umax;   // Parameter: upper saturation limit
    _iq  Umin;   // Parameter: lower saturation limit
    _iq  up;    // Data: proportional term
    _iq  ui;    // Data: integral term
    _iq  v1;    // Data: pre-saturated controller output
    _iq  i1;    // Data: integrator storage: ui(k-1)
    _iq  w1;    // Data: saturation record: [u(k-1) - v(k-1)]

    _iq Kd;     // Parameter: derivative gain
    _iq Km;     // Parameter: derivative weighting
    _iq c1;     // Internal: derivative filter coefficient 1
    _iq c2;     // Internal: derivative filter coefficient 2
    _iq d1;     // Data: differentiator storage: ud(k-1)
    _iq d2;     // Data: differentiator storage: d2(k-1)
    _iq ud;     // Data: derivative term
} PI_CONTROLLER;

#define PI_CONTROLLER_DEFAULTS {         \
       0,    /*Ref*/             \
       0,    /*Fbk*/             \
       0,    /*Out*/             \
              _IQ(0.0), /*Kp*/   \
              _IQ(0.0), /*Ki*/   \
              _IQ(0.5), /*Umax*/ \
              _IQ(-0.5), /*Umin*/\
              _IQ(0.0), /*up*/   \
              _IQ(0.0),  /*ui*/  \
              _IQ(0.0), /*v1*/   \
              _IQ(0.0), /*i1*/   \
              _IQ(1.0),  /*w1*/  \
 \
              _IQ(0.0), /*Kd*/   \
              _IQ(0.0), /*Km*/   \
              _IQ(0.0), /*c1*/   \
              _IQ(0.0), /*c2*/   \
              _IQ(0.0), /*d1*/   \
              _IQ(0.0), /*d2*/   \
              _IQ(0.0) /*ud*/    \
}

void InvParkTra ( IPARK *ipark );          // ·´PARK±ä»»
void FOC_Core ( S_MOTOR_INFO *psInfo, S_MOTOR_CMD *psCmd );
void Current_DLoop ( PI_CONTROLLER * v );
void Current_QLoop ( PI_CONTROLLER * v );
void FOC_PID_Init ( void );
void SetVector ( _iq Value, _iq Angle );

extern uint8_t u8StopFalg;


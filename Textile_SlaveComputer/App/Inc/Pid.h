
//------------------------------------------------------------------------------

#ifndef _PID_H_
#define _PID_H_
//-------------------- include files ----------------------------------------
#include "Driver.h"
#include "IQmath.h"
//-------------------- public definitions -----------------------------------


//-------------------- public data ------------------------------------------



//-------------------- public functions -------------------------------------


//-------------------- inline functions -------------------------------------

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


/*-----------------------------------------------------------------------------
Default initalisation values for the PI_GRANDO objects
-----------------------------------------------------------------------------*/

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


/*------------------------------------------------------------------------------
  PI_GRANDO Macro Definition
------------------------------------------------------------------------------*/

#define PI_MACRO(v)         \
                    \
 /* proportional term */    \
 v.up = v.Ref - v.Fbk;      \
                    \
 /* integral term */        \
 v.ui = (_IQrmpy(v.Ki, v.up)+ v.i1);          \
 v.ui = ((v.Out == v.v1)||(_IQabs( v.ui)< _IQabs( v.i1))) ? v.ui : _IQrmpy(v.i1, _IQ(0.99)); \
 v.i1 = v.ui;  /*限幅生效则积分停止增长且产生0.01的阻尼*/           \
                    \
 /* control output */       \
 v.v1 = _IQrmpy(v.Kp, (v.up + v.ui));  \
 v.Out= _IQsat(v.v1, v.Umax, v.Umin); \
 //v.w1 = (v.Out == v.v1) ? _IQ(1.0) : _IQ(0.0); 


#define PI_SPD_MACRO(v)     \
                      \
 /* proportional term */    \
 v.up = (v.Ref - v.Fbk);    \
                      \
 /* integral term */        \
 v.ui = (_IQrmpy(v.Ki, v.up)+ v.i1);          \
 v.ui = ((v.Out == v.v1)||(_IQabs( v.ui)< _IQabs( v.i1))) ? v.ui : _IQrmpy(v.i1, _IQ(0.99)); \
 v.i1 = v.ui;  /*限幅生效则积分停止增长且产生0.01的阻尼*/           \
 /* derivative term */      \
 v.d2 = _IQrmpy(v.Kd, _IQrmpy(v.c1, (_IQrmpy(v.Ref, v.Km) - v.Fbk))) - v.d2; \
 v.ud = v.d2 + v.d1;        \
 v.d1 = _IQrmpy(v.ud, v.c2); \
                      \
 /* control output */       \
 v.v1 = _IQrmpy(v.Kp, (v.up + v.ui + v.ud));    \
 v.Out= _IQsat(v.v1, v.Umax, v.Umin);   \
 //v.w1 = (v.Out == v.v1) ? _IQ(1.0) : _IQ(0.0);    

// ***********************************************************************************
//   This macro works with angles as inputs, hence error is rolled within -pi to +pi
// ***********************************************************************************
#define PI_POS_MACRO(v)     \
 /* proportional term */    \
 v.up =  (v.Ref - v.Fbk);   \
 /* integral term */        \
 v.ui = (_IQrmpy(v.Ki, v.up)+ v.i1);          \
 v.ui = ((v.Out == v.v1)||(_IQabs( v.ui)< _IQabs( v.i1))) ? v.ui : _IQrmpy(v.i1, _IQ(0.99)); \
 v.i1 = v.ui;  /*限幅生效则积分停止增长且产生0.01的阻尼*/           \
                    \
 /* control output */       \
 v.v1 = _IQrmpy(v.Kp, (v.up + v.ui));  \
 v.Out= _IQsat(v.v1, v.Umax, v.Umin); \
 //v.w1 = (v.Out == v.v1) ? _IQ(1.0) : _IQ(0.0);   










#endif /* Pid_H_ */
//-----------------------End of file------------------------------------------
/** */

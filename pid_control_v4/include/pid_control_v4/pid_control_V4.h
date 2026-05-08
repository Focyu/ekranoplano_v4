/*
 * pid_control_V4.h
 *
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * Code generation for model "pid_control_V4".
 *
 * Model version              : 12.168
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C++ source code generated on : Thu May  7 21:50:57 2026
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef pid_control_V4_h_
#define pid_control_V4_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros2_initialize.h"
#include "pid_control_V4_types.h"
#include "rmw/qos_profiles.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include <string.h>

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include <stddef.h>
#include "zero_crossing_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals for system '<Root>/MATLAB Function' */
struct B_MATLABFunction_pid_control__T {
  uint32_T lengthOut;                  /* '<Root>/MATLAB Function' */
  uint8_T stringOut[128];              /* '<Root>/MATLAB Function' */
};

/* Block signals for system '<S293>/Enabled Subsystem' */
struct B_EnabledSubsystem_pid_contro_T {
  SL_Bus_std_msgs_Bool In1;            /* '<S336>/In1' */
};

/* Block signals for system '<S294>/Enabled Subsystem' */
struct B_EnabledSubsystem_pid_cont_d_T {
  SL_Bus_std_msgs_Float64 In1;         /* '<S337>/In1' */
};

/* Block signals (default storage) */
struct B_pid_control_V4_T {
  SL_Bus_nav_msgs_Odometry BusAssignmentODOM;/* '<Root>/Bus Assignment-ODOM' */
  SL_Bus_sensor_msgs_Imu BusAssignmentIMU;/* '<Root>/Bus Assignment-IMU' */
  SL_Bus_gazebo_msgs_SetEntityStateRequest BusAssignmentMODELO;/* '<Root>/Bus Assignment-MODELO' */
  real_T IC[12];                       /* '<S19>/IC' */
  real_T x[12];                        /* '<S19>/Integrator' */
  real_T R[9];
  real_T RotationAnglestoDirectionCo[9];
                        /* '<S19>/Rotation Angles to Direction Cosine Matrix' */
  real_T dv[9];
  real_T TmpSignalConversionAtSFunct[5];/* '<S19>/MATLAB Function - MODEL' */
  char_T b_zeroDelimTopic[25];
  real_T wbe_b[3];
  real_T FE1_b[3];
  real_T F_b[3];
  real_T Product_l[3];                 /* '<S329>/Product' */
  real_T Dtot[3];
  char_T b_zeroDelimTopic_m[22];
  char_T b_zeroDelimTopic_c[22];
  char_T b_zeroDelimTopic_k[18];
  char_T b_zeroDelimTopic_cx[17];
  char_T b_zeroDelimTopic_b[17];
  char_T b_zeroDelimTopic_p[17];
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF deadline_c;
  sJ4ih70VmKcvCeguWN0mNVF deadline_f;
  sJ4ih70VmKcvCeguWN0mNVF deadline_g;
  sJ4ih70VmKcvCeguWN0mNVF deadline_g1;
  sJ4ih70VmKcvCeguWN0mNVF deadline_m;
  sJ4ih70VmKcvCeguWN0mNVF deadline_n;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF deadline_p;
  sJ4ih70VmKcvCeguWN0mNVF lifespan_l;
  sJ4ih70VmKcvCeguWN0mNVF deadline_j;
  real_T frac[2];
  real_T dv1[2];
  real_T GainZ;                        /* '<Root>/Gain-Z' */
  real_T Saturation1;                  /* '<Root>/Saturation1' */
  real_T FilterCoefficient;            /* '<S273>/Filter Coefficient' */
  real_T Saturation;                   /* '<S277>/Saturation' */
  real_T Switch3;                      /* '<Root>/Switch3' */
  real_T FilterCoefficient_j;          /* '<S113>/Filter Coefficient' */
  real_T Saturation_f;                 /* '<S117>/Saturation' */
  real_T RateLimiter2;                 /* '<Root>/Rate Limiter2' */
  real_T FilterCoefficient_p;          /* '<S219>/Filter Coefficient' */
  real_T Saturation_m;                 /* '<S223>/Saturation' */
  real_T Saturation_roll_sp;           /* '<Root>/Saturation_roll_sp' */
  real_T FilterCoefficient_c;          /* '<S61>/Filter Coefficient' */
  real_T Saturation_k;                 /* '<S65>/Saturation' */
  real_T RateLimitertheta_sp;          /* '<Root>/Rate Limiter-theta_sp' */
  real_T FilterCoefficient_m;          /* '<S167>/Filter Coefficient' */
  real_T Saturation_fh;                /* '<S171>/Saturation' */
  real_T Memory[3];                    /* '<S19>/Memory' */
  real_T Memory1[3];                   /* '<S19>/Memory1' */
  real_T Switch;                       /* '<S48>/Switch' */
  real_T SumI4;                        /* '<S102>/SumI4' */
  real_T Switch_a;                     /* '<S154>/Switch' */
  real_T SumI4_d;                      /* '<S208>/SumI4' */
  real_T Switch_j;                     /* '<S260>/Switch' */
  real_T Product[4];                   /* '<S308>/Product' */
  real_T Switch_p[3];                  /* '<S19>/Switch' */
  real_T Switch1[3];                   /* '<S19>/Switch1' */
  real_T data;
  real_T data_n;
  real_T Power;                        /* '<S19>/Product2' */
  real_T Gain3;                        /* '<S19>/Gain3' */
  real_T EnergykWh;                    /* '<S19>/Gain1' */
  real_T powerdemand;                  /* '<S19>/Divide' */
  real_T loadtorque;                   /* '<S19>/Divide1' */
  real_T Output;                       /* '<S288>/Output' */
  real_T Sum[3];                       /* '<S19>/Sum' */
  real_T Sum1[3];                      /* '<S19>/Sum1' */
  real_T XDOT[40];                     /* '<S19>/MATLAB Function - MODEL' */
  real_T CL_total;                     /* '<S19>/MATLAB Function - MODEL' */
  real_T mu_Lw_out;                    /* '<S19>/MATLAB Function - MODEL' */
  real_T mu_Dw_out;                    /* '<S19>/MATLAB Function - MODEL' */
  real_T w[2];                         /* '<S314>/w' */
  real_T w_a[2];                       /* '<S314>/w ' */
  real_T LwgV1[2];                     /* '<S314>/Lwg//V 1' */
  real_T w_g[2];                       /* '<S313>/w' */
  real_T w_e[2];                       /* '<S313>/w ' */
  real_T w1[2];                        /* '<S313>/w 1' */
  real_T w_n[2];                       /* '<S312>/w' */
  real_T w1_c[2];                      /* '<S312>/w1' */
  real_T w_d[2];                       /* '<S311>/w' */
  real_T w_e0[2];                      /* '<S310>/w' */
  real_T UnaryMinus[2];                /* '<S310>/Unary Minus' */
  real_T w_o[2];                       /* '<S309>/w' */
  real_T sigma_w[2];                   /* '<S309>/sigma_w' */
  real_T u1;
  real_T u2;
  real_T w_r;
  real_T Va;
  real_T beta;
  real_T q_aero;
  real_T hw;
  real_T hh;
  real_T Q;
  real_T CL_w_IGE;
  real_T CL_h_IGE;
  real_T CD_iw_IGE;
  real_T CD_ih_IGE;
  real_T u2_deg;
  real_T Ltot;
  real_T CQ;
  real_T Cl;
  real_T Vd1;
  real_T L_dec;
  real_T c_phi;
  real_T s_phi;
  real_T c_the;
  real_T s_the;
  real_T c_psi;
  real_T s_psi;
  real_T sina;
  real_T sinb;
  real_T sinc;
  real_T cosa;
  real_T cosb;
  real_T cosc;
  real_T z_out;                     /* '<Root>/MATLAB Function-ned_to_gazebo' */
  real_T SignPreSat;                   /* '<S48>/SignPreSat' */
  real_T FE_b;
  real_T Mcg_b_idx_0;
  real_T FE2_b_idx_0;
  real_T FE2_b_idx_2;
  real_T Fg_b_idx_2;
  real_T rtb_CoordinateTransformationC_d;
  real_T rtb_CoordinateTransformationC_g;
  real_T rtb_CoordinateTransformationC_l;
  real_T FE_b_idx_0;
  real_T FA_b_idx_0;
  real_T FA_b_idx_1;
  real_T FA_b_idx_2;
  real_T R_tmp;
  real_T R_tmp_d;
  real_T Ltot_tmp;
  real_T Switch2;                      /* '<Root>/Switch2' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2_k;/* '<S295>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2_p;/* '<S294>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2;/* '<S21>/SourceBlock' */
  uint32_T bpIndex[2];
  uint32_T lengthOut;                  /* '<Root>/MATLAB Function1' */
  uint32_T currentLen;                 /* '<Root>/MATLAB Function-Odometry1' */
  uint32_T receivedLen;                /* '<Root>/MATLAB Function-Odometry1' */
  uint32_T currentLen_o;               /* '<Root>/MATLAB Function-Odometry' */
  uint32_T receivedLen_j;              /* '<Root>/MATLAB Function-Odometry' */
  uint8_T stringOut[128];              /* '<Root>/MATLAB Function1' */
  uint8_T str_out[128];                /* '<Root>/MATLAB Function-Odometry1' */
  uint8_T str_out_a[128];              /* '<Root>/MATLAB Function-Odometry' */
  boolean_T Compare;                   /* '<S289>/Compare' */
  boolean_T AND3;                      /* '<S48>/AND3' */
  boolean_T Memory_a;                  /* '<S48>/Memory' */
  boolean_T AND3_j;                    /* '<S154>/AND3' */
  boolean_T Memory_n;                  /* '<S154>/Memory' */
  boolean_T AND3_c;                    /* '<S260>/AND3' */
  boolean_T Memory_h;                  /* '<S260>/Memory' */
  boolean_T SourceBlock_o1;            /* '<S21>/SourceBlock' */
  boolean_T SourceBlock_o1_o;          /* '<S20>/SourceBlock' */
  boolean_T SourceBlock_o1_h;          /* '<S296>/SourceBlock' */
  boolean_T SourceBlock_o1_d;          /* '<S295>/SourceBlock' */
  boolean_T SourceBlock_o1_c;          /* '<S294>/SourceBlock' */
  boolean_T SourceBlock_o1_k;          /* '<S293>/SourceBlock' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_a;/* '<S21>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_b;/* '<S20>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_contro_T EnabledSubsystem_pt;/* '<S296>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_p;/* '<S295>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_k;/* '<S294>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_contro_T EnabledSubsystem;/* '<S293>/Enabled Subsystem' */
  B_MATLABFunction_pid_control__T sf_MATLABFunctionIMU2;/* '<Root>/MATLAB Function-IMU2' */
  B_MATLABFunction_pid_control__T sf_MATLABFunction;/* '<Root>/MATLAB Function' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_pid_control_V4_T {
  ros_slros2_internal_block_Ser_T obj; /* '<S4>/ServiceCaller' */
  ros_slros2_internal_block_Pub_T obj_o;/* '<S17>/SinkBlock' */
  ros_slros2_internal_block_Pub_T obj_i;/* '<S16>/SinkBlock' */
  ros_slros2_internal_block_Sub_T obj_k;/* '<S21>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_m;/* '<S20>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h;/* '<S296>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_p;/* '<S295>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_hy;/* '<S294>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h4;/* '<S293>/SourceBlock' */
  real_T UnitDelay3_DSTATE;            /* '<Root>/Unit Delay3' */
  real_T UnitDelay2_DSTATE;            /* '<Root>/Unit Delay2' */
  real_T Memory2_PreviousInput[12];    /* '<S19>/Memory2' */
  real_T PrevY;                        /* '<Root>/Rate Limiter2' */
  real_T PrevY_g;                      /* '<Root>/Rate Limiter-theta_sp' */
  real_T LastMajorTime;                /* '<Root>/Rate Limiter-theta_sp' */
  real_T Memory_PreviousInput[3];      /* '<S19>/Memory' */
  real_T Memory1_PreviousInput[3];     /* '<S19>/Memory1' */
  real_T NextOutput[4];                /* '<S308>/White Noise' */
  real_T NextOutput_k;                 /* '<S288>/White Noise' */
  struct {
    void *LoggedData;
  } ToWorkspace_PWORK;                 /* '<Root>/To Workspace' */

  struct {
    void *LoggedData;
  } ToWorkspace_PWORK_g;               /* '<S19>/To Workspace' */

  struct {
    void *LoggedData;
  } ToWorkspace1_PWORK;                /* '<S19>/To Workspace1' */

  struct {
    void *LoggedData;
  } ToWorkspace10_PWORK;               /* '<S19>/To Workspace10' */

  struct {
    void *LoggedData;
  } ToWorkspace11_PWORK;               /* '<S19>/To Workspace11' */

  struct {
    void *LoggedData;
  } ToWorkspace12_PWORK;               /* '<S19>/To Workspace12' */

  struct {
    void *LoggedData;
  } ToWorkspace13_PWORK;               /* '<S19>/To Workspace13' */

  struct {
    void *LoggedData;
  } ToWorkspace14_PWORK;               /* '<S19>/To Workspace14' */

  struct {
    void *LoggedData;
  } ToWorkspace15_PWORK;               /* '<S19>/To Workspace15' */

  struct {
    void *LoggedData;
  } ToWorkspace16_PWORK;               /* '<S19>/To Workspace16' */

  struct {
    void *LoggedData;
  } ToWorkspace17_PWORK;               /* '<S19>/To Workspace17' */

  struct {
    void *LoggedData;
  } ToWorkspace18_PWORK;               /* '<S19>/To Workspace18' */

  struct {
    void *LoggedData;
  } ToWorkspace19_PWORK;               /* '<S19>/To Workspace19' */

  struct {
    void *LoggedData;
  } ToWorkspace2_PWORK;                /* '<S19>/To Workspace2' */

  struct {
    void *LoggedData;
  } ToWorkspace20_PWORK;               /* '<S19>/To Workspace20' */

  struct {
    void *LoggedData;
  } ToWorkspace21_PWORK;               /* '<S19>/To Workspace21' */

  struct {
    void *LoggedData;
  } ToWorkspace22_PWORK;               /* '<S19>/To Workspace22' */

  struct {
    void *LoggedData;
  } ToWorkspace23_PWORK;               /* '<S19>/To Workspace23' */

  struct {
    void *LoggedData;
  } ToWorkspace24_PWORK;               /* '<S19>/To Workspace24' */

  struct {
    void *LoggedData;
  } ToWorkspace25_PWORK;               /* '<S19>/To Workspace25' */

  struct {
    void *LoggedData;
  } ToWorkspace26_PWORK;               /* '<S19>/To Workspace26' */

  struct {
    void *LoggedData;
  } ToWorkspace27_PWORK;               /* '<S19>/To Workspace27' */

  struct {
    void *LoggedData;
  } ToWorkspace28_PWORK;               /* '<S19>/To Workspace28' */

  struct {
    void *LoggedData;
  } ToWorkspace29_PWORK;               /* '<S19>/To Workspace29' */

  struct {
    void *LoggedData;
  } ToWorkspace3_PWORK;                /* '<S19>/To Workspace3' */

  struct {
    void *LoggedData;
  } ToWorkspace30_PWORK;               /* '<S19>/To Workspace30' */

  struct {
    void *LoggedData;
  } ToWorkspace31_PWORK;               /* '<S19>/To Workspace31' */

  struct {
    void *LoggedData;
  } ToWorkspace32_PWORK;               /* '<S19>/To Workspace32' */

  struct {
    void *LoggedData;
  } ToWorkspace33_PWORK;               /* '<S19>/To Workspace33' */

  struct {
    void *LoggedData;
  } ToWorkspace4_PWORK;                /* '<S19>/To Workspace4' */

  struct {
    void *LoggedData;
  } ToWorkspace5_PWORK;                /* '<S19>/To Workspace5' */

  struct {
    void *LoggedData;
  } ToWorkspace6_PWORK;                /* '<S19>/To Workspace6' */

  struct {
    void *LoggedData;
  } ToWorkspace7_PWORK;                /* '<S19>/To Workspace7' */

  struct {
    void *LoggedData;
  } ToWorkspace8_PWORK;                /* '<S19>/To Workspace8' */

  struct {
    void *LoggedData;
  } ToWorkspace9_PWORK;                /* '<S19>/To Workspace9' */

  uint32_T PreLookUpIndexSearchprobofexcee;
                        /* '<S315>/PreLook-Up Index Search  (prob of exceed)' */
  uint32_T PreLookUpIndexSearchaltitude_DW;
                              /* '<S315>/PreLook-Up Index Search  (altitude)' */
  uint32_T RandSeed[4];                /* '<S308>/White Noise' */
  uint32_T RandSeed_a;                 /* '<S288>/White Noise' */
  robotics_slcore_internal_bloc_T obj_c;
                             /* '<Root>/Coordinate Transformation Conversion' */
  int8_T ifHeightMaxlowaltitudeelseifHei;
  /* '<S304>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  int8_T ifHeightMaxlowaltitudeelseifH_a;
  /* '<S303>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  boolean_T IC_FirstOutputTime;        /* '<S19>/IC' */
  boolean_T Integrator_DWORK1;         /* '<S19>/Integrator' */
  boolean_T PrevLimited;               /* '<Root>/Rate Limiter-theta_sp' */
  boolean_T Memory_PreviousInput_o;    /* '<S48>/Memory' */
  boolean_T Memory_PreviousInput_d;    /* '<S154>/Memory' */
  boolean_T Memory_PreviousInput_a;    /* '<S260>/Memory' */
  boolean_T objisempty;                /* '<S21>/SourceBlock' */
  boolean_T objisempty_g;              /* '<S20>/SourceBlock' */
  boolean_T objisempty_a;              /* '<S296>/SourceBlock' */
  boolean_T objisempty_e;              /* '<S295>/SourceBlock' */
  boolean_T objisempty_l;              /* '<S294>/SourceBlock' */
  boolean_T objisempty_c;              /* '<S293>/SourceBlock' */
  boolean_T objisempty_ev;             /* '<S17>/SinkBlock' */
  boolean_T objisempty_m;              /* '<S16>/SinkBlock' */
  boolean_T objisempty_d;    /* '<Root>/Coordinate Transformation Conversion' */
  boolean_T objisempty_f;              /* '<S4>/ServiceCaller' */
  boolean_T Hwgws_MODE;                /* '<S299>/Hwgw(s)' */
  boolean_T Hvgws_MODE;                /* '<S299>/Hvgw(s)' */
  boolean_T Hugws_MODE;                /* '<S299>/Hugw(s)' */
  boolean_T Hrgw_MODE;                 /* '<S298>/Hrgw' */
  boolean_T Hqgw_MODE;                 /* '<S298>/Hqgw' */
  boolean_T Hpgw_MODE;                 /* '<S298>/Hpgw' */
};

/* Continuous states (default storage) */
struct X_pid_control_V4_T {
  real_T Integrator_CSTATE[12];        /* '<S19>/Integrator' */
  real_T Integrator_CSTATE_f;          /* '<S270>/Integrator' */
  real_T Filter_CSTATE;                /* '<S265>/Filter' */
  real_T Integrator_CSTATE_n;          /* '<S110>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S105>/Filter' */
  real_T Integrator_CSTATE_d;          /* '<S216>/Integrator' */
  real_T Filter_CSTATE_fd;             /* '<S211>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S58>/Integrator' */
  real_T Filter_CSTATE_g;              /* '<S53>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S164>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S159>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S19>/Integrator1' */
  real_T TransferFcn_CSTATE[2];        /* '<S19>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S19>/Transfer Fcn1' */
  real_T wg_p1_CSTATE[2];              /* '<S314>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S314>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S313>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S313>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S312>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S311>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S310>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S309>/pgw_p' */
};

/* State derivatives (default storage) */
struct XDot_pid_control_V4_T {
  real_T Integrator_CSTATE[12];        /* '<S19>/Integrator' */
  real_T Integrator_CSTATE_f;          /* '<S270>/Integrator' */
  real_T Filter_CSTATE;                /* '<S265>/Filter' */
  real_T Integrator_CSTATE_n;          /* '<S110>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S105>/Filter' */
  real_T Integrator_CSTATE_d;          /* '<S216>/Integrator' */
  real_T Filter_CSTATE_fd;             /* '<S211>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S58>/Integrator' */
  real_T Filter_CSTATE_g;              /* '<S53>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S164>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S159>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S19>/Integrator1' */
  real_T TransferFcn_CSTATE[2];        /* '<S19>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S19>/Transfer Fcn1' */
  real_T wg_p1_CSTATE[2];              /* '<S314>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S314>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S313>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S313>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S312>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S311>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S310>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S309>/pgw_p' */
};

/* State disabled  */
struct XDis_pid_control_V4_T {
  boolean_T Integrator_CSTATE[12];     /* '<S19>/Integrator' */
  boolean_T Integrator_CSTATE_f;       /* '<S270>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S265>/Filter' */
  boolean_T Integrator_CSTATE_n;       /* '<S110>/Integrator' */
  boolean_T Filter_CSTATE_f;           /* '<S105>/Filter' */
  boolean_T Integrator_CSTATE_d;       /* '<S216>/Integrator' */
  boolean_T Filter_CSTATE_fd;          /* '<S211>/Filter' */
  boolean_T Integrator_CSTATE_m;       /* '<S58>/Integrator' */
  boolean_T Filter_CSTATE_g;           /* '<S53>/Filter' */
  boolean_T Integrator_CSTATE_p;       /* '<S164>/Integrator' */
  boolean_T Filter_CSTATE_m;           /* '<S159>/Filter' */
  boolean_T Integrator1_CSTATE;        /* '<S19>/Integrator1' */
  boolean_T TransferFcn_CSTATE[2];     /* '<S19>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE;       /* '<S19>/Transfer Fcn1' */
  boolean_T wg_p1_CSTATE[2];           /* '<S314>/wg_p1' */
  boolean_T wg_p2_CSTATE[2];           /* '<S314>/wg_p2' */
  boolean_T vg_p1_CSTATE[2];           /* '<S313>/vg_p1' */
  boolean_T vgw_p2_CSTATE[2];          /* '<S313>/vgw_p2' */
  boolean_T ug_p_CSTATE[2];            /* '<S312>/ug_p' */
  boolean_T rgw_p_CSTATE[2];           /* '<S311>/rgw_p' */
  boolean_T qgw_p_CSTATE[2];           /* '<S310>/qgw_p' */
  boolean_T pgw_p_CSTATE[2];           /* '<S309>/pgw_p' */
};

/* Zero-crossing (trigger) state */
struct PrevZCX_pid_control_V4_T {
  ZCSigState Integrator_Reset_ZCE;     /* '<S19>/Integrator' */
};

/* Invariant block signals (default storage) */
struct ConstB_pid_control_V4_T {
  real_T UnitConversion;               /* '<S297>/Unit Conversion' */
  real_T UnitConversion_k;             /* '<S307>/Unit Conversion' */
  real_T sigma_wg;                     /* '<S316>/sigma_wg ' */
  real_T UnitConversion_n;             /* '<S301>/Unit Conversion' */
  real_T UnitConversion_c;             /* '<S335>/Unit Conversion' */
  real_T PreLookUpIndexSearchprobofe;
                        /* '<S315>/PreLook-Up Index Search  (prob of exceed)' */
  real_T Sqrt[4];                      /* '<S308>/Sqrt' */
  real_T Sqrt1;                        /* '<S308>/Sqrt1' */
  real_T Divide[4];                    /* '<S308>/Divide' */
  real_T motorspeed;                   /* '<S19>/Gain2' */
  real_T Sum;                          /* '<S325>/Sum' */
  real_T Sum_a;                        /* '<S317>/Sum' */
  real_T sqrt_a;                       /* '<S314>/sqrt' */
  real_T w4;                           /* '<S309>/w4' */
  real_T u16;                          /* '<S309>/u^1//6' */
  uint32_T PreLookUpIndexSearchprobo_g;
                        /* '<S315>/PreLook-Up Index Search  (prob of exceed)' */
};

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
struct ODE4_IntgData {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
};

#endif

/* Constant parameters (default storage) */
struct ConstP_pid_control_V4_T {
  /* Pooled Parameter (Expression: x_nom)
   * Referenced by:
   *   '<S19>/IC'
   *   '<S19>/Memory2'
   */
  real_T pooled10[12];

  /* Expression: h_vec
   * Referenced by: '<S315>/PreLook-Up Index Search  (altitude)'
   */
  real_T PreLookUpIndexSearchaltitude_Br[12];

  /* Expression: sigma_vec'
   * Referenced by: '<S315>/Medium//High Altitude Intensity'
   */
  real_T MediumHighAltitudeIntensity_Tab[84];

  /* Computed Parameter: MediumHighAltitudeIntensity_max
   * Referenced by: '<S315>/Medium//High Altitude Intensity'
   */
  uint32_T MediumHighAltitudeIntensity_max[2];
};

/* Real-time Model Data Structure */
struct tag_RTM_pid_control_V4_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_pid_control_V4_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_pid_control_V4_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[42];
  real_T odeF[4][42];
  ODE4_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    boolean_T firstInitCondFlag;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

extern const ConstB_pid_control_V4_T pid_control_V4_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_pid_control_V4_T pid_control_V4_ConstP;

/* Class declaration for model pid_control_V4 */
class pid_control_V4
{
  /* public data and function members */
 public:
  /* Real-Time Model get method */
  RT_MODEL_pid_control_V4_T * getRTM();
  void ModelPrevZCStateInit();

  /* model start function */
  void start();

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  pid_control_V4();

  /* Destructor */
  ~pid_control_V4();

  /* private data and function members */
 private:
  /* Block signals */
  B_pid_control_V4_T pid_control_V4_B;

  /* Block states */
  DW_pid_control_V4_T pid_control_V4_DW;

  /* Block continuous states */
  X_pid_control_V4_T pid_control_V4_X;

  /* Block Continuous state disabled vector */
  XDis_pid_control_V4_T pid_control_V4_XDis;

  /* Triggered events */
  PrevZCX_pid_control_V4_T pid_control_V4_PrevZCX;

  /* private member function(s) for subsystem '<Root>/MATLAB Function'*/
  static void pid_control_V4_MATLABFunction(B_MATLABFunction_pid_control__T
    *localB);

  /* private member function(s) for subsystem '<S293>/Enabled Subsystem'*/
  static void pid_contr_EnabledSubsystem_Init(B_EnabledSubsystem_pid_contro_T
    *localB);
  static void pid_control_V4_EnabledSubsystem(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Bool *rtu_In1, B_EnabledSubsystem_pid_contro_T *localB);

  /* private member function(s) for subsystem '<S294>/Enabled Subsystem'*/
  static void pid_con_EnabledSubsystem_i_Init(B_EnabledSubsystem_pid_cont_d_T
    *localB);
  static void pid_control__EnabledSubsystem_k(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Float64 *rtu_In1, B_EnabledSubsystem_pid_cont_d_T *localB);

  /* private member function(s) for subsystem '<Root>'*/
  void pid_control_Publisher_setupImpl(const ros_slros2_internal_block_Pub_T
    *obj);
  rmw_qos_profile_t pi_ROS2PubSubBase_setQOSProfile(rmw_qos_profile_t rmwProfile,
    real_T qosDepth, real_T qosDeadline, real_T qosLifespan, real_T
    qosLeaseDuration, boolean_T qosAvoidROSNamespaceConventions);
  void pid_c_Subscriber_setupImpl_lfjn(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid__Subscriber_setupImpl_lfjny(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_con_ServiceCaller_setupImpl(const ros_slros2_internal_block_Ser_T
    *obj);
  void pid_co_Subscriber_setupImpl_lfj(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_contro_Subscriber_setupImpl(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_cont_Subscriber_setupImpl_l(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_con_Subscriber_setupImpl_lf(const ros_slros2_internal_block_Sub_T
    *obj);

  /* Global mass matrix */

  /* Continuous states update member function*/
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  /* Derivatives member function */
  void pid_control_V4_derivatives();

  /* Real-Time Model */
  RT_MODEL_pid_control_V4_T pid_control_V4_M;
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Display' : Unused code path elimination
 * Block '<S290>/Cast' : Eliminate redundant data type conversion
 * Block '<S290>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S290>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S290>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S290>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S290>/Cast To Double4' : Eliminate redundant data type conversion
 * Block '<S321>/Reshape' : Reshape block reduction
 * Block '<S321>/Reshape1' : Reshape block reduction
 * Block '<S323>/Reshape' : Reshape block reduction
 * Block '<S329>/Reshape' : Reshape block reduction
 * Block '<S329>/Reshape1' : Reshape block reduction
 * Block '<S331>/Reshape' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'pid_control_V4'
 * '<S1>'   : 'pid_control_V4/Blank Message'
 * '<S2>'   : 'pid_control_V4/Blank Message1'
 * '<S3>'   : 'pid_control_V4/Blank Message2'
 * '<S4>'   : 'pid_control_V4/Call Service'
 * '<S5>'   : 'pid_control_V4/MATLAB Function'
 * '<S6>'   : 'pid_control_V4/MATLAB Function-IMU2'
 * '<S7>'   : 'pid_control_V4/MATLAB Function-Odometry'
 * '<S8>'   : 'pid_control_V4/MATLAB Function-Odometry1'
 * '<S9>'   : 'pid_control_V4/MATLAB Function-ned_to_gazebo'
 * '<S10>'  : 'pid_control_V4/MATLAB Function1'
 * '<S11>'  : 'pid_control_V4/PID ALERON'
 * '<S12>'  : 'pid_control_V4/PID ALTURA'
 * '<S13>'  : 'pid_control_V4/PID ELEVATOR'
 * '<S14>'  : 'pid_control_V4/PID TIIMON'
 * '<S15>'  : 'pid_control_V4/PID VELOCIDAD'
 * '<S16>'  : 'pid_control_V4/Publish'
 * '<S17>'  : 'pid_control_V4/Publish1'
 * '<S18>'  : 'pid_control_V4/Ramp'
 * '<S19>'  : 'pid_control_V4/SUBSYSTEM_MODEL'
 * '<S20>'  : 'pid_control_V4/Subscribe-ALTURA'
 * '<S21>'  : 'pid_control_V4/Subscribe-YAW'
 * '<S22>'  : 'pid_control_V4/PID ALERON/Anti-windup'
 * '<S23>'  : 'pid_control_V4/PID ALERON/D Gain'
 * '<S24>'  : 'pid_control_V4/PID ALERON/External Derivative'
 * '<S25>'  : 'pid_control_V4/PID ALERON/Filter'
 * '<S26>'  : 'pid_control_V4/PID ALERON/Filter ICs'
 * '<S27>'  : 'pid_control_V4/PID ALERON/I Gain'
 * '<S28>'  : 'pid_control_V4/PID ALERON/Ideal P Gain'
 * '<S29>'  : 'pid_control_V4/PID ALERON/Ideal P Gain Fdbk'
 * '<S30>'  : 'pid_control_V4/PID ALERON/Integrator'
 * '<S31>'  : 'pid_control_V4/PID ALERON/Integrator ICs'
 * '<S32>'  : 'pid_control_V4/PID ALERON/N Copy'
 * '<S33>'  : 'pid_control_V4/PID ALERON/N Gain'
 * '<S34>'  : 'pid_control_V4/PID ALERON/P Copy'
 * '<S35>'  : 'pid_control_V4/PID ALERON/Parallel P Gain'
 * '<S36>'  : 'pid_control_V4/PID ALERON/Reset Signal'
 * '<S37>'  : 'pid_control_V4/PID ALERON/Saturation'
 * '<S38>'  : 'pid_control_V4/PID ALERON/Saturation Fdbk'
 * '<S39>'  : 'pid_control_V4/PID ALERON/Sum'
 * '<S40>'  : 'pid_control_V4/PID ALERON/Sum Fdbk'
 * '<S41>'  : 'pid_control_V4/PID ALERON/Tracking Mode'
 * '<S42>'  : 'pid_control_V4/PID ALERON/Tracking Mode Sum'
 * '<S43>'  : 'pid_control_V4/PID ALERON/Tsamp - Integral'
 * '<S44>'  : 'pid_control_V4/PID ALERON/Tsamp - Ngain'
 * '<S45>'  : 'pid_control_V4/PID ALERON/postSat Signal'
 * '<S46>'  : 'pid_control_V4/PID ALERON/preInt Signal'
 * '<S47>'  : 'pid_control_V4/PID ALERON/preSat Signal'
 * '<S48>'  : 'pid_control_V4/PID ALERON/Anti-windup/Cont. Clamping Parallel'
 * '<S49>'  : 'pid_control_V4/PID ALERON/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S50>'  : 'pid_control_V4/PID ALERON/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S51>'  : 'pid_control_V4/PID ALERON/D Gain/Internal Parameters'
 * '<S52>'  : 'pid_control_V4/PID ALERON/External Derivative/Error'
 * '<S53>'  : 'pid_control_V4/PID ALERON/Filter/Cont. Filter'
 * '<S54>'  : 'pid_control_V4/PID ALERON/Filter ICs/Internal IC - Filter'
 * '<S55>'  : 'pid_control_V4/PID ALERON/I Gain/Internal Parameters'
 * '<S56>'  : 'pid_control_V4/PID ALERON/Ideal P Gain/Passthrough'
 * '<S57>'  : 'pid_control_V4/PID ALERON/Ideal P Gain Fdbk/Disabled'
 * '<S58>'  : 'pid_control_V4/PID ALERON/Integrator/Continuous'
 * '<S59>'  : 'pid_control_V4/PID ALERON/Integrator ICs/Internal IC'
 * '<S60>'  : 'pid_control_V4/PID ALERON/N Copy/Disabled'
 * '<S61>'  : 'pid_control_V4/PID ALERON/N Gain/Internal Parameters'
 * '<S62>'  : 'pid_control_V4/PID ALERON/P Copy/Disabled'
 * '<S63>'  : 'pid_control_V4/PID ALERON/Parallel P Gain/Internal Parameters'
 * '<S64>'  : 'pid_control_V4/PID ALERON/Reset Signal/Disabled'
 * '<S65>'  : 'pid_control_V4/PID ALERON/Saturation/Enabled'
 * '<S66>'  : 'pid_control_V4/PID ALERON/Saturation Fdbk/Disabled'
 * '<S67>'  : 'pid_control_V4/PID ALERON/Sum/Sum_PID'
 * '<S68>'  : 'pid_control_V4/PID ALERON/Sum Fdbk/Disabled'
 * '<S69>'  : 'pid_control_V4/PID ALERON/Tracking Mode/Disabled'
 * '<S70>'  : 'pid_control_V4/PID ALERON/Tracking Mode Sum/Passthrough'
 * '<S71>'  : 'pid_control_V4/PID ALERON/Tsamp - Integral/TsSignalSpecification'
 * '<S72>'  : 'pid_control_V4/PID ALERON/Tsamp - Ngain/Passthrough'
 * '<S73>'  : 'pid_control_V4/PID ALERON/postSat Signal/Forward_Path'
 * '<S74>'  : 'pid_control_V4/PID ALERON/preInt Signal/Internal PreInt'
 * '<S75>'  : 'pid_control_V4/PID ALERON/preSat Signal/Forward_Path'
 * '<S76>'  : 'pid_control_V4/PID ALTURA/Anti-windup'
 * '<S77>'  : 'pid_control_V4/PID ALTURA/D Gain'
 * '<S78>'  : 'pid_control_V4/PID ALTURA/External Derivative'
 * '<S79>'  : 'pid_control_V4/PID ALTURA/Filter'
 * '<S80>'  : 'pid_control_V4/PID ALTURA/Filter ICs'
 * '<S81>'  : 'pid_control_V4/PID ALTURA/I Gain'
 * '<S82>'  : 'pid_control_V4/PID ALTURA/Ideal P Gain'
 * '<S83>'  : 'pid_control_V4/PID ALTURA/Ideal P Gain Fdbk'
 * '<S84>'  : 'pid_control_V4/PID ALTURA/Integrator'
 * '<S85>'  : 'pid_control_V4/PID ALTURA/Integrator ICs'
 * '<S86>'  : 'pid_control_V4/PID ALTURA/N Copy'
 * '<S87>'  : 'pid_control_V4/PID ALTURA/N Gain'
 * '<S88>'  : 'pid_control_V4/PID ALTURA/P Copy'
 * '<S89>'  : 'pid_control_V4/PID ALTURA/Parallel P Gain'
 * '<S90>'  : 'pid_control_V4/PID ALTURA/Reset Signal'
 * '<S91>'  : 'pid_control_V4/PID ALTURA/Saturation'
 * '<S92>'  : 'pid_control_V4/PID ALTURA/Saturation Fdbk'
 * '<S93>'  : 'pid_control_V4/PID ALTURA/Sum'
 * '<S94>'  : 'pid_control_V4/PID ALTURA/Sum Fdbk'
 * '<S95>'  : 'pid_control_V4/PID ALTURA/Tracking Mode'
 * '<S96>'  : 'pid_control_V4/PID ALTURA/Tracking Mode Sum'
 * '<S97>'  : 'pid_control_V4/PID ALTURA/Tsamp - Integral'
 * '<S98>'  : 'pid_control_V4/PID ALTURA/Tsamp - Ngain'
 * '<S99>'  : 'pid_control_V4/PID ALTURA/postSat Signal'
 * '<S100>' : 'pid_control_V4/PID ALTURA/preInt Signal'
 * '<S101>' : 'pid_control_V4/PID ALTURA/preSat Signal'
 * '<S102>' : 'pid_control_V4/PID ALTURA/Anti-windup/Back Calculation'
 * '<S103>' : 'pid_control_V4/PID ALTURA/D Gain/Internal Parameters'
 * '<S104>' : 'pid_control_V4/PID ALTURA/External Derivative/Error'
 * '<S105>' : 'pid_control_V4/PID ALTURA/Filter/Cont. Filter'
 * '<S106>' : 'pid_control_V4/PID ALTURA/Filter ICs/Internal IC - Filter'
 * '<S107>' : 'pid_control_V4/PID ALTURA/I Gain/Internal Parameters'
 * '<S108>' : 'pid_control_V4/PID ALTURA/Ideal P Gain/Passthrough'
 * '<S109>' : 'pid_control_V4/PID ALTURA/Ideal P Gain Fdbk/Disabled'
 * '<S110>' : 'pid_control_V4/PID ALTURA/Integrator/Continuous'
 * '<S111>' : 'pid_control_V4/PID ALTURA/Integrator ICs/Internal IC'
 * '<S112>' : 'pid_control_V4/PID ALTURA/N Copy/Disabled'
 * '<S113>' : 'pid_control_V4/PID ALTURA/N Gain/Internal Parameters'
 * '<S114>' : 'pid_control_V4/PID ALTURA/P Copy/Disabled'
 * '<S115>' : 'pid_control_V4/PID ALTURA/Parallel P Gain/Internal Parameters'
 * '<S116>' : 'pid_control_V4/PID ALTURA/Reset Signal/Disabled'
 * '<S117>' : 'pid_control_V4/PID ALTURA/Saturation/Enabled'
 * '<S118>' : 'pid_control_V4/PID ALTURA/Saturation Fdbk/Disabled'
 * '<S119>' : 'pid_control_V4/PID ALTURA/Sum/Sum_PID'
 * '<S120>' : 'pid_control_V4/PID ALTURA/Sum Fdbk/Disabled'
 * '<S121>' : 'pid_control_V4/PID ALTURA/Tracking Mode/Disabled'
 * '<S122>' : 'pid_control_V4/PID ALTURA/Tracking Mode Sum/Passthrough'
 * '<S123>' : 'pid_control_V4/PID ALTURA/Tsamp - Integral/TsSignalSpecification'
 * '<S124>' : 'pid_control_V4/PID ALTURA/Tsamp - Ngain/Passthrough'
 * '<S125>' : 'pid_control_V4/PID ALTURA/postSat Signal/Forward_Path'
 * '<S126>' : 'pid_control_V4/PID ALTURA/preInt Signal/Internal PreInt'
 * '<S127>' : 'pid_control_V4/PID ALTURA/preSat Signal/Forward_Path'
 * '<S128>' : 'pid_control_V4/PID ELEVATOR/Anti-windup'
 * '<S129>' : 'pid_control_V4/PID ELEVATOR/D Gain'
 * '<S130>' : 'pid_control_V4/PID ELEVATOR/External Derivative'
 * '<S131>' : 'pid_control_V4/PID ELEVATOR/Filter'
 * '<S132>' : 'pid_control_V4/PID ELEVATOR/Filter ICs'
 * '<S133>' : 'pid_control_V4/PID ELEVATOR/I Gain'
 * '<S134>' : 'pid_control_V4/PID ELEVATOR/Ideal P Gain'
 * '<S135>' : 'pid_control_V4/PID ELEVATOR/Ideal P Gain Fdbk'
 * '<S136>' : 'pid_control_V4/PID ELEVATOR/Integrator'
 * '<S137>' : 'pid_control_V4/PID ELEVATOR/Integrator ICs'
 * '<S138>' : 'pid_control_V4/PID ELEVATOR/N Copy'
 * '<S139>' : 'pid_control_V4/PID ELEVATOR/N Gain'
 * '<S140>' : 'pid_control_V4/PID ELEVATOR/P Copy'
 * '<S141>' : 'pid_control_V4/PID ELEVATOR/Parallel P Gain'
 * '<S142>' : 'pid_control_V4/PID ELEVATOR/Reset Signal'
 * '<S143>' : 'pid_control_V4/PID ELEVATOR/Saturation'
 * '<S144>' : 'pid_control_V4/PID ELEVATOR/Saturation Fdbk'
 * '<S145>' : 'pid_control_V4/PID ELEVATOR/Sum'
 * '<S146>' : 'pid_control_V4/PID ELEVATOR/Sum Fdbk'
 * '<S147>' : 'pid_control_V4/PID ELEVATOR/Tracking Mode'
 * '<S148>' : 'pid_control_V4/PID ELEVATOR/Tracking Mode Sum'
 * '<S149>' : 'pid_control_V4/PID ELEVATOR/Tsamp - Integral'
 * '<S150>' : 'pid_control_V4/PID ELEVATOR/Tsamp - Ngain'
 * '<S151>' : 'pid_control_V4/PID ELEVATOR/postSat Signal'
 * '<S152>' : 'pid_control_V4/PID ELEVATOR/preInt Signal'
 * '<S153>' : 'pid_control_V4/PID ELEVATOR/preSat Signal'
 * '<S154>' : 'pid_control_V4/PID ELEVATOR/Anti-windup/Cont. Clamping Parallel'
 * '<S155>' : 'pid_control_V4/PID ELEVATOR/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S156>' : 'pid_control_V4/PID ELEVATOR/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S157>' : 'pid_control_V4/PID ELEVATOR/D Gain/Internal Parameters'
 * '<S158>' : 'pid_control_V4/PID ELEVATOR/External Derivative/Error'
 * '<S159>' : 'pid_control_V4/PID ELEVATOR/Filter/Cont. Filter'
 * '<S160>' : 'pid_control_V4/PID ELEVATOR/Filter ICs/Internal IC - Filter'
 * '<S161>' : 'pid_control_V4/PID ELEVATOR/I Gain/Internal Parameters'
 * '<S162>' : 'pid_control_V4/PID ELEVATOR/Ideal P Gain/Passthrough'
 * '<S163>' : 'pid_control_V4/PID ELEVATOR/Ideal P Gain Fdbk/Disabled'
 * '<S164>' : 'pid_control_V4/PID ELEVATOR/Integrator/Continuous'
 * '<S165>' : 'pid_control_V4/PID ELEVATOR/Integrator ICs/Internal IC'
 * '<S166>' : 'pid_control_V4/PID ELEVATOR/N Copy/Disabled'
 * '<S167>' : 'pid_control_V4/PID ELEVATOR/N Gain/Internal Parameters'
 * '<S168>' : 'pid_control_V4/PID ELEVATOR/P Copy/Disabled'
 * '<S169>' : 'pid_control_V4/PID ELEVATOR/Parallel P Gain/Internal Parameters'
 * '<S170>' : 'pid_control_V4/PID ELEVATOR/Reset Signal/Disabled'
 * '<S171>' : 'pid_control_V4/PID ELEVATOR/Saturation/Enabled'
 * '<S172>' : 'pid_control_V4/PID ELEVATOR/Saturation Fdbk/Disabled'
 * '<S173>' : 'pid_control_V4/PID ELEVATOR/Sum/Sum_PID'
 * '<S174>' : 'pid_control_V4/PID ELEVATOR/Sum Fdbk/Disabled'
 * '<S175>' : 'pid_control_V4/PID ELEVATOR/Tracking Mode/Disabled'
 * '<S176>' : 'pid_control_V4/PID ELEVATOR/Tracking Mode Sum/Passthrough'
 * '<S177>' : 'pid_control_V4/PID ELEVATOR/Tsamp - Integral/TsSignalSpecification'
 * '<S178>' : 'pid_control_V4/PID ELEVATOR/Tsamp - Ngain/Passthrough'
 * '<S179>' : 'pid_control_V4/PID ELEVATOR/postSat Signal/Forward_Path'
 * '<S180>' : 'pid_control_V4/PID ELEVATOR/preInt Signal/Internal PreInt'
 * '<S181>' : 'pid_control_V4/PID ELEVATOR/preSat Signal/Forward_Path'
 * '<S182>' : 'pid_control_V4/PID TIIMON/Anti-windup'
 * '<S183>' : 'pid_control_V4/PID TIIMON/D Gain'
 * '<S184>' : 'pid_control_V4/PID TIIMON/External Derivative'
 * '<S185>' : 'pid_control_V4/PID TIIMON/Filter'
 * '<S186>' : 'pid_control_V4/PID TIIMON/Filter ICs'
 * '<S187>' : 'pid_control_V4/PID TIIMON/I Gain'
 * '<S188>' : 'pid_control_V4/PID TIIMON/Ideal P Gain'
 * '<S189>' : 'pid_control_V4/PID TIIMON/Ideal P Gain Fdbk'
 * '<S190>' : 'pid_control_V4/PID TIIMON/Integrator'
 * '<S191>' : 'pid_control_V4/PID TIIMON/Integrator ICs'
 * '<S192>' : 'pid_control_V4/PID TIIMON/N Copy'
 * '<S193>' : 'pid_control_V4/PID TIIMON/N Gain'
 * '<S194>' : 'pid_control_V4/PID TIIMON/P Copy'
 * '<S195>' : 'pid_control_V4/PID TIIMON/Parallel P Gain'
 * '<S196>' : 'pid_control_V4/PID TIIMON/Reset Signal'
 * '<S197>' : 'pid_control_V4/PID TIIMON/Saturation'
 * '<S198>' : 'pid_control_V4/PID TIIMON/Saturation Fdbk'
 * '<S199>' : 'pid_control_V4/PID TIIMON/Sum'
 * '<S200>' : 'pid_control_V4/PID TIIMON/Sum Fdbk'
 * '<S201>' : 'pid_control_V4/PID TIIMON/Tracking Mode'
 * '<S202>' : 'pid_control_V4/PID TIIMON/Tracking Mode Sum'
 * '<S203>' : 'pid_control_V4/PID TIIMON/Tsamp - Integral'
 * '<S204>' : 'pid_control_V4/PID TIIMON/Tsamp - Ngain'
 * '<S205>' : 'pid_control_V4/PID TIIMON/postSat Signal'
 * '<S206>' : 'pid_control_V4/PID TIIMON/preInt Signal'
 * '<S207>' : 'pid_control_V4/PID TIIMON/preSat Signal'
 * '<S208>' : 'pid_control_V4/PID TIIMON/Anti-windup/Back Calculation'
 * '<S209>' : 'pid_control_V4/PID TIIMON/D Gain/Internal Parameters'
 * '<S210>' : 'pid_control_V4/PID TIIMON/External Derivative/Error'
 * '<S211>' : 'pid_control_V4/PID TIIMON/Filter/Cont. Filter'
 * '<S212>' : 'pid_control_V4/PID TIIMON/Filter ICs/Internal IC - Filter'
 * '<S213>' : 'pid_control_V4/PID TIIMON/I Gain/Internal Parameters'
 * '<S214>' : 'pid_control_V4/PID TIIMON/Ideal P Gain/Passthrough'
 * '<S215>' : 'pid_control_V4/PID TIIMON/Ideal P Gain Fdbk/Disabled'
 * '<S216>' : 'pid_control_V4/PID TIIMON/Integrator/Continuous'
 * '<S217>' : 'pid_control_V4/PID TIIMON/Integrator ICs/Internal IC'
 * '<S218>' : 'pid_control_V4/PID TIIMON/N Copy/Disabled'
 * '<S219>' : 'pid_control_V4/PID TIIMON/N Gain/Internal Parameters'
 * '<S220>' : 'pid_control_V4/PID TIIMON/P Copy/Disabled'
 * '<S221>' : 'pid_control_V4/PID TIIMON/Parallel P Gain/Internal Parameters'
 * '<S222>' : 'pid_control_V4/PID TIIMON/Reset Signal/Disabled'
 * '<S223>' : 'pid_control_V4/PID TIIMON/Saturation/Enabled'
 * '<S224>' : 'pid_control_V4/PID TIIMON/Saturation Fdbk/Disabled'
 * '<S225>' : 'pid_control_V4/PID TIIMON/Sum/Sum_PID'
 * '<S226>' : 'pid_control_V4/PID TIIMON/Sum Fdbk/Disabled'
 * '<S227>' : 'pid_control_V4/PID TIIMON/Tracking Mode/Disabled'
 * '<S228>' : 'pid_control_V4/PID TIIMON/Tracking Mode Sum/Passthrough'
 * '<S229>' : 'pid_control_V4/PID TIIMON/Tsamp - Integral/TsSignalSpecification'
 * '<S230>' : 'pid_control_V4/PID TIIMON/Tsamp - Ngain/Passthrough'
 * '<S231>' : 'pid_control_V4/PID TIIMON/postSat Signal/Forward_Path'
 * '<S232>' : 'pid_control_V4/PID TIIMON/preInt Signal/Internal PreInt'
 * '<S233>' : 'pid_control_V4/PID TIIMON/preSat Signal/Forward_Path'
 * '<S234>' : 'pid_control_V4/PID VELOCIDAD/Anti-windup'
 * '<S235>' : 'pid_control_V4/PID VELOCIDAD/D Gain'
 * '<S236>' : 'pid_control_V4/PID VELOCIDAD/External Derivative'
 * '<S237>' : 'pid_control_V4/PID VELOCIDAD/Filter'
 * '<S238>' : 'pid_control_V4/PID VELOCIDAD/Filter ICs'
 * '<S239>' : 'pid_control_V4/PID VELOCIDAD/I Gain'
 * '<S240>' : 'pid_control_V4/PID VELOCIDAD/Ideal P Gain'
 * '<S241>' : 'pid_control_V4/PID VELOCIDAD/Ideal P Gain Fdbk'
 * '<S242>' : 'pid_control_V4/PID VELOCIDAD/Integrator'
 * '<S243>' : 'pid_control_V4/PID VELOCIDAD/Integrator ICs'
 * '<S244>' : 'pid_control_V4/PID VELOCIDAD/N Copy'
 * '<S245>' : 'pid_control_V4/PID VELOCIDAD/N Gain'
 * '<S246>' : 'pid_control_V4/PID VELOCIDAD/P Copy'
 * '<S247>' : 'pid_control_V4/PID VELOCIDAD/Parallel P Gain'
 * '<S248>' : 'pid_control_V4/PID VELOCIDAD/Reset Signal'
 * '<S249>' : 'pid_control_V4/PID VELOCIDAD/Saturation'
 * '<S250>' : 'pid_control_V4/PID VELOCIDAD/Saturation Fdbk'
 * '<S251>' : 'pid_control_V4/PID VELOCIDAD/Sum'
 * '<S252>' : 'pid_control_V4/PID VELOCIDAD/Sum Fdbk'
 * '<S253>' : 'pid_control_V4/PID VELOCIDAD/Tracking Mode'
 * '<S254>' : 'pid_control_V4/PID VELOCIDAD/Tracking Mode Sum'
 * '<S255>' : 'pid_control_V4/PID VELOCIDAD/Tsamp - Integral'
 * '<S256>' : 'pid_control_V4/PID VELOCIDAD/Tsamp - Ngain'
 * '<S257>' : 'pid_control_V4/PID VELOCIDAD/postSat Signal'
 * '<S258>' : 'pid_control_V4/PID VELOCIDAD/preInt Signal'
 * '<S259>' : 'pid_control_V4/PID VELOCIDAD/preSat Signal'
 * '<S260>' : 'pid_control_V4/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel'
 * '<S261>' : 'pid_control_V4/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S262>' : 'pid_control_V4/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S263>' : 'pid_control_V4/PID VELOCIDAD/D Gain/Internal Parameters'
 * '<S264>' : 'pid_control_V4/PID VELOCIDAD/External Derivative/Error'
 * '<S265>' : 'pid_control_V4/PID VELOCIDAD/Filter/Cont. Filter'
 * '<S266>' : 'pid_control_V4/PID VELOCIDAD/Filter ICs/Internal IC - Filter'
 * '<S267>' : 'pid_control_V4/PID VELOCIDAD/I Gain/Internal Parameters'
 * '<S268>' : 'pid_control_V4/PID VELOCIDAD/Ideal P Gain/Passthrough'
 * '<S269>' : 'pid_control_V4/PID VELOCIDAD/Ideal P Gain Fdbk/Disabled'
 * '<S270>' : 'pid_control_V4/PID VELOCIDAD/Integrator/Continuous'
 * '<S271>' : 'pid_control_V4/PID VELOCIDAD/Integrator ICs/Internal IC'
 * '<S272>' : 'pid_control_V4/PID VELOCIDAD/N Copy/Disabled'
 * '<S273>' : 'pid_control_V4/PID VELOCIDAD/N Gain/Internal Parameters'
 * '<S274>' : 'pid_control_V4/PID VELOCIDAD/P Copy/Disabled'
 * '<S275>' : 'pid_control_V4/PID VELOCIDAD/Parallel P Gain/Internal Parameters'
 * '<S276>' : 'pid_control_V4/PID VELOCIDAD/Reset Signal/Disabled'
 * '<S277>' : 'pid_control_V4/PID VELOCIDAD/Saturation/Enabled'
 * '<S278>' : 'pid_control_V4/PID VELOCIDAD/Saturation Fdbk/Disabled'
 * '<S279>' : 'pid_control_V4/PID VELOCIDAD/Sum/Sum_PID'
 * '<S280>' : 'pid_control_V4/PID VELOCIDAD/Sum Fdbk/Disabled'
 * '<S281>' : 'pid_control_V4/PID VELOCIDAD/Tracking Mode/Disabled'
 * '<S282>' : 'pid_control_V4/PID VELOCIDAD/Tracking Mode Sum/Passthrough'
 * '<S283>' : 'pid_control_V4/PID VELOCIDAD/Tsamp - Integral/TsSignalSpecification'
 * '<S284>' : 'pid_control_V4/PID VELOCIDAD/Tsamp - Ngain/Passthrough'
 * '<S285>' : 'pid_control_V4/PID VELOCIDAD/postSat Signal/Forward_Path'
 * '<S286>' : 'pid_control_V4/PID VELOCIDAD/preInt Signal/Internal PreInt'
 * '<S287>' : 'pid_control_V4/PID VELOCIDAD/preSat Signal/Forward_Path'
 * '<S288>' : 'pid_control_V4/SUBSYSTEM_MODEL/Band-Limited White Noise'
 * '<S289>' : 'pid_control_V4/SUBSYSTEM_MODEL/Compare To Constant'
 * '<S290>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))'
 * '<S291>' : 'pid_control_V4/SUBSYSTEM_MODEL/MATLAB Function - MODEL'
 * '<S292>' : 'pid_control_V4/SUBSYSTEM_MODEL/MATLAB Function-reset'
 * '<S293>' : 'pid_control_V4/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1'
 * '<S294>' : 'pid_control_V4/SUBSYSTEM_MODEL/Subscribe_HEAVE'
 * '<S295>' : 'pid_control_V4/SUBSYSTEM_MODEL/Subscribe_RATE'
 * '<S296>' : 'pid_control_V4/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA'
 * '<S297>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Angle Conversion'
 * '<S298>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates'
 * '<S299>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities'
 * '<S300>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Length Conversion'
 * '<S301>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Length Conversion1'
 * '<S302>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities'
 * '<S303>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates'
 * '<S304>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities'
 * '<S305>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths'
 * '<S306>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Velocity Conversion'
 * '<S307>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Velocity Conversion2'
 * '<S308>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/White Noise'
 * '<S309>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hpgw'
 * '<S310>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hqgw'
 * '<S311>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hrgw'
 * '<S312>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hugw(s)'
 * '<S313>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hvgw(s)'
 * '<S314>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hwgw(s)'
 * '<S315>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities/High Altitude Intensity'
 * '<S316>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities/Low Altitude Intensity'
 * '<S317>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates'
 * '<S318>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates'
 * '<S319>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Medium//High  altitude rates'
 * '<S320>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Merge Subsystems'
 * '<S321>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation'
 * '<S322>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation/convert to earth coords'
 * '<S323>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation'
 * '<S324>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation/convert to earth coords'
 * '<S325>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities'
 * '<S326>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities'
 * '<S327>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Medium//High  altitude velocities'
 * '<S328>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Merge Subsystems'
 * '<S329>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation'
 * '<S330>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation/convert to earth coords'
 * '<S331>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation'
 * '<S332>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation/convert to earth coords'
 * '<S333>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Low altitude scale length'
 * '<S334>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Medium//High altitude scale length'
 * '<S335>' : 'pid_control_V4/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Medium//High altitude scale length/Length Conversion'
 * '<S336>' : 'pid_control_V4/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1/Enabled Subsystem'
 * '<S337>' : 'pid_control_V4/SUBSYSTEM_MODEL/Subscribe_HEAVE/Enabled Subsystem'
 * '<S338>' : 'pid_control_V4/SUBSYSTEM_MODEL/Subscribe_RATE/Enabled Subsystem'
 * '<S339>' : 'pid_control_V4/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA/Enabled Subsystem'
 * '<S340>' : 'pid_control_V4/Subscribe-ALTURA/Enabled Subsystem'
 * '<S341>' : 'pid_control_V4/Subscribe-YAW/Enabled Subsystem'
 */
#endif                                 /* pid_control_V4_h_ */

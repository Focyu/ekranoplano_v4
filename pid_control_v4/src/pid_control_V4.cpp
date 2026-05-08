/*
 * pid_control_V4.cpp
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

#include "pid_control_V4.h"
#include "rtwtypes.h"
#include "pid_control_V4_types.h"
#include "rmw/qos_profiles.h"
#include <string.h>
#include "pid_control_V4_private.h"
#include <emmintrin.h>
#include <math.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include <stddef.h>
#include "zero_crossing_types.h"
#include "rt_defines.h"

uint32_T plook_bincpa(real_T u, const real_T bp[], uint32_T maxIndex, real_T
                      *fraction, uint32_T *prevIndex)
{
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'on'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32d_prevIdx(u, bp, *prevIndex, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex;
    *fraction = 0.0;
  }

  *prevIndex = bpIndex;
  return bpIndex;
}

real_T intrp2d_la_pw(const uint32_T bpIndex[], const real_T frac[], const real_T
                     table[], const uint32_T stride, const uint32_T maxIndex[])
{
  real_T y;
  real_T yL_0d0;
  uint32_T offset_1d;

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Overflow mode: 'portable wrapping'
   */
  offset_1d = bpIndex[1U] * stride + bpIndex[0U];
  if (bpIndex[0U] == maxIndex[0U]) {
    y = table[offset_1d];
  } else {
    yL_0d0 = table[offset_1d];
    y = (table[offset_1d + 1U] - yL_0d0) * frac[0U] + yL_0d0;
  }

  if (bpIndex[1U] == maxIndex[1U]) {
  } else {
    offset_1d += stride;
    if (bpIndex[0U] == maxIndex[0U]) {
      yL_0d0 = table[offset_1d];
    } else {
      yL_0d0 = table[offset_1d];
      yL_0d0 += (table[offset_1d + 1U] - yL_0d0) * frac[0U];
    }

    y += (yL_0d0 - y) * frac[1U];
  }

  return y;
}

uint32_T binsearch_u32d_prevIdx(real_T u, const real_T bp[], uint32_T startIndex,
  uint32_T maxIndex)
{
  uint32_T bpIndex;
  uint32_T found;
  uint32_T iLeft;
  uint32_T iRght;

  /* Binary Search using Previous Index */
  bpIndex = startIndex;
  iLeft = 0U;
  iRght = maxIndex;
  found = 0U;
  while (found == 0U) {
    if (u < bp[bpIndex]) {
      iRght = bpIndex - 1U;
      bpIndex = ((bpIndex + iLeft) - 1U) >> 1U;
    } else if (u < bp[bpIndex + 1U]) {
      found = 1U;
    } else {
      iLeft = bpIndex + 1U;
      bpIndex = ((bpIndex + iRght) + 1U) >> 1U;
    }
  }

  return bpIndex;
}

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
void pid_control_V4::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = static_cast<ODE4_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 42;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  pid_control_V4_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  this->step();
  pid_control_V4_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  this->step();
  pid_control_V4_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  this->step();
  pid_control_V4_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * Output and update for atomic system:
 *    '<Root>/MATLAB Function'
 *    '<Root>/MATLAB Function-IMU2'
 */
void pid_control_V4::pid_control_V4_MATLABFunction
  (B_MATLABFunction_pid_control__T *localB)
{
  static const uint8_T b[11] = { 101U, 107U, 114U, 97U, 110U, 111U, 112U, 108U,
    97U, 110U, 111U };

  memset(&localB->stringOut[0], 0, sizeof(uint8_T) << 7U);
  for (int32_T i = 0; i < 11; i++) {
    localB->stringOut[i] = b[i];
  }

  localB->lengthOut = 11U;
}

/*
 * System initialize for enable system:
 *    '<S293>/Enabled Subsystem'
 *    '<S296>/Enabled Subsystem'
 */
void pid_control_V4::pid_contr_EnabledSubsystem_Init
  (B_EnabledSubsystem_pid_contro_T *localB)
{
  /* SystemInitialize for SignalConversion generated from: '<S336>/In1' */
  memset(&localB->In1, 0, sizeof(SL_Bus_std_msgs_Bool));
}

/*
 * Output and update for enable system:
 *    '<S293>/Enabled Subsystem'
 *    '<S296>/Enabled Subsystem'
 */
void pid_control_V4::pid_control_V4_EnabledSubsystem(boolean_T rtu_Enable, const
  SL_Bus_std_msgs_Bool *rtu_In1, B_EnabledSubsystem_pid_contro_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S293>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S336>/Enable'
   */
  if (rtu_Enable) {
    /* SignalConversion generated from: '<S336>/In1' */
    localB->In1 = *rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S293>/Enabled Subsystem' */
}

/*
 * System initialize for enable system:
 *    '<S294>/Enabled Subsystem'
 *    '<S295>/Enabled Subsystem'
 *    '<S20>/Enabled Subsystem'
 *    '<S21>/Enabled Subsystem'
 */
void pid_control_V4::pid_con_EnabledSubsystem_i_Init
  (B_EnabledSubsystem_pid_cont_d_T *localB)
{
  /* SystemInitialize for SignalConversion generated from: '<S337>/In1' */
  memset(&localB->In1, 0, sizeof(SL_Bus_std_msgs_Float64));
}

/*
 * Output and update for enable system:
 *    '<S294>/Enabled Subsystem'
 *    '<S295>/Enabled Subsystem'
 *    '<S20>/Enabled Subsystem'
 *    '<S21>/Enabled Subsystem'
 */
void pid_control_V4::pid_control__EnabledSubsystem_k(boolean_T rtu_Enable, const
  SL_Bus_std_msgs_Float64 *rtu_In1, B_EnabledSubsystem_pid_cont_d_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S294>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S337>/Enable'
   */
  if (rtu_Enable) {
    /* SignalConversion generated from: '<S337>/In1' */
    localB->In1 = *rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S294>/Enabled Subsystem' */
}

void pid_control_V4::pid_control_Publisher_setupImpl(const
  ros_slros2_internal_block_Pub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[17] = "/ekranoplano/imu";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S16>/SinkBlock' */
  pid_control_V4_B.deadline_m.sec = 0.0;
  pid_control_V4_B.deadline_m.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V4_B.deadline_m, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S16>/SinkBlock' */
    pid_control_V4_B.b_zeroDelimTopic_p[i] = b_zeroDelimTopic[i];
  }

  Pub_pid_control_V4_548.createPublisher(&pid_control_V4_B.b_zeroDelimTopic_p[0],
    qos_profile);
}

rmw_qos_profile_t pid_control_V4::pi_ROS2PubSubBase_setQOSProfile
  (rmw_qos_profile_t rmwProfile, real_T qosDepth, real_T qosDeadline, real_T
   qosLifespan, real_T qosLeaseDuration, boolean_T
   qosAvoidROSNamespaceConventions)
{
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  real_T input;
  real_T sec;
  input = qosDeadline;

  /* Start for MATLABSystem: '<S17>/SinkBlock' */
  if (qosDeadline == (rtInf)) {
    input = 0.0;
  }

  sec = floor(input);
  pid_control_V4_B.deadline_j.sec = sec;

  /* Start for MATLABSystem: '<S17>/SinkBlock' */
  pid_control_V4_B.deadline_j.nsec = (input - sec) * 1.0E+9;
  input = qosLifespan;

  /* Start for MATLABSystem: '<S17>/SinkBlock' */
  if (qosLifespan == (rtInf)) {
    input = 0.0;
  }

  sec = floor(input);
  lifespan.sec = sec;

  /* Start for MATLABSystem: '<S17>/SinkBlock' */
  lifespan.nsec = (input - sec) * 1.0E+9;
  input = qosLeaseDuration;

  /* Start for MATLABSystem: '<S17>/SinkBlock' */
  if (qosLeaseDuration == (rtInf)) {
    input = 0.0;
  }

  sec = floor(input);
  liveliness_lease_duration.sec = sec;

  /* Start for MATLABSystem: '<S17>/SinkBlock' */
  liveliness_lease_duration.nsec = (input - sec) * 1.0E+9;
  SET_QOS_VALUES(rmwProfile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)qosDepth,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V4_B.deadline_j, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)qosAvoidROSNamespaceConventions);
  return rmwProfile;
}

void pid_control_V4::pid_c_Subscriber_setupImpl_lfjn(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[17] = "/setpoint/altura";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S20>/SourceBlock' */
  pid_control_V4_B.deadline_g1.sec = 0.0;
  pid_control_V4_B.deadline_g1.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V4_B.deadline_g1, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S20>/SourceBlock' */
    pid_control_V4_B.b_zeroDelimTopic_b[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V4_435.createSubscriber(&pid_control_V4_B.b_zeroDelimTopic_b[0],
    qos_profile);
}

void pid_control_V4::pid__Subscriber_setupImpl_lfjny(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[14];
  static const char_T b_zeroDelimTopic_0[14] = "/setpoint/yaw";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S21>/SourceBlock' */
  pid_control_V4_B.deadline_n.sec = 0.0;
  pid_control_V4_B.deadline_n.nsec = 0.0;
  pid_control_V4_B.lifespan.sec = 0.0;
  pid_control_V4_B.lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V4_B.deadline_n, pid_control_V4_B.lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 14; i++) {
    /* Start for MATLABSystem: '<S21>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_pid_control_V4_377.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
}

void pid_control_V4::pid_con_ServiceCaller_setupImpl(const
  ros_slros2_internal_block_Ser_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[25] = "/gazebo/set_entity_state";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S4>/ServiceCaller' */
  pid_control_V4_B.deadline.sec = 0.0;
  pid_control_V4_B.deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, pid_control_V4_B.deadline,
                 lifespan, RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
                 liveliness_lease_duration, (bool)
                 obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 25; i++) {
    /* Start for MATLABSystem: '<S4>/ServiceCaller' */
    pid_control_V4_B.b_zeroDelimTopic[i] = b_zeroDelimTopic[i];
  }

  ServCall_pid_control_V4_326.createServiceCaller
    (&pid_control_V4_B.b_zeroDelimTopic[0], qos_profile);
}

void pid_control_V4::pid_co_Subscriber_setupImpl_lfj(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[22] = "/setpoint/turbulencia";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S296>/SourceBlock' */
  pid_control_V4_B.deadline_f.sec = 0.0;
  pid_control_V4_B.deadline_f.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V4_B.deadline_f, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 22; i++) {
    /* Start for MATLABSystem: '<S296>/SourceBlock' */
    pid_control_V4_B.b_zeroDelimTopic_c[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V4_417.createSubscriber(&pid_control_V4_B.b_zeroDelimTopic_c[0],
    qos_profile);
}

void pid_control_V4::pid_contro_Subscriber_setupImpl(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[22] = "/setpoint/turbulencia";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S293>/SourceBlock' */
  pid_control_V4_B.deadline_c.sec = 0.0;
  pid_control_V4_B.deadline_c.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V4_B.deadline_c, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 22; i++) {
    /* Start for MATLABSystem: '<S293>/SourceBlock' */
    pid_control_V4_B.b_zeroDelimTopic_m[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V4_423.createSubscriber(&pid_control_V4_B.b_zeroDelimTopic_m[0],
    qos_profile);
}

void pid_control_V4::pid_cont_Subscriber_setupImpl_l(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[12];
  static const char_T b_zeroDelimTopic_0[12] = "/olas/heave";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S294>/SourceBlock' */
  pid_control_V4_B.deadline_p.sec = 0.0;
  pid_control_V4_B.deadline_p.nsec = 0.0;
  pid_control_V4_B.lifespan_l.sec = 0.0;
  pid_control_V4_B.lifespan_l.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V4_B.deadline_p, pid_control_V4_B.lifespan_l,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 12; i++) {
    /* Start for MATLABSystem: '<S294>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_pid_control_V4_443.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
}

void pid_control_V4::pid_con_Subscriber_setupImpl_lf(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[17] = "/olas/pitch_rate";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S295>/SourceBlock' */
  pid_control_V4_B.deadline_g.sec = 0.0;
  pid_control_V4_B.deadline_g.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V4_B.deadline_g, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S295>/SourceBlock' */
    pid_control_V4_B.b_zeroDelimTopic_cx[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V4_445.createSubscriber(&pid_control_V4_B.b_zeroDelimTopic_cx
    [0], qos_profile);
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T hi;
  uint32_T lo;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return static_cast<real_T>(*u) * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T si;
  real_T sr;
  real_T y;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/* Model step function */
void pid_control_V4::step()
{
  /* local block i/o variables */
  SL_Bus_std_msgs_Float64 rtb_SourceBlock_o2_d;
  SL_Bus_std_msgs_Bool rtb_SourceBlock_o2_j;
  SL_Bus_std_msgs_Bool rtb_SourceBlock_o2_dd;
  __m128d tmp_3;
  SL_Bus_gazebo_msgs_SetEntityStateResponse tmp;
  int32_T i;
  int32_T tmp_2;
  int8_T rtAction;
  int8_T rtPrevAction;
  boolean_T serverAvailableOnTime;
  boolean_T tmp_0;
  boolean_T tmp_1;
  static const uint8_T b[5] = { 119U, 111U, 114U, 108U, 100U };

  static const uint8_T b_0[11] = { 101U, 107U, 114U, 97U, 110U, 111U, 112U, 108U,
    97U, 110U, 111U };

  if (rtmIsMajorTimeStep((&pid_control_V4_M))) {
    /* set solver stop time */
    if (!((&pid_control_V4_M)->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&(&pid_control_V4_M)->solverInfo,
                            (((&pid_control_V4_M)->Timing.clockTickH0 + 1) *
        (&pid_control_V4_M)->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&(&pid_control_V4_M)->solverInfo,
                            (((&pid_control_V4_M)->Timing.clockTick0 + 1) *
        (&pid_control_V4_M)->Timing.stepSize0 + (&pid_control_V4_M)
        ->Timing.clockTickH0 * (&pid_control_V4_M)->Timing.stepSize0 *
        4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep((&pid_control_V4_M))) {
    (&pid_control_V4_M)->Timing.t[0] = rtsiGetT(&(&pid_control_V4_M)->solverInfo);
  }

  /* BusAssignment: '<Root>/Bus Assignment-IMU' */
  memset(&pid_control_V4_B.BusAssignmentIMU, 0, sizeof(SL_Bus_sensor_msgs_Imu));

  /* RelationalOperator: '<S289>/Compare' incorporates:
   *  Constant: '<S289>/Constant'
   */
  pid_control_V4_B.Compare = (pid_control_V4_X.Integrator_CSTATE[11] >= 0.05);

  /* Outputs for Enabled SubSystem: '<S298>/Hrgw' incorporates:
   *  EnablePort: '<S311>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S298>/Hqgw' incorporates:
   *  EnablePort: '<S310>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S298>/Hpgw' incorporates:
   *  EnablePort: '<S309>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S299>/Hwgw(s)' incorporates:
   *  EnablePort: '<S314>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S299>/Hvgw(s)' incorporates:
   *  EnablePort: '<S313>/Enable'
   */
  tmp_0 = rtmIsMajorTimeStep((&pid_control_V4_M));

  /* End of Outputs for SubSystem: '<S299>/Hvgw(s)' */
  /* End of Outputs for SubSystem: '<S299>/Hwgw(s)' */
  /* End of Outputs for SubSystem: '<S298>/Hpgw' */
  /* End of Outputs for SubSystem: '<S298>/Hqgw' */
  /* End of Outputs for SubSystem: '<S298>/Hrgw' */
  if (tmp_0) {
    /* MATLAB Function: '<S19>/MATLAB Function-reset' incorporates:
     *  Memory: '<S19>/Memory2'
     */
    memcpy(&pid_control_V4_B.IC[0], &pid_control_V4_DW.Memory2_PreviousInput[0],
           12U * sizeof(real_T));
    pid_control_V4_B.IC[2] = 0.0;
    pid_control_V4_B.IC[11] = 0.0;

    /* InitialCondition: '<S19>/IC' */
    if (pid_control_V4_DW.IC_FirstOutputTime) {
      pid_control_V4_DW.IC_FirstOutputTime = false;

      /* InitialCondition: '<S19>/IC' */
      memcpy(&pid_control_V4_B.IC[0], &pid_control_V4_ConstP.pooled10[0], 12U *
             sizeof(real_T));
    }

    /* End of InitialCondition: '<S19>/IC' */
  }

  /* Outputs for Enabled SubSystem: '<S298>/Hrgw' incorporates:
   *  EnablePort: '<S311>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S298>/Hqgw' incorporates:
   *  EnablePort: '<S310>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S298>/Hpgw' incorporates:
   *  EnablePort: '<S309>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S299>/Hwgw(s)' incorporates:
   *  EnablePort: '<S314>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S299>/Hvgw(s)' incorporates:
   *  EnablePort: '<S313>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S299>/Hugw(s)' incorporates:
   *  EnablePort: '<S312>/Enable'
   */
  /* If: '<S303>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
   *  If: '<S304>/if Height < Max low altitude  elseif Height > Min isotropic altitude '
   *  Integrator: '<S19>/Integrator'
   *  RateLimiter: '<Root>/Rate Limiter-theta_sp'
   */
  tmp_1 = rtsiIsModeUpdateTimeStep(&(&pid_control_V4_M)->solverInfo);

  /* End of Outputs for SubSystem: '<S299>/Hugw(s)' */
  /* End of Outputs for SubSystem: '<S299>/Hvgw(s)' */
  /* End of Outputs for SubSystem: '<S299>/Hwgw(s)' */
  /* End of Outputs for SubSystem: '<S298>/Hpgw' */
  /* End of Outputs for SubSystem: '<S298>/Hqgw' */
  /* End of Outputs for SubSystem: '<S298>/Hrgw' */

  /* Integrator: '<S19>/Integrator' incorporates:
   *  InitialCondition: '<S19>/IC'
   */
  if (tmp_1) {
    serverAvailableOnTime = (((pid_control_V4_PrevZCX.Integrator_Reset_ZCE ==
      POS_ZCSIG) != pid_control_V4_B.Compare) &&
      (pid_control_V4_PrevZCX.Integrator_Reset_ZCE != UNINITIALIZED_ZCSIG));
    pid_control_V4_PrevZCX.Integrator_Reset_ZCE = pid_control_V4_B.Compare;

    /* evaluate zero-crossings and the level of the reset signal */
    if (serverAvailableOnTime || pid_control_V4_B.Compare ||
        pid_control_V4_DW.Integrator_DWORK1) {
      memcpy(&pid_control_V4_X.Integrator_CSTATE[0], &pid_control_V4_B.IC[0],
             12U * sizeof(real_T));
      rtsiSetBlockStateForSolverChangedAtMajorStep(&(&pid_control_V4_M)
        ->solverInfo, true);
      rtsiSetContTimeOutputInconsistentWithStateAtMajorStep(&(&pid_control_V4_M
        )->solverInfo, true);
    }
  }

  /* Integrator: '<S19>/Integrator' */
  memcpy(&pid_control_V4_B.x[0], &pid_control_V4_X.Integrator_CSTATE[0], 12U *
         sizeof(real_T));

  /* SignalConversion generated from: '<Root>/Coordinate Transformation Conversion' incorporates:
   *  MATLABSystem: '<Root>/Coordinate Transformation Conversion'
   */
  _mm_storeu_pd(&pid_control_V4_B.wbe_b[0], _mm_div_pd(_mm_set_pd
    (pid_control_V4_B.x[6], pid_control_V4_B.x[8]), _mm_set1_pd(2.0)));

  /* MATLABSystem: '<Root>/Coordinate Transformation Conversion' incorporates:
   *  SignalConversion generated from: '<Root>/Coordinate Transformation Conversion'
   */
  pid_control_V4_B.wbe_b[2] = pid_control_V4_B.x[7] / 2.0;
  pid_control_V4_B.sina = sin(pid_control_V4_B.wbe_b[0]);
  pid_control_V4_B.sinb = sin(pid_control_V4_B.wbe_b[1]);
  pid_control_V4_B.sinc = sin(pid_control_V4_B.wbe_b[2]);
  pid_control_V4_B.cosa = cos(pid_control_V4_B.wbe_b[0]);
  pid_control_V4_B.cosb = cos(pid_control_V4_B.wbe_b[1]);
  pid_control_V4_B.cosc = cos(pid_control_V4_B.wbe_b[2]);

  /* Start for MATLABSystem: '<Root>/Coordinate Transformation Conversion' */
  pid_control_V4_B.rtb_CoordinateTransformationC_g = pid_control_V4_B.cosa *
    pid_control_V4_B.cosb;

  /* MATLABSystem: '<Root>/Coordinate Transformation Conversion' */
  pid_control_V4_B.rtb_CoordinateTransformationC_d = pid_control_V4_B.sina *
    pid_control_V4_B.sinb * pid_control_V4_B.sinc +
    pid_control_V4_B.rtb_CoordinateTransformationC_g * pid_control_V4_B.cosc;
  pid_control_V4_B.rtb_CoordinateTransformationC_g =
    pid_control_V4_B.rtb_CoordinateTransformationC_g * pid_control_V4_B.sinc -
    pid_control_V4_B.cosc * pid_control_V4_B.sina * pid_control_V4_B.sinb;
  pid_control_V4_B.rtb_CoordinateTransformationC_l = pid_control_V4_B.cosa *
    pid_control_V4_B.cosc * pid_control_V4_B.sinb + pid_control_V4_B.cosb *
    pid_control_V4_B.sina * pid_control_V4_B.sinc;
  pid_control_V4_B.sinc = pid_control_V4_B.cosb * pid_control_V4_B.cosc *
    pid_control_V4_B.sina - pid_control_V4_B.cosa * pid_control_V4_B.sinb *
    pid_control_V4_B.sinc;

  /* Gain: '<Root>/Gain-Z' */
  pid_control_V4_B.GainZ = -pid_control_V4_B.x[11];

  /* MATLAB Function: '<Root>/MATLAB Function-ned_to_gazebo' */
  pid_control_V4_B.sina = 0.0 * pid_control_V4_B.rtb_CoordinateTransformationC_d
    - pid_control_V4_B.sinc;
  pid_control_V4_B.sinb = 0.0 * pid_control_V4_B.rtb_CoordinateTransformationC_g
    - pid_control_V4_B.rtb_CoordinateTransformationC_l;
  pid_control_V4_B.cosa = 0.0 * pid_control_V4_B.rtb_CoordinateTransformationC_l
    + pid_control_V4_B.rtb_CoordinateTransformationC_g;
  pid_control_V4_B.z_out = 0.0 * pid_control_V4_B.sinc +
    pid_control_V4_B.rtb_CoordinateTransformationC_d;
  if (tmp_0) {
    /* MATLAB Function: '<Root>/MATLAB Function-IMU2' */
    pid_control_V4_MATLABFunction(&pid_control_V4_B.sf_MATLABFunctionIMU2);
  }

  /* BusAssignment: '<Root>/Bus Assignment-ODOM' */
  memset(&pid_control_V4_B.BusAssignmentODOM, 0, sizeof(SL_Bus_nav_msgs_Odometry));

  /* BusAssignment: '<Root>/Bus Assignment-IMU' */
  pid_control_V4_B.BusAssignmentIMU.orientation.x = pid_control_V4_B.sinb;
  pid_control_V4_B.BusAssignmentIMU.orientation.y = pid_control_V4_B.cosa;
  pid_control_V4_B.BusAssignmentIMU.orientation.z = pid_control_V4_B.z_out;
  pid_control_V4_B.BusAssignmentIMU.orientation.w = pid_control_V4_B.sina;
  pid_control_V4_B.BusAssignmentIMU.angular_velocity.x = pid_control_V4_B.x[3];
  pid_control_V4_B.BusAssignmentIMU.angular_velocity.y = pid_control_V4_B.x[4];
  pid_control_V4_B.BusAssignmentIMU.angular_velocity.z = pid_control_V4_B.x[5];
  memcpy(&pid_control_V4_B.BusAssignmentIMU.header.frame_id[0],
         &pid_control_V4_B.sf_MATLABFunctionIMU2.stringOut[0], sizeof(uint8_T) <<
         7U);
  pid_control_V4_B.BusAssignmentIMU.header.frame_id_SL_Info.CurrentLength =
    pid_control_V4_B.sf_MATLABFunctionIMU2.lengthOut;

  /* MATLABSystem: '<S16>/SinkBlock' */
  Pub_pid_control_V4_548.publish(&pid_control_V4_B.BusAssignmentIMU);
  if (tmp_0) {
    /* MATLAB Function: '<Root>/MATLAB Function-Odometry' */
    memset(&pid_control_V4_B.str_out_a[0], 0, sizeof(uint8_T) << 7U);
    for (i = 0; i < 5; i++) {
      pid_control_V4_B.str_out_a[i] = b[i];
    }

    pid_control_V4_B.currentLen_o = 5U;
    pid_control_V4_B.receivedLen_j = 5U;

    /* End of MATLAB Function: '<Root>/MATLAB Function-Odometry' */

    /* MATLAB Function: '<Root>/MATLAB Function-Odometry1' */
    memset(&pid_control_V4_B.str_out[0], 0, sizeof(uint8_T) << 7U);
    for (i = 0; i < 11; i++) {
      pid_control_V4_B.str_out[i] = b_0[i];
    }

    pid_control_V4_B.currentLen = 11U;
    pid_control_V4_B.receivedLen = 11U;

    /* End of MATLAB Function: '<Root>/MATLAB Function-Odometry1' */
  }

  /* BusAssignment: '<Root>/Bus Assignment-ODOM' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function-ned_to_gazebo'
   */
  pid_control_V4_B.BusAssignmentODOM.pose.pose.position.x = -pid_control_V4_B.x
    [10];
  pid_control_V4_B.BusAssignmentODOM.pose.pose.position.y = pid_control_V4_B.x[9];
  pid_control_V4_B.BusAssignmentODOM.pose.pose.position.z =
    pid_control_V4_B.GainZ;
  pid_control_V4_B.BusAssignmentODOM.pose.pose.orientation.x =
    pid_control_V4_B.sinb;
  pid_control_V4_B.BusAssignmentODOM.pose.pose.orientation.y =
    pid_control_V4_B.cosa;
  pid_control_V4_B.BusAssignmentODOM.pose.pose.orientation.z =
    pid_control_V4_B.z_out;
  pid_control_V4_B.BusAssignmentODOM.pose.pose.orientation.w =
    pid_control_V4_B.sina;
  pid_control_V4_B.BusAssignmentODOM.header.frame_id_SL_Info.CurrentLength =
    pid_control_V4_B.currentLen_o;
  pid_control_V4_B.BusAssignmentODOM.header.frame_id_SL_Info.ReceivedLength =
    pid_control_V4_B.receivedLen_j;
  memcpy(&pid_control_V4_B.BusAssignmentODOM.header.frame_id[0],
         &pid_control_V4_B.str_out_a[0], sizeof(uint8_T) << 7U);
  memcpy(&pid_control_V4_B.BusAssignmentODOM.child_frame_id[0],
         &pid_control_V4_B.str_out[0], sizeof(uint8_T) << 7U);
  pid_control_V4_B.BusAssignmentODOM.child_frame_id_SL_Info.CurrentLength =
    pid_control_V4_B.currentLen;
  pid_control_V4_B.BusAssignmentODOM.child_frame_id_SL_Info.ReceivedLength =
    pid_control_V4_B.receivedLen;

  /* MATLABSystem: '<S17>/SinkBlock' */
  Pub_pid_control_V4_697.publish(&pid_control_V4_B.BusAssignmentODOM);
  if (tmp_0) {
  }

  /* Step: '<S18>/Step' incorporates:
   *  RateLimiter: '<Root>/Rate Limiter-theta_sp'
   */
  pid_control_V4_B.sinc = (&pid_control_V4_M)->Timing.t[0];
  if (pid_control_V4_B.sinc < 0.0) {
    pid_control_V4_B.w_r = 0.0;
  } else {
    pid_control_V4_B.w_r = 4.04;
  }

  /* Product: '<S18>/Product' incorporates:
   *  Clock: '<S18>/Clock'
   *  Step: '<S18>/Step'
   */
  pid_control_V4_B.Saturation1 = pid_control_V4_B.w_r * (&pid_control_V4_M)
    ->Timing.t[0];

  /* Saturate: '<Root>/Saturation1' */
  if (pid_control_V4_B.Saturation1 > 20.2) {
    /* Product: '<S18>/Product' incorporates:
     *  Saturate: '<Root>/Saturation1'
     */
    pid_control_V4_B.Saturation1 = 20.2;
  } else if (pid_control_V4_B.Saturation1 < 0.0) {
    /* Product: '<S18>/Product' incorporates:
     *  Saturate: '<Root>/Saturation1'
     */
    pid_control_V4_B.Saturation1 = 0.0;
  }

  /* End of Saturate: '<Root>/Saturation1' */

  /* Gain: '<S267>/Integral Gain' incorporates:
   *  Sum: '<Root>/Sum3'
   */
  pid_control_V4_B.Switch_j = pid_control_V4_B.Saturation1 - pid_control_V4_B.x
    [0];

  /* Gain: '<S273>/Filter Coefficient' incorporates:
   *  Gain: '<S263>/Derivative Gain'
   *  Integrator: '<S265>/Filter'
   *  Sum: '<S265>/SumD'
   */
  pid_control_V4_B.FilterCoefficient = (0.005 * pid_control_V4_B.Switch_j -
    pid_control_V4_X.Filter_CSTATE) * 100.0;

  /* Sum: '<S279>/Sum' incorporates:
   *  Gain: '<S275>/Proportional Gain'
   *  Integrator: '<S270>/Integrator'
   */
  pid_control_V4_B.rtb_CoordinateTransformationC_d = (0.08 *
    pid_control_V4_B.Switch_j + pid_control_V4_X.Integrator_CSTATE_f) +
    pid_control_V4_B.FilterCoefficient;

  /* Saturate: '<S277>/Saturation' */
  if (pid_control_V4_B.rtb_CoordinateTransformationC_d > 1.0) {
    /* Saturate: '<S277>/Saturation' */
    pid_control_V4_B.Saturation = 1.0;
  } else if (pid_control_V4_B.rtb_CoordinateTransformationC_d < 0.0) {
    /* Saturate: '<S277>/Saturation' */
    pid_control_V4_B.Saturation = 0.0;
  } else {
    /* Saturate: '<S277>/Saturation' */
    pid_control_V4_B.Saturation =
      pid_control_V4_B.rtb_CoordinateTransformationC_d;
  }

  /* End of Saturate: '<S277>/Saturation' */
  if (tmp_0) {
    /* MATLABSystem: '<S20>/SourceBlock' */
    pid_control_V4_B.SourceBlock_o1_o = Sub_pid_control_V4_435.getLatestMessage(
      &rtb_SourceBlock_o2_d);

    /* Outputs for Enabled SubSystem: '<S20>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V4_B.SourceBlock_o1_o,
      &rtb_SourceBlock_o2_d, &pid_control_V4_B.EnabledSubsystem_b);

    /* End of Outputs for SubSystem: '<S20>/Enabled Subsystem' */

    /* Switch: '<Root>/Switch3' */
    if (pid_control_V4_B.SourceBlock_o1_o) {
      /* Switch: '<Root>/Switch3' */
      pid_control_V4_B.Switch3 = pid_control_V4_B.EnabledSubsystem_b.In1.data;
    } else {
      /* Switch: '<Root>/Switch3' incorporates:
       *  UnitDelay: '<Root>/Unit Delay3'
       */
      pid_control_V4_B.Switch3 = pid_control_V4_DW.UnitDelay3_DSTATE;
    }

    /* End of Switch: '<Root>/Switch3' */
  }

  /* Sum: '<Root>/Sum2' */
  pid_control_V4_B.cosb = pid_control_V4_B.Switch3 - pid_control_V4_B.GainZ;

  /* Gain: '<S113>/Filter Coefficient' incorporates:
   *  Gain: '<S103>/Derivative Gain'
   *  Integrator: '<S105>/Filter'
   *  Sum: '<S105>/SumD'
   */
  pid_control_V4_B.FilterCoefficient_j = (0.15 * pid_control_V4_B.cosb -
    pid_control_V4_X.Filter_CSTATE_f) * 15.0;

  /* Sum: '<S119>/Sum' incorporates:
   *  Gain: '<S115>/Proportional Gain'
   *  Integrator: '<S110>/Integrator'
   */
  pid_control_V4_B.cosc = (0.8 * pid_control_V4_B.cosb +
    pid_control_V4_X.Integrator_CSTATE_n) + pid_control_V4_B.FilterCoefficient_j;

  /* Saturate: '<S117>/Saturation' */
  if (pid_control_V4_B.cosc > 0.15) {
    /* Saturate: '<S117>/Saturation' */
    pid_control_V4_B.Saturation_f = 0.15;
  } else if (pid_control_V4_B.cosc < -0.1) {
    /* Saturate: '<S117>/Saturation' */
    pid_control_V4_B.Saturation_f = -0.1;
  } else {
    /* Saturate: '<S117>/Saturation' */
    pid_control_V4_B.Saturation_f = pid_control_V4_B.cosc;
  }

  /* End of Saturate: '<S117>/Saturation' */
  if (tmp_0) {
    /* MATLABSystem: '<S21>/SourceBlock' */
    pid_control_V4_B.SourceBlock_o1 = Sub_pid_control_V4_377.getLatestMessage
      (&pid_control_V4_B.SourceBlock_o2);

    /* Outputs for Enabled SubSystem: '<S21>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V4_B.SourceBlock_o1,
      &pid_control_V4_B.SourceBlock_o2, &pid_control_V4_B.EnabledSubsystem_a);

    /* End of Outputs for SubSystem: '<S21>/Enabled Subsystem' */

    /* Switch: '<Root>/Switch2' */
    if (pid_control_V4_B.SourceBlock_o1) {
      /* Switch: '<Root>/Switch2' */
      pid_control_V4_B.Switch2 = pid_control_V4_B.EnabledSubsystem_a.In1.data;
    } else {
      /* Switch: '<Root>/Switch2' incorporates:
       *  UnitDelay: '<Root>/Unit Delay2'
       */
      pid_control_V4_B.Switch2 = pid_control_V4_DW.UnitDelay2_DSTATE;
    }

    /* End of Switch: '<Root>/Switch2' */

    /* RateLimiter: '<Root>/Rate Limiter2' */
    pid_control_V4_B.Va = pid_control_V4_B.Switch2 - pid_control_V4_DW.PrevY;
    if (pid_control_V4_B.Va > 0.0005) {
      /* RateLimiter: '<Root>/Rate Limiter2' */
      pid_control_V4_B.RateLimiter2 = pid_control_V4_DW.PrevY + 0.0005;
    } else if (pid_control_V4_B.Va < -0.0005) {
      /* RateLimiter: '<Root>/Rate Limiter2' */
      pid_control_V4_B.RateLimiter2 = pid_control_V4_DW.PrevY - 0.0005;
    } else {
      /* RateLimiter: '<Root>/Rate Limiter2' */
      pid_control_V4_B.RateLimiter2 = pid_control_V4_B.Switch2;
    }

    pid_control_V4_DW.PrevY = pid_control_V4_B.RateLimiter2;

    /* End of RateLimiter: '<Root>/Rate Limiter2' */
  }

  /* Sum: '<Root>/Sum5' */
  pid_control_V4_B.rtb_CoordinateTransformationC_g =
    pid_control_V4_B.RateLimiter2 - pid_control_V4_B.x[8];

  /* Gain: '<S219>/Filter Coefficient' incorporates:
   *  Gain: '<S209>/Derivative Gain'
   *  Integrator: '<S211>/Filter'
   *  Sum: '<S211>/SumD'
   */
  pid_control_V4_B.FilterCoefficient_p = (-1.3 *
    pid_control_V4_B.rtb_CoordinateTransformationC_g -
    pid_control_V4_X.Filter_CSTATE_fd) * 10.0;

  /* Sum: '<S225>/Sum' incorporates:
   *  Gain: '<S221>/Proportional Gain'
   *  Integrator: '<S216>/Integrator'
   */
  pid_control_V4_B.rtb_CoordinateTransformationC_l = (-1.2 *
    pid_control_V4_B.rtb_CoordinateTransformationC_g +
    pid_control_V4_X.Integrator_CSTATE_d) + pid_control_V4_B.FilterCoefficient_p;

  /* Saturate: '<S223>/Saturation' */
  if (pid_control_V4_B.rtb_CoordinateTransformationC_l > 0.09) {
    /* Saturate: '<S223>/Saturation' */
    pid_control_V4_B.Saturation_m = 0.09;
  } else if (pid_control_V4_B.rtb_CoordinateTransformationC_l < -0.09) {
    /* Saturate: '<S223>/Saturation' */
    pid_control_V4_B.Saturation_m = -0.09;
  } else {
    /* Saturate: '<S223>/Saturation' */
    pid_control_V4_B.Saturation_m =
      pid_control_V4_B.rtb_CoordinateTransformationC_l;
  }

  /* End of Saturate: '<S223>/Saturation' */

  /* Saturate: '<Root>/Saturation_roll_sp' incorporates:
   *  Gain: '<Root>/Gain-roll_sp'
   */
  if (-pid_control_V4_B.Saturation_m > 0.1) {
    /* Saturate: '<Root>/Saturation_roll_sp' */
    pid_control_V4_B.Saturation_roll_sp = 0.1;
  } else if (-pid_control_V4_B.Saturation_m < -0.1) {
    /* Saturate: '<Root>/Saturation_roll_sp' */
    pid_control_V4_B.Saturation_roll_sp = -0.1;
  } else {
    /* Saturate: '<Root>/Saturation_roll_sp' */
    pid_control_V4_B.Saturation_roll_sp = -pid_control_V4_B.Saturation_m;
  }

  /* End of Saturate: '<Root>/Saturation_roll_sp' */

  /* Gain: '<Root>/Gain' */
  pid_control_V4_B.SignPreSat = 0.13 *
    pid_control_V4_B.rtb_CoordinateTransformationC_g;

  /* Saturate: '<Root>/Saturation' */
  if (pid_control_V4_B.SignPreSat > 0.14) {
    pid_control_V4_B.SignPreSat = 0.14;
  } else if (pid_control_V4_B.SignPreSat < -0.14) {
    pid_control_V4_B.SignPreSat = -0.14;
  }

  /* Gain: '<S55>/Integral Gain' incorporates:
   *  Saturate: '<Root>/Saturation'
   *  Sum: '<Root>/Sum4'
   */
  pid_control_V4_B.Switch = (pid_control_V4_B.Saturation_roll_sp +
    pid_control_V4_B.SignPreSat) - pid_control_V4_B.x[6];

  /* Gain: '<S61>/Filter Coefficient' incorporates:
   *  Gain: '<S51>/Derivative Gain'
   *  Integrator: '<S53>/Filter'
   *  Sum: '<S53>/SumD'
   */
  pid_control_V4_B.FilterCoefficient_c = (-0.2 * pid_control_V4_B.Switch -
    pid_control_V4_X.Filter_CSTATE_g) * 50.0;

  /* Sum: '<S67>/Sum' incorporates:
   *  Gain: '<S63>/Proportional Gain'
   *  Integrator: '<S58>/Integrator'
   */
  pid_control_V4_B.SignPreSat = (-0.12 * pid_control_V4_B.Switch +
    pid_control_V4_X.Integrator_CSTATE_m) + pid_control_V4_B.FilterCoefficient_c;

  /* Saturate: '<S65>/Saturation' */
  if (pid_control_V4_B.SignPreSat > 0.1) {
    /* Saturate: '<S65>/Saturation' */
    pid_control_V4_B.Saturation_k = 0.1;
  } else if (pid_control_V4_B.SignPreSat < -0.1) {
    /* Saturate: '<S65>/Saturation' */
    pid_control_V4_B.Saturation_k = -0.1;
  } else {
    /* Saturate: '<S65>/Saturation' */
    pid_control_V4_B.Saturation_k = pid_control_V4_B.SignPreSat;
  }

  /* End of Saturate: '<S65>/Saturation' */

  /* RateLimiter: '<Root>/Rate Limiter-theta_sp' */
  if (pid_control_V4_DW.LastMajorTime == (rtInf)) {
    /* RateLimiter: '<Root>/Rate Limiter-theta_sp' */
    pid_control_V4_B.RateLimitertheta_sp = pid_control_V4_B.Saturation_f;
  } else {
    pid_control_V4_B.w_r = pid_control_V4_B.sinc -
      pid_control_V4_DW.LastMajorTime;
    if (pid_control_V4_DW.LastMajorTime == pid_control_V4_B.sinc) {
      if (pid_control_V4_DW.PrevLimited) {
        /* RateLimiter: '<Root>/Rate Limiter-theta_sp' */
        pid_control_V4_B.RateLimitertheta_sp = pid_control_V4_DW.PrevY_g;
      } else {
        /* RateLimiter: '<Root>/Rate Limiter-theta_sp' */
        pid_control_V4_B.RateLimitertheta_sp = pid_control_V4_B.Saturation_f;
      }
    } else {
      pid_control_V4_B.sinc = pid_control_V4_B.w_r * 0.25;
      pid_control_V4_B.Va = pid_control_V4_B.Saturation_f -
        pid_control_V4_DW.PrevY_g;
      if (pid_control_V4_B.Va > pid_control_V4_B.sinc) {
        /* RateLimiter: '<Root>/Rate Limiter-theta_sp' */
        pid_control_V4_B.RateLimitertheta_sp = pid_control_V4_DW.PrevY_g +
          pid_control_V4_B.sinc;
        serverAvailableOnTime = true;
      } else {
        pid_control_V4_B.sinc = pid_control_V4_B.w_r * -0.2;
        if (pid_control_V4_B.Va < pid_control_V4_B.sinc) {
          /* RateLimiter: '<Root>/Rate Limiter-theta_sp' */
          pid_control_V4_B.RateLimitertheta_sp = pid_control_V4_DW.PrevY_g +
            pid_control_V4_B.sinc;
          serverAvailableOnTime = true;
        } else {
          /* RateLimiter: '<Root>/Rate Limiter-theta_sp' */
          pid_control_V4_B.RateLimitertheta_sp = pid_control_V4_B.Saturation_f;
          serverAvailableOnTime = false;
        }
      }

      if (tmp_1) {
        pid_control_V4_DW.PrevLimited = serverAvailableOnTime;
      }
    }
  }

  /* Math: '<S19>/Square1' incorporates:
   *  Sum: '<Root>/Sum1'
   */
  pid_control_V4_B.Switch_a = pid_control_V4_B.RateLimitertheta_sp -
    pid_control_V4_B.x[7];

  /* Gain: '<S167>/Filter Coefficient' incorporates:
   *  Gain: '<S157>/Derivative Gain'
   *  Integrator: '<S159>/Filter'
   *  Sum: '<S159>/SumD'
   */
  pid_control_V4_B.FilterCoefficient_m = (-0.2 * pid_control_V4_B.Switch_a -
    pid_control_V4_X.Filter_CSTATE_m) * 10.0;

  /* Sum: '<S173>/Sum' incorporates:
   *  Gain: '<S169>/Proportional Gain'
   *  Integrator: '<S164>/Integrator'
   */
  pid_control_V4_B.sinc = (-0.35 * pid_control_V4_B.Switch_a +
    pid_control_V4_X.Integrator_CSTATE_p) + pid_control_V4_B.FilterCoefficient_m;

  /* Saturate: '<S171>/Saturation' */
  if (pid_control_V4_B.sinc > 0.3490658503988659) {
    /* Saturate: '<S171>/Saturation' */
    pid_control_V4_B.Saturation_fh = 0.3490658503988659;
  } else if (pid_control_V4_B.sinc < -0.3490658503988659) {
    /* Saturate: '<S171>/Saturation' */
    pid_control_V4_B.Saturation_fh = -0.3490658503988659;
  } else {
    /* Saturate: '<S171>/Saturation' */
    pid_control_V4_B.Saturation_fh = pid_control_V4_B.sinc;
  }

  /* End of Saturate: '<S171>/Saturation' */
  if (tmp_0) {
    /* Memory: '<S19>/Memory' */
    pid_control_V4_B.Memory[0] = pid_control_V4_DW.Memory_PreviousInput[0];

    /* Memory: '<S19>/Memory1' */
    pid_control_V4_B.Memory1[0] = pid_control_V4_DW.Memory1_PreviousInput[0];

    /* Memory: '<S19>/Memory' */
    pid_control_V4_B.Memory[1] = pid_control_V4_DW.Memory_PreviousInput[1];

    /* Memory: '<S19>/Memory1' */
    pid_control_V4_B.Memory1[1] = pid_control_V4_DW.Memory1_PreviousInput[1];

    /* Memory: '<S19>/Memory' */
    pid_control_V4_B.Memory[2] = pid_control_V4_DW.Memory_PreviousInput[2];

    /* Memory: '<S19>/Memory1' */
    pid_control_V4_B.Memory1[2] = pid_control_V4_DW.Memory1_PreviousInput[2];
  }

  /* SignalConversion generated from: '<S291>/ SFunction ' incorporates:
   *  MATLAB Function: '<S19>/MATLAB Function - MODEL'
   */
  pid_control_V4_B.TmpSignalConversionAtSFunct[0] =
    pid_control_V4_B.Saturation_k;
  pid_control_V4_B.TmpSignalConversionAtSFunct[1] =
    pid_control_V4_B.Saturation_fh;
  pid_control_V4_B.TmpSignalConversionAtSFunct[2] =
    pid_control_V4_B.Saturation_m;
  pid_control_V4_B.TmpSignalConversionAtSFunct[3] = pid_control_V4_B.Saturation;
  pid_control_V4_B.TmpSignalConversionAtSFunct[4] = pid_control_V4_B.Saturation;

  /* MATLAB Function: '<S19>/MATLAB Function - MODEL' incorporates:
   *  Memory: '<S19>/Memory'
   */
  if (pid_control_V4_B.TmpSignalConversionAtSFunct[0] <= 0.3490658503988659) {
    pid_control_V4_B.u1 = pid_control_V4_B.TmpSignalConversionAtSFunct[0];
  } else {
    pid_control_V4_B.u1 = 0.3490658503988659;
  }

  if (!(pid_control_V4_B.u1 >= -0.3490658503988659)) {
    pid_control_V4_B.u1 = -0.3490658503988659;
  }

  if (pid_control_V4_B.TmpSignalConversionAtSFunct[1] <= 0.3490658503988659) {
    pid_control_V4_B.u2 = pid_control_V4_B.TmpSignalConversionAtSFunct[1];
  } else {
    pid_control_V4_B.u2 = 0.3490658503988659;
  }

  if (!(pid_control_V4_B.u2 >= -0.3490658503988659)) {
    pid_control_V4_B.u2 = -0.3490658503988659;
  }

  tmp_3 = _mm_add_pd(_mm_loadu_pd(&pid_control_V4_B.x[0]), _mm_loadu_pd
                     (&pid_control_V4_B.Memory[0]));
  _mm_storeu_pd(&pid_control_V4_B.dv1[0], tmp_3);

  /* MATLAB Function: '<S19>/MATLAB Function - MODEL' incorporates:
   *  Memory: '<S19>/Memory'
   *  Memory: '<S19>/Memory1'
   */
  pid_control_V4_B.w_r = pid_control_V4_B.x[2] + pid_control_V4_B.Memory[2];
  pid_control_V4_B.Va = sqrt((pid_control_V4_B.dv1[0] * pid_control_V4_B.dv1[0]
    + pid_control_V4_B.dv1[1] * pid_control_V4_B.dv1[1]) + pid_control_V4_B.w_r *
    pid_control_V4_B.w_r);
  if (pid_control_V4_B.Va < 0.5) {
    pid_control_V4_B.Va = 0.5;
    pid_control_V4_B.w_r = 0.0;
    pid_control_V4_B.beta = 0.0;
  } else {
    pid_control_V4_B.w_r = rt_atan2d_snf(pid_control_V4_B.w_r,
      pid_control_V4_B.dv1[0]);
    pid_control_V4_B.beta = pid_control_V4_B.dv1[1] / pid_control_V4_B.Va;
    if ((pid_control_V4_B.beta >= 1.0) || rtIsNaN(pid_control_V4_B.beta)) {
      pid_control_V4_B.beta = 1.0;
    }

    if (pid_control_V4_B.beta <= -1.0) {
      pid_control_V4_B.beta = -1.0;
    }

    pid_control_V4_B.beta = asin(pid_control_V4_B.beta);
  }

  pid_control_V4_B.q_aero = pid_control_V4_B.Memory1[1] + pid_control_V4_B.x[4];
  if ((-pid_control_V4_B.x[11] - 0.0505 <= 0.001) || rtIsNaN
      (-pid_control_V4_B.x[11] - 0.0505)) {
    pid_control_V4_B.hw = 0.001;
  } else {
    pid_control_V4_B.hw = -pid_control_V4_B.x[11] - 0.0505;
  }

  if ((-pid_control_V4_B.x[11] + 0.3475 <= 0.001) || rtIsNaN
      (-pid_control_V4_B.x[11] + 0.3475)) {
    pid_control_V4_B.hh = 0.001;
  } else {
    pid_control_V4_B.hh = -pid_control_V4_B.x[11] + 0.3475;
  }

  pid_control_V4_B.Q = pid_control_V4_B.Va * pid_control_V4_B.Va * 0.6125;
  pid_control_V4_B.wbe_b[0] = pid_control_V4_B.x[3];
  pid_control_V4_B.wbe_b[1] = pid_control_V4_B.x[4];
  pid_control_V4_B.wbe_b[2] = pid_control_V4_B.x[5];
  pid_control_V4_B.Vd1 = pid_control_V4_B.hw / 0.6977;
  pid_control_V4_B.mu_Lw_out = rt_powd_snf(pid_control_V4_B.Vd1, 0.787) * 288.0 *
    exp(rt_powd_snf(pid_control_V4_B.Vd1, 0.327) * -9.14) * 0.97986308862072491 /
    5.9129476540958859 + 1.0;
  pid_control_V4_B.mu_Dw_out = (1.0 - exp(rt_powd_snf(pid_control_V4_B.Vd1,
    0.814) * -4.74) * 0.97916641726789588) - exp(rt_powd_snf
    (pid_control_V4_B.Vd1, 0.758) * -3.88) * (pid_control_V4_B.Vd1 *
    pid_control_V4_B.Vd1);
  pid_control_V4_B.CD_ih_IGE = pid_control_V4_B.hh / 0.3808;
  pid_control_V4_B.hh = ((pid_control_V4_B.w_r - -0.065449846949787352) +
    0.026179938779914941) * 4.9604094530365153;
  pid_control_V4_B.hw = (((pid_control_V4_B.w_r - -0.043633231299858237) +
    0.0087266462599716477) - (0.56 / pid_control_V4_B.Va * 0.35 *
    pid_control_V4_B.q_aero + (pid_control_V4_B.w_r - -0.065449846949787352) *
    0.35)) * 4.8387748917360032;
  pid_control_V4_B.CL_w_IGE = pid_control_V4_B.hh * pid_control_V4_B.mu_Lw_out;
  pid_control_V4_B.CL_h_IGE = (rt_powd_snf(pid_control_V4_B.CD_ih_IGE, 0.787) *
    288.0 * exp(rt_powd_snf(pid_control_V4_B.CD_ih_IGE, 0.327) * -9.14) *
    0.95628590200128227 / 5.35300902982722 + 1.0) * pid_control_V4_B.hw;
  pid_control_V4_B.CD_iw_IGE = pid_control_V4_B.CL_w_IGE *
    pid_control_V4_B.CL_w_IGE / 21.205750411731103 * pid_control_V4_B.mu_Dw_out;
  pid_control_V4_B.CD_ih_IGE = ((1.0 - exp(rt_powd_snf
    (pid_control_V4_B.CD_ih_IGE, 0.814) * -4.74) * 0.96770634751485862) - exp
    (rt_powd_snf(pid_control_V4_B.CD_ih_IGE, 0.758) * -3.88) *
    (pid_control_V4_B.CD_ih_IGE * pid_control_V4_B.CD_ih_IGE)) *
    (pid_control_V4_B.CL_h_IGE * pid_control_V4_B.CL_h_IGE / 18.943803701146454);
  pid_control_V4_B.u2_deg = pid_control_V4_B.u2 * 180.0 / 3.1415926535897931;
  pid_control_V4_B.u2_deg = ((pid_control_V4_B.u2_deg * pid_control_V4_B.u2_deg *
    -1.08E-5 + 0.000715 * pid_control_V4_B.u2_deg) * 0.02164 +
    ((pid_control_V4_B.CD_iw_IGE * 0.0649 + 0.0027258) +
     pid_control_V4_B.CD_ih_IGE * 0.02164)) * pid_control_V4_B.Q;
  pid_control_V4_B.Ltot_tmp = pid_control_V4_B.CL_w_IGE * 0.0649 +
    pid_control_V4_B.CL_h_IGE * 0.02164;
  pid_control_V4_B.Ltot = pid_control_V4_B.Ltot_tmp * pid_control_V4_B.Q;
  pid_control_V4_B.CQ = -0.019 * pid_control_V4_B.beta * 180.0 /
    3.1415926535897931;
  pid_control_V4_B.FA_b_idx_0 = sin(pid_control_V4_B.w_r);
  pid_control_V4_B.FA_b_idx_1 = cos(pid_control_V4_B.w_r);
  pid_control_V4_B.R[0] = pid_control_V4_B.FA_b_idx_1;
  pid_control_V4_B.R[3] = 0.0;
  pid_control_V4_B.R[6] = -pid_control_V4_B.FA_b_idx_0;
  pid_control_V4_B.R[2] = pid_control_V4_B.FA_b_idx_0;
  pid_control_V4_B.R[5] = 0.0;
  pid_control_V4_B.R[8] = pid_control_V4_B.FA_b_idx_1;
  pid_control_V4_B.Dtot[0] = -pid_control_V4_B.u2_deg;
  pid_control_V4_B.Dtot[1] = pid_control_V4_B.CQ * pid_control_V4_B.Q * 0.0649;
  pid_control_V4_B.Dtot[2] = -pid_control_V4_B.Ltot;
  pid_control_V4_B.R[1] = 0.0;
  pid_control_V4_B.FA_b_idx_0 = 0.0;
  pid_control_V4_B.R[4] = 1.0;
  pid_control_V4_B.FA_b_idx_1 = 0.0;
  pid_control_V4_B.R[7] = 0.0;
  pid_control_V4_B.FA_b_idx_2 = 0.0;
  for (i = 0; i < 3; i++) {
    tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V4_B.R[3 * i]),
      _mm_set1_pd(pid_control_V4_B.Dtot[i])), _mm_set_pd
                       (pid_control_V4_B.FA_b_idx_1, pid_control_V4_B.FA_b_idx_0));
    _mm_storeu_pd(&pid_control_V4_B.dv1[0], tmp_3);
    pid_control_V4_B.FA_b_idx_0 = pid_control_V4_B.dv1[0];
    pid_control_V4_B.FA_b_idx_1 = pid_control_V4_B.dv1[1];
    pid_control_V4_B.FA_b_idx_2 += pid_control_V4_B.R[3 * i + 2] *
      pid_control_V4_B.Dtot[i];
  }

  pid_control_V4_B.Fg_b_idx_2 = 2.0 * pid_control_V4_B.Va;
  pid_control_V4_B.Cl = ((pid_control_V4_B.Memory1[0] + pid_control_V4_B.x[3]) *
    0.6977 / pid_control_V4_B.Fg_b_idx_2 * -2.0 + -0.12 * pid_control_V4_B.beta)
    + -0.5 * pid_control_V4_B.u1;
  pid_control_V4_B.u2 = ((exp(pid_control_V4_B.Vd1 * -4.0) * -0.05 + -1.14 *
    pid_control_V4_B.w_r) + pid_control_V4_B.q_aero * 0.093 /
    pid_control_V4_B.Fg_b_idx_2 * -5.0) + -3.0 * pid_control_V4_B.u2;
  if (pid_control_V4_B.TmpSignalConversionAtSFunct[2] <= 0.26179938779914941) {
    pid_control_V4_B.q_aero = pid_control_V4_B.TmpSignalConversionAtSFunct[2];
  } else {
    pid_control_V4_B.q_aero = 0.26179938779914941;
  }

  if (!(pid_control_V4_B.q_aero >= -0.26179938779914941)) {
    pid_control_V4_B.q_aero = -0.26179938779914941;
  }

  pid_control_V4_B.u1 = (((pid_control_V4_B.Memory1[2] + pid_control_V4_B.x[5]) *
    0.6977 / pid_control_V4_B.Fg_b_idx_2 * -1.5 + -0.1146 *
    pid_control_V4_B.beta) + -0.3 * pid_control_V4_B.q_aero) + -0.05 *
    pid_control_V4_B.u1;
  if (pid_control_V4_B.TmpSignalConversionAtSFunct[3] <= 1.0) {
    pid_control_V4_B.q_aero = pid_control_V4_B.TmpSignalConversionAtSFunct[3];
  } else {
    pid_control_V4_B.q_aero = 1.0;
  }

  if (!(pid_control_V4_B.q_aero >= 0.0)) {
    pid_control_V4_B.q_aero = 0.0;
  }

  pid_control_V4_B.Vd1 = (25.0 - pid_control_V4_B.Va) * pid_control_V4_B.q_aero
    + pid_control_V4_B.Va;
  if (pid_control_V4_B.TmpSignalConversionAtSFunct[4] <= 1.0) {
    pid_control_V4_B.q_aero = pid_control_V4_B.TmpSignalConversionAtSFunct[4];
  } else {
    pid_control_V4_B.q_aero = 1.0;
  }

  if (!(pid_control_V4_B.q_aero >= 0.0)) {
    pid_control_V4_B.q_aero = 0.0;
  }

  pid_control_V4_B.q_aero = (25.0 - pid_control_V4_B.Va) *
    pid_control_V4_B.q_aero + pid_control_V4_B.Va;
  pid_control_V4_B.Vd1 = 0.0044226062159978471 * pid_control_V4_B.Vd1 *
    (pid_control_V4_B.Vd1 - pid_control_V4_B.Va);
  pid_control_V4_B.q_aero = 0.0044226062159978471 * pid_control_V4_B.q_aero *
    (pid_control_V4_B.q_aero - pid_control_V4_B.Va);
  pid_control_V4_B.FE1_b[0] = pid_control_V4_B.Vd1 * 0.99619469809174555;
  pid_control_V4_B.FE1_b[2] = -pid_control_V4_B.Vd1 * 0.087155742747658166;
  pid_control_V4_B.FE2_b_idx_0 = pid_control_V4_B.q_aero * 0.99619469809174555;
  pid_control_V4_B.FE2_b_idx_2 = -pid_control_V4_B.q_aero * 0.087155742747658166;
  pid_control_V4_B.q_aero = -9.81 * sin(pid_control_V4_B.x[7]) * 1.2;
  _mm_storeu_pd(&pid_control_V4_B.dv1[0], _mm_mul_pd(_mm_mul_pd(_mm_mul_pd
    (_mm_set1_pd(9.81), _mm_set1_pd(cos(pid_control_V4_B.x[7]))), _mm_set_pd(cos
    (pid_control_V4_B.x[6]), sin(pid_control_V4_B.x[6]))), _mm_set1_pd(1.2)));
  pid_control_V4_B.Vd1 = pid_control_V4_B.dv1[0];
  pid_control_V4_B.Fg_b_idx_2 = pid_control_V4_B.dv1[1];
  pid_control_V4_B.FE_b = pid_control_V4_B.FE1_b[0] +
    pid_control_V4_B.FE2_b_idx_0;
  pid_control_V4_B.FE_b_idx_0 = pid_control_V4_B.FE_b;
  pid_control_V4_B.F_b[0] = (pid_control_V4_B.q_aero + pid_control_V4_B.FE_b) +
    pid_control_V4_B.FA_b_idx_0;
  pid_control_V4_B.F_b[1] = pid_control_V4_B.dv1[0] +
    pid_control_V4_B.FA_b_idx_1;
  pid_control_V4_B.FE_b = pid_control_V4_B.FE1_b[2] +
    pid_control_V4_B.FE2_b_idx_2;
  pid_control_V4_B.F_b[2] = (pid_control_V4_B.dv1[1] + pid_control_V4_B.FE_b) +
    pid_control_V4_B.FA_b_idx_2;
  pid_control_V4_B.L_dec = 0.6977 * pid_control_V4_B.Q * 0.0649;
  pid_control_V4_B.Mcg_b_idx_0 = (0.0834 * pid_control_V4_B.FE1_b[2] + -0.0834 *
    pid_control_V4_B.FE2_b_idx_2) + pid_control_V4_B.L_dec * pid_control_V4_B.Cl;
  pid_control_V4_B.Q = 0.093 * pid_control_V4_B.Q * 0.0649 * pid_control_V4_B.u2
    + ((-0.0396 * pid_control_V4_B.FE1_b[0] - 0.0721 * pid_control_V4_B.FE1_b[2])
       + (-0.0396 * pid_control_V4_B.FE2_b_idx_0 - 0.0721 *
          pid_control_V4_B.FE2_b_idx_2));
  pid_control_V4_B.FE2_b_idx_0 = ((0.0 - 0.0834 * pid_control_V4_B.FE1_b[0]) +
    (0.0 - -0.0834 * pid_control_V4_B.FE2_b_idx_0)) + pid_control_V4_B.L_dec *
    pid_control_V4_B.u1;
  pid_control_V4_B.L_dec = pid_control_V4_B.Mcg_b_idx_0 - 0.078414804174236352 *
    pid_control_V4_B.FE2_b_idx_0;
  pid_control_V4_B.FE2_b_idx_2 = pid_control_V4_B.FE2_b_idx_0 -
    0.2258731897286114 * pid_control_V4_B.Mcg_b_idx_0;
  memcpy(&pid_control_V4_B.R[0], &pid_control_V4_B.x[0], 9U * sizeof(real_T));
  pid_control_V4_B.FE1_b[0] = pid_control_V4_B.R[0];
  pid_control_V4_B.FE1_b[1] = pid_control_V4_B.R[1];
  pid_control_V4_B.FE1_b[2] = pid_control_V4_B.R[2];
  pid_control_V4_B.c_phi = cos(pid_control_V4_B.R[6]);
  pid_control_V4_B.s_phi = sin(pid_control_V4_B.R[6]);
  pid_control_V4_B.c_the = cos(pid_control_V4_B.R[7]);
  pid_control_V4_B.s_the = sin(pid_control_V4_B.R[7]);
  pid_control_V4_B.c_psi = cos(pid_control_V4_B.R[8]);
  pid_control_V4_B.s_psi = sin(pid_control_V4_B.R[8]);
  pid_control_V4_B.R[0] = pid_control_V4_B.c_the * pid_control_V4_B.c_psi;
  pid_control_V4_B.R_tmp = pid_control_V4_B.s_phi * pid_control_V4_B.s_the;
  pid_control_V4_B.R[3] = pid_control_V4_B.R_tmp * pid_control_V4_B.c_psi -
    pid_control_V4_B.c_phi * pid_control_V4_B.s_psi;
  pid_control_V4_B.R_tmp_d = pid_control_V4_B.c_phi * pid_control_V4_B.s_the;
  pid_control_V4_B.R[6] = pid_control_V4_B.R_tmp_d * pid_control_V4_B.c_psi +
    pid_control_V4_B.s_phi * pid_control_V4_B.s_psi;
  pid_control_V4_B.R[1] = pid_control_V4_B.c_the * pid_control_V4_B.s_psi;
  pid_control_V4_B.R[4] = pid_control_V4_B.R_tmp * pid_control_V4_B.s_psi +
    pid_control_V4_B.c_phi * pid_control_V4_B.c_psi;
  pid_control_V4_B.R[7] = pid_control_V4_B.R_tmp_d * pid_control_V4_B.s_psi -
    pid_control_V4_B.s_phi * pid_control_V4_B.c_psi;
  pid_control_V4_B.R[2] = -pid_control_V4_B.s_the;
  pid_control_V4_B.R[5] = pid_control_V4_B.s_phi * pid_control_V4_B.c_the;
  pid_control_V4_B.R[8] = pid_control_V4_B.c_phi * pid_control_V4_B.c_the;
  tmp_3 = _mm_sub_pd(_mm_mul_pd(_mm_set_pd(pid_control_V4_B.x[0],
    pid_control_V4_B.x[2]), _mm_loadu_pd(&pid_control_V4_B.x[4])), _mm_mul_pd
                     (_mm_loadu_pd(&pid_control_V4_B.x[1]), _mm_set_pd
                      (pid_control_V4_B.x[3], pid_control_V4_B.x[5])));
  _mm_storeu_pd(&pid_control_V4_B.Dtot[0], tmp_3);
  pid_control_V4_B.Dtot[2] = pid_control_V4_B.x[1] * pid_control_V4_B.x[3] -
    pid_control_V4_B.x[0] * pid_control_V4_B.x[4];
  pid_control_V4_B.dv[0] = 1.0;
  _mm_storeu_pd(&pid_control_V4_B.dv1[0], _mm_mul_pd(_mm_set_pd(cos
    (pid_control_V4_B.x[6]), sin(pid_control_V4_B.x[6])), _mm_set1_pd(tan
    (pid_control_V4_B.x[7]))));
  pid_control_V4_B.dv[3] = pid_control_V4_B.dv1[0];
  pid_control_V4_B.dv[6] = pid_control_V4_B.dv1[1];
  pid_control_V4_B.dv[1] = 0.0;
  pid_control_V4_B.dv[4] = cos(pid_control_V4_B.x[6]);
  pid_control_V4_B.dv[7] = -sin(pid_control_V4_B.x[6]);
  pid_control_V4_B.dv[2] = 0.0;
  _mm_storeu_pd(&pid_control_V4_B.dv1[0], _mm_div_pd(_mm_set_pd(cos
    (pid_control_V4_B.x[6]), sin(pid_control_V4_B.x[6])), _mm_set1_pd(cos
    (pid_control_V4_B.x[7]))));
  pid_control_V4_B.dv[5] = pid_control_V4_B.dv1[0];
  pid_control_V4_B.dv[8] = pid_control_V4_B.dv1[1];
  pid_control_V4_B.c_phi = 0.0;
  pid_control_V4_B.s_phi = 0.0;
  pid_control_V4_B.c_the = 0.0;
  pid_control_V4_B.s_the = 0.0;
  pid_control_V4_B.c_psi = 0.0;
  pid_control_V4_B.s_psi = 0.0;
  for (i = 0; i < 3; i++) {
    tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V4_B.dv[3 * i]),
      _mm_set1_pd(pid_control_V4_B.wbe_b[i])), _mm_set_pd(pid_control_V4_B.s_phi,
      pid_control_V4_B.c_phi));
    _mm_storeu_pd(&pid_control_V4_B.dv1[0], tmp_3);
    pid_control_V4_B.c_phi = pid_control_V4_B.dv1[0];
    pid_control_V4_B.s_phi = pid_control_V4_B.dv1[1];
    _mm_storeu_pd(&pid_control_V4_B.dv1[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V4_B.R[3 * i], pid_control_V4_B.dv[3 * i + 2]), _mm_set_pd
      (pid_control_V4_B.FE1_b[i], pid_control_V4_B.wbe_b[i])), _mm_set_pd
      (pid_control_V4_B.s_the, pid_control_V4_B.c_the)));
    pid_control_V4_B.c_the = pid_control_V4_B.dv1[0];
    pid_control_V4_B.s_the = pid_control_V4_B.dv1[1];
    tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V4_B.R[3 * i + 1]),
      _mm_set1_pd(pid_control_V4_B.FE1_b[i])), _mm_set_pd(pid_control_V4_B.s_psi,
      pid_control_V4_B.c_psi));
    _mm_storeu_pd(&pid_control_V4_B.dv1[0], tmp_3);
    pid_control_V4_B.c_psi = pid_control_V4_B.dv1[0];
    pid_control_V4_B.s_psi = pid_control_V4_B.dv1[1];
    pid_control_V4_B.XDOT[i] = 0.83333333333333337 * pid_control_V4_B.F_b[i] -
      pid_control_V4_B.Dtot[i];
  }

  pid_control_V4_B.XDOT[3] = ((0.023669 * pid_control_V4_B.L_dec + 0.001856 *
    pid_control_V4_B.FE2_b_idx_2) - (-2.6338496E-5 * pid_control_V4_B.x[3] +
    0.000144843342 * pid_control_V4_B.x[5]) * pid_control_V4_B.x[4]) /
    0.000191043437;
  pid_control_V4_B.XDOT[4] = ((pid_control_V4_B.Q - -0.015451999999999999 *
    pid_control_V4_B.x[3] * pid_control_V4_B.x[5]) - (pid_control_V4_B.x[3] *
    pid_control_V4_B.x[3] - pid_control_V4_B.x[5] * pid_control_V4_B.x[5]) *
    0.001856) / 0.017695;
  pid_control_V4_B.XDOT[5] = ((-2.6338496E-5 * pid_control_V4_B.x[5] +
    -7.4435989999999989E-5 * pid_control_V4_B.x[3]) * pid_control_V4_B.x[4] +
    (0.001856 * pid_control_V4_B.L_dec + 0.008217 * pid_control_V4_B.FE2_b_idx_2))
    / 0.000191043437;
  pid_control_V4_B.XDOT[9] = pid_control_V4_B.s_the;
  pid_control_V4_B.XDOT[10] = pid_control_V4_B.c_psi;
  pid_control_V4_B.XDOT[11] = pid_control_V4_B.s_psi;
  if (!(pid_control_V4_B.u2_deg >= 0.001)) {
    pid_control_V4_B.u2_deg = 0.001;
  }

  pid_control_V4_B.XDOT[12] = pid_control_V4_B.Ltot / pid_control_V4_B.u2_deg;
  pid_control_V4_B.XDOT[19] = pid_control_V4_B.CQ;
  pid_control_V4_B.XDOT[20] = pid_control_V4_B.Cl;
  pid_control_V4_B.XDOT[21] = pid_control_V4_B.u2;
  pid_control_V4_B.XDOT[22] = pid_control_V4_B.u1;
  pid_control_V4_B.XDOT[23] = pid_control_V4_B.w_r;
  pid_control_V4_B.XDOT[24] = pid_control_V4_B.beta;
  pid_control_V4_B.XDOT[25] = pid_control_V4_B.hh;
  pid_control_V4_B.XDOT[26] = pid_control_V4_B.hw;
  pid_control_V4_B.XDOT[27] = pid_control_V4_B.CL_w_IGE;
  pid_control_V4_B.XDOT[28] = pid_control_V4_B.CL_h_IGE;
  pid_control_V4_B.XDOT[29] = pid_control_V4_B.CD_iw_IGE;
  pid_control_V4_B.XDOT[30] = pid_control_V4_B.CD_ih_IGE;
  pid_control_V4_B.XDOT[6] = pid_control_V4_B.c_phi;
  pid_control_V4_B.XDOT[13] = pid_control_V4_B.F_b[0];
  pid_control_V4_B.XDOT[16] = pid_control_V4_B.Mcg_b_idx_0;
  pid_control_V4_B.XDOT[31] = pid_control_V4_B.q_aero;
  pid_control_V4_B.XDOT[34] = pid_control_V4_B.FE_b_idx_0;
  pid_control_V4_B.XDOT[37] = pid_control_V4_B.FA_b_idx_0;
  pid_control_V4_B.XDOT[7] = pid_control_V4_B.s_phi;
  pid_control_V4_B.XDOT[14] = pid_control_V4_B.F_b[1];
  pid_control_V4_B.XDOT[17] = pid_control_V4_B.Q;
  pid_control_V4_B.XDOT[32] = pid_control_V4_B.Vd1;
  pid_control_V4_B.XDOT[35] = 0.0;
  pid_control_V4_B.XDOT[38] = pid_control_V4_B.FA_b_idx_1;
  pid_control_V4_B.XDOT[8] = pid_control_V4_B.c_the;
  pid_control_V4_B.XDOT[15] = pid_control_V4_B.F_b[2];
  pid_control_V4_B.XDOT[18] = pid_control_V4_B.FE2_b_idx_0;
  pid_control_V4_B.XDOT[33] = pid_control_V4_B.Fg_b_idx_2;
  pid_control_V4_B.XDOT[36] = pid_control_V4_B.FE_b;
  pid_control_V4_B.XDOT[39] = pid_control_V4_B.FA_b_idx_2;
  pid_control_V4_B.CL_total = pid_control_V4_B.Ltot_tmp / 0.08654;
  if (tmp_0) {
    /* MATLAB Function: '<Root>/MATLAB Function' */
    pid_control_V4_MATLABFunction(&pid_control_V4_B.sf_MATLABFunction);

    /* MATLAB Function: '<Root>/MATLAB Function1' */
    memset(&pid_control_V4_B.stringOut[0], 0, sizeof(uint8_T) << 7U);
    for (i = 0; i < 5; i++) {
      pid_control_V4_B.stringOut[i] = b[i];
    }

    pid_control_V4_B.lengthOut = 5U;

    /* End of MATLAB Function: '<Root>/MATLAB Function1' */
  }

  /* BusAssignment: '<Root>/Bus Assignment-MODELO' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function-ned_to_gazebo'
   */
  memset(&pid_control_V4_B.BusAssignmentMODELO, 0, sizeof
         (SL_Bus_gazebo_msgs_SetEntityStateRequest));
  pid_control_V4_B.BusAssignmentMODELO.state.name_SL_Info.CurrentLength =
    pid_control_V4_B.sf_MATLABFunction.lengthOut;
  pid_control_V4_B.BusAssignmentMODELO.state.pose.position.x =
    -pid_control_V4_B.x[10];
  pid_control_V4_B.BusAssignmentMODELO.state.pose.position.y =
    pid_control_V4_B.x[9];
  pid_control_V4_B.BusAssignmentMODELO.state.pose.position.z =
    pid_control_V4_B.GainZ;
  pid_control_V4_B.BusAssignmentMODELO.state.pose.orientation.w =
    pid_control_V4_B.sina;
  pid_control_V4_B.BusAssignmentMODELO.state.pose.orientation.z =
    pid_control_V4_B.z_out;
  pid_control_V4_B.BusAssignmentMODELO.state.pose.orientation.y =
    pid_control_V4_B.cosa;
  pid_control_V4_B.BusAssignmentMODELO.state.pose.orientation.x =
    pid_control_V4_B.sinb;
  memcpy(&pid_control_V4_B.BusAssignmentMODELO.state.name[0],
         &pid_control_V4_B.sf_MATLABFunction.stringOut[0], sizeof(uint8_T) << 7U);
  memcpy(&pid_control_V4_B.BusAssignmentMODELO.state.reference_frame[0],
         &pid_control_V4_B.stringOut[0], sizeof(uint8_T) << 7U);
  pid_control_V4_B.BusAssignmentMODELO.state.reference_frame_SL_Info.CurrentLength
    = pid_control_V4_B.lengthOut;

  /* Outputs for Atomic SubSystem: '<Root>/Call Service' */
  /* MATLABSystem: '<S4>/ServiceCaller' */
  serverAvailableOnTime = ServCall_pid_control_V4_326.waitForServer(5.0);
  if (serverAvailableOnTime) {
    ServCall_pid_control_V4_326.call(&pid_control_V4_B.BusAssignmentMODELO, &tmp);
  }

  /* End of MATLABSystem: '<S4>/ServiceCaller' */
  /* End of Outputs for SubSystem: '<Root>/Call Service' */

  /* Gain: '<S48>/ZeroGain' */
  pid_control_V4_B.sina = 0.0 * pid_control_V4_B.SignPreSat;

  /* DeadZone: '<S50>/DeadZone' */
  if (pid_control_V4_B.SignPreSat > 0.1) {
    pid_control_V4_B.SignPreSat -= 0.1;
  } else if (pid_control_V4_B.SignPreSat >= -0.1) {
    pid_control_V4_B.SignPreSat = 0.0;
  } else {
    pid_control_V4_B.SignPreSat -= -0.1;
  }

  /* End of DeadZone: '<S50>/DeadZone' */

  /* Gain: '<S55>/Integral Gain' */
  pid_control_V4_B.Switch *= -0.05;

  /* Signum: '<S48>/SignPreSat' */
  if (rtIsNaN(pid_control_V4_B.SignPreSat)) {
    /* DataTypeConversion: '<S48>/DataTypeConv1' */
    i = 0;
  } else {
    if (pid_control_V4_B.SignPreSat < 0.0) {
      /* DataTypeConversion: '<S48>/DataTypeConv1' */
      pid_control_V4_B.w_r = -1.0;
    } else {
      /* DataTypeConversion: '<S48>/DataTypeConv1' */
      pid_control_V4_B.w_r = (pid_control_V4_B.SignPreSat > 0.0);
    }

    /* DataTypeConversion: '<S48>/DataTypeConv1' */
    i = static_cast<int32_T>(fmod(pid_control_V4_B.w_r, 256.0));
  }

  /* End of Signum: '<S48>/SignPreSat' */

  /* Signum: '<S48>/SignPreIntegrator' */
  if (rtIsNaN(pid_control_V4_B.Switch)) {
    /* DataTypeConversion: '<S48>/DataTypeConv2' */
    tmp_2 = 0;
  } else {
    if (pid_control_V4_B.Switch < 0.0) {
      /* DataTypeConversion: '<S48>/DataTypeConv2' */
      pid_control_V4_B.w_r = -1.0;
    } else {
      /* DataTypeConversion: '<S48>/DataTypeConv2' */
      pid_control_V4_B.w_r = (pid_control_V4_B.Switch > 0.0);
    }

    /* DataTypeConversion: '<S48>/DataTypeConv2' */
    tmp_2 = static_cast<int32_T>(fmod(pid_control_V4_B.w_r, 256.0));
  }

  /* End of Signum: '<S48>/SignPreIntegrator' */

  /* DataTypeConversion: '<S48>/DataTypeConv1' */
  if (i < 0) {
    i = static_cast<int8_T>(-static_cast<int8_T>(static_cast<uint8_T>(-
      static_cast<real_T>(i))));
  }

  /* DataTypeConversion: '<S48>/DataTypeConv2' */
  if (tmp_2 < 0) {
    tmp_2 = static_cast<int8_T>(-static_cast<int8_T>(static_cast<uint8_T>(-
      static_cast<real_T>(tmp_2))));
  }

  /* Logic: '<S48>/AND3' incorporates:
   *  DataTypeConversion: '<S48>/DataTypeConv1'
   *  DataTypeConversion: '<S48>/DataTypeConv2'
   *  RelationalOperator: '<S48>/Equal1'
   *  RelationalOperator: '<S48>/NotEqual'
   */
  pid_control_V4_B.AND3 = ((pid_control_V4_B.sina != pid_control_V4_B.SignPreSat)
    && (i == tmp_2));
  if (tmp_0) {
    /* Memory: '<S48>/Memory' */
    pid_control_V4_B.Memory_a = pid_control_V4_DW.Memory_PreviousInput_o;
  }

  /* Switch: '<S48>/Switch' */
  if (pid_control_V4_B.Memory_a) {
    /* Gain: '<S55>/Integral Gain' incorporates:
     *  Constant: '<S48>/Constant1'
     *  Switch: '<S48>/Switch'
     */
    pid_control_V4_B.Switch = 0.0;
  }

  /* End of Switch: '<S48>/Switch' */

  /* Sum: '<S102>/SumI4' incorporates:
   *  Gain: '<S102>/Kb'
   *  Gain: '<S107>/Integral Gain'
   *  Sum: '<S102>/SumI2'
   */
  pid_control_V4_B.SumI4 = (pid_control_V4_B.Saturation_f -
    pid_control_V4_B.cosc) * 0.1 + 0.03 * pid_control_V4_B.cosb;

  /* Gain: '<S154>/ZeroGain' */
  pid_control_V4_B.sina = 0.0 * pid_control_V4_B.sinc;

  /* DeadZone: '<S156>/DeadZone' */
  if (pid_control_V4_B.sinc > 0.3490658503988659) {
    pid_control_V4_B.sinc -= 0.3490658503988659;
  } else if (pid_control_V4_B.sinc >= -0.3490658503988659) {
    pid_control_V4_B.sinc = 0.0;
  } else {
    pid_control_V4_B.sinc -= -0.3490658503988659;
  }

  /* End of DeadZone: '<S156>/DeadZone' */

  /* Math: '<S19>/Square1' incorporates:
   *  Gain: '<S161>/Integral Gain'
   */
  pid_control_V4_B.Switch_a *= -0.05;

  /* Signum: '<S154>/SignPreSat' */
  if (rtIsNaN(pid_control_V4_B.sinc)) {
    /* DataTypeConversion: '<S154>/DataTypeConv1' */
    i = 0;
  } else {
    if (pid_control_V4_B.sinc < 0.0) {
      /* DataTypeConversion: '<S154>/DataTypeConv1' */
      pid_control_V4_B.w_r = -1.0;
    } else {
      /* DataTypeConversion: '<S154>/DataTypeConv1' */
      pid_control_V4_B.w_r = (pid_control_V4_B.sinc > 0.0);
    }

    /* DataTypeConversion: '<S154>/DataTypeConv1' */
    i = static_cast<int32_T>(fmod(pid_control_V4_B.w_r, 256.0));
  }

  /* End of Signum: '<S154>/SignPreSat' */

  /* Signum: '<S154>/SignPreIntegrator' */
  if (rtIsNaN(pid_control_V4_B.Switch_a)) {
    /* DataTypeConversion: '<S154>/DataTypeConv2' */
    tmp_2 = 0;
  } else {
    if (pid_control_V4_B.Switch_a < 0.0) {
      /* DataTypeConversion: '<S154>/DataTypeConv2' */
      pid_control_V4_B.w_r = -1.0;
    } else {
      /* DataTypeConversion: '<S154>/DataTypeConv2' */
      pid_control_V4_B.w_r = (pid_control_V4_B.Switch_a > 0.0);
    }

    /* DataTypeConversion: '<S154>/DataTypeConv2' */
    tmp_2 = static_cast<int32_T>(fmod(pid_control_V4_B.w_r, 256.0));
  }

  /* End of Signum: '<S154>/SignPreIntegrator' */

  /* DataTypeConversion: '<S154>/DataTypeConv1' */
  if (i < 0) {
    i = static_cast<int8_T>(-static_cast<int8_T>(static_cast<uint8_T>(-
      static_cast<real_T>(i))));
  }

  /* DataTypeConversion: '<S154>/DataTypeConv2' */
  if (tmp_2 < 0) {
    tmp_2 = static_cast<int8_T>(-static_cast<int8_T>(static_cast<uint8_T>(-
      static_cast<real_T>(tmp_2))));
  }

  /* Logic: '<S154>/AND3' incorporates:
   *  DataTypeConversion: '<S154>/DataTypeConv1'
   *  DataTypeConversion: '<S154>/DataTypeConv2'
   *  RelationalOperator: '<S154>/Equal1'
   *  RelationalOperator: '<S154>/NotEqual'
   */
  pid_control_V4_B.AND3_j = ((pid_control_V4_B.sina != pid_control_V4_B.sinc) &&
    (i == tmp_2));
  if (tmp_0) {
    /* Memory: '<S154>/Memory' */
    pid_control_V4_B.Memory_n = pid_control_V4_DW.Memory_PreviousInput_d;
  }

  /* Switch: '<S154>/Switch' */
  if (pid_control_V4_B.Memory_n) {
    /* Math: '<S19>/Square1' incorporates:
     *  Constant: '<S154>/Constant1'
     *  Switch: '<S154>/Switch'
     */
    pid_control_V4_B.Switch_a = 0.0;
  }

  /* End of Switch: '<S154>/Switch' */

  /* Sum: '<S208>/SumI4' incorporates:
   *  Gain: '<S208>/Kb'
   *  Gain: '<S213>/Integral Gain'
   *  Sum: '<S208>/SumI2'
   */
  pid_control_V4_B.SumI4_d = (pid_control_V4_B.Saturation_m -
    pid_control_V4_B.rtb_CoordinateTransformationC_l) * 0.2 + -0.04 *
    pid_control_V4_B.rtb_CoordinateTransformationC_g;

  /* Gain: '<S260>/ZeroGain' */
  pid_control_V4_B.sina = 0.0 * pid_control_V4_B.rtb_CoordinateTransformationC_d;

  /* DeadZone: '<S262>/DeadZone' */
  if (pid_control_V4_B.rtb_CoordinateTransformationC_d > 1.0) {
    pid_control_V4_B.rtb_CoordinateTransformationC_d--;
  } else if (pid_control_V4_B.rtb_CoordinateTransformationC_d >= 0.0) {
    pid_control_V4_B.rtb_CoordinateTransformationC_d = 0.0;
  }

  /* End of DeadZone: '<S262>/DeadZone' */

  /* Gain: '<S267>/Integral Gain' */
  pid_control_V4_B.Switch_j *= 0.015;

  /* Signum: '<S260>/SignPreSat' */
  if (rtIsNaN(pid_control_V4_B.rtb_CoordinateTransformationC_d)) {
    /* DataTypeConversion: '<S260>/DataTypeConv1' */
    i = 0;
  } else {
    if (pid_control_V4_B.rtb_CoordinateTransformationC_d < 0.0) {
      /* DataTypeConversion: '<S260>/DataTypeConv1' */
      pid_control_V4_B.w_r = -1.0;
    } else {
      /* DataTypeConversion: '<S260>/DataTypeConv1' */
      pid_control_V4_B.w_r = (pid_control_V4_B.rtb_CoordinateTransformationC_d >
        0.0);
    }

    /* DataTypeConversion: '<S260>/DataTypeConv1' */
    i = static_cast<int32_T>(fmod(pid_control_V4_B.w_r, 256.0));
  }

  /* End of Signum: '<S260>/SignPreSat' */

  /* Signum: '<S260>/SignPreIntegrator' */
  if (rtIsNaN(pid_control_V4_B.Switch_j)) {
    /* DataTypeConversion: '<S260>/DataTypeConv2' */
    tmp_2 = 0;
  } else {
    if (pid_control_V4_B.Switch_j < 0.0) {
      /* DataTypeConversion: '<S260>/DataTypeConv2' */
      pid_control_V4_B.w_r = -1.0;
    } else {
      /* DataTypeConversion: '<S260>/DataTypeConv2' */
      pid_control_V4_B.w_r = (pid_control_V4_B.Switch_j > 0.0);
    }

    /* DataTypeConversion: '<S260>/DataTypeConv2' */
    tmp_2 = static_cast<int32_T>(fmod(pid_control_V4_B.w_r, 256.0));
  }

  /* End of Signum: '<S260>/SignPreIntegrator' */

  /* DataTypeConversion: '<S260>/DataTypeConv1' */
  if (i < 0) {
    i = static_cast<int8_T>(-static_cast<int8_T>(static_cast<uint8_T>(-
      static_cast<real_T>(i))));
  }

  /* DataTypeConversion: '<S260>/DataTypeConv2' */
  if (tmp_2 < 0) {
    tmp_2 = static_cast<int8_T>(-static_cast<int8_T>(static_cast<uint8_T>(-
      static_cast<real_T>(tmp_2))));
  }

  /* Logic: '<S260>/AND3' incorporates:
   *  DataTypeConversion: '<S260>/DataTypeConv1'
   *  DataTypeConversion: '<S260>/DataTypeConv2'
   *  RelationalOperator: '<S260>/Equal1'
   *  RelationalOperator: '<S260>/NotEqual'
   */
  pid_control_V4_B.AND3_c = ((pid_control_V4_B.sina !=
    pid_control_V4_B.rtb_CoordinateTransformationC_d) && (i == tmp_2));
  if (tmp_0) {
    /* Memory: '<S260>/Memory' */
    pid_control_V4_B.Memory_h = pid_control_V4_DW.Memory_PreviousInput_a;
  }

  /* Switch: '<S260>/Switch' */
  if (pid_control_V4_B.Memory_h) {
    /* Gain: '<S267>/Integral Gain' incorporates:
     *  Constant: '<S260>/Constant1'
     *  Switch: '<S260>/Switch'
     */
    pid_control_V4_B.Switch_j = 0.0;
  }

  /* End of Switch: '<S260>/Switch' */

  /* UnitConversion: '<S300>/Unit Conversion' incorporates:
   *  Gain: '<S19>/Gain4'
   */
  /* Unit Conversion - from: m to: ft
     Expression: output = (3.28084*input) + (0) */
  pid_control_V4_B.rtb_CoordinateTransformationC_d = 3.280839895013123 *
    -pid_control_V4_B.x[11];

  /* UnitConversion: '<S306>/Unit Conversion' incorporates:
   *  MATLAB Function: '<S19>/MATLAB Function - MODEL'
   */
  /* Unit Conversion - from: m/s to: ft/s
     Expression: output = (3.28084*input) + (0) */
  pid_control_V4_B.sina = 3.280839895013123 * pid_control_V4_B.Va;

  /* Saturate: '<S333>/Limit Function 10ft to 1000ft' incorporates:
   *  Saturate: '<S316>/Limit Height h<1000ft'
   */
  if (pid_control_V4_B.rtb_CoordinateTransformationC_d > 1000.0) {
    pid_control_V4_B.sinb = 1000.0;
    pid_control_V4_B.sinc = 1000.0;
  } else {
    if (pid_control_V4_B.rtb_CoordinateTransformationC_d < 10.0) {
      pid_control_V4_B.sinb = 10.0;
    } else {
      pid_control_V4_B.sinb = pid_control_V4_B.rtb_CoordinateTransformationC_d;
    }

    if (pid_control_V4_B.rtb_CoordinateTransformationC_d < 0.0) {
      pid_control_V4_B.sinc = 0.0;
    } else {
      pid_control_V4_B.sinc = pid_control_V4_B.rtb_CoordinateTransformationC_d;
    }
  }

  /* End of Saturate: '<S333>/Limit Function 10ft to 1000ft' */

  /* Fcn: '<S333>/Low Altitude Scale Length' */
  pid_control_V4_B.cosa = pid_control_V4_B.sinb / rt_powd_snf(0.000823 *
    pid_control_V4_B.sinb + 0.177, 1.2);

  /* Product: '<S316>/sigma_ug, sigma_vg' incorporates:
   *  Fcn: '<S316>/Low Altitude Intensity'
   */
  pid_control_V4_B.rtb_CoordinateTransformationC_l = 1.0 / rt_powd_snf(0.000823 *
    pid_control_V4_B.sinc + 0.177, 0.4) * pid_control_V4_ConstB.sigma_wg;

  /* Interpolation_n-D: '<S315>/Medium//High Altitude Intensity' incorporates:
   *  PreLookup: '<S315>/PreLook-Up Index Search  (altitude)'
   */
  pid_control_V4_B.bpIndex[0] = plook_bincpa
    (pid_control_V4_B.rtb_CoordinateTransformationC_d,
     pid_control_V4_ConstP.PreLookUpIndexSearchaltitude_Br, 11U,
     &pid_control_V4_B.sinc, &pid_control_V4_DW.PreLookUpIndexSearchaltitude_DW);
  pid_control_V4_B.frac[0] = pid_control_V4_B.sinc;
  pid_control_V4_B.frac[1] = pid_control_V4_ConstB.PreLookUpIndexSearchprobofe;
  pid_control_V4_B.bpIndex[1] =
    pid_control_V4_ConstB.PreLookUpIndexSearchprobo_g;
  pid_control_V4_B.rtb_CoordinateTransformationC_g = intrp2d_la_pw
    (pid_control_V4_B.bpIndex, pid_control_V4_B.frac,
     pid_control_V4_ConstP.MediumHighAltitudeIntensity_Tab, 12U,
     pid_control_V4_ConstP.MediumHighAltitudeIntensity_max);
  if (tmp_0) {
    /* Product: '<S308>/Divide' incorporates:
     *  Product: '<S308>/Product'
     *  RandomNumber: '<S308>/White Noise'
     */
    tmp_3 = _mm_mul_pd(_mm_loadu_pd(&pid_control_V4_ConstB.Divide[0]),
                       _mm_loadu_pd(&pid_control_V4_DW.NextOutput[0]));

    /* Product: '<S308>/Product' */
    _mm_storeu_pd(&pid_control_V4_B.Product[0], tmp_3);

    /* Product: '<S308>/Divide' incorporates:
     *  Product: '<S308>/Product'
     *  RandomNumber: '<S308>/White Noise'
     */
    tmp_3 = _mm_mul_pd(_mm_loadu_pd(&pid_control_V4_ConstB.Divide[2]),
                       _mm_loadu_pd(&pid_control_V4_DW.NextOutput[2]));

    /* Product: '<S308>/Product' */
    _mm_storeu_pd(&pid_control_V4_B.Product[2], tmp_3);

    /* Outputs for Enabled SubSystem: '<S299>/Hugw(s)' incorporates:
     *  EnablePort: '<S312>/Enable'
     */
    if (tmp_1 && (!pid_control_V4_DW.Hugws_MODE)) {
      (void) memset(&(pid_control_V4_XDis.ug_p_CSTATE), 0,
                    2*sizeof(boolean_T));

      /* InitializeConditions for Integrator: '<S312>/ug_p' */
      pid_control_V4_X.ug_p_CSTATE[0] = 0.0;
      pid_control_V4_X.ug_p_CSTATE[1] = 0.0;
      pid_control_V4_DW.Hugws_MODE = true;
    }

    /* End of Outputs for SubSystem: '<S299>/Hugw(s)' */
  }

  /* Outputs for Enabled SubSystem: '<S299>/Hugw(s)' incorporates:
   *  EnablePort: '<S312>/Enable'
   */
  if (pid_control_V4_DW.Hugws_MODE) {
    /* Product: '<S312>/Lug//V' */
    pid_control_V4_B.frac[0] = pid_control_V4_B.cosa / pid_control_V4_B.sina;
    pid_control_V4_B.frac[1] = pid_control_V4_ConstB.UnitConversion_c /
      pid_control_V4_B.sina;

    /* Sqrt: '<S312>/sqrt' incorporates:
     *  Gain: '<S312>/(2//pi)'
     *  Integrator: '<S312>/ug_p'
     *  Product: '<S312>/Lug//V1'
     */
    tmp_3 = _mm_div_pd(_mm_sub_pd(_mm_mul_pd(_mm_set_pd(sqrt(0.63661977236758138
      * pid_control_V4_B.frac[1]), sqrt(0.63661977236758138 *
      pid_control_V4_B.frac[0])), _mm_set1_pd(pid_control_V4_B.Product[0])),
      _mm_loadu_pd(&pid_control_V4_X.ug_p_CSTATE[0])), _mm_loadu_pd
                       (&pid_control_V4_B.frac[0]));

    /* Product: '<S312>/w' */
    _mm_storeu_pd(&pid_control_V4_B.w_n[0], tmp_3);

    /* Integrator: '<S312>/ug_p' incorporates:
     *  Product: '<S312>/w1'
     */
    tmp_3 = _mm_mul_pd(_mm_loadu_pd(&pid_control_V4_X.ug_p_CSTATE[0]),
                       _mm_set_pd
                       (pid_control_V4_B.rtb_CoordinateTransformationC_g,
                        pid_control_V4_B.rtb_CoordinateTransformationC_l));

    /* Product: '<S312>/w1' */
    _mm_storeu_pd(&pid_control_V4_B.w1_c[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S299>/Hugw(s)' */

  /* Gain: '<S305>/Lv' */
  pid_control_V4_B.frac[1] = pid_control_V4_ConstB.UnitConversion_c;

  /* Outputs for Enabled SubSystem: '<S299>/Hvgw(s)' incorporates:
   *  EnablePort: '<S313>/Enable'
   */
  if (tmp_0 && tmp_1 && (!pid_control_V4_DW.Hvgws_MODE)) {
    (void) memset(&(pid_control_V4_XDis.vg_p1_CSTATE), 0,
                  4*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S313>/vg_p1' */
    pid_control_V4_X.vg_p1_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S313>/vgw_p2' */
    pid_control_V4_X.vgw_p2_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S313>/vg_p1' */
    pid_control_V4_X.vg_p1_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S313>/vgw_p2' */
    pid_control_V4_X.vgw_p2_CSTATE[1] = 0.0;
    pid_control_V4_DW.Hvgws_MODE = true;
  }

  if (pid_control_V4_DW.Hvgws_MODE) {
    /* Product: '<S313>/Lvg//V' incorporates:
     *  Gain: '<S305>/Lv'
     */
    pid_control_V4_B.sinc = pid_control_V4_B.cosa / pid_control_V4_B.sina;

    /* Product: '<S313>/w' incorporates:
     *  Gain: '<S313>/(1//pi)'
     *  Integrator: '<S313>/vg_p1'
     *  Product: '<S313>/Lug//V1'
     *  Sqrt: '<S313>/sqrt'
     *  Sum: '<S313>/Sum'
     */
    pid_control_V4_B.cosa = (sqrt(0.31830988618379069 * pid_control_V4_B.sinc) *
      pid_control_V4_B.Product[1] - pid_control_V4_X.vg_p1_CSTATE[0]) /
      pid_control_V4_B.sinc;
    pid_control_V4_B.w_g[0] = pid_control_V4_B.cosa;

    /* Product: '<S313>/w ' incorporates:
     *  Gain: '<S313>/sqrt(3)'
     *  Integrator: '<S313>/vg_p1'
     *  Integrator: '<S313>/vgw_p2'
     *  Product: '<S313>/Lvg//V '
     *  Sum: '<S313>/Sum1'
     */
    pid_control_V4_B.w_e[0] = (pid_control_V4_B.cosa * pid_control_V4_B.sinc *
      1.7320508075688772 + (pid_control_V4_X.vg_p1_CSTATE[0] -
      pid_control_V4_X.vgw_p2_CSTATE[0])) / pid_control_V4_B.sinc;

    /* Product: '<S313>/Lvg//V' */
    pid_control_V4_B.sinc = pid_control_V4_B.frac[1] / pid_control_V4_B.sina;

    /* Product: '<S313>/w' incorporates:
     *  Gain: '<S313>/(1//pi)'
     *  Integrator: '<S313>/vg_p1'
     *  Product: '<S313>/Lug//V1'
     *  Sqrt: '<S313>/sqrt'
     *  Sum: '<S313>/Sum'
     */
    pid_control_V4_B.cosa = (sqrt(0.31830988618379069 * pid_control_V4_B.sinc) *
      pid_control_V4_B.Product[1] - pid_control_V4_X.vg_p1_CSTATE[1]) /
      pid_control_V4_B.sinc;
    pid_control_V4_B.w_g[1] = pid_control_V4_B.cosa;

    /* Product: '<S313>/w ' incorporates:
     *  Gain: '<S313>/sqrt(3)'
     *  Integrator: '<S313>/vg_p1'
     *  Integrator: '<S313>/vgw_p2'
     *  Product: '<S313>/Lvg//V '
     *  Sum: '<S313>/Sum1'
     */
    pid_control_V4_B.w_e[1] = (pid_control_V4_B.cosa * pid_control_V4_B.sinc *
      1.7320508075688772 + (pid_control_V4_X.vg_p1_CSTATE[1] -
      pid_control_V4_X.vgw_p2_CSTATE[1])) / pid_control_V4_B.sinc;

    /* Product: '<S313>/w 1' incorporates:
     *  Integrator: '<S313>/vgw_p2'
     */
    tmp_3 = _mm_mul_pd(_mm_set_pd
                       (pid_control_V4_B.rtb_CoordinateTransformationC_g,
                        pid_control_V4_B.rtb_CoordinateTransformationC_l),
                       _mm_loadu_pd(&pid_control_V4_X.vgw_p2_CSTATE[0]));

    /* Product: '<S313>/w 1' */
    _mm_storeu_pd(&pid_control_V4_B.w1[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S299>/Hvgw(s)' */

  /* Gain: '<S305>/Lw' */
  pid_control_V4_B.frac[1] = pid_control_V4_ConstB.UnitConversion_c;

  /* Outputs for Enabled SubSystem: '<S299>/Hwgw(s)' incorporates:
   *  EnablePort: '<S314>/Enable'
   */
  if (tmp_0 && tmp_1 && (!pid_control_V4_DW.Hwgws_MODE)) {
    (void) memset(&(pid_control_V4_XDis.wg_p1_CSTATE), 0,
                  4*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S314>/wg_p1' */
    pid_control_V4_X.wg_p1_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S314>/wg_p2' */
    pid_control_V4_X.wg_p2_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S314>/wg_p1' */
    pid_control_V4_X.wg_p1_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S314>/wg_p2' */
    pid_control_V4_X.wg_p2_CSTATE[1] = 0.0;
    pid_control_V4_DW.Hwgws_MODE = true;
  }

  if (pid_control_V4_DW.Hwgws_MODE) {
    /* Product: '<S314>/Lwg//V' incorporates:
     *  Gain: '<S305>/Lw'
     */
    pid_control_V4_B.rtb_CoordinateTransformationC_l = pid_control_V4_B.sinb /
      pid_control_V4_B.sina;

    /* Product: '<S314>/w' incorporates:
     *  Gain: '<S314>/1//pi'
     *  Integrator: '<S314>/wg_p1'
     *  Product: '<S314>/Lug//V1'
     *  Sqrt: '<S314>/sqrt1'
     *  Sum: '<S314>/Sum'
     */
    pid_control_V4_B.sinc = (sqrt(0.31830988618379069 *
      pid_control_V4_B.rtb_CoordinateTransformationC_l) *
      pid_control_V4_B.Product[2] - pid_control_V4_X.wg_p1_CSTATE[0]) /
      pid_control_V4_B.rtb_CoordinateTransformationC_l;
    pid_control_V4_B.w[0] = pid_control_V4_B.sinc;

    /* Product: '<S314>/w ' incorporates:
     *  Integrator: '<S314>/wg_p1'
     *  Integrator: '<S314>/wg_p2'
     *  Product: '<S314>/Lwg//V'
     *  Product: '<S314>/Lwg//V '
     *  Sum: '<S314>/Sum1'
     */
    pid_control_V4_B.w_a[0] = (pid_control_V4_B.sinc *
      pid_control_V4_ConstB.sqrt_a *
      pid_control_V4_B.rtb_CoordinateTransformationC_l +
      (pid_control_V4_X.wg_p1_CSTATE[0] - pid_control_V4_X.wg_p2_CSTATE[0])) /
      pid_control_V4_B.rtb_CoordinateTransformationC_l;

    /* Product: '<S314>/Lwg//V' */
    pid_control_V4_B.rtb_CoordinateTransformationC_l = pid_control_V4_B.frac[1] /
      pid_control_V4_B.sina;

    /* Product: '<S314>/w' incorporates:
     *  Gain: '<S314>/1//pi'
     *  Integrator: '<S314>/wg_p1'
     *  Product: '<S314>/Lug//V1'
     *  Sqrt: '<S314>/sqrt1'
     *  Sum: '<S314>/Sum'
     */
    pid_control_V4_B.sinc = (sqrt(0.31830988618379069 *
      pid_control_V4_B.rtb_CoordinateTransformationC_l) *
      pid_control_V4_B.Product[2] - pid_control_V4_X.wg_p1_CSTATE[1]) /
      pid_control_V4_B.rtb_CoordinateTransformationC_l;
    pid_control_V4_B.w[1] = pid_control_V4_B.sinc;

    /* Product: '<S314>/w ' incorporates:
     *  Integrator: '<S314>/wg_p1'
     *  Integrator: '<S314>/wg_p2'
     *  Product: '<S314>/Lwg//V'
     *  Product: '<S314>/Lwg//V '
     *  Sum: '<S314>/Sum1'
     */
    pid_control_V4_B.w_a[1] = (pid_control_V4_B.sinc *
      pid_control_V4_ConstB.sqrt_a *
      pid_control_V4_B.rtb_CoordinateTransformationC_l +
      (pid_control_V4_X.wg_p1_CSTATE[1] - pid_control_V4_X.wg_p2_CSTATE[1])) /
      pid_control_V4_B.rtb_CoordinateTransformationC_l;

    /* Product: '<S314>/Lwg//V 1' incorporates:
     *  Integrator: '<S314>/wg_p2'
     */
    tmp_3 = _mm_mul_pd(_mm_set_pd
                       (pid_control_V4_B.rtb_CoordinateTransformationC_g,
                        pid_control_V4_ConstB.sigma_wg), _mm_loadu_pd
                       (&pid_control_V4_X.wg_p2_CSTATE[0]));

    /* Product: '<S314>/Lwg//V 1' */
    _mm_storeu_pd(&pid_control_V4_B.LwgV1[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S299>/Hwgw(s)' */

  /* Angle2Dcm: '<S19>/Rotation Angles to Direction Cosine Matrix' */
  pid_control_V4_B.rtb_CoordinateTransformationC_l = cos(pid_control_V4_B.x[6]);
  pid_control_V4_B.sinc = sin(pid_control_V4_B.x[6]);
  pid_control_V4_B.cosa = -sin(pid_control_V4_B.x[6]);
  pid_control_V4_B.cosb = cos(pid_control_V4_B.x[6]);
  pid_control_V4_B.hw = cos(pid_control_V4_B.x[7]);
  pid_control_V4_B.CD_ih_IGE = -sin(pid_control_V4_B.x[7]);
  pid_control_V4_B.cosc = sin(pid_control_V4_B.x[7]);
  pid_control_V4_B.z_out = cos(pid_control_V4_B.x[7]);
  pid_control_V4_B.SignPreSat = cos(pid_control_V4_B.x[8]);
  pid_control_V4_B.Va = sin(pid_control_V4_B.x[8]);
  pid_control_V4_B.w_r = -sin(pid_control_V4_B.x[8]);
  pid_control_V4_B.CL_w_IGE = cos(pid_control_V4_B.x[8]);
  pid_control_V4_B.beta = 0.0 * pid_control_V4_B.cosc + pid_control_V4_B.hw;
  pid_control_V4_B.hh = 0.0 * pid_control_V4_B.z_out +
    pid_control_V4_B.CD_ih_IGE;
  pid_control_V4_B.CL_h_IGE = pid_control_V4_B.SignPreSat * 0.0;
  pid_control_V4_B.CD_iw_IGE = 0.0 * pid_control_V4_B.hw;
  pid_control_V4_B.hw = (pid_control_V4_B.CD_iw_IGE + pid_control_V4_B.CL_h_IGE)
    + pid_control_V4_B.Va * pid_control_V4_B.cosc;
  pid_control_V4_B.SignPreSat += pid_control_V4_B.Va * 0.0;
  pid_control_V4_B.CD_ih_IGE *= 0.0;
  pid_control_V4_B.Va = (pid_control_V4_B.CD_ih_IGE + pid_control_V4_B.CL_h_IGE)
    + pid_control_V4_B.Va * pid_control_V4_B.z_out;
  pid_control_V4_B.CL_h_IGE = pid_control_V4_B.w_r * 0.0;
  pid_control_V4_B.cosc = (pid_control_V4_B.CD_iw_IGE +
    pid_control_V4_B.CL_h_IGE) + pid_control_V4_B.cosc *
    pid_control_V4_B.CL_w_IGE;
  pid_control_V4_B.w_r += pid_control_V4_B.CL_w_IGE * 0.0;
  pid_control_V4_B.z_out = (pid_control_V4_B.CD_ih_IGE +
    pid_control_V4_B.CL_h_IGE) + pid_control_V4_B.CL_w_IGE *
    pid_control_V4_B.z_out;
  pid_control_V4_B.CL_w_IGE = pid_control_V4_B.hh * 0.0;
  pid_control_V4_B.RotationAnglestoDirectionCo[0] = (pid_control_V4_B.beta *
    pid_control_V4_B.rtb_CoordinateTransformationC_l + 0.0 *
    pid_control_V4_B.cosa) + pid_control_V4_B.CL_w_IGE;
  pid_control_V4_B.CL_h_IGE = pid_control_V4_B.Va * 0.0;
  pid_control_V4_B.RotationAnglestoDirectionCo[1] =
    (pid_control_V4_B.rtb_CoordinateTransformationC_l * pid_control_V4_B.hw +
     pid_control_V4_B.cosa * pid_control_V4_B.SignPreSat) +
    pid_control_V4_B.CL_h_IGE;
  pid_control_V4_B.CD_iw_IGE = pid_control_V4_B.z_out * 0.0;
  pid_control_V4_B.RotationAnglestoDirectionCo[2] =
    (pid_control_V4_B.rtb_CoordinateTransformationC_l * pid_control_V4_B.cosc +
     pid_control_V4_B.cosa * pid_control_V4_B.w_r) + pid_control_V4_B.CD_iw_IGE;
  pid_control_V4_B.RotationAnglestoDirectionCo[3] = (pid_control_V4_B.beta *
    pid_control_V4_B.sinc + 0.0 * pid_control_V4_B.cosb) +
    pid_control_V4_B.CL_w_IGE;
  pid_control_V4_B.RotationAnglestoDirectionCo[4] = (pid_control_V4_B.sinc *
    pid_control_V4_B.hw + pid_control_V4_B.SignPreSat * pid_control_V4_B.cosb) +
    pid_control_V4_B.CL_h_IGE;
  pid_control_V4_B.RotationAnglestoDirectionCo[5] = (pid_control_V4_B.sinc *
    pid_control_V4_B.cosc + pid_control_V4_B.cosb * pid_control_V4_B.w_r) +
    pid_control_V4_B.CD_iw_IGE;
  pid_control_V4_B.RotationAnglestoDirectionCo[6] = pid_control_V4_B.beta * 0.0
    + pid_control_V4_B.hh;
  pid_control_V4_B.RotationAnglestoDirectionCo[7] = (pid_control_V4_B.hw * 0.0 +
    pid_control_V4_B.SignPreSat * 0.0) + pid_control_V4_B.Va;
  pid_control_V4_B.RotationAnglestoDirectionCo[8] = (pid_control_V4_B.cosc * 0.0
    + pid_control_V4_B.w_r * 0.0) + pid_control_V4_B.z_out;

  /* If: '<S304>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
   *  Constant: '<S325>/max_height_low'
   *  Product: '<S325>/Product1'
   *  Product: '<S330>/Product1'
   *  Product: '<S330>/Product2'
   *  Product: '<S332>/Product1'
   *  Product: '<S332>/Product2'
   *  Sum: '<S325>/Sum1'
   *  Sum: '<S325>/Sum2'
   *  Sum: '<S325>/Sum3'
   *  Sum: '<S330>/Sum'
   *  Sum: '<S332>/Sum'
   */
  rtPrevAction = pid_control_V4_DW.ifHeightMaxlowaltitudeelseifHei;
  if (tmp_1) {
    if (pid_control_V4_B.rtb_CoordinateTransformationC_d <= 1000.0) {
      rtAction = 0;
    } else if (pid_control_V4_B.rtb_CoordinateTransformationC_d >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    pid_control_V4_DW.ifHeightMaxlowaltitudeelseifHei = rtAction;
  } else {
    rtAction = pid_control_V4_DW.ifHeightMaxlowaltitudeelseifHei;
  }

  if (rtPrevAction != rtAction) {
    rtsiSetBlockStateForSolverChangedAtMajorStep(&(&pid_control_V4_M)
      ->solverInfo, true);
  }

  switch (rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S304>/Low altitude  velocities' incorporates:
     *  ActionPort: '<S326>/Action Port'
     */
    /* SignalConversion generated from: '<S331>/Vector Concatenate' */
    pid_control_V4_B.Product_l[2] = pid_control_V4_B.LwgV1[0];

    /* Trigonometry: '<S332>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S297>/Unit Conversion'
     */
    pid_control_V4_B.rtb_CoordinateTransformationC_l = sin
      (pid_control_V4_ConstB.UnitConversion);
    pid_control_V4_B.sinc = cos(pid_control_V4_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V4_B.Product_l[0], _mm_add_pd(_mm_mul_pd
      (_mm_set_pd(pid_control_V4_B.rtb_CoordinateTransformationC_l,
                  pid_control_V4_B.w1_c[0]), _mm_set_pd(pid_control_V4_B.w1_c[0],
      pid_control_V4_B.sinc)), _mm_mul_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V4_B.w1[0], pid_control_V4_B.rtb_CoordinateTransformationC_l),
      _mm_set_pd(pid_control_V4_B.sinc, pid_control_V4_B.w1[0])), _mm_set_pd(1.0,
      -1.0))));

    /* Product: '<S331>/Product' incorporates:
     *  Angle2Dcm: '<S19>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S331>/Vector Concatenate'
     *  Product: '<S332>/Product1'
     *  Product: '<S332>/Product2'
     *  Reshape: '<S331>/Reshape1'
     *  Sum: '<S332>/Sum'
     */
    pid_control_V4_B.rtb_CoordinateTransformationC_l = 0.0;
    pid_control_V4_B.sinc = 0.0;
    pid_control_V4_B.cosa = 0.0;
    for (i = 0; i < 3; i++) {
      tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V4_B.RotationAnglestoDirectionCo[3 * i]), _mm_set1_pd
        (pid_control_V4_B.Product_l[i])), _mm_set_pd(pid_control_V4_B.sinc,
        pid_control_V4_B.rtb_CoordinateTransformationC_l));
      _mm_storeu_pd(&pid_control_V4_B.dv1[0], tmp_3);
      pid_control_V4_B.rtb_CoordinateTransformationC_l = pid_control_V4_B.dv1[0];
      pid_control_V4_B.sinc = pid_control_V4_B.dv1[1];
      pid_control_V4_B.cosa += pid_control_V4_B.RotationAnglestoDirectionCo[3 *
        i + 2] * pid_control_V4_B.Product_l[i];
    }

    pid_control_V4_B.wbe_b[2] = pid_control_V4_B.cosa;
    pid_control_V4_B.wbe_b[1] = pid_control_V4_B.sinc;
    pid_control_V4_B.wbe_b[0] = pid_control_V4_B.rtb_CoordinateTransformationC_l;

    /* End of Product: '<S331>/Product' */
    /* End of Outputs for SubSystem: '<S304>/Low altitude  velocities' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S304>/Medium//High  altitude velocities' incorporates:
     *  ActionPort: '<S327>/Action Port'
     */
    /* Gain: '<S327>/Gain' */
    pid_control_V4_B.wbe_b[0] = pid_control_V4_B.w1_c[1];
    pid_control_V4_B.wbe_b[1] = pid_control_V4_B.w1[1];
    pid_control_V4_B.wbe_b[2] = pid_control_V4_B.LwgV1[1];

    /* End of Outputs for SubSystem: '<S304>/Medium//High  altitude velocities' */
    break;

   default:
    /* Outputs for IfAction SubSystem: '<S304>/Interpolate  velocities' incorporates:
     *  ActionPort: '<S325>/Action Port'
     */
    /* Trigonometry: '<S330>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S297>/Unit Conversion'
     */
    pid_control_V4_B.rtb_CoordinateTransformationC_l = sin
      (pid_control_V4_ConstB.UnitConversion);
    pid_control_V4_B.sinc = cos(pid_control_V4_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V4_B.wbe_b[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V4_B.rtb_CoordinateTransformationC_l, pid_control_V4_B.w1_c[0]),
      _mm_set_pd(pid_control_V4_B.w1_c[0], pid_control_V4_B.sinc)), _mm_mul_pd
      (_mm_mul_pd(_mm_set_pd(pid_control_V4_B.w1[0],
      pid_control_V4_B.rtb_CoordinateTransformationC_l), _mm_set_pd
                  (pid_control_V4_B.sinc, pid_control_V4_B.w1[0])), _mm_set_pd
       (1.0, -1.0))));

    /* SignalConversion generated from: '<S329>/Vector Concatenate' incorporates:
     *  Product: '<S330>/Product1'
     *  Product: '<S330>/Product2'
     *  Sum: '<S330>/Sum'
     */
    pid_control_V4_B.wbe_b[2] = pid_control_V4_B.LwgV1[0];

    /* Product: '<S329>/Product' incorporates:
     *  Angle2Dcm: '<S19>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S329>/Vector Concatenate'
     */
    pid_control_V4_B.rtb_CoordinateTransformationC_l = 0.0;
    pid_control_V4_B.sinc = 0.0;
    pid_control_V4_B.cosa = 0.0;
    for (i = 0; i < 3; i++) {
      tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V4_B.RotationAnglestoDirectionCo[3 * i]), _mm_set1_pd
        (pid_control_V4_B.wbe_b[i])), _mm_set_pd(pid_control_V4_B.sinc,
        pid_control_V4_B.rtb_CoordinateTransformationC_l));
      _mm_storeu_pd(&pid_control_V4_B.dv1[0], tmp_3);
      pid_control_V4_B.rtb_CoordinateTransformationC_l = pid_control_V4_B.dv1[0];
      pid_control_V4_B.sinc = pid_control_V4_B.dv1[1];
      pid_control_V4_B.cosa += pid_control_V4_B.RotationAnglestoDirectionCo[3 *
        i + 2] * pid_control_V4_B.wbe_b[i];
    }

    pid_control_V4_B.Product_l[2] = pid_control_V4_B.cosa;
    pid_control_V4_B.Product_l[1] = pid_control_V4_B.sinc;
    pid_control_V4_B.Product_l[0] =
      pid_control_V4_B.rtb_CoordinateTransformationC_l;
    tmp_3 = _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd(_mm_set_pd
      (pid_control_V4_B.w1[1], pid_control_V4_B.w1_c[1]), _mm_loadu_pd
      (&pid_control_V4_B.Product_l[0])), _mm_sub_pd(_mm_set1_pd
      (pid_control_V4_B.rtb_CoordinateTransformationC_d), _mm_set1_pd(1000.0))),
      _mm_set1_pd(pid_control_V4_ConstB.Sum)), _mm_loadu_pd
                       (&pid_control_V4_B.Product_l[0]));
    _mm_storeu_pd(&pid_control_V4_B.wbe_b[0], tmp_3);

    /* Sum: '<S325>/Sum3' incorporates:
     *  Constant: '<S325>/max_height_low'
     *  Product: '<S325>/Product1'
     *  Product: '<S329>/Product'
     *  Sum: '<S325>/Sum1'
     *  Sum: '<S325>/Sum2'
     */
    pid_control_V4_B.wbe_b[2] = (pid_control_V4_B.LwgV1[1] -
      pid_control_V4_B.cosa) * (pid_control_V4_B.rtb_CoordinateTransformationC_d
      - 1000.0) / pid_control_V4_ConstB.Sum + pid_control_V4_B.cosa;

    /* End of Outputs for SubSystem: '<S304>/Interpolate  velocities' */
    break;
  }

  /* UnitConversion: '<S290>/Unit Conversion' */
  /* Unit Conversion - from: ft/s to: m/s
     Expression: output = (0.3048*input) + (0) */
  tmp_3 = _mm_mul_pd(_mm_set1_pd(0.3048), _mm_loadu_pd(&pid_control_V4_B.wbe_b[0]));
  _mm_storeu_pd(&pid_control_V4_B.wbe_b[0], tmp_3);
  pid_control_V4_B.wbe_b[2] *= 0.3048;
  if (tmp_0) {
    /* MATLABSystem: '<S296>/SourceBlock' */
    pid_control_V4_B.SourceBlock_o1_h = Sub_pid_control_V4_417.getLatestMessage(
      &rtb_SourceBlock_o2_j);

    /* Outputs for Enabled SubSystem: '<S296>/Enabled Subsystem' */
    pid_control_V4_EnabledSubsystem(pid_control_V4_B.SourceBlock_o1_h,
      &rtb_SourceBlock_o2_j, &pid_control_V4_B.EnabledSubsystem_pt);

    /* End of Outputs for SubSystem: '<S296>/Enabled Subsystem' */
  }

  /* Switch: '<S19>/Switch' */
  if (pid_control_V4_B.EnabledSubsystem_pt.In1.data) {
    /* Switch: '<S19>/Switch' */
    pid_control_V4_B.Switch_p[0] = pid_control_V4_B.wbe_b[0];
    pid_control_V4_B.Switch_p[1] = pid_control_V4_B.wbe_b[1];
    pid_control_V4_B.Switch_p[2] = pid_control_V4_B.wbe_b[2];
  } else {
    /* Switch: '<S19>/Switch' incorporates:
     *  Constant: '<S19>/Constant'
     */
    pid_control_V4_B.Switch_p[0] = 0.0;
    pid_control_V4_B.Switch_p[1] = 0.0;
    pid_control_V4_B.Switch_p[2] = 0.0;
  }

  /* End of Switch: '<S19>/Switch' */

  /* Outputs for Enabled SubSystem: '<S298>/Hpgw' incorporates:
   *  EnablePort: '<S309>/Enable'
   */
  if (tmp_0 && tmp_1 && (!pid_control_V4_DW.Hpgw_MODE)) {
    (void) memset(&(pid_control_V4_XDis.pgw_p_CSTATE), 0,
                  2*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S309>/pgw_p' */
    pid_control_V4_X.pgw_p_CSTATE[0] = 0.0;
    pid_control_V4_X.pgw_p_CSTATE[1] = 0.0;
    pid_control_V4_DW.Hpgw_MODE = true;
  }

  if (pid_control_V4_DW.Hpgw_MODE) {
    /* Fcn: '<S309>/sqrt(0.8//V)' */
    pid_control_V4_B.w_r = sqrt(0.8 / pid_control_V4_B.sina);

    /* Product: '<S309>/w3' */
    pid_control_V4_B.rtb_CoordinateTransformationC_l = pid_control_V4_B.sina *
      pid_control_V4_ConstB.w4;

    /* Product: '<S309>/w' incorporates:
     *  Fcn: '<S309>/sqrt(0.8//V)'
     *  Gain: '<S305>/Lw'
     *  Integrator: '<S309>/pgw_p'
     *  Math: '<S309>/L^1//3'
     *  Product: '<S309>/Lug//V1'
     *  Product: '<S309>/w1'
     *  Product: '<S309>/w2'
     *  Sum: '<S309>/Sum'
     */
    pid_control_V4_B.w_o[0] = (pid_control_V4_B.w_r / rt_powd_snf
      (pid_control_V4_B.sinb, 0.33333333333333331) * pid_control_V4_ConstB.u16 *
      pid_control_V4_B.Product[3] - pid_control_V4_X.pgw_p_CSTATE[0]) *
      pid_control_V4_B.rtb_CoordinateTransformationC_l;

    /* Math: '<S309>/L^1//3' */
    if (pid_control_V4_B.frac[1] < 0.0) {
      pid_control_V4_B.sinb = -rt_powd_snf(-pid_control_V4_B.frac[1],
        0.33333333333333331);
    } else {
      pid_control_V4_B.sinb = rt_powd_snf(pid_control_V4_B.frac[1],
        0.33333333333333331);
    }

    /* Product: '<S309>/w' incorporates:
     *  Fcn: '<S309>/sqrt(0.8//V)'
     *  Integrator: '<S309>/pgw_p'
     *  Math: '<S309>/L^1//3'
     *  Product: '<S309>/Lug//V1'
     *  Product: '<S309>/w1'
     *  Product: '<S309>/w2'
     *  Sum: '<S309>/Sum'
     */
    pid_control_V4_B.w_o[1] = (pid_control_V4_B.w_r / pid_control_V4_B.sinb *
      pid_control_V4_ConstB.u16 * pid_control_V4_B.Product[3] -
      pid_control_V4_X.pgw_p_CSTATE[1]) *
      pid_control_V4_B.rtb_CoordinateTransformationC_l;

    /* Product: '<S309>/sigma_w' incorporates:
     *  Integrator: '<S309>/pgw_p'
     */
    tmp_3 = _mm_mul_pd(_mm_set_pd
                       (pid_control_V4_B.rtb_CoordinateTransformationC_g,
                        pid_control_V4_ConstB.sigma_wg), _mm_loadu_pd
                       (&pid_control_V4_X.pgw_p_CSTATE[0]));

    /* Product: '<S309>/sigma_w' */
    _mm_storeu_pd(&pid_control_V4_B.sigma_w[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S298>/Hpgw' */

  /* Outputs for Enabled SubSystem: '<S298>/Hqgw' incorporates:
   *  EnablePort: '<S310>/Enable'
   */
  if (tmp_0 && tmp_1 && (!pid_control_V4_DW.Hqgw_MODE)) {
    (void) memset(&(pid_control_V4_XDis.qgw_p_CSTATE), 0,
                  2*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S310>/qgw_p' */
    pid_control_V4_X.qgw_p_CSTATE[0] = 0.0;
    pid_control_V4_X.qgw_p_CSTATE[1] = 0.0;
    pid_control_V4_DW.Hqgw_MODE = true;
  }

  if (pid_control_V4_DW.Hqgw_MODE) {
    /* Gain: '<S310>/pi//4' */
    pid_control_V4_B.sinb = 0.78539816339744828 * pid_control_V4_B.sina;

    /* Product: '<S310>/w' incorporates:
     *  Integrator: '<S310>/qgw_p'
     *  Product: '<S310>/wg//V'
     *  Sum: '<S310>/Sum'
     */
    pid_control_V4_B.rtb_CoordinateTransformationC_g = (pid_control_V4_B.LwgV1[0]
      / pid_control_V4_B.sina - pid_control_V4_X.qgw_p_CSTATE[0]) *
      (pid_control_V4_B.sinb / pid_control_V4_ConstB.UnitConversion_n);
    pid_control_V4_B.w_e0[0] = pid_control_V4_B.rtb_CoordinateTransformationC_g;

    /* UnaryMinus: '<S310>/Unary Minus' */
    pid_control_V4_B.UnaryMinus[0] =
      -pid_control_V4_B.rtb_CoordinateTransformationC_g;

    /* Product: '<S310>/w' incorporates:
     *  Integrator: '<S310>/qgw_p'
     *  Product: '<S310>/wg//V'
     *  Sum: '<S310>/Sum'
     */
    pid_control_V4_B.rtb_CoordinateTransformationC_g = (pid_control_V4_B.LwgV1[1]
      / pid_control_V4_B.sina - pid_control_V4_X.qgw_p_CSTATE[1]) *
      (pid_control_V4_B.sinb / pid_control_V4_ConstB.UnitConversion_n);
    pid_control_V4_B.w_e0[1] = pid_control_V4_B.rtb_CoordinateTransformationC_g;

    /* UnaryMinus: '<S310>/Unary Minus' */
    pid_control_V4_B.UnaryMinus[1] =
      -pid_control_V4_B.rtb_CoordinateTransformationC_g;
  }

  /* End of Outputs for SubSystem: '<S298>/Hqgw' */

  /* Outputs for Enabled SubSystem: '<S298>/Hrgw' incorporates:
   *  EnablePort: '<S311>/Enable'
   */
  if (tmp_0 && tmp_1 && (!pid_control_V4_DW.Hrgw_MODE)) {
    (void) memset(&(pid_control_V4_XDis.rgw_p_CSTATE), 0,
                  2*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S311>/rgw_p' */
    pid_control_V4_X.rgw_p_CSTATE[0] = 0.0;
    pid_control_V4_X.rgw_p_CSTATE[1] = 0.0;
    pid_control_V4_DW.Hrgw_MODE = true;
  }

  if (pid_control_V4_DW.Hrgw_MODE) {
    /* Product: '<S311>/vg//V' incorporates:
     *  Gain: '<S311>/pi//3'
     *  Integrator: '<S311>/rgw_p'
     *  Product: '<S311>/w'
     */
    tmp_3 = _mm_mul_pd(_mm_sub_pd(_mm_div_pd(_mm_loadu_pd(&pid_control_V4_B.w1[0]),
      _mm_set1_pd(pid_control_V4_B.sina)), _mm_loadu_pd
      (&pid_control_V4_X.rgw_p_CSTATE[0])), _mm_div_pd(_mm_set1_pd
      (1.0471975511965976 * pid_control_V4_B.sina), _mm_set1_pd
      (pid_control_V4_ConstB.UnitConversion_n)));

    /* Product: '<S311>/w' */
    _mm_storeu_pd(&pid_control_V4_B.w_d[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S298>/Hrgw' */

  /* If: '<S303>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
   *  Constant: '<S317>/max_height_low'
   *  Product: '<S317>/Product1'
   *  Product: '<S322>/Product1'
   *  Product: '<S322>/Product2'
   *  Product: '<S324>/Product1'
   *  Product: '<S324>/Product2'
   *  Sum: '<S317>/Sum1'
   *  Sum: '<S317>/Sum2'
   *  Sum: '<S317>/Sum3'
   *  Sum: '<S322>/Sum'
   *  Sum: '<S324>/Sum'
   */
  rtPrevAction = pid_control_V4_DW.ifHeightMaxlowaltitudeelseifH_a;
  if (tmp_1) {
    if (pid_control_V4_B.rtb_CoordinateTransformationC_d <= 1000.0) {
      rtAction = 0;
    } else if (pid_control_V4_B.rtb_CoordinateTransformationC_d >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    pid_control_V4_DW.ifHeightMaxlowaltitudeelseifH_a = rtAction;
  } else {
    rtAction = pid_control_V4_DW.ifHeightMaxlowaltitudeelseifH_a;
  }

  if (rtPrevAction != rtAction) {
    rtsiSetBlockStateForSolverChangedAtMajorStep(&(&pid_control_V4_M)
      ->solverInfo, true);
  }

  switch (rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S303>/Low altitude  rates' incorporates:
     *  ActionPort: '<S318>/Action Port'
     */
    /* SignalConversion generated from: '<S323>/Vector Concatenate' */
    pid_control_V4_B.Product_l[2] = pid_control_V4_B.w_d[0];

    /* Trigonometry: '<S324>/Trigonometric Function1' incorporates:
     *  UnitConversion: '<S297>/Unit Conversion'
     */
    pid_control_V4_B.rtb_CoordinateTransformationC_d = sin
      (pid_control_V4_ConstB.UnitConversion);
    pid_control_V4_B.sina = cos(pid_control_V4_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V4_B.Product_l[0], _mm_add_pd(_mm_mul_pd
      (_mm_set_pd(pid_control_V4_B.rtb_CoordinateTransformationC_d,
                  pid_control_V4_B.sigma_w[0]), _mm_set_pd
       (pid_control_V4_B.sigma_w[0], pid_control_V4_B.sina)), _mm_mul_pd
      (_mm_mul_pd(_mm_set_pd(pid_control_V4_B.UnaryMinus[0],
      pid_control_V4_B.rtb_CoordinateTransformationC_d), _mm_set_pd
                  (pid_control_V4_B.sina, pid_control_V4_B.UnaryMinus[0])),
       _mm_set_pd(1.0, -1.0))));

    /* Product: '<S323>/Product' incorporates:
     *  Angle2Dcm: '<S19>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S323>/Vector Concatenate'
     *  Product: '<S324>/Product1'
     *  Product: '<S324>/Product2'
     *  Reshape: '<S323>/Reshape1'
     *  Sum: '<S324>/Sum'
     */
    pid_control_V4_B.rtb_CoordinateTransformationC_l = 0.0;
    pid_control_V4_B.sinc = 0.0;
    pid_control_V4_B.cosa = 0.0;
    for (i = 0; i < 3; i++) {
      tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V4_B.RotationAnglestoDirectionCo[3 * i]), _mm_set1_pd
        (pid_control_V4_B.Product_l[i])), _mm_set_pd(pid_control_V4_B.sinc,
        pid_control_V4_B.rtb_CoordinateTransformationC_l));
      _mm_storeu_pd(&pid_control_V4_B.dv1[0], tmp_3);
      pid_control_V4_B.rtb_CoordinateTransformationC_l = pid_control_V4_B.dv1[0];
      pid_control_V4_B.sinc = pid_control_V4_B.dv1[1];
      pid_control_V4_B.cosa += pid_control_V4_B.RotationAnglestoDirectionCo[3 *
        i + 2] * pid_control_V4_B.Product_l[i];
    }

    pid_control_V4_B.wbe_b[2] = pid_control_V4_B.cosa;
    pid_control_V4_B.wbe_b[1] = pid_control_V4_B.sinc;
    pid_control_V4_B.wbe_b[0] = pid_control_V4_B.rtb_CoordinateTransformationC_l;

    /* End of Product: '<S323>/Product' */
    /* End of Outputs for SubSystem: '<S303>/Low altitude  rates' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S303>/Medium//High  altitude rates' incorporates:
     *  ActionPort: '<S319>/Action Port'
     */
    /* Gain: '<S319>/Gain' */
    pid_control_V4_B.wbe_b[0] = pid_control_V4_B.sigma_w[1];
    pid_control_V4_B.wbe_b[1] = pid_control_V4_B.UnaryMinus[1];
    pid_control_V4_B.wbe_b[2] = pid_control_V4_B.w_d[1];

    /* End of Outputs for SubSystem: '<S303>/Medium//High  altitude rates' */
    break;

   default:
    /* Outputs for IfAction SubSystem: '<S303>/Interpolate  rates' incorporates:
     *  ActionPort: '<S317>/Action Port'
     */
    /* Trigonometry: '<S322>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S297>/Unit Conversion'
     */
    pid_control_V4_B.sina = sin(pid_control_V4_ConstB.UnitConversion);
    pid_control_V4_B.sinb = cos(pid_control_V4_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V4_B.wbe_b[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V4_B.sina, pid_control_V4_B.sigma_w[0]), _mm_set_pd
      (pid_control_V4_B.sigma_w[0], pid_control_V4_B.sinb)), _mm_mul_pd
      (_mm_mul_pd(_mm_set_pd(pid_control_V4_B.UnaryMinus[0],
      pid_control_V4_B.sina), _mm_set_pd(pid_control_V4_B.sinb,
      pid_control_V4_B.UnaryMinus[0])), _mm_set_pd(1.0, -1.0))));

    /* SignalConversion generated from: '<S321>/Vector Concatenate' incorporates:
     *  Product: '<S322>/Product1'
     *  Product: '<S322>/Product2'
     *  Sum: '<S322>/Sum'
     */
    pid_control_V4_B.wbe_b[2] = pid_control_V4_B.w_d[0];

    /* Product: '<S321>/Product' incorporates:
     *  Angle2Dcm: '<S19>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S321>/Vector Concatenate'
     */
    pid_control_V4_B.rtb_CoordinateTransformationC_l = 0.0;
    pid_control_V4_B.sinc = 0.0;
    pid_control_V4_B.cosa = 0.0;
    for (i = 0; i < 3; i++) {
      tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V4_B.RotationAnglestoDirectionCo[3 * i]), _mm_set1_pd
        (pid_control_V4_B.wbe_b[i])), _mm_set_pd(pid_control_V4_B.sinc,
        pid_control_V4_B.rtb_CoordinateTransformationC_l));
      _mm_storeu_pd(&pid_control_V4_B.dv1[0], tmp_3);
      pid_control_V4_B.rtb_CoordinateTransformationC_l = pid_control_V4_B.dv1[0];
      pid_control_V4_B.sinc = pid_control_V4_B.dv1[1];
      pid_control_V4_B.cosa += pid_control_V4_B.RotationAnglestoDirectionCo[3 *
        i + 2] * pid_control_V4_B.wbe_b[i];
    }

    pid_control_V4_B.Product_l[2] = pid_control_V4_B.cosa;
    pid_control_V4_B.Product_l[1] = pid_control_V4_B.sinc;
    pid_control_V4_B.Product_l[0] =
      pid_control_V4_B.rtb_CoordinateTransformationC_l;
    tmp_3 = _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd(_mm_set_pd
      (pid_control_V4_B.UnaryMinus[1], pid_control_V4_B.sigma_w[1]),
      _mm_loadu_pd(&pid_control_V4_B.Product_l[0])), _mm_sub_pd(_mm_set1_pd
      (pid_control_V4_B.rtb_CoordinateTransformationC_d), _mm_set1_pd(1000.0))),
      _mm_set1_pd(pid_control_V4_ConstB.Sum_a)), _mm_loadu_pd
                       (&pid_control_V4_B.Product_l[0]));
    _mm_storeu_pd(&pid_control_V4_B.wbe_b[0], tmp_3);

    /* Sum: '<S317>/Sum3' incorporates:
     *  Constant: '<S317>/max_height_low'
     *  Product: '<S317>/Product1'
     *  Product: '<S321>/Product'
     *  Sum: '<S317>/Sum1'
     *  Sum: '<S317>/Sum2'
     */
    pid_control_V4_B.wbe_b[2] = (pid_control_V4_B.w_d[1] - pid_control_V4_B.cosa)
      * (pid_control_V4_B.rtb_CoordinateTransformationC_d - 1000.0) /
      pid_control_V4_ConstB.Sum_a + pid_control_V4_B.cosa;

    /* End of Outputs for SubSystem: '<S303>/Interpolate  rates' */
    break;
  }

  if (tmp_0) {
    /* MATLABSystem: '<S293>/SourceBlock' */
    pid_control_V4_B.SourceBlock_o1_k = Sub_pid_control_V4_423.getLatestMessage(
      &rtb_SourceBlock_o2_dd);

    /* Outputs for Enabled SubSystem: '<S293>/Enabled Subsystem' */
    pid_control_V4_EnabledSubsystem(pid_control_V4_B.SourceBlock_o1_k,
      &rtb_SourceBlock_o2_dd, &pid_control_V4_B.EnabledSubsystem);

    /* End of Outputs for SubSystem: '<S293>/Enabled Subsystem' */
  }

  /* Switch: '<S19>/Switch1' */
  if (pid_control_V4_B.EnabledSubsystem.In1.data) {
    /* Switch: '<S19>/Switch1' */
    pid_control_V4_B.Switch1[0] = pid_control_V4_B.wbe_b[0];
    pid_control_V4_B.Switch1[1] = pid_control_V4_B.wbe_b[1];
    pid_control_V4_B.Switch1[2] = pid_control_V4_B.wbe_b[2];
  } else {
    /* Switch: '<S19>/Switch1' incorporates:
     *  Constant: '<S19>/Constant2'
     */
    pid_control_V4_B.Switch1[0] = 0.0;
    pid_control_V4_B.Switch1[1] = 0.0;
    pid_control_V4_B.Switch1[2] = 0.0;
  }

  /* End of Switch: '<S19>/Switch1' */
  if (tmp_0) {
    /* MATLABSystem: '<S294>/SourceBlock' */
    pid_control_V4_B.SourceBlock_o1_c = Sub_pid_control_V4_443.getLatestMessage(
      &pid_control_V4_B.SourceBlock_o2_p);

    /* Outputs for Enabled SubSystem: '<S294>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V4_B.SourceBlock_o1_c,
      &pid_control_V4_B.SourceBlock_o2_p, &pid_control_V4_B.EnabledSubsystem_k);

    /* End of Outputs for SubSystem: '<S294>/Enabled Subsystem' */

    /* SignalConversion generated from: '<S19>/Bus Selector2' */
    pid_control_V4_B.data = pid_control_V4_B.EnabledSubsystem_k.In1.data;

    /* MATLABSystem: '<S295>/SourceBlock' */
    pid_control_V4_B.SourceBlock_o1_d = Sub_pid_control_V4_445.getLatestMessage(
      &pid_control_V4_B.SourceBlock_o2_k);

    /* Outputs for Enabled SubSystem: '<S295>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V4_B.SourceBlock_o1_d,
      &pid_control_V4_B.SourceBlock_o2_k, &pid_control_V4_B.EnabledSubsystem_p);

    /* End of Outputs for SubSystem: '<S295>/Enabled Subsystem' */

    /* SignalConversion generated from: '<S19>/Bus Selector3' */
    pid_control_V4_B.data_n = pid_control_V4_B.EnabledSubsystem_p.In1.data;
  }

  /* Product: '<S19>/Product2' incorporates:
   *  Math: '<S19>/Square'
   *  Math: '<S19>/Square1'
   *  Math: '<S19>/Square2'
   *  Sqrt: '<S19>/Sqrt'
   *  Sum: '<S19>/Sum2'
   */
  pid_control_V4_B.Power = sqrt((pid_control_V4_B.x[0] * pid_control_V4_B.x[0] +
    pid_control_V4_B.x[1] * pid_control_V4_B.x[1]) + pid_control_V4_B.x[2] *
    pid_control_V4_B.x[2]) * pid_control_V4_B.XDOT[34];

  /* Gain: '<S19>/Gain3' */
  pid_control_V4_B.Gain3 = 0.001 * pid_control_V4_B.Power;
  if (tmp_0) {
  }

  /* Gain: '<S19>/Gain1' incorporates:
   *  Integrator: '<S19>/Integrator1'
   */
  pid_control_V4_B.EnergykWh = 2.7777777777777776E-7 *
    pid_control_V4_X.Integrator1_CSTATE;
  if (tmp_0) {
    /* SignalConversion generated from: '<S19>/To Workspace1' */
    pid_control_V4_B.TmpSignalConversionAtSFunct[0] =
      pid_control_V4_B.Saturation_k;
    pid_control_V4_B.TmpSignalConversionAtSFunct[1] =
      pid_control_V4_B.Saturation_fh;
    pid_control_V4_B.TmpSignalConversionAtSFunct[2] =
      pid_control_V4_B.Saturation_m;
    pid_control_V4_B.TmpSignalConversionAtSFunct[3] =
      pid_control_V4_B.Saturation;
    pid_control_V4_B.TmpSignalConversionAtSFunct[4] =
      pid_control_V4_B.Saturation;
  }

  /* Product: '<S19>/Divide' incorporates:
   *  Constant: '<S19>/thrust efficiency Cp?'
   */
  pid_control_V4_B.powerdemand = pid_control_V4_B.Gain3 / 0.57;
  if (tmp_0) {
  }

  /* Product: '<S19>/Divide1' */
  pid_control_V4_B.loadtorque = pid_control_V4_B.powerdemand /
    pid_control_V4_ConstB.motorspeed;
  if (tmp_0) {
    /* Gain: '<S288>/Output' incorporates:
     *  RandomNumber: '<S288>/White Noise'
     */
    pid_control_V4_B.Output = 10.0 * pid_control_V4_DW.NextOutput_k;
  }

  /* TransferFcn: '<S19>/Transfer Fcn' */
  pid_control_V4_B.sinb = 0.5303 * pid_control_V4_X.TransferFcn_CSTATE[0] + 0.0 *
    pid_control_V4_X.TransferFcn_CSTATE[1];

  /* Switch: '<S19>/Switch2' incorporates:
   *  Constant: '<S19>/Constant3'
   */
  if (!(pid_control_V4_B.data != 0.0)) {
    pid_control_V4_B.sinb = 0.0;
  }

  /* End of Switch: '<S19>/Switch2' */

  /* Sum: '<S19>/Sum' */
  pid_control_V4_B.Sum[0] = pid_control_V4_B.Switch_p[0];
  pid_control_V4_B.Sum[1] = pid_control_V4_B.Switch_p[1] + pid_control_V4_B.sinb;
  pid_control_V4_B.Sum[2] = pid_control_V4_B.Switch_p[2];

  /* Sum: '<S19>/Sum1' */
  pid_control_V4_B.Sum1[0] = pid_control_V4_B.Switch1[0];

  /* Switch: '<S19>/Switch3' incorporates:
   *  Constant: '<S19>/Constant4'
   *  TransferFcn: '<S19>/Transfer Fcn1'
   */
  if (pid_control_V4_B.data_n != 0.0) {
    pid_control_V4_B.w_r = -0.0003571 * pid_control_V4_X.TransferFcn1_CSTATE +
      0.03571 * pid_control_V4_B.Output;
  } else {
    pid_control_V4_B.w_r = 0.0;
  }

  /* Sum: '<S19>/Sum1' incorporates:
   *  Switch: '<S19>/Switch3'
   */
  pid_control_V4_B.Sum1[1] = pid_control_V4_B.Switch1[1] + pid_control_V4_B.w_r;
  pid_control_V4_B.Sum1[2] = pid_control_V4_B.Switch1[2];
  if (rtmIsMajorTimeStep((&pid_control_V4_M))) {
    if (rtmIsMajorTimeStep((&pid_control_V4_M))) {
      /* Update for Memory: '<S19>/Memory2' incorporates:
       *  Integrator: '<S19>/Integrator'
       */
      memcpy(&pid_control_V4_DW.Memory2_PreviousInput[0], &pid_control_V4_B.x[0],
             12U * sizeof(real_T));

      /* Update for UnitDelay: '<Root>/Unit Delay3' */
      pid_control_V4_DW.UnitDelay3_DSTATE = pid_control_V4_B.Switch3;

      /* Update for UnitDelay: '<Root>/Unit Delay2' */
      pid_control_V4_DW.UnitDelay2_DSTATE = pid_control_V4_B.Switch2;

      /* Update for Memory: '<S19>/Memory' incorporates:
       *  Sum: '<S19>/Sum'
       */
      pid_control_V4_DW.Memory_PreviousInput[0] = pid_control_V4_B.Sum[0];

      /* Update for Memory: '<S19>/Memory1' incorporates:
       *  Sum: '<S19>/Sum1'
       */
      pid_control_V4_DW.Memory1_PreviousInput[0] = pid_control_V4_B.Sum1[0];

      /* Update for Memory: '<S19>/Memory' incorporates:
       *  Sum: '<S19>/Sum'
       */
      pid_control_V4_DW.Memory_PreviousInput[1] = pid_control_V4_B.Sum[1];

      /* Update for Memory: '<S19>/Memory1' incorporates:
       *  Sum: '<S19>/Sum1'
       */
      pid_control_V4_DW.Memory1_PreviousInput[1] = pid_control_V4_B.Sum1[1];

      /* Update for Memory: '<S19>/Memory' incorporates:
       *  Sum: '<S19>/Sum'
       */
      pid_control_V4_DW.Memory_PreviousInput[2] = pid_control_V4_B.Sum[2];

      /* Update for Memory: '<S19>/Memory1' incorporates:
       *  Sum: '<S19>/Sum1'
       */
      pid_control_V4_DW.Memory1_PreviousInput[2] = pid_control_V4_B.Sum1[2];

      /* Update for Memory: '<S48>/Memory' */
      pid_control_V4_DW.Memory_PreviousInput_o = pid_control_V4_B.AND3;

      /* Update for Memory: '<S154>/Memory' */
      pid_control_V4_DW.Memory_PreviousInput_d = pid_control_V4_B.AND3_j;

      /* Update for Memory: '<S260>/Memory' */
      pid_control_V4_DW.Memory_PreviousInput_a = pid_control_V4_B.AND3_c;

      /* Update for RandomNumber: '<S308>/White Noise' */
      pid_control_V4_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V4_DW.RandSeed[0]);
      pid_control_V4_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V4_DW.RandSeed[1]);
      pid_control_V4_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V4_DW.RandSeed[2]);
      pid_control_V4_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V4_DW.RandSeed[3]);

      /* Update for RandomNumber: '<S288>/White Noise' */
      pid_control_V4_DW.NextOutput_k = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V4_DW.RandSeed_a);
    }

    /* Update for Integrator: '<S19>/Integrator' */
    pid_control_V4_DW.Integrator_DWORK1 = false;

    /* Update for RateLimiter: '<Root>/Rate Limiter-theta_sp' */
    pid_control_V4_DW.PrevY_g = pid_control_V4_B.RateLimitertheta_sp;
    pid_control_V4_DW.LastMajorTime = (&pid_control_V4_M)->Timing.t[0];

    /* ContTimeOutputInconsistentWithStateAtMajorOutputFlag is set, need to run a minor output */
    if (rtmIsMajorTimeStep((&pid_control_V4_M))) {
      if (rtsiGetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&pid_control_V4_M)->solverInfo)) {
        rtsiSetSimTimeStep(&(&pid_control_V4_M)->solverInfo,MINOR_TIME_STEP);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&pid_control_V4_M)->solverInfo, false);
        pid_control_V4::step();
        rtsiSetSimTimeStep(&(&pid_control_V4_M)->solverInfo, MAJOR_TIME_STEP);
      }
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep((&pid_control_V4_M))) {
    rt_ertODEUpdateContinuousStates(&(&pid_control_V4_M)->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&pid_control_V4_M)->Timing.clockTick0)) {
      ++(&pid_control_V4_M)->Timing.clockTickH0;
    }

    (&pid_control_V4_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&pid_control_V4_M)
      ->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      (&pid_control_V4_M)->Timing.clockTick1++;
      if (!(&pid_control_V4_M)->Timing.clockTick1) {
        (&pid_control_V4_M)->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void pid_control_V4::pid_control_V4_derivatives()
{
  XDot_pid_control_V4_T *_rtXdot;
  real_T tmp[2];
  _rtXdot = ((XDot_pid_control_V4_T *) (&pid_control_V4_M)->derivs);

  /* Derivatives for Integrator: '<S19>/Integrator' */
  if (!pid_control_V4_B.Compare) {
    memcpy(&_rtXdot->Integrator_CSTATE[0], &pid_control_V4_B.XDOT[0], 12U *
           sizeof(real_T));
  } else {
    /* level reset is active */
    memset(&_rtXdot->Integrator_CSTATE[0], 0, 12U * sizeof(real_T));
  }

  /* End of Derivatives for Integrator: '<S19>/Integrator' */

  /* Derivatives for Integrator: '<S270>/Integrator' */
  _rtXdot->Integrator_CSTATE_f = pid_control_V4_B.Switch_j;

  /* Derivatives for Integrator: '<S265>/Filter' */
  _rtXdot->Filter_CSTATE = pid_control_V4_B.FilterCoefficient;

  /* Derivatives for Integrator: '<S110>/Integrator' */
  _rtXdot->Integrator_CSTATE_n = pid_control_V4_B.SumI4;

  /* Derivatives for Integrator: '<S105>/Filter' */
  _rtXdot->Filter_CSTATE_f = pid_control_V4_B.FilterCoefficient_j;

  /* Derivatives for Integrator: '<S216>/Integrator' */
  _rtXdot->Integrator_CSTATE_d = pid_control_V4_B.SumI4_d;

  /* Derivatives for Integrator: '<S211>/Filter' */
  _rtXdot->Filter_CSTATE_fd = pid_control_V4_B.FilterCoefficient_p;

  /* Derivatives for Integrator: '<S58>/Integrator' */
  _rtXdot->Integrator_CSTATE_m = pid_control_V4_B.Switch;

  /* Derivatives for Integrator: '<S53>/Filter' */
  _rtXdot->Filter_CSTATE_g = pid_control_V4_B.FilterCoefficient_c;

  /* Derivatives for Integrator: '<S164>/Integrator' */
  _rtXdot->Integrator_CSTATE_p = pid_control_V4_B.Switch_a;

  /* Derivatives for Integrator: '<S159>/Filter' */
  _rtXdot->Filter_CSTATE_m = pid_control_V4_B.FilterCoefficient_m;

  /* Derivatives for Enabled SubSystem: '<S299>/Hugw(s)' */
  if (pid_control_V4_DW.Hugws_MODE) {
    /* Derivatives for Integrator: '<S312>/ug_p' */
    _rtXdot->ug_p_CSTATE[0] = pid_control_V4_B.w_n[0];
    _rtXdot->ug_p_CSTATE[1] = pid_control_V4_B.w_n[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V4_T *) (&pid_control_V4_M)->derivs)
             ->ug_p_CSTATE[0]);
      for (i1=0; i1 < 2; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S299>/Hugw(s)' */

  /* Derivatives for Enabled SubSystem: '<S299>/Hvgw(s)' */
  if (pid_control_V4_DW.Hvgws_MODE) {
    /* Derivatives for Integrator: '<S313>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[0] = pid_control_V4_B.w_g[0];

    /* Derivatives for Integrator: '<S313>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[0] = pid_control_V4_B.w_e[0];

    /* Derivatives for Integrator: '<S313>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[1] = pid_control_V4_B.w_g[1];

    /* Derivatives for Integrator: '<S313>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[1] = pid_control_V4_B.w_e[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V4_T *) (&pid_control_V4_M)->derivs)
             ->vg_p1_CSTATE[0]);
      for (i1=0; i1 < 4; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S299>/Hvgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S299>/Hwgw(s)' */
  if (pid_control_V4_DW.Hwgws_MODE) {
    /* Derivatives for Integrator: '<S314>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[0] = pid_control_V4_B.w[0];

    /* Derivatives for Integrator: '<S314>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[0] = pid_control_V4_B.w_a[0];

    /* Derivatives for Integrator: '<S314>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[1] = pid_control_V4_B.w[1];

    /* Derivatives for Integrator: '<S314>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[1] = pid_control_V4_B.w_a[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V4_T *) (&pid_control_V4_M)->derivs)
             ->wg_p1_CSTATE[0]);
      for (i1=0; i1 < 4; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S299>/Hwgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S298>/Hpgw' */
  if (pid_control_V4_DW.Hpgw_MODE) {
    /* Derivatives for Integrator: '<S309>/pgw_p' */
    _rtXdot->pgw_p_CSTATE[0] = pid_control_V4_B.w_o[0];
    _rtXdot->pgw_p_CSTATE[1] = pid_control_V4_B.w_o[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V4_T *) (&pid_control_V4_M)->derivs)
             ->pgw_p_CSTATE[0]);
      for (i1=0; i1 < 2; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S298>/Hpgw' */

  /* Derivatives for Enabled SubSystem: '<S298>/Hqgw' */
  if (pid_control_V4_DW.Hqgw_MODE) {
    /* Derivatives for Integrator: '<S310>/qgw_p' */
    _rtXdot->qgw_p_CSTATE[0] = pid_control_V4_B.w_e0[0];
    _rtXdot->qgw_p_CSTATE[1] = pid_control_V4_B.w_e0[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V4_T *) (&pid_control_V4_M)->derivs)
             ->qgw_p_CSTATE[0]);
      for (i1=0; i1 < 2; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S298>/Hqgw' */

  /* Derivatives for Enabled SubSystem: '<S298>/Hrgw' */
  if (pid_control_V4_DW.Hrgw_MODE) {
    /* Derivatives for Integrator: '<S311>/rgw_p' */
    _rtXdot->rgw_p_CSTATE[0] = pid_control_V4_B.w_d[0];
    _rtXdot->rgw_p_CSTATE[1] = pid_control_V4_B.w_d[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V4_T *) (&pid_control_V4_M)->derivs)
             ->rgw_p_CSTATE[0]);
      for (i1=0; i1 < 2; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S298>/Hrgw' */

  /* Derivatives for Integrator: '<S19>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = pid_control_V4_B.Power;

  /* Derivatives for TransferFcn: '<S19>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE[0] = 0.0;
  _rtXdot->TransferFcn_CSTATE[0] += -0.898 *
    pid_control_V4_X.TransferFcn_CSTATE[0];
  _rtXdot->TransferFcn_CSTATE[1] = 0.0;
  _rtXdot->TransferFcn_CSTATE[0] += -0.806 *
    pid_control_V4_X.TransferFcn_CSTATE[1];
  _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_set_pd(_rtXdot->TransferFcn_CSTATE[0],
    pid_control_V4_X.TransferFcn_CSTATE[0]), _mm_set_pd(pid_control_V4_B.Output,
    _rtXdot->TransferFcn_CSTATE[1])));
  _rtXdot->TransferFcn_CSTATE[1] = tmp[0];
  _rtXdot->TransferFcn_CSTATE[0] = tmp[1];

  /* Derivatives for TransferFcn: '<S19>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE = 0.0;
  _rtXdot->TransferFcn1_CSTATE += -0.01 * pid_control_V4_X.TransferFcn1_CSTATE;
  _rtXdot->TransferFcn1_CSTATE += pid_control_V4_B.Output;
}

/* Model initialize function */
void pid_control_V4::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&pid_control_V4_M)->solverInfo, &(&pid_control_V4_M
                          )->Timing.simTimeStep);
    rtsiSetTPtr(&(&pid_control_V4_M)->solverInfo, &rtmGetTPtr((&pid_control_V4_M)));
    rtsiSetStepSizePtr(&(&pid_control_V4_M)->solverInfo, &(&pid_control_V4_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&pid_control_V4_M)->solverInfo, &(&pid_control_V4_M)->derivs);
    rtsiSetContStatesPtr(&(&pid_control_V4_M)->solverInfo, (real_T **)
                         &(&pid_control_V4_M)->contStates);
    rtsiSetNumContStatesPtr(&(&pid_control_V4_M)->solverInfo,
      &(&pid_control_V4_M)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&pid_control_V4_M)->solverInfo,
      &(&pid_control_V4_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&pid_control_V4_M)->solverInfo,
      &(&pid_control_V4_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&pid_control_V4_M)->solverInfo,
      &(&pid_control_V4_M)->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&(&pid_control_V4_M)->solverInfo, (boolean_T**)
      &(&pid_control_V4_M)->contStateDisabled);
    rtsiSetErrorStatusPtr(&(&pid_control_V4_M)->solverInfo, (&rtmGetErrorStatus
      ((&pid_control_V4_M))));
    rtsiSetRTModelPtr(&(&pid_control_V4_M)->solverInfo, (&pid_control_V4_M));
  }

  rtsiSetSimTimeStep(&(&pid_control_V4_M)->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&(&pid_control_V4_M)->solverInfo, false);
  rtsiSetIsContModeFrozen(&(&pid_control_V4_M)->solverInfo, false);
  (&pid_control_V4_M)->intgData.y = (&pid_control_V4_M)->odeY;
  (&pid_control_V4_M)->intgData.f[0] = (&pid_control_V4_M)->odeF[0];
  (&pid_control_V4_M)->intgData.f[1] = (&pid_control_V4_M)->odeF[1];
  (&pid_control_V4_M)->intgData.f[2] = (&pid_control_V4_M)->odeF[2];
  (&pid_control_V4_M)->intgData.f[3] = (&pid_control_V4_M)->odeF[3];
  (&pid_control_V4_M)->contStates = ((X_pid_control_V4_T *) &pid_control_V4_X);
  (&pid_control_V4_M)->contStateDisabled = ((XDis_pid_control_V4_T *)
    &pid_control_V4_XDis);
  (&pid_control_V4_M)->Timing.tStart = (0.0);
  rtsiSetSolverData(&(&pid_control_V4_M)->solverInfo, static_cast<void *>
                    (&(&pid_control_V4_M)->intgData));
  rtsiSetSolverName(&(&pid_control_V4_M)->solverInfo,"ode4");
  rtmSetTPtr((&pid_control_V4_M), &(&pid_control_V4_M)->Timing.tArray[0]);
  (&pid_control_V4_M)->Timing.stepSize0 = 0.01;
  rtmSetFirstInitCond((&pid_control_V4_M), 1);

  {
    rmw_qos_profile_t qos_profile;
    int32_T i;
    static const char_T b_zeroDelimTopic[18] = "/ekranoplano/odom";

    /* Start for InitialCondition: '<S19>/IC' */
    memcpy(&pid_control_V4_B.IC[0], &pid_control_V4_ConstP.pooled10[0], 12U *
           sizeof(real_T));

    /* Start for InitialCondition: '<S19>/IC' */
    pid_control_V4_DW.IC_FirstOutputTime = true;

    /* Start for MATLABSystem: '<Root>/Coordinate Transformation Conversion' */
    pid_control_V4_DW.objisempty_d = true;
    pid_control_V4_DW.obj_c.isInitialized = 1;

    /* Start for MATLABSystem: '<S16>/SinkBlock' */
    pid_control_V4_DW.obj_i.QOSAvoidROSNamespaceConventions = false;
    pid_control_V4_DW.obj_i.matlabCodegenIsDeleted = false;
    pid_control_V4_DW.objisempty_m = true;
    pid_control_V4_DW.obj_i.isSetupComplete = false;
    pid_control_V4_DW.obj_i.isInitialized = 1;
    pid_control_Publisher_setupImpl(&pid_control_V4_DW.obj_i);
    pid_control_V4_DW.obj_i.isSetupComplete = true;

    /* Start for MATLABSystem: '<S17>/SinkBlock' */
    pid_control_V4_DW.obj_o.QOSAvoidROSNamespaceConventions = false;
    pid_control_V4_DW.obj_o.matlabCodegenIsDeleted = false;
    pid_control_V4_DW.objisempty_ev = true;
    pid_control_V4_DW.obj_o.isSetupComplete = false;
    pid_control_V4_DW.obj_o.isInitialized = 1;
    qos_profile = pi_ROS2PubSubBase_setQOSProfile(rmw_qos_profile_default, 10.0,
      (rtInf), (rtInf), (rtInf),
      pid_control_V4_DW.obj_o.QOSAvoidROSNamespaceConventions);
    for (i = 0; i < 18; i++) {
      pid_control_V4_B.b_zeroDelimTopic_k[i] = b_zeroDelimTopic[i];
    }

    Pub_pid_control_V4_697.createPublisher(&pid_control_V4_B.b_zeroDelimTopic_k
      [0], qos_profile);
    pid_control_V4_DW.obj_o.isSetupComplete = true;

    /* End of Start for MATLABSystem: '<S17>/SinkBlock' */

    /* Start for MATLABSystem: '<S20>/SourceBlock' */
    pid_control_V4_DW.obj_m.QOSAvoidROSNamespaceConventions = false;
    pid_control_V4_DW.obj_m.matlabCodegenIsDeleted = false;
    pid_control_V4_DW.objisempty_g = true;
    pid_control_V4_DW.obj_m.isSetupComplete = false;
    pid_control_V4_DW.obj_m.isInitialized = 1;
    pid_c_Subscriber_setupImpl_lfjn(&pid_control_V4_DW.obj_m);
    pid_control_V4_DW.obj_m.isSetupComplete = true;

    /* Start for MATLABSystem: '<S21>/SourceBlock' */
    pid_control_V4_DW.obj_k.QOSAvoidROSNamespaceConventions = false;
    pid_control_V4_DW.obj_k.matlabCodegenIsDeleted = false;
    pid_control_V4_DW.objisempty = true;
    pid_control_V4_DW.obj_k.isSetupComplete = false;
    pid_control_V4_DW.obj_k.isInitialized = 1;
    pid__Subscriber_setupImpl_lfjny(&pid_control_V4_DW.obj_k);
    pid_control_V4_DW.obj_k.isSetupComplete = true;

    /* Start for Atomic SubSystem: '<Root>/Call Service' */
    /* Start for MATLABSystem: '<S4>/ServiceCaller' */
    pid_control_V4_DW.obj.QOSAvoidROSNamespaceConventions = false;
    pid_control_V4_DW.obj.matlabCodegenIsDeleted = false;
    pid_control_V4_DW.objisempty_f = true;
    pid_control_V4_DW.obj.isSetupComplete = false;
    pid_control_V4_DW.obj.isInitialized = 1;
    pid_con_ServiceCaller_setupImpl(&pid_control_V4_DW.obj);
    pid_control_V4_DW.obj.isSetupComplete = true;

    /* End of Start for SubSystem: '<Root>/Call Service' */

    /* Start for Enabled SubSystem: '<S299>/Hugw(s)' */
    (void) memset(&(pid_control_V4_XDis.ug_p_CSTATE), 1,
                  2*sizeof(boolean_T));

    /* End of Start for SubSystem: '<S299>/Hugw(s)' */

    /* Start for Enabled SubSystem: '<S299>/Hvgw(s)' */
    (void) memset(&(pid_control_V4_XDis.vg_p1_CSTATE), 1,
                  4*sizeof(boolean_T));

    /* End of Start for SubSystem: '<S299>/Hvgw(s)' */

    /* Start for Enabled SubSystem: '<S299>/Hwgw(s)' */
    (void) memset(&(pid_control_V4_XDis.wg_p1_CSTATE), 1,
                  4*sizeof(boolean_T));

    /* End of Start for SubSystem: '<S299>/Hwgw(s)' */

    /* Start for If: '<S304>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
    pid_control_V4_DW.ifHeightMaxlowaltitudeelseifHei = -1;

    /* Start for MATLABSystem: '<S296>/SourceBlock' */
    pid_control_V4_DW.obj_h.QOSAvoidROSNamespaceConventions = false;
    pid_control_V4_DW.obj_h.matlabCodegenIsDeleted = false;
    pid_control_V4_DW.objisempty_a = true;
    pid_control_V4_DW.obj_h.isSetupComplete = false;
    pid_control_V4_DW.obj_h.isInitialized = 1;
    pid_co_Subscriber_setupImpl_lfj(&pid_control_V4_DW.obj_h);
    pid_control_V4_DW.obj_h.isSetupComplete = true;

    /* Start for Enabled SubSystem: '<S298>/Hpgw' */
    (void) memset(&(pid_control_V4_XDis.pgw_p_CSTATE), 1,
                  2*sizeof(boolean_T));

    /* End of Start for SubSystem: '<S298>/Hpgw' */

    /* Start for Enabled SubSystem: '<S298>/Hqgw' */
    (void) memset(&(pid_control_V4_XDis.qgw_p_CSTATE), 1,
                  2*sizeof(boolean_T));

    /* End of Start for SubSystem: '<S298>/Hqgw' */

    /* Start for Enabled SubSystem: '<S298>/Hrgw' */
    (void) memset(&(pid_control_V4_XDis.rgw_p_CSTATE), 1,
                  2*sizeof(boolean_T));

    /* End of Start for SubSystem: '<S298>/Hrgw' */

    /* Start for If: '<S303>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
    pid_control_V4_DW.ifHeightMaxlowaltitudeelseifH_a = -1;

    /* Start for MATLABSystem: '<S293>/SourceBlock' */
    pid_control_V4_DW.obj_h4.QOSAvoidROSNamespaceConventions = false;
    pid_control_V4_DW.obj_h4.matlabCodegenIsDeleted = false;
    pid_control_V4_DW.objisempty_c = true;
    pid_control_V4_DW.obj_h4.isSetupComplete = false;
    pid_control_V4_DW.obj_h4.isInitialized = 1;
    pid_contro_Subscriber_setupImpl(&pid_control_V4_DW.obj_h4);
    pid_control_V4_DW.obj_h4.isSetupComplete = true;

    /* Start for MATLABSystem: '<S294>/SourceBlock' */
    pid_control_V4_DW.obj_hy.QOSAvoidROSNamespaceConventions = false;
    pid_control_V4_DW.obj_hy.matlabCodegenIsDeleted = false;
    pid_control_V4_DW.objisempty_l = true;
    pid_control_V4_DW.obj_hy.isSetupComplete = false;
    pid_control_V4_DW.obj_hy.isInitialized = 1;
    pid_cont_Subscriber_setupImpl_l(&pid_control_V4_DW.obj_hy);
    pid_control_V4_DW.obj_hy.isSetupComplete = true;

    /* Start for MATLABSystem: '<S295>/SourceBlock' */
    pid_control_V4_DW.obj_p.QOSAvoidROSNamespaceConventions = false;
    pid_control_V4_DW.obj_p.matlabCodegenIsDeleted = false;
    pid_control_V4_DW.objisempty_e = true;
    pid_control_V4_DW.obj_p.isSetupComplete = false;
    pid_control_V4_DW.obj_p.isInitialized = 1;
    pid_con_Subscriber_setupImpl_lf(&pid_control_V4_DW.obj_p);
    pid_control_V4_DW.obj_p.isSetupComplete = true;
  }

  pid_control_V4_PrevZCX.Integrator_Reset_ZCE = UNINITIALIZED_ZCSIG;

  /* InitializeConditions for Memory: '<S19>/Memory2' */
  memcpy(&pid_control_V4_DW.Memory2_PreviousInput[0],
         &pid_control_V4_ConstP.pooled10[0], 12U * sizeof(real_T));

  /* InitializeConditions for Integrator: '<S19>/Integrator' */
  if (rtmIsFirstInitCond((&pid_control_V4_M))) {
    pid_control_V4_X.Integrator_CSTATE[0] = 0.1;
    pid_control_V4_X.Integrator_CSTATE[1] = 0.0;
    pid_control_V4_X.Integrator_CSTATE[2] = 0.0;
    pid_control_V4_X.Integrator_CSTATE[3] = 0.0;
    pid_control_V4_X.Integrator_CSTATE[4] = 0.0;
    pid_control_V4_X.Integrator_CSTATE[5] = 0.0;
    pid_control_V4_X.Integrator_CSTATE[6] = 0.0;
    pid_control_V4_X.Integrator_CSTATE[7] = 0.026179938779914945;
    pid_control_V4_X.Integrator_CSTATE[8] = 0.0;
    pid_control_V4_X.Integrator_CSTATE[9] = 0.0;
    pid_control_V4_X.Integrator_CSTATE[10] = 0.0;
    pid_control_V4_X.Integrator_CSTATE[11] = -0.55;
  }

  pid_control_V4_DW.Integrator_DWORK1 = true;

  /* End of InitializeConditions for Integrator: '<S19>/Integrator' */

  /* InitializeConditions for Integrator: '<S270>/Integrator' */
  pid_control_V4_X.Integrator_CSTATE_f = 0.0;

  /* InitializeConditions for Integrator: '<S265>/Filter' */
  pid_control_V4_X.Filter_CSTATE = 0.0;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay3' */
  pid_control_V4_DW.UnitDelay3_DSTATE = 1.55;

  /* InitializeConditions for Integrator: '<S110>/Integrator' */
  pid_control_V4_X.Integrator_CSTATE_n = 0.0;

  /* InitializeConditions for Integrator: '<S105>/Filter' */
  pid_control_V4_X.Filter_CSTATE_f = 0.0;

  /* InitializeConditions for Integrator: '<S216>/Integrator' */
  pid_control_V4_X.Integrator_CSTATE_d = 0.0;

  /* InitializeConditions for Integrator: '<S211>/Filter' */
  pid_control_V4_X.Filter_CSTATE_fd = 0.0;

  /* InitializeConditions for Integrator: '<S58>/Integrator' */
  pid_control_V4_X.Integrator_CSTATE_m = 0.0;

  /* InitializeConditions for Integrator: '<S53>/Filter' */
  pid_control_V4_X.Filter_CSTATE_g = 0.0;

  /* InitializeConditions for RateLimiter: '<Root>/Rate Limiter-theta_sp' */
  pid_control_V4_DW.LastMajorTime = (rtInf);

  /* InitializeConditions for Integrator: '<S164>/Integrator' */
  pid_control_V4_X.Integrator_CSTATE_p = 0.0;

  /* InitializeConditions for Integrator: '<S159>/Filter' */
  pid_control_V4_X.Filter_CSTATE_m = 0.0;

  /* InitializeConditions for RandomNumber: '<S308>/White Noise' */
  pid_control_V4_DW.RandSeed[0] = 1529675776U;
  pid_control_V4_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V4_DW.RandSeed[0]);
  pid_control_V4_DW.RandSeed[1] = 1529741312U;
  pid_control_V4_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V4_DW.RandSeed[1]);
  pid_control_V4_DW.RandSeed[2] = 1529806848U;
  pid_control_V4_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V4_DW.RandSeed[2]);
  pid_control_V4_DW.RandSeed[3] = 1529872384U;
  pid_control_V4_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V4_DW.RandSeed[3]);

  /* InitializeConditions for Integrator: '<S19>/Integrator1' */
  pid_control_V4_X.Integrator1_CSTATE = 0.0;

  /* InitializeConditions for RandomNumber: '<S288>/White Noise' */
  pid_control_V4_DW.RandSeed_a = 1529675776U;
  pid_control_V4_DW.NextOutput_k = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V4_DW.RandSeed_a);

  /* InitializeConditions for TransferFcn: '<S19>/Transfer Fcn' */
  pid_control_V4_X.TransferFcn_CSTATE[0] = 0.0;
  pid_control_V4_X.TransferFcn_CSTATE[1] = 0.0;

  /* InitializeConditions for TransferFcn: '<S19>/Transfer Fcn1' */
  pid_control_V4_X.TransferFcn1_CSTATE = 0.0;

  /* SystemInitialize for Enabled SubSystem: '<S20>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V4_B.EnabledSubsystem_b);

  /* End of SystemInitialize for SubSystem: '<S20>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S21>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V4_B.EnabledSubsystem_a);

  /* End of SystemInitialize for SubSystem: '<S21>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S299>/Hugw(s)' */
  /* InitializeConditions for Integrator: '<S312>/ug_p' */
  pid_control_V4_X.ug_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S299>/Hugw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S299>/Hvgw(s)' */
  /* InitializeConditions for Integrator: '<S313>/vg_p1' */
  pid_control_V4_X.vg_p1_CSTATE[0] = 0.0;

  /* InitializeConditions for Integrator: '<S313>/vgw_p2' */
  pid_control_V4_X.vgw_p2_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S299>/Hvgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S299>/Hwgw(s)' */
  /* InitializeConditions for Integrator: '<S314>/wg_p1' */
  pid_control_V4_X.wg_p1_CSTATE[0] = 0.0;

  /* InitializeConditions for Integrator: '<S314>/wg_p2' */
  pid_control_V4_X.wg_p2_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S299>/Hwgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S299>/Hugw(s)' */
  /* InitializeConditions for Integrator: '<S312>/ug_p' */
  pid_control_V4_X.ug_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S299>/Hugw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S299>/Hvgw(s)' */
  /* InitializeConditions for Integrator: '<S313>/vg_p1' */
  pid_control_V4_X.vg_p1_CSTATE[1] = 0.0;

  /* InitializeConditions for Integrator: '<S313>/vgw_p2' */
  pid_control_V4_X.vgw_p2_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S299>/Hvgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S299>/Hwgw(s)' */
  /* InitializeConditions for Integrator: '<S314>/wg_p1' */
  pid_control_V4_X.wg_p1_CSTATE[1] = 0.0;

  /* InitializeConditions for Integrator: '<S314>/wg_p2' */
  pid_control_V4_X.wg_p2_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S299>/Hwgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S296>/Enabled Subsystem' */
  pid_contr_EnabledSubsystem_Init(&pid_control_V4_B.EnabledSubsystem_pt);

  /* End of SystemInitialize for SubSystem: '<S296>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S298>/Hpgw' */
  /* InitializeConditions for Integrator: '<S309>/pgw_p' */
  pid_control_V4_X.pgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S298>/Hpgw' */

  /* SystemInitialize for Enabled SubSystem: '<S298>/Hqgw' */
  /* InitializeConditions for Integrator: '<S310>/qgw_p' */
  pid_control_V4_X.qgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S298>/Hqgw' */

  /* SystemInitialize for Enabled SubSystem: '<S298>/Hrgw' */
  /* InitializeConditions for Integrator: '<S311>/rgw_p' */
  pid_control_V4_X.rgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S298>/Hrgw' */

  /* SystemInitialize for Enabled SubSystem: '<S298>/Hpgw' */
  /* InitializeConditions for Integrator: '<S309>/pgw_p' */
  pid_control_V4_X.pgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S298>/Hpgw' */

  /* SystemInitialize for Enabled SubSystem: '<S298>/Hqgw' */
  /* InitializeConditions for Integrator: '<S310>/qgw_p' */
  pid_control_V4_X.qgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S298>/Hqgw' */

  /* SystemInitialize for Enabled SubSystem: '<S298>/Hrgw' */
  /* InitializeConditions for Integrator: '<S311>/rgw_p' */
  pid_control_V4_X.rgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S298>/Hrgw' */

  /* SystemInitialize for Enabled SubSystem: '<S293>/Enabled Subsystem' */
  pid_contr_EnabledSubsystem_Init(&pid_control_V4_B.EnabledSubsystem);

  /* End of SystemInitialize for SubSystem: '<S293>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S294>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V4_B.EnabledSubsystem_k);

  /* End of SystemInitialize for SubSystem: '<S294>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S295>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V4_B.EnabledSubsystem_p);

  /* End of SystemInitialize for SubSystem: '<S295>/Enabled Subsystem' */

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond((&pid_control_V4_M))) {
    rtmSetFirstInitCond((&pid_control_V4_M), 0);
  }
}

/* Model terminate function */
void pid_control_V4::terminate()
{
  /* Terminate for MATLABSystem: '<S16>/SinkBlock' */
  if (!pid_control_V4_DW.obj_i.matlabCodegenIsDeleted) {
    pid_control_V4_DW.obj_i.matlabCodegenIsDeleted = true;
    if ((pid_control_V4_DW.obj_i.isInitialized == 1) &&
        pid_control_V4_DW.obj_i.isSetupComplete) {
      Pub_pid_control_V4_548.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S16>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S17>/SinkBlock' */
  if (!pid_control_V4_DW.obj_o.matlabCodegenIsDeleted) {
    pid_control_V4_DW.obj_o.matlabCodegenIsDeleted = true;
    if ((pid_control_V4_DW.obj_o.isInitialized == 1) &&
        pid_control_V4_DW.obj_o.isSetupComplete) {
      Pub_pid_control_V4_697.resetPublisherPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S17>/SinkBlock' */

  /* Terminate for MATLABSystem: '<S20>/SourceBlock' */
  if (!pid_control_V4_DW.obj_m.matlabCodegenIsDeleted) {
    pid_control_V4_DW.obj_m.matlabCodegenIsDeleted = true;
    if ((pid_control_V4_DW.obj_m.isInitialized == 1) &&
        pid_control_V4_DW.obj_m.isSetupComplete) {
      Sub_pid_control_V4_435.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S20>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S21>/SourceBlock' */
  if (!pid_control_V4_DW.obj_k.matlabCodegenIsDeleted) {
    pid_control_V4_DW.obj_k.matlabCodegenIsDeleted = true;
    if ((pid_control_V4_DW.obj_k.isInitialized == 1) &&
        pid_control_V4_DW.obj_k.isSetupComplete) {
      Sub_pid_control_V4_377.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S21>/SourceBlock' */

  /* Terminate for Atomic SubSystem: '<Root>/Call Service' */
  /* Terminate for MATLABSystem: '<S4>/ServiceCaller' */
  if (!pid_control_V4_DW.obj.matlabCodegenIsDeleted) {
    pid_control_V4_DW.obj.matlabCodegenIsDeleted = true;
    if ((pid_control_V4_DW.obj.isInitialized == 1) &&
        pid_control_V4_DW.obj.isSetupComplete) {
      ServCall_pid_control_V4_326.resetSvcClientPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S4>/ServiceCaller' */
  /* End of Terminate for SubSystem: '<Root>/Call Service' */

  /* Terminate for MATLABSystem: '<S296>/SourceBlock' */
  if (!pid_control_V4_DW.obj_h.matlabCodegenIsDeleted) {
    pid_control_V4_DW.obj_h.matlabCodegenIsDeleted = true;
    if ((pid_control_V4_DW.obj_h.isInitialized == 1) &&
        pid_control_V4_DW.obj_h.isSetupComplete) {
      Sub_pid_control_V4_417.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S296>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S293>/SourceBlock' */
  if (!pid_control_V4_DW.obj_h4.matlabCodegenIsDeleted) {
    pid_control_V4_DW.obj_h4.matlabCodegenIsDeleted = true;
    if ((pid_control_V4_DW.obj_h4.isInitialized == 1) &&
        pid_control_V4_DW.obj_h4.isSetupComplete) {
      Sub_pid_control_V4_423.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S293>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S294>/SourceBlock' */
  if (!pid_control_V4_DW.obj_hy.matlabCodegenIsDeleted) {
    pid_control_V4_DW.obj_hy.matlabCodegenIsDeleted = true;
    if ((pid_control_V4_DW.obj_hy.isInitialized == 1) &&
        pid_control_V4_DW.obj_hy.isSetupComplete) {
      Sub_pid_control_V4_443.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S294>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S295>/SourceBlock' */
  if (!pid_control_V4_DW.obj_p.matlabCodegenIsDeleted) {
    pid_control_V4_DW.obj_p.matlabCodegenIsDeleted = true;
    if ((pid_control_V4_DW.obj_p.isInitialized == 1) &&
        pid_control_V4_DW.obj_p.isSetupComplete) {
      Sub_pid_control_V4_445.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S295>/SourceBlock' */
}

/* Constructor */
pid_control_V4::pid_control_V4() :
  pid_control_V4_B(),
  pid_control_V4_DW(),
  pid_control_V4_X(),
  pid_control_V4_XDis(),
  pid_control_V4_PrevZCX(),
  pid_control_V4_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
pid_control_V4::~pid_control_V4()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_pid_control_V4_T * pid_control_V4::getRTM()
{
  return (&pid_control_V4_M);
}

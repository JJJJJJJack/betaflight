/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config_reset.h"

#include "drivers/pwm_output.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/rpm_filter.h"
#include "flight/interpolated_setpoint.h"
#include "flight/position.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

#include "rx/rx.h"

typedef enum {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
} levelMode_e;

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

FAST_DATA_ZERO_INIT uint32_t targetPidLooptime;
FAST_DATA_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];
FAST_DATA_ZERO_INIT pidRuntime_t pidRuntime;
#ifdef QUATERNION_CONTROL
    float angularRateDesired[XYZ_AXIS_COUNT];
    float Yaw_desire;
    bool arming_state=false, previous_arming_state=false;
#endif

#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT float axisError[XYZ_AXIS_COUNT];
#endif

#if defined(USE_THROTTLE_BOOST)
FAST_DATA_ZERO_INIT float throttleBoost;
pt1Filter_t throttleLpf;
#endif

busDevice_t i2cDev;

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);

#if defined(STM32F1)
#define PID_PROCESS_DENOM_DEFAULT       8
#elif defined(STM32F3)
#define PID_PROCESS_DENOM_DEFAULT       4
#elif defined(STM32F411xE)
#define PID_PROCESS_DENOM_DEFAULT       2
#else
#define PID_PROCESS_DENOM_DEFAULT       1
#endif

#ifdef USE_RUNAWAY_TAKEOFF
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT,
    .runaway_takeoff_prevention = true,
    .runaway_takeoff_deactivate_throttle = 20,  // throttle level % needed to accumulate deactivation time
    .runaway_takeoff_deactivate_delay = 500     // Accumulated time (in milliseconds) before deactivation in successful takeoff
);
#else
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);
#endif

#ifdef USE_ACRO_TRAINER
#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f  // Max gyro rate for lookahead time scaling
#define ACRO_TRAINER_SETPOINT_LIMIT       1000.0f // Limit the correcting setpoint
#endif // USE_ACRO_TRAINER

#ifdef INVERTED_FLIGHT
#define TRAJ_ORDER 6
// Iyy/D/(0.45*9.8)
#define ACC_TO_SERVO_ANGLE 0.00732600732600732f
float Poly_Coeff[TRAJ_ORDER] = {
    -5.23598775028111,
    52.3598774353013,
    -130.899692993310,
    327.249231488540,
    -818.123081194098,
    818.123083052034,
};
#define SERVO_ANGLE_TO_PWM 159.1549430918954f
#endif

#define CRASH_RECOVERY_DETECTION_DELAY_US 1000000  // 1 second delay before crash recovery detection is active after entering a self-level mode

#define LAUNCH_CONTROL_YAW_ITERM_LIMIT 50 // yaw iterm windup limit when launch mode is "FULL" (all axes)

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 1);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  { 42, 85, 35, 90 },
            [PID_PITCH] = { 46, 90, 38, 95 },
            [PID_YAW] =   { 45, 90, 0, 90 },
            [PID_LEVEL] = { 50, 50, 75, 0 },
            [PID_MAG] =   { 40, 0, 0, 0 },
        },
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .yaw_lowpass_hz = 0,
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 0,
        .itermWindupPointPercent = 100,
        .pidAtMinThrottle = PID_STABILISATION_ON,
        .levelAngleLimit = 55,
        .feedForwardTransition = 0,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
        .itermThrottleThreshold = 250,
        .itermAcceleratorGain = 3500,
        .crash_time = 500,          // ms
        .crash_delay = 0,           // ms
        .crash_recovery_angle = 10, // degrees
        .crash_recovery_rate = 100, // degrees/second
        .crash_dthreshold = 50,     // degrees/second/second
        .crash_gthreshold = 400,    // degrees/second
        .crash_setpoint_threshold = 350, // degrees/second
        .crash_recovery = PID_CRASH_RECOVERY_OFF, // off by default
        .horizon_tilt_effect = 75,
        .horizon_tilt_expert_mode = false,
        .crash_limit_yaw = 200,
        .itermLimit = 400,
        .throttle_boost = 5,
        .throttle_boost_cutoff = 15,
        .iterm_rotation = false,
        .iterm_relax = ITERM_RELAX_RP,
        .iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .acro_trainer_gain = 75,
        .abs_control_gain = 0,
        .abs_control_limit = 90,
        .abs_control_error_limit = 20,
        .abs_control_cutoff = 11,
        .antiGravityMode = ANTI_GRAVITY_SMOOTH,
        .dterm_lowpass_hz = 150,    // NOTE: dynamic lpf is enabled by default so this setting is actually
                                    // overridden and the static lowpass 1 is disabled. We can't set this
                                    // value to 0 otherwise Configurator versions 10.4 and earlier will also
                                    // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
        .dterm_lowpass2_hz = 150,   // second Dterm LPF ON by default
        .dterm_filter_type = FILTER_PT1,
        .dterm_filter2_type = FILTER_PT1,
        .dyn_lpf_dterm_min_hz = 70,
        .dyn_lpf_dterm_max_hz = 170,
        .launchControlMode = LAUNCH_CONTROL_MODE_NORMAL,
        .launchControlThrottlePercent = 20,
        .launchControlAngleLimit = 0,
        .launchControlGain = 40,
        .launchControlAllowTriggerReset = true,
        .use_integrated_yaw = false,
        .integrated_yaw_relax = 200,
        .thrustLinearization = 0,
        .d_min = { 23, 25, 0 },      // roll, pitch, yaw
        .d_min_gain = 37,
        .d_min_advance = 20,
        .motor_output_limit = 100,
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .transient_throttle_limit = 0,
        .profileName = { 0 },
        .idle_min_rpm = 0,
        .idle_adjustment_speed = 50,
        .idle_p = 50,
        .idle_pid_limit = 200,
        .idle_max_increase = 150,
        .ff_interpolate_sp = FF_INTERPOLATE_ON,
        .ff_max_rate_limit = 100,
        .ff_smooth_factor = 37,
        .ff_boost = 15,
        .dyn_lpf_curve_expo = 5,
        .level_race_mode = false,
        .vbat_sag_compensation = 0,
    );
#ifndef USE_D_MIN
    pidProfile->pid[PID_ROLL].D = 30;
    pidProfile->pid[PID_PITCH].D = 32;
#endif
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

// Scale factors to make best use of range with D_LPF debugging, aiming for max +/-16K as debug values are 16 bit
#define D_LPF_RAW_SCALE 25
#define D_LPF_FILT_SCALE 22


void pidSetItermAccelerator(float newItermAccelerator)
{
    pidRuntime.itermAccelerator = newItermAccelerator;
}

bool pidOsdAntiGravityActive(void)
{
    return (pidRuntime.itermAccelerator > pidRuntime.antiGravityOsdCutoff);
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidRuntime.pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

float pidGetFfBoostFactor()
{
    return pidRuntime.ffBoostFactor;
}

#ifdef USE_INTERPOLATED_SP
float pidGetFfSmoothFactor()
{
    return pidRuntime.ffSmoothFactor;
}
#endif

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
#if defined(USE_ABSOLUTE_CONTROL)
        axisError[axis] = 0.0f;
#endif
    }
}

void pidUpdateAntiGravityThrottleFilter(float throttle)
{
    if (pidRuntime.antiGravityMode == ANTI_GRAVITY_SMOOTH) {
        pidRuntime.antiGravityThrottleHpf = throttle - pt1FilterApply(&pidRuntime.antiGravityThrottleLpf, throttle);
    }
}

#ifdef USE_ACRO_TRAINER
void pidAcroTrainerInit(void)
{
    pidRuntime.acroTrainerAxisState[FD_ROLL] = 0;
    pidRuntime.acroTrainerAxisState[FD_PITCH] = 0;
}
#endif // USE_ACRO_TRAINER

#ifdef USE_THRUST_LINEARIZATION
float pidCompensateThrustLinearization(float throttle)
{
    if (pidRuntime.thrustLinearization != 0.0f) {
        // for whoops where a lot of TL is needed, allow more throttle boost
        const float throttleReversed = (1.0f - throttle);
        throttle /= 1.0f + pidRuntime.throttleCompensateAmount * powerf(throttleReversed, 2);
    }
    return throttle;
}

float pidApplyThrustLinearization(float motorOutput)
{
    if (pidRuntime.thrustLinearization != 0.0f) {
        if (motorOutput > 0.0f) {
            const float motorOutputReversed = (1.0f - motorOutput);
            motorOutput *= 1.0f + powerf(motorOutputReversed, 2) * pidRuntime.thrustLinearization;
        }
    }
    return motorOutput;
}
#endif

#if defined(USE_ACC)
// calculates strength of horizon leveling; 0 = none, 1.0 = most leveling
STATIC_UNIT_TESTED float calcHorizonLevelStrength(void)
{
    // start with 1.0 at center stick, 0.0 at max stick deflection:
    float horizonLevelStrength = 1.0f - MAX(getRcDeflectionAbs(FD_ROLL), getRcDeflectionAbs(FD_PITCH));

    // 0 at level, 90 at vertical, 180 at inverted (degrees):
    const float currentInclination = MAX(ABS(attitude.values.roll), ABS(attitude.values.pitch)) / 10.0f;

    // horizonTiltExpertMode:  0 = leveling always active when sticks centered,
    //                         1 = leveling can be totally off when inverted
    if (pidRuntime.horizonTiltExpertMode) {
        if (pidRuntime.horizonTransition > 0 && pidRuntime.horizonCutoffDegrees > 0) {
	  // if d_level > 0 and horizonTiltEffect < 175
            // horizonCutoffDegrees: 0 to 125 => 270 to 90 (represents where leveling goes to zero)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations; 0.0 at horizonCutoffDegrees value:
            const float inclinationLevelRatio = constrainf((pidRuntime.horizonCutoffDegrees-currentInclination) / pidRuntime.horizonCutoffDegrees, 0, 1);
            // apply configured horizon sensitivity:
                // when stick is near center (horizonLevelStrength ~= 1.0)
                //  H_sensitivity value has little effect,
                // when stick is deflected (horizonLevelStrength near 0.0)
                //  H_sensitivity value has more effect:
            horizonLevelStrength = (horizonLevelStrength - 1) * 100 / pidRuntime.horizonTransition + 1;
            // apply inclination ratio, which may lower leveling
            //  to zero regardless of stick position:
            horizonLevelStrength *= inclinationLevelRatio;
        } else  { // d_level=0 or horizon_tilt_effect>=175 means no leveling
          horizonLevelStrength = 0;
        }
    } else { // horizon_tilt_expert_mode = 0 (leveling always active when sticks centered)
        float sensitFact;
        if (pidRuntime.horizonFactorRatio < 1.01f) { // if horizonTiltEffect > 0
            // horizonFactorRatio: 1.0 to 0.0 (larger means more leveling)
            // inclinationLevelRatio (0.0 to 1.0) is smaller (less leveling)
            //  for larger inclinations, goes to 1.0 at inclination==level:
            const float inclinationLevelRatio = (180 - currentInclination) / 180 * (1.0f - pidRuntime.horizonFactorRatio) + pidRuntime.horizonFactorRatio;
            // apply ratio to configured horizon sensitivity:
            sensitFact = pidRuntime.horizonTransition * inclinationLevelRatio;
        } else { // horizonTiltEffect=0 for "old" functionality
            sensitFact = pidRuntime.horizonTransition;
        }

        if (sensitFact <= 0) {           // zero means no leveling
            horizonLevelStrength = 0;
        } else {
            // when stick is near center (horizonLevelStrength ~= 1.0)
            //  sensitFact value has little effect,
            // when stick is deflected (horizonLevelStrength near 0.0)
            //  sensitFact value has more effect:
            horizonLevelStrength = ((horizonLevelStrength - 1) * (100 / sensitFact)) + 1;
        }
    }
    return constrainf(horizonLevelStrength, 0, 1);
}

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint) {
    // calculate error angle and limit the angle to the max inclination
    // rcDeflection is in range [-1.0, 1.0]
    float angle = pidProfile->levelAngleLimit * getRcDeflection(axis);
#ifdef USE_GPS_RESCUE
    angle += gpsRescueAngle[axis] / 100; // ANGLE IS IN CENTIDEGREES
#endif
    angle = constrainf(angle, -pidProfile->levelAngleLimit, pidProfile->levelAngleLimit);
    //printf("Desired angle on axis: %d,   %f\n", axis, angle);
    const float errorAngle = angle - ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
        // ANGLE mode - control is angle based
        currentPidSetpoint = errorAngle * pidRuntime.levelGain;
    } else {
        // HORIZON mode - mix of ANGLE and ACRO modes
        // mix in errorAngle to currentPidSetpoint to add a little auto-level feel
        const float horizonLevelStrength = calcHorizonLevelStrength();
        currentPidSetpoint = currentPidSetpoint + (errorAngle * pidRuntime.horizonGain * horizonLevelStrength);
    }
    //position_msp.msg2 = angularRateDesired[FD_YAW]*100.0f;
    #ifdef QUATERNION_CONTROL
        return angularRateDesired[axis];
    #endif
    return currentPidSetpoint;
}

// Quaternion control added by JJJJJJJack
// Date created: 12/10/2022
#ifdef QUATERNION_CONTROL
void angularRateFromQuaternionError(const pidProfile_t *pidProfile){
    float eulerAngleYPR[3], quat_des_RC[4], quat_des[4], quat_ang[4], quat_ang_inv[4], quat_diff[4], axisAngle[4];
    quaternion q_ang = QUATERNION_INITIALIZE;
    arming_state = ARMING_FLAG(ARMED);
    calc_psi_des(getRcDeflection(FD_YAW), DEGREES_TO_RADIANS(-attitude.raw[2]/10.0f), arming_state, previous_arming_state, &Yaw_desire);
    previous_arming_state = arming_state;
    eulerAngleYPR[0] = conv2std(Yaw_desire);
    eulerAngleYPR[1] = (pidProfile->levelAngleLimit * getRcDeflection(FD_PITCH))/57.3f;
    eulerAngleYPR[2] = pidProfile->levelAngleLimit * getRcDeflection(FD_ROLL)/57.3f;
    //eulerAngleYPR[2] = attitudeUpright() ? eulerAngleYPR[2] : 0 - eulerAngleYPR[2];
    //eul2quatZYX(eulerAngleYPR, quat_des_RC);

    /*#ifdef INVERTED_FLIGHT
    float eulerFeedForwardYPR[3] = {0,0,0}, quat_FeedForward[4];
    eulerFeedForwardYPR[1] = conv2std(getFeedForwardFlipAngle(micros()));
    eul2quatZYX(eulerFeedForwardYPR, quat_FeedForward);
    quaternionMultiply(quat_FeedForward, quat_des_RC, quat_des);
    #else
    memcpy(quat_des, quat_des_RC, 16);
    #endif*/

    
    #ifdef INVERTED_FLIGHT
        // When manually flip the vehicle for inverted takeoff, invert the yaw
        if(!ARMING_FLAG(ARMED) && !attitudeUpright())
            eulerAngleYPR[0] = conv2std(eulerAngleYPR[0] + M_PI);
        eulerAngleYPR[1] += getFeedForwardFlipAngle(micros());
        eulerAngleYPR[1] = conv2std(eulerAngleYPR[1]);
        eulerAngleYPR[2] = attitudeUpright() ? eulerAngleYPR[2] : 0 - eulerAngleYPR[2];
    #endif
    eul2quatZYX(eulerAngleYPR, quat_des);
    
    quaternionNormalize(quat_des, quat_des);
    getQuaternion(&q_ang);
    // Logging quaternion through blackbox debug
    // Date created 05/17/2023
    debug[0] = q_ang.w*1e4f;
    debug[1] = q_ang.x*1e4f;
    debug[2] = q_ang.y*1e4f;
    debug[3] = q_ang.z*1e4f;
    quat_ang[0] = q_ang.w; quat_ang[1] = q_ang.x; quat_ang[2] = q_ang.y; quat_ang[3] = q_ang.z;
    //position_msp.msg3 = quat_des[0]*100.0f;
    //position_msp.msg4 = quat_des[1]*100.0f;
    //position_msp.msg5 = quat_des[2]*100.0f;
    //position_msp.msg6 = quat_des[3]*100.0f;
    //position_msp.msg3 = q_ang.w*100.0f;
    //position_msp.msg4 = q_ang.x*100.0f;
    //position_msp.msg5 = q_ang.y*100.0f;
    //position_msp.msg6 = q_ang.z*100.0f;
    quaternionInverse(quat_ang, quat_ang_inv);
    quaternionMultiply(quat_ang_inv, quat_des, quat_diff);
    quaternionToAxisAngle(quat_diff, axisAngle);
    angularRateDesired[FD_ROLL] = RADIANS_TO_DEGREES(axisAngle[0] * axisAngle[3])* pidRuntime.levelGain;
    //if((FLIP_FORWARD && throttle_direction == THROTTLE_NORMAL) || (!FLIP_FORWARD && throttle_direction == THROTTLE_REVERSED)){
    // When throttle boost, disable attitude error, use rate only
    #ifdef INVERTED_FLIGHT
    if((micros() - FlipTriggerTimeMs) * 1e-06f <= (FLIP_FORWARD?ANGLE_RECOVER_TIME_FORWARD:ANGLE_RECOVER_TIME_BACKWARD)){

        // Disable angle control for angular rate feedforward
        axisAngle[1] = 0;
    }
    #endif
    angularRateDesired[FD_PITCH] = RADIANS_TO_DEGREES(axisAngle[1] * axisAngle[3]) * pidRuntime.levelGain;
    #ifdef INVERTED_FLIGHT
    angularRateDesired[FD_PITCH] += RADIANS_TO_DEGREES(getFeedForwardFlipAngularRate(micros()));
    #endif
    //Reduce the yaw gain for bicopter
    angularRateDesired[FD_YAW] = RADIANS_TO_DEGREES(axisAngle[2] * axisAngle[3]) * 1.2f;
    #ifdef INVERTED_FLIGHT
    // Temporarily disable yaw control in flip
    float flip_time = (micros() - FlipTriggerTimeMs) * 1e-06;
    if(flip_time >= 0 && flip_time <= FLIP_DISABLE_ROLLYAW_TIME){
        angularRateDesired[FD_YAW] = 0;
    }
    #endif

}
#endif

static void handleCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const rollAndPitchTrims_t *angleTrim,
    const int axis, const timeUs_t currentTimeUs, const float gyroRate, float *currentPidSetpoint, float *errorRate)
{
    if (pidRuntime.inCrashRecoveryMode && cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) > pidRuntime.crashTimeDelayUs) {
        if (crash_recovery == PID_CRASH_RECOVERY_BEEP) {
            BEEP_ON;
        }
        if (axis == FD_YAW) {
            *errorRate = constrainf(*errorRate, -pidRuntime.crashLimitYaw, pidRuntime.crashLimitYaw);
        } else {
            // on roll and pitch axes calculate currentPidSetpoint and errorRate to level the aircraft to recover from crash
            if (sensors(SENSOR_ACC)) {
                // errorAngle is deviation from horizontal
                const float errorAngle =  -(attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
                *currentPidSetpoint = errorAngle * pidRuntime.levelGain;
                *errorRate = *currentPidSetpoint - gyroRate;
            }
        }
        // reset iterm, since accumulated error before crash is now meaningless
        // and iterm windup during crash recovery can be extreme, especially on yaw axis
        pidData[axis].I = 0.0f;
        if (cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) > pidRuntime.crashTimeLimitUs
            || (getMotorMixRange() < 1.0f
                   && fabsf(gyro.gyroADCf[FD_ROLL]) < pidRuntime.crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_PITCH]) < pidRuntime.crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_YAW]) < pidRuntime.crashRecoveryRate)) {
            if (sensors(SENSOR_ACC)) {
                // check aircraft nearly level
                if (ABS(attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) < pidRuntime.crashRecoveryAngleDeciDegrees
                   && ABS(attitude.raw[FD_PITCH] - angleTrim->raw[FD_PITCH]) < pidRuntime.crashRecoveryAngleDeciDegrees) {
                    pidRuntime.inCrashRecoveryMode = false;
                    BEEP_OFF;
                }
            } else {
                pidRuntime.inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        }
    }
}

static void detectAndSetCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const int axis,
    const timeUs_t currentTimeUs, const float delta, const float errorRate)
{
    // if crash recovery is on and accelerometer enabled and there is no gyro overflow, then check for a crash
    // no point in trying to recover if the crash is so severe that the gyro overflows
    if ((crash_recovery || FLIGHT_MODE(GPS_RESCUE_MODE)) && !gyroOverflowDetected()) {
        if (ARMING_FLAG(ARMED)) {
            if (getMotorMixRange() >= 1.0f && !pidRuntime.inCrashRecoveryMode
                && fabsf(delta) > pidRuntime.crashDtermThreshold
                && fabsf(errorRate) > pidRuntime.crashGyroThreshold
                && fabsf(getSetpointRate(axis)) < pidRuntime.crashSetpointThreshold) {
                if (crash_recovery == PID_CRASH_RECOVERY_DISARM) {
                    setArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
                    disarm(DISARM_REASON_CRASH_PROTECTION);
                } else {
                    pidRuntime.inCrashRecoveryMode = true;
                    pidRuntime.crashDetectedAtUs = currentTimeUs;
                }
            }
            if (pidRuntime.inCrashRecoveryMode && cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) < pidRuntime.crashTimeDelayUs && (fabsf(errorRate) < pidRuntime.crashGyroThreshold
                || fabsf(getSetpointRate(axis)) > pidRuntime.crashSetpointThreshold)) {
                pidRuntime.inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        } else if (pidRuntime.inCrashRecoveryMode) {
            pidRuntime.inCrashRecoveryMode = false;
            BEEP_OFF;
        }
    }
}
#endif // USE_ACC

#ifdef USE_ACRO_TRAINER

int acroTrainerSign(float x)
{
    return x > 0 ? 1 : -1;
}

// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
// There are three states:
// 1. Current angle has exceeded limit
//    Apply correction to return to limit (similar to pidLevel)
// 2. Future overflow has been projected based on current angle and gyro rate
//    Manage the setPoint to control the gyro rate as the actual angle  approaches the limit (try to prevent overshoot)
// 3. If no potential overflow is detected, then return the original setPoint

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM. We accept the
// performance decrease when Acro Trainer mode is active under the assumption that user is unlikely to be
// expecting ultimate flight performance at very high loop rates when in this mode.
static FAST_CODE_NOINLINE float applyAcroTrainer(int axis, const rollAndPitchTrims_t *angleTrim, float setPoint)
{
    float ret = setPoint;

    if (!FLIGHT_MODE(ANGLE_MODE) && !FLIGHT_MODE(HORIZON_MODE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
        bool resetIterm = false;
        float projectedAngle = 0;
        const int setpointSign = acroTrainerSign(setPoint);
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const int angleSign = acroTrainerSign(currentAngle);

        if ((pidRuntime.acroTrainerAxisState[axis] != 0) && (pidRuntime.acroTrainerAxisState[axis] != setpointSign)) {  // stick has reversed - stop limiting
            pidRuntime.acroTrainerAxisState[axis] = 0;
        }

        // Limit and correct the angle when it exceeds the limit
        if ((fabsf(currentAngle) > pidRuntime.acroTrainerAngleLimit) && (pidRuntime.acroTrainerAxisState[axis] == 0)) {
            if (angleSign == setpointSign) {
                pidRuntime.acroTrainerAxisState[axis] = angleSign;
                resetIterm = true;
            }
        }

        if (pidRuntime.acroTrainerAxisState[axis] != 0) {
            ret = constrainf(((pidRuntime.acroTrainerAngleLimit * angleSign) - currentAngle) * pidRuntime.acroTrainerGain, -ACRO_TRAINER_SETPOINT_LIMIT, ACRO_TRAINER_SETPOINT_LIMIT);
        } else {

        // Not currently over the limit so project the angle based on current angle and
        // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
        // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f) * pidRuntime.acroTrainerLookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;
            const int projectedAngleSign = acroTrainerSign(projectedAngle);
            if ((fabsf(projectedAngle) > pidRuntime.acroTrainerAngleLimit) && (projectedAngleSign == setpointSign)) {
                ret = ((pidRuntime.acroTrainerAngleLimit * projectedAngleSign) - projectedAngle) * pidRuntime.acroTrainerGain;
                resetIterm = true;
            }
        }

        if (resetIterm) {
            pidData[axis].I = 0;
        }

        if (axis == pidRuntime.acroTrainerDebugAxis) {
            DEBUG_SET(DEBUG_ACRO_TRAINER, 0, lrintf(currentAngle * 10.0f));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 1, pidRuntime.acroTrainerAxisState[axis]);
            DEBUG_SET(DEBUG_ACRO_TRAINER, 2, lrintf(ret));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 3, lrintf(projectedAngle * 10.0f));
        }
    }

    return ret;
}
#endif // USE_ACRO_TRAINER

static float accelerationLimit(int axis, float currentPidSetpoint)
{
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];

    if (fabsf(currentVelocity) > pidRuntime.maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + pidRuntime.maxVelocity[axis] : previousSetpoint[axis] - pidRuntime.maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT])
{
    // rotate v around rotation vector rotation
    // rotation in radians, all elements must be small
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        int i_1 = (i + 1) % 3;
        int i_2 = (i + 2) % 3;
        float newV = v[i_1] + v[i_2] * rotation[i];
        v[i_2] -= v[i_1] * rotation[i];
        v[i_1] = newV;
    }
}

STATIC_UNIT_TESTED void rotateItermAndAxisError()
{
    if (pidRuntime.itermRotation
#if defined(USE_ABSOLUTE_CONTROL)
        || pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR
#endif
        ) {
        const float gyroToAngle = pidRuntime.dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
#if defined(USE_ABSOLUTE_CONTROL)
        if (pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR) {
            rotateVector(axisError, rotationRads);
        }
#endif
        if (pidRuntime.itermRotation) {
            float v[XYZ_AXIS_COUNT];
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                v[i] = pidData[i].I;
            }
            rotateVector(v, rotationRads );
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pidData[i].I = v[i];
            }
        }
    }
}

#ifdef USE_RC_SMOOTHING_FILTER
float FAST_CODE applyRcSmoothingDerivativeFilter(int axis, float pidSetpointDelta)
{
    float ret = pidSetpointDelta;
    if (axis == pidRuntime.rcSmoothingDebugAxis) {
        DEBUG_SET(DEBUG_RC_SMOOTHING, 1, lrintf(pidSetpointDelta * 100.0f));
    }
    if (pidRuntime.setpointDerivativeLpfInitialized) {
        switch (pidRuntime.rcSmoothingFilterType) {
            case RC_SMOOTHING_DERIVATIVE_PT1:
                ret = pt1FilterApply(&pidRuntime.setpointDerivativePt1[axis], pidSetpointDelta);
                break;
            case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                ret = biquadFilterApplyDF1(&pidRuntime.setpointDerivativeBiquad[axis], pidSetpointDelta);
                break;
        }
        if (axis == pidRuntime.rcSmoothingDebugAxis) {
            DEBUG_SET(DEBUG_RC_SMOOTHING, 2, lrintf(ret * 100.0f));
        }
    }
    return ret;
}
#endif // USE_RC_SMOOTHING_FILTER

#if defined(USE_ITERM_RELAX)
#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED void applyAbsoluteControl(const int axis, const float gyroRate, float *currentPidSetpoint, float *itermErrorRate)
{
    if (pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR) {
        const float setpointLpf = pt1FilterApply(&pidRuntime.acLpf[axis], *currentPidSetpoint);
        const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);
        float acErrorRate = 0;
        const float gmaxac = setpointLpf + 2 * setpointHpf;
        const float gminac = setpointLpf - 2 * setpointHpf;
        if (gyroRate >= gminac && gyroRate <= gmaxac) {
            const float acErrorRate1 = gmaxac - gyroRate;
            const float acErrorRate2 = gminac - gyroRate;
            if (acErrorRate1 * axisError[axis] < 0) {
                acErrorRate = acErrorRate1;
            } else {
                acErrorRate = acErrorRate2;
            }
            if (fabsf(acErrorRate * pidRuntime.dT) > fabsf(axisError[axis]) ) {
                acErrorRate = -axisError[axis] * pidRuntime.pidFrequency;
            }
        } else {
            acErrorRate = (gyroRate > gmaxac ? gmaxac : gminac ) - gyroRate;
        }

        if (isAirmodeActivated()) {
            axisError[axis] = constrainf(axisError[axis] + acErrorRate * pidRuntime.dT,
                -pidRuntime.acErrorLimit, pidRuntime.acErrorLimit);
            const float acCorrection = constrainf(axisError[axis] * pidRuntime.acGain, -pidRuntime.acLimit, pidRuntime.acLimit);
            *currentPidSetpoint += acCorrection;
            *itermErrorRate += acCorrection;
            DEBUG_SET(DEBUG_AC_CORRECTION, axis, lrintf(acCorrection * 10));
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 3, lrintf(acCorrection * 10));
            }
        }
        DEBUG_SET(DEBUG_AC_ERROR, axis, lrintf(axisError[axis] * 10));
    }
}
#endif

STATIC_UNIT_TESTED void applyItermRelax(const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint)
{
    const float setpointLpf = pt1FilterApply(&pidRuntime.windupLpf[axis], *currentPidSetpoint);
    const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);

    if (pidRuntime.itermRelax) {
        if (axis < FD_YAW || pidRuntime.itermRelax == ITERM_RELAX_RPY || pidRuntime.itermRelax == ITERM_RELAX_RPY_INC) {
            const float itermRelaxFactor = MAX(0, 1 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD);
            const bool isDecreasingI =
                ((iterm > 0) && (*itermErrorRate < 0)) || ((iterm < 0) && (*itermErrorRate > 0));
            if ((pidRuntime.itermRelax >= ITERM_RELAX_RP_INC) && isDecreasingI) {
                // Do Nothing, use the precalculed itermErrorRate
            } else if (pidRuntime.itermRelaxType == ITERM_RELAX_SETPOINT) {
                *itermErrorRate *= itermRelaxFactor;
            } else if (pidRuntime.itermRelaxType == ITERM_RELAX_GYRO ) {
                *itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
            } else {
                *itermErrorRate = 0.0f;
            }

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
                DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
                DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(*itermErrorRate));
            }
        }

#if defined(USE_ABSOLUTE_CONTROL)
        applyAbsoluteControl(axis, gyroRate, currentPidSetpoint, itermErrorRate);
#endif
    }
}
#endif

#ifdef USE_AIRMODE_LPF
void pidUpdateAirmodeLpf(float currentOffset)
{
    if (pidRuntime.airmodeThrottleOffsetLimit == 0.0f) {
        return;
    }

    float offsetHpf = currentOffset * 2.5f;
    offsetHpf = offsetHpf - pt1FilterApply(&pidRuntime.airmodeThrottleLpf2, offsetHpf);

    // During high frequency oscillation 2 * currentOffset averages to the offset required to avoid mirroring of the waveform
    pt1FilterApply(&pidRuntime.airmodeThrottleLpf1, offsetHpf);
    // Bring offset up immediately so the filter only applies to the decline
    if (currentOffset * pidRuntime.airmodeThrottleLpf1.state >= 0 && fabsf(currentOffset) > pidRuntime.airmodeThrottleLpf1.state) {
        pidRuntime.airmodeThrottleLpf1.state = currentOffset;
    }
    pidRuntime.airmodeThrottleLpf1.state = constrainf(pidRuntime.airmodeThrottleLpf1.state, -pidRuntime.airmodeThrottleOffsetLimit, pidRuntime.airmodeThrottleOffsetLimit);
}

float pidGetAirmodeThrottleOffset()
{
    return pidRuntime.airmodeThrottleLpf1.state;
}
#endif

#ifdef USE_LAUNCH_CONTROL
#define LAUNCH_CONTROL_MAX_RATE 100.0f
#define LAUNCH_CONTROL_MIN_RATE 5.0f
#define LAUNCH_CONTROL_ANGLE_WINDOW 10.0f  // The remaining angle degrees where rate dampening starts

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
static FAST_CODE_NOINLINE float applyLaunchControl(int axis, const rollAndPitchTrims_t *angleTrim)
{
    float ret = 0.0f;

    // Scale the rates based on stick deflection only. Fixed rates with a max of 100deg/sec
    // reached at 50% stick deflection. This keeps the launch control positioning consistent
    // regardless of the user's rates.
    if ((axis == FD_PITCH) || (pidRuntime.launchControlMode != LAUNCH_CONTROL_MODE_PITCHONLY)) {
        const float stickDeflection = constrainf(getRcDeflection(axis), -0.5f, 0.5f);
        ret = LAUNCH_CONTROL_MAX_RATE * stickDeflection * 2;
    }

#if defined(USE_ACC)
    // If ACC is enabled and a limit angle is set, then try to limit forward tilt
    // to that angle and slow down the rate as the limit is approached to reduce overshoot
    if ((axis == FD_PITCH) && (pidRuntime.launchControlAngleLimit > 0) && (ret > 0)) {
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        if (currentAngle >= pidRuntime.launchControlAngleLimit) {
            ret = 0.0f;
        } else {
            //for the last 10 degrees scale the rate from the current input to 5 dps
            const float angleDelta = pidRuntime.launchControlAngleLimit - currentAngle;
            if (angleDelta <= LAUNCH_CONTROL_ANGLE_WINDOW) {
                ret = scaleRangef(angleDelta, 0, LAUNCH_CONTROL_ANGLE_WINDOW, LAUNCH_CONTROL_MIN_RATE, ret);
            }
        }
    }
#else
    UNUSED(angleTrim);
#endif

    return ret;
}
#endif


float sign(float x)
{
  return x > 0 ? 1 : -1;
}

// Wrap angle in radians to [-pi pi]
float conv2std(float input_angle)
{
    while(input_angle < 0)
        input_angle += 2*M_PI;
    input_angle += M_PI;
    input_angle = fmod(input_angle, 2*M_PI);
    input_angle -= M_PI;
    return input_angle;
}

// Integration of psi stick input to the desired psi
// Input:
//     psi_sp_diff: Stick input
//     psi_current: Current yaw
//     armed: Current arm state
//     armed_prev: Last arm state
// Output:
//     psi_des: Pointer to the desired psi
void calc_psi_des(float psi_sp_diff, float psi_current,  bool armed, bool  armed_prev, float * psi_des)
{
  float psi = psi_current;
  
  if(!armed)
    *psi_des = psi + psi_sp_diff;

  if((armed_prev != armed) && armed) {
    /*  if ARMed */
    *psi_des = psi;
    #ifdef INVERTED_FLIGHT
    if(!attitudeUpright())
        *psi_des = conv2std(*psi_des + M_PI);
    #endif
  }
  
  if (fabs(psi_sp_diff) >= 0.01){
    /*  if rudder stick is at not at center, then change psi_sp. Else do not modify psi_sp */
    //float psi_integral = psi + psi_sp_diff;
    *psi_des += psi_sp_diff*0.005f;
  }
}

void eul2quatZYX(float eulerAngle[3], float * quaternion)
{
    // from Wiki: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // Abbreviations for the various angular functions
    double yaw = eulerAngle[0], pitch = eulerAngle[1], roll = eulerAngle[2];
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    quaternion[0] = cr * cp * cy + sr * sp * sy;
    quaternion[1] = sr * cp * cy - cr * sp * sy;
    quaternion[2] = cr * sp * cy + sr * cp * sy;
    quaternion[3] = cr * cp * sy - sr * sp * cy;
}

float quaternionNorm(float quat[4])
{
    return quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3];
}

void quaternionNormalize(float quat[4], float * result)
{
    float quaternion_norm = quaternionNorm(quat);
    result[0] = quat[0] / sqrtf(quaternion_norm);
    result[1] = quat[1] / sqrtf(quaternion_norm);
    result[2] = quat[2] / sqrtf(quaternion_norm);
    result[3] = quat[3] / sqrtf(quaternion_norm);
}

void quaternionMultiply(float q1[4], float q2[4], float * result)
{
    result[1] =  q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2] + q1[0] * q2[1];
    result[2] = -q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1] + q1[0] * q2[2];
    result[3] =  q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0] + q1[0] * q2[3];
    result[0] = -q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3] + q1[0] * q2[0];
}

void quaternionConjugate(float quaternion[4], float * result)
{
    result[1] = -quaternion[1];
    result[2] = -quaternion[2];
    result[3] = -quaternion[3];
    result[0] = quaternion[0];
}

void quaternionInverse(float quaternion[4], float * result)
{
    float quaternion_conjugate[4];
    quaternionConjugate(quaternion, quaternion_conjugate);
    float quaternion_norm = quaternionNorm(quaternion);
    result[0] = quaternion_conjugate[0] / quaternion_norm;
    result[1] = quaternion_conjugate[1] / quaternion_norm;
    result[2] = quaternion_conjugate[2] / quaternion_norm;
    result[3] = quaternion_conjugate[3] / quaternion_norm;
    //qinv  = quatconj( q )./(quatnorm( q )*ones(1,4));
}

void quaternionToAxisAngle(float quaternion[4], float * axisAngle)
{
    //Normalize the quaternions
    float quat_norm[4];
    quaternionNormalize(quaternion, quat_norm);

    //Normalize and generate the rotation vector and angle sequence
    //For a single quaternion q = [w x y z], the formulas are as follows:
    //(axis) v = [x y z] / norm([x y z]);
    //(angle) theta = 2 * acos(w)
    float axis[3], axis_norm, angle;
    axis_norm = sqrtf(quat_norm[1]*quat_norm[1] + quat_norm[2]*quat_norm[2] + quat_norm[3]*quat_norm[3]);
    if(axis_norm != 0){
        axis[0] = quat_norm[1] / axis_norm;
        axis[1] = quat_norm[2] / axis_norm;
        axis[2] = quat_norm[3] / axis_norm;
    }else{
        axis[0] = 0; axis[1] = 0; axis[2] = 1;
    }
    angle = conv2std(2.0f*acos(quat_norm[0]));

    axisAngle[0] = axis[0];
    axisAngle[1] = axis[1];
    axisAngle[2] = axis[2];
    axisAngle[3] = angle;
}


#ifdef USE_SO3
void arrayMultiply(float array1[3], float array2[3], float * result)
{
    for(int i = 0; i < 3; i++){
        result[i] = array1[i] * array2[i];
    }
}

void getRotVecFromQuaternion(float quaternion[4], float * rotVec)
{
    float ang = 2*acos(quaternion[0]);
    float axis[3] = {quaternion[1], quaternion[2], quaternion[3]};
    float mag = sqrt(pow(axis[0],2) + pow(axis[1],2) + pow(axis[2],2));
    for(int i = 0; i < 3; i++){
        if(mag > 1e-8)
            rotVec[i] = ang * axis[i] / mag;
        else
            rotVec[i] = 0;
    }
}

void rotateframe(float quaternion[4], float rotVec[3],float * errvec)
{
    float n = sqrt(quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
    float a = quaternion[0] / n;
    float b = quaternion[1] / n;
    float c = quaternion[2] / n;
    float d = quaternion[3] / n;
    float e = ((a * 0.0 - -b * rotVec[0]) - -c * rotVec[1]) - -d * rotVec[2];
    float f = ((a * rotVec[0] + -b * 0.0) + -c * rotVec[2]) - -d * rotVec[1];
    float g = ((a * rotVec[1] - -b * rotVec[2]) + -c * 0.0) + -d * rotVec[0];
    float h = ((a * rotVec[2] + -b * rotVec[1]) - -c * rotVec[0]) + -d * 0.0;
    errvec[0] = ((e * b + f * a) + g * d) - h * c;
    errvec[1] = ((e * c - f * d) + g * a) + h * b;
    errvec[2] = ((e * d + f * c) - g * b) + h * a;
}

void arrayCrossProduct(float array1[3], float array2[3], float * result)
{
    //[a0 a1 a2]   x    [b0 b1 b2]
    //= [ a1*b2 - a2*b1, a2*b0 - a0*b2, a0*b1 - a1*b0]
    result[0] = array1[1] * array2[2] - array1[2] * array2[1];
    result[1] = array1[2] * array2[0] - array1[0] * array2[2];
    result[2] = array1[0] * array2[1] - array1[1] * array2[0];
}

void arraySub(float array1[3], float array2[3], float * result)
{
    for(int i = 0; i < 3; i++){
        result[i] = array1[i] - array2[i];
    }
}

void arrayAdd(float array1[3], float array2[3], float * result)
{
    for(int i = 0; i < 3; i++){
        result[i] = array1[i] + array2[i];
    }
}

// Implementing the SO(3) controller from Paper
// High Performance Full Attitude Control of a Quadrotor on SO(3)
// Input: 
//          attitudeDesire : desired attitude (roll pitch yaw) in rad.
//          attitudeCurrent: current attitude (roll pitch yaw) in rad.
//          angularRate    : angular rate from gyro (roll pitch yaw) in rad/s
// Output:
//          pidSum         : Output for the roll pitch and yaw control effort.
void FAST_CODE SO3_controller(float attitudeDesire[3], float attitudeCurrent[3], float angularRate[3], float * pidSum)
{
  //printf("Desired angle: roll: %4.2f, pitch: %4.2f, yaw: %4.2f\n", attitudeDesire[0], attitudeDesire[1], attitudeDesire[2]);
  //printf("Rate: roll: %6.4f, pitch: %6.4f, yaw: %6.4f\n", angularRate[0], angularRate[1], angularRate[2]);
  //printf("current attitude: roll: %6.4f, pitch: %6.4f, yaw: %6.4f\n", attitudeCurrent[0], attitudeCurrent[1], attitudeCurrent[2]);
    // For debugging
    //attitudeDesire[0] = 0.2;
    //attitudeDesire[1] = 0;
    //attitudeDesire[2] = 0;
    //attitudeCurrent[0] = 0.0/57.3;
    //attitudeCurrent[1] = 0;
    //attitudeCurrent[2] = 0;
    //angularRate[0] = attitudeDesire[0]*57.3*10.0;
    //angularRate[1] = attitudeDesire[1]*57.3*10.0;
    //angularRate[2] = attitudeDesire[2]*57.3*10.0;
    //angularRate[0] = attitudeDesire[0]*57.3f;
    //angularRate[1] = attitudeDesire[1]*57.3f;
    //attitudeDesire[0] = 0;
    //attitudeDesire[1] = 0;
    //attitudeDesire[2] = 0;
    
    float quat_des[4], quat_ang[4], quat_error[4];
    float attitudeDesireZYX[3] = {conv2std(attitudeDesire[2]),attitudeDesire[1],attitudeDesire[0]};
    float attitudeCurrentZYX[3] = {conv2std(attitudeCurrent[2]),attitudeCurrent[1],attitudeCurrent[0]};
    float angleError[3], innertiaTerm[3], errorTerm1[3], errorTerm2[3], errorTerm[3], torqueOutput[3], u1, u2, u3, u4, thrust, rotVec[3];

    // Starting SO3 controller
    eul2quatZYX(attitudeDesireZYX, quat_des);
    eul2quatZYX(attitudeCurrentZYX, quat_ang);
    quaternionConjugate(quat_ang, quat_ang);
    quaternionMultiply(quat_des, quat_ang, quat_error);

    getRotVecFromQuaternion(quat_error, rotVec);
    rotateframe(quat_ang, rotVec, angleError);
    float Kp_att[3]        = {pidRuntime.pidCoefficient[FD_ROLL].Kp / PTERM_SCALE / 100.0, pidRuntime.pidCoefficient[FD_PITCH].Kp / PTERM_SCALE / 100.0, pidRuntime.pidCoefficient[FD_YAW].Kp / PTERM_SCALE / 100.0};
    float Kp_rate[3]       = {pidRuntime.pidCoefficient[FD_ROLL].Kd / DTERM_SCALE / 1000.0, pidRuntime.pidCoefficient[FD_PITCH].Kd / DTERM_SCALE / 1000.0, pidRuntime.pidCoefficient[FD_YAW].Kd / DTERM_SCALE / 1000.0};
    float innertiaArray[3] = {pidRuntime.pidCoefficient[FD_ROLL].Ki / ITERM_SCALE / 1000.0, pidRuntime.pidCoefficient[FD_PITCH].Ki / ITERM_SCALE / 1000.0, pidRuntime.pidCoefficient[FD_YAW].Ki / ITERM_SCALE / 1000.0 / 2.5};
    
    arrayMultiply(angleError, Kp_att, errorTerm1);
    arrayMultiply(angularRate, Kp_rate, errorTerm2);
    float innertiaTemp[3];
    arrayMultiply(innertiaArray, angularRate, innertiaTemp);
    arrayCrossProduct(angularRate, innertiaTemp, innertiaTerm);
    //printf("%6.3f, %6.3f, %6.3f\n", output[0], output[1], output[2]);
    //printf("%6.3f, %6.3f, %6.3f\n", innertiaArray[0], innertiaArray[1], innertiaArray[2]);
    //printf("%6.3f, %6.3f, %6.3f\n", innertiaTerm[0], innertiaTerm[1], innertiaTerm[2]);
    
    // torque = Kp * Vec(errorQuat) - kd * angulerRate + angularRate x (J * angularRate);
    arraySub(errorTerm1 , errorTerm2, errorTerm);
    arrayAdd(errorTerm, innertiaTerm, torqueOutput);
    // End SO3 calculation

    // Get vehicle parameters
    float servoLength = (pidRuntime.pidCoefficient[FD_PITCH].Kf == 0) ? 0.05 : pidRuntime.pidCoefficient[FD_PITCH].Kf / FEEDFORWARD_SCALE;
    float armLength   = (pidRuntime.pidCoefficient[FD_ROLL].Kf == 0) ? 0.12 : pidRuntime.pidCoefficient[FD_ROLL].Kf / FEEDFORWARD_SCALE;
    float K_motor     = (pidRuntime.pidCoefficient[FD_YAW].Kf == 0) ? 20 : pidRuntime.pidCoefficient[FD_YAW].Kf / FEEDFORWARD_SCALE * 100; // throttle to force scaling

    u1 = torqueOutput[1] / servoLength;
    u2 = torqueOutput[0]  / armLength;
    u3 = torqueOutput[2]  / armLength;
    thrust = K_motor * constrainf(pow(fabs(rcData[THROTTLE] - 1000) / 1000.0, 2), 0.4, 1.0);
    u4 = thrust;
    
    //printf("Roll: %d, Pitch: %d, Throttle: %d, Yaw: %d\n", rcData[ROLL], rcData[PITCH], rcData[THROTTLE], rcData[YAW]);
    //printf("Torque: roll: %4.2f, pitch: %4.2f, yaw: %4.2f\n", torqueOutput[0], torqueOutput[1], torqueOutput[2]);

    //printf("U1: %4.2f, U2: %4.2f, U3: %4.2f\n", u1, u2, u3);
    
    u1 = constrainf(u1, -thrust*0.7f, thrust*0.7f);
    u2 = constrainf(u2, -thrust*0.7f, thrust*0.7f);
    u3 = constrainf(u3, -thrust*0.7f, thrust*0.7f);

    
    // For finding servo mid only
    //u1 = 0; u2 = 0; u3 = 0; thrust = 2;

    // Old conversion
    /*
    float TRightsindelta1 = (u1-u3)/2.0;
    float TLeftsindelta2 = (u1+u3)/2.0;
    float TRightcosdelta1 = (u4-u2)/2.0;
    float TLeftcosdelta2 = (u2+u4)/2.0;
    float TRight = sqrt(pow(TRightsindelta1,2)+pow(TRightcosdelta1,2));
    float TLeft = sqrt(pow(TLeftsindelta2,2)+pow(TLeftcosdelta2,2));
    float deltaRight = atan2(TRightsindelta1, TRightcosdelta1);
    float deltaLeft = atan2(TLeftsindelta2, TLeftcosdelta2);
    */

    // Fixed conversion
    // u1 = Tr sin dr + Tl sin dl
    // u2 = Tl - Tr
    // u3 = Tl sin dl - Tr sin dr
    // u4 = Tl + Tr
    float TRightsindeltaRight = (u1-u3)/2.0;
    float TLeftsindeltaLeft   = (u1+u3)/2.0;
    float TRight = constrainf((u4-u2)/2.0, 0.01, 10000);
    float TLeft  = constrainf((u4+u2)/2.0, 0.01, 10000);
    float TCONST = 1.0f;
    float deltaRight = asin(TRightsindeltaRight/TCONST);
    float deltaLeft  = asin(TLeftsindeltaLeft/TCONST);
    

    // New conversion
    /*
    float tempT = sqrt(constrainf((- pow(thrust,2) + pow(u1,2) + pow(u2,2))*(- pow(thrust,2) + pow(u2,2) + pow(u3,2)), 0, 100000.0f));
    float TLeft =  (u2*tempT - thrust*pow(u2,2) + pow(thrust,3) + thrust*u1*u3)/(2*(pow(thrust,2) - pow(u2,2)));
    float TRight =  -(u2*tempT + thrust*pow(u2,2) - pow(thrust,3) + thrust*u1*u3)/(2*(pow(thrust,2) - pow(u2,2)));
    float deltaLeft =  -2*atan2((tempT + u1*u3 + pow(thrust,2) - pow(u2,2)), ((thrust + u2)*(u1 + u3))) + M_PI;
    float deltaRight = -2*atan2((tempT - u1*u3 + pow(thrust,2) - pow(u2,2)), ((thrust - u2)*(u1 - u3))) + M_PI;
    */
    //printf("TL:%4.3f, TR:%4.3f, deltaL:%4.3f, deltaR:%4.3f\n", TLeft, TRight, deltaLeft, deltaRight);

    // In Betaflight Bi-copter, the mixer does the following
    // servoLeft  = (-pidYaw - pidPitch) * PID_SERVO_MIXER_SCALING + 1500
    // servoRight = (-pidYaw + pidPitch) * PID_SERVO_MIXER_SCALING + 1500
    // motorLeft  = THROTTLE + pidRoll
    // motorRight = THROTTLE - pidRoll
    float motorLeftPWM  = constrainf(sqrt(TLeft/K_motor)*1000+1000, 1000, 2000);       // T = Kmotor * ((PWM-1000)/1000)^2
    float motorRightPWM = constrainf(sqrt(TRight/K_motor)*1000+1000, 1000, 2000);
    float servoLeftPWM  = constrainf(deltaLeft / SERVO_RANGE * 500 + 1500, 1000, 2000);   // servoAngle = (PWM - 1500) / 500 * (pi/2) * direction
    float servoRightPWM = constrainf(-deltaRight / SERVO_RANGE * 500 + 1500, 1000, 2000); // servoAngle = (PWM - 1500) / 500 * (pi/2) * direction
    //printf("motorLeftPWM: %f  motorRightPWM: %f   servoLeftPWM: %f  servoRightPWM: %f\n", motorLeftPWM, motorRightPWM, servoLeftPWM, servoRightPWM);

    // Checking parameters
    //printf("Kp: %f %f %f  KPrate: %f %f %f, innertiaArray: %f %f %f\n", Kp_att[0], Kp_att[1], Kp_att[2],
    //                                                                                      Kp_rate[0], Kp_rate[1], Kp_rate[2],
    //                                                                                      innertiaArray[0], innertiaArray[1], innertiaArray[2]);
    //printf("servoLength: %f, armLength: %f, K_motor: %f\n", servoLength, armLength, K_motor);

    // pidSum in the order of roll pitch yaw
    pidSum[0] = (motorRightPWM - motorLeftPWM) / 2.0;
    pidSum[1] = (servoRightPWM - servoLeftPWM) / PID_SERVO_MIXER_SCALING / 2.0;
    pidSum[2] = (-servoRightPWM - servoLeftPWM + 3000) / PID_SERVO_MIXER_SCALING / 2.0;
    //printf("pidRoll:%f,  pidPitch:%f,  pidYaw:%f\n", pidSum[0], pidSum[1], pidSum[2]);
}
#endif

// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void FAST_CODE pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    // 20221011 FIXME test I2C
    uint8_t i2cBuffer;
    /*uint8_t i2cBuffer[8];
    uint16_t i2cData = 1500;
    memcpy(i2cBuffer, &i2cData, 2);
    memcpy(i2cBuffer+2, &i2cData, 2);
    memcpy(i2cBuffer+4, &i2cData, 2);
    memcpy(i2cBuffer+6, &i2cData, 2);
    bool i2cResult = i2cWriteBuffer(I2C_DEVICE, 0x12, 0x12, 8, i2cBuffer);*/
    //busDeviceRegister(&i2cDev);
    i2cDev.bustype = BUSTYPE_I2C;
    i2cDev.busdev_u.i2c.device = 0;
    i2cDev.busdev_u.i2c.address = 0x12;
    //UNUSED(i2cDev);
    //busReadRegisterBuffer(&i2cDev, 0x12, &i2cBuffer, 1);
    //busWriteRegister(&i2cDev, 0x12, 1);

    static float previousGyroRateDterm[XYZ_AXIS_COUNT];
#ifdef USE_INTERPOLATED_SP
    static FAST_DATA_ZERO_INIT uint32_t lastFrameNumber;
#endif
    static float previousRawGyroRateDterm[XYZ_AXIS_COUNT];

#if defined(USE_ACC)
    static timeUs_t levelModeStartTimeUs = 0;
    static bool gpsRescuePreviousState = false;
#endif

    const float tpaFactor = getThrottlePIDAttenuation();

#if defined(USE_ACC)
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
#else
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);
#endif

#ifdef USE_TPA_MODE
    const float tpaFactorKp = (currentControlRateProfile->tpaMode == TPA_MODE_PD) ? tpaFactor : 1.0f;
#else
    const float tpaFactorKp = tpaFactor;
#endif

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinActive = gyroYawSpinDetected();
#endif

    const bool launchControlActive = isLaunchControlActive();

#if defined(USE_ACC)
    const bool gpsRescueIsActive = FLIGHT_MODE(GPS_RESCUE_MODE);
    levelMode_e levelMode;
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || gpsRescueIsActive) {
        if (pidRuntime.levelRaceMode && !gpsRescueIsActive) {
            levelMode = LEVEL_MODE_R;
        } else {
            levelMode = LEVEL_MODE_RP;
        }
    } else {
        levelMode = LEVEL_MODE_OFF;
    }

    // Keep track of when we entered a self-level mode so that we can
    // add a guard time before crash recovery can activate.
    // Also reset the guard time whenever GPS Rescue is activated.
    if (levelMode) {
        if ((levelModeStartTimeUs == 0) || (gpsRescueIsActive && !gpsRescuePreviousState)) {
            levelModeStartTimeUs = currentTimeUs;
        }
    } else {
        levelModeStartTimeUs = 0;
    }
    gpsRescuePreviousState = gpsRescueIsActive;
#endif

    // Dynamic i component,
    if ((pidRuntime.antiGravityMode == ANTI_GRAVITY_SMOOTH) && pidRuntime.antiGravityEnabled) {
        pidRuntime.itermAccelerator = fabsf(pidRuntime.antiGravityThrottleHpf) * 0.01f * (pidRuntime.itermAcceleratorGain - 1000);
        DEBUG_SET(DEBUG_ANTI_GRAVITY, 1, lrintf(pidRuntime.antiGravityThrottleHpf * 1000));
    }
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 0, lrintf(pidRuntime.itermAccelerator * 1000));

    float agGain = pidRuntime.dT * pidRuntime.itermAccelerator * AG_KI;

    // gradually scale back integration when above windup point
    float dynCi = pidRuntime.dT;
    if (pidRuntime.itermWindupPointInv > 1.0f) {
        dynCi *= constrainf((1.0f - getMotorMixRange()) * pidRuntime.itermWindupPointInv, 0.0f, 1.0f);
    }

    // Precalculate gyro deta for D-term here, this allows loop unrolling
    float gyroRateDterm[XYZ_AXIS_COUNT];
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        gyroRateDterm[axis] = gyro.gyroADCf[axis];
        // -----calculate raw, unfiltered D component

        // Divide rate change by dT to get differential (ie dr/dt).
        // dT is fixed and calculated from the target PID loop time
        // This is done to avoid DTerm spikes that occur with dynamically
        // calculated deltaT whenever another task causes the PID
        // loop execution to be delayed.
        const float delta =
            - (gyroRateDterm[axis] - previousRawGyroRateDterm[axis]) * pidRuntime.pidFrequency / D_LPF_RAW_SCALE;
        previousRawGyroRateDterm[axis] = gyroRateDterm[axis];

        // Log the unfiltered D
        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_D_LPF, 0, lrintf(delta));
        } else if (axis == FD_PITCH) {
            DEBUG_SET(DEBUG_D_LPF, 1, lrintf(delta));
        }

        gyroRateDterm[axis] = pidRuntime.dtermNotchApplyFn((filter_t *) &pidRuntime.dtermNotch[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = pidRuntime.dtermLowpassApplyFn((filter_t *) &pidRuntime.dtermLowpass[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = pidRuntime.dtermLowpass2ApplyFn((filter_t *) &pidRuntime.dtermLowpass2[axis], gyroRateDterm[axis]);

        #ifdef USE_SO3
        pidRuntime.gyroRateDterm[axis] = gyroRateDterm[axis];
        #endif
    }

    rotateItermAndAxisError();

#ifdef USE_RPM_FILTER
    rpmFilterUpdate();
#endif

#ifdef USE_INTERPOLATED_SP
    bool newRcFrame = false;
    if (lastFrameNumber != getRcFrameNumber()) {
        lastFrameNumber = getRcFrameNumber();
        newRcFrame = true;
    }
#endif

#ifdef USE_SO3
    if(mixerConfig()->mixerMode == MIXER_BICOPTER){
        // Use SO3 controller for bicopter only
        float attitudeCurrent[3] = {-attitude.values.roll/1800.0*M_PI, -attitude.values.pitch/1800.0*M_PI, attitude.values.yaw/1800.0*M_PI};
        // Pass through yaw command mapping PWM to +-150 deg/s
        //pidRuntime.desiredYAW = -getSetpointRate(FD_YAW) / 180.0f * M_PI;
	
        pidRuntime.arming_state = ARMING_FLAG(ARMED);
	calc_psi_des((rcData[YAW] - rxConfig()->midrc) / 500.0, attitudeCurrent[2], pidRuntime.arming_state, pidRuntime.previous_arming_state, &(pidRuntime.desiredYAW));
	pidRuntime.previous_arming_state = pidRuntime.arming_state;
        /*if(fabs(rcData[YAW] - rxConfig()->midrc) > rcControlsConfig()->yaw_deadband){
            // Add rc input to desired yaw
            pidRuntime.desiredYAW += (rcData[YAW] - rxConfig()->midrc) / 500.0 * currentControlRateProfile->rate_limit[YAW] / 180.0 * M_PI / 500.0;
            pidRuntime.desiredYAW = constrainf(pidRuntime.desiredYAW, conv2std(attitudeCurrent[2] - M_PI/6.0), conv2std(attitudeCurrent[2] + M_PI/6.0));
        }
        // Limit the yaw to be -pi to pi
        pidRuntime.desiredYAW = conv2std(pidRuntime.desiredYAW);*/
        float attitudeDesire[3] = {-(rcData[ROLL] - rxConfig()->midrc) / 500.0 * pidProfile->levelAngleLimit / 180.0 * M_PI ,
                                   -(rcData[PITCH] - rxConfig()->midrc) / 500.0 * pidProfile->levelAngleLimit / 180.0 * M_PI,
	                           pidRuntime.desiredYAW};
        // gyro are stored in the system with scaled value. Need to convert to rad/s
        // On TARGET OMNIBUSF4SD the pitch and yaw rate are in the opposite direction
        float angularRate[3] = {-gyro.gyroADCf[FD_ROLL] * 4.1 / 180.0 * M_PI,
                                -gyro.gyroADCf[FD_PITCH] * 4.1 / 180.0 * M_PI,
                                -gyro.gyroADCf[FD_YAW] * 4.1 / 180.0 * M_PI};
        //printf("%f, %f, %f\n", angularRate[0], angularRate[1], angularRate[2]);
        //printf("Currnet angularRate roll: %6.4f, pitch: %6.4f, yaw: %6.4f\n", angularRate[0], angularRate[1], angularRate[2]);
        //printf("Deisred attitude roll: %6.4f, pitch: %6.4f, yaw: %6.4f\n", attitudeDesire[0], attitudeDesire[1], attitudeDesire[2]);
        //printf("Currnet attitude roll: %6.4f, pitch: %6.4f, yaw: %6.4f\n", attitudeCurrent[0], attitudeCurrent[1], attitudeCurrent[2]);
        float SO3Output[3];
        SO3_controller(attitudeDesire, attitudeCurrent, angularRate, SO3Output);
        pidData[FD_ROLL].Sum = SO3Output[0];
        pidData[FD_PITCH].Sum = SO3Output[1];
        pidData[FD_YAW].Sum = SO3Output[2];
        //printf("output: pidRoll: %f,     pidPitch: %f,    pidYaw: %f\n", SO3Output[0], SO3Output[1], SO3Output[2]);
    }else{
    #endif
        // Quaternion control, edited by Xiang He
        // Date created: 12/10/2022
        #ifdef QUATERNION_CONTROL
            angularRateFromQuaternionError(pidProfile);
        #endif
        // ----------PID controller----------
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {

            float currentPidSetpoint = getSetpointRate(axis);
            if (pidRuntime.maxVelocity[axis]) {
                currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
            }
            // Yaw control is GYRO based, direct sticks control is applied to rate PID
            // When Race Mode is active PITCH control is also GYRO based in level or horizon mode
    #if defined(USE_ACC)
            switch (levelMode) {
            case LEVEL_MODE_OFF:

                break;
            case LEVEL_MODE_R:
                if (axis == FD_PITCH) {
                    break;
                }

                FALLTHROUGH;
            case LEVEL_MODE_RP:
                if (axis == FD_YAW) {
                    #ifdef QUATERNION_CONTROL
                        currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
                    #endif
                    break;
                }
                currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
            }
    #endif

    #ifdef USE_ACRO_TRAINER
            if ((axis != FD_YAW) && pidRuntime.acroTrainerActive && !pidRuntime.inCrashRecoveryMode && !launchControlActive) {
                currentPidSetpoint = applyAcroTrainer(axis, angleTrim, currentPidSetpoint);
            }
    #endif // USE_ACRO_TRAINER

    #ifdef USE_LAUNCH_CONTROL
            if (launchControlActive) {
    #if defined(USE_ACC)
                currentPidSetpoint = applyLaunchControl(axis, angleTrim);
    #else
                currentPidSetpoint = applyLaunchControl(axis, NULL);
    #endif
            }
    #endif

            // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
            // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below
    #ifdef USE_YAW_SPIN_RECOVERY
            if ((axis == FD_YAW) && yawSpinActive) {
                currentPidSetpoint = 0.0f;
            }
    #endif // USE_YAW_SPIN_RECOVERY

            // -----calculate error rate
            const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec
            float errorRate = currentPidSetpoint - gyroRate; // r - y
    #if defined(USE_ACC)
            handleCrashRecovery(
                pidProfile->crash_recovery, angleTrim, axis, currentTimeUs, gyroRate,
                &currentPidSetpoint, &errorRate);
    #endif

            const float previousIterm = pidData[axis].I;
            float itermErrorRate = errorRate;
    #ifdef USE_ABSOLUTE_CONTROL
            float uncorrectedSetpoint = currentPidSetpoint;
    #endif

    #if defined(USE_ITERM_RELAX)
            if (!launchControlActive && !pidRuntime.inCrashRecoveryMode) {
                applyItermRelax(axis, previousIterm, gyroRate, &itermErrorRate, &currentPidSetpoint);
                errorRate = currentPidSetpoint - gyroRate;
            }
    #endif
    #ifdef USE_ABSOLUTE_CONTROL
            float setpointCorrection = currentPidSetpoint - uncorrectedSetpoint;
    #endif

            // --------low-level gyro-based PID based on 2DOF PID controller. ----------
            // 2-DOF PID controller with optional filter on derivative term.
            // b = 1 and only c (feedforward weight) can be tuned (amount derivative on measurement or error).
            
            // -----calculate P component
            pidData[axis].P = pidRuntime.pidCoefficient[axis].Kp * errorRate * tpaFactorKp;
            // Increase P gain for bicopter in reversed flight
            /*if(mixerConfig()->mixerMode == MIXER_BICOPTER && FLIP_FORWARD && throttle_direction == THROTTLE_REVERSED){
                if(axis == FD_PITCH || axis == FD_YAW){
                    pidData[axis].P = pidRuntime.pidCoefficient[axis].Kp * 1.2f * errorRate * tpaFactorKp;
                }
            }*/
            if (axis == FD_YAW) {
                pidData[axis].P = pidRuntime.ptermYawLowpassApplyFn((filter_t *) &pidRuntime.ptermYawLowpass, pidData[axis].P);
            }

            // -----calculate I component
            float Ki;
            float axisDynCi;
    #ifdef USE_LAUNCH_CONTROL
            // if launch control is active override the iterm gains and apply iterm windup protection to all axes
            if (launchControlActive) {
                Ki = pidRuntime.launchControlKi;
                axisDynCi = dynCi;
            } else
    #endif
            {
                Ki = pidRuntime.pidCoefficient[axis].Ki;
                axisDynCi = (axis == FD_YAW) ? dynCi : pidRuntime.dT; // only apply windup protection to yaw
            }

            pidData[axis].I = constrainf(previousIterm + (Ki * axisDynCi + agGain) * itermErrorRate, -pidRuntime.itermLimit, pidRuntime.itermLimit);

            // -----calculate pidSetpointDelta
            float pidSetpointDelta = 0;
    #ifdef USE_INTERPOLATED_SP
            if (pidRuntime.ffFromInterpolatedSetpoint) {
                pidSetpointDelta = interpolatedSpApply(axis, newRcFrame, pidRuntime.ffFromInterpolatedSetpoint);
            } else {
                pidSetpointDelta = currentPidSetpoint - pidRuntime.previousPidSetpoint[axis];
            }
    #else
            pidSetpointDelta = currentPidSetpoint - pidRuntime.previousPidSetpoint[axis];
            pidRuntime.pidSetpointDelta[axis] = pidSetpointDelta;
    #endif
            pidRuntime.previousPidSetpoint[axis] = currentPidSetpoint;


    #ifdef USE_RC_SMOOTHING_FILTER
            pidSetpointDelta = applyRcSmoothingDerivativeFilter(axis, pidSetpointDelta);
            #ifdef USE_SO3
            pidRuntime.pidSetpointDelta[axis] = pidSetpointDelta;
            #endif
    #endif // USE_RC_SMOOTHING_FILTER

            // -----calculate D component
            // disable D if launch control is active
            if ((pidRuntime.pidCoefficient[axis].Kd > 0) && !launchControlActive) {

                // Divide rate change by dT to get differential (ie dr/dt).
                // dT is fixed and calculated from the target PID loop time
                // This is done to avoid DTerm spikes that occur with dynamically
                // calculated deltaT whenever another task causes the PID
                // loop execution to be delayed.
                const float delta =
                    - (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidRuntime.pidFrequency;
                float preTpaData = pidRuntime.pidCoefficient[axis].Kd * delta;
                // Increase P gain for bicopter in reversed flight
                /*if(mixerConfig()->mixerMode == MIXER_BICOPTER && FLIP_FORWARD && throttle_direction == THROTTLE_REVERSED){
                    if(axis == FD_PITCH || axis == FD_YAW){
                        preTpaData = pidRuntime.pidCoefficient[axis].Kd * delta * 1.3f;
                    }
                }*/

    #if defined(USE_ACC)
                if (cmpTimeUs(currentTimeUs, levelModeStartTimeUs) > CRASH_RECOVERY_DETECTION_DELAY_US) {
                    detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, currentTimeUs, delta, errorRate);
                }
    #endif

    #if defined(USE_D_MIN)
                float dMinFactor = 1.0f;
                if (pidRuntime.dMinPercent[axis] > 0) {
                    float dMinGyroFactor = biquadFilterApply(&pidRuntime.dMinRange[axis], delta);
                    dMinGyroFactor = fabsf(dMinGyroFactor) * pidRuntime.dMinGyroGain;
                    const float dMinSetpointFactor = (fabsf(pidSetpointDelta)) * pidRuntime.dMinSetpointGain;
                    dMinFactor = MAX(dMinGyroFactor, dMinSetpointFactor);
                    dMinFactor = pidRuntime.dMinPercent[axis] + (1.0f - pidRuntime.dMinPercent[axis]) * dMinFactor;
                    dMinFactor = pt1FilterApply(&pidRuntime.dMinLowpass[axis], dMinFactor);
                    dMinFactor = MIN(dMinFactor, 1.0f);
                    if (axis == FD_ROLL) {
                        DEBUG_SET(DEBUG_D_MIN, 0, lrintf(dMinGyroFactor * 100));
                        DEBUG_SET(DEBUG_D_MIN, 1, lrintf(dMinSetpointFactor * 100));
                        DEBUG_SET(DEBUG_D_MIN, 2, lrintf(pidRuntime.pidCoefficient[axis].Kd * dMinFactor * 10 / DTERM_SCALE));
                    } else if (axis == FD_PITCH) {
                        DEBUG_SET(DEBUG_D_MIN, 3, lrintf(pidRuntime.pidCoefficient[axis].Kd * dMinFactor * 10 / DTERM_SCALE));
                    }
                }

                // Apply the dMinFactor
                preTpaData *= dMinFactor;
    #endif
                pidData[axis].D = preTpaData * tpaFactor;

                // Log the value of D pre application of TPA
                preTpaData *= D_LPF_FILT_SCALE;
                #ifdef INVERTED_FLIGHT
                float flip_time = (micros() - FlipTriggerTimeMs) * 1e-06;
                if(flip_time >= 0 && flip_time <= (FLIP_FORWARD?FLIP_TIME_FORWARD:FLIP_TIME_BACKWARD)){
                    //pidData[FD_PITCH].D = 0.1f*pidData[FD_PITCH].D;
                    pidResetIterm();
                }

                if(FLIP_FORWARD && throttle_direction == THROTTLE_REVERSED && flip_time >= (FLIP_FORWARD?FLIP_TIME_FORWARD:FLIP_TIME_BACKWARD)){
                    // For pure bi-copter set PID gain boost to 200%
                    // The following value is for Tailsitter
                    pidData[FD_PITCH].D = 0.96f*pidData[FD_PITCH].D;
                    pidData[FD_PITCH].P = 1.08f*pidData[FD_PITCH].P;
                    pidData[FD_YAW].D = 0.45f*pidData[FD_YAW].D;
                    pidData[FD_YAW].P = 0.4f*pidData[FD_YAW].P;
                    pidData[FD_YAW].I = 0.2f*pidData[FD_YAW].I;
                }
                #endif

                if (axis == FD_ROLL) {
                    DEBUG_SET(DEBUG_D_LPF, 2, lrintf(preTpaData));
                } else if (axis == FD_PITCH) {
                    DEBUG_SET(DEBUG_D_LPF, 3, lrintf(preTpaData));
                }
            } else {
                pidData[axis].D = 0;

                if (axis == FD_ROLL) {
                    DEBUG_SET(DEBUG_D_LPF, 2, 0);
                } else if (axis == FD_PITCH) {
                    DEBUG_SET(DEBUG_D_LPF, 3, 0);
                }
            }

            previousGyroRateDterm[axis] = gyroRateDterm[axis];

            // -----calculate feedforward component
    #ifdef USE_ABSOLUTE_CONTROL
            // include abs control correction in FF
            pidSetpointDelta += setpointCorrection - pidRuntime.oldSetpointCorrection[axis];
            pidRuntime.oldSetpointCorrection[axis] = setpointCorrection;
    #endif

            // Only enable feedforward for rate mode and if launch control is inactive
            float feedforwardGain = (flightModeFlags || launchControlActive) ? 0.0f : pidRuntime.pidCoefficient[axis].Kf;
            #ifdef INVERTED_FLIGHT
            // Add feedforward for inverted flight, JJJJJJJack
            feedforwardGain = pidRuntime.pidCoefficient[axis].Kf;
            #endif
            if (feedforwardGain > 0) {
                // no transition if feedForwardTransition == 0
                float transition = pidRuntime.feedForwardTransition > 0 ? MIN(1.f, getRcDeflectionAbs(axis) * pidRuntime.feedForwardTransition) : 1;
                #ifdef INVERTED_FLIGHT
                float feedForward = 0;//feedforwardGain * getFeedForwardFlipAngularAcc(micros()) * 100.0f;
                #else
                float feedForward = feedforwardGain * pidSetpointDelta * pidRuntime.pidFrequency;
                #endif

    #ifdef USE_INTERPOLATED_SP
                pidData[axis].F = shouldApplyFfLimits(axis) ?
                    applyFfLimit(axis, feedForward, pidRuntime.pidCoefficient[axis].Kp, currentPidSetpoint) : feedForward;
    #else
                pidData[axis].F = feedForward;
    #endif
    #ifdef INVERTED_FLIGHT
                pidData[axis].F = feedForward;
    #endif
            } else {
                pidData[axis].F = 0;
            }

    #ifdef USE_YAW_SPIN_RECOVERY
            if (yawSpinActive) {
                pidData[axis].I = 0;  // in yaw spin always disable I
                if (axis <= FD_PITCH)  {
                    // zero PIDs on pitch and roll leaving yaw P to correct spin
                    pidData[axis].P = 0;
                    pidData[axis].D = 0;
                    pidData[axis].F = 0;
                }
            }
    #endif // USE_YAW_SPIN_RECOVERY

    #ifdef USE_LAUNCH_CONTROL
            // Disable P/I appropriately based on the launch control mode
            if (launchControlActive) {
                // if not using FULL mode then disable I accumulation on yaw as
                // yaw has a tendency to windup. Otherwise limit yaw iterm accumulation.
                const int launchControlYawItermLimit = (pidRuntime.launchControlMode == LAUNCH_CONTROL_MODE_FULL) ? LAUNCH_CONTROL_YAW_ITERM_LIMIT : 0;
                pidData[FD_YAW].I = constrainf(pidData[FD_YAW].I, -launchControlYawItermLimit, launchControlYawItermLimit);

                // for pitch-only mode we disable everything except pitch P/I
                if (pidRuntime.launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY) {
                    pidData[FD_ROLL].P = 0;
                    pidData[FD_ROLL].I = 0;
                    pidData[FD_YAW].P = 0;
                    // don't let I go negative (pitch backwards) as front motors are limited in the mixer
                    pidData[FD_PITCH].I = MAX(0.0f, pidData[FD_PITCH].I);
                }
            }
    #endif
            // calculating the PID sum
            const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;
    #ifdef USE_INTEGRATED_YAW_CONTROL
            if (axis == FD_YAW && pidRuntime.useIntegratedYaw) {
                pidData[axis].Sum += pidSum * pidRuntime.dT * 100.0f;
                pidData[axis].Sum -= pidData[axis].Sum * pidRuntime.integratedYawRelax / 100000.0f * pidRuntime.dT / 0.000125f;
            } else
    #endif
            {
                pidData[axis].Sum = pidSum;
            }
        } // end PID loop
    #ifdef USE_SO3
        #ifdef USE_LAUNCH_CONTROL
        // Disable P/I appropriately based on the launch control mode
        if (launchControlActive) {
            // if not using FULL mode then disable I accumulation on yaw as
            // yaw has a tendency to windup. Otherwise limit yaw iterm accumulation.
            const int launchControlYawItermLimit = (pidRuntime.launchControlMode == LAUNCH_CONTROL_MODE_FULL) ? LAUNCH_CONTROL_YAW_ITERM_LIMIT : 0;
            pidData[FD_YAW].I = constrainf(pidData[FD_YAW].I, -launchControlYawItermLimit, launchControlYawItermLimit);

            // for pitch-only mode we disable everything except pitch P/I
            if (pidRuntime.launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY) {
                pidData[FD_ROLL].P = 0;
                pidData[FD_ROLL].I = 0;
                pidData[FD_YAW].P = 0;
                // don't let I go negative (pitch backwards) as front motors are limited in the mixer
                pidData[FD_PITCH].I = MAX(0.0f, pidData[FD_PITCH].I);
            }
        }
        #endif
    }
    #endif

    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
    if (!pidRuntime.pidStabilisationEnabled || gyroOverflowDetected()) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;

            pidData[axis].Sum = 0;
        }
    } else if (pidRuntime.zeroThrottleItermReset) {
        pidResetIterm();
    }
}

bool crashRecoveryModeActive(void)
{
    return pidRuntime.inCrashRecoveryMode;
}

#ifdef USE_ACRO_TRAINER
void pidSetAcroTrainerState(bool newState)
{
    if (pidRuntime.acroTrainerActive != newState) {
        if (newState) {
            pidAcroTrainerInit();
        }
        pidRuntime.acroTrainerActive = newState;
    }
}
#endif // USE_ACRO_TRAINER

void pidSetAntiGravityState(bool newState)
{
    if (newState != pidRuntime.antiGravityEnabled) {
        // reset the accelerator on state changes
        pidRuntime.itermAccelerator = 0.0f;
    }
    pidRuntime.antiGravityEnabled = newState;
}

bool pidAntiGravityEnabled(void)
{
    return pidRuntime.antiGravityEnabled;
}

#ifdef USE_DYN_LPF
void dynLpfDTermUpdate(float throttle)
{
    unsigned int cutoffFreq;
    if (pidRuntime.dynLpfFilter != DYN_LPF_NONE) {
        if (pidRuntime.dynLpfCurveExpo > 0) {
            cutoffFreq = dynLpfCutoffFreq(throttle, pidRuntime.dynLpfMin, pidRuntime.dynLpfMax, pidRuntime.dynLpfCurveExpo);
        } else {
            cutoffFreq = fmax(dynThrottle(throttle) * pidRuntime.dynLpfMax, pidRuntime.dynLpfMin);
        }

         if (pidRuntime.dynLpfFilter == DYN_LPF_PT1) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, pidRuntime.dT));
            }
        } else if (pidRuntime.dynLpfFilter == DYN_LPF_BIQUAD) {
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&pidRuntime.dtermLowpass[axis].biquadFilter, cutoffFreq, targetPidLooptime);
            }
        }
    }
}
#endif

float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo) {
    const float expof = expo / 10.0f;
    static float curve;
    curve = throttle * (1 - throttle) * expof + throttle;
    return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

void pidSetItermReset(bool enabled)
{
    pidRuntime.zeroThrottleItermReset = enabled;
}

float pidGetPreviousSetpoint(int axis)
{
    return pidRuntime.previousPidSetpoint[axis];
}

float pidGetDT()
{
    return pidRuntime.dT;
}

float pidGetPidFrequency()
{
    return pidRuntime.pidFrequency;
}



#ifdef INVERTED_FLIGHT

// Function input:
//   t: current time, called by micros()
float getFeedForwardFlipAngle(timeUs_t t){
    /*float triggered_time = (t - FlipTriggerTimeMs) * 1e-06;
    float FFAngle = 0;
    if(triggered_time >= 0 && triggered_time <= FLIP_TIME){
        for(int i = (triggered_time < FLIP_TIME / 2.0f) ? 3 : 0; i < TRAJ_ORDER; i++){
            FFAngle += Poly_Coeff[i] * pow(triggered_time, i);
        }
        FFAngle = FLIP_FORWARD ? FFAngle : -FFAngle + M_PI;
        inverted_flight_angle[FD_PITCH] = FFAngle;
        return FFAngle;
    }else{
        return inverted_flight_angle[FD_PITCH];
    }*/
    return inverted_flight_angle[FD_PITCH];
}

float getFeedForwardFlipAngularRate(timeUs_t t){
    /*float triggered_time = (t - FlipTriggerTimeMs) * 1e-06;
    float FFAngularRate = 0;
    if(triggered_time >= 0 && triggered_time <= FLIP_TIME){
        for(int i = (triggered_time < FLIP_TIME / 2.0f) ? 3 : 1; i < TRAJ_ORDER; i++){
            FFAngularRate += i * Poly_Coeff[i] * pow(triggered_time, i-1);
        }
        FFAngularRate = FLIP_FORWARD ? FFAngularRate : -FFAngularRate;
        return FFAngularRate;
    }else{
        return 0;
    }*/
    /* 20230721 Good code reserved
    // Send max FFAngularRate only at the beginning of the flip
    float FFAngularRateMax = 10.0f;
    if(mixerConfig()->mixerMode == MIXER_BICOPTER)
        FFAngularRateMax = 20.0f;
    if(FLIP_FORWARD && throttle_direction == THROTTLE_NORMAL){
        return FFAngularRateMax;
    }else if(!FLIP_FORWARD && throttle_direction == THROTTLE_REVERSED){
        return FFAngularRateMax;
    }else{
        return 0;
    }
    */
    float FFAngularRate = 0;
    float triggered_time = (t - FlipTriggerTimeMs) * 1e-06;
    float ANG_RATE_UP_TIME = (FLIP_FORWARD ? FLIP_TIME_FORWARD : FLIP_TIME_BACKWARD) - M_PI/(FLIP_FORWARD?THETA_DOT_MAX_FORWARD:THETA_DOT_MAX_BACKWARD);
    float ANG_RATE_MAX_TIME = (FLIP_FORWARD ? FLIP_TIME_FORWARD : FLIP_TIME_BACKWARD) - 2.0f*ANG_RATE_UP_TIME;
    if(triggered_time >= 0 && triggered_time <= (FLIP_FORWARD ? ANGLE_RECOVER_TIME_FORWARD : ANGLE_RECOVER_TIME_BACKWARD)){
        if(triggered_time <= ANG_RATE_UP_TIME){
            FFAngularRate = triggered_time*(FLIP_FORWARD?THETA_DOT_MAX_FORWARD:THETA_DOT_MAX_BACKWARD)/ANG_RATE_UP_TIME;
        }else if(triggered_time <= ANG_RATE_UP_TIME + ANG_RATE_MAX_TIME){
            FFAngularRate = (FLIP_FORWARD?THETA_DOT_MAX_FORWARD:THETA_DOT_MAX_BACKWARD);
        }else if(triggered_time <= ANG_RATE_UP_TIME*2.0f + ANG_RATE_MAX_TIME){
            FFAngularRate = (FLIP_FORWARD?THETA_DOT_MAX_FORWARD:THETA_DOT_MAX_BACKWARD) - (triggered_time - (ANG_RATE_UP_TIME + ANG_RATE_MAX_TIME))*(FLIP_FORWARD?THETA_DOT_MAX_FORWARD:THETA_DOT_MAX_BACKWARD)/ANG_RATE_UP_TIME;
        }
        return FFAngularRate;
    }else{
        return 0;
    }
}

float getFeedForwardFlipAngularAcc(timeUs_t t){
    float triggered_time = (t - FlipTriggerTimeMs) * 1e-06;
    float FFAngularACC = 0;
    if(triggered_time >= 0 && triggered_time <= (FLIP_FORWARD ? FLIP_TIME_FORWARD : FLIP_TIME_BACKWARD)){
        for(int i = (triggered_time < (FLIP_FORWARD ? FLIP_TIME_FORWARD : FLIP_TIME_BACKWARD) / 2.0f) ? 3 : 2; i < TRAJ_ORDER; i++){
            FFAngularACC += i * (i-1) * Poly_Coeff[i] * pow(triggered_time, i-2);
        }
        if(mixerConfig()->mixerMode == MIXER_BICOPTER){
            // Reverse the second half ACC to make sure it continous in time
            if(triggered_time < (FLIP_FORWARD ? FLIP_TIME_FORWARD : FLIP_TIME_BACKWARD) / 2.0f)
                FFAngularACC = FLIP_FORWARD ? FFAngularACC : -FFAngularACC;
            else
                FFAngularACC = FLIP_FORWARD ? -FFAngularACC : FFAngularACC;
        }else
            FFAngularACC = FLIP_FORWARD ? FFAngularACC : -FFAngularACC;
        return FFAngularACC;
    }else{
        return 0;
    }
}

float getFeedForwardFlipServoPWM(timeUs_t t){
    return asin(getFeedForwardFlipAngularAcc(t)*ACC_TO_SERVO_ANGLE)*SERVO_ANGLE_TO_PWM;
}

#endif

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

#pragma once

#include "drivers/time.h"
#include "common/axis.h"
#include "fc/rc_controls.h"

#define INVERTED_FLIGHT

#ifdef INVERTED_FLIGHT
extern timeUs_t FlipTriggerTimeMs;
extern bool FLIP_FORWARD;
extern float inverted_flight_angle[XYZ_AXIS_COUNT];

/*********RAL Bicopter FLIP parameter*********
 * 
#define FLIP_DISABLE_ROLLYAW_TIME 0.45f
#define FLIP_TIME_FORWARD 0.2901f                 
#define THETA_DOT_MAX_FORWARD 20.4846f
#define THRUST_REVERSE_TIME_FORWARD 0.1074f
#define ANGLE_RECOVER_TIME_FORWARD 0.2740f
#define THROTTLE_BOOST_TIME_FORWARD 0.1074f
#define FLIP_TIME_BACKWARD 0.495f                  
#define THETA_DOT_MAX_BACKWARD 12.165f
#define THRUST_REVERSE_TIME_BACKWARD 0.120f
#define ANGLE_RECOVER_TIME_BACKWARD 0.349f
#define THROTTLE_BOOST_TIME_BACKWARD 0.120f
***********************************************/
#define FLIP_DISABLE_ROLLYAW_TIME 1.0f
#define FLIP_TIME_FORWARD 1.2f                 
#define THETA_DOT_MAX_FORWARD 10.0f
#define THRUST_REVERSE_TIME_FORWARD 0.6f
#define ANGLE_RECOVER_TIME_FORWARD 0.9f
#define THROTTLE_BOOST_TIME_FORWARD 0.6f
#define FLIP_TIME_BACKWARD 0.6f                  
#define THETA_DOT_MAX_BACKWARD 10.165f
#define THRUST_REVERSE_TIME_BACKWARD 0.3f
#define ANGLE_RECOVER_TIME_BACKWARD 0.5f
#define THROTTLE_BOOST_TIME_BACKWARD 0.3f

enum THROTTLE_DIRECTION {
    THROTTLE_NORMAL = 0,
    THROTTLE_REVERSED,
};
extern enum THROTTLE_DIRECTION throttle_direction;
#endif

typedef enum {
    INTERPOLATION_CHANNELS_RP,
    INTERPOLATION_CHANNELS_RPY,
    INTERPOLATION_CHANNELS_RPYT,
    INTERPOLATION_CHANNELS_T,
    INTERPOLATION_CHANNELS_RPT,
} interpolationChannels_e;

#ifdef USE_RC_SMOOTHING_FILTER
#define RC_SMOOTHING_AUTO_FACTOR_MIN 0
#define RC_SMOOTHING_AUTO_FACTOR_MAX 50
#endif

void processRcCommand(void);
float getSetpointRate(int axis);
float getRcDeflection(int axis);
float getRcDeflectionAbs(int axis);
float getThrottlePIDAttenuation(void);
void updateRcCommands(void);
void resetYawAxis(void);
void initRcProcessing(void);
bool isMotorsReversed(void);
bool rcSmoothingIsEnabled(void);
rcSmoothingFilter_t *getRcSmoothingData(void);
bool rcSmoothingAutoCalculate(void);
bool rcSmoothingInitializationComplete(void);
float getRawSetpoint(int axis);
float getRawDeflection(int axis);
float applyCurve(int axis, float deflection);
uint32_t getRcFrameNumber();
float getRcCurveSlope(int axis, float deflection);
void updateRcRefreshRate(timeUs_t currentTimeUs);
uint16_t getCurrentRxRefreshRate(void);
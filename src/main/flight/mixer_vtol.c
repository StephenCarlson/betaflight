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
#include <math.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "common/maths.h"
// #include "common/utils.h"

#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/mixer_vtol.h"

// #include "pg/pg.h"
// #include "pg/pg_ids.h"
// #include "pg/rx.h"

#include "rx/rx.h"


PG_REGISTER_WITH_RESET_TEMPLATE(vtolMixerConfig_t, vtolMixerConfig, PG_VTOL_CONFIG, 0);
// PG_REGISTER_ARRAY(vtolRule_t, MAX_VTOL_RULES, customVtolRules, PG_VTOL_CONFIG, 0);

PG_RESET_TEMPLATE(vtolMixerConfig_t, vtolMixerConfig,
    .dummy = 0
);

// PWM_RANGE_MIN is 1000, but not appropiate for 0 to 1000 range

const vtolRule_t vtolRules[] = {
    { .inputSource=4, .throttleFactor=1, .pts={0.00f,0.40f,0.80f,0.95f,1.00f} },
    { .inputSource=4, .throttleFactor=1, .pts={0.00f,0.40f,0.80f,0.95f,1.00f} },
    { .inputSource=4, .throttleFactor=0, .pts={0.00f,0.25f,0.60f,0.80f,1.00f} },
    };



// void mixerVtolMotorAttenuation(float* setpoint, int motor) // Pass by reference variant
// float mixerVtolMotorAttenuation(float setpoint, float throttle, int motor) // Pass by Value variant
float mixerVtolMotorAttenuation(float setpoint, int motor) // Removed throttle field due to trashed/tampered value in mixer.c
{
    // vtolRule_t currentVtolRule = *vtolRules[motor].rule;
    // vtolRule_t currentVtolRule = vtolRules[motor].rule;
    vtolRule_t currentVtolRule = vtolRules[motor];
    // float throttle = constrain(rcData[THROTTLE] - PWM_RANGE_MIN, 0, 1000); // Needs rc_command float, inject from outside instead.
    
    uint8_t ch = constrain(currentVtolRule.inputSource, ROLL, AUX8);
    int16_t vtolCondition = constrain( (rcData[ch] - PWM_RANGE_MIN), 0, (PWM_RANGE_MAX-PWM_RANGE_MIN));

    const int8_t maxIndex = sizeof(((vtolRule_t *)0)->pts)/sizeof(float) - 1; // 4
    const int16_t stride = ((PWM_RANGE_MAX-PWM_RANGE_MIN)/maxIndex); // 250

    int16_t xi = vtolCondition / stride; // [0:4] Index to array y values
    int16_t x  = vtolCondition % stride; // [0:249]
    float y0 = currentVtolRule.pts[xi]; // [0.0f : 1.0f]
    float y1 = (xi < maxIndex)? currentVtolRule.pts[xi+1] : y0;

    float throttle = (rcCommand[THROTTLE] - PWM_RANGE_MIN) / (PWM_RANGE_MAX-PWM_RANGE_MIN);
    throttle = constrainf(throttle, 0.0f, 1.0f);

    float attenuationCoeff = y0 + x * (y1 - y0)/(stride);
    float throttleComponent = (1.0f - attenuationCoeff) * throttle * currentVtolRule.throttleFactor;
    float newSetpoint = (setpoint * attenuationCoeff) + throttleComponent;
    return newSetpoint;
}

void mixerVtolInit(void)
{

}

#endif // USE_SERVOS

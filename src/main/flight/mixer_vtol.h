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

#include "platform.h"

#include "pg/pg.h"




// #define MAX_VTOL_RULES (MAX_SUPPORTED_MOTORS)

// typedef struct vtolMixerConfig_s {
//     uint8_t dummy;
// } vtolMixerConfig_t;

// PG_DECLARE(vtolMixerConfig_t, vtolMixerConfig);

typedef struct vtolRule_s {
    uint8_t inputSource;                  // RC Input channel that drives transfer function VTOL Condition, indexed via rc_controls.h->rc_alias
    float throttleFactor;                 // Range [0f : 1.0f]; In forward flight: 0f stops rotor, 1.0f is passthru equivalent.
    float pts[5];                         // Motor setpoints from PIDs modulated by this piecewise transfer function and VTOL Condition.
} vtolRule_t;

PG_DECLARE_ARRAY(vtolRule_t, MAX_SUPPORTED_MOTORS, customVtolRules);

void mixerVtolInit(void);
float mixerVtolMotorAttenuation(float setpoint, int motor);

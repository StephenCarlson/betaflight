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




// #define MAX_VTOL_RULES (2 * MAX_SUPPORTED_MOTORS)

typedef struct vtolMixerConfig_s {
    uint8_t dummy;
} vtolMixerConfig_t;

PG_DECLARE(vtolMixerConfig_t, vtolMixerConfig);

typedef struct vtolRule_s {
    // uint8_t targetMotor;               // Motor being modified (captured as index for now)
    uint8_t inputSource;                  // RC Input channel that drives transfer function
    uint8_t throttleMask;                 // Boolean {0,1} flag, 0 for simple attenuation, 1 if throttle is added back.
    float pts[5];                         // Set of points for piecewise transfer function
    // float p1;                             // Coefficient at inputSource=0
    // float p2;                             // Coefficient at inputSource=250
    // float p3;                             // Coefficient at inputSource=500
    // float p4;                             // Coefficient at inputSource=750
    // float p5;                             // Coefficient at inputSource=1000
} vtolRule_t;

// PG_DECLARE_ARRAY(vtolRule_t, MAX_VTOL_RULES, customVtolRules);

// typedef struct vtolRules_s {
//     // uint8_t vtolRuleCount;
//     // const vtolRule_t *rule;
//     const vtolRule_t rule;
// } vtolRules_t;

// extern
// const vtolRule_t vtolRules[] = {
//     { .inputSource=8, .throttleMask=1, .pts={0.00f,0.40f,0.80f,0.95f,1.00f} },
//     { .inputSource=8, .throttleMask=1, .pts={0.00f,0.40f,0.80f,0.95f,1.00f} },
//     { .inputSource=8, .throttleMask=0, .pts={0.00f,0.25f,0.60f,0.80f,1.00f} },
//     };
    // {8,0,0.00f,0.40f,0.80f,1.00f,1.00f},
    // {8,0,0.00f,0.40f,0.80f,1.00f,1.00f},
    // {8,1,0.00f,0.25f,0.80f,1.00f,1.00f}
    // {3,{8,0,0.00f,0.40f,0.80f,1.00f,1.00f}},
    // {3,{8,0,0.00f,0.40f,0.80f,1.00f,1.00f}},
    // {3,{8,1,0.00f,0.25f,0.80f,1.00f,1.00f}}
    // (sizeof(vtolRules) / sizeof(vtolRule_t))

void mixerVtolInit(void);
// void mixerVtolMotorAttenuation(float* setpoint, int motor);
float mixerVtolMotorAttenuation(float setpoint, float throttle, int motor);

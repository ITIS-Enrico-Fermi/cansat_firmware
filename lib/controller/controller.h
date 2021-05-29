/**
 * @file controller.h
 * @author sCanSati Team 2020-2021, ITIS E. Fermi, Modena
 * @date 28 May 2021
 * @brief FSM implementation for launch sequence management.
 */

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "lora.h"

#define ACCEL_Z_THRESHOLD 2

typedef enum {
    START = 0,
    INIT,  // Init sensors, fan, buzzer, measure pressure
    GET_ALT,
    WAITING_LAUNCH,  // Wait until accel_z > 2
    LAUNCHED,  // Start pressure measurement every 5 secs
    ALTITUDE_OVER_500M,
    FALLING,  // Fan on
    ALTITUDE_SUB_200M,  // Fan off
    LANDED,  // Buzzer on
    END
} State;

void controller_task(void *pv);
void controller_send_measurements(Payload_t *payload);
void controller_init();

#endif // !__CONTROLLER_H
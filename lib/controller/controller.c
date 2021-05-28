/**
 * @file controller.c
 * @author sCanSati Team 2020-2021, ITIS E. Fermi, Modena
 * @date 28 May 2021
 * @brief FSM implementation for launch sequence management.
 */

#include "controller.h"

static typedef enum {
    START,
    INIT,
    WAITING_LAUNCH,
} State;

static State current_state;

static void send_measurements(Payload_t *payload);

static void send_measurements(Payload_t *payload) {
    Payload_t p = *payload;


}
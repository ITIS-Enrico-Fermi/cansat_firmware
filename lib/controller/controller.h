/**
 * @file controller.h
 * @author sCanSati Team 2020-2021, ITIS E. Fermi, Modena
 * @date 28 May 2021
 * @brief FSM implementation for launch sequence management.
 */

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include "lora.h"

typedef struct {
    void (*send_measurements)(void *Payload_t);
} Controller;

#endif // !__CONTROLLER_H
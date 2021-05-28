/**
 * @file hook_manager.h
 * @author CanSat team
 * @date 28 May 2021
 * @brief General-purpose event manager based on js-like hooks.
 */

#ifndef __HOOK_MANAGER_H
#define __HOOK_MANAGER_H

typedef void (*Action)(void *param);

typedef enum {
    ON_POWER_ON,  // -> selftest
    ON_LAUNCH,  // -> write launch
    ON_MAX_HEIGHT,  // -> fan on
    ON_LANDING,  // -> write landing started
    ON_NEAR_GROUND,  // -> fan off
    ON_GROUND  // -> buzzer on
} Event;

typedef struct {
    Event event;
    Action action;
} Callback;

typedef struct {
    void (*invoke)(Event event, void *param);
} Manager;

Manager createManager(Callback *callbacks, int count);
Manager getManager();

#endif // !__HOOK_MANAGER_H
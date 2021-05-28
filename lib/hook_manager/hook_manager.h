/**
 * @file hook_manager.h
 * @author CanSat team
 * @date 28 May 2021
 * @brief General-purpose event manager based on js-like hooks.
 */

#ifndef __HOOK_MANAGER_H
#define __HOOK_MANAGER_H

#define len(x) sizeof(x)/sizeof(*x)

typedef void (*Action)(void *param);

typedef enum {
    ON_POWER_ON,
    ON_LAUNCH,
    ON_MAX_HEIGHT,
    ON_LANDING,
    ON_GROUND
} Event;

typedef struct {
    Event envent;
    Action action;
} Callback;

typedef struct {
    void (*invoke)(Event event, void *param);
} Manager;

Manager createManager(Callback *callbacks);

#endif // !__HOOK_MANAGER_H
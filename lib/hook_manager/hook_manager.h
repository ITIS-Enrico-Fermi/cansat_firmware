#ifndef HOOK_MANAGER_H
#define HOOK_MANAGER_H

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

#endif // !HOOK_MANAGER_H
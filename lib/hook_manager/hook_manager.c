/**
 * @file hook_manager.c
 * @author sCanSati Team 2020-2021, ITIS E. Fermi, Modena
 * @date 28 May 2021
 * @brief General-purpose event manager based on js-like hooks.
 */

#include "hook_manager.h"
#include "esp_log.h"

static Callback *__callbacks;
static int __callbacksLen;

static void invoke(Event e, void *p);
static Callback *callback(int index);

void invoke(Event e, void *p) {
    ESP_LOGI("invoke", "callback_len: %d", __callbacksLen);
    for (int i = 0; i < __callbacksLen; i++) {
        Callback *c = callback(i);

        if (c && c->action && c->event == e) {
            ESP_LOGI("invoke", "invoked");
            c->action(p);
        }
    }
}

Manager createManager(Callback *callbacks, int count) {
    __callbacks = callbacks;
    __callbacksLen = count;
    return (Manager) { .invoke = invoke };
}

Manager getManager() {
    return (Manager) { .invoke = invoke };
}

Callback *callback(int i) {
    return __callbacks + i;
}
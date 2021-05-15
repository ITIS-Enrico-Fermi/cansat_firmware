/**
 * @file buzzer.h
 * @author Scansati Team 2021
 * @date 13 May 2021
 * @brief Tools to manage buzzer
 */

#ifndef __BUZZER__
#define __BUZZER__

#ifdef __cplusplus
extern "C" {
#endif

void buzzer_init(int gpio_num);
void buzzer_on();
void buzzer_off();

#ifdef __cplusplus
}
#endif

#endif  // __BUZZER__
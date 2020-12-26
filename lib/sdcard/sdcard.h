/*
 * sdcard.h
 * 
 * SD Card over SPI bus facility
 * by Giulio Corradini
 * 
 * Cansat Firmware 2021
 * 
 */

#ifndef __sdcard_h
#define __sdcard_h
#endif

#define SD_MOUNTPOINT "/sdcard"

#ifdef __cplusplus
extern "C" {
#endif

void sdcard_init();

#ifdef __cplusplus
}
#endif
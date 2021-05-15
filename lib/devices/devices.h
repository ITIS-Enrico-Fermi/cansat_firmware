/*
 *  devices.h
 * 
 *  Header file for device registration, configuration and management.
 * 
 *  by Giulio Corradini
 *  (c) Cansat Firmware 2021
 * 
 */

typedef enum {
    DEV_BME280  =   (0x01 << 0),
    DEV_GPS     =   (0x01 << 1),
    DEV_IMU     =   (0x01 << 2),
    DEV_SPS30   =   (0x01 << 3),
    DEV_NTC     =   (0x01 << 4),
    DEV_RFM95   =   (0x01 << 5),
    DEV_FAN     =   (0x01 << 6),
    DEV_SD      =   (0x01 << 7),
    DEV_BUZZ  =   (0x01 << 8),
    DEV_TOTAL_REGISTERED
} Device_t;
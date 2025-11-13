/*
 * sigfox_ep_frames.h
 *
 *  Created on: 13 nov. 2025
 *      Author: Ludo
 */

#ifndef __SIGFOX_EP_FRAMES_H__
#define __SIGFOX_EP_FRAMES_H__

#include "hmd_flags.h"
#include "sigfox_types.h"
#include "types.h"

/*** SIGFOX EP FRAMES macros ***/

// Uplink payload sizes.
#define SIGFOX_EP_UL_PAYLOAD_SIZE_STARTUP           8
#define SIGFOX_EP_UL_PAYLOAD_SIZE_ERROR_STACK       12
#define SIGFOX_EP_UL_PAYLOAD_SIZE_MONITORING        6
#ifdef HMD_ENS16X_ENABLE
#define SIGFOX_EP_UL_PAYLOAD_SIZE_AIR_QUALITY       7
#endif
#ifdef HMD_FXLS89XXXX_ENABLE
#define SIGFOX_EP_UL_PAYLOAD_SIZE_ACCELEROMETER     1
#endif
// Error values.
#define SIGFOX_EP_ERROR_VALUE_ANALOG_16BITS         0xFFFF
#define SIGFOX_EP_ERROR_VALUE_TEMPERATURE           0x7FF
#define SIGFOX_EP_ERROR_VALUE_HUMIDITY              0xFF
#define SIGFOX_EP_ERROR_VALUE_TVOC                  0xFFFF
#define SIGFOX_EP_ERROR_VALUE_ECO2                  0xFFFF
#define SIGFOX_EP_ERROR_VALUE_AQI_UBA               0b1111
#define SIGFOX_EP_ERROR_VALUE_AQI_S                 0xFFF

/*** SIGFOX EP FRAMES structures ***/

/*******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_STARTUP];
    struct {
        unsigned reset_reason :8;
        unsigned major_version :8;
        unsigned minor_version :8;
        unsigned commit_index :8;
        unsigned commit_id :28;
        unsigned dirty_flag :4;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_startup_t;

/*******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_MONITORING];
    struct {
        unsigned vbatt_mv :16;
        unsigned unused :4;
        unsigned tamb_tenth_degrees :12;
        unsigned hamb_percent :8;
        unsigned status :8;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_monitoring_t;

#ifdef HMD_ENS16X_ENABLE
/*******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_AIR_QUALITY];
    struct {
        unsigned acquisition_duration_tens_seconds :6;
        unsigned acquisition_status :2;
        unsigned tvoc_ppb :16;
        unsigned eco2_ppm :16;
        unsigned aqi_uba :4;
        unsigned aqi_s :12;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_air_quality_t;
#endif

#ifdef HMD_FXLS89XXXX_ENABLE
/*******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_ACCELEROMETER];
    struct {
        unsigned event_source :8;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_accelerometer_t;
#endif

#endif /* __SIGFOX_EP_FRAMES_H__ */

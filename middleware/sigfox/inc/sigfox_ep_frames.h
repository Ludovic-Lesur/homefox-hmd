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
#ifdef HMD_AIR_QUALITY_ENABLE
#define SIGFOX_EP_UL_PAYLOAD_SIZE_AIR_QUALITY       7
#endif
#ifdef HMD_ACCELEROMETER_ENABLE
#define SIGFOX_EP_UL_PAYLOAD_SIZE_ACCELEROMETER     1
#endif
// Error values.
#define SIGFOX_EP_ERROR_VALUE_ANALOG_16BITS         0xFFFF
#define SIGFOX_EP_ERROR_VALUE_TEMPERATURE           0x7FF
#define SIGFOX_EP_ERROR_VALUE_HUMIDITY              0xFF
#ifdef HMD_AIR_QUALITY_ENABLE
#define SIGFOX_EP_ERROR_VALUE_TVOC                  0xFFFF
#define SIGFOX_EP_ERROR_VALUE_ECO2                  0xFFFF
#define SIGFOX_EP_ERROR_VALUE_AQI_UBA               0b1111
#define SIGFOX_EP_ERROR_VALUE_AQI_S                 0xFFF
#endif

/*** SIGFOX EP FRAMES structures ***/

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_startup_t
 * \brief Sigfox uplink startup frame format.
 *******************************************************************/
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

/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_monitoring_t
 * \brief Sigfox uplink monitoring frame format.
 *******************************************************************/
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

#ifdef HMD_AIR_QUALITY_ENABLE
/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_air_quality_t
 * \brief Sigfox uplink air quality frame format.
 *******************************************************************/
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

#ifdef HMD_ACCELEROMETER_ENABLE
/*!******************************************************************
 * \struct SIGFOX_EP_ul_payload_accelerometer_t
 * \brief Sigfox uplink accelerometer frame format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_EP_UL_PAYLOAD_SIZE_ACCELEROMETER];
    struct {
        unsigned event_source :8;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_ul_payload_accelerometer_t;
#endif

/*!******************************************************************
 * \enum SIGFOX_EP_dl_op_code_t
 * \brief Sigfox downlink operation codes.
 *******************************************************************/
typedef enum {
    SIGFOX_EP_DL_OP_CODE_NOP = 0,
    SIGFOX_EP_DL_OP_CODE_RESET,
    SIGFOX_EP_DL_OP_CODE_SET_TIMINGS,
    SIGFOX_EP_DL_OP_CODE_SET_LED_COLOR,
    SIGFOX_EP_DL_OP_CODE_LAST
} SIGFOX_EP_dl_op_code_t;

/*!******************************************************************
 * \enum SIGFOX_EP_dl_payload_t
 * \brief Sigfox downlink frames format.
 *******************************************************************/
typedef union {
    uint8_t frame[SIGFOX_DL_PAYLOAD_SIZE_BYTES];
    struct {
        unsigned op_code :8;
        union {
            struct {
                unsigned monitoring_period_minutes :16;
                unsigned air_quality_period_minutes :16;
                unsigned accelerometer_blanking_time_seconds :16;
                unsigned unused: 8;
            } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) set_timings;
        };
        union {
            struct {
                unsigned sigfox_uplink :8;
                unsigned sigfox_downlink :8;
                unsigned temperature_humidity_reading :8;
                unsigned air_quality_reading :8;
                unsigned accelerometer_reading :8;
                unsigned unused :16;
            } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) set_led_color;
        };
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} SIGFOX_EP_dl_payload_t;

#endif /* __SIGFOX_EP_FRAMES_H__ */

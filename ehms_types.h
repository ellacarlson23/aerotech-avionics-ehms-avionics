/**
 * @file ehms_types.h
 * @brief EHMS Core Type Definitions
 * 
 * @copyright Copyright (c) 2025 AeroTech Avionics Inc.
 * @note This file is subject to DO-178C Level B certification requirements
 * 
 * @version 2.4.1
 * @date 2025-01-15
 * 
 * CSCI: EHMS-CORE
 * CSC: TYPE-DEFINITIONS
 * 
 * Requirements Trace:
 *   SRS-EHMS-001: System shall define standardized data types
 *   SRS-EHMS-002: System shall support multi-engine configurations
 */

#ifndef EHMS_TYPES_H
#define EHMS_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * INCLUDES
 * ============================================================================ */
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * VERSION INFORMATION
 * ============================================================================ */
#define EHMS_VERSION_MAJOR      2U
#define EHMS_VERSION_MINOR      4U
#define EHMS_VERSION_PATCH      1U
#define EHMS_VERSION_STRING     "2.4.1"

/* ============================================================================
 * SYSTEM CONFIGURATION CONSTANTS
 * ============================================================================ */

/** @brief Maximum number of engines supported */
#define EHMS_MAX_ENGINES                    4U

/** @brief Maximum number of monitored parameters per engine */
#define EHMS_MAX_PARAMETERS                 64U

/** @brief Maximum sample rate in Hz */
#define EHMS_MAX_SAMPLE_RATE_HZ             100U

/** @brief Flight data retention in hours */
#define EHMS_DATA_RETENTION_HOURS           500U

/** @brief Maximum active alerts */
#define EHMS_MAX_ACTIVE_ALERTS              32U

/** @brief ARINC 429 bus count */
#define EHMS_ARINC429_BUS_COUNT             4U

/** @brief MIL-STD-1553B RT address */
#define EHMS_1553_RT_ADDRESS                0x05U

/* ============================================================================
 * FIXED-POINT SCALING FACTORS
 * ============================================================================ */

/** @brief Temperature scaling (0.1 degree resolution) */
#define EHMS_TEMP_SCALE_FACTOR              10U

/** @brief Pressure scaling (0.01 PSI resolution) */
#define EHMS_PRESSURE_SCALE_FACTOR          100U

/** @brief RPM scaling (0.1% N1/N2 resolution) */
#define EHMS_RPM_SCALE_FACTOR               10U

/** @brief Fuel flow scaling (0.1 lb/hr resolution) */
#define EHMS_FUEL_FLOW_SCALE_FACTOR         10U

/* ============================================================================
 * ENUMERATED TYPES
 * ============================================================================ */

/**
 * @brief Engine identification
 * @trace SRS-EHMS-010
 */
typedef enum
{
    EHMS_ENGINE_1       = 0U,   /**< Engine 1 (Left/Port) */
    EHMS_ENGINE_2       = 1U,   /**< Engine 2 (Right/Starboard) */
    EHMS_ENGINE_3       = 2U,   /**< Engine 3 (if applicable) */
    EHMS_ENGINE_4       = 3U,   /**< Engine 4 (if applicable) */
    EHMS_ENGINE_COUNT   = 4U    /**< Total engine count */
} ehms_engine_id_t;

/**
 * @brief System operational state
 * @trace SRS-EHMS-015
 */
typedef enum
{
    EHMS_STATE_OFF          = 0U,   /**< System powered off */
    EHMS_STATE_INIT         = 1U,   /**< Initialization in progress */
    EHMS_STATE_PBIT         = 2U,   /**< Power-on BIT executing */
    EHMS_STATE_NORMAL       = 3U,   /**< Normal operation */
    EHMS_STATE_DEGRADED     = 4U,   /**< Degraded mode (partial function) */
    EHMS_STATE_MAINTENANCE  = 5U,   /**< Maintenance mode */
    EHMS_STATE_FAULT        = 6U    /**< System fault detected */
} ehms_system_state_t;

/**
 * @brief Alert severity levels
 * @trace SRS-EHMS-020
 */
typedef enum
{
    EHMS_ALERT_NONE         = 0U,   /**< No alert */
    EHMS_ALERT_STATUS       = 1U,   /**< Status message (white) */
    EHMS_ALERT_ADVISORY     = 2U,   /**< Advisory (cyan) */
    EHMS_ALERT_CAUTION      = 3U,   /**< Caution (amber) */
    EHMS_ALERT_WARNING      = 4U    /**< Warning (red) */
} ehms_alert_level_t;

/**
 * @brief Parameter validity status
 * @trace SRS-EHMS-025
 */
typedef enum
{
    EHMS_PARAM_VALID        = 0U,   /**< Parameter is valid */
    EHMS_PARAM_STALE        = 1U,   /**< Data is stale (timeout) */
    EHMS_PARAM_FAILED       = 2U,   /**< Sensor/source failed */
    EHMS_PARAM_NCD          = 3U,   /**< No Computed Data */
    EHMS_PARAM_TEST         = 4U    /**< Test mode data */
} ehms_param_status_t;

/**
 * @brief Engine parameter identifiers
 * @trace SRS-EHMS-030
 */
typedef enum
{
    EHMS_PARAM_N1           = 0U,   /**< Fan speed (% RPM) */
    EHMS_PARAM_N2           = 1U,   /**< Core speed (% RPM) */
    EHMS_PARAM_EGT          = 2U,   /**< Exhaust Gas Temperature (°C) */
    EHMS_PARAM_FF           = 3U,   /**< Fuel Flow (lb/hr) */
    EHMS_PARAM_OIL_TEMP     = 4U,   /**< Oil Temperature (°C) */
    EHMS_PARAM_OIL_PRESS    = 5U,   /**< Oil Pressure (PSI) */
    EHMS_PARAM_OIL_QTY      = 6U,   /**< Oil Quantity (%) */
    EHMS_PARAM_VIB_FAN      = 7U,   /**< Fan Vibration (IPS) */
    EHMS_PARAM_VIB_CORE     = 8U,   /**< Core Vibration (IPS) */
    EHMS_PARAM_EPR          = 9U,   /**< Engine Pressure Ratio */
    EHMS_PARAM_ITT          = 10U,  /**< Interstage Turbine Temp (°C) */
    EHMS_PARAM_THRUST       = 11U,  /**< Computed Thrust (lbf) */
    EHMS_PARAM_BLEED_PRESS  = 12U,  /**< Bleed Air Pressure (PSI) */
    EHMS_PARAM_BLEED_TEMP   = 13U,  /**< Bleed Air Temperature (°C) */
    EHMS_PARAM_START_VALVE  = 14U,  /**< Start Valve Position */
    EHMS_PARAM_FUEL_VALVE   = 15U,  /**< Fuel Shutoff Valve Position */
    /* ... additional parameters up to EHMS_MAX_PARAMETERS */
    EHMS_PARAM_COUNT        = 48U   /**< Total parameter count */
} ehms_param_id_t;

/**
 * @brief Health assessment result
 * @trace SRS-EHMS-035
 */
typedef enum
{
    EHMS_HEALTH_NORMAL      = 0U,   /**< Engine operating normally */
    EHMS_HEALTH_MONITOR     = 1U,   /**< Monitor closely */
    EHMS_HEALTH_CAUTION     = 2U,   /**< Maintenance recommended */
    EHMS_HEALTH_ACTION_REQ  = 3U,   /**< Maintenance action required */
    EHMS_HEALTH_CRITICAL    = 4U    /**< Immediate action required */
} ehms_health_status_t;

/* ============================================================================
 * DATA STRUCTURES
 * ============================================================================ */

/**
 * @brief Timestamp structure (UTC)
 * @trace SRS-EHMS-040
 */
typedef struct
{
    uint16_t year;          /**< Year (2000-2099) */
    uint8_t  month;         /**< Month (1-12) */
    uint8_t  day;           /**< Day (1-31) */
    uint8_t  hour;          /**< Hour (0-23) */
    uint8_t  minute;        /**< Minute (0-59) */
    uint8_t  second;        /**< Second (0-59) */
    uint16_t millisecond;   /**< Millisecond (0-999) */
} ehms_timestamp_t;

/**
 * @brief Single parameter data
 * @trace SRS-EHMS-045
 */
typedef struct
{
    ehms_param_id_t     param_id;       /**< Parameter identifier */
    ehms_param_status_t status;         /**< Validity status */
    int32_t             raw_value;      /**< Raw scaled value */
    float               eng_value;      /**< Engineering units value */
    ehms_timestamp_t    timestamp;      /**< Sample timestamp */
    uint8_t             source_bus;     /**< Source bus ID */
} ehms_parameter_t;

/**
 * @brief Engine snapshot (all parameters at one time)
 * @trace SRS-EHMS-050
 */
typedef struct
{
    ehms_engine_id_t    engine_id;                          /**< Engine ID */
    ehms_timestamp_t    sample_time;                        /**< Snapshot time */
    uint32_t            flight_phase;                       /**< Current flight phase */
    ehms_parameter_t    parameters[EHMS_PARAM_COUNT];       /**< Parameter array */
    ehms_health_status_t health_status;                     /**< Overall health */
    uint32_t            crc32;                              /**< Data integrity CRC */
} ehms_engine_snapshot_t;

/**
 * @brief Alert message structure
 * @trace SRS-EHMS-055
 */
typedef struct
{
    uint32_t            alert_id;       /**< Unique alert identifier */
    ehms_alert_level_t  level;          /**< Severity level */
    ehms_engine_id_t    engine_id;      /**< Affected engine */
    ehms_param_id_t     param_id;       /**< Related parameter (if any) */
    ehms_timestamp_t    onset_time;     /**< Alert onset time */
    ehms_timestamp_t    clear_time;     /**< Alert clear time (if cleared) */
    bool                is_active;      /**< Alert currently active */
    bool                is_latched;     /**< Alert requires manual reset */
    bool                is_inhibited;   /**< Alert currently inhibited */
    char                message[64];    /**< Display message text */
    uint16_t            ecam_code;      /**< ECAM/EICAS message code */
} ehms_alert_t;

/**
 * @brief Predictive maintenance data
 * @trace SRS-EHMS-060
 */
typedef struct
{
    ehms_engine_id_t    engine_id;              /**< Engine ID */
    float               remaining_life_hours;   /**< Estimated RUL (hours) */
    float               confidence_level;       /**< Prediction confidence (0-1) */
    uint32_t            next_maint_flight_hrs;  /**< Recommended next maintenance */
    bool                trend_abnormal;         /**< Abnormal trend detected */
    char                recommendation[128];    /**< Maintenance recommendation */
} ehms_predictive_t;

/**
 * @brief System status summary
 * @trace SRS-EHMS-065
 */
typedef struct
{
    ehms_system_state_t     state;                          /**< Current system state */
    uint32_t                uptime_seconds;                 /**< System uptime */
    uint32_t                flight_hours_recorded;          /**< Total recorded hours */
    uint32_t                active_alert_count;             /**< Active alerts */
    ehms_alert_level_t      highest_alert_level;            /**< Highest active alert */
    bool                    bit_passed;                     /**< Last BIT result */
    bool                    comms_active;                   /**< Ground link active */
    ehms_health_status_t    engine_health[EHMS_MAX_ENGINES]; /**< Per-engine health */
} ehms_system_status_t;

/* ============================================================================
 * FUNCTION RETURN CODES
 * ============================================================================ */

/**
 * @brief Standard return codes
 * @trace SRS-EHMS-070
 */
typedef enum
{
    EHMS_OK                 = 0,    /**< Operation successful */
    EHMS_ERROR              = -1,   /**< General error */
    EHMS_ERROR_PARAM        = -2,   /**< Invalid parameter */
    EHMS_ERROR_RANGE        = -3,   /**< Value out of range */
    EHMS_ERROR_TIMEOUT      = -4,   /**< Operation timeout */
    EHMS_ERROR_BUSY         = -5,   /**< Resource busy */
    EHMS_ERROR_MEMORY       = -6,   /**< Memory allocation failed */
    EHMS_ERROR_HARDWARE     = -7,   /**< Hardware error */
    EHMS_ERROR_CONFIG       = -8,   /**< Configuration error */
    EHMS_ERROR_NOT_INIT     = -9,   /**< Not initialized */
    EHMS_ERROR_CRC          = -10   /**< CRC mismatch */
} ehms_result_t;

/* ============================================================================
 * MACRO FUNCTIONS
 * ============================================================================ */

/**
 * @brief Convert raw value to engineering units for temperature
 */
#define EHMS_RAW_TO_TEMP(raw)   ((float)(raw) / (float)EHMS_TEMP_SCALE_FACTOR)

/**
 * @brief Convert raw value to engineering units for pressure
 */
#define EHMS_RAW_TO_PRESS(raw)  ((float)(raw) / (float)EHMS_PRESSURE_SCALE_FACTOR)

/**
 * @brief Convert raw value to engineering units for RPM percentage
 */
#define EHMS_RAW_TO_RPM(raw)    ((float)(raw) / (float)EHMS_RPM_SCALE_FACTOR)

/**
 * @brief Check if alert level is crew-alerting
 */
#define EHMS_IS_CREW_ALERT(level) ((level) >= EHMS_ALERT_CAUTION)

/**
 * @brief Check if parameter status indicates valid data
 */
#define EHMS_PARAM_IS_VALID(status) ((status) == EHMS_PARAM_VALID)

#ifdef __cplusplus
}
#endif

#endif /* EHMS_TYPES_H */

/* END OF FILE */

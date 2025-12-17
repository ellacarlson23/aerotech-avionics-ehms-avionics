/**
 * @file alert_manager.c
 * @brief EHMS Alert Management Module
 * 
 * @copyright Copyright (c) 2025 AeroTech Avionics Inc.
 * @note DO-178C Level B - Safety Critical Software
 * 
 * CSCI: EHMS-ALERTS
 * CSC: ALERT-MANAGER
 * 
 * Requirements Trace:
 *   SRS-EHMS-200: System shall generate alerts within 100ms of threshold exceedance
 *   SRS-EHMS-201: System shall prioritize alerts by severity level
 *   SRS-EHMS-202: System shall support alert inhibit logic
 *   SRS-EHMS-203: System shall log all alert events
 */

#include "ehms_types.h"
#include "alert_manager.h"
#include "alert_thresholds.h"
#include "eicas_interface.h"
#include "flight_recorder.h"

/* ============================================================================
 * PRIVATE CONSTANTS
 * ============================================================================ */

#define ALERT_MAX_QUEUE_SIZE        64U
#define ALERT_DEBOUNCE_CYCLES       3U
#define ALERT_HYSTERESIS_PERCENT    2.0f

/* ============================================================================
 * PRIVATE TYPES
 * ============================================================================ */

typedef struct
{
    ehms_alert_t        alerts[EHMS_MAX_ACTIVE_ALERTS];
    uint32_t            active_count;
    uint32_t            next_alert_id;
    bool                master_caution;
    bool                master_warning;
    ehms_alert_level_t  highest_level;
} alert_state_t;

typedef struct
{
    ehms_param_id_t     param_id;
    ehms_alert_level_t  level;
    float               threshold;
    bool                high_limit;     /* true = alert if above, false = alert if below */
    uint16_t            ecam_code;
    const char*         message;
} alert_threshold_t;

/* ============================================================================
 * PRIVATE DATA
 * ============================================================================ */

static alert_state_t s_alert_state;

static const alert_threshold_t s_thresholds[] = 
{
    /* EGT Limits */
    { EHMS_PARAM_EGT, EHMS_ALERT_CAUTION,  950.0f, true,  0x1001, "ENG %d EGT HIGH" },
    { EHMS_PARAM_EGT, EHMS_ALERT_WARNING,  1000.0f, true, 0x1002, "ENG %d EGT OVERLIMIT" },
    
    /* Oil Pressure Limits */
    { EHMS_PARAM_OIL_PRESS, EHMS_ALERT_CAUTION,  25.0f, false, 0x2001, "ENG %d OIL PRESS LO" },
    { EHMS_PARAM_OIL_PRESS, EHMS_ALERT_WARNING,  15.0f, false, 0x2002, "ENG %d OIL PRESS CRIT" },
    
    /* Oil Temperature Limits */
    { EHMS_PARAM_OIL_TEMP, EHMS_ALERT_CAUTION,  140.0f, true, 0x2003, "ENG %d OIL TEMP HI" },
    { EHMS_PARAM_OIL_TEMP, EHMS_ALERT_WARNING,  155.0f, true, 0x2004, "ENG %d OIL TEMP CRIT" },
    
    /* Vibration Limits */
    { EHMS_PARAM_VIB_FAN,  EHMS_ALERT_CAUTION,  3.0f, true, 0x3001, "ENG %d FAN VIB HI" },
    { EHMS_PARAM_VIB_FAN,  EHMS_ALERT_WARNING,  5.0f, true, 0x3002, "ENG %d FAN VIB CRIT" },
    { EHMS_PARAM_VIB_CORE, EHMS_ALERT_CAUTION,  4.0f, true, 0x3003, "ENG %d CORE VIB HI" },
    { EHMS_PARAM_VIB_CORE, EHMS_ALERT_WARNING,  6.0f, true, 0x3004, "ENG %d CORE VIB CRIT" },
    
    /* N1/N2 Limits */
    { EHMS_PARAM_N1, EHMS_ALERT_WARNING, 104.0f, true, 0x4001, "ENG %d N1 OVERLIMIT" },
    { EHMS_PARAM_N2, EHMS_ALERT_WARNING, 105.0f, true, 0x4002, "ENG %d N2 OVERLIMIT" },
};

#define NUM_THRESHOLDS (sizeof(s_thresholds) / sizeof(s_thresholds[0]))

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize alert manager
 * @return EHMS_OK on success
 * @trace SRS-EHMS-200
 */
ehms_result_t alert_init(void)
{
    (void)memset(&s_alert_state, 0, sizeof(s_alert_state));
    s_alert_state.next_alert_id = 1U;
    s_alert_state.highest_level = EHMS_ALERT_NONE;
    
    return EHMS_OK;
}

/**
 * @brief Process parameters and generate alerts
 * @param[in] snapshot Current engine snapshot
 * @return EHMS_OK on success
 * @trace SRS-EHMS-200, SRS-EHMS-201
 */
ehms_result_t alert_process_snapshot(const ehms_engine_snapshot_t* snapshot)
{
    ehms_result_t result = EHMS_OK;
    
    if (snapshot == NULL)
    {
        result = EHMS_ERROR_PARAM;
    }
    else
    {
        /* Check each threshold */
        for (uint32_t t = 0U; t < NUM_THRESHOLDS; t++)
        {
            const alert_threshold_t* thresh = &s_thresholds[t];
            const ehms_parameter_t* param = &snapshot->parameters[thresh->param_id];
            
            /* Skip invalid parameters */
            if (param->status != EHMS_PARAM_VALID)
            {
                continue;
            }
            
            bool exceeded = false;
            
            if (thresh->high_limit)
            {
                exceeded = (param->eng_value >= thresh->threshold);
            }
            else
            {
                exceeded = (param->eng_value <= thresh->threshold);
            }
            
            if (exceeded)
            {
                /* Check if alert already active */
                bool already_active = false;
                for (uint32_t a = 0U; a < s_alert_state.active_count; a++)
                {
                    if ((s_alert_state.alerts[a].param_id == thresh->param_id) &&
                        (s_alert_state.alerts[a].engine_id == snapshot->engine_id) &&
                        (s_alert_state.alerts[a].level == thresh->level))
                    {
                        already_active = true;
                        break;
                    }
                }
                
                if (!already_active && (s_alert_state.active_count < EHMS_MAX_ACTIVE_ALERTS))
                {
                    /* Create new alert */
                    ehms_alert_t* new_alert = &s_alert_state.alerts[s_alert_state.active_count];
                    
                    new_alert->alert_id = s_alert_state.next_alert_id++;
                    new_alert->level = thresh->level;
                    new_alert->engine_id = snapshot->engine_id;
                    new_alert->param_id = thresh->param_id;
                    new_alert->onset_time = snapshot->sample_time;
                    new_alert->is_active = true;
                    new_alert->is_latched = (thresh->level >= EHMS_ALERT_WARNING);
                    new_alert->ecam_code = thresh->ecam_code;
                    
                    (void)snprintf(new_alert->message, sizeof(new_alert->message),
                                  thresh->message, (int)snapshot->engine_id + 1);
                    
                    s_alert_state.active_count++;
                    
                    /* Update master alerts */
                    if (thresh->level >= EHMS_ALERT_WARNING)
                    {
                        s_alert_state.master_warning = true;
                    }
                    else if (thresh->level >= EHMS_ALERT_CAUTION)
                    {
                        s_alert_state.master_caution = true;
                    }
                    
                    /* Update highest level */
                    if (thresh->level > s_alert_state.highest_level)
                    {
                        s_alert_state.highest_level = thresh->level;
                    }
                    
                    /* Send to EICAS */
                    (void)eicas_post_message(new_alert);
                    
                    /* Log to flight recorder */
                    (void)recorder_log_alert(new_alert);
                }
            }
        }
    }
    
    return result;
}

/**
 * @brief Get active alert count
 * @return Number of active alerts
 */
uint32_t alert_get_active_count(void)
{
    return s_alert_state.active_count;
}

/**
 * @brief Get highest active alert level
 * @return Highest alert level
 */
ehms_alert_level_t alert_get_highest_level(void)
{
    return s_alert_state.highest_level;
}

/**
 * @brief Check master warning status
 * @return true if master warning active
 */
bool alert_is_master_warning(void)
{
    return s_alert_state.master_warning;
}

/**
 * @brief Check master caution status
 * @return true if master caution active
 */
bool alert_is_master_caution(void)
{
    return s_alert_state.master_caution;
}

/**
 * @brief Acknowledge and clear master caution/warning
 * @param[in] level Level to acknowledge
 * @return EHMS_OK on success
 * @trace SRS-EHMS-202
 */
ehms_result_t alert_acknowledge(ehms_alert_level_t level)
{
    if (level >= EHMS_ALERT_WARNING)
    {
        s_alert_state.master_warning = false;
    }
    else if (level >= EHMS_ALERT_CAUTION)
    {
        s_alert_state.master_caution = false;
    }
    
    return EHMS_OK;
}

/* END OF FILE */

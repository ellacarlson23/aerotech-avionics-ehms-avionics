/**
 * @file data_acquisition.c
 * @brief EHMS Data Acquisition Module
 * 
 * @copyright Copyright (c) 2025 AeroTech Avionics Inc.
 * @note DO-178C Level B - Safety Critical Software
 * 
 * @version 2.4.1
 * @date 2025-01-15
 * 
 * CSCI: EHMS-CORE
 * CSC: DATA-ACQUISITION
 * 
 * Requirements Trace:
 *   SRS-EHMS-100: System shall acquire engine parameters at 100Hz
 *   SRS-EHMS-101: System shall validate all incoming data
 *   SRS-EHMS-102: System shall detect stale data within 100ms
 *   SRS-EHMS-103: System shall support redundant data sources
 */

/* ============================================================================
 * INCLUDES
 * ============================================================================ */
#include "ehms_types.h"
#include "ehms_config.h"
#include "data_acquisition.h"
#include "arinc429_driver.h"
#include "milstd1553_driver.h"
#include "parameter_database.h"
#include "error_handler.h"

#include <string.h>

/* ============================================================================
 * PRIVATE CONSTANTS
 * ============================================================================ */

/** @brief Data staleness timeout in milliseconds */
#define DAQ_STALE_TIMEOUT_MS            100U

/** @brief Maximum consecutive failures before source marked failed */
#define DAQ_MAX_CONSECUTIVE_FAILURES    5U

/** @brief Acquisition cycle period in microseconds (10ms = 100Hz) */
#define DAQ_CYCLE_PERIOD_US             10000U

/** @brief CRC polynomial for data validation */
#define DAQ_CRC32_POLYNOMIAL            0xEDB88320UL

/* ============================================================================
 * PRIVATE TYPES
 * ============================================================================ */

/**
 * @brief Data source tracking structure
 */
typedef struct
{
    bool                is_active;              /**< Source is active */
    bool                is_primary;             /**< Is primary source */
    uint8_t             bus_id;                 /**< Bus identifier */
    uint32_t            last_update_ms;         /**< Last update timestamp */
    uint32_t            failure_count;          /**< Consecutive failure count */
    uint32_t            total_samples;          /**< Total samples received */
    uint32_t            error_samples;          /**< Total error samples */
} daq_source_info_t;

/**
 * @brief Module state structure
 */
typedef struct
{
    bool                        is_initialized;
    ehms_system_state_t         state;
    uint32_t                    cycle_count;
    uint32_t                    current_time_ms;
    daq_source_info_t           sources[EHMS_ARINC429_BUS_COUNT];
    ehms_engine_snapshot_t      engine_data[EHMS_MAX_ENGINES];
    ehms_result_t               last_error;
} daq_module_state_t;

/* ============================================================================
 * PRIVATE DATA
 * ============================================================================ */

/** @brief Module state - static allocation for safety */
static daq_module_state_t s_daq_state;

/** @brief Parameter configuration table */
static const daq_param_config_t s_param_config[EHMS_PARAM_COUNT] = 
{
    /* param_id,           arinc_label, bus_primary, bus_backup, scale_factor, offset */
    { EHMS_PARAM_N1,       0o310,       0,           1,          0.1f,         0.0f   },
    { EHMS_PARAM_N2,       0o311,       0,           1,          0.1f,         0.0f   },
    { EHMS_PARAM_EGT,      0o312,       0,           1,          1.0f,         0.0f   },
    { EHMS_PARAM_FF,       0o313,       0,           1,          0.1f,         0.0f   },
    { EHMS_PARAM_OIL_TEMP, 0o314,       0,           1,          0.5f,         -40.0f },
    { EHMS_PARAM_OIL_PRESS,0o315,       0,           1,          0.1f,         0.0f   },
    { EHMS_PARAM_OIL_QTY,  0o316,       0,           1,          0.5f,         0.0f   },
    { EHMS_PARAM_VIB_FAN,  0o317,       2,           3,          0.001f,       0.0f   },
    { EHMS_PARAM_VIB_CORE, 0o320,       2,           3,          0.001f,       0.0f   },
    { EHMS_PARAM_EPR,      0o321,       0,           1,          0.001f,       0.0f   },
    /* ... remaining parameters configured similarly */
};

/* ============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ============================================================================ */

static ehms_result_t daq_validate_config(const daq_config_t* config);
static ehms_result_t daq_init_sources(void);
static ehms_result_t daq_read_arinc429_data(uint8_t bus_id, ehms_engine_id_t engine);
static ehms_result_t daq_read_1553_data(ehms_engine_id_t engine);
static ehms_result_t daq_validate_parameter(ehms_parameter_t* param);
static ehms_result_t daq_check_staleness(ehms_parameter_t* param);
static ehms_result_t daq_select_source(ehms_param_id_t param_id, uint8_t* selected_bus);
static uint32_t daq_calculate_crc32(const void* data, uint32_t length);
static void daq_update_statistics(uint8_t bus_id, bool success);

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize data acquisition module
 * 
 * @param[in] config  Pointer to configuration structure
 * @return EHMS_OK on success, error code otherwise
 * 
 * @trace SRS-EHMS-100
 * @trace SRS-EHMS-105
 */
ehms_result_t daq_init(const daq_config_t* config)
{
    ehms_result_t result = EHMS_OK;
    
    /* Validate input parameters */
    if (config == NULL)
    {
        result = EHMS_ERROR_PARAM;
    }
    else
    {
        result = daq_validate_config(config);
    }
    
    if (result == EHMS_OK)
    {
        /* Clear module state */
        (void)memset(&s_daq_state, 0, sizeof(s_daq_state));
        
        /* Initialize data sources */
        result = daq_init_sources();
    }
    
    if (result == EHMS_OK)
    {
        /* Initialize ARINC 429 interfaces */
        for (uint8_t bus = 0U; bus < EHMS_ARINC429_BUS_COUNT; bus++)
        {
            result = arinc429_init(bus, config->arinc_config[bus]);
            if (result != EHMS_OK)
            {
                error_report(ERR_MODULE_DAQ, ERR_SEVERITY_MAJOR, 
                            ERR_CODE_INIT_FAILED, bus);
                break;
            }
        }
    }
    
    if (result == EHMS_OK)
    {
        /* Initialize MIL-STD-1553B interface */
        result = milstd1553_init(EHMS_1553_RT_ADDRESS);
    }
    
    if (result == EHMS_OK)
    {
        s_daq_state.is_initialized = true;
        s_daq_state.state = EHMS_STATE_INIT;
    }
    
    return result;
}

/**
 * @brief Execute one acquisition cycle
 * 
 * @return EHMS_OK on success, error code otherwise
 * 
 * @trace SRS-EHMS-100
 * @trace SRS-EHMS-101
 * 
 * @note This function shall be called at 100Hz from the cyclic executive
 */
ehms_result_t daq_execute_cycle(void)
{
    ehms_result_t result = EHMS_OK;
    ehms_result_t engine_result;
    
    /* Check initialization */
    if (!s_daq_state.is_initialized)
    {
        result = EHMS_ERROR_NOT_INIT;
    }
    else
    {
        /* Update cycle timestamp */
        s_daq_state.current_time_ms = system_get_time_ms();
        s_daq_state.cycle_count++;
        
        /* Acquire data for each engine */
        for (ehms_engine_id_t eng = EHMS_ENGINE_1; 
             eng < (ehms_engine_id_t)config_get_engine_count(); 
             eng++)
        {
            /* Read ARINC 429 data */
            engine_result = daq_read_arinc429_data(
                s_param_config[0].bus_primary, eng);
            
            if (engine_result != EHMS_OK)
            {
                /* Try backup source */
                engine_result = daq_read_arinc429_data(
                    s_param_config[0].bus_backup, eng);
            }
            
            /* Read 1553 data (vibration, discrete) */
            if (engine_result == EHMS_OK)
            {
                engine_result = daq_read_1553_data(eng);
            }
            
            /* Validate all parameters */
            for (uint32_t p = 0U; p < EHMS_PARAM_COUNT; p++)
            {
                ehms_parameter_t* param = 
                    &s_daq_state.engine_data[eng].parameters[p];
                
                (void)daq_validate_parameter(param);
                (void)daq_check_staleness(param);
            }
            
            /* Calculate snapshot CRC */
            s_daq_state.engine_data[eng].crc32 = daq_calculate_crc32(
                &s_daq_state.engine_data[eng],
                sizeof(ehms_engine_snapshot_t) - sizeof(uint32_t));
            
            /* Update snapshot timestamp */
            s_daq_state.engine_data[eng].sample_time = 
                system_get_timestamp();
        }
    }
    
    return result;
}

/**
 * @brief Get current engine snapshot data
 * 
 * @param[in]  engine_id  Engine identifier
 * @param[out] snapshot   Pointer to receive snapshot data
 * @return EHMS_OK on success, error code otherwise
 * 
 * @trace SRS-EHMS-110
 */
ehms_result_t daq_get_engine_snapshot(ehms_engine_id_t engine_id,
                                       ehms_engine_snapshot_t* snapshot)
{
    ehms_result_t result = EHMS_OK;
    
    /* Validate parameters */
    if (snapshot == NULL)
    {
        result = EHMS_ERROR_PARAM;
    }
    else if (engine_id >= EHMS_ENGINE_COUNT)
    {
        result = EHMS_ERROR_RANGE;
    }
    else if (!s_daq_state.is_initialized)
    {
        result = EHMS_ERROR_NOT_INIT;
    }
    else
    {
        /* Verify CRC before copying */
        uint32_t calc_crc = daq_calculate_crc32(
            &s_daq_state.engine_data[engine_id],
            sizeof(ehms_engine_snapshot_t) - sizeof(uint32_t));
        
        if (calc_crc != s_daq_state.engine_data[engine_id].crc32)
        {
            result = EHMS_ERROR_CRC;
            error_report(ERR_MODULE_DAQ, ERR_SEVERITY_MAJOR,
                        ERR_CODE_CRC_MISMATCH, (uint32_t)engine_id);
        }
        else
        {
            /* Copy snapshot data */
            (void)memcpy(snapshot, &s_daq_state.engine_data[engine_id],
                        sizeof(ehms_engine_snapshot_t));
        }
    }
    
    return result;
}

/**
 * @brief Get single parameter value
 * 
 * @param[in]  engine_id  Engine identifier
 * @param[in]  param_id   Parameter identifier
 * @param[out] param      Pointer to receive parameter data
 * @return EHMS_OK on success, error code otherwise
 * 
 * @trace SRS-EHMS-111
 */
ehms_result_t daq_get_parameter(ehms_engine_id_t engine_id,
                                 ehms_param_id_t param_id,
                                 ehms_parameter_t* param)
{
    ehms_result_t result = EHMS_OK;
    
    /* Validate parameters */
    if (param == NULL)
    {
        result = EHMS_ERROR_PARAM;
    }
    else if ((engine_id >= EHMS_ENGINE_COUNT) || 
             (param_id >= EHMS_PARAM_COUNT))
    {
        result = EHMS_ERROR_RANGE;
    }
    else if (!s_daq_state.is_initialized)
    {
        result = EHMS_ERROR_NOT_INIT;
    }
    else
    {
        /* Copy parameter data */
        (void)memcpy(param, 
                    &s_daq_state.engine_data[engine_id].parameters[param_id],
                    sizeof(ehms_parameter_t));
    }
    
    return result;
}

/**
 * @brief Get data acquisition statistics
 * 
 * @param[out] stats  Pointer to receive statistics
 * @return EHMS_OK on success, error code otherwise
 */
ehms_result_t daq_get_statistics(daq_statistics_t* stats)
{
    ehms_result_t result = EHMS_OK;
    
    if (stats == NULL)
    {
        result = EHMS_ERROR_PARAM;
    }
    else
    {
        stats->cycle_count = s_daq_state.cycle_count;
        stats->current_time_ms = s_daq_state.current_time_ms;
        
        for (uint8_t i = 0U; i < EHMS_ARINC429_BUS_COUNT; i++)
        {
            stats->source_samples[i] = s_daq_state.sources[i].total_samples;
            stats->source_errors[i] = s_daq_state.sources[i].error_samples;
        }
    }
    
    return result;
}

/* ============================================================================
 * PRIVATE FUNCTIONS
 * ============================================================================ */

/**
 * @brief Validate configuration structure
 */
static ehms_result_t daq_validate_config(const daq_config_t* config)
{
    ehms_result_t result = EHMS_OK;
    
    if (config->sample_rate_hz > EHMS_MAX_SAMPLE_RATE_HZ)
    {
        result = EHMS_ERROR_RANGE;
    }
    else if (config->engine_count > EHMS_MAX_ENGINES)
    {
        result = EHMS_ERROR_RANGE;
    }
    
    return result;
}

/**
 * @brief Initialize data source tracking
 */
static ehms_result_t daq_init_sources(void)
{
    for (uint8_t i = 0U; i < EHMS_ARINC429_BUS_COUNT; i++)
    {
        s_daq_state.sources[i].is_active = true;
        s_daq_state.sources[i].is_primary = (i < 2U);
        s_daq_state.sources[i].bus_id = i;
        s_daq_state.sources[i].last_update_ms = 0U;
        s_daq_state.sources[i].failure_count = 0U;
        s_daq_state.sources[i].total_samples = 0U;
        s_daq_state.sources[i].error_samples = 0U;
    }
    
    return EHMS_OK;
}

/**
 * @brief Read ARINC 429 data for specified engine
 */
static ehms_result_t daq_read_arinc429_data(uint8_t bus_id, 
                                             ehms_engine_id_t engine)
{
    ehms_result_t result = EHMS_OK;
    arinc429_word_t word;
    
    for (uint32_t p = 0U; p < EHMS_PARAM_COUNT; p++)
    {
        if (s_param_config[p].bus_primary == bus_id)
        {
            /* Read ARINC 429 word */
            result = arinc429_read(bus_id, s_param_config[p].arinc_label, &word);
            
            if (result == EHMS_OK)
            {
                /* Convert to parameter */
                ehms_parameter_t* param = 
                    &s_daq_state.engine_data[engine].parameters[p];
                
                param->param_id = (ehms_param_id_t)p;
                param->raw_value = word.data;
                param->eng_value = (float)word.data * s_param_config[p].scale_factor
                                 + s_param_config[p].offset;
                param->source_bus = bus_id;
                param->status = EHMS_PARAM_VALID;
                param->timestamp = system_get_timestamp();
                
                daq_update_statistics(bus_id, true);
            }
            else
            {
                daq_update_statistics(bus_id, false);
            }
        }
    }
    
    return result;
}

/**
 * @brief Read MIL-STD-1553B data
 */
static ehms_result_t daq_read_1553_data(ehms_engine_id_t engine)
{
    ehms_result_t result;
    milstd1553_message_t msg;
    
    /* Read vibration data (subaddress 5) */
    result = milstd1553_read_subaddress(5U, &msg);
    
    if (result == EHMS_OK)
    {
        /* Parse vibration data from message */
        ehms_parameter_t* vib_fan = 
            &s_daq_state.engine_data[engine].parameters[EHMS_PARAM_VIB_FAN];
        ehms_parameter_t* vib_core = 
            &s_daq_state.engine_data[engine].parameters[EHMS_PARAM_VIB_CORE];
        
        vib_fan->raw_value = msg.data[0];
        vib_fan->eng_value = (float)msg.data[0] * 0.001f;
        vib_fan->status = EHMS_PARAM_VALID;
        
        vib_core->raw_value = msg.data[1];
        vib_core->eng_value = (float)msg.data[1] * 0.001f;
        vib_core->status = EHMS_PARAM_VALID;
    }
    
    return result;
}

/**
 * @brief Validate parameter against limits
 */
static ehms_result_t daq_validate_parameter(ehms_parameter_t* param)
{
    ehms_result_t result = EHMS_OK;
    
    /* Get limits from database */
    param_limits_t limits;
    if (param_db_get_limits(param->param_id, &limits) == EHMS_OK)
    {
        if ((param->eng_value < limits.min_value) ||
            (param->eng_value > limits.max_value))
        {
            param->status = EHMS_PARAM_FAILED;
            result = EHMS_ERROR_RANGE;
        }
    }
    
    return result;
}

/**
 * @brief Check for stale data
 */
static ehms_result_t daq_check_staleness(ehms_parameter_t* param)
{
    ehms_result_t result = EHMS_OK;
    uint32_t param_age_ms;
    
    /* Calculate age of data */
    param_age_ms = s_daq_state.current_time_ms - 
                   timestamp_to_ms(&param->timestamp);
    
    if (param_age_ms > DAQ_STALE_TIMEOUT_MS)
    {
        if (param->status == EHMS_PARAM_VALID)
        {
            param->status = EHMS_PARAM_STALE;
        }
        result = EHMS_ERROR_TIMEOUT;
    }
    
    return result;
}

/**
 * @brief Calculate CRC-32
 */
static uint32_t daq_calculate_crc32(const void* data, uint32_t length)
{
    const uint8_t* bytes = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFFUL;
    
    for (uint32_t i = 0U; i < length; i++)
    {
        crc ^= bytes[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1U) ^ DAQ_CRC32_POLYNOMIAL;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    
    return ~crc;
}

/**
 * @brief Update source statistics
 */
static void daq_update_statistics(uint8_t bus_id, bool success)
{
    if (bus_id < EHMS_ARINC429_BUS_COUNT)
    {
        s_daq_state.sources[bus_id].total_samples++;
        s_daq_state.sources[bus_id].last_update_ms = s_daq_state.current_time_ms;
        
        if (!success)
        {
            s_daq_state.sources[bus_id].error_samples++;
            s_daq_state.sources[bus_id].failure_count++;
            
            if (s_daq_state.sources[bus_id].failure_count >= 
                DAQ_MAX_CONSECUTIVE_FAILURES)
            {
                s_daq_state.sources[bus_id].is_active = false;
            }
        }
        else
        {
            s_daq_state.sources[bus_id].failure_count = 0U;
        }
    }
}

/* END OF FILE */

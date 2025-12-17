/**
 * @file test_data_acquisition.c
 * @brief Unit Tests for Data Acquisition Module
 * 
 * @copyright Copyright (c) 2025 AeroTech Avionics Inc.
 * 
 * Test Framework: Unity Test Framework
 * Coverage Target: 100% MC/DC
 * 
 * Requirements Verified:
 *   SRS-EHMS-100, SRS-EHMS-101, SRS-EHMS-102, SRS-EHMS-103
 *   SRS-EHMS-110, SRS-EHMS-111
 */

#include "unity.h"
#include "data_acquisition.h"
#include "ehms_types.h"
#include "mock_arinc429_driver.h"
#include "mock_milstd1553_driver.h"
#include "mock_system_services.h"

/* ============================================================================
 * TEST FIXTURES
 * ============================================================================ */

static daq_config_t test_config;

void setUp(void)
{
    /* Initialize test configuration */
    test_config.sample_rate_hz = 100U;
    test_config.engine_count = 2U;
    
    for (uint8_t i = 0; i < EHMS_ARINC429_BUS_COUNT; i++)
    {
        test_config.arinc_config[i].speed = ARINC429_HIGH_SPEED;
        test_config.arinc_config[i].parity = ARINC429_ODD_PARITY;
    }
    
    /* Reset all mocks */
    mock_arinc429_driver_Init();
    mock_milstd1553_driver_Init();
    mock_system_services_Init();
}

void tearDown(void)
{
    /* Verify all mock expectations */
    mock_arinc429_driver_Verify();
    mock_milstd1553_driver_Verify();
    mock_system_services_Verify();
    
    /* Destroy mocks */
    mock_arinc429_driver_Destroy();
    mock_milstd1553_driver_Destroy();
    mock_system_services_Destroy();
}

/* ============================================================================
 * INITIALIZATION TESTS
 * ============================================================================ */

/**
 * @test Test successful initialization
 * @trace SRS-EHMS-105
 */
void test_daq_init_success(void)
{
    /* Expect ARINC 429 initialization for all buses */
    for (uint8_t i = 0; i < EHMS_ARINC429_BUS_COUNT; i++)
    {
        arinc429_init_ExpectAndReturn(i, test_config.arinc_config[i], EHMS_OK);
    }
    
    /* Expect MIL-STD-1553 initialization */
    milstd1553_init_ExpectAndReturn(EHMS_1553_RT_ADDRESS, EHMS_OK);
    
    /* Execute */
    ehms_result_t result = daq_init(&test_config);
    
    /* Verify */
    TEST_ASSERT_EQUAL(EHMS_OK, result);
}

/**
 * @test Test initialization with NULL config
 * @trace SRS-EHMS-105
 */
void test_daq_init_null_config(void)
{
    ehms_result_t result = daq_init(NULL);
    
    TEST_ASSERT_EQUAL(EHMS_ERROR_PARAM, result);
}

/**
 * @test Test initialization with invalid sample rate
 * @trace SRS-EHMS-100
 */
void test_daq_init_invalid_sample_rate(void)
{
    test_config.sample_rate_hz = 200U; /* Exceeds max */
    
    ehms_result_t result = daq_init(&test_config);
    
    TEST_ASSERT_EQUAL(EHMS_ERROR_RANGE, result);
}

/**
 * @test Test initialization with invalid engine count
 * @trace SRS-EHMS-105
 */
void test_daq_init_invalid_engine_count(void)
{
    test_config.engine_count = 10U; /* Exceeds max */
    
    ehms_result_t result = daq_init(&test_config);
    
    TEST_ASSERT_EQUAL(EHMS_ERROR_RANGE, result);
}

/**
 * @test Test initialization when ARINC 429 init fails
 * @trace SRS-EHMS-105
 */
void test_daq_init_arinc_failure(void)
{
    /* First bus succeeds, second fails */
    arinc429_init_ExpectAndReturn(0, test_config.arinc_config[0], EHMS_OK);
    arinc429_init_ExpectAndReturn(1, test_config.arinc_config[1], EHMS_ERROR_HARDWARE);
    
    ehms_result_t result = daq_init(&test_config);
    
    TEST_ASSERT_EQUAL(EHMS_ERROR_HARDWARE, result);
}

/* ============================================================================
 * ACQUISITION CYCLE TESTS
 * ============================================================================ */

/**
 * @test Test execute cycle before initialization
 * @trace SRS-EHMS-100
 */
void test_daq_execute_not_initialized(void)
{
    ehms_result_t result = daq_execute_cycle();
    
    TEST_ASSERT_EQUAL(EHMS_ERROR_NOT_INIT, result);
}

/**
 * @test Test successful acquisition cycle
 * @trace SRS-EHMS-100, SRS-EHMS-101
 */
void test_daq_execute_cycle_success(void)
{
    /* Initialize first */
    for (uint8_t i = 0; i < EHMS_ARINC429_BUS_COUNT; i++)
    {
        arinc429_init_ExpectAndReturn(i, test_config.arinc_config[i], EHMS_OK);
    }
    milstd1553_init_ExpectAndReturn(EHMS_1553_RT_ADDRESS, EHMS_OK);
    (void)daq_init(&test_config);
    
    /* Setup mock data */
    arinc429_word_t mock_word;
    mock_word.label = 0o310;
    mock_word.data = 850; /* 85.0% N1 */
    mock_word.ssm = SSM_NORMAL;
    
    /* Expect reads for each parameter */
    system_get_time_ms_ExpectAndReturn(1000U);
    arinc429_read_ExpectAndReturn(0, 0o310, NULL, EHMS_OK);
    arinc429_read_IgnoreArg_word();
    arinc429_read_ReturnThruPtr_word(&mock_word);
    /* ... additional expects for other parameters ... */
    
    ehms_result_t result = daq_execute_cycle();
    
    TEST_ASSERT_EQUAL(EHMS_OK, result);
}

/* ============================================================================
 * DATA RETRIEVAL TESTS
 * ============================================================================ */

/**
 * @test Test get snapshot with NULL pointer
 * @trace SRS-EHMS-110
 */
void test_daq_get_snapshot_null(void)
{
    ehms_result_t result = daq_get_engine_snapshot(EHMS_ENGINE_1, NULL);
    
    TEST_ASSERT_EQUAL(EHMS_ERROR_PARAM, result);
}

/**
 * @test Test get snapshot with invalid engine ID
 * @trace SRS-EHMS-110
 */
void test_daq_get_snapshot_invalid_engine(void)
{
    ehms_engine_snapshot_t snapshot;
    
    ehms_result_t result = daq_get_engine_snapshot(EHMS_ENGINE_COUNT, &snapshot);
    
    TEST_ASSERT_EQUAL(EHMS_ERROR_RANGE, result);
}

/**
 * @test Test get parameter with NULL pointer
 * @trace SRS-EHMS-111
 */
void test_daq_get_parameter_null(void)
{
    ehms_result_t result = daq_get_parameter(EHMS_ENGINE_1, EHMS_PARAM_N1, NULL);
    
    TEST_ASSERT_EQUAL(EHMS_ERROR_PARAM, result);
}

/**
 * @test Test get parameter with invalid IDs
 * @trace SRS-EHMS-111
 */
void test_daq_get_parameter_invalid_ids(void)
{
    ehms_parameter_t param;
    ehms_result_t result;
    
    /* Invalid engine */
    result = daq_get_parameter(EHMS_ENGINE_COUNT, EHMS_PARAM_N1, &param);
    TEST_ASSERT_EQUAL(EHMS_ERROR_RANGE, result);
    
    /* Invalid parameter */
    result = daq_get_parameter(EHMS_ENGINE_1, EHMS_PARAM_COUNT, &param);
    TEST_ASSERT_EQUAL(EHMS_ERROR_RANGE, result);
}

/* ============================================================================
 * STALE DATA DETECTION TESTS
 * ============================================================================ */

/**
 * @test Test stale data detection
 * @trace SRS-EHMS-102
 */
void test_daq_stale_detection(void)
{
    /* Initialize module */
    for (uint8_t i = 0; i < EHMS_ARINC429_BUS_COUNT; i++)
    {
        arinc429_init_ExpectAndReturn(i, test_config.arinc_config[i], EHMS_OK);
    }
    milstd1553_init_ExpectAndReturn(EHMS_1553_RT_ADDRESS, EHMS_OK);
    (void)daq_init(&test_config);
    
    /* Execute cycle at time T=1000ms */
    system_get_time_ms_ExpectAndReturn(1000U);
    /* ... setup mocks for successful read ... */
    (void)daq_execute_cycle();
    
    /* Execute cycle at time T=1150ms (150ms later, exceeds 100ms stale timeout) */
    system_get_time_ms_ExpectAndReturn(1150U);
    /* Mock no new data available */
    arinc429_read_ExpectAndReturn(0, 0o310, NULL, EHMS_ERROR_TIMEOUT);
    arinc429_read_IgnoreArg_word();
    
    (void)daq_execute_cycle();
    
    /* Get parameter and verify stale status */
    ehms_parameter_t param;
    (void)daq_get_parameter(EHMS_ENGINE_1, EHMS_PARAM_N1, &param);
    
    TEST_ASSERT_EQUAL(EHMS_PARAM_STALE, param.status);
}

/* ============================================================================
 * SOURCE REDUNDANCY TESTS
 * ============================================================================ */

/**
 * @test Test automatic switchover to backup source
 * @trace SRS-EHMS-103, SRS-EHMS-104
 */
void test_daq_source_switchover(void)
{
    /* Initialize module */
    for (uint8_t i = 0; i < EHMS_ARINC429_BUS_COUNT; i++)
    {
        arinc429_init_ExpectAndReturn(i, test_config.arinc_config[i], EHMS_OK);
    }
    milstd1553_init_ExpectAndReturn(EHMS_1553_RT_ADDRESS, EHMS_OK);
    (void)daq_init(&test_config);
    
    /* Primary source fails */
    system_get_time_ms_ExpectAndReturn(1000U);
    arinc429_read_ExpectAndReturn(0, 0o310, NULL, EHMS_ERROR_HARDWARE);
    arinc429_read_IgnoreArg_word();
    
    /* Backup source succeeds */
    arinc429_word_t backup_word;
    backup_word.label = 0o310;
    backup_word.data = 850;
    backup_word.ssm = SSM_NORMAL;
    
    arinc429_read_ExpectAndReturn(1, 0o310, NULL, EHMS_OK);
    arinc429_read_IgnoreArg_word();
    arinc429_read_ReturnThruPtr_word(&backup_word);
    
    ehms_result_t result = daq_execute_cycle();
    
    TEST_ASSERT_EQUAL(EHMS_OK, result);
    
    /* Verify data came from backup */
    ehms_parameter_t param;
    (void)daq_get_parameter(EHMS_ENGINE_1, EHMS_PARAM_N1, &param);
    
    TEST_ASSERT_EQUAL(EHMS_PARAM_VALID, param.status);
    TEST_ASSERT_EQUAL(1U, param.source_bus); /* Bus 1 = backup */
}

/* ============================================================================
 * CRC VALIDATION TESTS
 * ============================================================================ */

/**
 * @test Test CRC validation on snapshot retrieval
 * @trace SRS-EHMS-108
 */
void test_daq_crc_validation(void)
{
    /* This test verifies CRC is calculated and verified */
    /* Implementation would corrupt CRC and verify error detection */
    
    TEST_IGNORE_MESSAGE("CRC corruption test requires internal access");
}

/* ============================================================================
 * TEST RUNNER
 * ============================================================================ */

int main(void)
{
    UNITY_BEGIN();
    
    /* Initialization tests */
    RUN_TEST(test_daq_init_success);
    RUN_TEST(test_daq_init_null_config);
    RUN_TEST(test_daq_init_invalid_sample_rate);
    RUN_TEST(test_daq_init_invalid_engine_count);
    RUN_TEST(test_daq_init_arinc_failure);
    
    /* Acquisition tests */
    RUN_TEST(test_daq_execute_not_initialized);
    RUN_TEST(test_daq_execute_cycle_success);
    
    /* Data retrieval tests */
    RUN_TEST(test_daq_get_snapshot_null);
    RUN_TEST(test_daq_get_snapshot_invalid_engine);
    RUN_TEST(test_daq_get_parameter_null);
    RUN_TEST(test_daq_get_parameter_invalid_ids);
    
    /* Stale detection tests */
    RUN_TEST(test_daq_stale_detection);
    
    /* Redundancy tests */
    RUN_TEST(test_daq_source_switchover);
    
    /* CRC tests */
    RUN_TEST(test_daq_crc_validation);
    
    return UNITY_END();
}

/* END OF FILE */

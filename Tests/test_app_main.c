/**
 ******************************************************************************
 * @file           : test_app_main.c
 * @brief          : Host tests for the example application's start-up
 * @author         : Grisham Balloo
 ******************************************************************************
 * @details
 * Runs AppMain from App/Src/app_main.c against the real driver and the scripted
 * transport. The STM32 port functions, HAL_Delay, SEGGER RTT and the stress
 * runner are fakes: the port forwards to the scripted transport and records the
 * bus's read-retry setting at every transmit, RTT collects the boot log, and
 * HAL_Delay(1000), used only by the halt loop, jumps back to the test.
 ******************************************************************************
 */
#include "unity.h"
#include "sts_servo.h"
#include "sts_servo_cmd.h"
#include "sts_registers.h"
#include "sts_ports_stm32.h"
#include "test_sts_utils.h"
#include "test_app_main.h"
#include "app_main.h"
#include "main.h"
#include "Hw_Tests.h"
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define APP_FIRST_ID        1U
#define APP_LAST_ID         15U
#define APP_SERVO_ID        3U
#define APP_SCAN_RETRIES    1U        /* Required during the scan */
#define APP_RUN_RETRIES     2U        /* Required after the scan */
#define APP_STRESS_RUNS     200U
#define APP_ACK_LEN         6U
#define APP_LOG_SIZE        2048U
#define APP_TX_RECORDS      64U
#define APP_IDX_ID          2U
#define APP_IDX_INSTRUCTION 4U
#define APP_IDX_ADDRESS     5U
#define APP_IDX_VALUE       6U
#define APP_INST_PING       0x01U
#define APP_INST_WRITE      0x03U

UART_HandleTypeDef huart2;

static mock_script_t app_script;
static char          app_log[APP_LOG_SIZE];
static size_t        app_log_len;
static jmp_buf       app_halt;
static uint32_t      delays_10ms;
static uint32_t      delays_50ms;
static uint8_t       retries_at_tx[APP_TX_RECORDS];
static uint32_t      tx_seen;
static uint8_t       half_duplex;
static uint32_t      stress_calls;
static uint8_t       stress_id;
static uint8_t       stress_retries;
static uint32_t      stress_runs;

/* --- Fakes --- */

int SEGGER_RTT_printf(unsigned BufferIndex, const char *sFormat, ...) {
    (void)BufferIndex;
    va_list args;
    va_start(args, sFormat);
    int n = vsnprintf(&app_log[app_log_len], APP_LOG_SIZE - app_log_len, sFormat, args);
    va_end(args);
    TEST_ASSERT_TRUE_MESSAGE(n >= 0 && (size_t)n < APP_LOG_SIZE - app_log_len, "app log full");
    app_log_len += (size_t)n;
    return n;
}

void HAL_Delay(uint32_t ms) {
    if (ms == 10U) {
        delays_10ms++;
    } else if (ms == 50U) {
        delays_50ms++;
    } else if (ms == 1000U) {
        longjmp(app_halt, 1);   /* the halt loop: leave AppMain */
    }
}

static sts_bus_t scripted(const sts_bus_t *bus) {
    sts_bus_t b = *bus;
    b.port_handle = &app_script;
    return b;
}

sts_result_t STM32_UART_Transmit(sts_bus_t *bus, const uint8_t *data, uint16_t len) {
    if (tx_seen < APP_TX_RECORDS) {
        retries_at_tx[tx_seen] = bus->max_read_retries;
    }
    tx_seen++;
    sts_bus_t b = scripted(bus);
    return mock_script_tx(&b, data, len);
}

sts_result_t STM32_UART_Receive(sts_bus_t *bus, uint8_t *data, uint16_t len, uint32_t timeout) {
    sts_bus_t b = scripted(bus);
    return mock_script_rx(&b, data, len, timeout);
}

sts_result_t STM32_UART_FlushRx(sts_bus_t *bus) {
    sts_bus_t b = scripted(bus);
    return mock_script_flush(&b);
}

void STM32_UART_SetHalfDuplex(void *huart, uint8_t enabled) {
    (void)huart;
    half_duplex = enabled;
}

void STS_RunStressTest(sts_servo_t *servo, uint32_t iterations) {
    stress_calls++;
    stress_id      = servo->id;
    stress_retries = servo->bus->max_read_retries;
    stress_runs    = iterations;
}

/* --- Helpers --- */

static uint8_t ack_servo[APP_ACK_LEN];
static uint8_t ack_write[APP_ACK_LEN];
static uint8_t ack_bad_1[APP_ACK_LEN];
static uint8_t ack_bad_servo[APP_ACK_LEN];
static uint8_t ack_hw_servo[APP_ACK_LEN];

void setUp(void) {
    memset(app_log, 0, sizeof(app_log));
    app_log_len = 0U;
    delays_10ms = delays_50ms = 0U;
    memset(retries_at_tx, 0xFF, sizeof(retries_at_tx));
    tx_seen = 0U;
    half_duplex = 0xFFU;
    stress_calls = stress_runs = 0U;
    stress_id = stress_retries = 0xFFU;
    simulate_servo_response(APP_SERVO_ID, STS_STATUS_OK, NULL, 0U, ack_servo);
    simulate_servo_response(APP_SERVO_ID, STS_STATUS_OK, NULL, 0U, ack_write);
    simulate_servo_response(APP_FIRST_ID, STS_STATUS_OK, NULL, 0U, ack_bad_1);
    ack_bad_1[APP_ACK_LEN - 1U] ^= 0xFFU;
    memcpy(ack_bad_servo, ack_servo, APP_ACK_LEN);
    ack_bad_servo[APP_ACK_LEN - 1U] ^= 0xFFU;
    simulate_servo_response(APP_SERVO_ID, STS_BIT_ERR_OVERHEAT, NULL, 0U, ack_hw_servo);
}

void tearDown(void) {
}

/* Runs AppMain; returns 1 if it reached the halt loop. */
static int run_app(const mock_step_t *steps, uint16_t step_count) {
    mock_script_reset(&app_script, steps, step_count);
    if (setjmp(app_halt) == 0) {
        AppMain();
        return 0;
    }
    return 1;
}

/* ">> Scanning..." line, then err=5 lines for IDs 1 to silent_ids, then tail. */
static void expect_log(uint32_t silent_ids, const char *tail) {
    char expected[APP_LOG_SIZE];
    size_t n = (size_t)snprintf(expected, sizeof(expected), ">> Scanning IDs 1-15...\n");
    for (uint32_t id = APP_FIRST_ID; id < APP_FIRST_ID + silent_ids; id++) {
        n += (size_t)snprintf(&expected[n], sizeof(expected) - n, "   ID %u: err=5\n", (unsigned)id);
    }
    (void)snprintf(&expected[n], sizeof(expected) - n, "%s", tail);
    TEST_ASSERT_EQUAL_STRING(expected, app_log);
}

static void expect_scan_pings(const uint8_t *ids, uint32_t count) {
    for (uint32_t i = 0U; i < count; i++) {
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(ids[i], app_script.tx_log[i][APP_IDX_ID], "pinged id");
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(APP_INST_PING, app_script.tx_log[i][APP_IDX_INSTRUCTION], "instruction");
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(APP_SCAN_RETRIES, retries_at_tx[i], "read retries during the scan");
    }
}

static void expect_started(uint32_t torque_tx) {
    TEST_ASSERT_EQUAL_UINT32(torque_tx + 1U, tx_seen);
    TEST_ASSERT_EQUAL_HEX8(APP_SERVO_ID, app_script.tx_log[torque_tx][APP_IDX_ID]);
    TEST_ASSERT_EQUAL_HEX8(APP_INST_WRITE, app_script.tx_log[torque_tx][APP_IDX_INSTRUCTION]);
    TEST_ASSERT_EQUAL_HEX8(STS_REG_TORQUE_ENABLE, app_script.tx_log[torque_tx][APP_IDX_ADDRESS]);
    TEST_ASSERT_EQUAL_HEX8(STS_TORQUE_ENABLE, app_script.tx_log[torque_tx][APP_IDX_VALUE]);
    TEST_ASSERT_EQUAL_UINT8(APP_RUN_RETRIES, retries_at_tx[torque_tx]);
    TEST_ASSERT_EQUAL_UINT32(1U, delays_50ms);
    TEST_ASSERT_EQUAL_UINT32(1U, stress_calls);
    TEST_ASSERT_EQUAL_UINT8(APP_SERVO_ID, stress_id);
    TEST_ASSERT_EQUAL_UINT8(APP_RUN_RETRIES, stress_retries);
    TEST_ASSERT_EQUAL_UINT32(APP_STRESS_RUNS, stress_runs);
    TEST_ASSERT_EQUAL_UINT8(1U, half_duplex);
}

/* --- Tests --- */

/* Servo at ID 3: IDs 1 and 2 get a ping and one retry each; one log line per ID. */
void test_App_Finds_Servo_With_One_Retry_Per_Ping(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_OK}, {NULL, 0U, STS_OK}, {NULL, 0U, STS_OK}, {NULL, 0U, STS_OK},
                                  {ack_servo, APP_ACK_LEN, STS_OK}, {ack_write, APP_ACK_LEN, STS_OK} };
    const uint8_t pings[] = {1U, 1U, 2U, 2U, APP_SERVO_ID};

    TEST_ASSERT_EQUAL_INT(0, run_app(steps, (uint16_t)(sizeof(steps) / sizeof(steps[0]))));
    expect_scan_pings(pings, sizeof(pings));
    expect_log(2U, "   ID 3: err=0\n>> Servo found at ID 3\n");
    TEST_ASSERT_EQUAL_UINT32(2U, delays_10ms);
    expect_started(sizeof(pings));
}

/* A corrupted first reply from the servo is recovered by the retry, logged once as found. */
void test_App_Retry_Recovers_Corrupted_Reply(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_OK}, {NULL, 0U, STS_OK}, {NULL, 0U, STS_OK}, {NULL, 0U, STS_OK},
                                  {ack_bad_servo, APP_ACK_LEN, STS_OK}, {ack_servo, APP_ACK_LEN, STS_OK},
                                  {ack_write, APP_ACK_LEN, STS_OK} };
    const uint8_t pings[] = {1U, 1U, 2U, 2U, APP_SERVO_ID, APP_SERVO_ID};

    TEST_ASSERT_EQUAL_INT(0, run_app(steps, (uint16_t)(sizeof(steps) / sizeof(steps[0]))));
    expect_scan_pings(pings, sizeof(pings));
    expect_log(2U, "   ID 3: err=0\n>> Servo found at ID 3\n");
    expect_started(sizeof(pings));
}

/* A servo reporting a fault (overheat) still answered: logged as found, without the invalid-reply suffix or wait. */
void test_App_Hardware_Status_Counts_As_Found(void) {
    const mock_step_t steps[] = { {NULL, 0U, STS_OK}, {NULL, 0U, STS_OK}, {NULL, 0U, STS_OK}, {NULL, 0U, STS_OK},
                                  {ack_hw_servo, APP_ACK_LEN, STS_OK}, {ack_write, APP_ACK_LEN, STS_OK} };
    const uint8_t pings[] = {1U, 1U, 2U, 2U, APP_SERVO_ID};

    TEST_ASSERT_EQUAL_INT(0, run_app(steps, (uint16_t)(sizeof(steps) / sizeof(steps[0]))));
    expect_scan_pings(pings, sizeof(pings));
    expect_log(2U, "   ID 3: err=10\n>> Servo found at ID 3\n");
    TEST_ASSERT_EQUAL_UINT32(2U, delays_10ms);
    expect_started(sizeof(pings));
}

/* Nothing answers: every ID is pinged twice, then the no-reply halt message. */
void test_App_Halts_With_No_Reply_Message(void) {
    uint8_t pings[2U * APP_LAST_ID];
    for (uint32_t i = 0U; i < sizeof(pings); i++) {
        pings[i] = (uint8_t)(APP_FIRST_ID + i / 2U);
    }

    TEST_ASSERT_EQUAL_INT(1, run_app(NULL, 0U));
    TEST_ASSERT_EQUAL_UINT32(sizeof(pings), tx_seen);
    expect_scan_pings(pings, sizeof(pings));
    expect_log(APP_LAST_ID, ">> No reply on IDs 1-15. Check wiring and power.\n");
    TEST_ASSERT_EQUAL_UINT32(APP_LAST_ID, delays_10ms);
    TEST_ASSERT_EQUAL_UINT32(0U, stress_calls);
}

/* Only corrupted replies: the line says so, and the halt message names the last error. */
void test_App_Halts_With_No_Valid_Reply_Message(void) {
    const mock_step_t steps[] = { {ack_bad_1, APP_ACK_LEN, STS_OK}, {ack_bad_1, APP_ACK_LEN, STS_OK} };
    char expected[APP_LOG_SIZE];
    size_t n = (size_t)snprintf(expected, sizeof(expected), ">> Scanning IDs 1-15...\n   ID 1: err=8, not a valid reply\n");
    for (uint32_t id = APP_FIRST_ID + 1U; id <= APP_LAST_ID; id++) {
        n += (size_t)snprintf(&expected[n], sizeof(expected) - n, "   ID %u: err=5\n", (unsigned)id);
    }
    (void)snprintf(&expected[n], sizeof(expected) - n, ">> No valid reply on IDs 1-15 (last err=8). See the results above.\n");

    TEST_ASSERT_EQUAL_INT(1, run_app(steps, (uint16_t)(sizeof(steps) / sizeof(steps[0]))));
    TEST_ASSERT_EQUAL_UINT32(2U * APP_LAST_ID, tx_seen);
    TEST_ASSERT_EQUAL_STRING(expected, app_log);
    TEST_ASSERT_EQUAL_UINT32(APP_LAST_ID, delays_10ms);
    TEST_ASSERT_EQUAL_UINT32(0U, stress_calls);
}

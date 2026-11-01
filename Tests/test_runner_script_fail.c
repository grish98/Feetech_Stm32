/**
 ******************************************************************************
 * @file           : test_runner_script_fail.c
 * @brief          : Expected-failure check for the scripted transport
 * @author         : Grisham Balloo
 ******************************************************************************
 * @details
 * Each test must fail through the scripted transport's default failure path.
 * CTest passes this executable only when both tests fail, so a broken default
 * handler or a broken NULL-hook fallback shows up as a missing failure.
 ******************************************************************************
 */

#include "unity.h"
#include "test_sts_utils.h"
#include <string.h>

void test_Default_Handler_Fails_The_Test(void);
void test_Null_Hook_Falls_Back_To_Default_Handler(void);

static mock_script_t script;
static sts_bus_t bus;
static const uint8_t packet[] = { 0xAAU };

void setUp(void) {
    memset(&bus, 0, sizeof(bus));
}

void tearDown(void) {
}

/* Reset installs mock_script_default_fail; the second transmit is past the cap. */
void test_Default_Handler_Fails_The_Test(void) {
    mock_script_reset(&script, NULL, 0U);
    script.tx_cap = 1U;
    (void)STS_Bus_Init(&bus, &script, mock_script_tx, mock_script_rx);

    (void)mock_script_tx(&bus, packet, sizeof(packet));
    (void)mock_script_tx(&bus, packet, sizeof(packet));
}

/* A script that was never reset has on_fail NULL and must still fail the test. */
void test_Null_Hook_Falls_Back_To_Default_Handler(void) {
    memset(&script, 0, sizeof(script));
    script.tx_cap = 1U;
    (void)STS_Bus_Init(&bus, &script, mock_script_tx, mock_script_rx);

    (void)mock_script_tx(&bus, packet, sizeof(packet));
    (void)mock_script_tx(&bus, packet, sizeof(packet));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_Default_Handler_Fails_The_Test);
    RUN_TEST(test_Null_Hook_Falls_Back_To_Default_Handler);
    return UNITY_END();
}

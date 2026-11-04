/**
 * @file  test_runner_app.c
 * @brief Runs the host tests for the example application's start-up.
 */
#include "unity.h"
#include "test_app_main.h"

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_App_Finds_Servo_With_One_Retry_Per_Ping);
    RUN_TEST(test_App_Retry_Recovers_Corrupted_Reply);
    RUN_TEST(test_App_Hardware_Status_Counts_As_Found);
    RUN_TEST(test_App_Halts_With_No_Reply_Message);
    RUN_TEST(test_App_Halts_With_No_Valid_Reply_Message);

    return UNITY_END();
}

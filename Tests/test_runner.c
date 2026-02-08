#include "unity.h"

extern void test_sanity_check(void);

void setUp(void) {}
void tearDown(void) {}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_sanity_check);
    return UNITY_END();
}
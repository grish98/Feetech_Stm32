/**
 ******************************************************************************
 * @file           : test_runner_ring_buffer.c
 * @brief          : Ring Buffer Module Unit Test Runner
 * @author         : Grisham Balloo
 * @date           : 2026-10-29
 ******************************************************************************
 * @details
 * This file serves as the main entry point for the ring buffer unit tests.
 * It utilises the Unity Test Framework to orchestrate the execution of test
 * cases defined in 'test_sts_ring_buffer.c'.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#include "unity.h"
#include "test_sts_ring_buffer.h"
#include <stdio.h>

int main(void) {
    UNITY_BEGIN();

    printf("========== Ring Buffer Unit Tests ==========\n");

    printf("\n--- Init and Argument Checks ---\n");
    RUN_TEST(test_Init_StartsAtPositionZero);
    RUN_TEST(test_Init_NullPointers);
    RUN_TEST(test_Init_RejectsCapacityNotPowerOfTwo);
    RUN_TEST(test_Init_FailedReinitLeavesRingUnchanged);
    RUN_TEST(test_Init_AcceptsPowersOfTwo);
    RUN_TEST(test_Calls_OnZeroInitialisedRing_ReportNullPtr);
    RUN_TEST(test_Calls_NullArguments);

    printf("\n--- Unread Count and Overrun Boundary ---\n");
    RUN_TEST(test_Unread_EmptyRingIsZero);
    RUN_TEST(test_Unread_CountsProducedBytes);
    RUN_TEST(test_Unread_ProducerBehindCursorIsReported);
    RUN_TEST(test_Unread_HalfRangeBoundary);
    RUN_TEST(test_Boundary_CapacityMinusOneIsIntact);
    RUN_TEST(test_Boundary_ExactlyCapacityIsOverrun);

    printf("\n--- Read: the Check Before the Copy ---\n");
    RUN_TEST(test_Read_CopiesOldestBytesAndAdvances);
    RUN_TEST(test_Read_ShortWhenBytesNotYetArrived);
    RUN_TEST(test_Read_ZeroLengthIsOk);
    RUN_TEST(test_Read_ZeroLengthNullDstIsNullPtr);
    RUN_TEST(test_Read_ZeroLengthIgnoresRingState);
    RUN_TEST(test_Read_LengthAtOrAboveCapacityIsInvalid);
    RUN_TEST(test_Read_MaxLengthCopyFromEverySlotStaysInsideDst);
    RUN_TEST(test_Read_EndingExactlyAtStorageEnd);
    RUN_TEST(test_Read_ProducerBehindCursorIsReported);
    RUN_TEST(test_Read_ExactlyCapacityIsOverrun);
    RUN_TEST(test_Read_OverrunReportedBeforeCopy);

    printf("\n--- Read: the Check After the Copy ---\n");
    RUN_TEST(test_Read_DetectsOverwriteBetweenSamples);
    RUN_TEST(test_Read_OverwriteDuringCopyIsCaught);
    RUN_TEST(test_Read_CopyFinishesBeforeSecondSample);
    RUN_TEST(test_Read_ExactlyCapacityAtSecondSampleIsOverrun);
    RUN_TEST(test_Read_SamplerOneByteBehindStillCaught);
    RUN_TEST(test_Read_ProgressBetweenSamplesIsAccepted);
    RUN_TEST(test_Read_PositionWentBackwardsIsProducerBehind);
    RUN_TEST(test_Read_RestartBetweenSamplesIsProducerBehind);
    RUN_TEST(test_Read_BackwardsWithinRingIsNotDetected);
    RUN_TEST(test_Read_SampleFailureBeforeCopy);
    RUN_TEST(test_Read_SampleFailureAfterCopy);

    printf("\n--- Skip ---\n");
    RUN_TEST(test_Skip_DropsOldestBytes);
    RUN_TEST(test_Skip_ShortUntilAllBytesArrive);
    RUN_TEST(test_Skip_LappedCursorIsOverrun);
    RUN_TEST(test_Skip_ProducerBehindCursorIsReported);
    RUN_TEST(test_Skip_LengthAtOrAboveCapacityIsInvalid);
    RUN_TEST(test_Skip_ZeroLengthIgnoresRingState);
    RUN_TEST(test_Skip_SampleFailure);

    printf("\n--- Long Reads, Rollover, Discard ---\n");
    RUN_TEST(test_LongRead_LongerThanRing_NoFalseOverrun);
    RUN_TEST(test_Rollover_ReadAcrossUint32Wrap);
    RUN_TEST(test_Rollover_ErrorBoundariesAcrossUint32Wrap);
    RUN_TEST(test_Horizon_ResultsRepeatEvery2To32);
    RUN_TEST(test_Horizon_LappedReadIsAccepted);
    RUN_TEST(test_Discard_DropsUnreadBytes);
    RUN_TEST(test_Discard_RecoversFromOverrun);
    RUN_TEST(test_Discard_RequiredAfterProducerRestart);

    printf("\n--- Capacity Range ---\n");
    RUN_TEST(test_Capacity_TwoReadsOneByteAtATime);
    RUN_TEST(test_Capacity_LargestMaxLengthReadAcrossEnd);
    RUN_TEST(test_Capacity_SlotsAbove255);

    printf("\n--- Reference Model ---\n");
    RUN_TEST(test_Model_RandomOperationsMatchReference);

    return UNITY_END();
}

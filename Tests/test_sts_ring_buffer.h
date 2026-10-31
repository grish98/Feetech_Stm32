/**
 ******************************************************************************
 * @file           : test_sts_ring_buffer.h
 * @brief          : Prototypes for the Ring Buffer Verification Suite
 * @author         : Grisham Balloo
 * @date           : 2026-10-29
 ******************************************************************************
 * @note All functions conform to the Unity Test Framework signature: void func(void).
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */

#pragma once

/* =========================================================================
   INIT AND ARGUMENT CHECKS
   ========================================================================= */
extern void test_Init_StartsAtPositionZero(void);
extern void test_Init_NullPointers(void);
extern void test_Init_RejectsCapacityNotPowerOfTwo(void);
extern void test_Init_FailedReinitLeavesRingUnchanged(void);
extern void test_Init_AcceptsPowersOfTwo(void);
extern void test_Calls_OnZeroInitialisedRing_ReportNullPtr(void);
extern void test_Calls_NullArguments(void);

/* =========================================================================
   UNREAD COUNT AND THE OVERRUN BOUNDARY
   ========================================================================= */
extern void test_Unread_EmptyRingIsZero(void);
extern void test_Unread_CountsProducedBytes(void);
extern void test_Unread_ProducerBehindCursorIsReported(void);
extern void test_Unread_HalfRangeBoundary(void);
extern void test_Boundary_CapacityMinusOneIsIntact(void);
extern void test_Boundary_ExactlyCapacityIsOverrun(void);

/* =========================================================================
   READ: THE CHECK BEFORE THE COPY
   ========================================================================= */
extern void test_Read_CopiesOldestBytesAndAdvances(void);
extern void test_Read_ShortWhenBytesNotYetArrived(void);
extern void test_Read_ZeroLengthIsOk(void);
extern void test_Read_ZeroLengthNullDstIsNullPtr(void);
extern void test_Read_ZeroLengthIgnoresRingState(void);
extern void test_Read_LengthAtOrAboveCapacityIsInvalid(void);
extern void test_Read_MaxLengthCopyFromEverySlotStaysInsideDst(void);
extern void test_Read_EndingExactlyAtStorageEnd(void);
extern void test_Read_ProducerBehindCursorIsReported(void);
extern void test_Read_ExactlyCapacityIsOverrun(void);
extern void test_Read_OverrunReportedBeforeCopy(void);

/* =========================================================================
   READ: THE CHECK AFTER THE COPY
   ========================================================================= */
extern void test_Read_DetectsOverwriteBetweenSamples(void);
extern void test_Read_OverwriteDuringCopyIsCaught(void);
extern void test_Read_CopyFinishesBeforeSecondSample(void);
extern void test_Read_ExactlyCapacityAtSecondSampleIsOverrun(void);
extern void test_Read_SamplerOneByteBehindStillCaught(void);
extern void test_Read_ProgressBetweenSamplesIsAccepted(void);
extern void test_Read_PositionWentBackwardsIsProducerBehind(void);
extern void test_Read_RestartBetweenSamplesIsProducerBehind(void);
extern void test_Read_BackwardsWithinRingIsNotDetected(void);
extern void test_Read_SampleFailureBeforeCopy(void);
extern void test_Read_SampleFailureAfterCopy(void);

/* =========================================================================
   SKIP
   ========================================================================= */
extern void test_Skip_DropsOldestBytes(void);
extern void test_Skip_ShortUntilAllBytesArrive(void);
extern void test_Skip_LappedCursorIsOverrun(void);
extern void test_Skip_ProducerBehindCursorIsReported(void);
extern void test_Skip_LengthAtOrAboveCapacityIsInvalid(void);
extern void test_Skip_ZeroLengthIgnoresRingState(void);
extern void test_Skip_SampleFailure(void);

/* =========================================================================
   LONG READS, ROLLOVER, DISCARD
   ========================================================================= */
extern void test_LongRead_LongerThanRing_NoFalseOverrun(void);
extern void test_Rollover_ReadAcrossUint32Wrap(void);
extern void test_Rollover_ErrorBoundariesAcrossUint32Wrap(void);
extern void test_Horizon_ResultsRepeatEvery2To32(void);
extern void test_Horizon_LappedReadIsAccepted(void);
extern void test_Discard_DropsUnreadBytes(void);
extern void test_Discard_RecoversFromOverrun(void);
extern void test_Discard_RequiredAfterProducerRestart(void);

/* =========================================================================
   CAPACITY RANGE
   ========================================================================= */
extern void test_Capacity_TwoReadsOneByteAtATime(void);
extern void test_Capacity_LargestMaxLengthReadAcrossEnd(void);
extern void test_Capacity_SlotsAbove255(void);

/* =========================================================================
   REFERENCE MODEL
   ========================================================================= */
extern void test_Model_RandomOperationsMatchReference(void);

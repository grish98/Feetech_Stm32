/**
 ******************************************************************************
 * @file           : test_sts_ring_buffer.c
 * @brief          : Unit Tests for the Ring Buffer Module
 * @author         : Grisham Balloo
 * @date           : 2026-10-29
 ******************************************************************************
 * @details
 * A scripted producer fills an 8-byte ring with position-tagged bytes and
 * injects writes or failures around the read's two samples. Focused tests
 * cover API boundaries, overwrite timing, rollover, recovery and capacity
 * limits. A fixed-seed model checks 50,000 operations at each of capacities
 * 2, 8 and 64 against an independent byte history.
 *
 * @see Lib/STS_Servo/Inc/sts_ring_buffer.h for the target API documentation.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#include "unity.h"
#include "sts_ring_buffer.h"
#include "test_sts_ring_buffer.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TEST_CAPACITY   8U
#define FILL_BYTE       0x55U
#define UNREAD_SENTINEL 0xFFFFU /* never a valid count: a ring reports at most 32767 */
#define POSITION_RANGE  UINT64_C(0x100000000) /* 2^32: positions repeat after this many bytes */

static uint8_t storage[TEST_CAPACITY];
static sts_ring_buffer_t rb;
static uint32_t produced; /* the fake producer's absolute position */

static void produce(uint16_t n) {
    for (uint16_t i = 0U; i < n; i++) {
        storage[produced % TEST_CAPACITY] = (uint8_t)produced;
        produced++;
    }
}

/*
 * The fake producer behind the sampler. Sample numbers count from 1, from the
 * last reset; 0 in a *_at field means never.
 */
struct sts_ring_buffer_producer {
    const uint32_t *position;   /* reported by each sample: &produced unless a test points it elsewhere */
    void (*write)(uint16_t n);  /* writes n more bytes: produce() unless a test replaces it */
    uint32_t samples;           /* samples taken so far */
    uint32_t write_at;          /* the sample that first writes `burst` bytes, so its position counts them */
    uint16_t burst;
    uint32_t write_after_at;    /* the sample that writes `burst_after` bytes once its position is taken */
    uint16_t burst_after;
    uint32_t override_at;       /* the sample that reports `override_position` instead */
    uint32_t override_position;
    uint32_t fail_at;           /* the sample that fails */
};

static sts_ring_buffer_producer_t fake;

static sts_ring_buffer_sample_status_t fake_sample(sts_ring_buffer_producer_t *producer, uint32_t *position) {
    producer->samples++;
    if (producer->samples == producer->write_at) {
        producer->write(producer->burst);
    }
    if (producer->samples == producer->fail_at) {
        return STS_RING_BUFFER_SAMPLE_FAILED;
    }
    *position = (producer->samples == producer->override_at) ? producer->override_position
                                                              : *producer->position;
    if (producer->samples == producer->write_after_at) {
        producer->write(producer->burst_after); /* the DMA keeps going after the sample */
    }
    return STS_RING_BUFFER_SAMPLE_OK;
}

static void reset_fake(void) {
    fake = (sts_ring_buffer_producer_t){.position = &produced, .write = produce}; /* every *_at field 0: never */
}

static sts_ring_buffer_status_t read_ring(uint8_t *dst, uint16_t len) {
    return sts_ring_buffer_read(&rb, fake_sample, &fake, dst, len);
}

static sts_ring_buffer_status_t skip_ring(uint16_t len) {
    return sts_ring_buffer_skip(&rb, fake_sample, &fake, len);
}

/* The position a producer reports after running `lead` bytes past the cursor;
 * a uint32_t position keeps only the lead modulo 2^32. */
static uint32_t position_after_lead(uint64_t lead) {
    return (uint32_t)((uint64_t)rb.cursor + lead);
}

/* Puts both the producer and the cursor at an arbitrary position. */
static void start_at(uint32_t position) {
    produced = position;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_discard(&rb, position));
}

void setUp(void) {
    memset(storage, 0xAA, sizeof(storage));
    produced = 0U;
    reset_fake();
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_init(&rb, storage, TEST_CAPACITY));
}

static void model_trace_report(void);

void tearDown(void) {
    model_trace_report(); /* prints only after a failed model run */
}

/* =========================================================================
   INIT AND ARGUMENT CHECKS
   ========================================================================= */

/* Re-initialises a ring whose cursor has moved, so a stale cursor would show. */
void test_Init_StartsAtPositionZero(void) {
    start_at(37U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_init(&rb, storage, TEST_CAPACITY));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
    TEST_ASSERT_EQUAL_UINT16(TEST_CAPACITY, rb.capacity);
    TEST_ASSERT_EQUAL_PTR(storage, rb.buffer);
}

void test_Init_NullPointers(void) {
    sts_ring_buffer_t other = {NULL, 0U, 0U};
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_init(NULL, storage, TEST_CAPACITY));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_init(&other, NULL, TEST_CAPACITY));
}

void test_Init_RejectsCapacityNotPowerOfTwo(void) {
    sts_ring_buffer_t other = {NULL, 0U, 0U};
    const uint16_t bad[] = {0U, 1U, 3U, 6U, 255U, 384U, 65535U};
    for (size_t i = 0U; i < sizeof(bad) / sizeof(bad[0]); i++) {
        TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_INVALID_ARG, sts_ring_buffer_init(&other, storage, bad[i]));
    }
    TEST_ASSERT_NULL(other.buffer); /* left unchanged */
}

/* A rejected init on a ring in use changes nothing: a capacity written before
 * the check would let later reads run past the real storage. */
void test_Init_FailedReinitLeavesRingUnchanged(void) {
    static uint8_t other_storage[16];
    start_at(37U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_INVALID_ARG, sts_ring_buffer_init(&rb, other_storage, 6U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_INVALID_ARG, sts_ring_buffer_init(&rb, other_storage, 0U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_init(&rb, NULL, 16U));
    TEST_ASSERT_EQUAL_PTR(storage, rb.buffer);
    TEST_ASSERT_EQUAL_UINT16(TEST_CAPACITY, rb.capacity);
    TEST_ASSERT_EQUAL_UINT32(37U, rb.cursor);
}

/* Every accepted capacity is stored, with the buffer and a cursor of 0. The
 * ring starts dirty each time, so a field init forgot to write would show. */
void test_Init_AcceptsPowersOfTwo(void) {
    const uint16_t good[] = {2U, 256U, 32768U};
    for (size_t i = 0U; i < sizeof(good) / sizeof(good[0]); i++) {
        sts_ring_buffer_t other = {NULL, 1U, 37U};
        /* init never touches the storage, so the small array is enough here */
        TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_init(&other, storage, good[i]));
        TEST_ASSERT_EQUAL_PTR(storage, other.buffer);
        TEST_ASSERT_EQUAL_UINT16(good[i], other.capacity);
        TEST_ASSERT_EQUAL_UINT32(0U, other.cursor);
    }
}

/* No call samples the producer before it has checked its pointers. */
void test_Calls_OnZeroInitialisedRing_ReportNullPtr(void) {
    sts_ring_buffer_t zeroed = {NULL, 0U, 0U};
    uint8_t dst[1];
    uint16_t unread = UNREAD_SENTINEL;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_unread(&zeroed, 0U, &unread));
    TEST_ASSERT_EQUAL_UINT16(UNREAD_SENTINEL, unread); /* not written on NULL_PTR */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_read(&zeroed, fake_sample, &fake, dst, 1U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_skip(&zeroed, fake_sample, &fake, 1U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_discard(&zeroed, 0U));
    TEST_ASSERT_EQUAL_UINT32(0U, fake.samples);
}

void test_Calls_NullArguments(void) {
    uint8_t dst[1];
    uint16_t unread = UNREAD_SENTINEL;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_unread(NULL, 0U, &unread));
    TEST_ASSERT_EQUAL_UINT16(UNREAD_SENTINEL, unread); /* not written on NULL_PTR */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_unread(&rb, 0U, NULL));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_read(NULL, fake_sample, &fake, dst, 1U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_read(&rb, NULL, &fake, dst, 1U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_read(&rb, fake_sample, NULL, dst, 1U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_read(&rb, fake_sample, &fake, NULL, 1U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_skip(NULL, fake_sample, &fake, 1U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_skip(&rb, NULL, &fake, 1U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_skip(&rb, fake_sample, NULL, 1U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_discard(NULL, 0U));
    TEST_ASSERT_EQUAL_UINT32(0U, fake.samples);
}

/* =========================================================================
   UNREAD COUNT AND THE OVERRUN BOUNDARY
   ========================================================================= */

void test_Unread_EmptyRingIsZero(void) {
    uint16_t unread = UNREAD_SENTINEL;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_unread(&rb, produced, &unread));
    TEST_ASSERT_EQUAL_UINT16(0U, unread);
}

void test_Unread_CountsProducedBytes(void) {
    uint16_t unread = 0U;
    produce(5U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_unread(&rb, produced, &unread));
    TEST_ASSERT_EQUAL_UINT16(5U, unread);
}

void test_Unread_ProducerBehindCursorIsReported(void) {
    uint16_t unread = UNREAD_SENTINEL;
    start_at(10U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_PRODUCER_BEHIND, sts_ring_buffer_unread(&rb, 9U, &unread));
    TEST_ASSERT_EQUAL_UINT16(0U, unread); /* zeroed on PRODUCER_BEHIND */
}

/* Distances below 2^31 read as laps; from 2^31 up they read as negative. */
void test_Unread_HalfRangeBoundary(void) {
    uint16_t unread = 0U;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, sts_ring_buffer_unread(&rb, 0x7FFFFFFFU, &unread));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_PRODUCER_BEHIND, sts_ring_buffer_unread(&rb, 0x80000000U, &unread));
}

/* capacity - 1 is the most the ring reports as intact. */
void test_Boundary_CapacityMinusOneIsIntact(void) {
    uint16_t unread = 0U;
    produce(TEST_CAPACITY - 1U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_unread(&rb, produced, &unread));
    TEST_ASSERT_EQUAL_UINT16(TEST_CAPACITY - 1U, unread);
}

/* Exactly capacity: nothing is overwritten yet, but it is refused by design. */
void test_Boundary_ExactlyCapacityIsOverrun(void) {
    uint16_t unread = UNREAD_SENTINEL;
    produce(TEST_CAPACITY);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, sts_ring_buffer_unread(&rb, produced, &unread));
    TEST_ASSERT_EQUAL_UINT16(0U, unread);
}

/* =========================================================================
   READ: THE CHECK BEFORE THE COPY
   ========================================================================= */

/* A read copies the oldest bytes and moves the cursor past them, sampling twice. */
void test_Read_CopiesOldestBytesAndAdvances(void) {
    const uint8_t first[] = {0x00U, 0x01U, 0x02U};
    const uint8_t rest[] = {0x03U, 0x04U};
    uint8_t dst[3];
    produce(5U);

    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 3U));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(first, dst, 3U);
    TEST_ASSERT_EQUAL_UINT32(3U, rb.cursor);
    TEST_ASSERT_EQUAL_UINT32(2U, fake.samples);

    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 2U));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(rest, dst, 2U);
    TEST_ASSERT_EQUAL_UINT32(5U, rb.cursor);
}

/* Refused at the first sample: nothing is copied and no second sample is taken. */
void test_Read_ShortWhenBytesNotYetArrived(void) {
    uint8_t dst[3];
    memset(dst, FILL_BYTE, sizeof(dst));
    produce(2U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_SHORT, read_ring(dst, 3U));
    TEST_ASSERT_EACH_EQUAL_HEX8(FILL_BYTE, dst, sizeof(dst));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
    TEST_ASSERT_EQUAL_UINT32(1U, fake.samples);
}

void test_Read_ZeroLengthIsOk(void) {
    uint8_t dst[1] = {FILL_BYTE};
    produce(3U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 0U));
    TEST_ASSERT_EQUAL_HEX8(FILL_BYTE, dst[0]);
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
    TEST_ASSERT_EQUAL_UINT32(0U, fake.samples);
}

void test_Read_ZeroLengthNullDstIsNullPtr(void) {
    produce(3U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_NULL_PTR, sts_ring_buffer_read(&rb, fake_sample, &fake, NULL, 0U));
}

/* Zero length succeeds without sampling: an overrun or a producer behind the
 * cursor is reported by the next non-zero call instead. */
void test_Read_ZeroLengthIgnoresRingState(void) {
    uint8_t dst[1] = {FILL_BYTE};
    produce(TEST_CAPACITY + 1U); /* overrun */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 0U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, read_ring(dst, 1U));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);

    start_at(10U);
    produced = 9U; /* producer behind */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 0U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_PRODUCER_BEHIND, read_ring(dst, 1U));
    TEST_ASSERT_EQUAL_HEX8(FILL_BYTE, dst[0]);
    TEST_ASSERT_EQUAL_UINT32(10U, rb.cursor);
}

/* Rejected before any sample is taken, so the destination and cursor stay as they were. */
void test_Read_LengthAtOrAboveCapacityIsInvalid(void) {
    const uint16_t bad[] = {TEST_CAPACITY, TEST_CAPACITY + 1U, UINT16_MAX};
    uint8_t dst[1] = {FILL_BYTE};
    produce(TEST_CAPACITY - 1U);
    for (size_t i = 0U; i < sizeof(bad) / sizeof(bad[0]); i++) {
        TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_INVALID_ARG, read_ring(dst, bad[i]));
    }
    TEST_ASSERT_EQUAL_HEX8(FILL_BYTE, dst[0]);
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
    TEST_ASSERT_EQUAL_UINT32(0U, fake.samples);
}

/* Guard bytes either side of the destination: a maximum-length read from every
 * starting slot (so every split between the two runs) writes exactly len bytes. */
void test_Read_MaxLengthCopyFromEverySlotStaysInsideDst(void) {
    uint8_t guarded[TEST_CAPACITY + 1U]; /* guard, TEST_CAPACITY - 1 bytes, guard */
    uint8_t *const dst = &guarded[1];
    const uint16_t len = TEST_CAPACITY - 1U;
    for (uint32_t slot = 0U; slot < TEST_CAPACITY; slot++) {
        start_at(slot);
        produce(len);
        memset(guarded, FILL_BYTE, sizeof(guarded));
        TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, len));
        TEST_ASSERT_EQUAL_HEX8(FILL_BYTE, guarded[0]);
        TEST_ASSERT_EQUAL_HEX8(FILL_BYTE, guarded[TEST_CAPACITY]);
        for (uint16_t i = 0U; i < len; i++) {
            TEST_ASSERT_EQUAL_HEX8((uint8_t)(slot + i), dst[i]);
        }
        TEST_ASSERT_EQUAL_UINT32(slot + len, rb.cursor);
    }
}

void test_Read_EndingExactlyAtStorageEnd(void) {
    const uint8_t expected[] = {0x05U, 0x06U, 0x07U};
    uint8_t dst[4];
    memset(dst, FILL_BYTE, sizeof(dst));
    start_at(5U);
    produce(3U); /* slots 5, 6, 7: the second run is empty */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 3U));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, dst, 3U);
    TEST_ASSERT_EQUAL_HEX8(FILL_BYTE, dst[3]); /* nothing written past len */
}

/* What PRODUCER_BEHIND does catch: a restarted producer (back near 0) behind
 * a cursor that had moved on. Nothing is copied. */
void test_Read_ProducerBehindCursorIsReported(void) {
    uint8_t dst[3];
    memset(dst, FILL_BYTE, sizeof(dst));
    start_at(37U);
    produced = 0U; /* the producer restarts */
    produce(3U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_PRODUCER_BEHIND, read_ring(dst, 3U));
    TEST_ASSERT_EACH_EQUAL_HEX8(FILL_BYTE, dst, sizeof(dst));
    TEST_ASSERT_EQUAL_UINT32(37U, rb.cursor);
}

/* Exactly capacity unread at the first sample: the one-byte margin applies
 * before the copy too. */
void test_Read_ExactlyCapacityIsOverrun(void) {
    uint8_t dst[1] = {FILL_BYTE};
    produce(TEST_CAPACITY);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, read_ring(dst, 1U));
    TEST_ASSERT_EQUAL_HEX8(FILL_BYTE, dst[0]);
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
}

void test_Read_OverrunReportedBeforeCopy(void) {
    uint8_t dst[1] = {FILL_BYTE};
    produce(TEST_CAPACITY + 1U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, read_ring(dst, 1U));
    TEST_ASSERT_EQUAL_HEX8(FILL_BYTE, dst[0]);
    TEST_ASSERT_EQUAL_UINT32(1U, fake.samples);
}

/* =========================================================================
   READ: THE CHECK AFTER THE COPY
   ========================================================================= */

/* The check before the copy passes; the fake DMA then writes far enough to
 * overwrite the oldest copied byte before the second sample. These bytes land
 * after the copy, so only the position shows the overwrite;
 * test_Read_OverwriteDuringCopyIsCaught has the copy read it as well. */
void test_Read_DetectsOverwriteBetweenSamples(void) {
    uint8_t dst[TEST_CAPACITY - 1U];
    produce(TEST_CAPACITY - 1U);
    fake.write_at = 2U;
    fake.burst = 2U; /* position 8 lands in slot 0, over position 0 */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, read_ring(dst, TEST_CAPACITY - 1U));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
}

/* The DMA keeps writing once the first sample is taken, so the copy itself
 * reads the overwritten byte: position 8 is in slot 0 before the copy gets
 * there. Only the second sample can show it. */
void test_Read_OverwriteDuringCopyIsCaught(void) {
    uint8_t dst[TEST_CAPACITY - 1U];
    memset(dst, FILL_BYTE, sizeof(dst));
    produce(TEST_CAPACITY - 1U);
    fake.write_after_at = 1U;
    fake.burst_after = 2U; /* positions 7 and 8: 8 lands in slot 0, over position 0 */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, read_ring(dst, TEST_CAPACITY - 1U));
    TEST_ASSERT_EQUAL_HEX8(0x08U, dst[0]); /* the copy did read position 8 */
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
}

/* The second sample is the last look at the producer, so the copy must be
 * finished by then: bytes written after it are not counted, and a copy that
 * ran later would return position 8 as position 0 with an OK. */
void test_Read_CopyFinishesBeforeSecondSample(void) {
    const uint8_t expected[] = {0x00U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U};
    uint8_t dst[TEST_CAPACITY - 1U];
    produce(TEST_CAPACITY - 1U);
    fake.write_after_at = 2U;
    fake.burst_after = 2U; /* positions 7 and 8, just after the second sample */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, TEST_CAPACITY - 1U));
    TEST_ASSERT_EQUAL_HEX8(0x08U, storage[0]); /* the overwrite did land */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, dst, TEST_CAPACITY - 1U);
    TEST_ASSERT_EQUAL_UINT32(TEST_CAPACITY - 1U, rb.cursor);
}

/* One byte between the samples: nothing overwritten yet, still refused (the margin). */
void test_Read_ExactlyCapacityAtSecondSampleIsOverrun(void) {
    uint8_t dst[TEST_CAPACITY - 1U];
    produce(TEST_CAPACITY - 1U);
    fake.write_at = 2U;
    fake.burst = 1U;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, read_ring(dst, TEST_CAPACITY - 1U));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
}

/* A second sample one byte behind storage counts exactly capacity, so the margin refuses the read. */
void test_Read_SamplerOneByteBehindStillCaught(void) {
    uint8_t dst[TEST_CAPACITY - 1U];
    produce(TEST_CAPACITY - 1U);
    fake.write_after_at = 1U;
    fake.burst_after = 2U;                  /* positions 7 and 8: 8 lands in slot 0 during the copy */
    fake.override_at = 2U;
    fake.override_position = TEST_CAPACITY; /* counts position 7 but not yet 8 */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, read_ring(dst, TEST_CAPACITY - 1U));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
}

/* The normal case with traffic still arriving: bytes written between the
 * samples but short of an overwrite do not fail the read, and stay unread. */
void test_Read_ProgressBetweenSamplesIsAccepted(void) {
    const uint8_t expected[] = {0x00U, 0x01U, 0x02U};
    const uint8_t rest[] = {0x03U, 0x04U, 0x05U, 0x06U};
    uint8_t dst[4];
    uint16_t unread = 0U;
    produce(3U);
    fake.write_at = 2U;
    fake.burst = 4U; /* position 7: the oldest copied byte is still intact */

    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 3U));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, dst, 3U);
    TEST_ASSERT_EQUAL_UINT32(3U, rb.cursor);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_unread(&rb, produced, &unread));
    TEST_ASSERT_EQUAL_UINT16(4U, unread);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 4U));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(rest, dst, 4U);
}

/* The second sample is lower than the first by enough to leave fewer than
 * len bytes: the position went backwards during the copy. */
void test_Read_PositionWentBackwardsIsProducerBehind(void) {
    uint8_t dst[3];
    produce(5U);
    fake.override_at = 2U;
    fake.override_position = 2U;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_PRODUCER_BEHIND, read_ring(dst, 3U));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
}

/* A producer restart between the samples, back behind the cursor. */
void test_Read_RestartBetweenSamplesIsProducerBehind(void) {
    uint8_t dst[3];
    start_at(37U);
    produce(3U);
    fake.override_at = 2U;
    fake.override_position = 1U; /* restarted at 0, one byte in */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_PRODUCER_BEHIND, read_ring(dst, 3U));
    TEST_ASSERT_EQUAL_UINT32(37U, rb.cursor);
}

/* Documents a limit, not a feature: PRODUCER_BEHIND is best-effort. A second
 * sample that went backwards but still lands between cursor + len and
 * cursor + capacity is accepted. Detecting a producer restart is the caller's
 * job (discard after every restart, before the next read). */
void test_Read_BackwardsWithinRingIsNotDetected(void) {
    uint8_t dst[3];
    produce(7U);
    fake.override_at = 2U;
    fake.override_position = 4U; /* went back from 7 to 4 */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 3U));
    TEST_ASSERT_EQUAL_UINT32(3U, rb.cursor);
}

/* The sampler fails at the first sample: nothing is copied and the cursor stays. */
void test_Read_SampleFailureBeforeCopy(void) {
    uint8_t dst[3];
    memset(dst, FILL_BYTE, sizeof(dst));
    produce(5U);
    fake.fail_at = 1U;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_SAMPLE, read_ring(dst, 3U));
    TEST_ASSERT_EACH_EQUAL_HEX8(FILL_BYTE, dst, sizeof(dst));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
    TEST_ASSERT_EQUAL_UINT32(1U, fake.samples);
}

/* The sampler fails at the second sample: the copy ran, but the read is not
 * accepted, and the next read returns the same bytes. */
void test_Read_SampleFailureAfterCopy(void) {
    const uint8_t expected[] = {0x00U, 0x01U, 0x02U};
    uint8_t dst[3];
    produce(5U);
    fake.fail_at = 2U;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_SAMPLE, read_ring(dst, 3U));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
    TEST_ASSERT_EQUAL_UINT32(2U, fake.samples);

    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 3U));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, dst, 3U);
    TEST_ASSERT_EQUAL_UINT32(3U, rb.cursor);
}

/* =========================================================================
   SKIP (DROPPING BYTES WITHOUT COPYING THEM)
   ========================================================================= */

void test_Skip_DropsOldestBytes(void) {
    const uint8_t expected[] = {0x03U, 0x04U};
    uint8_t dst[2];
    produce(5U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, skip_ring(3U));
    TEST_ASSERT_EQUAL_UINT32(3U, rb.cursor);
    TEST_ASSERT_EQUAL_UINT32(1U, fake.samples);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 2U));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, dst, 2U);
}

/* Too few bytes is SHORT, and the call can simply be repeated (an echo still
 * arriving, say). */
void test_Skip_ShortUntilAllBytesArrive(void) {
    produce(2U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_SHORT, skip_ring(6U));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);

    produce(4U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, skip_ring(6U));
    TEST_ASSERT_EQUAL_UINT32(6U, rb.cursor);
}

/* Same margin as read: exactly capacity unread is refused. */
void test_Skip_LappedCursorIsOverrun(void) {
    produce(TEST_CAPACITY);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, skip_ring(1U));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
}

void test_Skip_ProducerBehindCursorIsReported(void) {
    start_at(37U);
    produced = 0U; /* the producer restarts */
    produce(3U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_PRODUCER_BEHIND, skip_ring(3U));
    TEST_ASSERT_EQUAL_UINT32(37U, rb.cursor);
}

void test_Skip_LengthAtOrAboveCapacityIsInvalid(void) {
    const uint16_t bad[] = {TEST_CAPACITY, TEST_CAPACITY + 1U, UINT16_MAX};
    produce(TEST_CAPACITY - 1U);
    for (size_t i = 0U; i < sizeof(bad) / sizeof(bad[0]); i++) {
        TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_INVALID_ARG, skip_ring(bad[i]));
    }
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
    TEST_ASSERT_EQUAL_UINT32(0U, fake.samples);
}

/* As for read: zero length succeeds whatever the ring's state, without sampling. */
void test_Skip_ZeroLengthIgnoresRingState(void) {
    produce(TEST_CAPACITY + 1U); /* overrun */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, skip_ring(0U));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
    TEST_ASSERT_EQUAL_UINT32(0U, fake.samples);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, skip_ring(1U));
}

void test_Skip_SampleFailure(void) {
    produce(5U);
    fake.fail_at = 1U;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_SAMPLE, skip_ring(3U));
    TEST_ASSERT_EQUAL_UINT32(0U, rb.cursor);
}

/* =========================================================================
   LONG READS, ROLLOVER, DISCARD
   ========================================================================= */

/* A reply longer than the ring, read as it arrives (a Bulk Read, say). The
 * overwrite check follows the cursor, so there is no false overrun. */
void test_LongRead_LongerThanRing_NoFalseOverrun(void) {
    uint8_t dst[5];
    uint8_t expected[5];
    for (uint32_t chunk = 0U; chunk < 10U; chunk++) {
        produce(5U);
        TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 5U));
        for (uint32_t i = 0U; i < 5U; i++) {
            expected[i] = (uint8_t)(chunk * 5U + i);
        }
        TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, dst, 5U);
    }
    TEST_ASSERT_EQUAL_UINT32(50U, rb.cursor);
}

/* Positions 0xFFFFFFFC..0x00000001 occupy slots 4, 5, 6, 7, 0, 1: the read
 * crosses the storage end and the uint32_t rollover at the same point. */
void test_Rollover_ReadAcrossUint32Wrap(void) {
    const uint8_t expected[] = {0xFCU, 0xFDU, 0xFEU, 0xFFU, 0x00U, 0x01U};
    uint8_t dst[6];
    uint16_t unread = 0U;
    start_at(0xFFFFFFFCU);
    produce(6U);

    TEST_ASSERT_EQUAL_UINT32(2U, produced);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_unread(&rb, produced, &unread));
    TEST_ASSERT_EQUAL_UINT16(6U, unread);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 6U));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, dst, 6U);
    TEST_ASSERT_EQUAL_UINT32(2U, rb.cursor);
}

/* The error boundaries with the cursor just below the rollover and the
 * producer positions on the far side of it. */
void test_Rollover_ErrorBoundariesAcrossUint32Wrap(void) {
    const uint32_t cursor = 0xFFFFFFFCU;
    uint16_t unread = UNREAD_SENTINEL;
    start_at(cursor);

    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_unread(&rb, cursor + (TEST_CAPACITY - 1U), &unread));
    TEST_ASSERT_EQUAL_UINT16(TEST_CAPACITY - 1U, unread);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, sts_ring_buffer_unread(&rb, cursor + TEST_CAPACITY, &unread));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, sts_ring_buffer_unread(&rb, cursor + TEST_CAPACITY + 1U, &unread));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_PRODUCER_BEHIND, sts_ring_buffer_unread(&rb, cursor - 1U, &unread));

    /* The same across the rollover in the other direction: cursor just above 0 */
    start_at(2U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_PRODUCER_BEHIND, sts_ring_buffer_unread(&rb, 0xFFFFFFFFU, &unread));
}

/* Documents a limit, not a feature: the lead is only known modulo 2^32, so
 * results are exact below 2^31 and repeat every 2^32 bytes. */
void test_Horizon_ResultsRepeatEvery2To32(void) {
    uint16_t unread = UNREAD_SENTINEL;
    start_at(0x10U);

    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_unread(&rb, position_after_lead(3U), &unread));
    TEST_ASSERT_EQUAL_UINT16(3U, unread);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN,
                      sts_ring_buffer_unread(&rb, position_after_lead(UINT64_C(0x7FFFFFFF)), &unread));

    /* An overrun reported as PRODUCER_BEHIND */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_PRODUCER_BEHIND,
                      sts_ring_buffer_unread(&rb, position_after_lead(UINT64_C(0x80000003)), &unread));

    /* A full lap later the pattern starts again */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_unread(&rb, position_after_lead(POSITION_RANGE + 3U), &unread));
    TEST_ASSERT_EQUAL_UINT16(3U, unread);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN,
                      sts_ring_buffer_unread(&rb, position_after_lead(POSITION_RANGE + TEST_CAPACITY), &unread));
}

/* Documents a limit, not a feature: a producer 2^32 + 3 bytes ahead passes
 * every check, and the read returns 3 bytes as if nothing had been missed.
 * Keeping the lead below 2^31 is the caller's job. */
void test_Horizon_LappedReadIsAccepted(void) {
    uint8_t dst[3];
    start_at(0x10U);
    produced = position_after_lead(POSITION_RANGE + 3U); /* what the sampler now reports */

    TEST_ASSERT_EQUAL_UINT32(0x13U, produced);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 3U));
    TEST_ASSERT_EQUAL_UINT32(0x13U, rb.cursor);
}

void test_Discard_DropsUnreadBytes(void) {
    const uint8_t expected[] = {0x05U, 0x06U};
    uint8_t dst[2];
    uint16_t unread = UNREAD_SENTINEL;
    produce(5U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_discard(&rb, produced));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_unread(&rb, produced, &unread));
    TEST_ASSERT_EQUAL_UINT16(0U, unread);

    produce(2U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 2U));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, dst, 2U);
}

void test_Discard_RecoversFromOverrun(void) {
    uint16_t unread = 0U;
    produce(20U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, sts_ring_buffer_unread(&rb, produced, &unread));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_discard(&rb, produced));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_unread(&rb, produced, &unread));
    TEST_ASSERT_EQUAL_UINT16(0U, unread);
}

/* A producer restart begins again at 0. Without a discard the old cursor is
 * ahead of it; after discarding to 0, reading resumes from slot 0. */
void test_Discard_RequiredAfterProducerRestart(void) {
    const uint8_t expected[] = {0x00U, 0x01U, 0x02U};
    uint8_t dst[3];
    uint16_t unread = 0U;
    start_at(37U);
    produced = 0U; /* the producer restarts */
    produce(3U);

    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_PRODUCER_BEHIND, sts_ring_buffer_unread(&rb, produced, &unread));

    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_discard(&rb, 0U));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, read_ring(dst, 3U));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, dst, 3U);
}

/* =========================================================================
   CAPACITY RANGE
   ========================================================================= */

#define LARGEST_CAPACITY 32768U

static uint8_t large_storage[LARGEST_CAPACITY];
static uint8_t large_dst[LARGEST_CAPACITY - 1U];

/* Mixes in the position's second byte, so slots 256 apart hold different
 * values; the 8-byte ring's (uint8_t)position pattern repeats every 256. */
static uint8_t tag(uint32_t position) {
    return (uint8_t)(position ^ (position >> 8));
}

/* The fake producer for rings backed by large_storage. */
static void produce_tagged(const sts_ring_buffer_t *ring, uint32_t *position, uint32_t n) {
    for (uint32_t i = 0U; i < n; i++) {
        large_storage[*position & ((uint32_t)ring->capacity - 1U)] = tag(*position);
        (*position)++;
    }
}

static void assert_tagged(const uint8_t *dst, uint32_t from, uint32_t len) {
    for (uint32_t i = 0U; i < len; i++) {
        TEST_ASSERT_EQUAL_HEX8(tag(from + i), dst[i]);
    }
}

/* The smallest ring holds one unread byte, so reads go one byte at a time. */
void test_Capacity_TwoReadsOneByteAtATime(void) {
    sts_ring_buffer_t two;
    uint32_t position = 0U;
    uint16_t unread = UNREAD_SENTINEL;
    uint8_t byte = FILL_BYTE;
    fake.position = &position;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_init(&two, large_storage, 2U));

    for (uint32_t i = 0U; i < 5U; i++) { /* several laps of the 2-byte storage */
        produce_tagged(&two, &position, 1U);
        TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_read(&two, fake_sample, &fake, &byte, 1U));
        TEST_ASSERT_EQUAL_HEX8(tag(i), byte);
    }
    TEST_ASSERT_EQUAL_UINT32(5U, two.cursor);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_INVALID_ARG, sts_ring_buffer_read(&two, fake_sample, &fake, &byte, 2U));

    produce_tagged(&two, &position, 1U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_unread(&two, position, &unread));
    TEST_ASSERT_EQUAL_UINT16(1U, unread);
    produce_tagged(&two, &position, 1U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, sts_ring_buffer_unread(&two, position, &unread));
}

/* The largest ring: a maximum-length read of 32767 bytes that crosses the end
 * of storage, then the overrun boundary at that size. */
void test_Capacity_LargestMaxLengthReadAcrossEnd(void) {
    sts_ring_buffer_t largest;
    uint32_t position = 40000U; /* slot 7232: the copy wraps after 25536 bytes */
    uint16_t unread = 0U;
    fake.position = &position;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_init(&largest, large_storage, LARGEST_CAPACITY));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_discard(&largest, position));
    produce_tagged(&largest, &position, LARGEST_CAPACITY - 1U);

    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_unread(&largest, position, &unread));
    TEST_ASSERT_EQUAL_UINT16(LARGEST_CAPACITY - 1U, unread);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK,
                      sts_ring_buffer_read(&largest, fake_sample, &fake, large_dst, LARGEST_CAPACITY - 1U));
    assert_tagged(large_dst, 40000U, LARGEST_CAPACITY - 1U);
    TEST_ASSERT_EQUAL_UINT32(40000U + LARGEST_CAPACITY - 1U, largest.cursor);

    produce_tagged(&largest, &position, LARGEST_CAPACITY);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_ERR_OVERRUN, sts_ring_buffer_unread(&largest, position, &unread));
}

/* Slots above 255 in a 512-byte ring (a Sync Read size). A slot index held in
 * 8 bits would read the wrong slots here, and never shows in the 8-byte ring. */
void test_Capacity_SlotsAbove255(void) {
    sts_ring_buffer_t ring;
    uint32_t position = 300U;
    fake.position = &position;
    memset(large_storage, 0xAA, 512U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_init(&ring, large_storage, 512U));

    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_discard(&ring, position));
    produce_tagged(&ring, &position, 10U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_read(&ring, fake_sample, &fake, large_dst, 10U));
    assert_tagged(large_dst, 300U, 10U);

    position = 1020U; /* slot 508: slots 508 to 511, then 0 to 3 */
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_discard(&ring, position));
    produce_tagged(&ring, &position, 8U);
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_read(&ring, fake_sample, &fake, large_dst, 8U));
    assert_tagged(large_dst, 1020U, 8U);
}

/* =========================================================================
   REFERENCE MODEL
   ========================================================================= */

#define MODEL_SEED         UINT32_C(0x2545F491)
#define MODEL_STEPS        50000U
#define MODEL_START        UINT32_C(0xFFFFFF00) /* 256 bytes before the uint32_t rollover, which each run crosses */
#define MODEL_HISTORY      0x100000U            /* bytes the model can remember */
#define MODEL_MAX_CAPACITY 64U
#define MODEL_STATUSES     (STS_RING_BUFFER_ERR_SAMPLE + 1) /* ERR_SAMPLE is the last status */

/* Every byte ever produced, indexed by its true offset from MODEL_START: the
 * model uses no masks and no modular arithmetic, unlike the ring. A restart
 * moves the producer back, and history then holds its latest write at each
 * offset, as the storage does. */
static uint8_t history[MODEL_HISTORY];
static uint8_t model_storage[MODEL_MAX_CAPACITY];
static sts_ring_buffer_t model_rb;
static uint16_t model_capacity;
static uint32_t rng_state;

typedef struct {
    uint32_t produced; /* true offsets from MODEL_START */
    uint32_t cursor;
} Model;

static Model model;
static uint32_t model_position; /* what the fake sampler reports during the model run */

enum { MODEL_READ, MODEL_SKIP, MODEL_UNREAD, MODEL_OPS };
static uint32_t outcomes[MODEL_OPS][MODEL_STATUSES];
static uint32_t overruns_after_copy;
static uint32_t copies_overwritten; /* reads refused because their copy read overwritten bytes */
static uint32_t late_overwrites;    /* reads accepted whose bytes were overwritten after the second sample */
static uint32_t restarts_without_discard;

/* When a read's burst lands, relative to its copy and its two samples. */
enum { BURST_BEFORE_SECOND_SAMPLE, BURST_DURING_COPY, BURST_AFTER_SECOND_SAMPLE, BURST_TIMINGS };

/* The last operations of a model run, printed by tearDown if it fails. */
#define MODEL_TRACE_LEN 16U
enum { TRACE_TRAFFIC, TRACE_READ, TRACE_SKIP, TRACE_UNREAD, TRACE_DISCARD, TRACE_RESTART };

typedef struct {
    uint32_t step;
    uint8_t op;
    int32_t lead;    /* before the operation */
    uint16_t len;    /* bytes asked for, produced, or moved back */
    uint16_t burst;
    uint8_t timing;
    uint8_t fail_at;
    uint8_t expected;
    uint8_t got;
    uint16_t count;  /* what unread reported */
    bool discarded;  /* a restart followed by a discard */
} TraceEntry;

static TraceEntry model_trace[MODEL_TRACE_LEN];
static uint32_t model_trace_count;
static uint32_t model_step;
static bool model_running;

static void trace(TraceEntry entry) {
    entry.step = model_step;
    model_trace[model_trace_count % MODEL_TRACE_LEN] = entry;
    model_trace_count++;
}

static void model_trace_report(void) {
    static const char *const ops[] = {"traffic", "read", "skip", "unread", "discard", "restart"};
    static const char *const statuses[] = {"OK", "NULL_PTR", "INVALID_ARG", "SHORT",
                                           "OVERRUN", "PRODUCER_BEHIND", "SAMPLE"};
    static const char *const timings[] = {"before 2nd sample", "during copy", "after 2nd sample"};
    _Static_assert(sizeof(statuses) / sizeof(statuses[0]) == MODEL_STATUSES, "one name per status");

    const bool failed = model_running && (Unity.CurrentTestFailed != 0U);
    model_running = false;
    if (!failed) {
        return;
    }

    const uint32_t shown = (model_trace_count < MODEL_TRACE_LEN) ? model_trace_count : MODEL_TRACE_LEN;
    printf("\nModel trace, capacity %u, last %lu operations:\n", (unsigned)model_capacity, (unsigned long)shown);
    for (uint32_t i = model_trace_count - shown; i < model_trace_count; i++) {
        const TraceEntry *e = &model_trace[i % MODEL_TRACE_LEN];
        printf("  step %5lu %-7s lead %4ld", (unsigned long)e->step, ops[e->op], (long)e->lead);
        if (e->op == TRACE_TRAFFIC) {
            printf(" +%u bytes", (unsigned)e->len);
        } else if (e->op == TRACE_READ) {
            printf(" len %u burst %u %s fail_at %u: expected %s, got %s", (unsigned)e->len, (unsigned)e->burst,
                   timings[e->timing], (unsigned)e->fail_at, statuses[e->expected], statuses[e->got]);
        } else if (e->op == TRACE_SKIP) {
            printf(" len %u fail_at %u: expected %s, got %s", (unsigned)e->len, (unsigned)e->fail_at,
                   statuses[e->expected], statuses[e->got]);
        } else if (e->op == TRACE_UNREAD) {
            printf(": expected %s, got %s, count %u", statuses[e->expected], statuses[e->got], (unsigned)e->count);
        } else if (e->op == TRACE_RESTART) {
            printf(" back %u, %s", (unsigned)e->len, e->discarded ? "discarded" : "no discard");
        }
        printf("\n");
    }
}

/* Counts one outcome. A status added after ERR_SAMPLE would index past the
 * table, so it fails here instead. */
static void count_outcome(size_t op, sts_ring_buffer_status_t got) {
    TEST_ASSERT_TRUE((size_t)got < (size_t)MODEL_STATUSES);
    outcomes[op][got]++;
}

/* xorshift32: the fixed seed gives the same sequence on every run. */
static uint32_t rng_next(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

static uint16_t rng_below(uint16_t n) {
    return (uint16_t)(rng_next() % n);
}

/* The position the ring sees: the true offset, wrapped as the producer's counter wraps. */
static uint32_t ring_position(uint32_t offset) {
    return MODEL_START + offset;
}

/* The model's producer, also used by the fake sampler to write during a read. */
static void model_produce(uint16_t n) {
    for (uint16_t i = 0U; i < n; i++) {
        TEST_ASSERT_TRUE(model.produced < MODEL_HISTORY);
        const uint8_t byte = (uint8_t)rng_next();
        model_storage[ring_position(model.produced) % model_capacity] = byte;
        history[model.produced] = byte;
        model.produced++;
    }
    model_position = ring_position(model.produced);
}

/* The producer's true lead over the cursor; negative after a restart that
 * has not been followed by a discard. */
static int64_t model_lead(void) {
    return (int64_t)model.produced - (int64_t)model.cursor;
}

/* What the header promises for the check against one sample, from the true lead. */
static sts_ring_buffer_status_t model_check(int64_t lead, uint16_t len, sts_ring_buffer_status_t too_few) {
    if (lead < 0) {
        return STS_RING_BUFFER_ERR_PRODUCER_BEHIND;
    }
    if (lead >= (int64_t)model_capacity) {
        return STS_RING_BUFFER_ERR_OVERRUN;
    }
    return (lead < (int64_t)len) ? too_few : STS_RING_BUFFER_OK;
}

/* Scripts the fake sampler for the next call: the burst and when it lands,
 * and which sample (if any) fails. */
static void model_script_sampler(uint16_t burst, uint16_t timing, uint32_t fail_at) {
    fake.samples = 0U;
    fake.write_at = 0U;
    fake.write_after_at = 0U;
    if (timing == BURST_BEFORE_SECOND_SAMPLE) {
        fake.write_at = 2U;
    } else if (timing == BURST_DURING_COPY) {
        fake.write_after_at = 1U;
    } else {
        fake.write_after_at = 2U;
    }
    fake.burst = burst;
    fake.burst_after = burst;
    fake.fail_at = fail_at;
}

/* A read as the header describes it: the length rule, then the first sample
 * and its check, the copy, then the second sample and its check. The burst
 * lands just after the first sample (the copy can read it), just before the
 * second (after the copy, counted by that sample), or just after the second
 * (the copy is done and no sample has counted it yet). */
static void model_read(uint8_t *guarded, size_t guarded_size) {
    const uint16_t len = rng_below((uint16_t)(model_capacity + 2U));
    const uint16_t burst = rng_below((uint16_t)(model_capacity + 1U));
    const uint16_t timing = rng_below((uint16_t)BURST_TIMINGS);
    const uint32_t fail_at = (rng_below(32U) == 0U) ? (uint32_t)(1U + rng_below(2U)) : 0U;
    const uint32_t cursor = model.cursor;
    const int64_t lead = model_lead();
    /* The burst reaches the oldest copied byte's slot once it takes the producer past cursor + capacity. */
    const bool overwrites_copy = (lead + burst) > (int64_t)model_capacity;
    sts_ring_buffer_status_t expected = STS_RING_BUFFER_OK;
    bool second_check = false;

    if (len == 0U) {
        expected = STS_RING_BUFFER_OK;
    } else if (len >= model_capacity) {
        expected = STS_RING_BUFFER_ERR_INVALID_ARG;
    } else if (fail_at == 1U) {
        expected = STS_RING_BUFFER_ERR_SAMPLE;
    } else {
        expected = model_check(lead, len, STS_RING_BUFFER_ERR_SHORT);
        if (expected == STS_RING_BUFFER_OK) {
            /* The second sample counts the burst unless it lands after that sample. */
            const int64_t counted = (timing == BURST_AFTER_SECOND_SAMPLE) ? lead : lead + burst;
            second_check = true;
            expected = (fail_at == 2U) ? STS_RING_BUFFER_ERR_SAMPLE
                                       : model_check(counted, len, STS_RING_BUFFER_ERR_PRODUCER_BEHIND);
        }
    }

    memset(guarded, FILL_BYTE, guarded_size);
    model_script_sampler(burst, timing, fail_at);
    const sts_ring_buffer_status_t got = sts_ring_buffer_read(&model_rb, fake_sample, &fake, guarded, len);
    trace((TraceEntry){.op = TRACE_READ, .lead = (int32_t)lead, .len = len, .burst = burst,
                       .timing = (uint8_t)timing, .fail_at = (uint8_t)fail_at,
                       .expected = (uint8_t)expected, .got = (uint8_t)got});
    TEST_ASSERT_EQUAL(expected, got);
    count_outcome(MODEL_READ, got);
    if (second_check && got == STS_RING_BUFFER_ERR_OVERRUN) {
        overruns_after_copy++;
        if (timing == BURST_DURING_COPY && overwrites_copy) {
            copies_overwritten++;
        }
    }

    if (got == STS_RING_BUFFER_OK) {
        if (len > 0U) {
            TEST_ASSERT_EQUAL_HEX8_ARRAY(&history[cursor], guarded, len);
        }
        if (second_check && timing == BURST_AFTER_SECOND_SAMPLE && overwrites_copy) {
            late_overwrites++; /* and the bytes just checked are still the ones from before the burst */
        }
        model.cursor += len;
    }
    for (size_t i = len; i < guarded_size; i++) { /* nothing past len, whatever the result */
        TEST_ASSERT_EQUAL_HEX8(FILL_BYTE, guarded[i]);
    }
}

static void model_skip(void) {
    const uint16_t len = rng_below((uint16_t)(model_capacity + 2U));
    const uint32_t fail_at = (rng_below(32U) == 0U) ? 1U : 0U;
    const int64_t lead = model_lead();
    sts_ring_buffer_status_t expected = STS_RING_BUFFER_OK;

    if (len == 0U) {
        expected = STS_RING_BUFFER_OK;
    } else if (len >= model_capacity) {
        expected = STS_RING_BUFFER_ERR_INVALID_ARG;
    } else if (fail_at == 1U) {
        expected = STS_RING_BUFFER_ERR_SAMPLE;
    } else {
        expected = model_check(lead, len, STS_RING_BUFFER_ERR_SHORT);
    }

    model_script_sampler(0U, BURST_BEFORE_SECOND_SAMPLE, fail_at);
    const sts_ring_buffer_status_t got = sts_ring_buffer_skip(&model_rb, fake_sample, &fake, len);
    trace((TraceEntry){.op = TRACE_SKIP, .lead = (int32_t)lead, .len = len, .fail_at = (uint8_t)fail_at,
                       .expected = (uint8_t)expected, .got = (uint8_t)got});
    TEST_ASSERT_EQUAL(expected, got);
    count_outcome(MODEL_SKIP, got);
    if (got == STS_RING_BUFFER_OK) {
        model.cursor += len;
    }
}

static void model_unread(void) {
    const int64_t lead = model_lead();
    const sts_ring_buffer_status_t expected = model_check(lead, 0U, STS_RING_BUFFER_OK);
    uint16_t unread = UNREAD_SENTINEL;
    const sts_ring_buffer_status_t got = sts_ring_buffer_unread(&model_rb, model_position, &unread);
    trace((TraceEntry){.op = TRACE_UNREAD, .lead = (int32_t)lead, .expected = (uint8_t)expected,
                       .got = (uint8_t)got, .count = unread});
    TEST_ASSERT_EQUAL(expected, got);
    TEST_ASSERT_EQUAL_UINT32((expected == STS_RING_BUFFER_OK) ? (uint32_t)lead : 0U, unread);
    count_outcome(MODEL_UNREAD, got);
}

/* A producer restart: its position jumps back behind the cursor. Half the
 * time the caller discards straight away, as the header requires; otherwise
 * the stale cursor stays, and later calls see a producer behind it until the
 * producer has caught up again (the documented best-effort limit). */
static void model_restart(void) {
    const uint32_t back = 1U + rng_below((uint16_t)(2U * model_capacity));
    if (model.cursor < back) {
        return; /* too close to the start of history to move back */
    }
    const int64_t lead = model_lead();
    model.produced = model.cursor - back;
    model_position = ring_position(model.produced);
    const bool discard = (rng_below(2U) == 0U);
    trace((TraceEntry){.op = TRACE_RESTART, .lead = (int32_t)lead, .len = (uint16_t)back, .discarded = discard});
    if (discard) {
        TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_discard(&model_rb, model_position));
        model.cursor = model.produced;
    } else {
        restarts_without_discard++;
    }
}

/* One fixed-seed run at one capacity; the caller checks the outcomes it reached. */
static void model_run(uint16_t capacity) {
    uint8_t guarded[MODEL_MAX_CAPACITY + 2U]; /* the longest len asked for, plus a guard byte */
    const size_t guarded_size = (size_t)capacity + 2U;
    model_capacity = capacity;
    model = (Model){0U, 0U};
    model_position = MODEL_START;
    memset(outcomes, 0, sizeof(outcomes));
    overruns_after_copy = 0U;
    copies_overwritten = 0U;
    late_overwrites = 0U;
    restarts_without_discard = 0U;
    model_trace_count = 0U;
    model_running = true;
    rng_state = MODEL_SEED;
    fake.position = &model_position;
    fake.write = model_produce;
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_init(&model_rb, model_storage, capacity));
    TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_discard(&model_rb, MODEL_START));

    for (uint32_t step = 0U; step < MODEL_STEPS; step++) {
        model_step = step;
        switch (rng_below(16U)) {
        case 0U:
        case 1U:
        case 2U:
        case 3U: { /* traffic: mostly a little, sometimes a burst that overruns */
            const uint16_t n = (rng_below(16U) == 0U) ? (uint16_t)(capacity + rng_below(4U))
                                                      : rng_below((uint16_t)(capacity / 2U + 2U));
            trace((TraceEntry){.op = TRACE_TRAFFIC, .lead = (int32_t)model_lead(), .len = n});
            model_produce(n);
            break;
        }
        case 4U:
        case 5U:
        case 6U:
        case 7U:
            model_read(guarded, guarded_size);
            break;
        case 8U:
        case 9U:
            model_skip();
            break;
        case 10U:
        case 11U:
        case 12U:
            model_unread();
            break;
        case 13U:
        case 14U: /* a discard */
            trace((TraceEntry){.op = TRACE_DISCARD, .lead = (int32_t)model_lead()});
            TEST_ASSERT_EQUAL(STS_RING_BUFFER_OK, sts_ring_buffer_discard(&model_rb, model_position));
            model.cursor = model.produced;
            break;
        default:
            model_restart();
            break;
        }
        TEST_ASSERT_EQUAL_UINT32(ring_position(model.cursor), model_rb.cursor);
    }
}

/* Fixed-seed random mixes of traffic, reads (with traffic landing during the
 * copy, between it and the second sample, or after that sample, and sampler
 * failures), skips, counts, discards and producer restarts, at three
 * capacities, checked after every step against the model: status, copied
 * bytes, the bytes after them, the count and the cursor. */
void test_Model_RandomOperationsMatchReference(void) {
    static const uint16_t capacities[] = {2U, 8U, 64U};
    static const struct {
        size_t op;
        sts_ring_buffer_status_t status;
    } required[] = {
        {MODEL_READ, STS_RING_BUFFER_OK},              {MODEL_READ, STS_RING_BUFFER_ERR_SHORT},
        {MODEL_READ, STS_RING_BUFFER_ERR_OVERRUN},     {MODEL_READ, STS_RING_BUFFER_ERR_INVALID_ARG},
        {MODEL_READ, STS_RING_BUFFER_ERR_SAMPLE},      {MODEL_READ, STS_RING_BUFFER_ERR_PRODUCER_BEHIND},
        {MODEL_SKIP, STS_RING_BUFFER_OK},              {MODEL_SKIP, STS_RING_BUFFER_ERR_SHORT},
        {MODEL_SKIP, STS_RING_BUFFER_ERR_OVERRUN},     {MODEL_SKIP, STS_RING_BUFFER_ERR_INVALID_ARG},
        {MODEL_SKIP, STS_RING_BUFFER_ERR_SAMPLE},      {MODEL_SKIP, STS_RING_BUFFER_ERR_PRODUCER_BEHIND},
        {MODEL_UNREAD, STS_RING_BUFFER_OK},            {MODEL_UNREAD, STS_RING_BUFFER_ERR_OVERRUN},
        {MODEL_UNREAD, STS_RING_BUFFER_ERR_PRODUCER_BEHIND},
    };
    for (size_t c = 0U; c < sizeof(capacities) / sizeof(capacities[0]); c++) {
        model_run(capacities[c]);

        /* Each run is only evidence for the outcomes it actually reached. */
        for (size_t i = 0U; i < sizeof(required) / sizeof(required[0]); i++) {
            TEST_ASSERT_TRUE(outcomes[required[i].op][required[i].status] > 0U);
        }
        TEST_ASSERT_TRUE(overruns_after_copy > 0U);
        TEST_ASSERT_TRUE(copies_overwritten > 0U);
        TEST_ASSERT_TRUE(late_overwrites > 0U);
        TEST_ASSERT_TRUE(restarts_without_discard > 0U);
        TEST_ASSERT_TRUE(model.produced > UINT32_C(0) - MODEL_START); /* crossed the rollover */
    }
    model_running = false;
}

/**
 ******************************************************************************
 * @file           : sts_ring_buffer.c
 * @brief          : Ring buffer read-side implementation
 * @author         : Grisham Balloo
 * @date           : 2026-10-29
 ******************************************************************************
 * @details
 * Checks the producer position before and after copying, then advances the
 * consumer cursor only if the copied bytes are intact.
 *
 * @see Lib/STS_Servo/Inc/sts_ring_buffer.h for the API contract.
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#include <stddef.h>
#include <stdint.h>
#include "sts_ring_buffer.h"

/** A distance at or above this is read as negative: the producer is behind the cursor. */
#define STS_RING_BUFFER_HALF_RANGE UINT32_C(0x80000000)

/*
 * distance = produced - cursor, in modular uint32_t arithmetic:
 *   distance <  capacity   positions [cursor, produced) are intact
 *   distance >= capacity   overrun. At exactly capacity nothing is lost yet,
 *                          but the producer's next byte overwrites the oldest
 *                          unread one; treating it as lost is a deliberate
 *                          one-byte margin.
 *   distance >= 2^31       produced is behind the cursor.
 *
 * These verdicts require the true lead to stay below 2^31; see
 * docs/ring-buffer.md for the reasoning and limits.
 */
static sts_ring_buffer_status_t classify(const sts_ring_buffer_t *rb, uint32_t produced, uint16_t *unread) {
    const uint32_t distance = produced - rb->cursor;

    if (distance >= STS_RING_BUFFER_HALF_RANGE) {
        return STS_RING_BUFFER_ERR_PRODUCER_BEHIND;
    }
    if (distance >= (uint32_t)rb->capacity) {
        return STS_RING_BUFFER_ERR_OVERRUN;
    }
    *unread = (uint16_t)distance;
    return STS_RING_BUFFER_OK;
}

/* Byte loop on purpose: memcpy would discard the volatile qualifier. */
static void copy_from_storage(uint8_t *dst, const volatile uint8_t *src, uint16_t n) {
    for (uint16_t i = 0U; i < n; i++) {
        dst[i] = src[i];
    }
}

/*
 * The check before the copy, then the copy. dst is written only on OK.
 * The caller has already checked the pointers and that 0 < len < capacity.
 */
static sts_ring_buffer_status_t peek(const sts_ring_buffer_t *rb, uint32_t produced, uint8_t *dst, uint16_t len) {
    uint16_t unread = 0U;
    const sts_ring_buffer_status_t status = classify(rb, produced, &unread);
    if (status != STS_RING_BUFFER_OK) {
        return status;
    }
    if (unread < len) {
        return STS_RING_BUFFER_ERR_SHORT;
    }

    /* Two runs: from the cursor's slot to the end of storage, then from slot 0. */
    const uint16_t start = (uint16_t)(rb->cursor & ((uint32_t)rb->capacity - 1U));
    uint16_t first = (uint16_t)(rb->capacity - start);
    if (first > len) {
        first = len;
    }
    copy_from_storage(dst, &rb->buffer[start], first);
    copy_from_storage(&dst[first], rb->buffer, (uint16_t)(len - first));
    return STS_RING_BUFFER_OK;
}

/*
 * The check that lets the cursor move past len bytes, then the move. Shared
 * by read (after its copy) and skip, which differ only in what fewer than len
 * unread bytes means: too_few is the status returned in that case.
 * The caller has already checked the pointers and that 0 < len < capacity.
 */
static sts_ring_buffer_status_t accept(sts_ring_buffer_t *rb, uint32_t produced, uint16_t len,
                                       sts_ring_buffer_status_t too_few) {
    uint16_t unread = 0U;
    const sts_ring_buffer_status_t status = classify(rb, produced, &unread);
    if (status != STS_RING_BUFFER_OK) {
        return status;
    }
    if (unread < len) {
        return too_few;
    }

    rb->cursor += len;
    return STS_RING_BUFFER_OK;
}

sts_ring_buffer_status_t sts_ring_buffer_init(sts_ring_buffer_t *rb, const volatile uint8_t *buffer, uint16_t capacity) {
    if (rb == NULL || buffer == NULL) {
        return STS_RING_BUFFER_ERR_NULL_PTR;
    }
    if (capacity < 2U || (capacity & (capacity - 1U)) != 0U) {
        return STS_RING_BUFFER_ERR_INVALID_ARG;
    }

    rb->buffer = buffer;
    rb->capacity = capacity;
    rb->cursor = 0U;
    return STS_RING_BUFFER_OK;
}

sts_ring_buffer_status_t sts_ring_buffer_discard(sts_ring_buffer_t *rb, uint32_t produced) {
    if (rb == NULL || rb->buffer == NULL) {
        return STS_RING_BUFFER_ERR_NULL_PTR;
    }

    rb->cursor = produced;
    return STS_RING_BUFFER_OK;
}

sts_ring_buffer_status_t sts_ring_buffer_unread(const sts_ring_buffer_t *rb, uint32_t produced, uint16_t *unread) {
    if (rb == NULL || rb->buffer == NULL || unread == NULL) {
        return STS_RING_BUFFER_ERR_NULL_PTR;
    }

    *unread = 0U;
    return classify(rb, produced, unread);
}

sts_ring_buffer_status_t sts_ring_buffer_read(sts_ring_buffer_t *rb, sts_ring_buffer_sample_fn sample,
                                              sts_ring_buffer_producer_t *producer, uint8_t *dst, uint16_t len) {
    if (rb == NULL || rb->buffer == NULL || sample == NULL || producer == NULL || dst == NULL) {
        return STS_RING_BUFFER_ERR_NULL_PTR;
    }
    if (len == 0U) {
        return STS_RING_BUFFER_OK; /* nothing requested: no sample is taken */
    }
    if (len >= rb->capacity) {
        return STS_RING_BUFFER_ERR_INVALID_ARG;
    }

    uint32_t produced = 0U;
    if (sample(producer, &produced) != STS_RING_BUFFER_SAMPLE_OK) {
        return STS_RING_BUFFER_ERR_SAMPLE;
    }
    const sts_ring_buffer_status_t status = peek(rb, produced, dst, len);
    if (status != STS_RING_BUFFER_OK) {
        return status;
    }

    if (sample(producer, &produced) != STS_RING_BUFFER_SAMPLE_OK) {
        return STS_RING_BUFFER_ERR_SAMPLE;
    }
    /* The first sample saw at least len bytes, so fewer now means the position went backwards. */
    return accept(rb, produced, len, STS_RING_BUFFER_ERR_PRODUCER_BEHIND);
}

sts_ring_buffer_status_t sts_ring_buffer_skip(sts_ring_buffer_t *rb, sts_ring_buffer_sample_fn sample,
                                              sts_ring_buffer_producer_t *producer, uint16_t len) {
    if (rb == NULL || rb->buffer == NULL || sample == NULL || producer == NULL) {
        return STS_RING_BUFFER_ERR_NULL_PTR;
    }
    if (len == 0U) {
        return STS_RING_BUFFER_OK; /* nothing requested: no sample is taken */
    }
    if (len >= rb->capacity) {
        return STS_RING_BUFFER_ERR_INVALID_ARG;
    }

    uint32_t produced = 0U;
    if (sample(producer, &produced) != STS_RING_BUFFER_SAMPLE_OK) {
        return STS_RING_BUFFER_ERR_SAMPLE;
    }
    /* Nothing was copied, so fewer than len bytes means the rest has not arrived yet. */
    return accept(rb, produced, len, STS_RING_BUFFER_ERR_SHORT);
}

/**
 ******************************************************************************
 * @file           : sts_ring_buffer.h
 * @brief          : Read side of a ring buffer filled by an external producer
 * @author         : Grisham Balloo
 * @date           : 2026-10-29
 ******************************************************************************
 * @details
 * Reads bytes written by an external producer, such as circular DMA, and
 * reports overruns when the consumer falls behind.
 *
 * Typical use (sample_position and producer are supplied by the port):
 * @code{.c}
 * uint8_t storage[64], bytes[4];
 * sts_ring_buffer_t ring;
 * sts_ring_buffer_init(&ring, storage, sizeof storage);
 * // Start the producer on storage with its byte counter at 0.
 * sts_ring_buffer_status_t status =
 *     sts_ring_buffer_read(&ring, sample_position, &producer, bytes, sizeof bytes);
 * // Use bytes only on OK; retry SHORT later; recover other errors in the port.
 * @endcode
 *
 * Use one consumer context and positions from the same producer session.
 * After a restart, call sts_ring_buffer_discard with the new position before
 * reading or skipping. Keep the producer's lead below 2^31 bytes. Storage
 * must be coherent with the producer: use non-cacheable RAM or maintain the
 * data cache as needed; volatile alone does not provide cache coherence.
 *
 * @see @ref ring_buffer_design for recovery guidance and design rationale
 *      (docs/ring-buffer.md in the source tree).
 *
 * @attention
 * Copyright (c) 2026 Grisham Balloo. All rights reserved.
 ******************************************************************************
 */
#pragma once

#include <stdint.h>

/** Result of every sts_ring_buffer function. */
typedef enum {
    STS_RING_BUFFER_OK = 0,
    STS_RING_BUFFER_ERR_NULL_PTR,        /**< Required pointer is NULL, including storage in a zero-initialised ring */
    STS_RING_BUFFER_ERR_INVALID_ARG,     /**< Capacity not a power of two (2 to 32768), or len >= capacity */
    STS_RING_BUFFER_ERR_SHORT,           /**< Fewer than len unread bytes: the rest has not arrived yet */
    STS_RING_BUFFER_ERR_OVERRUN,         /**< Producer is capacity or more bytes past the cursor */
    STS_RING_BUFFER_ERR_PRODUCER_BEHIND, /**< Position is behind cursor, or cursor + len after a copy; best-effort */
    STS_RING_BUFFER_ERR_SAMPLE           /**< Sampler could not provide a valid position */
} sts_ring_buffer_status_t;

/** What a sampler reports for one sample. */
typedef enum {
    STS_RING_BUFFER_SAMPLE_OK = 0, /**< The position written is valid */
    STS_RING_BUFFER_SAMPLE_FAILED  /**< No valid position (an uncertain sample or a stopped producer, for example) */
} sts_ring_buffer_sample_status_t;

/**
 * @brief Producer state passed unchanged to the sampler.
 * The port defines struct sts_ring_buffer_producer once for the program.
 */
typedef struct sts_ring_buffer_producer sts_ring_buffer_producer_t;

/**
 * @brief Reports the total bytes written in this session, modulo 2^32.
 * Count every completed write, including laps, and no unwritten bytes. Return
 * SAMPLE_FAILED if the position is uncertain; the ring reports ERR_SAMPLE.
 * Called in the consumer's context; must not re-enter the same ring.
 * @param producer      The state passed to read or skip.
 * @param[out] produced Valid only on SAMPLE_OK.
 */
typedef sts_ring_buffer_sample_status_t (*sts_ring_buffer_sample_fn)(sts_ring_buffer_producer_t *producer,
                                                                      uint32_t *produced);

/**
 * @brief Caller-allocated consumer state. Fields may be inspected, never modified.
 * Initialise before use. A zero-initialised ring reports NULL_PTR until init;
 * an uninitialised local variable cannot be checked safely.
 */
typedef struct {
    const volatile uint8_t *buffer; /**< Producer-written storage, read one volatile byte at a time */
    uint16_t capacity;              /**< Storage size in bytes, a power of two */
    uint32_t cursor;                /**< Absolute position of the next unread byte */
} sts_ring_buffer_t;

/**
 * @brief Binds the view to its storage and sets the cursor to position 0.
 * @param rb       The view to set up.
 * @param buffer   Producer-written storage; need not be declared volatile.
 * @param capacity Power of two from 2 to 32768, no larger than the real storage.
 *                 Init does not access the storage.
 * @return OK, NULL_PTR, or INVALID_ARG. Failure leaves rb unchanged.
 */
sts_ring_buffer_status_t sts_ring_buffer_init(sts_ring_buffer_t *rb, const volatile uint8_t *buffer, uint16_t capacity);

/**
 * @brief Drops every unread byte: the cursor jumps to @p produced.
 * Use before a transaction or to recover from overrun. Required after a
 * producer restart; pass the new session's sampled position (normally 0).
 * @param rb       The ring.
 * @param produced Sampled position to adopt. Accepted without validation.
 * @return OK or NULL_PTR.
 */
sts_ring_buffer_status_t sts_ring_buffer_discard(sts_ring_buffer_t *rb, uint32_t produced);

/**
 * @brief Number of bytes between the cursor and @p produced.
 * @param rb          The ring.
 * @param produced    A position the caller has sampled.
 * @param[out] unread The count when the result is OK; 0 on OVERRUN or
 *                    PRODUCER_BEHIND. Not written on NULL_PTR.
 * @return OK, NULL_PTR, OVERRUN, or PRODUCER_BEHIND.
 */
sts_ring_buffer_status_t sts_ring_buffer_unread(const sts_ring_buffer_t *rb, uint32_t produced, uint16_t *unread);

/**
 * @brief Copies the oldest @p len bytes and advances the cursor only on OK.
 * Samples before and after copying to detect overwrite. Exactly capacity
 * unread bytes is treated as overrun, reserving a one-byte safety margin.
 * SHORT is safe to retry. On any error, copied bytes must be ignored.
 * @param rb       The ring.
 * @param sample   Called twice on success; once if the first sample/check fails.
 * @param producer Passed to @p sample unchanged.
 * @param dst      Non-NULL destination with room for len bytes; must not overlap
 *                 storage. Nothing past len bytes is written.
 * @param len      Below capacity. Zero returns OK after NULL checks, without
 *                 sampling, copying, or advancing the cursor.
 * @return OK, NULL_PTR, INVALID_ARG, SAMPLE, SHORT, OVERRUN, or PRODUCER_BEHIND.
 *         Either sample can report OVERRUN or PRODUCER_BEHIND.
 */
sts_ring_buffer_status_t sts_ring_buffer_read(sts_ring_buffer_t *rb, sts_ring_buffer_sample_fn sample,
                                              sts_ring_buffer_producer_t *producer, uint8_t *dst, uint16_t len);

/**
 * @brief Drops the oldest @p len unread bytes without copying them.
 * Samples once and advances only on OK; SHORT is safe to retry. The same
 * exactly-capacity overrun margin as read applies.
 * @param rb       The ring.
 * @param sample   Called once for a valid nonzero length.
 * @param producer Passed to @p sample unchanged.
 * @param len      Below capacity. Zero returns OK after NULL checks, without
 *                 sampling or advancing the cursor.
 * @return OK, NULL_PTR, INVALID_ARG, SAMPLE, SHORT, OVERRUN, or PRODUCER_BEHIND.
 */
sts_ring_buffer_status_t sts_ring_buffer_skip(sts_ring_buffer_t *rb, sts_ring_buffer_sample_fn sample,
                                              sts_ring_buffer_producer_t *producer, uint16_t len);

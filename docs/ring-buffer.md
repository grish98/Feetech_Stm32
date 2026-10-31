# Ring buffer design {#ring_buffer_design}

The API header, sts_ring_buffer.h, contains the usage example and function
contracts. This note explains the overwrite checks and the responsibilities
of a port that supplies the producer position.

## Setup and recovery

Initialise the view before starting a producer at position 0. If the producer
is already running, initialise and then discard to its sampled position.
Storage slot 0 corresponds to absolute position 0 in each producer session.

| Result | Consumer action |
| --- | --- |
| `OK` | Use the copied bytes; the cursor has advanced. |
| `ERR_SHORT` | Retry later when more bytes have arrived. |
| `ERR_OVERRUN` | Abandon the partial transaction, sample the producer and discard to that position. |
| `ERR_PRODUCER_BEHIND` | Check the producer session and resynchronise with a valid sample. |
| `ERR_SAMPLE` | Resolve the producer's sampling uncertainty or fault before retrying. |

Read errors leave the cursor unchanged, but the destination may contain an
unusable copy. A discard drops all pending bytes. After every producer restart,
discard to the new session's position before the next read or skip, even if
no error has been reported. `ERR_PRODUCER_BEHIND` is only a best-effort check:
a restarted position can fall inside an otherwise valid unread window.

## Absolute positions and rollover

Positions count every produced byte, including complete laps, modulo 2^32.
The unread distance is unsigned `produced - cursor`; the storage slot is
`position & (capacity - 1)`. A power-of-two capacity makes this mapping survive
counter rollover because the capacity divides 2^32. The `uint16_t` capacity
parameter caps it at 32768.

The module keeps only the consumer cursor. The port supplies fresh producer
positions for read and skip; unread and discard take caller-sampled positions.

## Why read samples twice

The first sample checks that all requested bytes have arrived and the cursor
has not been lapped. The module copies those bytes, then samples again before
accepting the copy and advancing the cursor. Both checks use the original
cursor, so an overwrite during the copy is detected. The check follows the
cursor across successive reads, allowing a transaction to exceed the ring size.

The private `peek` and `accept` helpers keep these steps inside one public
call, preventing a caller from pairing a copy with a stale sample or a
different length. Skip uses the same acceptance check without copying.

Exactly `capacity` unread bytes is treated as overrun: the oldest byte is
still present, but the next write overwrites it. This margin also catches a
sample one byte behind storage. It does not catch a sample ahead of storage,
such as a DMA counter updated before its store lands, so the sampler must not
count a byte until it is stored.

## Counter horizon and restarts

Modular subtraction is unambiguous while the producer's true lead stays
below 2^31 bytes. A larger lead from 2^31 to 2^32 - 1 reports
`ERR_PRODUCER_BEHIND` instead of overrun. Results repeat every 2^32 bytes:
a lead of 2^32 + k, with k below capacity, looks like k unread bytes.
At 1 Mbaud, 8N1, continuous traffic produces 100,000 bytes/s and reaches
2^31 bytes in about six hours. These limits apply to unread lead; normal
consumption allows the counters to wrap repeatedly. Discarding to a valid
current sample resets the lead to zero.

A restart cannot be inferred reliably from positions alone: a restarted
position between `cursor + len` and `cursor + capacity` passes either check.
The port must detect session changes and resynchronise explicitly.

## Memory and ownership

All calls and the sampler run in one consumer context. The producer never
modifies the ring struct; the module needs no locks or atomics. The port
owns any interrupt coordination needed to produce a reliable sample.

Storage is read one volatile byte at a time, in up to two runs split at its
end. This preserves the volatile accesses; it does not bypass a data cache.
Use non-cacheable RAM or the platform's required cache maintenance before
reads. The STM32F103 and STM32F446 have no data cache in front of SRAM;
ports with cacheable storage, such as a Cortex-M7 port, need to address it.

The public struct supports static allocation and embedding without a heap.
Only module functions may modify its fields.

A zero-initialised view used before init reports `ERR_NULL_PTR`. An
uninitialised local view contains indeterminate values and cannot be
validated safely. The module also cannot detect storage smaller than the
capacity supplied by the caller or an overlapping read destination.

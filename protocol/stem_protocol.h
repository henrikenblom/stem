#ifndef STEM_PROTOCOL_H
#define STEM_PROTOCOL_H

/*
 * Stem Bus Protocol — shared definitions for master and slave.
 *
 * Frame format: [CMD 16 bytes] [DATA payload+CRC bytes]
 * CMD:  header(1) + app_cmd(1) + payload(10) + CRC32(4) = 16
 * DATA: payload(STEM_DATA_PAYLOAD_SIZE) + CRC32(4)
 */

#include <stdint.h>

/* --- Configurable payload size ---
 * Both master and slave must agree on this value.
 * Larger payloads reduce per-transaction overhead. */
#define STEM_DATA_PAYLOAD_SIZE 1024

/* --- Derived sizes (do not modify) --- */
#define STEM_CRC_SIZE          4
#define STEM_CMD_SIZE          16
#define STEM_CMD_OVERHEAD      2
#define STEM_CMD_PAYLOAD_SIZE  (STEM_CMD_SIZE - STEM_CMD_OVERHEAD - STEM_CRC_SIZE)
#define STEM_DATA_SIZE         (STEM_DATA_PAYLOAD_SIZE + STEM_CRC_SIZE)
#define STEM_STATUS_SIZE       8

/* --- Pin assignments (same GPIO on both boards) --- */
#define STEM_DATA_BASE_GPIO    2
#define STEM_CLK_GPIO          10
#define STEM_CS_GPIO           11
#define STEM_READY_GPIO        12

/* --- Protocol constants --- */
#define STEM_ACK               0xA5
#define STEM_NACK              0x5A
#define STEM_PING_ECHO         0x42
#define STEM_RETRY_COUNT       3
#define STEM_TIMEOUT_MS        10

/* --- Header byte encoding --- */
#define STEM_RW_WRITE          0x00
#define STEM_RW_READ           0x80
#define STEM_DATA_FLAG         0x40
#define STEM_TYPE_MASK         0x0F
#define STEM_RW_MASK           0x80

#define STEM_TYPE_PING         0x00
#define STEM_TYPE_APP          0x08

#define STEM_HEADER_WRITE(type)      ((uint8_t)(STEM_RW_WRITE | ((type) & STEM_TYPE_MASK)))
#define STEM_HEADER_READ(type)       ((uint8_t)(STEM_RW_READ  | ((type) & STEM_TYPE_MASK)))
#define STEM_HEADER_WRITE_DATA(type) ((uint8_t)(STEM_RW_WRITE | STEM_DATA_FLAG | ((type) & STEM_TYPE_MASK)))
#define STEM_HEADER_TYPE(hdr)        ((uint8_t)((hdr) & STEM_TYPE_MASK))
#define STEM_HEADER_IS_READ(hdr)     (((hdr) & STEM_RW_MASK) != 0)
#define STEM_HEADER_HAS_DATA(hdr)    (((hdr) & STEM_DATA_FLAG) != 0)

/* --- CRC32 (IEEE 802.3, reflected) --- */
#define STEM_CRC32_INIT        0xFFFFFFFFul

/* --- Response signal encoding (D0/D1 via PIO set pins) ---
 * D0=0 means "ready", D1=1 means ACK, D1=0 means NACK.
 * set pins value: bit0=D0, bit1=D1 */
#define STEM_SET_READY_ACK     2   /* D0=0 (ready), D1=1 (ACK) */
#define STEM_SET_READY_NACK    0   /* D0=0 (ready), D1=0 (NACK) */

#endif

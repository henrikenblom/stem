#ifndef STEM_MASTER_H
#define STEM_MASTER_H

#include <stdint.h>
#include <stdbool.h>

/* Initialize the Stem bus master (PIO, DMA, GPIO). */
void stem_init(void);

/* Send a ping and wait for ACK. Returns true on success. */
bool stem_ping(void);

/* Write data to slave (fire-and-forget with ACK). Returns true on success. */
bool stem_write(uint8_t app_cmd, const uint8_t *data, uint16_t len);

/* Write data and read response. Returns true on success. */
bool stem_write_read(uint8_t app_cmd, const uint8_t *data, uint16_t data_len,
                     uint8_t *response, uint16_t max_len, uint16_t *actual_len);

#endif

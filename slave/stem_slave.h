#ifndef STEM_SLAVE_H
#define STEM_SLAVE_H

#include <stdint.h>
#include <stdbool.h>

/*
 * Dispatch callback. Called for each received command.
 *
 * type:         STEM_TYPE_PING, STEM_TYPE_APP, etc.
 * payload:      [app_cmd, data...] for APP commands
 * payload_len:  total bytes in payload
 * response:     buffer to fill with response data
 * response_len: set to number of response bytes (0 for no response)
 */
typedef void (*stem_dispatch_fn)(uint8_t type, const uint8_t *payload,
                                  uint16_t payload_len,
                                  uint8_t *response, uint16_t *response_len);

/* Initialize the Stem bus slave (PIO, DMA, GPIO). */
void stem_slave_init(stem_dispatch_fn dispatch);

/* Poll the slave state machine. Call from the main loop. */
void stem_slave_poll(void);

#endif

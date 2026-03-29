/*
 * Stem bus slave example.
 *
 * Responds to pings and echoes APP commands back to the master.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "stem_slave.h"
#include "stem_protocol.h"

static void dispatch(uint8_t type, const uint8_t *payload,
                     uint16_t payload_len,
                     uint8_t *response, uint16_t *response_len) {
    /* Ping: no response data needed */
    if (type == STEM_TYPE_PING) {
        *response_len = 0;
        return;
    }

    /* APP commands: echo the payload back */
    if (type == STEM_TYPE_APP && payload && payload_len >= 1) {
        uint16_t data_len = payload_len - 1;
        if (data_len > STEM_DATA_PAYLOAD_SIZE)
            data_len = STEM_DATA_PAYLOAD_SIZE;
        memcpy(response, &payload[1], data_len);
        *response_len = data_len;
        return;
    }

    *response_len = 0;
}

int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("Stem slave starting...\n");

    stem_slave_init(dispatch);

    while (true) {
        stem_slave_poll();
    }

    return 0;
}

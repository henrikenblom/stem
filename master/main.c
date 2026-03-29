/*
 * Stem bus master example.
 *
 * Sends a ping every second and reports the result.
 * Sends an echo request and verifies the response.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "stem_master.h"
#include "stem_protocol.h"

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("Stem master starting...\n");

    stem_init();

    uint32_t count = 0;
    while (true) {
        sleep_ms(1000);
        count++;

        if (stem_ping()) {
            printf("[%lu] ping OK\n", (unsigned long)count);
        } else {
            printf("[%lu] ping FAILED\n", (unsigned long)count);
            continue;
        }

        /* Echo test: send 32 bytes, read them back */
        uint8_t send_buf[32];
        for (int i = 0; i < 32; i++)
            send_buf[i] = (uint8_t)(count + i);

        uint8_t recv_buf[STEM_DATA_PAYLOAD_SIZE];
        uint16_t recv_len = 0;

        if (stem_write_read(0x01, send_buf, 32, recv_buf, sizeof(recv_buf), &recv_len)) {
            if (recv_len >= 32 && memcmp(send_buf, recv_buf, 32) == 0) {
                printf("[%lu] echo OK (32 bytes)\n", (unsigned long)count);
            } else {
                printf("[%lu] echo MISMATCH (got %u bytes)\n",
                       (unsigned long)count, recv_len);
            }
        } else {
            printf("[%lu] echo FAILED\n", (unsigned long)count);
        }
    }

    return 0;
}

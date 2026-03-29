/*
 * Stem bus master driver.
 *
 * Uses PIO1 for the bus protocol. DMA channels 2 (TX) and 3 (RX).
 * CRC32 computed via DMA sniffer (channel 0).
 */

#include "stem_master.h"
#include "stem_protocol.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "stem_master.pio.h"
#include <string.h>
#include <stdio.h>

#define DATA_BASE  STEM_DATA_BASE_GPIO
#define DATA_COUNT 8
#define CLK_PIN    STEM_CLK_GPIO
#define CS_PIN     STEM_CS_GPIO
#define READY_PIN  STEM_READY_GPIO
#define TX_DMA     2
#define RX_DMA     3
#define CRC_DMA    0
#define FRAME_SIZE (STEM_CMD_SIZE + STEM_DATA_SIZE)
#define BUS_FREQ   17500000

static PIO pio;
static uint sm;
static uint prog_offset;

/* --- CRC32 via DMA sniffer --- */

static uint8_t crc_dummy;

static uint32_t calculate_crc(const uint8_t *data, uint32_t length) {
    dma_channel_config c = dma_channel_get_default_config(CRC_DMA);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_sniff_enable(&c, true);
    dma_sniffer_set_data_accumulator(STEM_CRC32_INIT);
    dma_sniffer_set_output_reverse_enabled(true);
    dma_sniffer_enable(CRC_DMA, DMA_SNIFF_CTRL_CALC_VALUE_CRC32R, true);
    dma_channel_configure(CRC_DMA, &c, &crc_dummy, data, length, true);
    dma_channel_wait_for_finish_blocking(CRC_DMA);
    return ~dma_sniffer_get_data_accumulator();
}

static bool validate_crc(const uint8_t *buf, uint32_t crc_offset) {
    uint32_t computed = calculate_crc(buf, crc_offset);
    uint32_t received;
    memcpy(&received, &buf[crc_offset], sizeof(received));
    return computed == received;
}

/* --- Packet building --- */

static uint8_t cmd_buf[STEM_CMD_SIZE] __attribute__((aligned(4)));
static uint8_t data_block[STEM_DATA_SIZE] __attribute__((aligned(4)));
static uint8_t tx_frame[FRAME_SIZE] __attribute__((aligned(4)));
static uint8_t rx_buf[STEM_DATA_SIZE] __attribute__((aligned(4)));

static void build_cmd(uint8_t header, uint8_t app_cmd,
                      const uint8_t *payload, uint16_t len) {
    memset(cmd_buf, 0, STEM_CMD_SIZE);
    cmd_buf[0] = header;
    cmd_buf[1] = app_cmd;
    if (payload && len > 0) {
        uint16_t n = (len > STEM_CMD_PAYLOAD_SIZE) ? STEM_CMD_PAYLOAD_SIZE : len;
        memcpy(&cmd_buf[2], payload, n);
    }
    uint32_t crc = calculate_crc(cmd_buf, STEM_CMD_SIZE - STEM_CRC_SIZE);
    memcpy(&cmd_buf[STEM_CMD_SIZE - STEM_CRC_SIZE], &crc, sizeof(crc));
}

static void build_data(const uint8_t *payload, uint16_t len) {
    memset(data_block, 0, STEM_DATA_SIZE);
    if (payload && len > 0) {
        uint16_t n = (len > STEM_DATA_PAYLOAD_SIZE) ? STEM_DATA_PAYLOAD_SIZE : len;
        memcpy(data_block, payload, n);
    }
    uint32_t crc = calculate_crc(data_block, STEM_DATA_PAYLOAD_SIZE);
    memcpy(&data_block[STEM_DATA_PAYLOAD_SIZE], &crc, sizeof(crc));
}

static void build_frame(void) {
    memcpy(tx_frame, cmd_buf, STEM_CMD_SIZE);
    memcpy(&tx_frame[STEM_CMD_SIZE], data_block, STEM_DATA_SIZE);
}

/* --- PIO helpers --- */

static void load_x(uint32_t value) {
    pio_sm_put(pio, sm, value);
    pio_sm_exec(pio, sm, pio_encode_out(pio_x, 32));
}

static void load_y(uint32_t value) {
    pio_sm_put(pio, sm, value);
    pio_sm_exec(pio, sm, pio_encode_out(pio_y, 32));
}

static bool nack_flagged(void) {
    bool flagged = pio_interrupt_get(pio, 2);
    if (flagged) pio_interrupt_clear(pio, 2);
    return flagged;
}

/* --- Transaction --- */

static bool transact(const uint8_t *tx, uint32_t tx_len,
                     uint8_t *rx, uint32_t rx_len) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_out(pio_null, 32));
    pio_sm_clear_fifos(pio, sm);
    pio_sm_set_consecutive_pindirs(pio, sm, DATA_BASE, DATA_COUNT, true);

    load_x(tx_len - 1);
    load_y(rx_len > 0 ? rx_len - 1 : 0);

    /* TX DMA */
    dma_channel_config tc = dma_channel_get_default_config(TX_DMA);
    channel_config_set_transfer_data_size(&tc, DMA_SIZE_32);
    channel_config_set_read_increment(&tc, true);
    channel_config_set_write_increment(&tc, false);
    channel_config_set_dreq(&tc, pio_get_dreq(pio, sm, true));
    dma_channel_configure(TX_DMA, &tc, &pio->txf[sm], tx, tx_len / 4, true);

    /* RX DMA (only for read transactions) */
    if (rx_len > 0) {
        dma_channel_config rc = dma_channel_get_default_config(RX_DMA);
        channel_config_set_transfer_data_size(&rc, DMA_SIZE_32);
        channel_config_set_read_increment(&rc, false);
        channel_config_set_write_increment(&rc, true);
        channel_config_set_dreq(&rc, pio_get_dreq(pio, sm, false));
        dma_channel_configure(RX_DMA, &rc, rx, &pio->rxf[sm], rx_len / 4, true);
    }

    pio_interrupt_clear(pio, 2);

    /* Wait for slave READY (active LOW) */
    absolute_time_t deadline = make_timeout_time_ms(STEM_TIMEOUT_MS);
    while (gpio_get(READY_PIN)) {
        if (time_reached(deadline)) return false;
        tight_loop_contents();
    }

    /* Start transaction */
    gpio_put(CS_PIN, 1);  /* CS active HIGH */
    pio_interrupt_clear(pio, 0);
    pio_sm_exec(pio, sm, pio_encode_jmp(prog_offset));
    pio_sm_set_enabled(pio, sm, true);

    /* Wait for PIO completion */
    while (!pio_interrupt_get(pio, 0)) {
        if (time_reached(deadline)) {
            dma_channel_abort(TX_DMA);
            dma_channel_abort(RX_DMA);
            pio_sm_set_enabled(pio, sm, false);
            pio_sm_set_consecutive_pindirs(pio, sm, DATA_BASE, DATA_COUNT, false);
            gpio_put(CS_PIN, 0);
            return false;
        }
        tight_loop_contents();
    }

    if (rx_len > 0)
        dma_channel_wait_for_finish_blocking(RX_DMA);

    /* Cleanup */
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_exec(pio, sm, pio_encode_mov(pio_pins, pio_null));
    pio_sm_set_consecutive_pindirs(pio, sm, DATA_BASE, DATA_COUNT, false);
    while (!pio_sm_is_rx_fifo_empty(pio, sm))
        pio_sm_get(pio, sm);
    pio_interrupt_clear(pio, 0);
    gpio_put(CS_PIN, 0);  /* CS deassert */

    return true;
}

/* --- Public API --- */

void stem_init(void) {
    pio = pio1;
    sm = pio_claim_unused_sm(pio, true);
    dma_channel_claim(TX_DMA);
    dma_channel_claim(RX_DMA);
    dma_channel_claim(CRC_DMA);

    prog_offset = pio_add_program(pio, &stem_master_program);
    uint16_t clkdiv = clock_get_hz(clk_sys) / (BUS_FREQ * CLKDIV);

    pio_sm_config cfg = stem_master_program_get_default_config(prog_offset);
    sm_config_set_out_pins(&cfg, DATA_BASE, DATA_COUNT);
    sm_config_set_in_pins(&cfg, DATA_BASE);
    sm_config_set_sideset_pins(&cfg, CLK_PIN);
    sm_config_set_jmp_pin(&cfg, DATA_BASE + 1);  /* D1 for ACK/NACK */
    sm_config_set_out_shift(&cfg, true, true, 32);
    sm_config_set_in_shift(&cfg, true, true, 32);
    sm_config_set_clkdiv_int_frac(&cfg, clkdiv, 0);
    pio_sm_init(pio, sm, prog_offset, &cfg);

    for (uint pin = DATA_BASE; pin < DATA_BASE + DATA_COUNT; pin++) {
        pio_gpio_init(pio, pin);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_2MA);
    }
    pio_gpio_init(pio, CLK_PIN);
    gpio_set_slew_rate(CLK_PIN, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(CLK_PIN, GPIO_DRIVE_STRENGTH_2MA);

    pio_sm_set_consecutive_pindirs(pio, sm, CLK_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, DATA_BASE, DATA_COUNT, true);
    pio->input_sync_bypass |= ((0xFFu << DATA_BASE) | (1u << CLK_PIN));

    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 0);  /* deasserted */

    gpio_init(READY_PIN);
    gpio_set_dir(READY_PIN, GPIO_IN);
    gpio_pull_up(READY_PIN);

    printf("[STEM] Master init: PIO1 SM%u, clkdiv=%u, bus=~%u MHz\n",
           sm, clkdiv, BUS_FREQ / 1000000);

    /* Startup ping to settle D0 (RP2350 default pull-down workaround) */
    stem_ping();
}

bool stem_ping(void) {
    uint8_t echo = STEM_PING_ECHO;
    build_cmd(STEM_HEADER_WRITE(STEM_TYPE_PING), 0, &echo, 1);
    build_data(NULL, 0);
    build_frame();

    for (int i = 0; i < STEM_RETRY_COUNT; i++) {
        if (transact(tx_frame, FRAME_SIZE, NULL, 0) && !nack_flagged())
            return true;
    }
    return false;
}

bool stem_write(uint8_t app_cmd, const uint8_t *data, uint16_t len) {
    if (len > STEM_DATA_PAYLOAD_SIZE) return false;

    if (len > STEM_CMD_PAYLOAD_SIZE) {
        build_cmd(STEM_HEADER_WRITE_DATA(STEM_TYPE_APP), app_cmd, NULL, 0);
        build_data(data, len);
    } else {
        build_cmd(STEM_HEADER_WRITE(STEM_TYPE_APP), app_cmd, data, len);
        build_data(NULL, 0);
    }
    build_frame();

    for (int i = 0; i < STEM_RETRY_COUNT; i++) {
        if (transact(tx_frame, FRAME_SIZE, NULL, 0) && !nack_flagged())
            return true;
    }
    return false;
}

bool stem_write_read(uint8_t app_cmd, const uint8_t *data, uint16_t data_len,
                     uint8_t *response, uint16_t max_len, uint16_t *actual_len) {
    if (data_len > STEM_CMD_PAYLOAD_SIZE) {
        build_cmd(STEM_HEADER_WRITE_DATA(STEM_TYPE_APP) | STEM_RW_READ,
                  app_cmd, NULL, 0);
        build_data(data, data_len);
    } else {
        build_cmd(STEM_HEADER_READ(STEM_TYPE_APP), app_cmd, data, data_len);
        build_data(NULL, 0);
    }
    build_frame();

    for (int i = 0; i < STEM_RETRY_COUNT; i++) {
        if (!transact(tx_frame, FRAME_SIZE, rx_buf, STEM_DATA_SIZE))
            continue;
        if (nack_flagged()) continue;
        if (!validate_crc(rx_buf, STEM_DATA_PAYLOAD_SIZE)) continue;

        uint16_t n = (STEM_DATA_PAYLOAD_SIZE < max_len)
                     ? STEM_DATA_PAYLOAD_SIZE : max_len;
        memcpy(response, rx_buf, n);
        *actual_len = n;
        return true;
    }
    return false;
}

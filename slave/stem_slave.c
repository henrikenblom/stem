/*
 * Stem bus slave driver.
 *
 * Uses PIO1 for the bus protocol. DMA channels 2 (RX), 3 (TX), 0 (CRC).
 * Non-blocking: the poll function advances one state per call and returns.
 * RX PIO uses push noblock — never stalls on FIFO overflow.
 */

#include "stem_slave.h"
#include "stem_protocol.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "stem_slave.pio.h"
#include <string.h>
#include <stdio.h>

#define DATA_COUNT 8
#define RX_DMA     2
#define TX_DMA     3
#define CRC_DMA    0
#define FRAME_SIZE (STEM_CMD_SIZE + STEM_DATA_SIZE)
#define DMA_TIMEOUT_US 200000

/* TX PIO patch offsets */
#define TX_PATCH_SIGNAL       5
#define TX_PATCH_WAIT_CLK_HIGH 8
#define TX_PATCH_WAIT_CLK_LOW  9

/* RX PIO patch offsets */
#define RX_PATCH_WAIT_CS       0
#define RX_PATCH_WAIT_CLK_HIGH 1
#define RX_PATCH_WAIT_CLK_LOW  3

enum StemState {
    STATE_IDLE,
    STATE_RECEIVING,
    STATE_VALIDATING,
    STATE_PROCESSING,
    STATE_RESPONDING,
    STATE_WAITING_TX,
};

static stem_dispatch_fn dispatch_fn;
static PIO pio;
static uint rx_sm, tx_sm;
static uint rx_offset, tx_offset;
static enum StemState state;
static uint32_t rx_dma_start_time;

/* Buffers */
static uint8_t rx_frame[FRAME_SIZE] __attribute__((aligned(4)));
static uint8_t *const cmd_rx_buf = rx_frame;
static uint8_t *const data_rx_buf = rx_frame + STEM_CMD_SIZE;
static uint8_t status_tx_buf[STEM_STATUS_SIZE] __attribute__((aligned(4)));
static uint8_t data_tx_buf[STEM_DATA_SIZE] __attribute__((aligned(4)));
static uint8_t dispatch_buf[1 + STEM_DATA_PAYLOAD_SIZE];
static uint8_t pending_response[STEM_DATA_PAYLOAD_SIZE];
static uint16_t pending_response_len;
static uint8_t rx_header;
static bool rx_has_data;

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

/* --- Response building --- */

static void build_status(uint8_t status_byte) {
    memset(status_tx_buf, 0, STEM_STATUS_SIZE);
    status_tx_buf[0] = status_byte;
    uint32_t crc = calculate_crc(status_tx_buf, STEM_STATUS_SIZE - STEM_CRC_SIZE);
    memcpy(&status_tx_buf[STEM_STATUS_SIZE - STEM_CRC_SIZE], &crc, sizeof(crc));
}

static void build_data_response(const uint8_t *payload, uint16_t len) {
    memset(data_tx_buf, 0, STEM_DATA_SIZE);
    if (payload && len > 0) {
        uint16_t n = (len > STEM_DATA_PAYLOAD_SIZE) ? STEM_DATA_PAYLOAD_SIZE : len;
        memcpy(data_tx_buf, payload, n);
    }
    uint32_t crc = calculate_crc(data_tx_buf, STEM_DATA_PAYLOAD_SIZE);
    memcpy(&data_tx_buf[STEM_DATA_PAYLOAD_SIZE], &crc, sizeof(crc));
}

/* --- PIO management --- */

static void arm_rx(void) {
    pio_sm_set_enabled(pio, rx_sm, false);
    pio_sm_restart(pio, rx_sm);
    pio_sm_exec(pio, rx_sm, pio_encode_out(pio_null, 32));
    pio_sm_set_consecutive_pindirs(pio, rx_sm, STEM_DATA_BASE_GPIO, DATA_COUNT, false);

    /* Unjoin FIFO to access TX FIFO for register loading */
    hw_clear_bits(&pio->sm[rx_sm].shiftctrl, 1u << PIO_SM0_SHIFTCTRL_FJOIN_RX_LSB);
    pio_sm_clear_fifos(pio, rx_sm);

    /* Load X = byte count, Y = push sub-counter */
    pio_sm_put(pio, rx_sm, FRAME_SIZE - 1);
    pio_sm_exec(pio, rx_sm, pio_encode_out(pio_x, 32));
    pio_sm_put(pio, rx_sm, 3);
    pio_sm_exec(pio, rx_sm, pio_encode_out(pio_y, 32));

    /* Rejoin for 8-word RX FIFO depth */
    hw_set_bits(&pio->sm[rx_sm].shiftctrl, 1u << PIO_SM0_SHIFTCTRL_FJOIN_RX_LSB);
    pio_sm_clear_fifos(pio, rx_sm);

    /* DMA: FRAME_SIZE bytes into rx_frame */
    dma_channel_config c = dma_channel_get_default_config(RX_DMA);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, rx_sm, false));
    dma_channel_configure(RX_DMA, &c, rx_frame, &pio->rxf[rx_sm],
                          FRAME_SIZE / 4, true);

    pio_interrupt_clear(pio, 0);
    pio_sm_exec(pio, rx_sm, pio_encode_jmp(rx_offset));
    pio_sm_set_enabled(pio, rx_sm, true);

    rx_dma_start_time = time_us_32();
    gpio_put(STEM_READY_GPIO, 0);  /* READY active LOW */
}

static void start_tx(const uint8_t *buf, uint32_t len, uint32_t signal) {
    pio_sm_set_enabled(pio, tx_sm, false);
    pio_sm_restart(pio, tx_sm);
    pio_sm_clear_fifos(pio, tx_sm);

    pio->instr_mem[tx_offset + TX_PATCH_SIGNAL] = pio_encode_set(pio_pins, signal);

    pio_sm_put(pio, tx_sm, len - 1);
    pio_sm_exec(pio, tx_sm, pio_encode_pull(false, true));
    pio_sm_exec(pio, tx_sm, pio_encode_mov(pio_x, pio_osr));

    dma_channel_config c = dma_channel_get_default_config(TX_DMA);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, tx_sm, true));
    dma_channel_configure(TX_DMA, &c, &pio->txf[tx_sm], buf, len / 4, true);

    pio_interrupt_clear(pio, 1);
    state = STATE_WAITING_TX;
    pio_set_irq0_source_enabled(pio, pis_interrupt1, true);
    pio_sm_exec(pio, tx_sm, pio_encode_jmp(tx_offset));
    pio_sm_set_enabled(pio, tx_sm, true);
}

static void abort_tx(void) {
    pio_set_irq0_source_enabled(pio, pis_interrupt1, false);
    dma_channel_abort(TX_DMA);
    pio_sm_set_enabled(pio, tx_sm, false);
    pio_interrupt_clear(pio, 1);
    pio_sm_set_consecutive_pindirs(pio, tx_sm, STEM_DATA_BASE_GPIO, DATA_COUNT, false);
}

/* --- ISRs --- */

static void tx_complete_isr(void) {
    gpio_put(STEM_READY_GPIO, 1);
    pio_set_irq0_source_enabled(pio, pis_interrupt1, false);
    dma_channel_abort(TX_DMA);
    pio_sm_set_enabled(pio, tx_sm, false);
    pio_interrupt_clear(pio, 1);
    pio_sm_set_consecutive_pindirs(pio, tx_sm, STEM_DATA_BASE_GPIO, DATA_COUNT, false);
    arm_rx();
    state = STATE_RECEIVING;
}

static void cs_deassert_isr(uint gpio, uint32_t events) {
    gpio_put(STEM_READY_GPIO, 1);
    if (state == STATE_WAITING_TX)
        abort_tx();
    arm_rx();
    state = STATE_RECEIVING;
}

/* --- State machine (one state per poll call) --- */

static void do_receiving(void) {
    if (!pio_interrupt_get(pio, 0)) {
        if (dma_channel_is_busy(RX_DMA)) {
            if ((time_us_32() - rx_dma_start_time) > DMA_TIMEOUT_US) {
                dma_channel_abort(RX_DMA);
                arm_rx();
            }
        }
        return;
    }
    if (dma_channel_is_busy(RX_DMA)) return;
    state = STATE_VALIDATING;
}

static void do_validating(void) {
    rx_header = cmd_rx_buf[0];

    if (!validate_crc(cmd_rx_buf, STEM_CMD_SIZE - STEM_CRC_SIZE)) {
        build_data_response(NULL, 0);
        start_tx(data_tx_buf, STEM_DATA_SIZE, STEM_SET_READY_NACK);
        return;
    }

    rx_has_data = STEM_HEADER_HAS_DATA(rx_header);
    if (rx_has_data) {
        if (!validate_crc(data_rx_buf, STEM_DATA_PAYLOAD_SIZE)) {
            build_data_response(NULL, 0);
            start_tx(data_tx_buf, STEM_DATA_SIZE, STEM_SET_READY_NACK);
            return;
        }
    }
    state = STATE_PROCESSING;
}

static void do_processing(void) {
    uint8_t type = STEM_HEADER_TYPE(rx_header);
    uint8_t app_cmd = cmd_rx_buf[1];
    uint16_t dispatch_len;

    if (rx_has_data) {
        dispatch_buf[0] = app_cmd;
        memcpy(&dispatch_buf[1], data_rx_buf, STEM_DATA_PAYLOAD_SIZE);
        dispatch_len = 1 + STEM_DATA_PAYLOAD_SIZE;
    } else {
        dispatch_buf[0] = app_cmd;
        memcpy(&dispatch_buf[1], &cmd_rx_buf[STEM_CMD_OVERHEAD], STEM_CMD_PAYLOAD_SIZE);
        dispatch_len = 1 + STEM_CMD_PAYLOAD_SIZE;
    }

    uint16_t response_len = 0;
    dispatch_fn(type, dispatch_buf, dispatch_len,
                pending_response, &response_len);
    pending_response_len = response_len;
    state = STATE_RESPONDING;
}

static void do_responding(void) {
    bool respond_with_data = STEM_HEADER_IS_READ(rx_header);

    if (respond_with_data) {
        build_data_response(pending_response, pending_response_len);
        pending_response_len = 0;
        start_tx(data_tx_buf, STEM_DATA_SIZE, STEM_SET_READY_ACK);
    } else {
        build_status(STEM_ACK);
        start_tx(status_tx_buf, STEM_STATUS_SIZE, STEM_SET_READY_ACK);
    }
}

static void do_waiting_tx(void) {
    if (!gpio_get(STEM_CS_GPIO)) {
        abort_tx();
        arm_rx();
        state = STATE_RECEIVING;
    }
}

/* --- Public API --- */

void stem_slave_init(stem_dispatch_fn dispatch) {
    dispatch_fn = dispatch;

    pio = pio1;
    rx_sm = pio_claim_unused_sm(pio, true);
    tx_sm = pio_claim_unused_sm(pio, true);
    dma_channel_claim(RX_DMA);
    dma_channel_claim(TX_DMA);
    dma_channel_claim(CRC_DMA);

    rx_offset = pio_add_program(pio, &stem_slave_rx_program);
    tx_offset = pio_add_program(pio, &stem_slave_tx_program);

    /* RX SM config — autopush DISABLED (push noblock in PIO program) */
    pio_sm_config rx_cfg = stem_slave_rx_program_get_default_config(rx_offset);
    sm_config_set_in_pins(&rx_cfg, STEM_DATA_BASE_GPIO);
    sm_config_set_out_pins(&rx_cfg, STEM_DATA_BASE_GPIO, DATA_COUNT);
    sm_config_set_in_shift(&rx_cfg, true, false, 32);
    sm_config_set_out_shift(&rx_cfg, true, true, 32);
    pio_sm_init(pio, rx_sm, rx_offset, &rx_cfg);

    /* Patch RX PIO with actual pin numbers */
    pio->instr_mem[rx_offset + RX_PATCH_WAIT_CS] =
        pio_encode_wait_gpio(true, STEM_CS_GPIO);
    pio->instr_mem[rx_offset + RX_PATCH_WAIT_CLK_HIGH] =
        pio_encode_wait_gpio(true, STEM_CLK_GPIO);
    pio->instr_mem[rx_offset + RX_PATCH_WAIT_CLK_LOW] =
        pio_encode_wait_gpio(false, STEM_CLK_GPIO);

    /* TX SM config — set pins are D0,D1 for in-band signaling */
    pio_sm_config tx_cfg = stem_slave_tx_program_get_default_config(tx_offset);
    sm_config_set_out_pins(&tx_cfg, STEM_DATA_BASE_GPIO, DATA_COUNT);
    sm_config_set_set_pins(&tx_cfg, STEM_DATA_BASE_GPIO, 2);
    sm_config_set_out_shift(&tx_cfg, true, true, 32);
    pio_sm_init(pio, tx_sm, tx_offset, &tx_cfg);

    /* Patch TX PIO with actual pin numbers */
    pio->instr_mem[tx_offset + TX_PATCH_WAIT_CLK_HIGH] =
        pio_encode_wait_gpio(true, STEM_CLK_GPIO);
    pio->instr_mem[tx_offset + TX_PATCH_WAIT_CLK_LOW] =
        pio_encode_wait_gpio(false, STEM_CLK_GPIO);

    /* Data bus GPIO setup */
    for (uint i = 0; i < DATA_COUNT; i++) {
        uint pin = STEM_DATA_BASE_GPIO + i;
        pio_gpio_init(pio, pin);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_2MA);
    }

    /* CS pin — input, active HIGH */
    gpio_init(STEM_CS_GPIO);
    gpio_set_dir(STEM_CS_GPIO, GPIO_IN);
    gpio_pull_up(STEM_CS_GPIO);
    gpio_set_input_hysteresis_enabled(STEM_CS_GPIO, true);
    gpio_set_irq_enabled_with_callback(STEM_CS_GPIO, GPIO_IRQ_EDGE_FALL,
                                        true, cs_deassert_isr);

    /* READY pin — output, start HIGH (not ready) */
    gpio_init(STEM_READY_GPIO);
    gpio_set_dir(STEM_READY_GPIO, GPIO_OUT);
    gpio_put(STEM_READY_GPIO, 1);

    /* CLK pin — input */
    gpio_init(STEM_CLK_GPIO);
    gpio_set_dir(STEM_CLK_GPIO, GPIO_IN);
    gpio_set_input_hysteresis_enabled(STEM_CLK_GPIO, true);

    /* Input sync bypass for speed */
    pio->input_sync_bypass |=
        ((0xFFu << STEM_DATA_BASE_GPIO) |
         (1u << STEM_CLK_GPIO) |
         (1u << STEM_CS_GPIO));

    /* TX complete ISR */
    pio_set_irq0_source_enabled(pio, pis_interrupt1, false);
    irq_set_exclusive_handler(PIO1_IRQ_0, tx_complete_isr);
    irq_set_enabled(PIO1_IRQ_0, true);

    pending_response_len = 0;
    state = STATE_IDLE;

    printf("[STEM] Slave init: PIO1, RX SM%u, TX SM%u\n", rx_sm, tx_sm);
}

void stem_slave_poll(void) {
    switch (state) {
    case STATE_IDLE:
        arm_rx();
        state = STATE_RECEIVING;
        break;
    case STATE_RECEIVING:
        do_receiving();
        break;
    case STATE_VALIDATING:
        do_validating();
        break;
    case STATE_PROCESSING:
        do_processing();
        break;
    case STATE_RESPONDING:
        do_responding();
        break;
    case STATE_WAITING_TX:
        do_waiting_tx();
        break;
    }
}

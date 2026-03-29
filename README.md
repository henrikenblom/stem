# Stem Bus Protocol

An 8-bit parallel bus protocol for high-speed communication between RP2040/RP2350 microcontrollers. Uses PIO state machines for wire-speed transfers with DMA, CRC32 error detection, and in-band flow control.

## Performance

- 40+ Mbps sustained throughput (17.75 MHz bus clock, 1024-byte payload)
- 0% error rate with proper wiring and pull-ups
- Hardware CRC32 via DMA sniffer
- Non-blocking slave (never stalls on FIFO overflow)

## Wiring

Connect two Pico 2 (or Pico) boards with 12 wires plus ground:

| Signal  | GPIO | Direction    | Description                            |
|---------|------|--------------|----------------------------------------|
| D0-D7   | 2-9  | Bidirectional| 8-bit data bus                         |
| CLK     | 10   | Master out   | Bus clock (PIO sideset)                |
| CS      | 11   | Master out   | Chip select (active HIGH)              |
| READY   | 12   | Slave out    | Slave ready (active LOW)               |
| GND     | GND  | -            | Common ground (required)               |

All signal pins use the same GPIO numbers on both boards.

### Pull-up Resistors

Add a **4.7K pull-up to 3.3V** on every signal pin (D0-D7, CLK, CS, READY). These are essential for signal integrity at bus speed and for correct idle states. Place them close to the slave board.

```
3.3V ----[4.7K]---- signal pin
```

## Protocol Overview

Every transaction sends a fixed-size frame: 16-byte CMD header + DATA block (payload + 4-byte CRC32). The DATA block size is configurable (default 1024 bytes payload).

### Write Transaction (master to slave)

```
Master: assert READY poll → CS HIGH → TX frame → release bus → wait D0 LOW
Slave:  RX frame → validate CRC → process → drive D0=0 D1=1 (ACK)
Master: read D1 (ACK/NACK) → CS LOW → done
```

### Read Transaction (master to slave, slave responds with data)

```
Master: assert READY poll → CS HIGH → TX frame → release bus → wait D0 LOW
Slave:  RX frame → validate CRC → process → drive D0=0 D1=1 (ACK) → TX response
Master: read D1 (ACK) → RX response → CS LOW → done
```

### Flow Control

- **READY pin**: Slave drives LOW when armed and ready for a frame. Master polls before asserting CS.
- **D0/D1 in-band**: After frame TX, master releases bus and waits for D0 LOW (slave ready with response). D1 carries ACK (HIGH) or NACK (LOW). No dedicated ACK/NACK pins needed.
- **Retries**: Master retries up to 3 times on timeout or NACK.
- **Non-blocking RX**: Slave PIO uses `push noblock` to prevent stalls on FIFO overflow. Dropped data causes CRC failure and retry.

## Building

Requires the Raspberry Pi Pico SDK. Set `PICO_SDK_PATH` before building.

```bash
# Master
cd master
mkdir build && cd build
cmake ..
make

# Slave
cd slave
mkdir build && cd build
cmake ..
make
```

Flash `master.uf2` to one board and `slave.uf2` to the other. Connect via UART (115200 baud) to see output.

## License

MIT

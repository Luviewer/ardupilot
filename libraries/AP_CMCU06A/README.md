# CMCU-06A Modbus-RTU Protocol

This library implements the minimum serial protocol needed to use a CMCU-06A
load/strain transmitter from ArduPilot.

## Serial Settings

- Protocol: Modbus-RTU
- Default slave address: `1`
- Default UART: `9600 8N1`
- Recommended polling interval: at least `100 ms`

The device manual warns that RS485 is half duplex and that the automatic
transmit modes can collide with host commands. This driver only uses host
initiated polling.

## Implemented Registers

| Register | Access | Meaning |
| -------- | ------ | ------- |
| `0` | read | Final data, low word of signed 32-bit value |
| `1` | read | Final data, high word of signed 32-bit value |
| `21` | write | Trigger register, write `1` to tare |

The final data is a signed 32-bit integer. Register `0` is the low word and
register `1` is the high word:

```text
value = int32_t((reg1 << 16) | reg0)
```

Examples from the manual:

- `reg0 = 0x01F4`, `reg1 = 0x0000` gives `500`.
- `reg0 = 0xFFCE`, `reg1 = 0xFFFF` gives `-50`.

The physical unit depends on calibration. The transmitter returns an integer
scale count, not a fixed SI unit.

## Read Data

Read registers `0` and `1` with function code `0x03`.

```text
Request:  01 03 00 00 00 02 C4 0B
Response: 01 03 04 reg0_hi reg0_lo reg1_hi reg1_lo crc_lo crc_hi
```

The Modbus CRC16 is sent low byte first.

## Tare

Write `1` to register `21` with function code `0x06`.

```text
Request:  01 06 00 15 00 01 59 CE
Response: 01 06 00 15 00 01 59 CE
```

After tare, the manual recommends allowing roughly `600 ms` before querying
data again because the transmitter performs internal debounce and calculation.

## Not Implemented

This library intentionally does not implement:

- zero calibration
- weight/span calibration
- write-protect changes
- address or baud-rate changes
- cancel tare
- automatic transmit modes

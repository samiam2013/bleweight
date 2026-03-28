# bleweight

ESP32 BLE client for the Renpho ES-CS20M bathroom scale. Connects via Bluetooth Low Energy, reads weight measurements, and logs the final stable weight.

## Protocol

The scale uses the Lefu protocol on BLE service `0x1A10`:

- `0x2A10` (notify) - Weight data and status from scale
- `0x2A11` (write-no-rsp) - Commands to scale

Frame format: `55 AA [cmd] 00 [payload_len] [payload...] [checksum]`

### Handshake

1. Subscribe to notifications on `0x2A10`
2. Send user profile command (`0x97`) to `0x2A11`
3. Scale sends `0x11` (start), weight frames (`0x14`), then `0x11` (stop)

Weight is extracted from `0x14` frames: payload bytes 3-4 as uint16 big-endian / 100 = kg.

## Requirements

- ESP-IDF v6.0+
- ESP32 development board

## Build and Flash

```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

To exit the serial monitor, press `Ctrl-]`.

## License

AGPL-3.0 - See [LICENSE](LICENSE) for details.

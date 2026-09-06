# UAV GPS + LoRa firmware (STM32H753)

This is an independent experimental variant based on the GPS branch of
[`songjulim/30th-Drone`](https://github.com/songjulim/30th-Drone/tree/GPS).
It is published separately to preserve the original repository and show the
GPS + LoRa changes without modifying the upstream project.

This STM32H753 project combines the existing GPS branch GNSS receiver with the
SX1276/77/78/79 LoRa register configuration from `lora_drone`. It intentionally
does not add flight control, RTCM relay, autonomous waypoint execution, or
unsolicited periodic telemetry. It can receive and store waypoint coordinates,
but it does not fly them autonomously.

## Wiring used by this project

| Function | STM32H753 pin |
|---|---|
| GNSS USART2 TX / RX | PA2 / PA3 |
| LoRa SPI4 SCK / MISO / MOSI | PE2 / PE5 / PE6 |
| LoRa NSS (CS) | PE3 |
| LoRa DIO0 | PE4 (EXTI4) |
| LoRa RESET | PC13 |
| Debug UART | existing USART1 setup |

SPI4 is mode 0, 8-bit, prescaler 16 (7.5 Mbit/s). Radio settings are 922.1 MHz,
BW 125 kHz, coding rate 4/5, spreading factor 7, explicit header, payload CRC,
8-symbol preamble, sync word `0x12`, PA_BOOST 17 dBm, and OCP 140 mA.

## Ground protocol

Envelope: `[A4][SRC][DST][TYPE][SEQ_L][SEQ_H][LEN][PAYLOAD...]`

- GCS `0xFF`, UAV `0xFD`
- Poll `0x01`: GCS to UAV, empty payload
- Waypoint `0x02`: payload `[count][lat_i32_le][lon_i32_le]...`, max 15
- ACK `0x03`: payload `[0x02][status]`; status 0 is accepted
- UAV GPS `0x10`: payload `[lat_i32_le][lon_i32_le][valid_u8]`

The response echoes the request sequence. GPS is sent only after a valid UAV
poll. `valid` is 1 only for a fresh 2D-or-better GNSS fix; otherwise coordinates
are zero and `valid` is 0. After TxDone or timeout the radio immediately returns
to continuous RX. Packets addressed to other nodes are ignored.

The GPS response is queued for a non-blocking 10 ms RX/TX turnaround guard after
each poll. Exactly one response is transmitted for each accepted poll.

For backward compatibility the ground decoder should accept the former 8-byte
GPS payload as valid and the new 9-byte payload with the explicit validity byte.

## Bench test

1. Connect the LoRa module and GNSS receiver using the pin table above.
2. Build and flash the `Debug` configuration in STM32CubeIDE.
3. Open the existing USART1/ST-LINK serial terminal at 115200 baud.
4. At the OLED boot menu press **LEFT** for `GPS + LoRa TEST`. This mode does not
   initialize the motors, PID loop, or IMUs.
5. Confirm `[GPS]` updates and `[LORA] RX Continuous`.
6. Send a UAV poll from the GCS and verify matching RX sequence, GPS response,
   `TX done`, then return to RX.
7. Send UAV waypoints and verify each stored coordinate plus an ACK with the same
   sequence. Send a UGV-addressed packet and verify it is ignored.

Received packets include a bounded HEX dump plus RSSI/SNR. CRC, magic, destination,
type, and payload-length failures are logged from main context; the EXTI callback
only records the DIO0 event flag.

Shared-channel legacy UGV packets beginning with `A1`, `A2`, or `A3` are ignored
silently by the UAV and do not produce misleading protocol-length errors.

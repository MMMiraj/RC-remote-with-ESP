# ESP32 RC Transmitter/Receiver — NRF24L01

A 10-channel RC transmitter and receiver built on ESP32, using an
**NRF24L01(+)** radio module for the wireless link (via the RF24 library).

## Files

| File | Role |
|---|---|
| `RC_Transmitter_NRF24.ino` | Handheld TX: joysticks/switches/pots + LCD menu + NRF24 send |
| `RC_Receiver_NRF24.ino` | Onboard RX: NRF24 receive + PWM (10 servo/ESC outputs) or PPM (1 pin) |

## Hardware

**Transmitter**
- ESP32 dev board
- NRF24L01(+) radio module
- 16x2 LCD with PCF8574 I2C backpack
- 2x XY joystick (4 analog axes: LX, LY, RX, RY)
- 1x rotary encoder with push button (menu navigation + OK/select)
- 4x toggle/momentary switches
- 2x potentiometers

**Receiver**
- ESP32 dev board
- NRF24L01(+) radio module
- 10x servo/ESC signal outputs (PWM mode), or 1x PPM output pin (PPM mode)

## Required libraries

- `RF24` by TMRh20 (install via Arduino Library Manager: search "RF24")
- `LiquidCrystal_I2C` by Frank de Brabander (or "LiquidCrystal I2C")
- `SPI.h`, `Preferences.h` — bundled with the ESP32 Arduino core

## NRF24L01 wiring (both TX and RX, identical)

| NRF24L01 pin | ESP32 pin |
|---|---|
| VCC | **3.3V — not 5V.** Add a 100–470uF cap across VCC/GND right at the module; these modules are power-hungry and brown out easily without one. |
| GND | GND |
| CE | GPIO 17 |
| CSN | GPIO 5 |
| SCK | GPIO 18 (VSPI, fixed) |
| MOSI | GPIO 23 (VSPI, fixed) |
| MISO | GPIO 19 (VSPI, fixed) |
| IRQ | not connected |

## Other pinout (transmitter)

| Function | GPIO |
|---|---|
| LCD SDA / SCL | 21 / 22 |
| Left joystick X / Y | 34 / 35 |
| Right joystick X / Y | 32 / 33 |
| Pot 1 / Pot 2 | 36 / 39 |
| Encoder CLK / DT / BTN | 25 / 26 / 27 |
| Switch 1–4 | 13 / 14 / 15 / 4 |

## Other pinout (receiver)

| Function | GPIO |
|---|---|
| PWM channels 1–10 | 13, 12, 14, 27, 26, 25, 33, 32, 15, 2 |
| PPM output (PPM mode only) | 13 |

## Radio settings

Both sketches must agree on these — set at the top of each `.ino`:

```cpp
#define RADIO_CHANNEL   100          // 0-125; pick something away from busy WiFi
const byte RADIO_ADDR[6] = "RCTX1";  // 5-char pipe address
```

Unlike the ESP-NOW version, there's no MAC address to copy over — the NRF24
talks point-to-point using this address/channel pair, and
`radio.write()`/`radio.read()` handle delivery.

Defaults used in both sketches: `RF24_250KBPS` data rate, `RF24_PA_LOW`
power amp level, 5 retries with 15 attempts each. If range is poor, try
raising the PA level (`RF24_PA_HIGH` / `RF24_PA_MAX`) once wiring/power is
confirmed solid — high PA levels are the most common cause of brownouts on
cheap modules.

## Multiple transmitter/receiver pairs

The NRF24 link is unicast by nature, so this is simpler than the ESP-NOW
version:

- Give each TX/RX pair its own unique `RADIO_ADDR` (e.g. `"RCTX1"`,
  `"RCTX2"`, `"RCTX3"`) — both ends of a pair must match each other.
- Pairs can share the same `RADIO_CHANNEL`, since the address filters at the
  hardware level. If you're running several pairs close together, spreading
  them across a few channels (e.g. 76, 100, 110) reduces airtime collisions
  and retry latency.

## Transmitter menu

Navigate with the rotary encoder, select with the push button (OK).

- **Channel config** — assign a physical control (joystick axis, pot, or
  switch) to each of the 10 output channels. Move the control you want and
  it auto-assigns.
- **Output mode** — PWM (10 individual servo-style outputs) or PPM (single
  pin, all channels multiplexed).
- **RC mode** — starts transmitting at ~50Hz and shows link status
  (Connected / No link), based on real per-packet ACKs from the receiver.
- **Calibrate** — set the center/zero point for an assigned analog channel.
- **Test** — live readout of every joystick/pot/switch, useful for wiring
  checks before assigning channels.
- **Invert** — reverse the direction of any channel (servo reversing).
  Scroll to a channel, press OK to toggle `[ON]`/`[off]`; stays on the list
  so you can flip several channels in a row. Applied after calibration, so
  center/travel aren't affected — it just mirrors around center. Persists
  across reboots (stored in NVS along with the rest of the channel config).

All channel assignments, calibration, output mode, and invert settings are
saved to NVS (`Preferences`) and survive a reboot.

## Failsafe

If the receiver stops getting packets for 500ms, it falls back to safe
defaults — analog/PWM channels center, switch channels go OFF — until the
link comes back.

## Output mode

Set independently on **both** sketches — the receiver doesn't read the
transmitter's output-mode setting over the air, it uses its own
`OUTPUT_MODE_PPM` `#define`. If you change output mode on the TX menu, set
the matching value in `RC_Receiver_NRF24.ino` and reflash the receiver.

```cpp
#define OUTPUT_MODE_PPM  0   // 0 = PWM (10 pins), 1 = PPM (1 pin)
```

## Receive path note

The RF24 IRQ pin is left unconnected in this build, so the receiver polls
`radio.available()` once per `loop()` iteration instead of using an
interrupt-driven callback. This is functionally equivalent at the ~50Hz
packet rate this project runs at.

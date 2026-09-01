# ESP32 RC Transmitter/Receiver — ESP-NOW

A 10-channel RC transmitter and receiver built on ESP32, using **ESP-NOW** as
the wireless link. No external radio module required — this is meant for
bench testing / quick setups where you just want two ESP32 boards talking to
each other with zero extra hardware.

## Files

| File | Role |
|---|---|
| `RC_Transmitter_ESPNOW.ino` | Handheld TX: joysticks/switches/pots + LCD menu + ESP-NOW send |
| `RC_Receiver_ESPNOW.ino` | Onboard RX: ESP-NOW receive + PWM (10 servo/ESC outputs) or PPM (1 pin) |

## Hardware

**Transmitter**
- ESP32 dev board
- 16x2 LCD with PCF8574 I2C backpack
- 2x XY joystick (4 analog axes: LX, LY, RX, RY)
- 1x rotary encoder with push button (menu navigation + OK/select)
- 4x toggle/momentary switches
- 2x potentiometers

**Receiver**
- ESP32 dev board
- 10x servo/ESC signal outputs (PWM mode), or 1x PPM output pin (PPM mode)

## Required libraries

- `LiquidCrystal_I2C` by Frank de Brabander (or "LiquidCrystal I2C")
- `WiFi.h`, `esp_now.h`, `esp_wifi.h`, `Preferences.h` — all bundled with the
  ESP32 Arduino core, no separate install needed

## Pinout (transmitter)

| Function | GPIO |
|---|---|
| LCD SDA / SCL | 21 / 22 |
| Left joystick X / Y | 34 / 35 |
| Right joystick X / Y | 32 / 33 |
| Pot 1 / Pot 2 | 36 / 39 |
| Encoder CLK / DT / BTN | 25 / 26 / 27 |
| Switch 1–4 | 13 / 14 / 15 / 4 |

## Pinout (receiver)

| Function | GPIO |
|---|---|
| PWM channels 1–10 | 13, 12, 14, 27, 26, 25, 33, 32, 15, 2 |
| PPM output (PPM mode only) | 13 |

## How it works

**Broadcast mode by default.** `RECEIVER_MAC` in the transmitter sketch is
set to `FF:FF:FF:FF:FF:FF`. Any ESP32 running the matching receiver sketch
and listening on the same WiFi channel picks up the packets — no MAC address
setup required. Great for getting one TX/RX pair running quickly.

**Trade-off:** broadcast frames get no MAC-layer ACK from a specific peer,
so the TX's "Connected" status really just means "this TX is transmitting,"
not "a receiver definitely got it."

**Switching to unicast** (recommended once you're past initial bring-up, and
*required* if you'll ever run more than one TX/RX pair — see below):
1. Flash the receiver sketch, open Serial Monitor at 115200 baud, and copy
   the `Receiver MAC:` address it prints on boot.
2. Paste those 6 bytes into `RECEIVER_MAC` in the transmitter sketch.
3. `esp_now_send()` will now report real per-packet delivery success, and
   the TX's link status becomes meaningful.

Both sketches force a fixed WiFi channel (`1`) and disable modem sleep on
setup — mismatched channels or power-save mode are the most common reasons
ESP-NOW sends silently fail.

## Multiple transmitter/receiver pairs

Broadcast mode does **not** scale to multiple pairs — every receiver on the
same WiFi channel will hear every transmitter's packets, and they'll collide
in the mix. For more than one pair:

1. Flash each receiver, note its MAC from Serial Monitor.
2. Set each transmitter's `RECEIVER_MAC` to its *own* receiver's MAC
   (unicast, not broadcast).
3. All devices still need to sit on the same WiFi channel (a receiver only
   listens on one channel), so that part of `espNowSetup()` doesn't change.

## Transmitter menu

Navigate with the rotary encoder, select with the push button (OK).

- **Channel config** — assign a physical control (joystick axis, pot, or
  switch) to each of the 10 output channels. Move the control you want and
  it auto-assigns.
- **Output mode** — PWM (10 individual servo-style outputs) or PPM (single
  pin, all channels multiplexed).
- **RC mode** — starts transmitting at ~50Hz and shows link status
  (Connected / No link).
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
the matching value in `RC_Receiver_ESPNOW.ino` and reflash the receiver.

```cpp
#define OUTPUT_MODE_PPM  0   // 0 = PWM (10 pins), 1 = PPM (1 pin)
```

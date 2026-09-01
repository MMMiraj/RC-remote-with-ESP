/*
  ============================================================================
  ESP32 RC RECEIVER — NRF24L01 version (RF24 library)
  10 channel output (PWM or PPM)
  ============================================================================
  Hardware:
    - ESP32 dev board
    - NRF24L01(+) radio module
    - 10x servo/ESC signal outputs (PWM mode) OR 1x PPM output pin (PPM mode)

  Required libraries: RF24 by TMRh20, SPI.h (bundled with ESP32 core).

  ----------------------------------------------------------------------------
  NRF24L01 WIRING (default VSPI pins on most ESP32 dev boards)
  ----------------------------------------------------------------------------
    NRF24L01 pin   ESP32 pin
    VCC            3.3V  (NOT 5V — power with a 100-470uF cap across
                   VCC/GND right at the module to avoid brownout resets)
    GND            GND
    CE             GPIO 17
    CSN            GPIO 5
    SCK            GPIO 18   (VSPI SCK, fixed)
    MOSI           GPIO 23   (VSPI MOSI, fixed)
    MISO           GPIO 19   (VSPI MISO, fixed)
    IRQ            not connected (unused)

  RADIO_ADDR and RADIO_CHANNEL below must match the transmitter sketch
  exactly, or no packets will arrive. setDataRate/setPALevel should also
  match — mismatched settings between TX and RX are the most common reason
  an NRF24 link silently fails.

  The output mode (PWM vs PPM) is chosen on the TRANSMITTER's "Output mode"
  menu, but this receiver doesn't parse that byte at runtime yet — it uses
  the OUTPUT_MODE_PPM #define below instead. Set it to match whatever you
  picked on the TX, then reflash the receiver.
  ============================================================================
*/
#include <SPI.h>
#include <RF24.h>

// Set this to match the transmitter's Output Mode selection
#define OUTPUT_MODE_PPM  0   // 0 = PWM (10 individual pins), 1 = PPM (1 pin)

// ---------- Protocol (must match transmitter) ----------
#define NUM_CHANNELS   10
#define CH_MIN         1000
#define CH_MID         1500
#define CH_MAX         2000

struct RCPacket {
  uint16_t channels[NUM_CHANNELS];
  uint16_t digitalMask;  // bit i set = channel i is a switch -> drive plain
                          // HIGH/LOW instead of a variable-width PWM pulse
  uint8_t  seq;
};
RCPacket packet;

// ---------- NRF24L01 ----------
#define RF24_CE_PIN     17
#define RF24_CSN_PIN    5
#define RADIO_CHANNEL   100   // must match transmitter exactly
const byte RADIO_ADDR[6] = "RCTX1";  // must match transmitter's RADIO_ADDR exactly

RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);

// ---------- PWM output pins (10) ----------
const uint8_t pwmPins[NUM_CHANNELS] = {13, 12, 14, 27, 26, 25, 33, 32, 15, 4};
#define PWM_FREQ_HZ   50
#define PWM_RES_BITS  16   // 50Hz @ 16-bit -> 1 tick = 1/(50*65536) s

// ---------- PPM output pin (1) ----------
#define PPM_PIN        13
#define PPM_FRAME_US   20000   // total frame length (20ms, standard 50Hz)
#define PPM_PULSE_US   400     // sync pulse width per channel

// ---------- link / failsafe ----------
uint16_t lastChannels[NUM_CHANNELS];
uint16_t lastDigitalMask = 0;   // which channels are switch-driven (persists across failsafe)
uint32_t lastPacketMillis = 0;
#define FAILSAFE_MS 500

void setFailsafeDefaults() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    bool isDigital = lastDigitalMask & (1 << i);
    // Switches fail safe to OFF; analog/PWM channels fail safe to center.
    lastChannels[i] = isDigital ? CH_MIN : CH_MID;
  }
}

// ============================================================================
// PWM (LEDC) output — core 3.x API
// ============================================================================
void pwmSetup() {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    ledcAttach(pwmPins[i], PWM_FREQ_HZ, PWM_RES_BITS);
  }
}
void pwmWriteAll() {
  uint32_t maxDuty = (1UL << PWM_RES_BITS) - 1;
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (lastDigitalMask & (1 << i)) {
      // Switch channel: drive a clean, constant HIGH or LOW — no pulsing.
      // (Duty 0% = pin held low; duty 100% = pin held high; the LEDC
      // peripheral just never toggles mid-cycle at either extreme, so
      // there's no PWM "flicker" for a relay/switch load to see.)
      uint32_t duty = (lastChannels[i] >= CH_MID) ? maxDuty : 0;
      ledcWrite(pwmPins[i], duty);
    } else {
      // Analog/regulator channel: normal variable-width servo-style pulse.
      uint32_t us = lastChannels[i];
      uint32_t duty = (uint64_t)us * maxDuty / 20000UL; // period = 20000us @ 50Hz
      ledcWrite(pwmPins[i], duty);
    }
  }
}

// ============================================================================
// PPM output (bit-banged using hardware timer for accurate pulse timing)
// core 3.x hw_timer API
// ============================================================================
hw_timer_t *ppmTimer = NULL;
volatile uint8_t ppmChIndex = 0;
volatile bool ppmPinState = false;

void IRAM_ATTR ppmISR() {
  uint32_t nextDelay;
  if (ppmPinState) {
    digitalWrite(PPM_PIN, LOW);
    ppmPinState = false;
    nextDelay = PPM_PULSE_US;
  } else {
    digitalWrite(PPM_PIN, HIGH);
    ppmPinState = true;
    if (ppmChIndex < NUM_CHANNELS) {
      uint32_t us = lastChannels[ppmChIndex];
      nextDelay = (us > PPM_PULSE_US) ? (us - PPM_PULSE_US) : PPM_PULSE_US;
      ppmChIndex++;
    } else {
      uint32_t used = 0;
      for (int i = 0; i < NUM_CHANNELS; i++) used += lastChannels[i];
      uint32_t gap = (PPM_FRAME_US > used) ? (PPM_FRAME_US - used) : 4000;
      nextDelay = gap;
      ppmChIndex = 0;
    }
  }
  timerAlarm(ppmTimer, nextDelay, false, 0);
}

void ppmSetup() {
  pinMode(PPM_PIN, OUTPUT);
  digitalWrite(PPM_PIN, LOW);
  ppmTimer = timerBegin(1000000);              // 1 MHz -> 1 tick = 1 us
  timerAttachInterrupt(ppmTimer, &ppmISR);
  timerAlarm(ppmTimer, PPM_PULSE_US, false, 0); // kick off the first one-shot
}

// ============================================================================
// NRF24L01 (RF24)
// ============================================================================
// RF24 has no receive interrupt/callback wired up here (IRQ pin left
// unconnected), so we poll radio.available() once per loop() instead of
// ESP-NOW's push-style callback. Functionally equivalent at 50Hz+ poll rates.
void radioCheck() {
  while (radio.available()) {
    radio.read(&packet, sizeof(packet));   // fixed payload size (set in radioSetup) — no length check needed
    lastDigitalMask = packet.digitalMask;
    for (int i = 0; i < NUM_CHANNELS; i++) {
      uint16_t v = packet.channels[i];
      if (v < CH_MIN) v = CH_MIN;
      if (v > CH_MAX) v = CH_MAX;
      lastChannels[i] = v;
    }
    lastPacketMillis = millis();

    static uint32_t rxCount = 0;
    rxCount++;
    if (rxCount % 50 == 0) { // print every ~1s at 50Hz so it doesn't flood the console
      Serial.print("RX: packets received = ");
      Serial.println(rxCount);
    }
  }
}

void radioSetup() {
  if (!radio.begin()) {
    Serial.println("NRF24 init failed");
    return;
  }

  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_250KBPS);   // must match transmitter
  radio.setPALevel(RF24_PA_LOW);     // must match transmitter's PA level to size the link budget
  radio.setPayloadSize(sizeof(packet));
  radio.openReadingPipe(1, RADIO_ADDR);
  radio.startListening();

  Serial.println("NRF24 radio ready");
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  setFailsafeDefaults();

  radioSetup();

#if OUTPUT_MODE_PPM
  ppmSetup();
#else
  pwmSetup();
#endif
}

void loop() {
  radioCheck();   // poll for incoming packets (no RX interrupt/callback wired up)

  // failsafe: center everything if link is lost
  if (millis() - lastPacketMillis > FAILSAFE_MS) {
    setFailsafeDefaults();
  }

#if !OUTPUT_MODE_PPM
  pwmWriteAll();   // PPM path is driven entirely by the hardware timer ISR
#endif

  delay(2);
}

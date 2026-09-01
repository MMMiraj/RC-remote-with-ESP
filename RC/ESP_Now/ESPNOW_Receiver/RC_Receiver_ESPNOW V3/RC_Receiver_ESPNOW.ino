/*
  ============================================================================
  ESP32 RC RECEIVER — ESP-NOW version (for bench testing, no NRF24L01)
  10 channel output (PWM or PPM)
  ============================================================================
  Hardware:
    - ESP32 dev board
    - 10x servo/ESC signal outputs (PWM mode) OR 1x PPM output pin (PPM mode)

  Required libraries: WiFi.h, esp_now.h — bundled with ESP32 core.

  IMPORTANT: Open Serial Monitor at 115200 baud right after flashing this —
  it prints "Receiver MAC: xx:xx:xx:xx:xx:xx" on boot. You only need that if
  you switch the transmitter from broadcast to unicast for a real per-packet
  delivery confirmation (see the TX sketch's notes). Broadcast mode (the
  default on both sketches) works without copying anything.

  The output mode (PWM vs PPM) is chosen on the TRANSMITTER's "Output mode"
  menu, but this receiver doesn't parse that byte at runtime yet — it uses
  the OUTPUT_MODE_PPM #define below instead. Set it to match whatever you
  picked on the TX, then reflash the receiver.
  ============================================================================
*/
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Set this to match the transmitter's Output Mode selection
#define OUTPUT_MODE_PPM  0   // 0 = PWM (10 individual pins), 1 = PPM (1 pin)

// ---------- Protocol (must match transmitter) ----------
#define NUM_CHANNELS   10
#define CH_MIN         1000
#define CH_MID         1500
#define CH_MAX         2000

struct RCPacket {
  uint16_t channels[NUM_CHANNELS];
  uint8_t  seq;
};
RCPacket packet;

// ---------- PWM output pins (10) ----------
const uint8_t pwmPins[NUM_CHANNELS] = {13, 12, 14, 27, 26, 25, 33, 32, 15, 2};
#define PWM_FREQ_HZ   50
#define PWM_RES_BITS  16   // 50Hz @ 16-bit -> 1 tick = 1/(50*65536) s

// ---------- PPM output pin (1) ----------
#define PPM_PIN        13
#define PPM_FRAME_US   20000   // total frame length (20ms, standard 50Hz)
#define PPM_PULSE_US   400     // sync pulse width per channel

// ---------- link / failsafe ----------
uint16_t lastChannels[NUM_CHANNELS];
uint32_t lastPacketMillis = 0;
#define FAILSAFE_MS 500

void setFailsafeDefaults() {
  for (int i = 0; i < NUM_CHANNELS; i++) lastChannels[i] = CH_MID; // center / safe
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
  for (int i = 0; i < NUM_CHANNELS; i++) {
    uint32_t us = lastChannels[i];
    uint32_t maxDuty = (1UL << PWM_RES_BITS) - 1;
    uint32_t duty = (uint64_t)us * maxDuty / 20000UL; // period = 20000us @ 50Hz
    ledcWrite(pwmPins[i], duty);
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
// ESP-NOW
// ============================================================================
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  if (len != sizeof(RCPacket)) return;
  memcpy(&packet, incomingData, sizeof(packet));
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

void espNowSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);   // drop any saved AP creds so we don't inherit a stale channel
  delay(100);

  // Must match the transmitter's fixed channel exactly.
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  setFailsafeDefaults();

  espNowSetup();

#if OUTPUT_MODE_PPM
  ppmSetup();
#else
  pwmSetup();
#endif
}

void loop() {
  // failsafe: center everything if link is lost
  if (millis() - lastPacketMillis > FAILSAFE_MS) {
    setFailsafeDefaults();
  }

#if !OUTPUT_MODE_PPM
  pwmWriteAll();   // PPM path is driven entirely by the hardware timer ISR
#endif

  delay(2);
}

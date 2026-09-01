/*
  ============================================================================
  RADIO LINK TEST - TRANSMITTER
  ----------------------------------------------------------------------------
  Bare-bones sketch to confirm your two ESP32 + NRF24L01 boards can talk to
  each other, BEFORE you wire up the LCD/encoder/joysticks/switches.

  Wiring (NRF24L01 -> ESP32):
    VCC  -> 3.3V   (NOT 5V - this kills the module)
    GND  -> GND
    CE   -> GPIO 4
    CSN  -> GPIO 5
    SCK  -> GPIO 18
    MOSI -> GPIO 23
    MISO -> GPIO 19
    IRQ  -> not connected

  TIP: NRF24L01 modules are notoriously power-hungry/noisy. If it won't
  connect, put a 10-100uF capacitor across the module's VCC and GND pins,
  and keep the wires short. Use the "+PA+LNA" module? Also needs its own
  clean 3.3V supply (not the ESP32's onboard regulator) if you have one.

  Flash THIS file to one board, RADIO_LINK_TEST_RX.ino to the other.
  Open Serial Monitor at 115200 baud on both.
  ============================================================================
*/
#include <SPI.h>
#include <RF24.h>

#define NRF_CE   4
#define NRF_CSN  5
#define RF_CHANNEL     76
#define RADIO_ADDRESS  "RCTX1"

RF24 radio(NRF_CE, NRF_CSN);
uint32_t counter = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!radio.begin()) {
    Serial.println("ERROR: NRF24 not detected! Check wiring/power.");
    while (1) delay(1000);
  }

  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(5, 15);
  radio.openWritingPipe((const uint8_t*)RADIO_ADDRESS);
  radio.stopListening();

  Serial.println("TX ready. Sending a counter every 500ms...");
}

void loop() {
  counter++;
  bool ok = radio.write(&counter, sizeof(counter));
  Serial.printf("Sent %lu -> %s\n", counter, ok ? "ACK received (link OK)" : "NO ACK (check RX side)");
  delay(500);
}

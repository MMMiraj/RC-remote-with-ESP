/*
  ============================================================================
  RADIO LINK TEST - RECEIVER
  ----------------------------------------------------------------------------
  Pairs with RADIO_LINK_TEST_TX.ino. Flash this to your receiver board.
  Wiring is identical to the TX test sketch - see comment there.

  Open Serial Monitor at 115200 baud. You should see an incrementing
  counter arrive roughly twice a second once the TX board is powered on.
  ============================================================================
*/
#include <SPI.h>
#include <RF24.h>

#define NRF_CE   4
#define NRF_CSN  5
#define RF_CHANNEL     76
#define RADIO_ADDRESS  "RCTX1"

RF24 radio(NRF_CE, NRF_CSN);

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
  radio.openReadingPipe(1, (const uint8_t*)RADIO_ADDRESS);
  radio.startListening();

  Serial.println("RX ready. Waiting for packets...");
}

void loop() {
  if (radio.available()) {
    uint32_t val;
    radio.read(&val, sizeof(val));
    Serial.printf("Received: %lu\n", val);
  }
}

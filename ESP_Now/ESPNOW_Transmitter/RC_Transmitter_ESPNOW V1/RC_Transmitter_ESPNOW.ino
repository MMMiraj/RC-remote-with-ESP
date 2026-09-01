/*
  ============================================================================
  ESP32 RC TRANSMITTER — ESP-NOW version (for bench testing, no NRF24L01)
  ============================================================================
  Hardware:
    - ESP32 dev board
    - 16x2 LCD with PCF8574 I2C backpack
    - 2x XY joystick (4 analog axes: LX, LY, RX, RY)
    - 1x rotary encoder with push button (menu navigation + OK/select)
    - 4x toggle/momentary switches
    - 2x potentiometers

  Required libraries:
    - LiquidCrystal_I2C  by Frank de Brabander (or "LiquidCrystal I2C")
    - WiFi.h, esp_now.h, Preferences.h — all bundled with ESP32 core

  ----------------------------------------------------------------------------
  ESP-NOW NOTES
  ----------------------------------------------------------------------------
  This defaults to BROADCAST mode: any ESP32 running the matching receiver
  sketch and listening on the same WiFi channel will pick up the packets,
  with zero MAC-address setup. Good for quick testing.

  Trade-off: broadcast frames get no MAC-layer ACK from a specific peer, so
  the "Connected" status here really means "this TX is transmitting", not
  "a receiver definitely got it". For a real link-quality indicator, switch
  to unicast:
    1) Flash the receiver sketch, open its Serial Monitor at 115200 baud,
       and copy the "Receiver MAC:" address it prints on boot.
    2) Paste those 6 bytes into RECEIVER_MAC below.
    3) esp_now_send() will then report real per-packet delivery success.

  Once you're happy with everything on ESP-NOW, swapping back to the
  NRF24L01/RF24 version only requires re-adding the SPI/RF24 include block
  and radioSetup()/radioSendChannels() — the rest of the menu/state-machine
  code is unchanged.
  ============================================================================
*/
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>

// ---------- I2C LCD ----------
#define LCD_ADDR   0x27          // run an I2C scanner if unsure
#define LCD_SDA    21
#define LCD_SCL    22

// ---------- Joysticks ----------
#define PIN_LX     34
#define PIN_LY     35
#define PIN_RX     32
#define PIN_RY     33

// ---------- Pots ----------
#define PIN_POT1   36
#define PIN_POT2   39

// ---------- Encoder ----------
#define ENC_CLK    25
#define ENC_DT     26
#define ENC_BTN    27

// ---------- Switches (wired to GND, use internal pull-up) ----------
#define PIN_SW1    13
#define PIN_SW2    14
#define PIN_SW3    15
#define PIN_SW4    4

// ============================================================================
// ESP-NOW PROTOCOL (must be identical on receiver)
// ============================================================================
#define NUM_CHANNELS   10
#define CH_MIN         1000
#define CH_MID         1500
#define CH_MAX         2000

struct RCPacket {
  uint16_t channels[NUM_CHANNELS];
  uint16_t digitalMask;  // bit i set = channel i is a switch -> RX drives plain
                          // HIGH/LOW instead of a variable-width PWM pulse
  uint8_t  seq;
};
RCPacket packet;

// Broadcast address = every ESP-NOW listener on this WiFi channel receives it.
// Replace with the receiver's real MAC (see notes above) to go unicast.
uint8_t RECEIVER_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

volatile bool lastSendDone = false;
volatile bool lastSendOk   = false;

// Core 3.x changed the send-callback signature: the first argument is now a
// wifi_tx_info_t* (carrying dest MAC + PHY info) instead of a raw MAC array.
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  lastSendOk   = (status == ESP_NOW_SEND_SUCCESS);
  lastSendDone = true;
  // Throttled debug print (every ~50 packets, ~1s at 50Hz) so it doesn't
  // spam the UART and add latency to every single send.
  static uint32_t sendCount = 0;
  sendCount++;
  if (sendCount % 50 == 0) {
    Serial.print("TX: sends ok=");
    Serial.println(lastSendOk);
  }
}

// ============================================================================
// PHYSICAL CONTROL SOURCES
// ============================================================================
enum SourceType : uint8_t {
  SRC_NONE = 0,
  SRC_LX, SRC_LY, SRC_RX, SRC_RY,
  SRC_POT1, SRC_POT2,
  SRC_SW1, SRC_SW2, SRC_SW3, SRC_SW4,
  SRC_COUNT
};

const char* sourceName(uint8_t s) {
  switch (s) {
    case SRC_LX: return "L-X Joy";
    case SRC_LY: return "L-Y Joy";
    case SRC_RX: return "R-X Joy";
    case SRC_RY: return "R-Y Joy";
    case SRC_POT1: return "Pot 1";
    case SRC_POT2: return "Pot 2";
    case SRC_SW1: return "Switch 1";
    case SRC_SW2: return "Switch 2";
    case SRC_SW3: return "Switch 3";
    case SRC_SW4: return "Switch 4";
    default: return "None";
  }
}
bool isAnalogSource(uint8_t s) { return s >= SRC_LX && s <= SRC_POT2; }
bool isDigitalSource(uint8_t s) { return s >= SRC_SW1 && s <= SRC_SW4; }

// live raw readings
int16_t rawAnalog[6];   // LX,LY,RX,RY,POT1,POT2  (index matches SRC_LX..SRC_POT2 - 1)
bool    rawDigital[4];  // SW1..SW4               (index matches SRC_SW1..SRC_SW4 - SRC_SW1)

// ============================================================================
// PERSISTED CONFIG (NVS)
// ============================================================================
struct TXConfig {
  uint8_t  channelSource[NUM_CHANNELS]; // SRC_* assigned to each output channel
  int16_t  center[NUM_CHANNELS];        // calibration raw center (analog sources only)
  uint8_t  outputMode;                  // 0 = PWM, 1 = PPM
  bool     invert[NUM_CHANNELS];        // true = reverse this channel's direction
};
TXConfig cfg;
Preferences prefs;

void loadConfig() {
  prefs.begin("rctx", false);
  size_t n = prefs.getBytes("cfg", &cfg, sizeof(cfg));
  if (n != sizeof(cfg)) {
    // first boot / corrupt -> sensible defaults
    for (int i = 0; i < NUM_CHANNELS; i++) {
      cfg.channelSource[i] = SRC_NONE;
      cfg.center[i] = 2048;
      cfg.invert[i] = false;
    }
    cfg.outputMode = 0;
    prefs.putBytes("cfg", &cfg, sizeof(cfg));
  }
}
void saveConfig() {
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
}

// ============================================================================
// LCD
// ============================================================================
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

void lcdTwoLines(const String &l0, const String &l1) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(l0.substring(0, 16));
  lcd.setCursor(0, 1); lcd.print(l1.substring(0, 16));
}

// Render a scrollable menu: shows current item on row0 with '>' and next item on row1
void lcdMenu(const char* title, const char* items[], uint8_t count, uint8_t sel) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(">");
  lcd.print(String(items[sel]).substring(0, 15));
  if (count > 1) {
    uint8_t nextIdx = (sel + 1) % count;
    lcd.setCursor(0, 1);
    lcd.print(" ");
    lcd.print(String(items[nextIdx]).substring(0, 15));
  }
}

// ============================================================================
// ENCODER (interrupt driven quadrature) + push button (debounced, edge-triggered)
// ============================================================================
volatile int encDelta = 0;
volatile uint8_t lastEncState = 0;

void IRAM_ATTR encoderISR() {
  uint8_t clk = digitalRead(ENC_CLK);
  uint8_t dt  = digitalRead(ENC_DT);
  uint8_t state = (clk << 1) | dt;
  static const int8_t table[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  uint8_t idx = (lastEncState << 2) | state;
  encDelta += table[idx & 0x0F];
  lastEncState = state;
}

int readEncoderStep() {
  // one detent = 4 ISR transitions typically; adjust divisor if too sensitive
  noInterrupts();
  int d = encDelta;
  encDelta = 0;
  interrupts();
  static int accum = 0;
  accum += d;
  int steps = accum / 4;
  accum -= steps * 4;
  return steps;
}

bool btnPressed() {           // true once per physical press (edge + debounce)
  static bool lastState = HIGH;
  static uint32_t lastChange = 0;
  bool state = digitalRead(ENC_BTN);
  bool pressedEvent = false;
  if (state != lastState && millis() - lastChange > 30) {
    lastChange = millis();
    lastState = state;
    if (state == LOW) pressedEvent = true; // active low
  }
  return pressedEvent;
}

// ============================================================================
// INPUT READING
// ============================================================================
void readInputs() {
  rawAnalog[SRC_LX - SRC_LX]   = analogRead(PIN_LX);
  rawAnalog[SRC_LY - SRC_LX]   = analogRead(PIN_LY);
  rawAnalog[SRC_RX - SRC_LX]   = analogRead(PIN_RX);
  rawAnalog[SRC_RY - SRC_LX]   = analogRead(PIN_RY);
  rawAnalog[SRC_POT1 - SRC_LX] = analogRead(PIN_POT1);
  rawAnalog[SRC_POT2 - SRC_LX] = analogRead(PIN_POT2);

  rawDigital[SRC_SW1 - SRC_SW1] = (digitalRead(PIN_SW1) == LOW);
  rawDigital[SRC_SW2 - SRC_SW1] = (digitalRead(PIN_SW2) == LOW);
  rawDigital[SRC_SW3 - SRC_SW1] = (digitalRead(PIN_SW3) == LOW);
  rawDigital[SRC_SW4 - SRC_SW1] = (digitalRead(PIN_SW4) == LOW);
}

int16_t getRawForSource(uint8_t src) {
  if (isAnalogSource(src)) return rawAnalog[src - SRC_LX];
  if (isDigitalSource(src)) return rawDigital[src - SRC_SW1] ? 4095 : 0;
  return 2048;
}

// Detect if a source has moved/changed "enough" to count as "operator touched it"
// used by Channel Config to auto-detect which control the user is moving.
bool sourceActivated(uint8_t src, int16_t baseline) {
  if (isAnalogSource(src)) {
    return abs(rawAnalog[src - SRC_LX] - baseline) > 600; // threshold
  } else {
    return rawDigital[src - SRC_SW1] != (baseline != 0);
  }
}

// Compute final channel value (1000-2000us) for one output channel
uint16_t computeChannelValue(uint8_t ch) {
  uint8_t src = cfg.channelSource[ch];
  if (src == SRC_NONE) return CH_MID;

  if (isDigitalSource(src)) {
    bool on = rawDigital[src - SRC_SW1];
    if (cfg.invert[ch]) on = !on;
    return on ? CH_MAX : CH_MIN;
  }

  int16_t raw = rawAnalog[src - SRC_LX];
  int16_t center = cfg.center[ch];
  int32_t offset = raw - center;
  if (cfg.invert[ch]) offset = -offset;
  // scale +-2048 (max possible deviation from center) to +-500us
  int32_t us = CH_MID + (offset * 500L) / 2048L;
  if (us < CH_MIN) us = CH_MIN;
  if (us > CH_MAX) us = CH_MAX;
  return (uint16_t)us;
}

// ============================================================================
// ESP-NOW
// ============================================================================
void espNowSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);   // drop any saved AP creds so we don't inherit a stale channel
  delay(100);

  // Force a known fixed channel and turn off power-save — mismatched channels
  // or modem sleep are the two most common reasons ESP-NOW sends silently fail.
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.print("Transmitter MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    lcdTwoLines("ESP-NOW init", "FAILED!");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = 0;      // 0 = use current WiFi channel
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    lcdTwoLines("ESP-NOW peer", "add failed!");
    delay(1500);
  }
}

// Sends current channel values. Returns true if the packet was queued/sent
// by the radio driver (not a guarantee of reception in broadcast mode).
bool radioSendChannels() {
  static uint8_t seq = 0;
  packet.digitalMask = 0;
  for (int i = 0; i < NUM_CHANNELS; i++) {
    packet.channels[i] = computeChannelValue(i);
    if (isDigitalSource(cfg.channelSource[i])) packet.digitalMask |= (1 << i);
  }
  packet.seq = seq++;
  lastSendDone = false;
  esp_err_t result = esp_now_send(RECEIVER_MAC, (uint8_t*)&packet, sizeof(packet));
  return (result == ESP_OK);
}

// ============================================================================
// SCREEN / STATE MACHINE
// ============================================================================
enum Screen {
  SCR_MAIN,
  SCR_CHCFG_LIST, SCR_CHCFG_WAIT,
  SCR_OUTMODE,
  SCR_RCMODE,
  SCR_CALIB_LIST, SCR_CALIB_WAIT,
  SCR_TEST,
  SCR_INVERT_LIST
};
Screen screen = SCR_MAIN;

const char* mainItems[]  = {"Channel config", "Output mode", "RC mode", "Calibrate", "Test", "Invert"};
const char* chItems[]    = {"Channel 1","Channel 2","Channel 3","Channel 4","Channel 5",
                             "Channel 6","Channel 7","Channel 8","Channel 9","Channel 10","Back"};
const char* outItems[]   = {"PWM", "PPM", "Back"};
const char* rcItems[]    = {"Connect", "Back"};
const char* testItems[]  = {"L-X joystick","L-Y joystick","R-X joystick","R-Y joystick",
                             "POT 1","POT 2","Switch 1","Switch 2","Switch 3","Switch 4","Back"};

uint8_t selMain = 0, selCh = 0, selOut = 0, selRc = 0, selCalib = 0, selTest = 0, selInvert = 0;
uint8_t pendingChannel = 0;      // which channel index is being assigned/calibrated
int16_t assignBaseline[SRC_COUNT];
bool rcConnected = false;
uint32_t lastTxMillis = 0;
uint32_t lastAckMillis = 0;

void enterChCfgList() { screen = SCR_CHCFG_LIST; selCh = 0; }
void enterChCfgWait(uint8_t ch) {
  screen = SCR_CHCFG_WAIT;
  pendingChannel = ch;
  for (uint8_t s = SRC_LX; s < SRC_COUNT; s++) assignBaseline[s] = getRawForSource(s);
  lcdTwoLines("Ch " + String(ch + 1) + ": move a", "control to assign");
}
void enterCalibList() { screen = SCR_CALIB_LIST; selCalib = 0; }
void enterCalibWait(uint8_t ch) {
  screen = SCR_CALIB_WAIT;
  pendingChannel = ch;
  uint8_t src = cfg.channelSource[ch];
  if (src == SRC_NONE) { lcdTwoLines("Ch " + String(ch+1) + ": no", "control assigned"); delay(1000); enterCalibList(); return; }
  lcdTwoLines("Ch " + String(ch + 1) + ": set pos,", "press OK to zero");
}

// Builds "Ch<n> [ON]/[off]" labels on the fly (buffer must stay alive across
// the lcdMenu() call that consumes it, hence static).
void drawInvertList() {
  static char buf[11][17];
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    snprintf(buf[i], sizeof(buf[i]), "Ch%d [%s]", i + 1, cfg.invert[i] ? "ON" : "off");
  }
  strcpy(buf[10], "Back");
  const char* items[11];
  for (uint8_t i = 0; i < 11; i++) items[i] = buf[i];
  lcdMenu("Invert", items, 11, selInvert);
}
void enterInvertList() { screen = SCR_INVERT_LIST; selInvert = 0; drawInvertList(); }

void handleMain() {
  int step = readEncoderStep();
  if (step != 0) {
    selMain = (selMain + (step > 0 ? 1 : 5)) % 6; // wrap safely (5 == -1 mod 6)
    lcdMenu("Main", mainItems, 6, selMain);
  }
  if (btnPressed()) {
    switch (selMain) {
      case 0: enterChCfgList(); break;
      case 1: screen = SCR_OUTMODE; selOut = cfg.outputMode; break;
      case 2: screen = SCR_RCMODE; selRc = 0; rcConnected = false; break;
      case 3: enterCalibList(); break;
      case 4: screen = SCR_TEST; selTest = 0; break;
      case 5: enterInvertList(); break;
    }
  }
}

void handleChCfgList() {
  int step = readEncoderStep();
  if (step != 0) {
    selCh = (selCh + (step > 0 ? 1 : 10)) % 11;
    lcdMenu("Channel cfg", chItems, 11, selCh);
  }
  if (btnPressed()) {
    if (selCh == 10) { screen = SCR_MAIN; lcdMenu("Main", mainItems, 6, selMain); }
    else enterChCfgWait(selCh);
  }
}

void handleChCfgWait() {
  // scan all sources for activity vs baseline; first one that moves gets assigned
  for (uint8_t s = SRC_LX; s < SRC_COUNT; s++) {
    if (sourceActivated(s, assignBaseline[s])) {
      cfg.channelSource[pendingChannel] = s;
      cfg.center[pendingChannel] = (isAnalogSource(s)) ? rawAnalog[s - SRC_LX] : 2048;
      saveConfig();
      lcdTwoLines("Ch " + String(pendingChannel + 1) + " assigned:", sourceName(s));
      delay(1200);
      enterChCfgList();
      return;
    }
  }
  if (btnPressed()) { enterChCfgList(); } // press OK with no movement = cancel
}

void handleOutMode() {
  int step = readEncoderStep();
  if (step != 0) {
    selOut = (selOut + (step > 0 ? 1 : 2)) % 3;
  }
  String l0 = String(selOut == 0 ? ">" : " ") + "[" + (cfg.outputMode == 0 ? "X" : " ") + "] PWM";
  String l1;
  if (selOut == 0) l1 = String(" [") + (cfg.outputMode == 1 ? "X" : " ") + "] PPM";
  else if (selOut == 1) l0 = String(">[") + (cfg.outputMode == 1 ? "X" : " ") + "] PPM", l1 = " Back";
  else l0 = " [" + String(cfg.outputMode == 1 ? "X" : " ") + "] PPM", l1 = ">Back";
  lcdTwoLines(l0, l1);
  if (btnPressed()) {
    if (selOut == 0) { cfg.outputMode = 0; saveConfig(); }
    else if (selOut == 1) { cfg.outputMode = 1; saveConfig(); }
    else { screen = SCR_MAIN; lcdMenu("Main", mainItems, 6, selMain); }
  }
}

void handleRcMode() {
  int step = readEncoderStep();
  if (step != 0 && !rcConnected) selRc = (selRc + 1) % 2; // only 2 items, any turn toggles
  if (!rcConnected) {
    lcdMenu("RC mode", rcItems, 2, selRc);
    if (btnPressed()) {
      if (selRc == 0) { rcConnected = true; lcdTwoLines("Connecting...", ""); }
      else { screen = SCR_MAIN; lcdMenu("Main", mainItems, 6, selMain); }
    }
  } else {
    // actively transmitting
    if (millis() - lastTxMillis >= 20) {   // ~50Hz
      lastTxMillis = millis();
      readInputs();
      radioSendChannels();
    }
    if (lastSendDone) {
      if (lastSendOk) lastAckMillis = millis();
      lastSendDone = false;
    }
    bool linked = (millis() - lastAckMillis) < 500;

    // Only touch the LCD a few times a second, and only redraw when the
    // status text actually changes — clearing/reprinting every 5ms was
    // hammering the I2C bus and could make the status look stuck/flickery.
    static bool lastLinked = false;
    static uint32_t lastLcdUpdate = 0;
    if ((linked != lastLinked) || (millis() - lastLcdUpdate > 200)) {
      lastLinked = linked;
      lastLcdUpdate = millis();
      lcdTwoLines(linked ? "Status: Connected" : "Status: No link", "Press OK = stop");
    }
    if (btnPressed()) { rcConnected = false; screen = SCR_RCMODE; selRc = 0; }
  }
}

void handleCalibList() {
  int step = readEncoderStep();
  if (step != 0) {
    selCalib = (selCalib + (step > 0 ? 1 : 10)) % 11;
    lcdMenu("Calibrate", chItems, 11, selCalib);
  }
  if (btnPressed()) {
    if (selCalib == 10) { screen = SCR_MAIN; lcdMenu("Main", mainItems, 6, selMain); }
    else enterCalibWait(selCalib);
  }
}

void handleCalibWait() {
  if (btnPressed()) {
    uint8_t src = cfg.channelSource[pendingChannel];
    if (isAnalogSource(src)) {
      cfg.center[pendingChannel] = rawAnalog[src - SRC_LX];
      saveConfig();
    }
    lcdTwoLines("Ch " + String(pendingChannel + 1) + " zero set!", "");
    delay(800);
    enterCalibList();
  }
}

void handleTest() {
  int step = readEncoderStep();
  if (step != 0) selTest = (selTest + (step > 0 ? 1 : 10)) % 11;

  readInputs();
  String l0, l1;
  switch (selTest) {
    case 0: l0 = "L-X joystick"; l1 = String(rawAnalog[0]); break;
    case 1: l0 = "L-Y joystick"; l1 = String(rawAnalog[1]); break;
    case 2: l0 = "R-X joystick"; l1 = String(rawAnalog[2]); break;
    case 3: l0 = "R-Y joystick"; l1 = String(rawAnalog[3]); break;
    case 4: l0 = "POT 1";        l1 = String(rawAnalog[4]); break;
    case 5: l0 = "POT 2";        l1 = String(rawAnalog[5]); break;
    case 6: l0 = "Switch 1";     l1 = rawDigital[0] ? "ON" : "OFF"; break;
    case 7: l0 = "Switch 2";     l1 = rawDigital[1] ? "ON" : "OFF"; break;
    case 8: l0 = "Switch 3";     l1 = rawDigital[2] ? "ON" : "OFF"; break;
    case 9: l0 = "Switch 4";     l1 = rawDigital[3] ? "ON" : "OFF"; break;
    case 10: l0 = "Back"; l1 = "press OK"; break;
  }
  lcdTwoLines(l0, l1);
  if (btnPressed() && selTest == 10) { screen = SCR_MAIN; lcdMenu("Main", mainItems, 6, selMain); }
}

void handleInvertList() {
  int step = readEncoderStep();
  if (step != 0) {
    selInvert = (selInvert + (step > 0 ? 1 : 10)) % 11;
    drawInvertList();
  }
  if (btnPressed()) {
    if (selInvert == 10) { screen = SCR_MAIN; lcdMenu("Main", mainItems, 6, selMain); return; }
    cfg.invert[selInvert] = !cfg.invert[selInvert];
    saveConfig();
    drawInvertList(); // stay on the list so multiple channels can be toggled in a row
  }
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);

  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT, INPUT);
  pinMode(ENC_BTN, INPUT_PULLUP);
  pinMode(PIN_SW1, INPUT_PULLUP);
  pinMode(PIN_SW2, INPUT_PULLUP);
  pinMode(PIN_SW3, INPUT_PULLUP);
  pinMode(PIN_SW4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), encoderISR, CHANGE);

  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();
  lcd.backlight();
  lcdTwoLines("RC Transmitter", "booting...");

  loadConfig();
  espNowSetup();

  delay(500);
  lcdMenu("Main", mainItems, 6, selMain);
}

void loop() {
  readInputs();

  switch (screen) {
    case SCR_MAIN:        handleMain();       break;
    case SCR_CHCFG_LIST:  handleChCfgList();  break;
    case SCR_CHCFG_WAIT:  handleChCfgWait();  break;
    case SCR_OUTMODE:     handleOutMode();    break;
    case SCR_RCMODE:      handleRcMode();     break;
    case SCR_CALIB_LIST:  handleCalibList();  break;
    case SCR_CALIB_WAIT:  handleCalibWait();  break;
    case SCR_TEST:        handleTest();       break;
    case SCR_INVERT_LIST: handleInvertList(); break;
  }
  delay(5);
}

/*
  Modified 13 April 2026 by Dean Souleles, KK4DAS (with Claude Code assist)
  -- Removed fixed audio offset from band frequency table; offset is now
     calculated at transmit time as a random value within the 200 Hz WSPR
     audio window (1400-1594 Hz) so the beacon does not always transmit on
     the same frequency within the window.
  -- Added TX_PERCENT constant to control the percentage of available
     even-minute slots on which the beacon will transmit (default 50%).

  Modified 7 April 2026 by Dean Souleles, KK4DAS
  -- Changed timing logic to use timer interrupts vs blocking delays (code assist from Claude Code)
  -- Added retry logic to Wifi initialization
  -- Added 2 slow led blinks on wifi connect,  10 sets of 3 rapid blinks on wifi connect fail

  Modified 2 October 2024 by Bob Fontana AK3Y
  -- Added dialog boxes for Call, Grid Square and Power entries
  -- Added Double Reset Detector to configure ESP Reset button as "WiFi on Demand"
  Modified 20 September 2024 by Bob Fontana AK3Y  
  -- Added WiFiManager library to enable WiFi Provisioning

  *** TIMING & NTP FIXES (see change log below) ***
  -- Replaced delay(683) symbol loop with hardware Timer1 ISR for precise,
     jitter-free symbol timing immune to WiFi background task interruptions.
  -- Suspended WiFi during transmission to eliminate all interrupt-driven
     timing disruption; WiFi is reconnected automatically after TX ends.
  -- Fixed NTP staleness: micros() is latched the instant the NTP packet is
     received; elapsed time is added back to epoch before computing the wait
     interval, removing the ~2-second systematic late-start error.
  
  Acknowledgements
  
  WSPR Beacon Code Segments

 * Very simple WSPR beacon using NTP for time synchronisation and an Si5351 oscillator.
 * Created on a WeMos D1 R2 (ESP8266 on Arduino style board) by Peter Marks VK3TPM
 * Heavily based on work by Jason Milgram & Michael Margolis

  UDP NTP Client

  Get the time from a Network Time Protocol (NTP) time server
  Demonstrates use of UDP sendPacket and ReceivePacket
  For more on NTP time servers and the messages needed to communicate with them,
  see http://en.wikipedia.org/wiki/Network_Time_Protocol

  Created 4 Sep 2010 by Michael Margolis
  Modified 9 Apr 2012 by Tom Igoe
  Updated for the ESP8266 12 Apr 2015 by Ivan Grokhotkov

  This code is in the public domain.
*/

/*
  Library Dependencies

  Install the following libraries:
  	ESP_EEPROM (by j-watson)
    Etherkit JTEncode (by Jason Mildrum)
    Etherkit Si5351 (by Jason Mildrum)
    WiFiManager (by tablatronix, tzapu)
    DoubleResetDetector (by Stephen Denne)
  
  Add the following link to the "Additional Boards Manager" in File/Preferences
    http://arduino.esp8266.com/stable/package_esp8266com_index.json

*/

#include <ESP_EEPROM.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <si5351.h>
#include <JTEncode.h>
#include <Wire.h>
#include <DoubleResetDetector.h>
#include <cstdlib>

// int32_t cal_factor = 118300;         // si5351 crystal calibration offset - should be 0 with TCXO
int32_t cal_factor = 0;  

int LED_INTERNAL = 2;           // NodeMCU onboard LED
unsigned int localPort = 2390;  // local port to listen for UDP packets
bool shouldSaveConfig = false;

#define DRD_ADDRESS 0   // Double Reset Detector Parameters
#define DRD_TIMEOUT 10  // Number of seconds after reset during which a subsequent
                        // reset will be considered a double reset
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

IPAddress timeServerIP;
const char* ntpServerName = "time.nist.gov";  // NTP server address
const int NTP_PACKET_SIZE = 48;               // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];           // buffer to hold incoming and outgoing packets

#define TONE_SPACING 146  // ~1.46 Hz in hundredths of Hz (exact: 146.484375)
#define SYMBOL_COUNT WSPR_SYMBOL_COUNT

// ---------------------------------------------------------------------------
// Hardware Timer1 symbol timing
// ---------------------------------------------------------------------------
// ESP8266 Timer1 with TIM_DIV256 prescaler runs at 80 MHz / 256 = 312,500 Hz.
// WSPR symbol period = 8192 / 12000 s = 0.682666... s
// Required ticks = 0.682666... * 312,500 = 213,333.33  -> round to 213,333
// Resulting period = 213,333 / 312,500 = 0.682666 s  (error < 0.001 ms/symbol,
// cumulative drift over 162 symbols < 0.2 ms -- well within WSPR tolerance).
#define WSPR_TIMER_TICKS 213333UL

// ---------------------------------------------------------------------------
// WSPR audio window randomization
// The 200 Hz WSPR passband runs 1400-1600 Hz. The signal is ~6 Hz wide
// (4 tones x ~1.46 Hz), so the max safe lower-tone offset is 1594 Hz.
// ---------------------------------------------------------------------------
#define WSPR_OFFSET_MIN 1400UL
#define WSPR_OFFSET_MAX 1594UL

// ---------------------------------------------------------------------------
// Transmission duty cycle
// WSPR best practice is to skip some slots to reduce channel congestion.
// Set to a value 1-100 representing the percentage of even-minute slots
// on which the beacon will actually transmit.
// ---------------------------------------------------------------------------
#define TX_PERCENT 100

volatile bool symbol_ready = false;   // set by ISR; cleared by transmit loop

bool hopMode = false;
uint8_t hopBandIndex = 0;

// ISR must reside in IRAM on ESP8266 (ICACHE_RAM_ATTR)
void ICACHE_RAM_ATTR wspr_symbol_isr() {
  symbol_ready = true;
}
// ---------------------------------------------------------------------------

Si5351 si5351;
JTEncode jtencode;
WiFiUDP udp;

unsigned long offset = 1500UL;  // Default WSPR audio offset (Hz) - retained for reference

unsigned long freq2200 = 136000UL;    // 2200 meter band
unsigned long freq630  = 474200UL;    // 630 meter band
unsigned long freq160  = 1836600UL;   // 160 meter band
unsigned long freq80   = 3568600UL;   // 80 meter band
unsigned long freq60   = 5287200UL;   // 60 meter band
unsigned long freq40   = 7038600UL;   // 40 meter band
unsigned long freq30   = 10138700UL;  // 30 meter band
unsigned long freq20   = 14095600UL;  // 20 meter band
unsigned long freq17   = 18104600UL;  // 17 meter band
unsigned long freq15   = 21094600UL;  // 15 meter band
unsigned long freq12   = 24924600UL;  // 12 meter band
unsigned long freq10   = 28124600UL;  // 10 meter band

unsigned long freq[] = { freq80, freq40, freq30, freq20, freq17, freq15, freq12, freq10 };

uint8_t tx_buffer[SYMBOL_COUNT];

struct WSPRparams {
  char myCall[13];
  char myGrid[7];
  uint8_t mydBm;
} myWSPRparams;

void saveConfigCallback() {
  Serial.println("Should save configuration");
  shouldSaveConfig = true;
}

void configModeCallback(WiFiManager* myWiFiManager) {
  Serial.println("Entered configuration mode");
  Serial.println(WiFi.softAPIP());
  drd.stop();
}

void flashLED(int flashes, int onMs, int offMs) {
  for (int i = 0; i < flashes; i++) {
    digitalWrite(LED_INTERNAL, LOW);
    delay(onMs);
    digitalWrite(LED_INTERNAL, HIGH);
    delay(offMs);
  }
}

void si5351Test() {
  uint64_t target_freq = 1000000000ULL;  // 10 MHz in hundredths of Hz
  si5351.set_freq(target_freq, SI5351_CLK0);
  digitalWrite(LED_INTERNAL, LOW);
  si5351.set_clock_pwr(SI5351_CLK0, 1);
  delay(60000);
  digitalWrite(LED_INTERNAL, HIGH);
}

void setupSi5351() {
  bool i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, cal_factor);
  if (i2c_found == true) {
    Serial.println("i2c found!");
    si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
    si5351.set_clock_pwr(SI5351_CLK0, 0);
  } else {
    Serial.println("i2c NOT found!");
  }
}

int getFreqIndex() {
  return digitalRead(D5) + 2 * digitalRead(D6) + 4 * digitalRead(D7);
}

int getActiveBandIndex() {
  if (hopMode) {
    return hopBandIndex;
  }
  return getFreqIndex();
}

// ---------------------------------------------------------------------------
// transmitWSPR  --  hardware-timer-driven, WiFi-suspended symbol loop
// ---------------------------------------------------------------------------
// Strategy:
//   1. Encode the WSPR message into tx_buffer.
//   2. Suspend WiFi so no background interrupt can disturb I2C or timer.
//   3. Attach Timer1 ISR; for each symbol:
//        a. Set the Si5351 frequency for that symbol.
//        b. Load timer for one symbol period (WSPR_TIMER_TICKS).
//        c. Busy-wait until the ISR fires (symbol_ready flag).
//      The busy-wait is safe here because WiFi is suspended and the only
//      thing that needs the CPU is the lightweight timer ISR.
//   4. After all 162 symbols, shut down the clock and restore WiFi.
// ---------------------------------------------------------------------------
void transmitWSPR() {
  // --- Encode ---
  jtencode.wspr_encode(myWSPRparams.myCall, myWSPRparams.myGrid,
                       myWSPRparams.mydBm, tx_buffer);

  // Pick a random audio offset within the WSPR window each transmission
  unsigned long txOffset = random(WSPR_OFFSET_MIN, WSPR_OFFSET_MAX + 1);
  unsigned long txFreq = freq[getActiveBandIndex()] + txOffset;
  Serial.print("Band Index: ");
  Serial.println(getActiveBandIndex());
  Serial.print("Base Frequency: ");
  Serial.println(freq[getActiveBandIndex()]);
  Serial.print("TX audio offset Hz: ");
  Serial.println(txOffset);

  // --- Suspend WiFi to prevent interrupt-driven timing jitter ---
  WiFi.forceSleepBegin();
  delay(100);  // give WiFi stack time to quiesce before entering TX

  si5351.set_clock_pwr(SI5351_CLK0, 1);
  digitalWrite(LED_INTERNAL, LOW);

  // --- Arm Timer1 (single-shot mode; re-armed each iteration) ---
  timer1_attachInterrupt(wspr_symbol_isr);
  timer1_enable(TIM_DIV256, TIM_EDGE, TIM_SINGLE);

  for (uint8_t i = 0; i < SYMBOL_COUNT; i++) {
    // Set frequency BEFORE starting the timer so the full symbol period
    // is spent transmitting the correct tone.  I2C takes ~1 ms; because
    // the timer has not been loaded yet, this does not eat into symbol time.
    si5351.set_freq((txFreq * 100ULL) + (tx_buffer[i] * TONE_SPACING),
                    SI5351_CLK0);

    // Clear flag, then start the one-shot timer
    symbol_ready = false;
    timer1_write(WSPR_TIMER_TICKS);

    // Busy-wait for ISR.  WiFi is suspended so no other interrupt-driven
    // work will compete here.
    while (!symbol_ready) { /* intentional busy wait */ }
  }

  // --- End of transmission ---
  timer1_disable();
  timer1_detachInterrupt();

  si5351.set_clock_pwr(SI5351_CLK0, 0);
  digitalWrite(LED_INTERNAL, HIGH);

  // --- Restore WiFi ---
  WiFi.forceSleepWake();
  delay(100);

  // Reconnect using the credentials that WiFiManager saved previously.
  WiFi.begin();
  Serial.print("Reconnecting WiFi");
  uint8_t retries = 40;          // up to 20 seconds
  while (WiFi.status() != WL_CONNECTED && retries > 0) {
    delay(500);
    Serial.print(".");
    retries--;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi reconnected");
  } else {
    Serial.println("\nWiFi reconnect failed -- will retry next cycle");
  }
}

void getDatafromEEPROM() {
  EEPROM.get(0, myWSPRparams);
  Serial.println("Getting data from EEPROM");
  Serial.println(myWSPRparams.myCall);
  Serial.println(myWSPRparams.myGrid);
  Serial.println(myWSPRparams.mydBm);
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFiManager wm;

  Serial.print("WiFiManager version: ");
  Serial.println(WM_VERSION_STR);

  wm.setAPCallback(configModeCallback);
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setConfigPortalTimeout(180);
  wm.setConnectTimeout(10);
  wm.setConnectRetries(5);
  wm.setSaveConnectTimeout(15);

  WiFiManagerParameter CallSignBox("CallSignHtml",
    "Enter your Call Sign (max 12 characters)", "", 12);
  WiFiManagerParameter GridSquareBox("GridSquareHtml",
    "Enter your Grid Square (max 6 characters)", "", 6);
  WiFiManagerParameter PowerLevelBox("Powerhtml",
    "Enter your power in dBm (max 4 characters)", "", 4);
  wm.addParameter(&CallSignBox);
  wm.addParameter(&GridSquareBox);
  wm.addParameter(&PowerLevelBox);

  bool connected = false;

  if (drd.detectDoubleReset()) {
    Serial.println("Starting Configuration Portal");
    connected = wm.startConfigPortal("WSPRSetup");
  } else {
    Serial.println("Starting Auto Connect Process");
    shouldSaveConfig = false;
    connected = wm.autoConnect("WSPRSetup");
  }

  if (!connected) {
    Serial.println("WiFi connection FAILED - check SSID and password");
    for (int i = 0; i < 10; i++) {
      flashLED(3, 100, 100);
      delay(600);
    }
    ESP.restart();
  }

  Serial.println("WiFi connected successfully");
  flashLED(2, 500, 200);

  if (shouldSaveConfig) {
    char myChar[5];
    Serial.println("*** Storing Parameters into EEProm ***");
    strcpy(myWSPRparams.myCall, CallSignBox.getValue());
    strcpy(myWSPRparams.myGrid, GridSquareBox.getValue());
    strcpy(myChar, PowerLevelBox.getValue());
    myWSPRparams.mydBm = atoi(myChar);
    EEPROM.put(0, myWSPRparams);
    EEPROM.commit();
  }
}

void hw_wdt_disable() {
  *((volatile uint32_t*)0x60000900) &= ~(1);
}

void setup() {
  Serial.begin(9600);
  randomSeed(micros());  // seed RNG from free-running timer for random offset
  delay(1000);
  Serial.println("WSPR WiFi Setup");
  hw_wdt_disable();
  EEPROM.begin(sizeof(WSPRparams));
  pinMode(LED_INTERNAL, OUTPUT);
  pinMode(D5, INPUT_PULLUP);
  pinMode(D6, INPUT_PULLUP);
  pinMode(D7, INPUT_PULLUP);
  digitalWrite(LED_INTERNAL, HIGH);
  setupSi5351();

  //si5351Test();  // uncomment to verify calibration on 10 MHz

  setupWiFi();

  if (getFreqIndex() == 7) {
    hopMode = true;
    Serial.println("Band Hop Mode Enabled");
  }
}

// ---------------------------------------------------------------------------
// delay1  --  yield-friendly delay (used only while WiFi is active)
// ---------------------------------------------------------------------------
void delay1(unsigned long ms) {
  uint32_t start = micros();
  while (ms > 0) {
    yield();
    while (ms > 0 && (micros() - start) >= 1000) {
      ms--;
      start += 1000;
    }
  }
}

void sendNTPpacket(IPAddress& address) {
  Serial.println("Sending NTP packet...");
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0]  = 0b11100011;  // LI, Version, Mode
  packetBuffer[1]  = 0;
  packetBuffer[2]  = 6;
  packetBuffer[3]  = 0xEC;
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  udp.beginPacket(address, 123);
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------
void loop() {
  drd.loop();
  udp.begin(localPort);
  WiFi.hostByName(ntpServerName, timeServerIP);
  sendNTPpacket(timeServerIP);
  delay(2000);  // wait for NTP reply

  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("No NTP packet received");
    delay(1000);
  } else {
    // -------------------------------------------------------------------
    // NTP staleness fix:
    // Latch the local microsecond counter the instant the packet arrives.
    // After all epoch arithmetic is done, we add back the elapsed time so
    // that secondsToWait is computed from the TRUE current time, not the
    // time the NTP server stamped the packet.
    // -------------------------------------------------------------------
    uint32_t packet_rx_micros = micros();

    udp.read(packetBuffer, NTP_PACKET_SIZE);

    unsigned long highWord     = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord      = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;

    // Add the time that has elapsed since the packet was received.
    // micros() can wrap (every ~71 min) but the subtraction handles that
    // correctly for small deltas.
    uint32_t elapsed_us = micros() - packet_rx_micros;
    epoch += elapsed_us / 1000000UL;           // whole seconds
    uint32_t residual_us = elapsed_us % 1000000UL;  // leftover microseconds

    int minute = (epoch % 3600) / 60;
    int second  = epoch % 60;

    // How many whole seconds remain until the next even-minute boundary?
    int minutesToWait  = ((minute + 1) % 2);
    int secondsToWait  = (minutesToWait * 60) + (60 - second);

    // Subtract the sub-second residual so we start as close to the
    // exact boundary as possible (residual is typically < 50 ms).
    unsigned long waitMs = (unsigned long)secondsToWait * 1000UL;
    if (waitMs > residual_us / 1000UL) {
      waitMs -= residual_us / 1000UL;
    }

    Serial.print("Seconds to wait = ");
    Serial.println(secondsToWait);
    Serial.print("Residual us subtracted = ");
    Serial.println(residual_us);

    delay1(waitMs);   // yield-friendly wait; WiFi stays alive until TX

    getDatafromEEPROM();
    if (random(100) < TX_PERCENT) {
      Serial.println("WSPR TX start");
      transmitWSPR();   // suspends WiFi internally; restores it on return
      Serial.println("WSPR TX end");

      if (hopMode) {
        hopBandIndex = (hopBandIndex + 1) % 8;
      }
    } else {
      Serial.println("WSPR TX skipped (duty cycle)");
    }

    delay(10000);  // pause before next NTP query
  }
}

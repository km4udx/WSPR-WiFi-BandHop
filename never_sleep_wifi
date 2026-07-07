/*
 
  Maintainers:
    - km4udx (primary), Dean Souleles KK4DAS, Bob Fontana AK3Y, et al.

  Project repository:
    https://github.com/km4udx/WSPR-WiFi-BandHop

  License:
    Project code and this header are public-domain / permissive (follow the
    project's license). Adapt as needed.
*/


#define SUSPEND_WIFI_FOR_TX_DEFAULT 0     // 0 = keep WiFi on during TX (recommended)
#define DEFAULT_NTP_SERVER          "time.google.com"
#define DEFAULT_UDP_LOCAL_PORT      2390
#define DEFAULT_TX_PERCENT          100   // percentage of even-minute slots to transmit
#define DEFAULT_WSPR_OFFSET_MIN     1400UL
#define DEFAULT_WSPR_OFFSET_MAX     1594UL

/* Why these settings were chosen (summary)
   - SUSPEND_WIFI_FOR_TX_DEFAULT = 0:
       We keep WiFi active during WSPR TX by default because suspending the WiFi
       stack caused UDP/NTP socket rebind and reply loss after transmissions.
       With the hardware Timer1 ISR handling symbol timing in IRAM and the busy
       transmit loop preventing yields, symbol timing remains precise even with
       WiFi enabled. Keeping WiFi on improves NTP reliability and simplifies
       networking.
   - DEFAULT_NTP_SERVER = "time.google.com":
       Switched from pool.ntp.org to a stable provider to reduce intermittent
       failures from rotating pool members. A try-list or IP cache is recommended
       for maximum resiliency.
   - DEFAULT_UDP_LOCAL_PORT = 2390:
       The sketch binds a single UDP port once after WiFi connects. The code
       also tracks `udpBound` and rebinds after WiFi reconnection if needed.
*/

/* Change log — high-level (short)
   - 2026-07-05 (km4udx)
     * Fixed hopMode re-evaluation after EEPROM read
     * Improved WiFi reconnection behavior after TX
     * Automated 12-hour watchdog reboot refresh
   - 2026-06-13 (KK4DAS)
     * Portal-driven Band Hop (myHopMode in EEPROM) replaces old jumper-only hop
   - 2026-04-13
     * Randomized WSPR audio offset each TX (within 1400-1594 Hz window)
     * Added TX_PERCENT control
   - 2026-04-07
     * Replaced blocking symbol timing with Timer1 ISR; improved WiFi init retry
   - earlier
     * WiFiManager portal, EEPROM storage, and UI changes
*/


/*

*/


// Compile-time toggle: set to 1 to suspend WiFi during transmission (original behavior).
// Set to 0 to keep WiFi active during TX (recommended for stable NTP/UDP).
#define SUSPEND_WIFI_FOR_TX 0

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
unsigned int localPort = DEFAULT_UDP_LOCAL_PORT;  // local port to listen for UDP packets
bool shouldSaveConfig = false;

#define DRD_ADDRESS 0   // Double Reset Detector Parameters
#define DRD_TIMEOUT 10  // Number of seconds after reset during which a subsequent
                        // reset will be considered a double reset
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);

IPAddress timeServerIP;
const char* ntpServerName = DEFAULT_NTP_SERVER;  // NTP server address
const int NTP_PACKET_SIZE = 48;               // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];           // buffer to hold incoming and outgoing packets

#define TONE_SPACING 146  // ~1.46 Hz in hundredths of Hz (exact: 146.484375)
#define SYMBOL_COUNT WSPR_SYMBOL_COUNT

// ---------------------------------------------------------------------------
// Hardware Timer1 symbol timing
// ---------------------------------------------------------------------------
#define WSPR_TIMER_TICKS 213333UL

// ---------------------------------------------------------------------------
// WSPR audio window randomization
// ---------------------------------------------------------------------------
#define WSPR_OFFSET_MIN DEFAULT_WSPR_OFFSET_MIN
#define WSPR_OFFSET_MAX DEFAULT_WSPR_OFFSET_MAX

// ---------------------------------------------------------------------------
// Transmission duty cycle
// ---------------------------------------------------------------------------
#define TX_PERCENT DEFAULT_TX_PERCENT

volatile bool symbol_ready = false;   // set by ISR; cleared by transmit loop

bool hopMode = false;
uint8_t hopBandIndex = 0;

// Track whether we've successfully bound the UDP port
bool udpBound = false;

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
unsigned long freq40   = 7038600UL;  // 40 meter band
unsigned long freq30   = 10138700UL;  // 30 meter band
unsigned long freq20   = 14095600UL;  // 20 meter band
unsigned long freq17   = 18104600UL;  // 17 meter band
unsigned long freq15   = 21094600UL;  // 15 meter band
unsigned long freq12   = 24924600UL;  // 12 meter band
unsigned long freq10   = 28124600UL;  // 10 meter band

unsigned long freq[] = { freq80, freq40, freq30, freq20, freq17, freq15, freq12, freq10 };

uint8_t tx_buffer[SYMBOL_COUNT];

// ---------------------------------------------------------------------------
// EEPROM parameter struct
// ---------------------------------------------------------------------------
struct WSPRparams {
  char    myCall[13];
  char    myGrid[7];
  uint8_t mydBm;
  uint8_t myHopMode;   // 1 = band hop enabled, 0 = fixed band via jumpers
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

// ---------------------------------------------------------------------------
// getActiveBandIndex
// ---------------------------------------------------------------------------
int getActiveBandIndex() {
  if (hopMode) {
    return hopBandIndex;
  }
  return getFreqIndex();
}

// ---------------------------------------------------------------------------
// transmitWSPR
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

#if SUSPEND_WIFI_FOR_TX
  // --- Suspend WiFi to prevent interrupt-driven timing jitter ---
  WiFi.forceSleepBegin();
  delay(100);  // give WiFi stack time to quiesce before entering TX
#endif

  si5351.set_clock_pwr(SI5351_CLK0, 1);
  digitalWrite(LED_INTERNAL, LOW);

  // --- Arm Timer1 (single-shot mode; re-armed each iteration) ---
  timer1_attachInterrupt(wspr_symbol_isr);
  timer1_enable(TIM_DIV256, TIM_EDGE, TIM_SINGLE);

  for (uint8_t i = 0; i < SYMBOL_COUNT; i++) {
    si5351.set_freq((txFreq * 100ULL) + (tx_buffer[i] * TONE_SPACING),
                    SI5351_CLK0);

    symbol_ready = false;
    timer1_write(WSPR_TIMER_TICKS);

    while (!symbol_ready) { /* intentional busy wait */ }
  }

  // --- End of transmission ---
  timer1_disable();
  timer1_detachInterrupt();

  si5351.set_clock_pwr(SI5351_CLK0, 0);
  digitalWrite(LED_INTERNAL, HIGH);

#if SUSPEND_WIFI_FOR_TX
  // --- Restore WiFi ---
  WiFi.forceSleepWake();
  delay(100);

  WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str());
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

  // Re-bind UDP after WiFi reconnect (necessary after forceSleep)
  udp.stop(); // close previous socket mapping (safe even if not open)
  if (udp.begin(localPort) == 1) {
    Serial.println("UDP re-bound after TX");
    udpBound = true;
  } else {
    Serial.println("UDP re-bind failed after TX");
    udpBound = false;
  }
#endif
}

// ---------------------------------------------------------------------------

void getDatafromEEPROM() {
  EEPROM.get(0, myWSPRparams);
  Serial.println("Getting data from EEPROM");
  Serial.println(myWSPRparams.myCall);
  Serial.println(myWSPRparams.myGrid);
  Serial.println(myWSPRparams.mydBm);
  Serial.print("Hop Mode from EEPROM: ");
  Serial.println(myWSPRparams.myHopMode ? "Y" : "N");
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

  WiFiManagerParameter HopModeBox("HopModeHtml",
    "Band Hop Mode: Y = hop all bands, N = fixed band via jumpers", "N", 2);

  wm.addParameter(&CallSignBox);
  wm.addParameter(&GridSquareBox);
  wm.addParameter(&PowerLevelBox);
  wm.addParameter(&HopModeBox);

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

  // Bind UDP once after WiFi is up (avoid re-binding in loop)
  udp.stop();
  if (udp.begin(localPort) == 1) {
    Serial.print("UDP bound to port ");
    Serial.println(localPort);
    udpBound = true;
  } else {
    Serial.print("UDP bind reported non-1 result on port ");
    Serial.println(localPort);
    udpBound = false;
  }

  if (shouldSaveConfig) {
    char myChar[5];
    Serial.println("*** Storing Parameters into EEProm ***");

    strcpy(myWSPRparams.myCall, CallSignBox.getValue());
    strcpy(myWSPRparams.myGrid, GridSquareBox.getValue());

    strcpy(myChar, PowerLevelBox.getValue());
    myWSPRparams.mydBm = atoi(myChar);

    // Accept Y or y as hop mode enabled; anything else = fixed band
    const char* hopVal = HopModeBox.getValue();
    myWSPRparams.myHopMode = (hopVal[0] == 'Y' || hopVal[0] == 'y') ? 1 : 0;

    EEPROM.put(0, myWSPRparams);
    EEPROM.commit();
  }
}

void hw_wdt_disable() {
  *((volatile uint32_t*)0x60000900) &= ~(1);
}

void setup() {
  Serial.begin(9600);
  delay(500); // Give the serial port a moment to stabilize
  Serial.println("\n=============================================");
  Serial.println("!!! BEACON Power up or REBOOT EXECUTED / RESET COMPLETED !!!");
  Serial.println("=============================================\n");
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

  // Print runtime mode
  Serial.print("WiFi suspend during TX: ");
  Serial.println(SUSPEND_WIFI_FOR_TX ? "ENABLED" : "DISABLED");

  setupWiFi();

  hopMode = (myWSPRparams.myHopMode == 1);
  if (hopMode) {
    Serial.println("Band Hop Mode Enabled (via portal configuration)");
  } else {
    Serial.print("Fixed Band Mode - jumper index: ");
    Serial.println(getFreqIndex());
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

  // Ensure UDP port is bound before attempting NTP
  if (!udpBound) {
    Serial.println("UDP not bound, attempting to bind before NTP...");
    udp.stop();
    if (udp.begin(localPort) == 1) {
      Serial.println("UDP bind succeeded");
      udpBound = true;
    } else {
      Serial.println("UDP bind failed; will retry later");
      delay(1000);
      return;
    }
  }

  // Ensure WiFi is up before attempting DNS/UDP
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("WiFi not connected (status=");
    Serial.print(WiFi.status());
    Serial.println("). Waiting 5s...");
    delay(5000);
    return;
  }

  // Resolve NTP server name to IP (check return value)
  Serial.print("Resolving NTP server: ");
  Serial.println(ntpServerName);
  if (!WiFi.hostByName(ntpServerName, timeServerIP)) {
    Serial.println("hostByName FAILED - using fallback NTP IP (time.nist.gov example)");
    // fallback — test-only IP (replace if you want another)
    timeServerIP = IPAddress(129, 6, 15, 28); // example IP (time.nist.gov)
    Serial.print("Fallback NTP IP: ");
    Serial.println(timeServerIP);
  } else {
    Serial.print("Resolved NTP IP: ");
    Serial.println(timeServerIP);
  }

  // Build NTP packet in packetBuffer
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;
  packetBuffer[1] = 0;
  packetBuffer[2] = 6;
  packetBuffer[3] = 0xEC;
  packetBuffer[12] = 49;
  packetBuffer[13] = 49;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  Serial.print("Sending NTP packet to ");
  Serial.println(timeServerIP);
  udp.beginPacket(timeServerIP, 123);
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();

  // Wait for a reply (slightly longer timeout while debugging)
  uint32_t ntpTimer = millis();
  int packetSize = 0;
  while ((packetSize = udp.parsePacket()) == 0 && millis() - ntpTimer < 5000) {
    delay(50);
  }

  Serial.print("udp.parsePacket returned packetSize=");
  Serial.println(packetSize);

  if (packetSize == 0) {
    Serial.println("No NTP packet received");
    yield();
    delay(10000);
    return; // go back to top of loop and try again
  }

  // packet received -> process it
  uint32_t packet_rx_micros = micros();
  udp.read(packetBuffer, NTP_PACKET_SIZE); 

  unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
  unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
  unsigned long secsSince1900 = highWord << 16 | lowWord;

  const unsigned long seventyYears = 2208988800UL;
  unsigned long epoch = secsSince1900 - seventyYears;

  uint32_t elapsed_us = micros() - packet_rx_micros;
  epoch += elapsed_us / 1000000UL;
  uint32_t residual_us = elapsed_us % 1000000UL;

  int minute = (epoch % 3600) / 60;
  int second = epoch % 60;
  int currentHour = (epoch % 86400) / 3600;

  // Convert epoch to year/month/day correctly.
  unsigned long daysSince1970 = epoch / 86400UL;

  // Gregorian conversion (Howard Hinnant's algorithm)
  long z = (long)daysSince1970 + 719468L;
  long era = (z >= 0 ? z : z - 146096L) / 146097L;
  long doe = z - era * 146097L;           // [0, 146096]
  long yoe = (doe - doe/1460 + doe/36524 - doe/146096) / 365;   // [0,399]
  long y = yoe + era * 400;               // year within 400-year era
  long doy = doe - (365*yoe + yoe/4 - yoe/100);  // [0, 365]
  long mp = (5*doy + 2) / 153;            // [0, 11]
  int currentDay = (int)(doy - (153*mp + 2)/5 + 1);
  int currentMonth = (int)(mp + (mp < 10 ? 3 : -9));
  int currentYear = (int)(y + (currentMonth <= 2));

  int minutesToWait = ((minute + 1) % 2);
  int secondsToWait = (minutesToWait * 60) + (60 - second);

  unsigned long waitMs = (unsigned long)secondsToWait * 1000UL;
  if (waitMs > residual_us / 1000UL) {
    waitMs -= residual_us / 1000UL;
  }

  Serial.print("Seconds to wait = ");
  Serial.println(secondsToWait);
  Serial.print("Residual us subtracted = ");
  Serial.println(residual_us);

  delay(waitMs);

  getDatafromEEPROM();
  hopMode = (myWSPRparams.myHopMode == 1);
  
  if (!hopMode) {
    if (digitalRead(D3) == HIGH && digitalRead(D4) == HIGH && digitalRead(D5) == HIGH && digitalRead(D6) == HIGH && digitalRead(D7) == HIGH) {
      hopBandIndex = 7; 
      Serial.println("Fixed Band Mode - jumper index: 7 (10m - All Open)");
    } else {
      hopBandIndex = 0;
      if (digitalRead(D3) == LOW) hopBandIndex += 1;
      if (digitalRead(D4) == LOW) hopBandIndex += 2;
      if (digitalRead(D5) == LOW) hopBandIndex += 4;
      Serial.print("Fixed Band Mode - jumper index: ");
      Serial.println(hopBandIndex);
    }
  }

  if (random(100) < TX_PERCENT) {
    Serial.print("[");
    if (currentMonth < 10) Serial.print("0");
    Serial.print(currentMonth);
    Serial.print("/");
    if (currentDay < 10) Serial.print("0");
    Serial.print(currentDay);
    Serial.print("/");
    Serial.print(currentYear);
    Serial.print(" ");
    if (currentHour < 10) Serial.print("0");
    Serial.print(currentHour);
    Serial.print(":");
    if (minute < 10) Serial.print("0");
    Serial.print(minute);
    Serial.print(":");
    if (second < 10) Serial.print("0");
    Serial.print(second);
    Serial.print(" UTC] ");
    Serial.println("WSPR TX start");
    
    transmitWSPR();
    Serial.println("WSPR TX end");

    if (hopMode) {
      hopBandIndex = (hopBandIndex + 1) % 8;
    }
  } else {
    Serial.println("WSPR TX skipped (duty cycle)");
  }

  delay(10000);  

  if (millis() > 43200000UL) { 
    Serial.print("[");
    if (currentMonth < 10) Serial.print("0");
    Serial.print(currentMonth);
    Serial.print("/");
    if (currentDay < 10) Serial.print("0");
    Serial.print(currentDay);
    Serial.print("/");
    Serial.print(currentYear);
    Serial.print(" ");
    if (currentHour < 10) Serial.print("0");
    Serial.print(currentHour);
    Serial.print(":");
    if (minute < 10) Serial.print("0");
    Serial.print(minute);
    Serial.print(":");
    if (second < 10) Serial.print("0");
    Serial.print(second);
    Serial.print(" UTC] ");
    Serial.println("Performing scheduled every 12 hour refresh reboot...");
    
    delay(1000);
    ESP.restart(); 
  }
}

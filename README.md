# ESP8266 Multi-Band WSPR Beacon

An automated multi-band WSPR amateur radio beacon using an ESP8266 (WeMos D1 R2) microcontroller and an Etherkit Si5351 clock generator chip. Synchronizes timing with atomic accuracy via NTP internet time servers.

## Key Features & Modifications
* **100% TX Duty Cycle:** Configured to transmit continuously on every available even-minute sequence window.
* **WiFi-Controlled 8-Band Hopping:** Bypasses physical hardware jumper pins to cycle sequentially through all 8 configured amateur radio bands using the captive web setup portal.
* **Dynamic Connection Recovery:** Utilizes targeted credential extraction out of memory to snap the WiFi radio back online instantly post-transmission, resolving background channel scanning delays and locking down first-try NTP synchronization packets.
* **12-Hour Automated Watchdog Safety Refresh:** Features a scheduled twice-daily internal memory and network stack flush (`43200000UL` ms countdown) to permanently eliminate long-term network freezes and stack drift lockups.
* **Precise Jitter-Free Timing:** Operates via a dedicated hardware Timer1 interrupt service routine (ISR) to completely insulate transmission timing from concurrent network tasks.
* **Randomized Passband Scattering:** Audio offset frequency varies randomly within the 1400–1594 Hz WSPR passband on every cycle to distribute channel congestion safely.

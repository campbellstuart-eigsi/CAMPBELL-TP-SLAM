# ESP32 Wi-Fi as the robot transport: access point or station

Written 2026-11-04 for Phase B (`docs/specs/autonomy_phase_b.md`). The sources are Espressif's
official documentation (links at the end).

Each point is labelled:
- **[doc]**: stated in the documentation;
- **[reasoning]**: an engineering conclusion drawn from it;
- **[to verify]**: needs a measurement on our hardware or network.

Nothing here has been measured on our robot.

## 1. Findings
**Throughput**
- [doc] Espressif rates ESP32 Wi-Fi at about **30 Mbit/s UDP and 20 Mbit/s TCP over the air**
  (iperf, lab air test). Shielded-box figures are higher. Results depend strongly on the buffer
  and window configuration.
- [reasoning] One 456-point scan is about 2.3 kB, so continuous 10 Hz scanning needs about
  23 kB/s, or 0.19 Mbit/s: **under 1 %** of that rating. USB serial at 115200 baud gives about
  11.5 kB/s, which cannot carry 10 Hz scans. Wi-Fi is what makes continuous scanning (M4) possible.

**Latency and power save**
- [doc] The default power-save mode is `WIFI_PS_MIN_MODEM`. With modem sleep on, received data
  can be delayed by **up to one DTIM period** (minimum mode) or the listen interval (maximum
  mode).
- [doc] `WIFI_PS_NONE` gives the lowest receive latency, at a much higher current.
- [doc] With Wi-Fi/Bluetooth coexistence, modem sleep cannot be fully disabled.
- [reasoning] A DTIM period is usually around 100 ms or more (it depends on the AP), which is a
  third of our 300 ms heartbeat timeout. The robot must run with **power save disabled**
  (`WiFi.setSleep(false)`) and **Bluetooth off**.

**Access-point (SoftAP) mode**
- [doc] Current ESP-IDF allows up to 15 stations (Arduino default `max_connection` = 4).
- [reasoning] We need exactly one: the laptop. Set `max_connection = 1`, so a second device
  cannot even associate.

**Running both modes at once (APSTA)**
- [doc] The chip has one radio channel. In APSTA mode the soft-AP **follows the station's
  channel**, and it announces channel switches to its clients (CSA).
- [reasoning] AP + STA together makes our link's channel depend on the venue network. Don't
  combine them.

**Station-mode recovery**
- [doc] Reconnection is the application's job: call `esp_wifi_connect()` on
  `WIFI_EVENT_STA_DISCONNECTED`, but not after a deliberate disconnect. Read the disconnect
  reason.
- [doc] Retries can fail while a scan runs, so retry with a delay or backoff.
- [doc] By default, a disconnect clears the netif and TCP/UDP state; sockets should be closed and
  re-created.
- [doc] Arduino exposes `setAutoReconnect()` and `reconnect()`.

**Enterprise networks (eduroam)**
- [doc] On the Arduino core, WPA2-Enterprise works only through `WiFiMulti.addAP()`, and only
  when enterprise support is compiled in.
- [doc] EAP-TLS is not supported there, and no CA certificate can be given to validate a PEAP
  server.
- [doc] Enterprise authentication works in station mode only.
- [reasoning] A campus eduroam network is a poor fit: there is no server validation in Arduino,
  and campus networks commonly isolate clients from one another (an assumption; see
  [to verify]).

**ADC2**
- [doc] ADC2 is used by the Wi-Fi driver; ADC2 reads fail while Wi-Fi runs.
- [doc] The hardware design guidelines recommend ADC1 (GPIO 32-39).
- [reasoning] Any future battery or motor-current sensing must use ADC1 pins. GPIO32/33 are
  already taken by the LD06 (per the TP notes).

**ESP-NOW (alternative)**
- [doc] The default rate is 1 Mbit/s, and the measured one-to-one throughput is about
  **214 kbit/s** in the open.
- [doc] The payload is 250 B (v1.0) or 1470 B (v2.0, ESP-IDF ≥ 5.4).
- [doc] Espressif advises waiting for each send callback before sending the next packet.
- [reasoning] It would need a second ESP32 as a USB dongle on the laptop, which tethers the
  laptop instead. Its throughput is enough for stop-and-scan but leaves little margin.
  **Not recommended** as the primary transport.

**TCP or UDP**
- [reasoning] TCP retransmits lost segments and delivers in order, so one loss delays every later
  byte (head-of-line blocking). A STOP sent after a lost MOVE waits behind the MOVE's
  retransmission, and old heartbeats arrive in a burst after an outage.
- [reasoning] Our frames already carry a sequence number, CRCs and our own retry rules, so
  **UDP with one frame per datagram** fits better: every frame arrives on its own, late or never.
  The design must then handle loss, duplication, delay and reordering itself.

## 2. Comparison

| | **SoftAP (robot creates the network)** | **Station (robot joins an existing network)** |
|---|---|---|
| Works anywhere (classroom, demo room) | yes: no dependence on the venue | only where we have credentials and peer-to-peer traffic is allowed |
| Laptop internet while connected | lost on that adapter (unless a second adapter or Ethernet is used) | kept |
| Channel and interference | we pick the channel (it can be moved away from busy ones) | set by the venue AP; roaming is possible |
| Range | ESP32 antenna to laptop, direct | through the venue AP, which can be farther or nearer |
| Recovery after loss | the laptop re-associates; robot address fixed (default 192.168.4.1) | the robot must reconnect (event + backoff) and may get a new DHCP address; host needs discovery or a reservation |
| Security | WPA2-PSK password; `max_connection = 1` | the venue's security; other clients may reach the robot's port |
| Campus eduroam | n/a | weak in Arduino (no server-certificate validation); client isolation likely [to verify] |
| Debuggability | simple, self-contained | depends on network admins |

## 3. Recommendation
**SoftAP as the default for the robot.** It is self-contained and predictable: the robot address
is fixed, there's one client (`max_connection = 1`), and no venue network is involved.

Station mode stays a build-time option for a home or lab WPA2-PSK network, using the same UDP
protocol. Not eduroam.

Both modes use:
- UDP, one protocol frame per datagram, with a fixed port;
- `WiFi.setSleep(false)` and Bluetooth off;
- the firmware watchdog as the only safety mechanism. Wi-Fi events only drive reconnection; the
  firmware never relies on them to stop the motors.

## 4. To verify on hardware (each step needs your approval to flash)
1. **Round-trip latency** in SoftAP mode with power save off: median and 99th percentile at 1 m,
   5 m and through a wall. Target: 99th percentile well below the 300 ms heartbeat timeout.
2. **Datagram loss and burst length** at the same distances, including the time while the LD06
   streams. Check that a 5-chunk scan arrives complete.
3. **Recovery time** after the laptop leaves range and comes back, and after a robot reboot.
4. **Classroom network check**, if station mode is ever wanted there: is client-to-client UDP
   allowed (client isolation)?
5. **Current draw** with Wi-Fi TX and power save off, to inform the power-supply design in the
   motor hardware stage.

## Sources
- [ESP-IDF Wi-Fi API: soft-AP `max_connection`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_wifi.html)
- [ESP-IDF Wi-Fi driver overview: AP/STA channel, ESP-NOW keys](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi-driver/overview.html)
- [Arduino-ESP32 Wi-Fi API: softAP defaults, enterprise, reconnect](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/wifi.html)
- [ESP-IDF Wi-Fi performance and power save (throughput tables, PS modes)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi-driver/wifi-performance-and-power-save.html)
- [ESP-IDF v4.4.4 Wi-Fi driver guide: DTIM latency, enterprise, disconnect handling](https://docs.espressif.com/projects/esp-idf/en/v4.4.4/esp32/api-guides/wifi.html)
- [ESP-IDF station scenarios](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/wifi-driver/station-scenarios.html)
- [ESP-FAQ: Wi-Fi](https://docs.espressif.com/projects/esp-faq/en/latest/software-framework/wifi.html)
- [ESP-IDF v4.2 ADC: ADC2 and Wi-Fi](https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/adc.html)
- [ESP-FAQ: ADC](https://docs.espressif.com/projects/esp-faq/en/latest/software-framework/peripherals/adc.html)
- [ESP32 hardware design guidelines (PDF)](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32/esp-hardware-design-guidelines-en-master-esp32.pdf)
- [ESP-IDF ESP-NOW](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_now.html)
- [ESP-FAQ: ESP-NOW throughput](https://docs.espressif.com/projects/esp-faq/en/latest/application-solution/esp-now.html)

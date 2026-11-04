# Decision: Wi-Fi SoftAP + UDP is the robot transport; USB serial is the fallback

Date: 2026-11-04. Status: Approved by the user ("Approve SoftAP-by-default and the protocol v2
design").

**What:**
- The untethered robot uses an ESP32 SoftAP (one client, power save off) and UDP, with one
  protocol v2 frame per datagram.
- USB serial carries identical frames as a development fallback.
- Station mode is a build option for WPA2-PSK networks only, never eduroam.

**Why:**
- The robot must run without a USB cable.
- SoftAP is self-contained and has a fixed address.
- UDP avoids TCP head-of-line blocking of STOP.
- Wi-Fi throughput allows continuous scanning later.
- The original TP firmware already uses a SoftAP at 192.168.4.1.

**Alternatives:**
- Station mode on the venue network: it depends on the venue, and client isolation is likely.
- ESP-NOW: about 214 kbit/s, and it needs a second ESP32.
- TCP/HTTP, as in the original TP firmware: head-of-line blocking, and no watchdog.

**Details:** `docs/design/wifi_transport.md`, `docs/research/esp32_wifi_transport.md`.

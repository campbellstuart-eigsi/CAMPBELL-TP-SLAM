# ESP32SlamRobot dependencies (exact)

Checked by `compile_check.py` on 2026-11-04. Toolchain: Arduino CLI 1.5.1 (bundled with Arduino
IDE 2), board `esp32:esp32:esp32` (ESP32 Dev Module).

| Dependency | Version | Source | Licence | Install |
|---|---|---|---|---|
| esp32 board package (Espressif) | **3.3.12** | Boards Manager | LGPL-2.1 (core) | Boards Manager |
| Async TCP (ESP32Async) | **3.5.0** | Library Manager; `Async_TCP-3.5.0.zip` | LGPL-3.0 | Library Manager |
| ESP Async WebServer (ESP32Async) | **3.12.1** | Library Manager; `ESP_Async_WebServer-3.12.1.zip` | LGPL-3.0 | Library Manager |
| MecanumCar_v2 | (none stated) | course file `MecanumCar_v2.zip` | **none**: not redistributed | Add .ZIP Library |
| LD06 library (Robot-Maker), 3 files | 1.0.0 | `SLAM\LD06.zip`, `LD06/src/` | README says MIT; no LICENSE file: not redistributed | copy into the sketch folder |

## SHA-256
| File | SHA-256 |
|---|---|
| `LD06.zip` | `32f928be379d9f8136d94718d61baf0b423206d8e1989998e0e93eb79fc7b529` |
| `LD06/src/ld06.h` | `c9110d9fb5b40cb0187be431b1375dbdf5139f60affa11a0ba57d131bf571232` |
| `LD06/src/ld06.cpp` | `d7aa8f062e13875c273b6bf3f0f1a14feaff822c07ec733b473a0d0d97d20561` |
| `LD06/src/ld06crc.h` | `b7f447cbd2ea17f4b2ccc13b7011cee16006d29ba522610b7ba60812f69f3d15` |
| `MecanumCar_v2.zip` | `4b03d2cedb5a380cc7e26610b45e751ee51d8b4f0aa57c5c4b4f0f65426aaff0` |
| `MecanumCar_v2/MecanumCar_v2.h` | `20154750eb91a42b9701d4cae72452c7177541f4053a9e38059833ec898ba88a` |
| `MecanumCar_v2/MecanumCar_v2.cpp` | `daa57471342cd2f160d88d211f51eb465ccf563fb663ec6498babdf4b5ef6da1` |
| `Async_TCP-3.5.0.zip` (downloads.arduino.cc) | `0e7f17e62339ba6d820d34e4493824389c18d1b820cdb7ba4ec178454af6d741` |
| `ESP_Async_WebServer-3.12.1.zip` (downloads.arduino.cc) | `ff41fcf62fc0a848f1743bec88fbf95fb0915c10b265185481fb3ed0f379989e` |

Only one compiler warning comes from these files: an unused variable `reverse` in `ld06.cpp`
(harmless).

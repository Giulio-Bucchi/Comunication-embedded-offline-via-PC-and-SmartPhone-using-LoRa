
# AirChat: Mobile-to-Radio Bridge

**AirChat** is an offline communication bridge between iPhone/PC using LoRa and BLE.

PocketLoRa enables you to send text messages over long distances without the need for cellular or Wi-Fi networks. The system uses an **Arduino** as a gateway, receiving messages via **Bluetooth (BLE)** from an iPhone and relaying them via **LoRa** to an **STM32** microcontroller, and vice versa.

**Communication Flow:**

```
PC <-> STM32 <-> LoRa <-> Arduino <-> BLE <-> iPhone
```

## Features

- **Zero-Network:** Fully offline operation—no internet or cellular required.
- **Dual Interface:** Supports input from both Serial Monitor (PC) and Smartphone App.


## Hardware Connection Diagram

- STM32 <-> LoRa module
- Arduino <-> LoRa module and BLE module (AT-09)

## Development Notes

- The STM32 firmware is developed using **STM32CubeMX** and built with **CMake** for flexible and modern project management.

## Using with iPhone

1. Download a BLE terminal app (e.g., **LightBlue**).
2. Connect to the BLE module (usually identified as `BT05`).
3. Select **Characteristic 6 (FFE1)**.
4. Write a message: it will be instantly transmitted via LoRa to the STM32.


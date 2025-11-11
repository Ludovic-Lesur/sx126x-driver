# Description

This repository contains the **SX126x** RF transceiver driver.

# Dependencies

The driver relies on:

* An external `types.h` header file defining the **standard C types** of the targeted MCU.
* The **embedded utility functions** defined in the [embedded-utils](https://github.com/Ludovic-Lesur/embedded-utils) repository.

Here is the versions compatibility table:

| **sx126x-driver** | **embedded-utils** |
|:---:|:---:|
| [sw2.0](https://github.com/Ludovic-Lesur/sx126x-driver/releases/tag/sw2.0) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |
| [sw1.0](https://github.com/Ludovic-Lesur/sx126x-driver/releases/tag/sw1.0) | >= [sw5.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw5.0) |

# Compilation flags

| **Flag name** | **Value** | **Description** |
|:---:|:---:|:---:|
| `SX126X_DRIVER_DISABLE_FLAGS_FILE` | `defined` / `undefined` | Disable the `sx126x_driver_flags.h` header file inclusion when compilation flags are given in the project settings or by command line. |
| `SX126X_DRIVER_DISABLE` | `defined` / `undefined` | Disable the SX126X driver. |
| `SX126X_DRIVER_SPI_ERROR_BASE_LAST` | `<value>` | Last error base of the low level SPI driver. |
| `SX126X_DRIVER_DEVICE_SX1262` | `defined` / `undefined` | Select SX1262 chip if defined, SX1261 otherwise. |
| `SX126X_DRIVER_FXOSC_HZ` | `<value>` | Oscillator frequency in Hz. |
| `SX126X_DRIVER_TX_ENABLE` | `defined` / `undefined` | Enable radio transmission functions. |
| `SX126X_DRIVER_RX_ENABLE` | `defined` / `undefined` | Enable radio reception functions. |

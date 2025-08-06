# PN532 NFC/RFID Driver for ESP-IDF

This repository contains a C driver for the PN532 NFC/RFID reader, designed for use with ESP-IDF (Espressif IoT Development Framework) and FreeRTOS. It provides functionalities to interact with the PN532 module via SPI, enabling communication with various NFC/RFID tags, including MIFARE Classic and MIFARE Ultralight.

## Features

-   **PN532 Communication:** Supports basic communication with the PN532 module, including firmware version retrieval and GPIO control.
-   **NFC/RFID Tag Interaction:** Provides functions for reading and writing data to MIFARE Classic and MIFARE Ultralight tags.
-   **NDEF Support:** Includes definitions for NDEF URI prefixes, facilitating the handling of NDEF records.
-   **SPI Interface:** Implements manual SPI communication for controlling the PN532.
-   **FreeRTOS Compatibility:** Designed to work within a FreeRTOS environment, utilizing `vTaskDelay` for delays.

## PN532 Commands Supported

The driver defines a comprehensive set of PN532 commands, including:

-   `PN532_COMMAND_DIAGNOSE`
-   `PN532_COMMAND_GETFIRMWAREVERSION`
-   `PN532_COMMAND_GETGENERALSTATUS`
-   `PN532_COMMAND_READREGISTER`
-   `PN532_COMMAND_WRITEREGISTER`
-   `PN532_COMMAND_READGPIO`
-   `PN532_COMMAND_WRITEGPIO`
-   `PN532_COMMAND_SAMCONFIGURATION`
-   `PN532_COMMAND_INLISTPASSIVETARGET`
-   `PN532_COMMAND_INDATAEXCHANGE`
-   And many more for various NFC operations.

## MIFARE Commands Supported

Key MIFARE commands are also defined:

-   `MIFARE_CMD_AUTH_A`
-   `MIFARE_CMD_AUTH_B`
-   `MIFARE_CMD_READ`
-   `MIFARE_CMD_WRITE`
-   `MIFARE_ULTRALIGHT_CMD_WRITE`

## NDEF URI Prefixes

The driver includes a wide range of NDEF URI prefixes for common protocols and schemes, such as HTTP, HTTPS, TEL, MAILTO, FTP, and various URN types.

## Usage

To use this driver in your ESP-IDF project:

1.  Include the `pn532.h` header file in your source code.
2.  Initialize the PN532 driver using `pn532_spi_init()` with your desired SPI pins.
3.  Call `pn532_begin()` to initialize the PN532 module.
4.  Use the provided functions (e.g., `pn532_getFirmwareVersion()`, `pn532_readPassiveTargetID()`, `pn532_mifareclassic_ReadDataBlock()`) to interact with the PN532 and NFC/RFID tags.

## Debugging

Conditional compilation flags are available for debugging:

-   `PN532_DEBUG_EN`: Enable PN532 specific debug messages.
-   `MIFARE_DEBUG_EN`: Enable MIFARE specific debug messages.

To enable debugging, uncomment the respective `#define` lines in the source code.

## Dependencies

This driver relies on the following ESP-IDF components:

-   `freertos/FreeRTOS.h`
-   `freertos/task.h`
-   `freertos/queue.h`
-   `sdkconfig.h`
-   `esp_log.h`
-   `esp_log_internal.h`
-   `driver/gpio.h`



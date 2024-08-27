# LoRa SX1278 Driver and STM32 Example

## Overview

This repository contains a driver for the SX1278 LoRa module and an STM32 example project demonstrating its use. The driver includes `lora.h` and `lora.c` files, which provide functions for configuring and interfacing with the LoRa module. The STM32 project shows how to integrate this driver into a practical application.


## LoRa Driver

### `lora.h`

Header file defining registers, modes, and functions for interacting with the SX1278 module.

**Key Definitions:**
- Registers: `RegFifo`, `RegOpMode`, `RegPaConfig`, etc.
- Modes: `SleepMode`, `LoraMode`, `StdbyMode`, etc.
- Configuration: `Config1`, `Config2`, `Config3`, etc.

**Functions:**
- `void Lora_vInit();`  
  Initialize the LoRa module.

- `void Lora_vWriteReg(uint8_t regAdress, uint8_t data);`  
  Write data to a specified register.

- `uint8_t Lora_u8ReadReg(uint8_t regAdress);`  
  Read data from a specified register.

- `Lora_RxStates_ten Lora_tenRxMode(uint8_t* RxBuffer, uint8_t payloadLength);`  
  Configure and enter receive mode.

- `Lora_TxStates_ten Lora_tenTxMode(uint8_t* TxBuffer, uint8_t payloadLength);`  
  Configure and enter transmit mode.

### `lora.c`

Source file implementing the functions declared in `lora.h`, providing the necessary functionality to interact with the SX1278 module.

## STM32 Example Project

The STM32 example project demonstrates how to integrate the LoRa driver into a complete application. It includes configuration files, initialization routines, and an example of a ping pong application of sending and receiving data using the SX1278 module.

**Key Files:**
- `main.c`: Contains the main application code where the LoRa driver is used.

For any questions or issues, feel free to contact me at amara.merieem@gmail.com.

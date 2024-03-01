# ModusToolbox™ Software Training Level3 Bluetooth® Type1

This class will teach you how to use Bluetooth® Low Energy in ModusToolbox™ applications. The descriptions and exercises use a CYW20829 or a PSoC™ 6 MCU as a host to a CYW43012 device, but they apply to any AIROC™ Bluetooth® device with a host MCU or single-chip AIROC™ Bluetooth® device with dual MCUs.

A partial list of development kits that are covered by the material in this class are as follows:

- CYW920829M2EVK-02
- CY8CKIT-062S2-43012
- CY8CKIT-062-WIFI-BT
- CY8CKIT-064B0S2-4343W
- CY8CKIT-064S0S2-4343W
- CY8CPROTO-062-4343W
- CY8CPROTO-062S3-4343W
- CY8CPROTO-062S2-43439
- CY8CEVAL-062S2-LAI-4373M2
- CY8CEVAL-062S2-MUR-43439M2
- CYW9P62S1-43012EVB-01
- CYW9P62S1-43438EVB-01

*Note: some exercises may not work as-is on all of the kits listed above.*

After completing this class, you should be able to create and debug full Bluetooth® applications using the ModusToolbox™ ecosystem including peripherals, centrals, and beacons.

## Pre-requisites

- ModusToolbox™ Software Training Level 1 Getting Started
- ModusToolbox™ Software Training Level 2 PSoC™ MCUs

## Organization

- *Manual*:    This directory contains the manual chapters.
- *Projects*:  This directory contains solutions to exercises.
- *Templates*: This directory contains template starter projects for some exercises.

## Manual Chapters

The manual consists of the following chapters:

- Chapter 1: Introduction - Overall introduction to the class
- Chapter 2: Bluetooth® Protocol - General description of Bluetooth® terms and protocols
- Chapter 3: Bluetooth® LE Basic Peripheral - How to create a basic Bluetooth® peripheral using the ModusToolbox™ ecosystem
- Chapter 4: Notify and Indicate - How to implement notifications and indications in a peripheral
- Chapter 5: Pairing, Bonding & Privacy - How to implement various types of security in Bluetooth®
- Chapter 6: BLE Centrals - How to implement a BLE central device
- Chapter 7: Beacons - How to implement different types of Bluetooth® beacons
- Chapter 8: Low Power - How to reduce system power consumption in a Bluetooth® design
- Chapter 9: Over the Air Update (OTA) - How to update firmware over a Bluetooth® link
- Chapter 10: Debugging - Debugging techniques and tools for Bluetooth® devices

## Hardware

The following hardware can used for the exercises in this class:

Option 1:

- CYW20829 (Bluetooth LE 5.4 Controller and Cortex M33 MCU)
- Ammeter (for low power chapter exercises)

Option 2: 

- CY8CKIT-062S2-43012 (PSoC™ 62S2 Wi-Fi/Bluetooth Pioneer Kit)
- Ammeter (for low power chapter exercises)

Option 3: As a less expensive alternative, the following kit can be used:

- CY8CPROTO-062-4343W or CY8CPROTO-062S2-43439 (PSoC™ 62 Wi-Fi BT Prototyping Kits)
- Ammeter (for low power chapter exercises)

*The prototyping kits only have one user LED. Exercises that use a second LED as a connection status indicator will still function, but the status indicator will not be available unless an external LED is connected between VTARG and P10_0.*

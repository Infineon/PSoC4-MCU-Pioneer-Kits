## Table of Contents

* [Introduction](#introduction)
* [Navigating the Repository](#navigating-the-repository)
* [Required Tools](#required-tools)
* [Code Examples List](#code-examples-list)
* [References](#references)

# Introduction

This repository contains examples and demos for [PSoC 4 MCU](http://www.cypress.com/products/32-bit-arm-cortex-m0-psoc-4) family of devices, world's Most Flexible 32-bit ARM Cortex-M0 One-Chip Solution. Cypress provides a wealth of data at [www.cypress.com](http://www.cypress.com/) to help you select the right PSoC device and effectively integrate it into your design. Visit our [PSoC 4 MCU](http://www.cypress.com/products/32-bit-arm-cortex-m0-psoc-4) webpage to explore more about PSoC 4 MCU family of device.

Feel free to explore through the code example source files and Let us innovate together!

# Navigating the Repository

Cypress provides PSoC 4 based development kits for various family of PSoC 4 devices. These development kit helps you validate your prototype design before starting the actual design development. Refer to the Hardware section for the list of PSoC 4 development kits.

Each Development Kit is packaged with set of code examples which can get you started with PSoC 4 MCU as well as develop the hardware for your application by using the Kit hardware configuration as a reference. 
This repository contains all the kit related examples and the kit user guide. Refer to the respective kit user guide for the project implementation details.  
If you are new to developing projects with PSoC 4 MCU, we recommend you to refer the PSoC 4 Getting Started GitHub page which can help you familiarize with device features and guides you to create a simple PSoC 4 design with PSoC Creator IDE. For other block specific design please visit the following GitHub Pages:

#### 1. Analog Designs
#### 2. Digital Designs
#### 3. Connectivity Designs
#### 4. Device Related Designs
#### 5. System Designs

# Required Tools

## Software
### Integrated Development Environment (IDE)
To use the code examples in this repository, please download and install
[PSoC Creator](http://www.cypress.com/products/psoc-creator)

## Hardware
### PSoC 4 MCU Development Kits
* [CY8CKIT-042-BLE-A Bluetooth® Low Energy 4.2 Compliant Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/cy8ckit-042-ble-bluetooth-low-energy-42-compliant-pioneer-kit).

* [CY8CKIT-041-41XX PSoC 4100S CapSense Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/cy8ckit-041-41xx-psoc-4100s-capsense-pioneer-kit). 

* [CY8CKIT-046 PSoC 4 L-Series Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/cy8ckit-046-psoc-4-l-series-pioneer-kit)

* [CY8CKIT-044 PSoC Analog Coprocessor Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/cy8ckit-048-psoc-analog-coprocessor-pioneer-kit)

* [PSoC 4 CY8CKIT-049 4xxx Prototyping Kits](http://www.cypress.com/documentation/development-kitsboards/psoc-4-cy8ckit-049-4xxx-prototyping-kits)

* [CY8CKIT-042 PSoC 4 Pioneer Kit](http://www.cypress.com/documentation/development-kitsboards/cy8ckit-042-psoc-4-pioneer-kit) 

**Note** Please refer to the code example documnetation for selecting the appropriate kit for testing the project

## Code Example List

### CY8CKIT-041-41XX PSoC 4100S CapSense Pioneer Kit 
#### 1. CE214022 LP CapSense Buttons
This code example implements two CapSense buttons using CY8CKIT-041-41XX. The left button is used to control the onboard RGB LED color, and the right button is used to control the brightness of the RGB LED. Using the low-power modes available in the PSoC 4100S device, an average current of 5 µA per button is achieved when the touch is not detected.
#### 2. CE214023 Proximity Sensing
This code example demonstrates CapSense based proximity sensing using a PCB trace as a proximity sensor. A proximitysensing distance of 5 cm is achieved using a rectangular loop sensor with a 9-cm diagonal. Proximity detection is indicated by controlling the brightness of an LED. The LED has a minimum brightness when the hand is at a distance of 5 cm; it gradually increases as the hand approaches the sensor. Using the low-power modes available in the PSoC 4100S device, an average current of 25 µA is achieved while detecting the proximity of a hand at 5 cm.
#### 3. CE214025 Trackpad With Color Gamut
This code example implements a CapSense based trackpad as a user interface. The trackpad has the CIE 1931 color gamut imprinted; user inputs (touch coordinates) are converted to the corresponding color coordinates. The RGB LED on the board is used to illustrate the chosen color by modulating the associated signal densities. The brightness of the RGB LED is controlled by using the two CapSense buttons.
#### 4. CE216892 USB-HID Trackpad
This code example implements a CapSense trackpad and two button sensors using a PSoC 4100S device. The PSoC device is interfaced to a Windows PC as a mouse using the USB HID protocol. The trackpad controls the cursor on the PC and the two button sensors act as right-click and left-click buttons. To optimize device power consumption and provide optimum touch response, this code example implements two power modes: Fast Scan Mode and Slow Scan Mode.

### CY8CKIT-042 PSoC 4 Pioneer Kit
#### 1. Blinking LED
This example uses a pulse-width modulator (PWM) to illuminate the RGB LED. The PWM output is connected to pin P0_3 (blue) of the RGB LED. The frequency of blinking is set to 1 Hz with a duty cycle of 50 percent. The blinking frequency and duty cycle can be varied by varying the period and compare value respectively.
#### 2. PWM
This code example demonstrates the use of the PWM component. The project uses three PWM outputs to set the color of RGB LED on the Pioneer Kit. The LED cycles through seven colors – violet > indigo > blue > green > yellow > orange > red (VIBGYOR). Each color is maintained for a duration of one second. The different colors are achieved by changing the pulse width of the PWMs
#### 3. Deep Sleep
LED is on for one second and turns off, which indicates that the device has entered Deep-Sleep mode. Press SW2 switch to wake up the
device from Deep-Sleep mode and enter Active mode. The device goes back to sleep after one second..
#### 4. CapSense
The brightness of the green and red LEDs are varied based on the position of the user’s finger on the CapSense slider. When the finger is on segment 5 (P1[5]) of the slider, the green LED is brighter than the red LED; when the finger is on segment 1 (P1[1]) of the slider, the red LED is brighter than the green LED.


### CY8CKIT-042-BLE Pioneer Kit
#### 1. CapSense Slider and LED
This project demonstrates connectivity between the BLE Pioneer Kit (acting as a Peripheral and GATT server device) and CySmart Central Emulation tool or mobile device running the CySmart mobile application (acting as a Central and GATT client device). It transfers CapSense Slider data and RGB LED color data between the connected devices over BLE.
#### 2. CapSense Proximity
This project demonstrates connectivity between the BLE Pioneer Kit (acting as a Peripheral and GATT server device) and CySmart Central Emulation tool or mobile device running the CySmart mobile application (acting as a Central and GATT client device). It transfers CapSense proximity data from the kit to the connected BLE client device.
#### 3. Central Mode
This example project demonstrates the Central and GATT client mode where the kit will scan for a Peripheral device, connect to it, and send commands. In this project, the BLE Pioneer Kit scans and autoconnects to a particular Peripheral device supporting Immediate Alert Service (IAS).
#### 4. Direct Test Mode(DTM)
This project configures the kit for Direct Test Mode using Host Controller Interface (HCI). Direct Test Mode (DTM) is a method to test the BLE PHY layer and provide a report back to the tester.
#### 5. Eddystone
This project demonstrates a BLE beacon based on Google's Eddystone™ protocol on the BLE Pioneer Kit. A beacon is a wireless device that broadcasts data (such as temperature) over a periodic radio signal from a known location. BLE-based beacons use the BLE advertisement packets to broadcast data. This project demonstrates the following:
■ Beacon (non-connectable advertisement)
■ Beacon configuration (connectable advertisement)


## References
#### 1. PSoC 4 MCU
PSoC 4 is the world's Most Flexible 32-bit ARM Cortex-M0 One-Chip Solution. PSoC 4 has tackled some of the complex portions of embedded system design making it easier for you to get your product to market. Functions such as analog sensor integration, capacitive touch, and wireless connectivity have been integrated and optimized in PSoC 4 to “just work” so you don’t have to. To learn more on the device. Learn more: [PSoC 4 MCU](http://www.cypress.com/products/32-bit-arm-cortex-m0-psoc-4)

####  2. PSoC 4 MCU Learning resource list
##### 2.1 PSoC 4 MCU Datasheets
Device datasheets list the features and electrical specifications of PSoC 4 families of devices: [PSoC 4 MCU Datasheets](http://www.cypress.com/search/all?f%5b0%5d=meta_type%3Atechnical_documents&f%5b1%5d=field_related_products%3A1297&f%5b2%5d=resource_meta_type%3A575)
##### 2.2 PSoC 4 MCU Application Notes
Application notes are available on the Cypress website to assist you with designing your PSoC application: [A list of PSoC 4 MCU ANs](https://community.cypress.com/external-link.jspa?url=http%3A%2F%2Fwww.cypress.com%2Fsearch%2Fall%3Ff%255b0%255d%3Dmeta_type%253Atechnical_documents%26f%255b1%255d%3Dfield_related_products%253A1297%26f%255b2%255d%3Dresource_meta_type%253A574)
##### 2.3 PSoC 4 MCU Component Datasheets
PSoC Creator utilizes "components" as interfaces to functional Hardware (HW). Each component in PSoC Creator has an associated datasheet that describes the functionality, APIs, and electrical specifications for the HW. You can access component datasheets in PSoC Creator by right-clicking a component on the schematic page or by going through the component library listing. You can also access component datasheets from the Cypress website: [PSo 4 Component Datasheets](https://community.cypress.com/external-link.jspa?url=http%3A%2F%2Fwww.cypress.com%2Fsearch%2Fall%3Ff%255b0%255d%3Dmeta_type%253Asoftware_tools%26f%255b1%255d%3Dfield_related_products%253A1297%26f%255b2%255d%3Dsoftware_tools_meta_type%253A532)
##### 2.4 PSoC 4 MCU Technical Reference Manuals (TRM)
The TRM provides detailed descriptions of the internal architecture of PSoC 4 devices:[PSoC 4 MCU TRMs](https://community.cypress.com/external-link.jspa?url=http%3A%2F%2Fwww.cypress.com%2Fsearch%2Fall%3Ff%255b0%255d%3Dmeta_type%253Atechnical_documents%26f%255b1%255d%3Dfield_related_products%253A1297%26f%255b2%255d%3Dresource_meta_type%253A583)

## FAQ

### Technical Support
If you have any queries or questions, our technical support team would be happy to assist you. You can create a support request at [This link] (https://secure.cypress.com/myaccount/?id=25&techSupport=1), or if you are in the United States, you can talk to our technical support team by calling our toll-free number +1-800-541-4736 and then selecting option 8 at the IVR prompt. 

You can also use the following support resources if you need quick assistance: 
Self-help: [http://www.cypress.com/support] (http://www.cypress.com/support)
Local Sales office locations: [http://www.cypress.com/about-us/sales-offices] (http://www.cypress.com/about-us/sales-offices)

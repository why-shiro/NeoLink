
<div align="center"> <img src="NEOSTELLARlogov2.png" width="600"> </div>

> **Before reading this file, as Neo Stellar, we will not continue the work of our 2.4 GHz module in the future due to the fact that we cannot find suitable parts and the supply of parts is expensive. But we'll include the code for E28 (SX1280).**

>  **W.I.P!! We do not suggest you working on this code or building projects with this code!!**

>  **Source code is currently very messy, we'll make it more organised after finishing the prototype.**
 
# NeoLink 2.4 Ghz Telemetry Module

NeoLink is a long-range radio telemetry module based on LoRa technology, operating in the 2.4GHz band.

NeoLink offers an impressive range, making it suitable for various telemetry applications that require data transmission over extended distances. Its long-range capabilities enable seamless communication between remote devices and centralized systems, even in challenging environments with obstacles or signal obstructions.


# Hardware

NeoLink module is used in conjunction with the Ebyte E28 (SX1280) module. To enhance its performance, it is equipped with a dual-core ESP32 processor (with WiFi mode disabled).

 - <a href="https://www.cdebyte.com/products/E28-2G4M27S">E28-2G4M27S</a>
 - <a href="https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwi-g5uzrJ-AAxVBVPEDHb4rBc4QFnoECA8QAQ&url=https%3A%2F%2Fwww.espressif.com%2Fsites%2Fdefault%2Ffiles%2Fdocumentation%2Fesp32-wroom-32d_esp32-wroom-32u_datasheet_en.pdf&usg=AOvVaw3UsvStCajXbJlvYkXPsawD&opi=89978449">ESP32-WROOM-32D</a>
 - <a href="https://www.renesas.com/us/en/products/rf-products/rf-amplifiers/f1475-1w-high-linearity-rf-amplifier-700mhz-2800mhz">1W RF Power Amplifier</a>

## Test and Results

| Antenna | Power (dB)|Range (km)|
|--|--|--|
| Isotropic Antenna | 10dB | 5km (Maximum Range Test)|
|Yagi Antena with AeroWave Tracker| 10 dB | 3km (within the city) |

The isotropic antenna test was conducted in an open area, in the outdoors. The Yagi antenna test, on the other hand, was performed within the city. **If additional comments are to be added, it should be noted that the Yagi antenna causes packet losses within the city.**

## How does the System Work?

NeoLink is based on the Time Division Duplex (TDD) working principle. The module alternates between sending and listening to packets during specific time intervals. All these operations occur rapidly within a short time frame. On average, a 32 byte data is transmitted in approximately 10 - 25 milliseconds.

**This graph show how pairing works:**

```mermaid
sequenceDiagram
NeoLink Module 1 ->> NeoLink Module 2: Hey, I want to pair with you
NeoLink Module 2 -->> NeoLink Module 2: I am not paired so I need to reply
NeoLink Module 2 ->> NeoLink Module 1: Hey, let's do pairing. Here is my secret key!
NeoLink Module 1 ->> NeoLink Module 2: Oh, here is an encrypted message with your key!
NeoLink Module 2 ->> NeoLink Module 1: That looks good! Let's start communicating.
NeoLink Module 3 --x NeoLink Module 2: I want to pair with you too! (No response)
```

**This graph show how data transmitting works:**

> In our module, each tick is calculated by the ESP32 before it is sent.

```mermaid
sequenceDiagram
NeoLink Module 1 ->> NeoLink Module 2: This is the data I need to send you ( tick )
NeoLink Module 1 ->> NeoLink Module 2: This is the data I need to send you ( tick )
NeoLink Module 1 ->> NeoLink Module 2: This is the data I need to send you, do you need to send me anything? (tick)
NeoLink Module 2 ->> NeoLink Module 2: Checks for update
NeoLink Module 2 ->> NeoLink Module 1: Oh, I have very important updates which I need to send you (tick)
NeoLink Module 1 ->> NeoLink Module 1: Updates itself with new data
NeoLink Module 1 ->> NeoLink Module 2: This is the data I need to send you ( tick )
NeoLink Module 1 ->> NeoLink Module 2: This is the data I need to send you ( tick )
```
and the loop continues...

## Power Consumption Tests (20000mAh 5V, 1A Powerbank)

| Type | Current (mA) | Estimated Time |
|--|--|--|
| 10 dDm Isotropic Antenna (without Amp.) | 120 ~ 250| 65 Hrs|
| 27 dDm Yagi Antenna (with 1W Amp.)  |x| x|

## Multi-Frequency Performance Test (NEW)

Multi-Frequency mode helps you to send your flight data on different frequencies to make transmission more secure and lowers the success rate of the jammer attack on the drone.

| Range (Km) | Total Data Sent | Lost Packages | Success Rate (%)|
|--|--|--|--|
| 0.1| 2000| 2|99.9|
|0.1|12000|5|99.96|

## Safety Protections

 - AES-256 Encryption
 - Pair to Pair communication
 - Creating secret key with two modules to encrypt data.
 - Frequency Hopping Spread Spectrum (FHSS)

For suggestions, please contact us at : info@neostellar.net

## To Do List

| No. | Mission |
|---|---|
| 1|Implementing TDD algorithm|
| 2 |Changing Ebyte E28 to another LoRa Module (933mHz Freq.)|
| 3 |Testing long-range communication with 1W Amp. and reporting power consumption (for 933MHz)|
| 4 |Testing long-range communication with 5W Amp. and reporting power consumption (for 933MHz)|

## Updates

As Neo Stellar, we are still working on this project. As soon as we finish first alpha version of the project, we'll start uploading source code of this project.

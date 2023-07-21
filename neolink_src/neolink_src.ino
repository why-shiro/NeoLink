#include <pitches.h>
#include <Wire.h>
#include <Keypad.h>
#include <sstream>
#include <SPI.h>
#include <SX128XLT.h>
#include "Settings.h"

#define Program_Version "V1.1"

uint32_t RXpacketCount;
uint32_t errors;
bool device_var = false;

uint8_t RXBUFFER[RXBUFFER_SIZE];                 //create the buffer that received packets are copied into

uint8_t RXPacketL;                               //stores length of packet received
int16_t PacketRSSI;                              //stores RSSI of received packet
int8_t  PacketSNR;                               //stores signal to noise ratio (SNR) of received packet

SX128XLT LT; // create a library class instance called LT

#include <LiquidCrystal_I2C.h>
#include <Keypad_I2C.h>
uint8_t TXPacketL;
uint32_t TXPacketCount, startmS, endmS;
uint8_t size;
LiquidCrystal_I2C lcd(0x27, 16, 2);

#include "SparkFunMPL3115A2.h"
MPL3115A2 myPressure;

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {0, 1, 2, 3};
byte colPins[COLS] = {4, 5, 6, 7};

int i2caddress = 0x20;

Keypad_I2C kpd(makeKeymap(keys), rowPins, colPins, ROWS, COLS, i2caddress);

int menu = 0;
int submenu = 0;
int checker = 0;
char key;
int key_value = 0;
char key1;
float pressure = 0.0;
float altitude = 0.0;
float temperature = 0.0;
uint8_t butter[] = "testing";
void setup()
{
    Wire.begin();
    myPressure.begin();                // Get sensor online
    myPressure.setModeAltimeter();     // Measure altitude above sea level in meters
    myPressure.setOversampleRate(4);   // Set Oversample to the recommended 128
    myPressure.enableEventFlags();
    lcd.init();
    kpd.begin();
    lcd.backlight();
    lcd.setCursor(0, 0);
    Serial.begin(9600);
    lcd.print("Initializing...");
    lcd.setCursor(0, 1);
    lcd.print("NeoStellar v1.0");
    delay(1500);

    pinMode(LED1, OUTPUT);
    led_Flash(2, 125);
    Serial.println();
    Serial.print(F(__TIME__));
    Serial.print(F(" "));
    Serial.println(F(__DATE__));
    Serial.println(F(Program_Version));
    Serial.println();
    Serial.println(F("103_LoRa_Transmitter_Detailed_Setup Starting"));

    SPI.begin();
    lcd.clear();
    updateMenu();
}


void LoRa_INIT(){

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, DIO2, DIO3, RX_EN, TX_EN, LORA_DEVICE))
    {
        Serial.println(F("LoRa Device found"));
        led_Flash(2, 125);
        delay(1000);
    }
    else
    {
        Serial.println(F("No device responding"));
        while (1)
        {
            led_Flash(50, 50);
        }
    }
    setupLoRa();
}

void setupLoRa(){
    LT.setMode(MODE_STDBY_RC);
    LT.setRegulatorMode(USE_LDO);
    LT.setPacketType(PACKET_TYPE_LORA);
    LT.setRfFrequency(Frequency, Offset);
    LT.setBufferBaseAddress(0, 0);
    LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
    LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
    LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
    //***************************************************************************************************

    Serial.println();
    LT.printModemSettings();         //reads and prints the configured LoRa settings, useful check
    Serial.println();
    LT.printOperatingSettings();     //reads and prints the configured operating settings, useful check
    Serial.println();
    Serial.println();
    LT.printRegisters(0x900, 0x9FF); //print contents of device registers, normally 0x900 to 0x9FF
    Serial.println();
    Serial.println();

    Serial.print(F("Ready"));
    Serial.println();
}

void loop()
{
    if (checker == 0)
    {
        menuChanger();
    }
    else
    {
        subMenuChanger();
    }
}

void updateReadings()
{
    pressure = myPressure.readAltitude();
    altitude = myPressure.readAltitudeFt();
    temperature = myPressure.readTemp();
}

void subMenuChanger()
{
    key = kpd.getKey();
    if (key != NO_KEY)
    {
        if (key == '#' || key == '*')
        {
            submenu = (submenu + 1) % 2;
            updateSubMenuSelection();
            delay(100);
            while (kpd.getKey() == key)
                ;
        }
        else if (key == '0')
        {
            updateSubMenuSelection();
            executeSubMenuAction();
            delay(100);
            while (kpd.getKey() == '0')
                ;
        }
    }
}

void menuChanger()
{
    key = kpd.getKey();
    if (key != NO_KEY)
    {
        if (key == '#' || key == '*')
        {
            menu = (menu + (key == '#' ? 1 : 3)) % 4;
            updateMenu();
            delay(100);
            while (kpd.getKey() == key)
                ;
        }
        else if (key == '0')
        {
            executeAction();
            if (checker == 0)
            {
                updateMenu();
            }
            delay(100);
            while (kpd.getKey() == '0')
                ;
        }
    }
}

void updateMenu()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    switch (menu)
    {
    case 0:
        lcd.print(">Auto Config");
        lcd.setCursor(0, 1);
        lcd.print("Manual Config");
        break;
    case 1:
        lcd.print("Auto Config");
        lcd.setCursor(0, 1);
        lcd.print(">Manual Config");
        break;
    case 2:
        lcd.print("Manual Config");
        lcd.setCursor(0, 1);
        lcd.print(">Readings");
        break;
    case 3:
        lcd.print("Readings");
        lcd.setCursor(0, 1);
        lcd.print(">Setting");
        break;
    }
}

void executeAction()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    switch (menu)
    {
    case 0:
        lcd.print("Setting 1");
        break;
    case 1:
        updateSubMenuSelection();
        checker = 1;
        break;
    case 2:
        key1 = 0;
        while (true)
        {
            key1 = kpd.getKey();
            if (key1 == '0')
            {
                break;
            }
            updateReadings();
            std::stringstream butter;
            butter << pressure << " " << altitude << " " << temperature;
            std::string concatenatedString = butter.str();
            uint8_t *buff = (uint8_t *)concatenatedString.c_str();
            uint8_t size = concatenatedString.length();
            transfer(buff, size);
            
        }
        device_var=false;
        break;
    case 3:
key1 = 0;
        while (true)
        {
            key1 = kpd.getKey();
            if (key1 == '0')
            {
                break;
            }
            receiver();
        }
        device_var=false;
break;


    
    delay(1000);
}
}

void updateSubMenuSelection()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    switch (submenu)
    {
    case 0:
        lcd.print(">FDD");
        lcd.setCursor(0, 1);
        lcd.print("TDD");
        break;
    case 1:
        lcd.print("FDD");
        lcd.setCursor(0, 1);
        lcd.print(">TDD");
        break;
    }
}

void executeSubMenuAction()
{
    lcd.clear();
    lcd.setCursor(0, 0);
    int go2 = 0; // Move the variable declaration here
    switch (submenu)
    {
    case 0:
        lcd.print("FDD is selected");
        lcd.setCursor(0, 1);
        go2 = gettingNum();
        checker = 0;
        break;
    case 1:
        lcd.print("TDD is selected");
        checker = 0;
        break;
    }

    // Use the `go2` variable here if needed
}
int gettingNum()
{
    key = kpd.getKey();
    while (key != '#')
    {
        key = kpd.getKey();
        if ((key != NO_KEY) && (key != '#'))
        {
            lcd.print(key);
            key_value = 10 * key_value + (key - '0');
            key = 0;
        }
    }
    return key_value;
}

void packet_is_OK(uint8_t *buff)
{
    uint16_t localCRC;

    Serial.print(F("  BytesSent,"));
    Serial.print(TXPacketL);
    localCRC = LT.CRCCCITT(buff, TXPacketL, 0xFFFF);
    Serial.print(F("  CRC,"));
    Serial.print(localCRC, HEX);
    Serial.print(F("  TransmitTime,"));
    Serial.print(endmS - startmS);
    Serial.print(F("mS"));
    Serial.print(F("  PacketsSent,"));
    Serial.print(TXPacketCount);
}

void packet_is_Error()
{
    uint16_t IRQStatus;
    IRQStatus = LT.readIrqStatus();
    Serial.print(F(" SendError,"));
    Serial.print(F("Length,"));
    Serial.print(TXPacketL);
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);
    LT.printIrqStatus();
}

void led_Flash(uint16_t flashes, uint16_t delaymS)
{
    uint16_t index;
    for (index = 1; index <= flashes; index++)
    {
        digitalWrite(LED1, HIGH);
        delay(delaymS);
        digitalWrite(LED1, LOW);
        delay(delaymS);
    }
}

void transfer(uint8_t *buff, uint8_t size)
{
     if (!device_var) {
    // Call your function here
     LoRa_INIT();

    // Set the flag to true to prevent the function from running again
    device_var = true;
  }
    Serial.print(TXpower); //print the transmit power defined
    Serial.print(F("dBm "));
    Serial.print(F("Packet> "));
    Serial.flush();
    TXPacketL = size;              //set TXPacketL to length of array
    buff[TXPacketL - 1] = '*';     //replace null character at buffer end so its visible on receiver

    LT.printASCIIPacket(buff, TXPacketL); //print the buffer (the sent packet) as ASCII

    digitalWrite(LED1, HIGH);
    startmS = millis(); //start transmit timer
    if (LT.transmit(buff, TXPacketL, 10000, TXpower, WAIT_TX)) //will return packet length sent if OK, otherwise 0 if transmit error
    {
        endmS = millis(); //packet sent, note end time
        TXPacketCount++;
        packet_is_OK(buff);
    }
    else
    {
        packet_is_Error(); //transmit packet returned 0, there was an error
    }

    digitalWrite(LED1, LOW);
    Serial.println();
    delay(packet_delay);
}


void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F("s"));
}

void packet_is_OK_RX()
{
  uint16_t IRQStatus, localCRC;

  IRQStatus = LT.readIrqStatus();                 //read the LoRa device IRQ status register

  RXpacketCount++;

  printElapsedTime();                             //print elapsed time to Serial Monitor
  Serial.print(F("  "));
  LT.printASCIIPacket(RXBUFFER, RXPacketL);       //print the packet as ASCII characters

  localCRC = LT.CRCCCITT(RXBUFFER, RXPacketL, 0xFFFF);  //calculate the CRC, this is the external CRC calculation of the RXBUFFER
  Serial.print(F(",CRC,"));                       //contents, not the LoRa device internal CRC
  Serial.print(localCRC, HEX);
  Serial.print(F(",RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Length,"));
  Serial.print(RXPacketL);
  Serial.print(F(",Packets,"));
  Serial.print(RXpacketCount);
  Serial.print(F(",Errors,"));
  Serial.print(errors);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);
}


void packet_is_Error_RX()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                   //read the LoRa device IRQ status register

  printElapsedTime();                               //print elapsed time to Serial Monitor

  if (IRQStatus & IRQ_RX_TIMEOUT)                   //check for an RX timeout
  {
    Serial.print(F(" RXTimeout"));
  }
  else
  {
    errors++;
    Serial.print(F(" PacketError"));
    Serial.print(F(",RSSI,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm,SNR,"));
    Serial.print(PacketSNR);
    Serial.print(F("dB,Length,"));
    Serial.print(LT.readRXPacketL());               //get the device packet length
    Serial.print(F(",Packets,"));
    Serial.print(RXpacketCount);
    Serial.print(F(",Errors,"));
    Serial.print(errors);
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);
    LT.printIrqStatus();                            //print the names of the IRQ registers set
  }

  delay(250);                                       //gives a longer buzzer and LED flash for error 
  
}

void receiver(){
  if (!device_var) {
    // Call your function here
     LoRa_INIT();

    // Set the flag to true to prevent the function from running again
    device_var = true;
  }
  RXPacketL = LT.receive(RXBUFFER, RXBUFFER_SIZE, 3000, WAIT_RX); //wait for a packet to arrive with 60seconds (60000mS) timeout

  digitalWrite(LED1, HIGH);                      //something has happened

  PacketRSSI = LT.readPacketRSSI();              //read the recived RSSI value
  PacketSNR = LT.readPacketSNR();                //read the received SNR value

  if (RXPacketL == 0)                            //if the LT.receive() function detects an error, RXpacketL is 0
  {
    packet_is_Error_RX();
  }
  else
  {
    packet_is_OK_RX();
  }

  digitalWrite(LED1, LOW);                       //LED off

  Serial.println();
}

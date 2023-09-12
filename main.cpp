/*************************************************************************
      T1 Interface

      V2.0 (12.09.2023)

      Peter Jonas DL5DLA

      This code, running on an ESP32, requests the current frequency
      from the ICOM IC-705 via bluetooth connection, and sends the
      related band number to the Elecraft T1. Based on the band
      number, the T1 applies the stored tuning settings. A re-tuning
      is possible directly after changing the frequency. For the next
      few seconds the T1 waits for a carrier and will tune again. 

      For first use, the ESP32 must be paired to the IC-705. Later on
      the ESP32 should be powered already, before the IC-705 is
      switched on. If someone knows how the bluetooth connection
      can be setup the other way around (switch on the IC-705, then
      the esp32), please let me know.

      The code is compiled on PlatformIO, but should also work using
      the Arduino IDE.

      This code is based on the code from
        Ondrej Kolonicny (ok1cdj) https://github.com/ok1cdj/IC705-BT-CIV
      and
        Matthew Robinson (VK6MR) https://zensunni.org/blog/2011/01/19/arduino-elecraft-t1-interface/

      Many thanks to both for their example codes giving me the first insight on how
      to deal with the communication between the IC-705 and T1.

      In this version the ESP32 device is set as bluetooth master and the IC-705 as client.
      Due to this the bluetooth connection is more robust, and the IC-705 is reconnected
      no matter if the TRX or the ESP32 is switched off temporarly.

      It is important now to set the "bd_address" in the next lines!
*************************************************************************/

#include <Arduino.h>
#include "BluetoothSerial.h"

// ######################################################################
// Enter the BD_ADDRESS of your IC-705. You can find it in the Bluetooth
// settings in section 'Bluetooth Device Information'

uint8_t bd_address[6]  = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
// ######################################################################


#define DATA_PIN  18              // GPIO18 input/output
#define TUNE_PIN  26              // GPIO26 (output)

#define CONTROLLER_ADDRESS   0xE0 //Controller address

#define START_BYTE           0xFE // Start byte
#define STOP_BYTE            0xFD // Stop byte

#define CMD_READ_FREQ        0x03 // Read operating frequency data

// Function prototypes:
void configRadioBaud(uint16_t);
uint8_t readLine(void);
bool searchRadio();
void sendCatRequest(uint8_t);
void printFrequency(void);
void processCatMessages();
void sendBit(int);
void sendBand(byte);

const uint32_t decMulti[]    = {1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};

#define BAUD_RATES_SIZE 4
const uint16_t baudRates[BAUD_RATES_SIZE]       = {19200, 9600, 4800, 1200};

void callback(esp_spp_cb_event_t, esp_spp_cb_param_t *);
uint8_t   radio_address;     //Transceiver address
uint16_t  baud_rate;         //Current baud speed
uint32_t  readtimeout;       //Serial port read timeout
uint8_t   read_buffer[12];   //Read buffer
uint32_t  frequency;         //Current frequency in Hz
uint32_t  timer;

#define NUM_OF_BANDS 13
const uint32_t bands[][2] = 
{
  {   1810,  2000 },  // 160m
  {   3500,  3800 },  // 80m
  {   5351,  5367 },  // 60m
  {   7000,  7200 },  // 40m
  {  10100, 10150 },  // 30m
  {  14000, 14350 },  // 20m
  {  18068, 18168 },  // 17m
  {  21000, 21450 },  // 15m
  {  24890, 24990 },  // 12m
  {  28000, 29700 },  // 10m
  {  50030, 51000 },  // 6m
  { 144000,146000 },  // 2m
  { 430000,440000 }   // UHF
};

String modes;

byte prev_band = 0xFF;
boolean btConnected = false;

BluetoothSerial SerialBT;


// ------------------------------------------
//   Callback to get info about connection
// ------------------------------------------
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {

  if(event == ESP_SPP_SRV_OPEN_EVT){    // 34
    btConnected = true;
    Serial.println("Client Connected");
  }
  else if (event == ESP_SPP_CLOSE_EVT)  // 27
  {
    btConnected = false;
    Serial.println("Client disconnected");
  }

}

// ----------------------------------------
//      Initialize bluetooth
// ----------------------------------------
void configRadioBaud(uint16_t baudrate)
{
  SerialBT.register_callback(callback);

  // Setup bluetooth as master:
  if(!SerialBT.begin("T1-INTERFACE",true)) {
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("Bluetooth initialized");
    Serial.println("The device started, now you can pair it with bluetooth!");
  }

  // Connect to client:
  Serial.print("Connect to bluetooth client ...");
  
  btConnected = SerialBT.connect(bd_address);

  while (!btConnected) {
    btConnected = SerialBT.connect(bd_address);
    Serial.println( "Need Pairing" );
  }
  Serial.println( "Transceiver connected" );
}

// ----------------------------------------
//    Read incoming line from bluetooth
// ----------------------------------------
uint8_t readLine(void)
{
  uint8_t byte;
  uint8_t counter = 0;
  uint32_t ed = readtimeout;  // not initialized!

  while (btConnected) {
    while (!SerialBT.available()) {
      if (--ed == 0 || !btConnected ) return 0; // leave the loop if BT connection is lost
    }
    ed = readtimeout;
    byte = SerialBT.read();
    if (byte == 0xFF) continue; //TODO skip to start byte instead

    read_buffer[counter++] = byte;
    if (STOP_BYTE == byte) break;

    if (counter >= sizeof(read_buffer)) return 0;
  }
  return counter;
}

// ----------------------------------------
//       Get address of transceiver
// ----------------------------------------
bool searchRadio()
{
  for (uint8_t baud = 0; baud < BAUD_RATES_SIZE; baud++) {
    configRadioBaud(baudRates[baud]);
    sendCatRequest(CMD_READ_FREQ);

    if (readLine() > 0)
    {
      if (read_buffer[0] == START_BYTE && read_buffer[1] == START_BYTE) {
        radio_address = read_buffer[3];
      }
      return true;
    }

  }

  radio_address = 0xFF;
  return false;
}

// ----------------------------------------
//    get band from frequency 
// ----------------------------------------
  byte getBand(uint32_t freq)
  {
    for(uint8_t i=0; i<NUM_OF_BANDS;i++) {
      if(freq >= bands[i][0] && freq <= bands[i][1] ) {
        if(i==12) return 12;  // T1 tuner does not have different settings for 2m and UHF 
        return i+1;
      }
    }
    return 0xFF;  // no band for considered frequency found
      
  }


// ----------------------------------------
//      Send CAT Request
// ----------------------------------------
void sendCatRequest(uint8_t requestCode)
{
  uint8_t req[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS, requestCode, STOP_BYTE};

  for (uint8_t i = 0; i < sizeof(req); i++) {
    SerialBT.write(req[i]);
  }

}


// ----------------------------------------
//      Print the received frequency
// ----------------------------------------
void printFrequency(void)
{
  frequency = 0;
  //FE FE E0 42 03 <00 00 58 45 01> FD ic-820
  //FE FE 00 40 00 <00 60 06 14> FD ic-732
  for (uint8_t i = 0; i < 5; i++) {
    if (read_buffer[9 - i] == 0xFD) continue; //spike
    frequency += (read_buffer[9 - i] >> 4) * decMulti[i * 2];
    frequency += (read_buffer[9 - i] & 0x0F) * decMulti[i * 2 + 1];
  }

}


// --------------------------------------------------
//   Process the received messages from transceiver
// --------------------------------------------------
void processCatMessages()
{
  /*
    <FE FE E0 42 04 00 01 FD  - LSB
    <FE FE E0 42 03 00 00 58 45 01 FD  -145.580.000

    FE FE - start bytes
    00/E0 - target address (broadcast/controller)
    42 - source address
    00/03 - data type
    <data>
    FD - stop byte
  */

  while (SerialBT.available()) {
    bool knowncommand = true;

    if (readLine() > 0) {

      if (read_buffer[0] == START_BYTE && read_buffer[1] == START_BYTE) {
        if (read_buffer[3] == radio_address) {
         if (read_buffer[2] == CONTROLLER_ADDRESS) {
            switch (read_buffer[4]) {
              case CMD_READ_FREQ:
                printFrequency();
                break;
              default:
                knowncommand = false;
            }
          }
        }
      }
    }
  }

}

// ----------------------------------------
//         sendBit
// ----------------------------------------
void sendBit(int bit) {

  digitalWrite(DATA_PIN, HIGH);
  if (bit != 0) {
    delay(4);
  } else {
    delayMicroseconds(1500);
  }

  digitalWrite(DATA_PIN, LOW);
  delayMicroseconds(1500);
}


// ----------------------------------------
//         sendBand
// ----------------------------------------
void sendBand(byte band) {

  unsigned long previousTime = 0;
  const long maxWaitTime = 50;

  // Pull the TUNE_PIN line high for half a second
  digitalWrite(TUNE_PIN, HIGH);
  delay(500);
  digitalWrite(TUNE_PIN, LOW);

  // The ATU will pull the DATA_PIN line HIGH for 50ms

  previousTime = millis();
  while(digitalRead(DATA_PIN) == LOW) {
    // Measure time the while loop is active, and jump out after maxWaiTime.
    // This esnures that the program does not lock in case the communication
    // with the ATU is temporarly broken
    
    unsigned long currentTime = millis();
    if (currentTime - previousTime > maxWaitTime) { 
      Serial.println("Error: No positive pulse from T1 detected!");
      return; 
    }
  }
  
  while(digitalRead(DATA_PIN) == HIGH) {
  }
  // Wait 10ms
  delay(10);

  // and then send data on the DATA line
  pinMode(DATA_PIN, OUTPUT);

  // 1 bits are HIGH for 4ms 
  // 0 bits are HIGH for 1.5ms
  // Gap between digits is 1.5ms LOW

  // 1st bit
  sendBit(band & 8);
  sendBit(band & 4);
  sendBit(band & 2);
  sendBit(band & 1);

  // Leave the line LOW
  digitalWrite(DATA_PIN, LOW);

  // and switch it back to an input
  pinMode(DATA_PIN, INPUT);
}

// ----------------------------------------
//         Setup
// ----------------------------------------
void setup()
{
  Serial.begin(115200);

  pinMode(DATA_PIN,INPUT);
  pinMode(TUNE_PIN, OUTPUT);
  digitalWrite(TUNE_PIN,LOW);

  while (radio_address == 0x00) {
    if (!searchRadio()) {
      Serial.println("Radio not found");
    } else {
      Serial.print("Radio found at ");
      Serial.print(radio_address, HEX);
      Serial.println();
    }
  }

}

// ----------------------------------------
//         Main loop
// ----------------------------------------
void loop()
{
  while (!btConnected) {
      Serial.println( "Connecting ..." );
      btConnected = SerialBT.connect(bd_address);
      if(btConnected)
          Serial.println( "Transceiver reconnected" );
  }

  sendCatRequest(CMD_READ_FREQ);
  processCatMessages();

  byte band = getBand(frequency/1000);

  if( (band != prev_band) && (band != 0xFF) ) {
    Serial.print("Frequency: ");
    Serial.print(frequency/1000);
    Serial.print(" -> band: ");
    Serial.println(band);
    sendBand(band);
    prev_band=band;
  }
  delay(50);  // TEST
}

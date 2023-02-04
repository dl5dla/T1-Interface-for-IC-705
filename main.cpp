/*************************************************************************
      T1 Interface

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

      Need for improvement:
      The code for controlling the T1 includes some delay() statements, which block the 
      bluetooth communication for a while. This leads to error messages
      "esp_spp_cb(): RX Full! Discarding 22 bytes" within the PlatformIO console.
      Although it seems to have no impact to the correct function, it should be 
      solved later on.
*************************************************************************/

#include "BluetoothSerial.h"

//#define DEBUG 1

#define DATA_PIN  18              // GPIO18 input/output
#define TUNE_PIN  26              // GPIO26 (output)

#define BROADCAST_ADDRESS    0x00 // Broadcast address
#define CONTROLLER_ADDRESS   0xE0 //Controller address

#define START_BYTE           0xFE // Start byte
#define STOP_BYTE            0xFD // Stop byte

#define CMD_SET_FREQ         0x00 // Set operating frequency data
#define CMD_SET_MODE         0x01 // Set operating mode data

#define CMD_READ_FREQ        0x03 // Read operating frequency data
#define CMD_READ_MODE        0x04 // Read operating mode data

#define CMD_WRITE_FREQ       0x05 // Write operating frequency data
#define CMD_WRITE_MODE       0x06 // Write operating mode data

#define IF_PASSBAND_WIDTH_WIDE     0x01
#define IF_PASSBAND_WIDTH_MEDIUM   0x02
#define IF_PASSBAND_WIDTH_NARROW   0x03

// Function prototypes:
void configRadioBaud(uint16_t);
uint8_t readLine(void);
bool searchRadio();
void radioSetMode(uint8_t, uint8_t);
void sendCatRequest(uint8_t);
void printFrequency(void);
void printMode(void);
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

const char* mode[] = {"LSB", "USB", "AM", "CW", "RTTY", "FM", "WFM",
                       "CW-R","RTTY-R","","","","","","","","","","","","","","","DV"};
#define MODE_TYPE_LSB     0x00
#define MODE_TYPE_USB     0x01
#define MODE_TYPE_AM      0x02
#define MODE_TYPE_CW      0x03
#define MODE_TYPE_RTTY    0x04
#define MODE_TYPE_FM      0x05
#define MODE_TYPE_WFM     0x06
#define MODE_TYPE_CW_R    0x07
#define MODE_TYPE_RTTY_R  0x08
#define MODE_TYPE_DV      0x17

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

byte prev_band = -1;

BluetoothSerial CAT;

// ------------------------------------------
//   Callback to get info about connection
// ------------------------------------------
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) 
{
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Client Connected");
  }

}

// ----------------------------------------
//      Initialize bluetooth
// ----------------------------------------
void configRadioBaud(uint16_t  baudrate)
{
  if (!CAT.begin("T1-INTERFACE")) //Bluetooth device name
  {
    Serial.println("An error occurred initializing Bluetooth");

  } else {
    CAT.register_callback(callback);
    Serial.println("Bluetooth initialized");
    Serial.println("The device started, now you can pair it with bluetooth!");
  }
}

// ----------------------------------------
//    Read incoming line from bluetooth
// ----------------------------------------
uint8_t readLine(void)
{
  uint8_t byte;
  uint8_t counter = 0;
  uint32_t ed = readtimeout;

  while (true)
  {
    while (!CAT.available()) {
      if (--ed == 0)return 0;
    }
    ed = readtimeout;
    byte = CAT.read();
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
#ifdef DEBUG
    Serial.print("Try baudrate ");
    Serial.println(baudRates[baud]);
#endif
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
  uint8_t getBand(uint32_t freq)
  {

    for(uint8_t i=0; i<NUM_OF_BANDS;i++) {
      if(freq >= bands[i][0] && freq <= bands[i][1] ) {
        if(i==12) return 12;  // T1 tuner does not have different settings for 2m and UHF 
        return i+1;
      }
    }
    return -1;  // no band for considered frequency found
      
  }

// ----------------------------------------
//      Set mode
// ----------------------------------------
void radioSetMode(uint8_t modeid, uint8_t modewidth)
{
  uint8_t req[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS, CMD_WRITE_MODE, modeid, modewidth, STOP_BYTE};
#ifdef DEBUG
  Serial.print(">");
#endif
  for (uint8_t i = 0; i < sizeof(req); i++) {
    CAT.write(req[i]);
#ifdef DEBUG
    if (req[i] < 16) Serial.print("0");
    Serial.print(req[i], HEX);
    Serial.print(" ");
#endif
  }
#ifdef DEBUG
  Serial.println();
#endif
}

// ----------------------------------------
//      Send CAT Request
// ----------------------------------------
void sendCatRequest(uint8_t requestCode)
{
  uint8_t req[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS, requestCode, STOP_BYTE};
#ifdef DEBUG
  Serial.print(">");
#endif

  for (uint8_t i = 0; i < sizeof(req); i++) {
    CAT.write(req[i]);

#ifdef DEBUG
    if (req[i] < 16)Serial.print("0");
    Serial.print(req[i], HEX);
    Serial.print(" ");
#endif
  }
#ifdef DEBUG
  Serial.println();
#endif
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
  #ifdef DEBUG
      if (read_buffer[9 - i] < 16)Serial.print("0");
    Serial.print(read_buffer[9 - i], HEX);
#endif

    frequency += (read_buffer[9 - i] >> 4) * decMulti[i * 2];
    frequency += (read_buffer[9 - i] & 0x0F) * decMulti[i * 2 + 1];
  }
#ifdef DEBUG
  Serial.println();
#endif
}

// ----------------------------------------
//      Print the received mode
// ----------------------------------------
void printMode(void)
{
  //FE FE E0 42 04 <00 01> FD
#ifdef DEBUG
  Serial.print(read_buffer[5]);Serial.print("  ");
  Serial.println(mode[read_buffer[5]]);
#endif
  modes = mode[read_buffer[5]];
  //read_buffer[6] -> 01 - Wide, 02 - Medium, 03 - Narrow

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

  while (CAT.available()) {
    bool knowncommand = true;

    if (readLine() > 0) {

      if (read_buffer[0] == START_BYTE && read_buffer[1] == START_BYTE) {
        if (read_buffer[3] == radio_address) {
          if (read_buffer[2] == BROADCAST_ADDRESS) {

            switch (read_buffer[4]) {
              case CMD_SET_FREQ:
                printFrequency();
                break;
              case CMD_SET_MODE:
                printMode();
                break;
              default:
                knowncommand = false;
            }
          } else if (read_buffer[2] == CONTROLLER_ADDRESS) {
            switch (read_buffer[4]) {
              case CMD_READ_FREQ:
                printFrequency();
                break;
              case CMD_READ_MODE:
                printMode();
                break;
              default:
                knowncommand = false;
            }
          }
        } else {
#ifdef DEBUG
          Serial.print(read_buffer[3]);
          Serial.println(" also on-line?!");
#endif
        }
      }
    }

#ifdef DEBUG
    //if(!knowncommand){
    Serial.print("<");
    for (uint8_t i = 0; i < sizeof(read_buffer); i++) {
      if (read_buffer[i] < 16)Serial.print("0");
      Serial.print(read_buffer[i], HEX);
      Serial.print(" ");
      if (read_buffer[i] == STOP_BYTE)break;
    }
    Serial.println();
    //}
#endif
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
  Serial.begin(9600);

  pinMode(DATA_PIN,INPUT);
  pinMode(TUNE_PIN, OUTPUT);
  digitalWrite(TUNE_PIN,LOW);

  while (radio_address == 0x00) {
    if (!searchRadio()) {
#ifdef DEBUG
      Serial.println("Radio not found");
#endif
    } else {
//#ifdef DEBUG
      Serial.print("Radio found at ");
      Serial.print(radio_address, HEX);
      Serial.println();
//#endif
    }
  }

}

// ----------------------------------------
//         Main loop
// ----------------------------------------
void loop()
{
  sendCatRequest(CMD_READ_FREQ);
  processCatMessages();

  byte band = getBand(frequency/1000);

  if( band != prev_band) {
    Serial.print("Frequency: ");
    Serial.print(frequency/1000);
    Serial.print(" -> band: ");
    Serial.println(band);
    sendBand(band);
    prev_band=band;
  }
  delay(50);  // TEST
}

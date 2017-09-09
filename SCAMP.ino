
#include "mcp_can.h"
#include <SPI.h>
#include <Servo.h>
#include <Encoder.h>

#define CAN_CS    A2    // ATMEGA Pin 25, PC2, ADC2
#define CAN_SCK   13    // ATMEGA Pin 19, PB5
#define CAN_MOSI  11    // ATMEGA Pin 17, PB3
#define CAN_MISO  12    // ATMEGA Pin 18, PB4
#define CAN_RST   A3    // ATMEGA Pin 26, PC3, ADC3
#define CAN_INT   A4    // ATMEGA Pin 27, PC4, ADC4

#define LED_0   A5  // ATMEGA Pin 28, PC5
#define LED_1   7   // ATMEGA Pin 13, PD7
#define LED_2   0   // ATMEGA Pin 2, PD0
#define LED_3   1   // ATMEGA Pin 3, PD1

// #define MTR_INA     // MTR_2, ATMEGA Pin 9, PB6
// #define MTR_INB     // MTR_3, ATMEGA Pin 10, PB7
#define MTR_PWMA  10  // MTR_1, ATMEGA Pin 16, PB2
#define MTR_PWMB  9   // MTR_0, ATMEGA Pin 15, PB1
#define MTR_CS    4   // MTR_C, ATMEGA Pin 6, PD4


#define ANL_0   A0  // ATMEGA Pin 23, PC0, ADC0
#define ANL_1   A1  // ATMEGA Pin 24, PC1, ADC1
#define DIG_0   6   // ATMEGA Pin 12, PD6
#define DIG_1   5   // ATMEGA Pin 11, PD5
#define DIG_2   3   // ATMEGA Pin 5, PD3, INT1
#define DIG_3   2   // ATMEGA Pin 4, PD2, INT0

// #define SPARE      // Spare pin on ATMEGA pin 14, PB0

// Initialize these methods
void initCAN();
int loopCAN();
int sendCANMessage(byte data[], int dataLength);



int step_time = 50;
long last_time = millis() + step_time;
int count = 0;


// Initialize CAN Stuff
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
// Array to store serial string
char msgString[128];
MCP_CAN CAN0(CAN_CS);     // Set CS to pin 10

Encoder enc(DIG_2, DIG_3);

void setup(){
  Serial.begin(115200);
  analogReference(DEFAULT);
  // Initialize LEDs
  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);

  // Blink one led to verify that the chip is working
  digitalWrite(LED_1, HIGH);
  delay(500);
  digitalWrite(LED_1, LOW);

  initCAN();
}

byte tempData[8] = {0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned long lastsend = 0;
unsigned long lastreceive = 0;
int lastenc = 0;
int airmarval = 0;

void loop() {
  int newMessages = loopCAN();
  // Blink LED 0 once every 1 second
  if(last_time < millis()){
    last_time += step_time;

    digitalWrite(LED_0, count++ % 2);

    *(unsigned int*)(tempData) = millis();
    *(int*)(tempData+2) = enc.read();
    *(int*)(tempData+4) = airmarval;
    *(int*)(tempData+6) = lastenc;
   // Serial.println(enc.read());
    
    int response = sendCANMessage(0x94ff0315, tempData, 8);
    lastsend = micros();
  }
}


// Initialize the CAN Stuff
void initCAN(){
  // set chip reset to be high (on)
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_16MHZ) == CAN_OK){
  } else {
    Serial.println("Error Initializing MCP2515...");
  }

  // Set operation mode to normal so the MCP2515 sends acks to received data.
  CAN0.setMode(MCP_NORMAL);

  // Configuring pin for /INT input
  pinMode(CAN_INT, INPUT);
}

// Call this in the main loop
// It checks for new CAN Messages
// Return the number of CAN Messages
int loopCAN(){
  int numNewMessages = 0;

  // Read any messages
  // If CAN_INT pin is low, read receive buffer
  if(!digitalRead(CAN_INT)){
    numNewMessages++;

    // Read data: len = data length, buf = data byte(s)
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    // Determine if ID is standard (11 bits) or extended (29 bits)
    if((rxId & 0x80000000) == 0x80000000){
      // Attitude
      if (false && (rxId & 0x00FFFF00) == 0xF11900 && len >= 2) {
        // Pitch + Roll calculations are equivalent
        //airmarval = *(int*)(rxBuf+3); // pitch
        airmarval = (unsigned)rxBuf[5] + ((unsigned)rxBuf[6] << 8); // roll
        lastenc = enc.read();
        lastreceive = micros();
      }
      // Heading
      if (false && (rxId & 0x00FFFF00) == 0xF11200 && len >= 2) {
        airmarval = *(int*)(rxBuf+1);
        lastenc = enc.read();
        lastreceive = micros();
      }
      // Rate of Turn
      if ((rxId & 0x00FFFF00) == 0xF11300 && len >= 2) {
        airmarval = *(int*)(rxBuf+3);
        lastenc = enc.read();
        lastreceive = micros();
      }
    }
  }

  return numNewMessages;
}


// Send a CAN Message
// Return 1 on success, 0 otherwise
int sendCANMessage(long id, byte data[], int dataLength){
  // send data:  ID = 0x94101315, Extended CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = CAN0.sendMsgBuf(id, 1, 8, data);
  if(sndStat == CAN_OK){
    return 1;
  } else {
    return 0;
  }
}















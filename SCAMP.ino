#include "mcp_can.h"
#include <SPI.h>
#include <ServoTimer2.h>

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

#define DIG_2 3 // ATMEGA Pin 5, PD3, INT1

// Prototypes
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
MCP_CAN CAN0(CAN_CS);     // Set CS to pin 10

const int kRudderPort = DIG_2;
ServoTimer2 rudder;

void setup(){
  #ifdef __DEBUG_PRINT__
  Serial.begin(115200);
  Serial.println("Testing");
  #endif

  analogReference(DEFAULT);
  rudder.attach(kRudderPort);
  rudder.write(DEFAULT_PULSE_WIDTH);

  // Initialize LEDs
  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);

  // Blink one led to verify that the chip is working
  digitalWrite(LED_1, HIGH);
  delay(500);
  digitalWrite(LED_1, LOW);

  initCAN();
}

void loop(){
  int newMessages = loopCAN();

  #ifdef __DEBUG_PRINT__
  if(newMessages){
    Serial.println("New messages found");
  }
  #endif

  // Blink LED 0 once every 1 second
  if(last_time < millis()){
    last_time += step_time;
    digitalWrite(LED_0, count++ % 2);
  }
}


// Initialize the CAN Stuff
void initCAN(){
  // set chip reset to be high (on)
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_16MHZ) == CAN_OK){
    #ifdef __DEBUG_PRINT__
    Serial.println("MCP2515 Initialized Successfully!");
    #endif
  } else {
    #ifdef __DEBUG_PRINT__
    Serial.println("Error Initializing MCP2515...");
    #endif
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
      if (rxId == 0x94ff0215 && len >= 2) {
        rudder.write(map(rxBuf[1], 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
        #ifdef __DEBUG_PRINT__
        Serial.print("Setting winch, rudder, ballast: ");
        Serial.print(rxBuf[0]);
        Serial.print(", ");
        Serial.print(rxBuf[1]);
        Serial.print(", ");
        Serial.println(rxBuf[2]);
        #endif
      }
    }
  }

  return numNewMessages;
}


// Send a CAN Message
// Return 1 on success, 0 otherwise
int sendCANMessage(byte data[], int dataLength){
  // send data:  ID = 0x94101315, Extended CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = CAN0.sendMsgBuf(0x94ff0115, 1, 8, data);
  if(sndStat == CAN_OK){
    return 1;
  } else {
    return 0;
  }
}

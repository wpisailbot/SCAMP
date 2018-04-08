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

// #define MTR_INA     // MTR_2, ATMEGA Pin 9, PB6
// #define MTR_INB     // MTR_3, ATMEGA Pin 10, PB7
//#define MTR_PWMA  10  // MTR_1, ATMEGA Pin 16, PB2
//#define MTR_PWMB  9   // MTR_0, ATMEGA Pin 15, PB1
//#define MTR_CS    4   // MTR_C, ATMEGA Pin 6, PD4


#define ANL_0   A0  // ATMEGA Pin 23, PC0, ADC0
#define ANL_1   A1  // ATMEGA Pin 24, PC1, ADC1
#define DIG_0   6   // ATMEGA Pin 12, PD6
#define DIG_1   5   // ATMEGA Pin 11, PD5
#define DIG_2   3   // ATMEGA Pin 5, PD3, INT1
#define DIG_3   2   // ATMEGA Pin 4, PD2, INT0

#define PIN_FWD 8   //PB0
#define PIN_BWD 7   //PD7
#define PIN_PWM 10   //PB2
#define PIN_CURR 5  //ADC5
#define CURRENT_SENSE_SCALE 1
#define T_DWN1 1
#define T_DWN2 1

// #define SPARE      // Spare pin on ATMEGA pin 14, PB0

// Initialize these methods
void initCAN();
int loopCAN();
int sendCANMessage(byte data[], int dataLength);
void initMotor(uint8_t fwd_pin, uint8_t bwd_pin, uint8_t pwm_pin, uint8_t curr_pin, uint8_t fault1_pin, uint8_t fault2_pin);
void setMotor(int8_t power);
uint8_t isMotorFault();
float getCurrent();
uint8_t parseInt(int16_t* val);


int step_time = 50;
long last_time = millis() + step_time;
int count = 0;

// Initialize stuff for motor controller


// Initialize CAN Stuff
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
// Array to store serial string
char msgString[128];
MCP_CAN CAN0(CAN_CS);     // Set CS to pin 10

const int kWinchPort = DIG_1;
const int kRudderPort = DIG_2;
ServoTimer2 winch, rudder;

void setup(){
  Serial.begin(9600);
  Serial.println("Testing");
  analogReference(DEFAULT);
  //winch.attach(kWinchPort);
  rudder.attach(kRudderPort);
  //winch.write(DEFAULT_PULSE_WIDTH);
  rudder.write(DEFAULT_PULSE_WIDTH);

  // Initialize LEDs
  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);

  // Blink one led to verify that the chip is working
  digitalWrite(LED_1, HIGH);
  delay(500);
  digitalWrite(LED_1, LOW);

  initCAN();
  //initMotor(PIN_FWD, PIN_BWD, PIN_PWM, PIN_CURR, T_DWN1, T_DWN2);
}

byte tempData[8] = {0x00, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void loop(){
  int newMessages = loopCAN();

  if(newMessages){
    Serial.println("New messages found");
  }

  int16_t my_power = 0;
  //Read Serial values for motor power testing
  /*if(parseInt(&my_power)) {
    Serial.print("Received Power: ");
    Serial.println(int16_t(my_power));
    setMotor(my_power);
  }*/

  // Echo serial communications
//   if(Serial.available()>0) {
//     //digitalWrite(LED_1, HIGH);
//     Serial.write(Serial.read());
//     delay(10);
//   }
  // digitalWrite(LED_1, LOW);




  // Blink LED 0 once every 1 second
  if(last_time < millis()){
    last_time += step_time;

    digitalWrite(LED_0, count++ % 2);
//    tempData[0] = 0x00;
//    tempData[1] = 0x01;
//    tempData[2] = 0x02;
//    tempData[3] = 0x03;
//    tempData[4] = count & 0xff000000;
//    tempData[5] = count & 0x00ff0000;
//    tempData[6] = count & 0x0000ff00;
//    tempData[7] = count & 0x000000ff;

    // Package analog read. Note that we divide by 4 to package into 8 bits
    //tempData[0] = (byte) (analogRead(A0) / 4);
    //tempData[0] = (byte) 200;  //Magic value to trick the code into thinking the winch is in a safe position for testing
    /*int response = sendCANMessage(tempData, 8);
    if(response){
      Serial.println("Message sent");
    } else {
      Serial.println("Message fail");
    }*/
  }
}






// Initialize the CAN Stuff
void initCAN(){
  // set chip reset to be high (on)
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_STDEXT, CAN_250KBPS, MCP_16MHZ) == CAN_OK){
    Serial.println("MCP2515 Initialized Successfully!");
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
      if (rxId == 0x94ff0215 && len >= 2) {
        winch.write(map(rxBuf[0], 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
        rudder.write(map(rxBuf[1], 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH));
        setMotor(rxBuf[2]-90);
        Serial.print("Setting winch, rudder, ballast: ");
        Serial.print(rxBuf[0]);
        Serial.print(", ");
        Serial.print(rxBuf[1]);
        Serial.print(", ");
        Serial.println(rxBuf[2]);
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

void initMotor(uint8_t fwd_pin, uint8_t bwd_pin, uint8_t pwm_pin, uint8_t curr_pin, uint8_t fault1_pin, uint8_t fault2_pin) {
  pinMode(fwd_pin, OUTPUT);
  digitalWrite(fwd_pin, LOW);
  pinMode(bwd_pin, OUTPUT);
  digitalWrite(bwd_pin, LOW);
  analogWrite(pwm_pin, 0);
  pinMode(fault1_pin, INPUT);
  pinMode(fault2_pin, INPUT);
}



void setMotor(int8_t power) {
  if (power < 0) {
    //Go Forward
    digitalWrite(PIN_BWD, LOW);
    digitalWrite(PIN_FWD, HIGH);
    Serial.print("Writing Forward ");
    Serial.println(uint8_t (abs(power)*2));
    analogWrite(PIN_PWM, uint8_t(abs(power)*2));
        
  } else if (power>0) {
    //Go Backward
    digitalWrite(PIN_FWD, LOW);
    digitalWrite(PIN_BWD, HIGH);
    Serial.print("Writing Backward ");
    Serial.println(uint8_t (abs(power)*2));
    analogWrite(PIN_PWM, uint8_t(abs(power)*2));
  } else {
    //Brake/Coast
    digitalWrite(PIN_FWD, LOW);
    digitalWrite(PIN_BWD, LOW);
    analogWrite(PIN_PWM, 0);
    Serial.println("Writing Brake");
  }
}

uint8_t parseInt(int16_t* val) {
  static char state = 0;
  static char negative = 0;
  char c;  //temp buffer
  while (Serial.available()>0) {
    switch(state) {
      case 0:  //Find the begin character
        if(Serial.read() == 'p') {
          negative = 0;
          state = 1;
          *val = 0;
        }
        break;
      case 1:  //Parse the sign
        if (Serial.peek() == '-') {
          negative = 1;
          Serial.read();  //Throw away the sign character
        } else if (isDigit(Serial.peek())) {
          state = 2; 
        } else {
          //Garbage data following p
          Serial.read();
          state = 0;
        }
        break;
      case 2:  //Parse the digit
        c = Serial.read();
        if (isDigit(c)) {
          *val = (*val)*10 + (c-'0');
        } else if (c == '\n' || c=='\r') {
          if (negative) {
            *val = *val * -1;
          }
          state = 0;
          return true;
        } else {
          //Garbage data
          state = 0;
        }
        break;
      default:
        state = 0;
        break;
    }
    delay(10);
  }
  return false;
}



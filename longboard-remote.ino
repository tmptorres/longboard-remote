/*
   Dec 2014 - TMRh20 - Updated
   Derived from examples by J. Coliz <maniacbug@ymail.com>
*/
/**
   Example for efficient call-response using ack-payloads

   This example continues to make use of all the normal functionality of the radios including
   the auto-ack and auto-retry features, but allows ack-payloads to be written optionlly as well.
   This allows very fast call-response communication, with the responding radio never having to
   switch out of Primary Receiver mode to send back a payload, but having the option to switch to
   primary transmitter if wanting to initiate communication instead of respond to a commmunication.
*/

/* TODO:
 *  
 */

// Arduino
#include <Servo.h>		// Disables analogWrite() on pins 9 and 10

// Github.com: nRF24/RF24
#include <SPI.h>
#include "RF24.h"
#include <printf.h>

// Github.com: RollingGecko/VescUartControl
#include "src/VescUartControl/VescUart.h"
#include "src/VescUartControl/datatypes.3.40.h"

// My messages
#include "structures.h"
#include "typeSwitch.h"

/* ---------------------------- Configs/Inits ---------------------------- */

// Controler Configs
#define CONTROLER_LED1_PIN 3                // Led 1: Link Up - Red
#define CONTROLER_LED2_PIN 5                // Led 2: Battery - Green
#define CONTROLER_LED3_PIN 6                // Led 3: Communication - Blue
#define CONTROLER_BUTTON1_PIN 4             // Extra button on board
#define CONTROLER_JOY_BUTTON_PIN A0
#define CONTROLER_JOY_AXIS1_PIN A1
#define CONTROLER_JOY_AXIS2_PIN A2

#define CONTROLRATE 100                      // 100Hz (Servo PWM Signals are 50Hz)
#define CONTROLER_ACCEL_CURRENT 40.0        // Amps
#define CONTROLER_BRAKE_CURRENT -20.0       // Amps
#define CONTROLER_POSITIVE_RAMP_TIME 2      // In seconds
#define CONTROLER_NEGATIVE_RAMP_TIME 1      // In seconds
#define CONTROLER_CURRENT_MAXSLEWRATE CONTROLER_ACCEL_CURRENT / ( CONTROLRATE * CONTROLER_POSITIVE_RAMP_TIME*1.0 )
#define CONTROLER_CURRENT_MINSLEWRATE CONTROLER_BRAKE_CURRENT / ( CONTROLRATE * CONTROLER_NEGATIVE_RAMP_TIME*1.0 )


// Radio Configs
rf24_datarate_e dataRate = RF24_250KBPS;      // Datarate. Options: {1MBPS, 2MBPS, 250KBPS}
uint8_t channelOffset = 105;                  // Range: 0 - 125. Frequency is 2.4Ghz + channelOffset * 1MHz
rf24_crclength_e crcSize = RF24_CRC_8;        // CRC length. Options: {DISABLED, 8, 16}. Use 8-bit CRC for performance

byte addresses[][6] = { "~Tmp^", "^Tmp~"};   // Radio pipe addresses for the 2 nodes to communicate.
#ifdef RADIO_CONTROLER
  byte *localAddress  = addresses[0];
  byte *remoteAddress = addresses[1];
#endif
#ifdef RADIO_RECIEVER
  byte *localAddress  = addresses[1];
  byte *remoteAddress = addresses[0];
#endif

#ifdef DEBUG
  uint8_t PALevel = RF24_PA_MIN;              // Options {MIN,LOW,HIGH,MAX}. Default is RF24_PA_MAX
#else
  uint8_t PALevel = RF24_PA_MIN;
#endif

// Reciever Configs
#define SERVO_LED1_PIN 3
#define SERVO_PWM_PIN1 6
#define SERVO_PWM_PIN2 5
#define SERVO_MINPULSE 1000         // Minimum pulse witdth in microseconds (us)
#define SERVO_MAXPULSE 2000         // Maximum pulse witdth in microseconds (us)
#define SERVO_TIMEOUT_MS 500        // Timeout in ms before applying brakes
#define SERVO_TIMEOUT_BRAKE_PULSE 500 // Timeout brake pulse
// The following can only be defined after the min and max pulse are defined
#define CONTROLER_PULSE_MAXSLEWRATE (SERVO_MAXPULSE-SERVO_MINPULSE) / ( CONTROLRATE * CONTROLER_POSITIVE_RAMP_TIME )
#define CONTROLER_PULSE_MINSLEWRATE -SERVO_MINPULSE / ( CONTROLRATE * CONTROLER_NEGATIVE_RAMP_TIME )

// Initializations/Allocations
#ifdef RADIO_RECIEVER
  Servo servo1;
  Servo servo2;
#endif

// Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8
#define NRF24_CE_PIN 7  // Chip Enable pin
#define NRF24_CSN_PIN 8 // Chip Select pin
RF24 radio(NRF24_CE_PIN, NRF24_CSN_PIN);


/* ---------------------------- Setup ---------------------------- */

int controlerAxis1Neutral = 512;
int controlerAxis2Neutral = 512;

void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    printf_begin();  // This enables the program's calls to Serial.print to be directed to the Arduino's IDE serial monitor
                     // Used in radio.printDetails()
    Serial.println(F("Beggining"));
    Serial.print(F("Sizeof packet2Recv: ")); Serial.print( sizeof(packet2Recv) ); Serial.println(F(" bytes"));
    Serial.print(F("Sizeof packet2Cont: ")); Serial.print( sizeof(packet2Cont) ); Serial.println(F(" bytes"));
    
  #elif defined RADIO_RECIEVER
    SERIALIO.begin(115200);
  #endif

  // Pin Initializations
  #ifdef RADIO_CONTROLER
    pinMode(CONTROLER_LED1_PIN, OUTPUT);
    pinMode(CONTROLER_LED2_PIN, OUTPUT);
    pinMode(CONTROLER_LED3_PIN, OUTPUT);
    pinMode(CONTROLER_BUTTON1_PIN, INPUT_PULLUP);
    pinMode(CONTROLER_JOY_BUTTON_PIN, INPUT_PULLUP);
    
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Loop here to average measurements
    controlerAxis1Neutral = analogRead(CONTROLER_JOY_AXIS1_PIN);
    controlerAxis2Neutral = analogRead(CONTROLER_JOY_AXIS2_PIN);
  #endif
  #ifdef RADIO_RECIEVER
    pinMode(SERVO_LED1_PIN, OUTPUT);
    servo1.attach(SERVO_PWM_PIN1, SERVO_MINPULSE, SERVO_MAXPULSE);
    servo2.attach(SERVO_PWM_PIN2, SERVO_MINPULSE, SERVO_MAXPULSE);
  #endif

  // Setup and configure radios
  radio.begin();
  radio.setChannel(channelOffset);
  radio.setPALevel(PALevel);
  radio.setDataRate(dataRate);
  radio.setCRCLength(crcSize);

  // Set payload sizes to avoid sending packets padded with zeros in the end
  #ifdef RADIO_CONTROLER
    radio.setPayloadSize(sizeof(packet2Recv));
  #endif
  #ifdef RADIO_RECIEVER
    radio.setPayloadSize(sizeof(packet2Cont));
  #endif
  radio.setPayloadSize( max(sizeof(packet2Recv), sizeof(packet2Cont)) );

  radio.enableAckPayload();                     // Allow optional ack payloads
  radio.enableDynamicPayloads();                // Ack payloads are dynamic payloads

  radio.openWritingPipe(remoteAddress);         // Both radios listen on the same pipes by default, but opposite addresses
  radio.openReadingPipe(1, localAddress);       // Open a reading pipe on the address and pipe 1
  radio.startListening();                       // Start listening

  #ifdef RADIO_RECIEVER
    packet2Cont packetSend;                     // Preload Ack packet
    packetSend.pingCounter = 0;
    
    radio.writeAckPayload(1, &packetSend, sizeof(packet2Cont) );
    #ifdef DEBUG
      Serial.println(F("Preloaded ACK response: "));
    #endif
  #endif

  #ifdef DEBUG
    radio.printDetails();                       // Needs printf_begin() in setup
  #endif
}

/* ---------------------------- Controler ---------------------------- */
#ifdef RADIO_CONTROLER

bool handbrake = false;
float currentDelayed = 0;
int pulseDelayed = (SERVO_MAXPULSE + SERVO_MINPULSE)/2;

void loop(void) {
  packet2Recv packetSend;
  packet2Cont packetRecv;
  int analogValue = controlerAxis2Neutral;
  
  unsigned long time1 = micros();                // Record the current microsecond count

  // Build packet to send
  #ifdef DEBUG
    Serial.print(F("Button: ")); Serial.print(digitalRead(CONTROLER_BUTTON1_PIN));
    Serial.print(F(", Button Joystick: ")); Serial.print(digitalRead(CONTROLER_JOY_BUTTON_PIN));
  #endif

  // Make as an interrupt
//  if( !digitalRead(CONTROLER_BUTTON1_PIN) ){
//    if( handbrake ){
//      handbrake = false;
//    }else{
//      handbrake = true;
//    }
//  }
  
  switch( 0 ){
    
    case 0:
      // PPM control
      packetSend.mode = 0;
      if( !handbrake ){
        analogValue = analogRead(CONTROLER_JOY_AXIS1_PIN);
        packetSend.servoPulse = map( analogValue, 0, 1023, SERVO_MINPULSE, SERVO_MAXPULSE);
      }else{
        packetSend.servoPulse = SERVO_MINPULSE;
      }

      // Filter the pulse sent
      packetSend.servoPulse = pulseSlewRateFilter(packetSend.servoPulse, pulseDelayed , CONTROLER_PULSE_MAXSLEWRATE, CONTROLER_PULSE_MINSLEWRATE);
      pulseDelayed = packetSend.servoPulse;
      
      #ifdef DEBUG
        Serial.print(F(", analogRead: ")); Serial.print(analogValue);
        Serial.print(F(", Sent pulse: ")); Serial.println(packetSend.servoPulse);
      #endif
      break;
      
    case 1:
      // Current control
      packetSend.mode = 1;
      if( !handbrake ){
        analogValue = analogRead(CONTROLER_JOY_AXIS1_PIN);
        if( analogValue >= controlerAxis2Neutral ){
          packetSend.current = map2Float( analogValue, controlerAxis2Neutral, 1023, 0, CONTROLER_ACCEL_CURRENT);
        }else{
          packetSend.current = map2Float( analogValue, 0, controlerAxis2Neutral-1, CONTROLER_BRAKE_CURRENT, 0);
        }
      }else{
        packetSend.current = CONTROLER_BRAKE_CURRENT;
      }
      // Filter the current sent
      packetSend.current = currentSlewRateFilter(packetSend.current, currentDelayed , CONTROLER_CURRENT_MAXSLEWRATE, CONTROLER_CURRENT_MINSLEWRATE);
      currentDelayed = packetSend.current;
      
      #ifdef DEBUG
        Serial.print(F(", analogRead: ")); Serial.print(analogValue);
        Serial.print(F(", Sent current: ")); Serial.println(packetSend.current);
      #endif
      break;
  }

  // Send
  radio.stopListening();                        // First, stop listening so we can talk.
  if ( radio.write(&packetSend, sizeof(packet2Recv)) ) {
    // If sent
    digitalWrite(CONTROLER_LED3_PIN,HIGH);
    #ifdef DEBUG
      Serial.println(F("Sent success."));
    #endif

    if ( radio.available() ) {
      while ( radio.available() ) {             // If an ack with payload was received
        
        digitalWrite(CONTROLER_LED1_PIN,HIGH);
        radio.read(&packetRecv, sizeof(packet2Cont));         // Read it, and display the response time

        #ifdef DEBUG
          Serial.print(F("Got response: ")); Serial.println(packetRecv.pingCounter);
          Serial.print(F("Round-trip delay: "));
          Serial.print(micros() - time1);
          Serial.println(F(" microseconds"));
        #endif
      }
    } else {
      // If nothing in the buffer, we got an ack but it is blank
      digitalWrite(CONTROLER_LED1_PIN,LOW);
      
      #ifdef DEBUG
        Serial.print(F("Got blank response. round-trip delay: "));
        Serial.print(micros() - time1);
        Serial.println(F(" microseconds"));
      #endif
    }

  } else {
    digitalWrite(CONTROLER_LED1_PIN,LOW);
    
    #ifdef DEBUG
      Serial.println(F("Sending failed."));   // If no ack response, sending failed
    #endif
  }
  
  delay( max(0, 1e3 / CONTROLRATE - 1e-3*(micros() - time1) ) );
  digitalWrite(CONTROLER_LED3_PIN, LOW);
}
#endif


/****************** Servo Role ***************************/
// Refrain from running the receiver in DEBUG mode while
// connected to the Vesc, as the serial interface is the
// same. Thus the Vesc receives all the debug text !!!

#ifdef RADIO_RECIEVER
void loop(void) {
  packet2Recv packetRecv;
  packet2Cont packetSend;
  byte pipeNo;
  //unsigned long time = micros();                              // Record the current microsecond count
  uint32_t pingCounter = 0;
  unsigned long time2 = micros();  

  while ( true ) {

    digitalWrite(SERVO_LED1_PIN,LOW);
    // Check if there is a time out
    if(micros() - time2 > SERVO_TIMEOUT_MS * 1e3){
      servo1.writeMicroseconds( SERVO_TIMEOUT_BRAKE_PULSE );
      servo2.writeMicroseconds( SERVO_TIMEOUT_BRAKE_PULSE );
      #ifdef DEBUG
        Serial.println(F("Timed out! Applying brakes!!!"));
      #endif
    }
    
    while ( radio.available(&pipeNo) ) {          // Read all available payloads
      
      radio.read( &packetRecv, sizeof(packet2Recv) );
      time2 = micros();

      digitalWrite(SERVO_LED1_PIN,HIGH);

      switch( packetRecv.mode ){
        case 0:
          // PPM control
          servo1.writeMicroseconds( packetRecv.servoPulse );
          servo2.writeMicroseconds( packetRecv.servoPulse );
          #ifdef DEBUG
            Serial.print(F("Mode 0. Recieved pulse: "));
            Serial.println(packetRecv.servoPulse);
          #endif
          break;
        case 1:
          // Current control
          if( packetRecv.current >= 0 ){
            VescUartSetCurrent( packetRecv.current );
          }else{
            VescUartSetCurrentBrake( -packetRecv.current );
          }
          #ifdef DEBUG
            Serial.print(F("Mode 1. Recieved current: "));
            Serial.println(packetRecv.servoPulse);
          #endif
          break;
      }

      // MISSING READING VALUES FROM VESC!!!!!
      // IN THIS VERSION NOTHING IS BEING FED BACK
      // the function for this should be -> VescUartGetValue( packetSend.values );

      packetSend.pingCounter = ++pingCounter;

      // Since this is a call-response. Respond directly with an ack payload.
      // Ack payloads are much more efficient than switching to transmit mode to respond to a call
      radio.writeAckPayload(pipeNo, &packetSend, sizeof(packet2Cont) ); // This can be commented out to send empty payloads.
      #ifdef DEBUG
        Serial.print(F("Sent: "));
        Serial.println(packetSend.pingCounter);
      #endif
    }
  }
}
#endif

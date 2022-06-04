// Here we go.  It is the test version of it.
// -*- mode: C++ -*-
// Based off the Adafruit of the rf69 demo tx rx.pde
// 
// 

#include <Servo.h>
#include <SPI.h>
#include <ESC.h>

// TO WORK THIS SCRIPT YOU WILL NEED TO UPDATE RH_ASK.h to uncomment the line of 
// RH_ASK.cpp that states "//#define RH_ASK_ARDUINO_USE_TIMER2".  This will allow RH_Ask to use timer two.
// This is because the Arduino libraries use the timer 1 and are pretty picky about it apparently.
// This is why it gets 
#include <RadioHead.h>
#include <radio_config_Si4460.h>
#include <RHCRC.h>
#include <RHDatagram.h>
#include <RHEncryptedDriver.h>
#include <RHGenericDriver.h>
#include <RHGenericSPI.h>
#include <RHHardwareSPI.h>
#include <RHMesh.h>
#include <RHNRFSPIDriver.h>
#include <RHReliableDatagram.h>
#include <RHRouter.h>
#include <RHSoftwareSPI.h>
#include <RHSPIDriver.h>
#include <RHTcpProtocol.h>
#include <RH_ASK.h>
#include <RH_CC110.h>
#include <RH_E32.h>
#include <RH_MRF89.h>
#include <RH_NRF24.h>
#include <RH_NRF51.h>
#include <RH_NRF905.h>
#include <RH_RF22.h>
#include <RH_RF24.h>
#include <RH_RF69.h>
#include <RH_RF95.h>
#include <RH_Serial.h>
#include <RH_TCP.h>








/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match TX's freq!
#define RF69_FREQ 915.0

// who am i? (server address)
#define MY_ADDRESS     1


#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     9
  #define LED           13
#endif

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
#endif
/*
#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
#endif
*/
/* Teensy 3.x w/wing
#define RFM69_RST     9   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     4    // "C"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/
 
/* WICED Feather w/wing 
#define RFM69_RST     PA4     // "A"
#define RFM69_CS      PB4     // "B"
#define RFM69_IRQ     PA15    // "C"
#define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);


int8_t packetnum = 0;  // packet counter, we increment per xmission

// Stuff for determining the time since last received transmission
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

///////// END OF RADIO SET UP.  BEGIN CONTROLLER SET UP AND DATA DEF. 

uint8_t l_stick_def_val;
uint8_t r_stick_def_val;
uint8_t tLPos_def_val;
uint8_t tRPos_def_val;
uint8_t t1Pos_def_val;

// Ways to write out to Serial the values received.
char l_stick_val_char[4];
char r_stick_val_char[4];
char tLPosVal_char[3];
char tRPosVal_char[3];
char turButt_char[3];


/////////////// Start of Receiver's control of other components.

// 
// For bidirectional ESC, Neutral = 1500, full forward = 2000, full aft = 1000

int max_aft_pulse_val=1200;
int max_fwd_pulse_val=1700;


// The Pins that will be used. 
uint8_t left_esc_pin=6;
uint8_t left_laserPin=;
uint8_t right_esc_pin=9;


ESC left_esc (left_laserPin,max_aft_pulse_val , max_fwd_pulse_val, 500);
ESC right_esc (right_esc_pin,max_aft_pulse_val , max_fwd_pulse_val, 500);


void setup() 
{
  Serial.begin(115200);
  //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather Addressed RFM69 RX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);

  //
  // Setting initial values of joysticks and buttons.
  l_stick_def_val = 127; // Left Stick Position
  r_stick_def_val = 127; // Right Stick Position
  tLPos_def_val=1; // Left Button Status
  tRPos_def_val=1; // Right Button Status
  t1Pos_def_val=1; // Center Button 1 Status
  //
  
  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}


// Dont put this on the stack:
uint8_t data[] = "Data received";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop() {
  if (rf69_manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      lastReceiveTime = millis();
      buf[len] = 0; // zero out remaining string
      
      itoa(buf[0],l_stick_val_char,10);
      itoa(buf[1], r_stick_val_char, 10);
      Serial.print("Got packet from #"); Serial.println(from);
      Serial.print(" [RSSI :");
      Serial.print(rf69.lastRssi());
      Serial.println("] : ");
      analogWrite(left_laserPin , buf[0]);
      analogWrite(right_laserPin , buf[1]);
      Serial.print("Left stick value of : "); Serial.println(l_stick_val_char);
      Serial.print("Right stick value of : "); Serial.println(r_stick_val_char);
      
      Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks

      // Send a reply back to the originator client
  if (!rf69_manager.sendtoWait(data, sizeof(data), from))
        Serial.println("Sending failed (no ack)");
      }
  }
  
  currentTime=millis();
  if ((currentTime - lastReceiveTime)>1000) {
    Serial.println("Setting inputs to Defaults");
    delay(50);
  }

  
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}

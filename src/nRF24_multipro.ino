/*
 ******************************************************************************
 This is a fork of the Multi-Protocol nRF24L01 Tx project
 from goebish on RCgroups / github
 This version accepts serial port strings and converts
 them to ppm commands which are then transmitted via
 the nRF24L01. 

 The purpose of this code is to enable control over the Cheerson CX-10 
 drone via code running on a PC.  In my case, I am developing Python code to
 fly the drone. 
 
 This code can be easily adapted to the other mini-drones 
 that the Multi-protocol board supports. 

 The format for the serial command is:
 ch1value,ch2value,ch3value,...
 e.g.:  1500,1800,1200,1100, ...

 Up to 12 channel commands can be submitted. The channel order is defined
 by chan_order. The serial port here is running at 115200bps. 

 Python code in serial_test.py was written to generate the serial strings.  

 Hardware used:
 This code was tested on the Arduino Uno and nRF24L01 module. 
 Wiring diagrams and more info on this project at www.makehardware.com/pc-mini-drone-controller.html
 
 I believe this code will remain compatible with goebish's 
 nRF24L01 Multi-Protocol board.  A way to 
 connect to the serial port will be needed (such as the FTDI). 
 
 Perry Tsao 29 Feb 2016
 perrytsao on github.com
 *********************************************************************************

 
 ##########################################
 #####   MultiProtocol nRF24L01 Tx   ######
 ##########################################
 #        by goebish on rcgroups          #
 #                                        #
 #   Parts of this project are derived    #
 #     from existing work, thanks to:     #
 #                                        #
 #   - PhracturedBlue for DeviationTX     #
 #   - victzh for XN297 emulation layer   #
 #   - Hasi for Arduino PPM decoder       #
 #   - hexfet, midelic, closedsink ...    #
 ##########################################


 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License.
 If not, see <http://www.gnu.org/licenses/>.
 */

#include <util/atomic.h>
#include <EEPROM.h>
#include "iface_nrf24l01.h"
#include <string.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

extern "C" {
  #include "utility/twi.h"
}

/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel1 = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag1   = Adafruit_LSM303_Mag_Unified(30302); 

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel2 = Adafruit_LSM303_Accel_Unified(30303);
Adafruit_LSM303_Mag_Unified   mag2   = Adafruit_LSM303_Mag_Unified(30304); 

// ############ Wiring ################
#define PPM_pin   2  // PPM in
//SPI Comm.pins with nRF24L01
#define MOSI_pin  3  // MOSI - D3
#define SCK_pin   4  // SCK  - D4
#define CE_pin    5  // CE   - D5
#define MISO_pin  A0 // MISO - A0
#define CS_pin    A1 // CS   - A1
#define TCAADDR   0x70

#define ledPin    13 // LED  - D13

// SPI outputs
#define MOSI_on PORTD |= _BV(3)  // PD3
#define MOSI_off PORTD &= ~_BV(3)// PD3
#define SCK_on PORTD |= _BV(4)   // PD4
#define SCK_off PORTD &= ~_BV(4) // PD4
#define CE_on PORTD |= _BV(5)    // PD5
#define CE_off PORTD &= ~_BV(5)  // PD5
#define CS_on PORTC |= _BV(1)    // PC1
#define CS_off PORTC &= ~_BV(1)  // PC1
// SPI input
#define  MISO_on (PINC & _BV(0)) // PC0

#define RF_POWER TX_POWER_80mW 

// PPM stream settings
#define CHANNELS 12 // number of channels in ppm stream, 12 ideally
enum chan_order{
    THROTTLE,
    AILERON,
    ELEVATOR,
    RUDDER,
    AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  still camera (snapshot)
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
    AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7)
    AUX8,  // (CH12) Reset / Rebind
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)

// supported protocols
enum {
    PROTO_V2X2 = 0,     // WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
    PROTO_CG023,        // EAchine CG023, CG032, 3D X4
    PROTO_CX10_BLUE,    // Cheerson CX-10 blue board, newer red board, CX-10A, CX-10C, Floureon FX-10, CX-Stars (todo: add DM007 variant)
    PROTO_CX10_GREEN,   // Cheerson CX-10 green board
    PROTO_H7,           // EAchine H7, MoonTop M99xx
    PROTO_BAYANG,       // EAchine H8(C) mini, H10, BayangToys X6, X7, X9, JJRC JJ850, Floureon H101
    PROTO_SYMAX5C1,     // Syma X5C-1 (not older X5C), X11, X11C, X12
    PROTO_YD829,        // YD-829, YD-829C, YD-822 ...
    PROTO_H8_3D,        // EAchine H8 mini 3D, JJRC H20, H22
    PROTO_END
};

// EEPROM locations
enum{
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};

uint16_t overrun_cnt=0;
uint8_t transmitterID[4];
uint8_t current_protocol;
static volatile bool ppm_ok = false;
uint8_t packet[32];
static bool reset=true;
volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
char *p, *i;
char* c = new char[200 + 1]; // match 200 characters reserved for inputString later
char* errpt;
uint8_t ppm_cnt;

// vars to send to drone
int throttle, aileron, elevator, rudder;

// IMU data
float roll, throttle_pitch, elevator_pitch, yaw;

void tcaselect(uint8_t i)
{
  if(i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();

}

void setup()
{
    
    randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));
    pinMode(PPM_pin, INPUT);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);

    // PPM ISR setup
    //attachInterrupt(PPM_pin - 2, ISR_ppm, CHANGE);
    TCCR1A = 0;  //reset timer1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz

    set_txid(false);

    // Serial port input/output setup
    Serial.begin(115200);
    // reserve 200 bytes for the inputString:
    //inputString.reserve(200);
      Wire.begin();
      for(uint8_t t =0; t<8; t++)
      {
        tcaselect(t);
        Serial.print("TCA Port #"); Serial.println(t);

        for(uint8_t addr = 0; addr <=127; addr++)
        {
          //Serial.print("addr: "); Serial.println(addr);
          if(addr == TCAADDR) continue;

          uint8_t data;
          if(!twi_writeTo(addr, &data, 0, 1, 1))
          {
            Serial.print("Found I2C 0x"); Serial.println(addr, HEX);
          }
        }
      }
      /* Initialise the sensors */
      tcaselect(6);
      if(!accel1.begin())
      {
        /* There was a problem detecting the LSM303 ... check your connections */
        Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
        while(1);
      }
      if(!mag1.begin())
      {
        /* There was a problem detecting the LSM303 ... check your connections */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while(1);
      }

      tcaselect(7);
      if(!accel2.begin())
      {
        Serial.println(F("accel 2 not detected"));
        while(1);
      }

      if(!mag2.begin())
      {
        Serial.println(F("mag2 not detected"));
      }
      throttle = 1000;
      aileron = elevator = rudder = 1500;
      roll = throttle_pitch = yaw = elevator_pitch = 0.0;

  /* Display some basic information on this sensor */
}

void loop()
{
    uint32_t timeout;
    sensors_event_t event;


    // reset / rebind
    //Serial.println("begin loop");
    if(reset || ppm[AUX8] > PPM_MAX_COMMAND) {
        reset = false;
        Serial.println("selecting protocol");
        selectProtocol();        
        Serial.println("selected protocol.");
        NRF24L01_Reset();
        Serial.println("nrf24l01 reset.");
        NRF24L01_Initialize();
        Serial.println("nrf24l01 init.");
        init_protocol();
        Serial.println("init protocol complete.");
    }
    // process protocol
    //Serial.println("processing protocol.");
    switch(current_protocol) {
        case PROTO_CG023: 
        case PROTO_YD829:
            timeout = process_CG023();
            break;
        case PROTO_V2X2: 
            timeout = process_V2x2();
            break;
        case PROTO_CX10_GREEN:
        case PROTO_CX10_BLUE:
            timeout = process_CX10(); // returns micros()+6000 for time to next packet. 
            break;
        case PROTO_H7:
            timeout = process_H7();
            break;
        case PROTO_BAYANG:
            timeout = process_Bayang();
            break;
        case PROTO_SYMAX5C1:
            timeout = process_SymaX(); 
            break;
        case PROTO_H8_3D:
            timeout = process_H8_3D();
            break;
    }
    // updates ppm values out of ISR
    //update_ppm();
    overrun_cnt=0;
   

      sensors_event_t accel_event1;
      sensors_event_t mag_event1;
      sensors_vec_t   orientation1;

      sensors_event_t accel_event2;
      sensors_event_t mag_event2;
      sensors_vec_t   orientation2;

      tcaselect(6);
      /* Calculate pitch and roll from the raw accelerometer data */
      accel1.getEvent(&accel_event1);
      if (dof.accelGetOrientation(&accel_event1, &orientation1))
      {

        throttle_pitch = orientation1.pitch;
        Serial.print(F("Sensor 1: Pitch: "));
        Serial.print(orientation1.pitch);
        Serial.print(F("; "));
      }

      // throttle control
      if(throttle_pitch > 35.0)
      {
        throttle = 1410;
      }
      else if(throttle_pitch < -35)
      {
        if(throttle <= 1000){
          throttle = 1000;
        }
        else{
          throttle -= 2;
        }
      }
      else{
        if(throttle > 1430)
        {
          throttle = 1400;
        }
        else if(throttle < 1000){
          throttle = 1000;
        }
       }
       Serial.println("");
       Serial.print(F("Throttle: "));
       Serial.print(throttle);
       
      // end throttle control

      tcaselect(7);
      accel2.getEvent(&accel_event2);
      if (dof.accelGetOrientation(&accel_event2, &orientation2))
      {
        Serial.println();
        elevator_pitch = orientation2.pitch;
        Serial.print(F("Sensor 2: Pitch: "));
        Serial.print(orientation2.pitch);
        Serial.print(F("; "));

        roll = orientation2.roll;
        Serial.print(F("Sensor 2 Roll: "));
        Serial.print(orientation2.roll);
        Serial.print(F("; "));
      }
      
      if(roll < -25.0)
      {
         aileron = 1370;
      }
      else if(roll > 25)
      {
        aileron = 1630;
      }
      else{
        aileron = 1500;
       }
       Serial.println("");
       Serial.print(F("Aileron: "));
       Serial.print(aileron);

      // Elevator control
      if(elevator_pitch < -25.0)
      {
        // create a ceiling so that the value for throttle doesn't increase infinitely 
        elevator = 1630;
        
      }
      else if(elevator_pitch > 25)
      {
        elevator = 1350;
        
      }
      else{
        elevator = 1500;
       }
       Serial.println("");
       Serial.print(F("Elevator: "));
       Serial.print(elevator);

       //delay(1000);
      ppm[0] = throttle;
      ppm[1] = aileron;
      ppm[2] = elevator;
      ppm[3] = rudder;

      Serial.println(F(""));

    // wait before sending next packet
    while(micros() < timeout) // timeout for CX-10 blue = 6000microseconds. 
    {
      //overrun_cnt+=1;
    };
}

void set_txid(bool renew)
{
    uint8_t i;
    for(i=0; i<4; i++)
        transmitterID[i] = EEPROM.read(ee_TXID0+i);
    if(renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
        for(i=0; i<4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0+i, transmitterID[i]); 
        }            
    }
}

void selectProtocol()
{
    // Modified and commented out lines so that Cheerson CX-10 Blue is always selected
  
    // wait for multiple complete ppm frames
    ppm_ok = false;

    //if(ppm[RUDDER] < PPM_MIN_COMMAND)        // Rudder left
    set_txid(true);                      // Renew Transmitter ID
    
    current_protocol = PROTO_CX10_BLUE;  // Cheerson CX10(blue pcb, newer red pcb)/CX10-A/CX11/CX12 ... 
    
    // update eeprom 
    EEPROM.update(ee_PROTOCOL_ID, current_protocol);
}

void init_protocol()
{
    switch(current_protocol) {
        case PROTO_CG023:
        case PROTO_YD829:
            CG023_init();
            CG023_bind();
            break;
        case PROTO_V2X2:
            V2x2_init();
            V2x2_bind();
            break;
        case PROTO_CX10_GREEN:
        case PROTO_CX10_BLUE:
            CX10_init();
            CX10_bind();
            Serial.println("cx10-initialized and bound");
            break;
        case PROTO_H7:
            H7_init();
            H7_bind();
            break;
        case PROTO_BAYANG:
            Bayang_init();
            Bayang_bind();
            break;
        case PROTO_SYMAX5C1:
            Symax_init();
            SymaX_bind();
            break;
        case PROTO_H8_3D:
            H8_3D_init();
            H8_3D_bind();
            break;
    }
}

/* This function not needed - ppm values are updated in main loop
// update ppm values out of ISR    
void update_ppm()
{
    for(uint8_t ch=0; ch<CHANNELS; ch++) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            ppm[ch] = Servo_data[ch];
        }
    }    
}
*/


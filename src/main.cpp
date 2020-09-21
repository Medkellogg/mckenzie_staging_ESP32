//
//Jeroen Gerritsen's B&O McKenzie Division - Staging Yard Project
//by: Mark Kellogg - Began: 4/23/2020
//
//---------------------------Github Repository------------------------------
// NAME:
// The repository has all code etc. along with flow control overview and 
// module graphics.  
//
// Designed for use on Jeroen Gerritsen's B&O McKenzie Div layout.
// User controled remote panels are located on the fascia, connected 
// to an ESP32 and H-bridge chips to drive the Tortoise switch machines, 
// control track power, etc.  They are connected with a flat 8C modular 
// cable and RJ45 jacks.  A rotary encoder on the control panel is directly
// connected to the ESP32 and the OLED uses I2C, both via the modular cable.
//
// Uses +12VDC for driving the Tortoises; +5VC for the ESP32 VCC.  The ESP32
// uses a 3.3VDC logic level.  The sensors and the OLED are supplied with 3.3V.
// 
// Functions
//   *Rotary encoder with switch and OLED screen are the controls on the panel
//   *Sequence:
//      Single click or turn encoder to wake up screen. 
//      Select track choice with knob
//      Single click knob to align track: track power off and route aligns 
//      Alingnment completes: Track power on, 4 minute timer on
//      After 4 minutes or outbound train clears throat: track power off
//      Double-click to stop timer and return to standby
//      If more time is needed: the current track is displayed on the OLED, 
//        user may single-click to select again for another 4 minutes 
//      After the timer expires the system will return to the standby screen
//      Setup Mode can be entered by holding the click knob for seconds


//----------------------------Track Sensors Descriptions--------------------
// All four staging yards have a single yard lead, from which all 
// the dead end staging tracks fan out.  The yard lead of three of the four 
// yards continues on to a reverse loop.
//
//   Sensors - The yard lead entry turnout and reverse loop turnout each have a 
//   sensor pair to track entry and exits from those points.
//
//   Sensor Names - Sensors are named "mainSens" for the yard throat, and 
//   "revSens" for the reverse loop leadout.
//
//   Direction Naming Convention - In all cases you may think of a point "between"
//   the yard lead turnout and the reverse loop turnout as the center of the
//   universe.  Therefore INBOUND is always towards that point on either sensor
//   and OUTBOUND the reverse.   
//
//
// Module Output- Each sensor pair returns: 
//   Train Direction- mainDirection or revDirection: their output(s) are 
//   INBOUND, OUTBOUND, and are active when the train is within the 
//   small sensor area.
//   
//   Train PassBy - which senses the train has passed by the sensor completely in 
//   the direction it arrived from.  If a train backs out without completely
//   passing by PassBy will not report true. It stays active as long as the 
//   train is within the sensor pair.
//
//   Sensor Busy - Sensor reports busy when either sensor of either sensor 
//   pair is true and remains so until all return false.
//---------------------------------------------------------------------------

#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Bounce2.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include "bcsjTimer.h"
#include <OneButton.h>
#include <EEPROM.h>

#define Wheeling crntMap
//#define Parkersburg crntMap
//#define Philadelphia crntMap
//#define Cumberland crntMap



//------------Setup sensor debounce from Bounce2 library-----
const byte mainSensInpin {26};
const byte mainSensOutpin{27};
const byte revSensInpin  {12};
const byte revSensOutpin {14};

const byte INBOUND   {1};
const byte OUTBOUND  {2};
const byte CLEAR     {0};
const byte ON        {0};
const byte OFF       {1};

const byte trackPowerLED_PIN  {13};  //debug

const byte MAX_LADDER_TRACKS {13};
const byte MAX_TURNOUTS      {13};

//---Turnout bit masks are encoded for shift register input. T0 is
//   always lsb for shift register 
const uint16_t	THROWN_T0	  {0x0001};	//0000000000001	1
const uint16_t	THROWN_T1	  {0x0002};	//0000000000010	2
const uint16_t	THROWN_T2	  {0x0004};	//0000000000100	4
const uint16_t	THROWN_T3	  {0x0008};	//0000000001000	8
const uint16_t	THROWN_T4	  {0x0010};	//0000000010000	16
const uint16_t	THROWN_T5	  {0x0020};	//0000000100000	32
const uint16_t	THROWN_T6	  {0x0040};	//0000001000000	64
const uint16_t	THROWN_T7	  {0x0080};	//0000010000000	128
const uint16_t	THROWN_T8	  {0x0100};	//0000100000000	256
const uint16_t	THROWN_T9	  {0x0200};	//0001000000000	512
const uint16_t	THROWN_T10	{0x0400};	//0010000000000	1024
const uint16_t	THROWN_T11	{0x0800};	//0100000000000	2048
const uint16_t	THROWN_T12	{0x1000};	//1000000000000	4096

struct turnoutMap_t {
  uint8_t       numTracks;
  uint8_t       startTrack;
  byte          defaultTrack;
  bool          revL;
  unsigned long trainIOtime;
  char          mapName[16];
  uint16_t      routes[MAX_LADDER_TRACKS];
};

//----------------------Wheeling Staging Yard-------------------------
//  All turnouts are RH: the "normal" position selects active track.  
//  RevLoop power does not cycle off when RevLoop route selected.        
//         ______W1
//        /      ______W2
//       /      /      ______W3
//      /      /      /      ______W4
//     /      /      /      /      ______W5
//    /      /      /      /      /      
//___T0_____T1_____T2_____T3_____T4___________RevLoop


//const turnoutMap_t Wheeling = {
//             6,                 // numTracks
//             1,                 // startTrack
//             6,                 // defaultTrack
//             true,              // have reverse track?
//             1000000L * 60 * 1, // delay time in TRACK_ACTIVE
//             "Wheeling",
/* trk W0   */  //0,
/* trk W1   */  //THROWN_T1+THROWN_T2+THROWN_T3+THROWN_T4,
/* trk W2   */  //THROWN_T0+THROWN_T2+THROWN_T3+THROWN_T4,
/* trk W3   */  //THROWN_T0+THROWN_T1+THROWN_T3+THROWN_T4,
/* trk W4   */  //THROWN_T0+THROWN_T1+THROWN_T2+THROWN_T4,
/* trk W5   */  //THROWN_T0+THROWN_T1+THROWN_T2+THROWN_T3,
/* trk RevL */  //THROWN_T0+THROWN_T1+THROWN_T2+THROWN_T3+THROWN_T4,
/* trk A7   */  //0, /* not used */
/* trk W8   */  //0, /* not used */ 
/* trk W9   */  //0, /* not used */
/* trk W10  */  //0, /* not used */
/* trk W11  */  //0, /* not used */ 
/* trk W12  */  //0  /* not used */
//};

/**********************************************************************
*    THIS WHEELING PLAN IS A DEBUG SECTION--REMOVE--REMOVE--REMOVE--  *
*    AFTER TESTING REPLACE WITH SECTION ABOVE                         *
**********************************************************************/
const turnoutMap_t Wheeling = {
             12,                 // numTracks
             7,                  // startTrack
             12,                 // default track
             true,              // have reverse track?
             1000000L * 60 * 1,  // delay time in TRACK_ACTIVE
             "Wheeling",
/* trk W0   */  0, /* not used */
/* trk W1   */  0, /* not used */
/* trk W2   */  0, /* not used */
/* trk W3   */  0, /* not used */
/* trk W4   */  0, /* not used */
/* trk W5   */  0, /* not used */
/* trk W6   */  0, /* not used */
/* trk A7   */  THROWN_T9+THROWN_T10+THROWN_T11+THROWN_T12,
/* trk W8   */  THROWN_T8+THROWN_T10+THROWN_T11+THROWN_T12,
/* trk W9   */  THROWN_T8+THROWN_T9+THROWN_T11+THROWN_T12,
/* trk W10  */  THROWN_T8+THROWN_T9+THROWN_T10+THROWN_T12,
/* trk W11  */  THROWN_T8+THROWN_T9+THROWN_T10+THROWN_T11, 
/* trk W12  */  THROWN_T8+THROWN_T9+THROWN_T10+THROWN_T11+THROWN_T12
};

//--------------------Parkersburg Staging Yard------------------            
//         
//       P1  P2  P3  P4  P5  P6
//      /   /   /   /   /   /
//     /   /   /   /   /   /
//    /   /   /   /   /   /
//   /   /   /   /   /   /  
//  |   /   /   /  T3___/  
//  T4_/   /  T2__/    
//  |     T1__/
//  |    /     
//  T0__/  
//  | 
//  
const turnoutMap_t Parkersburg = {
             6,                 // track count
             5,                 // turnout count
             1,                 // default track
             false,             // have reverse track?
             1000000L * 60 * 1, // delay time in TRACK_ACTIVE
             "Parkersburg",
/* trk P0   */  0,
/* trk P1   */  0,
/* trk P2   */  THROWN_T4,
/* trk P3   */  THROWN_T0,
/* trk P4   */  THROWN_T0+THROWN_T1,
/* trk P5   */  THROWN_T0+THROWN_T1+THROWN_T2,
/* trk P6   */  THROWN_T0+THROWN_T1+THROWN_T2+THROWN_T3,
/* trk P7   */  0, /* not used */
/* trk P8   */  0, /* not used */ 
/* trk P9   */  0, /* not used */
/* trk P10  */  0, /* not used */
/* trk P11  */  0, /* not used */ 
/* trk P12  */  0  /* not used */
};

//----------------------Philadelphia Staging Yard-------------------------
//  All turnouts are RH: the "normal" position selects active track.  
//  RevLoop power does not cycle off when RevLoop route selected.        
//         ______W1
//        /      ______W2
//       /      /      ______W3
//      /      /      /      ______W4
//     /      /      /      /      ______W5
//    /      /      /      /      /      
//___T0_____T1_____T2_____T3_____T4___________RevLoop

const turnoutMap_t Philadelphia = {
             6,                 // track count
             5,                 // turnout count
             12,                // default track
             true,              // have reverse track?
             1000000L * 60 * 1, // delay time in TRACK_ACTIVE
             "Philadelphia",
/* trk W0   */  0,
/* trk W1   */  THROWN_T1+THROWN_T2+THROWN_T3+THROWN_T4,
/* trk W2   */  THROWN_T0+THROWN_T2+THROWN_T3+THROWN_T4,
/* trk W3   */  THROWN_T0+THROWN_T1+THROWN_T3+THROWN_T4,
/* trk W4   */  THROWN_T0+THROWN_T1+THROWN_T2+THROWN_T4,
/* trk W5   */  THROWN_T0+THROWN_T1+THROWN_T2+THROWN_T3,
/* trk RevL */  THROWN_T0+THROWN_T1+THROWN_T2+THROWN_T3+THROWN_T4,
/* trk A7   */  0, /* not used */
/* trk W8   */  0, /* not used */ 
/* trk W9   */  0, /* not used */
/* trk W10  */  0, /* not used */
/* trk W11  */  0, /* not used */ 
/* trk W12  */  0  /* not used */
};

//----------------------Cumberland Staging Yard-------------------------
//  All turnouts are RH: the "normal" position selects active track.  
//  RevLoop power does not cycle off when RevLoop route selected.        
//         ______W1
//        /      ______W2
//       /      /      ______W3
//      /      /      /      ______W4
//     /      /      /      /      ______W5
//    /      /      /      /      /      
//___T0_____T1_____T2_____T3_____T4___________RevLoop

const turnoutMap_t Cumberland = {
             6,                 // track count
             5,                 // turnout count
             12,                // default track
             true,              // have reverse track?
             1000000L * 60 * 1, // delay time in TRACK_ACTIVE
             "Cumberland",
/* trk W0   */  0,
/* trk W1   */  THROWN_T1+THROWN_T2+THROWN_T3+THROWN_T4,
/* trk W2   */  THROWN_T0+THROWN_T2+THROWN_T3+THROWN_T4,
/* trk W3   */  THROWN_T0+THROWN_T1+THROWN_T3+THROWN_T4,
/* trk W4   */  THROWN_T0+THROWN_T1+THROWN_T2+THROWN_T4,
/* trk W5   */  THROWN_T0+THROWN_T1+THROWN_T2+THROWN_T3,
/* trk RevL */  THROWN_T0+THROWN_T1+THROWN_T2+THROWN_T3+THROWN_T4,
/* trk A7   */  0, /* not used */
/* trk W8   */  0, /* not used */ 
/* trk W9   */  0, /* not used */
/* trk W10  */  0, /* not used */
/* trk W11  */  0, /* not used */ 
/* trk W12  */  0  /* not used */
};

//-----Setup pins for 74HC595 shift register 
const int latchPin = 33;   
const int clockPin = 32;   
const int dataPin  = 25; 

//-----declare latch function----
void writeTrackBits( uint16_t track);

//---Instantiate a bcsjTimer.h object for screen sleep
bcsjTimer  timerOLED;
bcsjTimer  timerTortoise;
bcsjTimer  timerTrainIO;

//---Timer Variables---
unsigned long interval_OLED    = 1000000L * 60 * 0.5;
unsigned long tortoiseInterval = 1000000L * 3;
unsigned long intervalTrainIO  = crntMap.trainIOtime;

// Instantiate a Bounce object
Bounce debouncer1 = Bounce(); Bounce debouncer2 = Bounce(); 
Bounce debouncer3 = Bounce(); Bounce debouncer4 = Bounce();

//------------Set up OLED Screen-----
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 

//-------Declaration for an SSD1306 display - using I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//---------------------OLED Display Functions------------------//
byte oledState = true;
void oledOn();
void oledOff();
void bandoText(String text, int x, int y, int size, boolean d);
void tracknumChoiceText();
void tracknumActiveText();  //TODO______may not need----review----
void tracknumActChoText();

//---RotaryEncoder DEFINEs for numbers of tracks to access with encoder
#define ROTARYSTEPS 1
#define ROTARYMIN   crntMap.startTrack
#define ROTARYMAX   crntMap.numTracks

//--- Setup a RotaryEncoder for GPIO pins 16, 17:
RotaryEncoder encoder(16, 17);
byte          lastPos = -1;              //-- Last known rotary position.
OneButton     encoderSw1(4, true);      //---Setup  OneButton for rotary 
                                         //   encoder sw on pin 13 - active low

//---RotaryEncoder Setup and variables are in this section---------
byte tracknumChoice  = ROTARYMAX;
byte tracknumActive  = ROTARYMAX;
byte tracknumDisplay = ROTARYMAX;

//---Rotary Encoder Switch Variables-------------------------------
byte knobPosition = ROTARYMAX;
bool knobToggle   = true;       //active low 
void readEncoder();             //--RotaryEncoder Function------------------

//--OneButton Function delarations for RotaryEncoder switch
void click1();
void doubleclick1();
void longPressStart1();

bool bailOut = true;  //active low, set active by doubleclick to end timer 

//---------------SETUP STATE Machine and State Functions----------------------
enum {HOUSEKEEP, STAND_BY, TRACK_SETUP, TRACK_ACTIVE, OCCUPIED, MENU} mode;
void runHOUSEKEEP();
void runSTAND_BY();
void runTRACK_SETUP();
void runTRACK_ACTIVE();
void runOCCUPIED();
void runMENU();
void leaveTrack_Setup();
void leaveTrack_Active();

//---Sensor Function Declarations---------------
void readMainSens();
void readRevSens();
void rptMainDirection();
void rptRevDirection();
void readAllSens();

//---State Machine Variables
byte railPower = OFF;

//---Sensor variables
byte mainSensTotal      = 0,      mainSens_Report    = 0; 
byte mainPassByState    = false,  mainPassByTotal    = 0;
byte mainInValue        = 1,      mainIn_LastValue   = 1; 
byte mainOutValue       = 1,      mainOut_LastValue  = 1;
byte main_LastDirection = 0,      mainDirection      = 0;

byte revSensTotal       = 0,      revSens_Report     = 0; 
byte revPassByState     = false,  revPassByTotal     = 0;
byte revInValue         = 1,      revIn_LastValue    = 1; 
byte revOutValue        = 1,      revOut_LastValue   = 1;
byte revDirection       = 0,      rev_LastDirection  = 0;

//--------------------------------------------------------------//
//                         void setup()                         //
//--------------------------------------------------------------//

void setup() 
{
  Serial.begin(115200);
  delay(1000);  //time to bring up serial monitor
  Wire.setClock(1000000L);
    
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
      Serial.println(F("SSD1306 allocation failed"));
      for (;;); // Don't proceed, loop forever
    }

  display.setFont(&FreeSansBold9pt7b);
    
  //---Setup the sensor pins
  pinMode(mainSensInpin, INPUT_PULLUP); pinMode(mainSensOutpin, INPUT_PULLUP);
  pinMode(revSensInpin, INPUT_PULLUP);  pinMode(revSensOutpin, INPUT_PULLUP);

  //---setup the Bounce pins and intervals :
  debouncer1.attach(mainSensInpin); debouncer2.attach(mainSensOutpin);
  debouncer3.attach(revSensInpin);  debouncer4.attach(revSensOutpin);
  debouncer1.interval(5);           debouncer2.interval(5); // interval in ms
  debouncer3.interval(5);           debouncer4.interval(5); 

  pinMode(trackPowerLED_PIN, OUTPUT); // for relay in final version

  encoder.setPosition(ROTARYMAX / ROTARYSTEPS); // start with ROTARYMAX value
  encoderSw1.attachClick(click1);
  encoderSw1.attachDoubleClick(doubleclick1);
  encoderSw1.attachLongPressStart(longPressStart1);

  //---Shift register pins
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin,  OUTPUT);  
  pinMode(clockPin, OUTPUT); 
  
  digitalWrite(trackPowerLED_PIN, HIGH);
  
  display.clearDisplay();
  bandoText("B&O RAIL",25,12,1,false);
  //bandoText("JEROEN GERRITSEN'S",8,20,1,true);
  bandoText("Staging Yard:",0,36,1,true);
  bandoText(crntMap.mapName,0,60,1,true);
  display.display();

  tracknumChoice = crntMap.routes[crntMap.defaultTrack];
  writeTrackBits(crntMap.routes[crntMap.defaultTrack]);  //align to default track
  
  
  delay(5000);
  display.clearDisplay();
  digitalWrite(trackPowerLED_PIN, LOW);
} //-----------------------End setup-----------------------------

//--------------------------------------------------------------//
//                          void loop()                         //
//--------------------------------------------------------------//

void loop() 
{
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else                 digitalWrite(trackPowerLED_PIN, LOW);

  if (mode == HOUSEKEEP)         {runHOUSEKEEP();}
  else if (mode ==     STAND_BY) {runSTAND_BY();}
  else if (mode ==  TRACK_SETUP) {runTRACK_SETUP();}
  else if (mode == TRACK_ACTIVE) {runTRACK_ACTIVE();}
  else if (mode ==     OCCUPIED) {runOCCUPIED();}
  else if (mode ==         MENU) {runMENU();}
  
  /*----debug terminal print----------------
                          Serial.print("mainSensTotal:      ");
                          Serial.print(mainSensTotal);
                          Serial.print("           revSensTotal:  ");
                          Serial.println(revSensTotal);
                          Serial.print("mainPassByState:    ");
                          Serial.print(mainPassByState);
                          Serial.print("          revPassByState: ");
                          Serial.println(revPassByState); 
                          Serial.print("main_LastDirection: ");
                          Serial.print(main_LastDirection);
                          Serial.print("       rev_lastDirection: ");
                          Serial.println(rev_LastDirection);
                          Serial.print("tracknumActive:    ");
                          Serial.print(tracknumActive);
 //---end debug printing-------------*/
}     
 //------------------------END void loop-------------------
 

/* -------------------State Machine Functions---------------------*
 *                          BEGIN HERE                            *
 *----------------------------------------------------------------*/

//--------------------HOUSEKEEP Function--------------------------
void runHOUSEKEEP()
{
  display.ssd1306_command(0xAF); // turn OLED on
  oledOn();
                                  Serial.println();
                                  Serial.println("--------------------HOUSEKEEP---");
  
  if((tracknumActive < ROTARYMAX) || (crntMap.revL == false)) railPower = OFF;
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else  digitalWrite(trackPowerLED_PIN, LOW);
    
  display.clearDisplay();
  tracknumActChoText();
  bandoText("   -SELECT-",0,12,1,false);
  bandoText("TRACK:",0,36,1,false);
  bandoText("Active:",0,60,1,true);
  
  timerOLED.start(interval_OLED);   /*--start sleep timer here for when HOUSEKEEP 
                                      state is entered after moving through states
                                      and no knob twist.                         */
  mode = STAND_BY;
}  

//-----------------------STAND_BY Function-----------------
void runSTAND_BY()
{
                                 Serial.println("-----------------------STAND_BY---");
    //---Begin main do-while loop checking for user input--------
  do
  {    
    if(timerOLED.done() == true){     //---check screen timer and put in
      oledOff();                      //   sleep mode if time out
    }
    readEncoder();
    readAllSens();
    encoderSw1.tick();  //check for clicks
    if((mainSens_Report > 0) || (revSens_Report > 0))
    {
                                Serial.println("---to OCCUPIED from STAND_BY---"); 
      oledOn();                    
      runOCCUPIED();
    }
  }
  while (knobToggle == true);        //---check rotary switch pressed to select a 
                                     //   track (active low)
  
  tracknumActive = tracknumChoice;  
  knobToggle = true;                 //--reset so readEncoder will run in stand_by
  timerOLED.disable();              
  oledOn();
  display.display(); 
  mode = TRACK_SETUP;                //---move on with new track assignment
} 

//-----------------------TRACK_SETUP- State Function-----------------------
void runTRACK_SETUP()
{
  readAllSens();
  railPower = OFF;
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else  digitalWrite(trackPowerLED_PIN, LOW);
                            Serial.println("------------------------TRACK_SETUP---");
  writeTrackBits(crntMap.routes[tracknumActive]);

  display.clearDisplay();
  bandoText(" -ALIGNING-",0,12,1,false);
  bandoText("TRACK:",0,36,1,false);
  tracknumChoiceText();  
  bandoText("WAIT!",20,60,1,true);
  
  timerTortoise.start(tortoiseInterval);   //--begin delay for Tortoises
  while(timerTortoise.running() == true)
  {
   readAllSens();
  }
  railPower = ON;
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else  digitalWrite(trackPowerLED_PIN, LOW);
  bailOut = true;                          //--reset (active low)
  leaveTrack_Setup();
  
}  //---end track setup function-------------------

void leaveTrack_Setup()
{
                            Serial.println("---Entering leaveTrack_Setup---");
  readAllSens();
  if((mainSens_Report > 0) || (revSens_Report > 0))
  {
                            Serial.println("---to OCCUPIED from leaveTrack_Setup---");
    mode = OCCUPIED;
  }
  else 
  {
                            Serial.println("--times up--leaving TrackSetup--");
    mode = TRACK_ACTIVE;
  }
}

//-----------------------TRACK_ACTIVE State Function------------------
void runTRACK_ACTIVE()
{
  readAllSens();
  display.clearDisplay();
  bandoText(" -PROCEED-",0,12,1,false);
  bandoText("TRACK:",0,36,1,false);
  tracknumChoiceText();    
  bandoText("Timer is ON",0,60,1,true);
                            Serial.println("-----------------------TRACK_ACTIVE---");
  rev_LastDirection = 0; //reset for use during the next TRACK_ACTIVE call
  main_LastDirection = 0;
  timerTrainIO.start(intervalTrainIO);
  do
  {
     readAllSens();
     encoderSw1.tick();
     
                            //debug
                            /*Serial.print("main_LastDirection: ");
                            Serial.print(rev_LastDirection);
                            Serial.println(main_LastDirection);
                            Serial.println("-----Waiting for Train to Exit!");  */
     
    if (bailOut == 0)       //active low: active if doubleclick encoder knob
    {
      break;
    }
        //--true when outbound train completely leaves sensor  
    if (((mainPassByState == 1) && (main_LastDirection == 2)) ||
         ((rev_LastDirection == 2) && (revPassByState == 1)))           
    {
      break;
    }
  }
  while(timerTrainIO.running() == true);

  mainPassByState = false;
  revPassByState = false;
  leaveTrack_Active();
}  //--end runTrack_Active---

//---------------------leaveTrack_Active Function--------------------

void leaveTrack_Active()
{
  readAllSens();
  if((mainSens_Report > 0) || (revSens_Report > 0))
  {
                            Serial.println("--to OCCUPIED from leavTrack_Active-");
    mode = OCCUPIED;
  }
  else 
  {
                            Serial.println("--times up leaving TrackActive--");
    mode = HOUSEKEEP;
  }
}

//-------------------------OCCUPIED State Function--------------------
void runOCCUPIED()
{
  Serial.println("OCCUPIED");
  display.clearDisplay();
  while((mainSens_Report > 0) || (revSens_Report > 0))
  {
    readAllSens();
    Serial.println("----to OCCUPIED from OCCUPIED---");
    bandoText("YARD LEAD",0,12,1,false);
    bandoText("OCCUPIED",0,36,1,false);
    bandoText("STOP!",20,60,1,true);
  }
                          Serial.println("----Leaving OCCUPIED---");
  runHOUSEKEEP();
}

//------------------------ReadEncoder Function----------------------

void readEncoder()
{ 
  encoder.tick();
  // get the choice physical position and calc the logical position
  int newPos = encoder.getPosition() * ROTARYSTEPS;
  if (newPos < ROTARYMIN) {
    encoder.setPosition(ROTARYMIN / ROTARYSTEPS);
    newPos = ROTARYMIN;
  } 
  else if (newPos > ROTARYMAX) {
    encoder.setPosition(ROTARYMAX / ROTARYSTEPS);
    newPos = ROTARYMAX;
  } 
  if (lastPos != newPos) {
    lastPos = newPos;
    tracknumChoice = newPos;
    //oledOn();
                        /*Serial.print(newPos);   
                        Serial.println();
                        Serial.print("Choice: ");
                        Serial.println(tracknumChoice);
                        Serial.print("Active: ");
                        Serial.println(tracknumActive);
                        Serial.print("bailOut:   ");
                        Serial.println(bailOut);  */
                        //delay(50); 

    timerOLED.start(interval_OLED);   //--sleep timer for STAND_BY mode
    display.clearDisplay();
    tracknumActChoText();
    bandoText("Active:",0,60,1,false);
    bandoText("-SELECT-",0,12,1,false);
    bandoText("TRACK:",0,36,1,true);
    
  }
} 

void runMENU()
{  
                        Serial.println("------------------------------runMENU---");

  oledOn();
  display.clearDisplay();
  bandoText("SETUP MENU",0,12,1,false);
  bandoText("Wrt EEPROM",0,36,1,true);
  delay(3000);
  
  runHOUSEKEEP();
}

//---------------------Updating Sensor Functions------------------
//  All functions in this section update and track sensor information: 
//  Busy, Direction, PassBy.  Only the mainOut sensor is documented.  
//  The remaining three work identically.
//------------------------------end of note-----------------------

void readAllSens() 
  {
    readMainSens();
    readRevSens();
  }   

void readMainSens() {
  debouncer1.update();  
  int mainInValue = debouncer1.read();
   if(mainInValue != mainIn_LastValue)
   {
      if(mainInValue == 0) bitSet(mainSens_Report, 0);
      else bitClear(mainSens_Report, 0); 
      mainIn_LastValue = mainInValue;
      if (mainSens_Report > 0) 
      {
        mainSensTotal = mainSensTotal + mainSens_Report;
        mainPassByTotal = mainSensTotal;
      }
      else mainSensTotal = 0;   
    }
  debouncer2.update();                 //--read mainOut sensor
  int mainOutValue = debouncer2.read();
                                       //--update history register: *Sens_Report    
  if(mainOutValue != mainOut_LastValue)   
    {
      if(mainOutValue == 0) bitSet(mainSens_Report, 1);
      else bitClear(mainSens_Report, 1); 

      mainOut_LastValue = mainOutValue;
                                        /*--add running total to "*"SensTotal to 
                                        track PassBy status                    */
      if (mainSens_Report > 0) 
      {
        mainSensTotal = mainSensTotal + mainSens_Report;
        mainPassByTotal = mainSensTotal;
      }
      else mainSensTotal = 0; 
    }
    //---PassByTotal greater than "6" means train has cleared the sensor 
    //  successfully.  TODO--TODO--TODO  FIX BACKING OUT PROBLEM WHEN STARTING
    //  ENTERING OUTBOUND AND BACKING OUT INBOUND - TURNS OFF TIMER
    //---------end of note------  
    if(mainSensTotal == 0 && mainPassByTotal >= 6) 
      {
       mainPassByState = true;
       mainPassByTotal = 0;
      }
    else if(mainSensTotal == 0) mainPassByTotal = 0;
    if((mainSensTotal == 2) && (mainSens_Report == 2)) //--report mainLine Direction
    { 
      mainDirection = 2;
      main_LastDirection = 2;
    }
    else if((mainSensTotal == 1) && (mainSens_Report == 1)) 
    {
      mainDirection = 1;
      main_LastDirection = 1;
    }
    if((mainSensTotal == 0) && (mainSens_Report == 0)) 
    {
     mainDirection = 0;
    } 
}  // end readMainSen--

void readRevSens() 
{ 
  debouncer3.update();
  int revInValue = debouncer3.read();
  if(revInValue != revIn_LastValue)     
  {
    if(revInValue == 0) bitSet(revSens_Report, 0);
    else bitClear(revSens_Report, 0); 
    revIn_LastValue = revInValue;
     if (revSens_Report > 0) 
    {
      revSensTotal = revSensTotal + revSens_Report;
      revPassByTotal = revSensTotal;
    }
    else revSensTotal = 0;   
  }
      
  debouncer4.update();
  int revOutValue = debouncer4.read();
   if(revOutValue != revOut_LastValue)     
  {
    if(revOutValue == 0) bitSet(revSens_Report, 1);
    else bitClear(revSens_Report, 1); 
    revOut_LastValue = revOutValue;
    if (revSens_Report > 0) 
    {
      revSensTotal = revSensTotal + revSens_Report;
      revPassByTotal = revSensTotal;
    }
    else revSensTotal = 0; 
  }
    if(revSensTotal == 0 && revPassByTotal >= 6) 
    {
      revPassByState = true;
      revPassByTotal = 0;
    }
    else if(revSensTotal == 0) revPassByTotal = 0;
    if((revSensTotal == 2) && (revSens_Report == 2)) //--report revLoop Direction
    {
      revDirection = 2;
      rev_LastDirection = 2;
    }
    else if((revSensTotal == 1) && (revSens_Report == 1)) 
    {
      revDirection = 1;
      rev_LastDirection = 1;
    }
    if((revSensTotal == 0) && (revSens_Report == 0)) 
    {
      revDirection = 0;
    }
}  // end readrevSen--

// -----------------------Display Functions---------------------//
//                          BEGIN HERE                          //
//--------------------------------------------------------------//

void bandoText(String text, int x, int y, int size, boolean d){
  display.setTextSize(size);
  display.setTextColor(WHITE);
  display.setCursor(x,y);
  display.println(text);
  if(d){
  display.display();
  }
}  

void tracknumChoiceText()
{
  enum {BufSize=3};  
  char choiceBuf[BufSize];
  snprintf (choiceBuf, BufSize, "%2d", tracknumChoice);
    if((tracknumChoice == ROTARYMAX) && (crntMap.revL  == true) ) bandoText("RevL",78,36,1,false);
    else bandoText(choiceBuf,82,36,1,false);
}

void tracknumActiveText()    //TODO________may not need--review-----
{
  enum {BufSize=3};  
  char activeBuf[BufSize];
  snprintf (activeBuf, BufSize, "%2d", tracknumActive);
    if(tracknumActive == ROTARYMAX) bandoText("RevL",78,60,1,false);
    else  bandoText(activeBuf,75,60,1,false);
}

void tracknumActChoText()
{
  enum {BufSize=3};  
  char activeBuf[BufSize];
  char choiceBuf[BufSize];
  snprintf (choiceBuf, BufSize, "%2d", tracknumChoice);
    if((tracknumChoice == ROTARYMAX) && (crntMap.revL  == true) ) bandoText("RevL",78,36,1,false);
    else  bandoText(choiceBuf,88,40,2,false);
  snprintf (activeBuf, BufSize, "%2d", tracknumActive);
    if((tracknumActive == ROTARYMAX) && (crntMap.revL  == true)) bandoText("RevL",78,60,1,false);
    else  bandoText(activeBuf,75,60,1,false); 
}  

void oledOn()
 {
  display.ssd1306_command(0xAF);
  oledState = true;
 }
 
void oledOff()
 {
  display.ssd1306_command(0xAE);
  oledState = false;
  }

//----------------Rotary EncoderSw Click Functions--------------//
//                          BEGIN HERE                          //
//--------------------------------------------------------------//

void click1(){                //--singleclick: if sleep awaken OLED
  if(oledState == false){       
    timerOLED.start(interval_OLED);
    oledOn();
    display.display();
  }
                  
  else knobToggle = false;    //  else set trackChoice and move to setup             
}

void doubleclick1(){          //--doubleclick: reset trainIO timer to 0
    timerTrainIO.disable();
}

void longPressStart1(){       //--hold for 3 seconds goto Main Setup Menu
  runMENU();
}

//----------------Shift Register Function--------------//

void writeTrackBits(uint16_t track)
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, (track >> 8));
  shiftOut(dataPin, clockPin, MSBFIRST, track);
  digitalWrite(latchPin, HIGH);
            //Serial.print("track: ");
            //Serial.println(track);
            //Serial.println(track, BIN);
  
}  


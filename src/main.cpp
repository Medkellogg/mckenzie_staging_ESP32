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
// control track power, etc.  They are connected with a flat 8C8P modular 
// cable and RJ45 jacks.  A rotary encoder on the control panel is directly
// connected to the ESP32 and the OLED uses I2C, both via the modular cable.
//
// Uses +12VDC for driving the Tortoises; +5VC for the ESP32 VCC.  The ESP32
// uses a 3.3VDC logic level.  The sensors and the OLED are supplied with 3.3V.
// 
// Functions
//   *Rotary encoder with switch and OLED screen are the controls on the panel
//   *Sequence:
//      Single click to wake up screen. 
//      Select track choice with knob
//      Single click knob to align track: track power off and route aligns 
//      Alingnment completes: Track power on, "n" minute timer on
//      After "n" minutes or outbound train clears throat: track power off
//      Double-click to stop timer and return to standby
//      If more time is needed: the current track is displayed on the OLED, 
//      user may single-click to select again for another "n" minutes 
//      After the timer expires the system will return to the standby screen
//      Setup Mode can be entered by holding the click knob for 6 seconds

//    *Setup Mode:  Plug in the 3 button select board to the I/O  18, 19, 23, and
//     the administrator can select the staging yard and delay time to be used
//     by the board.  Once selections have been made the system needs to be
//     rebooted to enable the new choices to be written, during setup, from the 
//     EEPROM.

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
//   universe.  Therefore, INBOUND is always towards that point on either sensor
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
//   pair is true, and remains so until all return false.
//      ADDED A BIG LAST LINE TO THE DESCRIPTION FROM THE LAPTOP
//---------------------------------------------------------------------------

#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Bounce2.h>
#include <SPI.h>
#include <Wire.h>
#include "bcsjTimer.h"
#include <OneButton.h>
#include <EEPROM.h>
#include <U8g2lib.h>

#define swVer "v2.5 - (1/11/2025)"

//---Constructor for OLED screen
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//This is a change in the code on the laptop

/**********Staging yard "Map to number" Converion Table***************
*            &Wheeling,    #0                                       *
*            &Parkersburg, #1                                        *
*            &Bayview,     #2                                        *
*            &Cumberland,  #3                                        *
*            &Test,        #4                                        *
*            &Charleston,  #5                                        *
*            &Curtis_Bay,  #6                                        *
*            &WestStging,  #7                                        *
*********************************************************************/

//---------Variables written to EEPROM by "MENU" Function ------------
byte crntMapChoice = 3;           
byte trackActiveDelayChoice = 1;

//--------Variables set in void.setup() upon reading EEPROM----------
byte crntMap = 4;
byte trackActiveDelay = 1;

//-----------------------setup read/write to ESP32 flash (EEPROM)----
#define EEPROM_SIZE 8

//------------Setup sensor debounce from Bounce2 library-----
const byte mainSensInpin {26};
const byte mainSensOutpin{27};
const byte revSensInpin  {14};
const byte revSensOutpin {12};


const byte INBOUND   {1};
const byte OUTBOUND  {2};
const byte CLEAR     {0};
const byte ON        {0};
const byte OFF       {1};

const byte trackPowerLED_PIN  {2};  

const byte MAX_LADDER_TRACKS {17};
const byte MAX_TURNOUTS      {17};

//---Turnout bit masks are encoded for shift register input. T0 is
//   always lsb for shift register 
const uint16_t	THROWN_S1	  {0x0001};	//0000000000000001	1
const uint16_t	THROWN_S2	  {0x0002};	//0000000000000010	2
const uint16_t	THROWN_S3	  {0x0004};	//0000000000000100	4
const uint16_t	THROWN_S4	  {0x0008};	//0000000000001000	8
const uint16_t	THROWN_S5	  {0x0010};	//0000000000010000	16
const uint16_t	THROWN_S6	  {0x0020};	//0000000000100000	32
const uint16_t	THROWN_S7	  {0x0040};	//0000000001000000	64
const uint16_t	THROWN_S8   {0x0080};	//0000000010000000	128
const uint16_t	THROWN_S9	  {0x0100};	//0000000100000000	256
const uint16_t	THROWN_S10  {0x0200};	//0000001000000000	512
const uint16_t	THROWN_S11	{0x0400};	//0000010000000000	1024
const uint16_t	THROWN_S12	{0x0800};	//0000100000000000	2048
const uint16_t	THROWN_S13	{0x1000};	//0001000000000000	4096
const uint16_t	THROWN_S14	{0x2000};	//0010000000000000	8192
const uint16_t	THROWN_S15	{0x4000};	//0100000000000000	16384
const uint16_t	THROWN_S16	{0x8000};	//1000000000000000	32768

struct turnoutMap {
  uint8_t       numTracks;
  uint8_t       startTrack;
  uint8_t       defaultTrack;
  bool          revL;
  char          mapName[16];
  uint16_t      routes[MAX_LADDER_TRACKS];
};

//----------------------Wheeling Staging Yard-------------------------

turnoutMap Wheeling = {         // map #0
             6,                 // numTracks
             1,                 // startTrack
             6,                 // defaultTrack
             true,              // have reverse track?
             "Wheeling",        // mapName
/* trk W0   */  0,
/* trk W1   */  0,
/* trk W2   */  THROWN_S1,
/* trk W3   */  THROWN_S1+THROWN_S2,
/* trk W4   */  THROWN_S1+THROWN_S2+THROWN_S3,
/* trk W5   */  THROWN_S1+THROWN_S2+THROWN_S3+THROWN_S4+THROWN_S5,
/* trk RevL */  THROWN_S1+THROWN_S2+THROWN_S3+THROWN_S4, 
};


//--------------------Parkersburg Staging Yard------------------            


turnoutMap Parkersburg = {      // Map #1
             6,                 // numTrack
             1,                 // startTrack
             1,                 // default track
             false,             // have reverse track?
             "Parkersburg",     // map name
/* trk P0   */  0,
/* trk P1   */  0,
/* trk P2   */  THROWN_S5,
/* trk P3   */  THROWN_S1,
/* trk P4   */  THROWN_S1+THROWN_S2,
/* trk P5   */  THROWN_S1+THROWN_S2+THROWN_S3,
/* trk P6   */  THROWN_S1+THROWN_S2+THROWN_S3+THROWN_S4,
};

//--------------------Bayview Staging Yard------------------

turnoutMap Bayview = {          // Map #2
             12,                // numTracks
             1,                 // startTrack
             12,                // default track
             true,              // have reverse track?
             "Bayview",         // map name
/* trk 0    */  0,  /* not used  */
/* trk B1   */  THROWN_S1+THROWN_S2+THROWN_S3,
/* trk B2   */  THROWN_S1+THROWN_S2,
/* trk B3   */  THROWN_S1+THROWN_S5,
/* trk B4   */  THROWN_S1,
/* trk B5   */  THROWN_S6+THROWN_S7+THROWN_S8,
/* trk B6   */  THROWN_S6+THROWN_S7,
/* trk B7   */  THROWN_S6+THROWN_S9,
/* trk B8   */  THROWN_S6,
/* trk B9   */  THROWN_S10+THROWN_S11, 
/* trk B10  */  THROWN_S10,
/* trk B11  */  0,  /*  all switches in Normal position  */
/* trk RevL */  THROWN_S1+THROWN_S2+THROWN_S3+THROWN_S4,
};

//----------------------Cumberland Staging Yard-------------------------

turnoutMap Cumberland = {       // Map #3
             12,                // numTrack - total in yard
             1,                 // startTrack - start counting at this num
             12,                // default track - startup track
             true,              // have reverse track?
             "Cumberland",      // map name
/* trk W0   */  0,
/* trk W1   */  THROWN_S1+THROWN_S10,
/* trk W2   */  THROWN_S1+THROWN_S11,
/* trk W3   */  THROWN_S1,
/* trk W4   */  THROWN_S2+THROWN_S8,
/* trk W5   */  THROWN_S2+THROWN_S9,
/* trk C6   */  THROWN_S2,
/* trk A7   */  THROWN_S3+THROWN_S6,
/* trk W8   */  THROWN_S3+THROWN_S7, 
/* trk W9   */  THROWN_S3,
/* trk W10  */  THROWN_S4+THROWN_S5,
/* trk W11  */  THROWN_S4, 
/* trk RevL */  0,
};


/**********************************************************************
*    Test Yard IS USED TO TEST SINGLE TORTOISES.  
*    A SINGLE TORTOISE IS SELECTED FOR EACH ROUTE.                         
**********************************************************************/
turnoutMap Test = {            // Map #4
             16,               // numTracks
             0,                // startTrack
             0,                // default track
             false,            // have reverse track?
             "Test",           // map name
/* trk W0       */  0, 
/* SWITCH 1     */  THROWN_S1, 
/* SWITCH 2     */  THROWN_S2, 
/* SWITCH 3     */  THROWN_S3, 
/* SWITCH 4     */  THROWN_S4,  
/* SWITCH 5     */  THROWN_S5, 
/* SWITCH 6     */  THROWN_S6, 
/* SWITCH 7     */  THROWN_S7,
/* SWITCH 8     */  THROWN_S8,
/* SWITCH 9     */  THROWN_S9,
/* SWITCH 10    */  THROWN_S10,
/* SWITCH 11    */  THROWN_S11, 
/* SWITCH 12    */  THROWN_S12,
/* SWITCH 13    */  THROWN_S13,
/* SWITCH 14    */  THROWN_S14,
/* SWITCH 15    */  THROWN_S15,
/* SWITCH 16    */  THROWN_S16,
};

turnoutMap Charleston = {       // map #5
             6,                 // numTracks
             1,                 // startTrack
             1,                 // defaultTrack
             true,              // have reverse track?
             "Charleston",      // mapName
/* trk W0   */  0,
/* trk W1   */  0,
/* trk W2   */  THROWN_S1,
/* trk W3   */  THROWN_S1+THROWN_S2,
/* trk W4   */  THROWN_S1+THROWN_S2+THROWN_S3,
/* trk W5   */  THROWN_S1+THROWN_S2+THROWN_S3+THROWN_S4,
/* trk RevL */  THROWN_S1+THROWN_S2+THROWN_S3+THROWN_S4+THROWN_S5, 
};

turnoutMap Curtis_Bay = {       // map #6
             5,                 // numTracks
             1,                 // startTrack
             1,                 // defaultTrack
             false,             // have reverse track?
             "Curtis Bay",      // mapName
/* trk W0   */  0,
/* trk W1   */  THROWN_S1,
/* trk W2   */  0,
/* trk W3   */  THROWN_S4,
/* trk W4   */  THROWN_S3,
/* trk W5   */  THROWN_S2,
};

turnoutMap WestStaging = {       // map #7
             12,                 // numTracks
             1,                  // startTrack
             12,                 // defaultTrack
             true,               // have reverse track?
             "West Staging",     // mapName
/* trk P0    */  0,
/* trk WS1   */  0,
/* trk WS2   */  THROWN_S3,
/* trk WS3   */  THROWN_S2,
/* trk WS4   */  THROWN_S2+THROWN_S4,
/* trk WS5   */  THROWN_S2+THROWN_S4+THROWN_S5,
/* trk WS6   */  THROWN_S2+THROWN_S4+THROWN_S5+THROWN_S6,
/* trk WS7   */  THROWN_S1,
/* trk WS8   */  THROWN_S1+THROWN_S7,
/* trk WS9   */  THROWN_S1+THROWN_S7+THROWN_S8,
/* trk WS10  */  THROWN_S1+THROWN_S7+THROWN_S8+THROWN_S9,
/* trk WS11  */  THROWN_S1+THROWN_S7+THROWN_S8+THROWN_S9+THROWN_S10+THROWN_S11,
/* trk RevL  */  THROWN_S1+THROWN_S7+THROWN_S8+THROWN_S9+THROWN_S10,
};

//--------------Map yard memory addresses with pointer for crntMap----
const turnoutMap *mapData[8] = {
  &Wheeling,      // #0
  &Parkersburg,   // #1
  &Bayview,       // #2
  &Cumberland,    // #3
  &Test,          // #4
  &Charleston,    // #5
  &Curtis_Bay,    // #6
  &WestStaging,   // #7
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
bcsjTimer  timerTrackSelect;

//---Timer Variables---
unsigned long additionalScreenTime  = 1000000L * 60 * 1;    //+ screen timeout for sleep
unsigned long interval_Tortoise     = 1000000L * 3;         //Tortoise run time interval
unsigned long interval_OLED         = trackActiveDelay + additionalScreenTime;
unsigned long interval_TrackSelect  = 1000000L * 5;         //---Display "new track selection for 5 //
                                                            //seconds before return to Active Track //


// Instantiate a Bounce object
Bounce debouncer1 = Bounce(); Bounce debouncer2 = Bounce(); 
Bounce debouncer3 = Bounce(); Bounce debouncer4 = Bounce();

//---------------------OLED Display Functions------------------//
byte oledState = true;
void oledOn();   
void oledOff();    
void tracknumChoiceText();
void tracknumActiveText();
void tracknumActiveTextSm();
void tracknumActChoText();  //DISPLAY______may not need----review----

//---RotaryEncoder DEFINEs for numbers of tracks to access with encoder
#define ROTARYSTEPS 1
#define ROTARYMIN  mapData[crntMap]->startTrack
#define ROTARYMAX  mapData[crntMap]->numTracks

//--- Setup a RotaryEncoder for GPIO pins 16, 17:
RotaryEncoder encoder(17, 16);           //--digital pins to read encoder
uint8_t       lastPos = -1;              //-- Last known rotary position.
OneButton     encoderSw1(4, true);       //---Setup  OneButton for rotary 
                                         //   encoder sw on pin 13 - active low

//---RotaryEncoder Setup and variables are in this section---------
uint16_t tracknumChoice  = ROTARYMAX;
uint16_t tracknumActive  = ROTARYMAX;
//uint16_t tracknumDisplay = ROTARYMAX;

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
//void selectYARD();
//void selectTIME();
void leaveTrack_Setup();
void leaveTrack_Active();

//---runMenu Functions Declarations--------------
void runMAINMENU();
void runYARDMENU();
void runDELAYMENU();
void runACTIONMENU();

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
  u8g2.begin(/*Select=*/ 19, /*Right/Next=*/ 18, /*Left/Prev=*/ 23);

  /*---- Setup EEPROM and variables for Menu function------------------*
  *      crntMap and trackActiveDelay variables dictate which staging  *
  *      map and time delay are used by this board.                    *  
  *      The "choice" variables are set to current values for use with *
  *      the runMenu to setup a different combinations when needed.    *
  *********************************************************************/
  EEPROM.begin(EEPROM_SIZE);
  crntMap          = EEPROM.read(0);         //read staging yard map from EEPROM
  trackActiveDelay = EEPROM.read(1);         //read trk pwr delay time from EEPROM
  crntMapChoice          = crntMap;          
  trackActiveDelayChoice = trackActiveDelay; 
      
  //---Setup the sensor pins
  pinMode(mainSensInpin, INPUT_PULLUP); pinMode(mainSensOutpin, INPUT_PULLUP);
  pinMode(revSensInpin, INPUT_PULLUP);  pinMode(revSensOutpin, INPUT_PULLUP);

  //---setup the Bounce pins and intervals :
  debouncer1.attach(mainSensInpin); debouncer2.attach(mainSensOutpin);
  debouncer3.attach(revSensInpin);  debouncer4.attach(revSensOutpin);
  debouncer1.interval(5);           debouncer2.interval(5); // interval in ms
  debouncer3.interval(5);           debouncer4.interval(5); 

  //---set pin for driving track power relay 
  pinMode(trackPowerLED_PIN, OUTPUT); 

  //---set up click routines for the encoder switch
  encoder.setPosition(ROTARYMAX / ROTARYSTEPS); // start with ROTARYMAX value
  encoderSw1.attachClick(click1);
  encoderSw1.attachDoubleClick(doubleclick1);
  encoderSw1.attachLongPressStart(longPressStart1);
  encoderSw1.setPressMs(6000);              //---set longPress delay to 6 seconds,
                                               //    Used to call menu function by
                                               //    holding the encoder switch down

  //---Shift register pins
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin,  OUTPUT);  
  pinMode(clockPin, OUTPUT); 
  

  //---setup variables for start sequence
  writeTrackBits(mapData[crntMap]->routes[mapData[crntMap]->defaultTrack]);
  digitalWrite(trackPowerLED_PIN, HIGH);
              
  tracknumChoice = (mapData[crntMap]->defaultTrack);
  tracknumActive = (mapData[crntMap]->defaultTrack);
  lastPos        = (mapData[crntMap]->defaultTrack);
  
  //---display settings for this board during boot sequence
  u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvB08_te);     
    u8g2.drawStr(30,8, ("STARTING UP!")); 
    u8g2.drawStr(3,25, "YARD:");
    u8g2.drawStr(60,25,mapData[crntMap]->mapName);
    u8g2.drawStr(3,43, "DELAY:");
    enum {BufSize=3};  
    char choiceBuf[BufSize];
      snprintf (choiceBuf, BufSize, "%2d", trackActiveDelay);
    u8g2.drawStr(60,43,choiceBuf);
    u8g2.drawStr(3,64,"S/W:");
    u8g2.drawStr(35,64,swVer);
    u8g2.drawHLine(0, 45, 128); 
   u8g2.sendBuffer();

  delay(5000);
  digitalWrite(trackPowerLED_PIN, LOW);
  
} //-----------------------End setup-----------------------------

//--------------------------------------------------------------//
//                       M A I N  L O O P                       //
//--------------------------------------------------------------//

void loop() 
{                         /*DEBUG__________________________
                          Serial.println("---------------------void Loop-----");
                          Serial.print("trackNumChoice(void loop ent):    ");
                          Serial.println(tracknumChoice);
                          Serial.print("tracknumActive(void loop ent):    ");
                          Serial.println(tracknumActive); */
  
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else                 digitalWrite(trackPowerLED_PIN, LOW);

  if (mode == HOUSEKEEP)         {runHOUSEKEEP();}
  else if (mode ==     STAND_BY) {runSTAND_BY();}
  else if (mode ==  TRACK_SETUP) {runTRACK_SETUP();}
  else if (mode == TRACK_ACTIVE) {runTRACK_ACTIVE();}
  else if (mode ==     OCCUPIED) {runOCCUPIED();}
  else if (mode ==         MENU) {runMENU();}
  
  /*----debug terminal print----------------*/
                          //Serial.print("mainSensTotal:      ");
                          //Serial.print(mainSensTotal);
                          //Serial.print("           revSensTotal:  ");
                          //Serial.println(revSensTotal);
                          //Serial.print("mainPassByState:    ");
                          //Serial.print(mainPassByState);
                          //Serial.print("          revPassByState: ");
                          //Serial.println(revPassByState); 
                          //Serial.print("main_LastDirection: ");
                          //Serial.print(main_LastDirection);
                          //Serial.print("       rev_lastDirection: ");
                          //Serial.println(rev_LastDirection);
                          //Serial.print("tracknumActive(vloop exit):    ");
                          //Serial.println(tracknumActive);
                          //Serial.print("Mode(vloop exit):  ");
                          //Serial.println(mode);

 //---end debug printing-------------
}     
 //------------------------END main loop-------------------
 

/* ---------------------------------------------------------------*
 *                                                                * 
 *                                                                * 
 *            S T A T E  M A C H I N E  F U N C T I O N S         *
 *                                                                * 
 *                                                                * 
 *                                                                * 
 *----------------------------------------------------------------*/

//--------------------HOUSEKEEP Function--------------------------
void runHOUSEKEEP()
{
  oledOn();
                                  /*---DEBUG
                                  Serial.println();
                                  Serial.println("--------------------HOUSEKEEP---"); */
  
  if((tracknumActive < ROTARYMAX) || (mapData[crntMap]->revL == false)) railPower = OFF;
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else  digitalWrite(trackPowerLED_PIN, LOW);
    
  u8g2.clearBuffer();
  tracknumChoiceText();
  
      u8g2.setFont(u8g2_font_helvB10_te);     
      u8g2.drawStr(3,18, "PUSH TO"); 
      //u8g2.setFont(u8g2_font_helvR08_te);   commented out 1/10/2025
      u8g2.drawStr(3,35, "SELECT");
      u8g2.setFont(u8g2_font_helvB10_te);
      if (railPower == ON) {u8g2.drawStr(3,64,"TRK POWER ON"); }
      else {u8g2.drawStr(3,64,"TRK POWER OFF"); }  //added 1/10/2025
  u8g2.drawHLine(0, 45, 128);
  u8g2.sendBuffer(); 
    
  timerOLED.start(interval_OLED);   /*--start sleep timer here for when HOUSEKEEP 
                                      state is entered after moving through states
                                      and no knob twist.                         */
  mode = STAND_BY;
}  

//-----------------------STAND_BY Function-----------------
void runSTAND_BY()
{
                    /*---DEBUG  
                    Serial.println("-----------------------STAND_BY---");
                    Serial.print("------standBy Last Position:  ");
                    Serial.println(lastPos);  */

//---Begin main do-while loop checking for user input--------
  do
  {    
    /*if(timerOLED.done() == true){     //---check screen timer and put in
      oledOff();                      //   sleep mode if time out
    }*/  //---1/10/2025 - Commented out this if statement to keep sreen live at all times

    readEncoder();
    readAllSens();
    encoderSw1.tick();  //check for clicks
    if((mainSens_Report > 0) || (revSens_Report > 0))
    {
                                
                    //--DEBUG: Serial.println("---to OCCUPIED from STAND_BY---");
      oledOn();    
      u8g2.sendBuffer();                
      runOCCUPIED();
    }
  }
  while (knobToggle == true);        //---check rotary switch pressed to select a 
                                     //   track (active low)
  
  tracknumActive = tracknumChoice;  
  knobToggle = true;                 //--reset so readEncoder will run in stand_by
  timerOLED.disable();

  u8g2.sendBuffer();
  
  oledOn();
  
  mode = TRACK_SETUP;                //---move on with new track assignment
} 

//-----------------------TRACK_SETUP- State Function-----------------------
void runTRACK_SETUP()
{
  readAllSens();
  railPower = OFF;
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else  digitalWrite(trackPowerLED_PIN, LOW);
                            //--DEBUG: Serial.println("------------------------TRACK_SETUP---");
  writeTrackBits(mapData[crntMap]->routes[tracknumActive]);

  u8g2.clearBuffer();
  tracknumChoiceText();
    u8g2.setFont(u8g2_font_helvB10_te);     
    u8g2.drawStr(3,18, "WAIT"); 
    u8g2.setFont(u8g2_font_helvB10_te); 
    u8g2.drawStr(3,64,"ALIGNING");
    u8g2.drawHLine(0, 45, 128);
  u8g2.sendBuffer();
  
  timerTortoise.start(interval_Tortoise);   //--begin delay for Tortoises
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
                          //--DEBUG: Serial.println("---Entering leaveTrack_Setup---");
  readAllSens();
  if((mainSens_Report > 0) || (revSens_Report > 0))
  {
                          //--DEBUG: Serial.println("---to OCCUPIED from leaveTrack_Setup---");
    mode = OCCUPIED;
  }
  else 
  {
                          //--DEBUG: Serial.println("--times up--leaving TrackSetup--");
    mode = TRACK_ACTIVE;
  }
}

//-----------------------TRACK_ACTIVE State Function------------------
void runTRACK_ACTIVE()
{
    //---begin timere to keep track power on for "n" minutes
  unsigned long interval_TrainIO  = 1000000L * 60 * trackActiveDelay;  
  
  readAllSens();
    
  u8g2.clearBuffer();
    tracknumChoiceText();
    u8g2.setFont(u8g2_font_helvB10_te);     
    u8g2.drawStr(3,18, "START");
    u8g2.drawStr(3,35, "NOW!"); 
    u8g2.setFont(u8g2_font_helvB10_te); 
    if (railPower == ON) {u8g2.drawStr(3,64,"TRK POWER ON"); }
    else {u8g2.drawStr(3,64,"TRK POWER OFF"); }
    u8g2.drawHLine(0, 45, 128);  
  u8g2.sendBuffer();
  
                    //--DEBUG: Serial.println("-----------------------TRACK_ACTIVE---");
  rev_LastDirection = 0; //reset for use during the next TRACK_ACTIVE call
  main_LastDirection = 0;
  timerTrainIO.start(interval_TrainIO);
  do
  {
     readAllSens();
     encoderSw1.tick();
     
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
                    //--DEBUG: Serial.println("--to OCCUPIED from leavTrack_Active-");
    mode = OCCUPIED;
  }
  else 
  {
                    //--DEBUG: Serial.println("--times up leaving TrackActive--");
    mode = HOUSEKEEP;
  }
}

//-------------------------OCCUPIED State Function--------------------
void runOCCUPIED()
{
                   //--DEBUG: Serial.println("OCCUPIED");
  
  while((mainSens_Report > 0) || (revSens_Report > 0))
  {
    readAllSens();
                    //--DEBUG: Serial.println("----to OCCUPIED from OCCUPIED---");

    u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_helvB10_te);     
      u8g2.drawStr(3,18, "TRACK IS BUSY"); 
      u8g2.setFont(u8g2_font_helvR08_te); 
      u8g2.drawStr(3,35, "Wait");
      u8g2.setFont(u8g2_font_helvB10_te); 
      if (railPower == ON) {u8g2.drawStr(3,64,"TRK POWER ON"); }
      else {u8g2.drawStr(3,64,"TRK POWER OFF"); }
      u8g2.drawHLine(0, 45, 128); 
    u8g2.sendBuffer();
  }
                    //--DEBUG: Serial.println("----Leaving OCCUPIED---");
  runHOUSEKEEP();
}

//------------------------ReadEncoder Function----------------------

void readEncoder()
{ 
  encoder.tick();
  int newPos = encoder.getPosition() * ROTARYSTEPS;
                    /*---DEBUG
                    Serial.println("-------ENCODER");
                    Serial.print("lastPos: ");
                    Serial.println(lastPos);   
                    Serial.print("newPos: ");
                    Serial.println(newPos);  */
                           
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
    
    oledOn();
                    /*---DEBUG
                    Serial.println("-------ENCODER");
                    Serial.print("newPos: ");
                    Serial.println(newPos);   
                    Serial.print("Choice: ");
                    Serial.println(tracknumChoice);
                    Serial.print("Active: ");
                    Serial.println(tracknumActive);
                    Serial.print("bailOut:   ");
                    Serial.println(bailOut);  
                    Serial.println("leave ENCODER"); */ 
                        
    timerOLED.start(interval_OLED);          //--sleep timer for STAND_BY mode

    u8g2.clearBuffer();
      tracknumChoiceText();
      //tracknumActiveTextSm();  commented out 1/10/2024
      u8g2.setFont(u8g2_font_helvB10_te);     
      u8g2.drawStr(3,18, "PUSH TO"); 
      //u8g2.setFont(u8g2_font_helvR08_te); commented out 1/10/2024
      u8g2.drawStr(3,35, "SELECT");
      u8g2.setFont(u8g2_font_helvB10_te); 
      //u8g2.drawStr(3,64,"ACTIVE");
      u8g2.drawStr(3,64,"TRK POWER OFF");
      u8g2.drawHLine(0, 45, 128);  
    u8g2.sendBuffer();
    Serial.println("---------sendBuffer CHOICE");
  }
} 

/****************runMENU functions note******************************
*   See the notes in the main code explanation above.               *
*********************************************************************/

void runMENU()  
{
  oledOn();
  runMAINMENU();
}

void runMAINMENU() {
    int menuSelect = 0;
    
    menuSelect = u8g2.userInterfaceSelectionList(
      "Select Task", 
      1, 
      "Yard\n"
      "Time\n"
      "Accept & Exit\n"
      "EXIT"
      );

    if     (menuSelect == 1) runYARDMENU();  
    else if(menuSelect == 2) runDELAYMENU();  
    else if(menuSelect == 3) {
      EEPROM.write(0, crntMapChoice);         //---Write new yard selection to eeprom
      EEPROM.commit();
      EEPROM.write(1, trackActiveDelayChoice);//---Write new delay time
      EEPROM.commit();
      runHOUSEKEEP();
    }  
    else if(menuSelect == 4) {
      crntMapChoice = crntMap;
      trackActiveDelayChoice = trackActiveDelay;
      runHOUSEKEEP();
    }
  }

 void runYARDMENU() {          //---select the yard for this board
    int yardSelect = 0;
    
    yardSelect = u8g2.userInterfaceSelectionList(
      "Select Yard", 
      1, 
      "Wheeling\n"    
      "Parkersburg\n" 
      "Bayview\n"
      "Cumberland\n"
      "Test\n"
      "Charleston\n"
      "Curtis_Bay\n"
      "WestStaging\n"
      "Cancel"
      );

    if (yardSelect >= 1 && yardSelect < 9) {
      crntMapChoice = yardSelect - 1;
      runMAINMENU();
      
    }
    else if (yardSelect == 8) runMAINMENU();
  }

  void runDELAYMENU() {       //---select the delay time for this board
    int delaySelect = 0;
    delaySelect = u8g2.userInterfaceSelectionList(
      "Select Delay Min", 
      1, 
      "No Delay\n"
      "1 Minute\n"
      "2 Minutes\n"
      "3 Minutes\n"
      "4 Minutes\n"
      "5 Minutes\n"
      "6 Minutes\n"
      "Cancel"
      );

    if (delaySelect >= 1 && delaySelect <8) {   
      trackActiveDelayChoice = delaySelect - 1;
      runMAINMENU();
      
    }
    else if (delaySelect == 8) runMAINMENU();  //---if choice is cancel: bail
  }
//-----END runMENU FUNCTIONS------------------------------------------


/***********************UPDATE SENSOR FUNCTIONS*************************
*   All functions in this section update and track sensor information: * 
*   Busy, Direction, PassBy.  Only the mainOut sensor is documented.   *  
*   The remaining three work identically.                              *
********************************end of note****************************/

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

// -----------------------DISPLAY FUNCTIONS---------------------//
//                          BEGIN HERE                          //
//--------------------------------------------------------------//

void tracknumChoiceText()
{
  u8g2.setFont(u8g2_font_fub35_tf);
  enum {BufSize=3};  
  char choiceBuf[BufSize];
  snprintf (choiceBuf, BufSize, "%2d", tracknumChoice);
    if((tracknumChoice == ROTARYMAX) && (mapData[crntMap]->revL  == true)) u8g2.drawStr(72,40,"RL");  
    else u8g2.drawStr(72,40,choiceBuf);   
}

void tracknumActiveText()    
{
  u8g2.setFont(u8g2_font_fub35_tf);
  enum {BufSize=3};  
  char activeBuf[BufSize];
  snprintf (activeBuf, BufSize, "%2d", tracknumActive);
  if((tracknumActive == ROTARYMAX) && (mapData[crntMap]->revL  == true)) u8g2.drawStr(68,40,"RL");   
  else  u8g2.drawStr(68,40,activeBuf ); 
}

void tracknumActiveTextSm()    //---Small text, used in the bottom line of the OLED
{
  u8g2.setFont(u8g2_font_helvB12_te);
  enum {BufSize=3};  
  char activeBuf[BufSize];
  snprintf (activeBuf, BufSize, "%2d", tracknumActive);
  if((tracknumActive == ROTARYMAX) && (mapData[crntMap]->revL  == true)) u8g2.drawStr(90,64,"RL");   
  else  u8g2.drawStr(90,64,activeBuf ); 
}

void tracknumActChoText()
{
  u8g2.setFont(u8g2_font_fub35_tf);
  enum {BufSize=3};  
  char choiceBuf[BufSize];
  snprintf (choiceBuf, BufSize, "%2d", tracknumChoice);
    if((tracknumChoice == ROTARYMAX) && (mapData[crntMap]->revL  == true)) u8g2.drawStr(72,40,"RL");  
    else u8g2.drawStr(72,40,choiceBuf);   
}  

void oledOn()
 {
  u8g2.setPowerSave(0);
  oledState = true;
 }


void oledOff()
 {
  u8g2.setPowerSave(1);
  oledState = false;
  }

//----------ROTARY ENCODER AND ENCODER SWITCH FUNCTIONS-----------//
//                          BEGIN HERE                            //
//----------------------------------------------------------------//

void click1(){                //--singleclick: if sleeping: awaken OLED  
  if(oledState == false){
    timerOLED.start(interval_OLED);
    oledOn();
    u8g2.sendBuffer();
  }
  else knobToggle = false;    //  else set trackChoice and move to setup             
}

void doubleclick1(){          //--doubleclick: reset trainIO timer to 0
    timerTrainIO.disable();
}

void longPressStart1(){       //--hold for 6 seconds: goto Main Setup Menu
    runMENU();
}

//----------------Shift Register Function--------------//

void writeTrackBits(uint16_t track)
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, (track >> 8));
  shiftOut(dataPin, clockPin, MSBFIRST, track);
  digitalWrite(latchPin, HIGH);
            /*/--DEBUG: 
            Serial.print("trackFunction: ");
            Serial.println(track);
            Serial.println(track, BIN);  */
}  



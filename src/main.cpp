//
//Jeroen Garritsen's B&O McKenzie Division - Staging Yard Project
//by: Mark Kellogg - Began: 4/23/2020
//
//---------------------------Github Repository------------------------------
// NAME:
// The repository has all code etc. along with flow control overview and 
// module graphics.  
//
// Designed for use on Jeroen Gerritsen's B&O McKenzie Div layout.
// A main control panel uses an ESP32 to set up active staging yard
// tracks with a second ESP32 and H-bridge chips to drive the Tortoise
// switch machines, control track power, etc. The project requires remote 
// control panels, so the second ESP32 is used to eliminate long multiple
// cable runs to the control panels.
// 
//
// Panel
//
// Track power timer
//
// Entrance and Exit sensors - 
//
//----------------------------Track Sensors Descriptions--------------------
// All four staging yards have a single yard lead, from which all 
// the dead end staging tracks fan out.  The yard lead of three of the four 
// yards continue on to a reverse loop.
//
//   Sensors - The yard lead entry turnout and reverse loop turnout each have a 
//   sensor pair to track entry and exits from that point.
//
//   Sensor Names - Sensors are named "mainSens" for the yard throat, and 
//   "revSens" for the reverse loop leadout.
//
//   Direction Naming Convention - In all cases you may think of a point "between"
//   the yard lead turnout and the reverse loop turnout as the center of the
//   universe.  Therefore INBOUND is always towards that point on either sensor
//   and OUTBOUND obviously the reverse.   
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
#include "bcsjTimer.h"
#include <OneButton.h>

//------------Setup sensor debounce from Bounce2 library-----
#define mainSensInpin  26
#define mainSensOutpin 27
#define revSensInpin   12 
#define revSensOutpin  14

//------------------Wheeling Track Map---------------------------------
#define	THROWN_T0	  0x0001	//0000000000001	1
#define	THROWN_T1	  0x0002	//0000000000010	2
#define	THROWN_T2	  0x0004	//0000000000100	4
#define	THROWN_T3	  0x0008	//0000000001000	8
#define	THROWN_T4	  0x0010	//0000000010000	16
#define	THROWN_T5	  0x0020	//0000000100000	32
#define	THROWN_T6	  0x0040	//0000001000000	64
#define	THROWN_T7	  0x0080	//0000010000000	128
#define	THROWN_T8	  0x0100	//0000100000000	256
#define	THROWN_T9	  0x0200	//0001000000000	512
#define	THROWN_T10	0x0400	//0010000000000	1024
#define	THROWN_T11	0x0800	//0100000000000	2048
#define	THROWN_T12	0x1000	//1000000000000	4096

const int W1 = THROWN_T1+THROWN_T2+THROWN_T3+THROWN_T4;
const int W2 = THROWN_T0+THROWN_T2+THROWN_T3+THROWN_T4;
const int W3 = THROWN_T0+THROWN_T1+THROWN_T3+THROWN_T4;
const int W4 = THROWN_T0+THROWN_T1+THROWN_T2+THROWN_T4;
const int W5 = THROWN_T0+THROWN_T1+THROWN_T2+THROWN_T3;
 



#define INBOUND   1    
#define OUTBOUND  2
#define CLEAR     0
#define ON        0
#define OFF       1

#define trackPowerLED_PIN  4  //debug

#define tortiOne   2   //test torti

//---Instantiate a bcsjTimer.h object for screen sleep
bcsjTimer  timerOLED;
bcsjTimer  timerTortoise;
bcsjTimer  timerTrainIO;

//---Timer Variables---
unsigned long interval_OLED    = 1000000L * 60 * 0.5;
unsigned long tortoiseInterval = 1000000L * 6;
unsigned long intervalTrainIO  = 1000000L * 60 * .25;

// Instantiate a Bounce object
Bounce debouncer1 = Bounce(); Bounce debouncer2 = Bounce(); 
Bounce debouncer3 = Bounce(); Bounce debouncer4 = Bounce();

//------------Set up OLED Screen-----
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//-------Declaration for an SSD1306 display - using I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//--RotaryEncoder DEFINEs for numbers of tracks to access with encoder
#define ROTARYSTEPS 1
#define ROTARYMIN   1
#define ROTARYMAX   6

//--- Setup a RotaryEncoder for pins A2 and A3:
RotaryEncoder encoder(16, 17);
byte lastPos = -1;                   //--- Last known rotary position.
//const int rotarySwitch   = 13;       //---Setup Rotary Encoder switch on 
                                     //   pin 13 - active low ----------- 
OneButton encoderSw1(13, true);    // Setup a new OneButton on pin 13

//--OneButton Functions for RotaryEncoder switch
void click1();
void doubleclick1();
void longPressStart1();


//------RotaryEncoder Setup and variables are in this section---------
byte tracknumChoice  = ROTARYMAX;
byte tracknumActive  = ROTARYMAX;
byte tracknumDisplay = ROTARYMAX;
byte tracknumLast    = ROTARYMAX;

//Rotary Encoder Switch Variables
byte knobPosition = ROTARYMAX;
bool knobToggle   = true;       //active low 
void readEncoder();           //--RotaryEncoder Function------------------



//---------------------OLED Display Functions------------------//
void bandoText(String text, int x, int y, int size, boolean d);

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
//--end sensor functions---

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

bool entry_ExitBusy = false;
//----end of sensor variables

//DEBUG SECTION
const int bailOutSW  = 15;  // setup switch for bailout
byte      bailOut    = 1;  //active low
//----END DEBUG--------------- //

//--------------------------------------------------------------//
//                         void setup()                         //
//--------------------------------------------------------------//

void setup() 
{
  Serial.begin(115200);
  delay(1000);  //time to bring up serial monitor
  //tracknumLast = ROTARYMAX;
  
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
      Serial.println(F("SSD1306 allocation failed"));
      for (;;); // Don't proceed, loop forever
    }
  
  //---Setup the button (using external pull-up) :
  pinMode(mainSensInpin, INPUT_PULLUP); pinMode(mainSensOutpin, INPUT_PULLUP);
  pinMode(revSensInpin, INPUT_PULLUP);  pinMode(revSensOutpin, INPUT_PULLUP);

  // After setting up the button, setup the Bounce instances :
  debouncer1.attach(mainSensInpin); debouncer2.attach(mainSensOutpin);
  debouncer3.attach(revSensInpin);  debouncer4.attach(revSensOutpin);
  debouncer1.interval(5);           debouncer2.interval(5); // interval in ms
  debouncer3.interval(5);           debouncer4.interval(5); 

  //DEBUG Section - these are manual switches until functions are ready
  //pinMode(bailOutSW, INPUT_PULLUP);
  pinMode(trackPowerLED_PIN, OUTPUT);

  pinMode(tortiOne, OUTPUT);
  
  //----END DEBUG---------------

  encoder.setPosition(ROTARYMIN / ROTARYSTEPS); // start with the value of ROTARYMIN

  encoderSw1.attachClick(click1);
  encoderSw1.attachDoubleClick(doubleclick1);
  encoderSw1.attachLongPressStart(longPressStart1); 

  
  digitalWrite(trackPowerLED_PIN, HIGH);
  display.clearDisplay();
  bandoText("B&O RAIL",25,0,2,false);
  bandoText("JEROEN GARRITSEN'S",8,20,1,true);
  bandoText("McKENZIE",0,33,2,true);
  bandoText("DIVISION",30,50,2,true);
  display.display();
  delay(5000);
  display.clearDisplay();
  digitalWrite(trackPowerLED_PIN, LOW);
}  
//---End setup

//--------------------------------------------------------------//
//                          void loop()                         //
//--------------------------------------------------------------//

void loop() 
{
      if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
      else  digitalWrite(trackPowerLED_PIN, LOW);

      if (mode == HOUSEKEEP)
  {
      runHOUSEKEEP();
  }

  else if (mode == STAND_BY)
  {
      runSTAND_BY();
  }

  else if (mode == TRACK_SETUP)
  {
    runTRACK_SETUP();
  }

  else if (mode == TRACK_ACTIVE)
  {
    runTRACK_ACTIVE();
  }

  else if (mode == OCCUPIED)
  {
    runOCCUPIED();
  }

  else if (mode == MENU){
    runMENU();
  }
  
  //----debug terminal print----------------
      
      //Serial.print("mainOutValue: ");
      //Serial.print(mainOutValue);
      //Serial.print("        revOutValue: ");
      //Serial.println(revOutValue);
      //Serial.print("mainSens_Report: ");
      //Serial.print(mainSens_Report);
      //Serial.print("     revSens_Report: ");
      //Serial.println(revSens_Report);
      Serial.print("mainSensTotal:      ");
      Serial.print(mainSensTotal);
      Serial.print("           revSensTotal:  ");
      Serial.println(revSensTotal);
      //Serial.print("mainPassByTotal: ");
      //Serial.print(mainPassByTotal);
      /////Serial.print("     revPassByTotal: ");
      /////Serial.println(revPassByTotal); 
      Serial.print("mainPassByState:    ");
      Serial.print(mainPassByState);
      Serial.print("          revPassByState: ");
      Serial.println(revPassByState); 
      //Serial.print("entryExitBusy: ");
      //Serial.println(entry_ExitBusy); 
      //Serial.print("mainDirection: ");
      //Serial.print(mainDirection);  
      //Serial.print("   revDirection: ");
      //Serial.println(revDirection);  
      Serial.print("main_LastDirection: ");
      Serial.print(main_LastDirection);
      Serial.print("       rev_lastDirection: ");
      Serial.println(rev_LastDirection);
      Serial.print("tracknumActive:    ");
      Serial.print(tracknumActive);
      Serial.print("           tracknumLast: ");
      Serial.println(tracknumLast);
      /*
      Serial.println();
      Serial.println("=======Report Starts Here!=======");
      delay(307);
      //---end debug printing   
      */

}  
//---END void loop
 
// ---------------State Machine Functions Section----------------//
//                          BEGINS HERE                          //
//---------------------------------------------------------------//



//--------------------HOUSEKEEP Function-----------------
void runHOUSEKEEP()
{
  display.ssd1306_command(0xAF); // turn OLED on
  
  Serial.println();
  Serial.println("-----------------------------------------HOUSEKEEP---");

  if(tracknumLast < ROTARYMAX) railPower = OFF;
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else  digitalWrite(trackPowerLED_PIN, LOW);
  
  tracknumChoice = tracknumLast;

  enum {BufSize=3};  
  char buf[BufSize];
  snprintf (buf, BufSize, "%2d", tracknumLast);
  display.clearDisplay();
  bandoText("SELECT NOW",0,0,2,false);
  bandoText("TRACK",0,20,2,false);
   bandoText("W",70,20,2,false);
  if(tracknumChoice == ROTARYMAX) bandoText("Rev",88,20,2,false);
  else bandoText(buf,82,20,2,false);
  bandoText("PUSH BUTTON TO SELECT",0,46,1,false);
  bandoText("TRACK POWER  -HK-",0,56,1,true);

  timerOLED.start(interval_OLED);   /*--start sleep timer here for when HOUSEKEEP 
                                      state is entered after moving through states
                                      and no knob twist.                         */
  mode = STAND_BY;
}  

//-----------------------STAND_BY Function-----------------
void runSTAND_BY()
{
    
    
  Serial.println("-----------------------------------------STAND_BY---");
  do
  {    
    if(timerOLED.done() == true){       //--check display sleep timer
    display.ssd1306_command(0xAE);      //--turn off display if timed out
    }
    
    readEncoder();
    encoderSw1.tick();
            
    readAllSens();
    if((mainSens_Report > 0) || (revSens_Report > 0))
    {
      Serial.println("---to OCCUPIED from STAND_BY---");  //debug
      display.ssd1306_command(0xAF);  //--awake display in case it is in sleep mode 
                                      //before moving to OCCUPIED.
      runOCCUPIED();
    }
  }
  while (knobToggle == true);        //check rotary switch pressed to select a track (active low)
  
  tracknumActive = tracknumChoice;  
  knobToggle = true;                //--reset so readEncoder will run in stand_by
  timerOLED.disable();              //--turn off screen sleep timer
  display.ssd1306_command(0xAF);    //--awake display if in sleep mode
  display.display(); 
  mode = TRACK_SETUP;               //---move on with new track assignment
} 


//-----------------------TRACK_SETUP- State Function-----------------------
void runTRACK_SETUP()
{
  readAllSens();
  
  railPower = OFF;
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else  digitalWrite(trackPowerLED_PIN, LOW);

  Serial.println("-----------------------------------------TRACK_SETUP---");
  tracknumLast = tracknumActive;

  

  display.clearDisplay();
  enum {BufSize=3};  
  char buf[BufSize];
  snprintf (buf, BufSize, "%2d", tracknumActive);
  display.clearDisplay();
  bandoText("ALIGNING",0,0,2,false);
  bandoText("TRACK",0,20,2,false);
  bandoText("W",70,20,2,false);
  if(tracknumChoice == ROTARYMAX) bandoText("Rev",88,20,2,false);
  else bandoText(buf,82,20,2,false);
  bandoText("HAVE A NICE DAY",0,46,1,false);
  bandoText("TRACK POWER  -OFF-",0,56,1,true);
    
  
  timerTortoise.start(tortoiseInterval);   //--begin delay for Tortoises
  while(timerTortoise.running() == true)
  {
   readAllSens();
  }
  railPower = ON;
  if(railPower == ON)  digitalWrite(trackPowerLED_PIN, HIGH);
  else  digitalWrite(trackPowerLED_PIN, LOW);
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
    bandoText("PROCEED ",20,0,2,false);
    bandoText("TIMER ON",0,20,2,false);
    bandoText("TRACK POWER  -ON-",0,56,1,true);

  Serial.println("-----------------------------------------TRACK_ACTIVE---");
  rev_LastDirection = 0; //reset for use during the next TRACK_ACTIVE call
  main_LastDirection = 0;
 
  timerTrainIO.start(intervalTrainIO);
  do
  {
     readAllSens();
     bailOut = digitalRead(bailOutSW);
     //---debug     
     Serial.print("main_LastDirection: ");
     Serial.print(rev_LastDirection);
     Serial.println(main_LastDirection);
     Serial.println("-----Waiting for Train to Exit!");
     //debug
     
        //--true when outbound train completely leaves sensor  
    if (((mainPassByState == 1) && (main_LastDirection == 2)) ||
         ((rev_LastDirection == 2) && (revPassByState == 1)))           
    {
      break;
    }

    if(bailOut == 0)     //bailout is toggle switch on panel used to end timer if desired
    {
      break;
    }
  }
  
  while(timerTrainIO.running() == true);

  mainPassByState = false;
  revPassByState = false;
  leaveTrack_Active();
}  //--end runTrack_Active---


void leaveTrack_Active()
{
  readAllSens();
  if((mainSens_Report > 0) || (revSens_Report > 0))
  {
    Serial.println("----to OCCUPIED from leavTrack_Active---");
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
    bandoText("YARD LEAD",0,0,2,false);
    bandoText("OCCUPIED",0,20,2,false);
    bandoText("STOP!",20,42,2,true);
  }
   
  Serial.println("----Leaving OCCUPIED---");
  runHOUSEKEEP();
}

//------------------------ReadEncoder Function----------------------

void readEncoder()
{
 
  encoder.tick();
  //Serial.println(".tick");  //debug
  // get the current physical position and calc the logical position
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
    
    display.ssd1306_command(0xAF);
    Serial.print(newPos);   
    Serial.println();
    Serial.print("Choice: ");
    Serial.println(tracknumChoice);
    Serial.print("Active: ");
    Serial.println(tracknumActive);
    Serial.print("Last: ");
    Serial.println(tracknumLast); 
    delay(50);

    timerOLED.start(interval_OLED);   //--sleep timer for STAND_BY mode

    enum {BufSize=3};  
    char buf[BufSize];
    snprintf (buf, BufSize, "%2d", tracknumChoice);
    display.clearDisplay();
    bandoText("SELECT NOW",0,0,2,false);
    bandoText("TRACK",0,20,2,false);
    bandoText("W",70,20,2,false);
    if(tracknumChoice == ROTARYMAX) bandoText("Rev",88,20,2,false);
    else bandoText(buf,82,20,2,false);
    bandoText("PUSH BUTTON TO SELECT",0,46,1,false);
    bandoText("TRACK POWER  -OFF-",0,56,1,true);
    
  }
} 

void runMENU()
{
  display.ssd1306_command(0xAF);
  display.clearDisplay();
  bandoText("SETUP MENU",0,0,2,false);
  bandoText("DEMO ONLY",0,20,2,false);
  bandoText("LATER, DUDE",0,56,1,true);
  mode = HOUSEKEEP;
}

//---------------------Updating Sensor Functions------------------
//  All functions in this section update and track sensor information: 
//  Busy, Direction, PassBy.  Only the mainOut sensor is documented.  
//  The remaining three work identically.
//------------------------------end of note-----------------------

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
    //---PassByTotal of "6" means train has cleared the sensor success-
    //  fully.  If train were to back out of sensors the sensors would
    //  fire and up the count, the condition would not ever be met.
    //---------end of note------  

    if(mainSensTotal == 0 && mainPassByTotal == 6) 
      {
       mainPassByState = true;
       mainPassByTotal = 0;
      }
    else if(mainSensTotal == 0) mainPassByTotal = 0;

    //--report mainLine Direction
    if((mainSensTotal == 2) && (mainSens_Report == 2)) 
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
    if(revSensTotal == 0 && revPassByTotal == 6) 
    {
      revPassByState = true;
      revPassByTotal = 0;
    }
    else if(revSensTotal == 0) revPassByTotal = 0;

    //--report revLoop Direction
    if((revSensTotal == 2) && (revSens_Report == 2)) 
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

  
void readAllSens() 
  {
    readMainSens();
    readRevSens();
  }   

// ------------------Display Functions Section-------------------//
//                          BEGINS HERE                          //
//---------------------------------------------------------------//

void bandoText(String text, int x, int y, int size, boolean d){
  display.setTextSize(size);
  display.setTextColor(WHITE);
  display.setCursor(x,y);
  display.println(text);
  if(d){
    display.display();
  }
}  //--display function end

//----------Rotary Encoder Click Functions----------- -----------//

void click1() {                     //--wake display on single click
  display.ssd1306_command(0xAF);    //--turn on display
  timerOLED.start(interval_OLED);   //--sleep timer 
  enum {BufSize=3};  
  char buf[BufSize];
  snprintf (buf, BufSize, "%2d", tracknumChoice);
  display.clearDisplay();
  bandoText("SELECT NOW",0,0,2,false);
  bandoText("TRACK",0,20,2,false);
  if(tracknumChoice == ROTARYMAX) bandoText("RevL",70,20,2,false);
  else bandoText(buf,80,20,2,false);
  bandoText("PUSH BUTTON TO SELECT",0,46,1,false);
  bandoText("TRACK POWER  -OFF-",0,56,1,true);
} // click1


void doubleclick1(){                //--double click: read track, goto setup
  knobToggle = false;                
}

void longPressStart1(){             //--hold for 3 seconds goto Main Setup Menu
  runMENU();
}

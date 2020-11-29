    enum {HOUSEKEEP, STAND_BY, TRACK_SETUP, TRACK_ACTIVE, OCCUPIED, MENU, YARD_SEL, TRKPWR_SEL} mode;
    void runHOUSEKEEP();
    void runSTAND_BY();
    void runTRACK_SETUP();
    void runTRACK_ACTIVE();
    void runOCCUPIED();
    void runMENU();
    void runYARD_SEL();
    void runTRKPWR_SEL();
    void leaveTrack_Setup();
    void leaveTrack_Active();



 void runMENU()
{  
                        Serial.println("------------------------------runMENU---");

  oledOn();
  display.clearDisplay();
  bandoText("SETUP MENU",0,12,1,false);
  bandoText("Set Board for:",0,36,1,true);
  bandoText("yard and delay.",0,60,1,true);
  delay(3000);

  
  

  
  runHOUSEKEEP();
}
  
void readEncoder_1(int mainMenu_max, int mainMenu_min)
{ 
  byte lastPos = -1;
    
  encoder.tick();
  // get "choice" physical position and calc the logical position
  int newPos = encoder.getPosition() * ROTARYSTEPS;
  if (newPos < mainMenu_min) {
    encoder.setPosition(mainMenu_min / ROTARYSTEPS);
    newPos = mainMenu_min;
  } 
  else if (newPos > mainMenu_max) {
    encoder.setPosition(mainMenu_max / ROTARYSTEPS);
    newPos = mainMenu_max;
  } 
  if (lastPos != newPos) {
    lastPos = newPos;
    crntMapChoice = newPos;
    
    /*                    Serial.print(newPos);   
                        Serial.println();
                        Serial.print("Choice: ");
                        Serial.println(tracknumChoice);
                        Serial.print("Active: ");
                        Serial.println(tracknumActive);
                        Serial.print("bailOut:   ");
                        Serial.println(bailOut);
    */                      
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

  bandoText(">",0,12,1,false);
  bandoText("EXIT to PGM",13,12,1,false);
  bandoText("Yard Select",13,36,1,false);
  bandoText("TrkPwr Time",13,60,1,true);
  delay(5000);
  

  /*EEPROM.write(0, crntMapChoice);
  EEPROM.commit();
  EEPROM.write(1, trackActiveDelayChoice);
  EEPROM.commit();
  
  Serial.print("crntMapChoice:...............");
  Serial.println(crntMapChoice);
  Serial.print("trackActiveDelayChoice:...");
  Serial.println(trackActiveDelayChoice);


  runHOUSEKEEP();
  */
}

  
  
  
  
  
  
  
  
  
  
  
  
  
  
  EEPROM.write(0, crntMapChoice);
  EEPROM.commit();
  EEPROM.write(1, trackActiveDelayChoice);
  EEPROM.commit();
  
  Serial.print("crntMapChoice:...............");
  Serial.println(crntMapChoice);
  Serial.print("trackActiveDelayChoice:...");
  Serial.println(trackActiveDelayChoice);


   
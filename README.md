# mckenzie_staging_ESP32
Jeroen Gerritsen's B&O McKenzie Division - Staging Yard Project
by: Mark Kellogg - Began: 4/23/2020


 Designed for use on Jeroen Gerritsen's B&O McKenzie Div layout. A main control panel uses an ESP32 to set up active staging yard tracks using H-bridge drivers for the Tortoise stall motor switch machines, control track power, etc. The project requires remote control panels and uses I2C extenders for up to 20 foot cable runs.
 

 Panel - OlED 1.3" screen, rotary encoder with switch, and a reset switch

 Track power timer - Track power is turned off after trains either clear the yard or park

 Entrance and Exit sensors - Control signals and access to the yard leads

----------------------------Track Sensors Descriptions--------------------
 All four staging yards have a single yard lead, from which all the dead end staging tracks fan out.  The yard lead of three of the four yards continue on to a reverse loop.

   Sensors - The yard lead entry turnout and reverse loop turnout each have a sensor pair to track entry and exits from that point.

   Sensor Names - Sensors are named "mainSens" for the yard throat, and "revSens" for the reverse loop leadout.

   Direction Naming Convention - In all cases you may think of a point "between" the yard lead turnout and the reverse loop turnout as      the center of the universe.  Therefore INBOUND is always towards that point on either sensor and OUTBOUND obviously the reverse.   


 Module Output- Each sensor pair returns: 
   Train Direction- mainDirection or revDirection: their output(s) are INBOUND, OUTBOUND, and are active when the train is within the       small sensor area.
   
   Train PassBy - which senses the train has passed by the sensor completely in the direction it arrived from.  If a train backs out        without completely passing by PassBy will not report true. It stays active as long as the train is within the sensor pair.

   Sensor Busy - Sensor reports busy when either sensor of either sensor pair is true and remains so until all return false.

   Release History
    11/28/2020 Test Release v1.0 to be installed on layout with two test boards and one Control Panel Box to share between the two.
    Boards, two each:  Cumberland and Bayview
    Delivered test system burned with TEST yard.

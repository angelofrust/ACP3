# ACP3
Artemis Spaceship Bridge Simulator Controls  
  https://artemisspaceshipbridge.com/

Attached sketches are written for Teensy-LC using Arduino/Teensyduino:  
  https://www.arduino.cc/en/Main/Software  
  https://www.pjrc.com/teensy/td_download.html
  
Current sketches are work-in-progress, adapting code from a previous version of the control panels, but the same Teensy-LC microcontroller.  
ACP3-X-CTL-DMX_REV_A.ino //test master controller code to check functionality of the ACP3 control panel communciations  
ACP3-N-CTL_REV_A.ino //fully-functional code to play helm using the old center and SRS panels, but new nav panel  
ACP3-E-CTL_REV_A.ino //incomplete engineering panel code  

Artemis_Weapons_block3_rev_A_20180327.ino //old weapons code  
Artemis_Helm_block3_rev_B_20180319 //old helm code  
Artemis_Engineering_block3_rev_B_20180319 //old engineering code  
Artemis_Science_block3_rev_A_20180327 //old science code  
Artemis_Comms_block3_rev_A_20180404 //old communications code  
Panel_MCP_20180401 // old master control code - End of Line.  

DMXcommands.xml //DMX script for Artemis to use master control -- place in "dat" folder

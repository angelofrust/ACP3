# ACP3
Artemis Spaceship Bridge Simulator Controls  
  https://artemisspaceshipbridge.com/

Attached sketches are written for Teensy-LC using Arduino/Teensyduino:  
  https://www.arduino.cc/en/Main/Software  
  https://www.pjrc.com/teensy/td_download.html
  
ACP3 Guide...pdf  --- Very important documentation to help get started

Current controller sketches have been updated. 201910 is the latest version. Alll sketches are for the Teensy-LC microcontroller.  
ACP3-H-DMX. . . .ino //code for Helm control panel; doubles as master controller for all connected controllers  
ACP3-E-CTL. . . .ino //engineering panel code 
ACP3-W-CTL. . . .ino //weapons panel code  
ACP3-S-CTL. . . .ino //science panel code  
ACP3-C-CTL. . . .ino //comms panel code  

Artemis_Weapons_block3_rev_A_20180327.ino //old weapons code  
Artemis_Helm_block3_rev_B_20180319 //old helm code  
Artemis_Engineering_block3_rev_B_20180319 //old engineering code  
Artemis_Science_block3_rev_A_20180327 //old science code  
Artemis_Comms_block3_rev_A_20180404 //old communications code  
Panel_MCP_20180401 // old master control code - End of Line.  

DMXcommands.xml //DMX script for Artemis to use master control -- place in "dat" folder

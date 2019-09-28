// Artemis SBS Controller Code
// ACP3 WEAPONS Controller Prototype

// code optimized for 1280 x 720 client resolution - will not work with other resolutions

// digital inputs from the keypad matrices are arranged active HIGH with a pull-down resistor to make use of the shift registers

// connected panels array - used to tell controller which pins should be read when reading controls - false disables input
bool connected_panels[4] = {true, true, true, false};
// 0 - "right" panel - pins 5, 6, 7, (22) - pin 22 available for additional buttons
// 1 - "left" panel - pins 24, 25, 26, (23) - pin 23 available for additional buttons
// 2 - "center" panel - pins 8, 10, 11, (12) - pin 12 available for additional buttons
// 3 - "center" panel - pins 14, 15, 16, 17, 18, 19, 20, 21 - intended for analog input

// control button functions array - stores button code and press state by relative pin reference
int button_array[8][3][4][2];
// first index is to shift register pin number - generally only one pin will be active at a time
// second index is to the input panel number (0 = "right", 1 = left, 2 = "center")
// third index is to the input pin number - all input pins will generally be available simultaneously
// fourth index is to the button state and button code:
    // button state - index 0: number of cycles active
    // button code - index 1: numberic code for the command associated with the button
int active_button_pin = 0;  // stores which of the eight button pins on the shift register is on
#define MAX_BUTTON_PIN 7   // last pin of the button array
#define BUTTON_PUSH_THRESHOLD 3   // number of cycles the button switch must be closed to register as a button push

// control button pin references - stores pin numbers associated with each pin reference in the functions array
int button_pins[3][4] = {
  {5, 6, 7, 7},      // panel 0 "right": 5 - shift serial, 6&7 - button inputs
  {24, 25, 26, 21},     // panel 1 "left": 24 - shift serial, 25&26 - button inputs
  {8, 10, 11, 20}    // panel 2 "center": 8 - shift serial, 10&11 - button inputs
};

#define MAX_COMMAND_QUEUE 30
int command_queue[MAX_COMMAND_QUEUE][3];   // array to store commands to be executed by the button-push code
  // index 0: commmand number
  // index 1: x coord
  // index 2: y coord
int current_command = 0;    // variable that points to the current command in the queue
int last_command = 0;    // variable that points to the command after the last command enterred in the queue
unsigned long command_timer = 0;   // variable to store the time between consecutive keyboard commands
#define COMMAND_INTERVAL 50 // milliseconds in between consecutive keyboard commands to the connected PC

// panel indicator light array - stores light states -- true = on, false = off
bool light_array[8][8][3];  // 1st index = supply register 0 - 7
                            // 2nd index = sink register 0 - 7
                            // 3rd index = panel (0 = "right", 1 = "left", 2 = "center")
#define MAX_LIGHT_SUPPLY 7  // last pin of the current supply chips for the entire panel
                            // controller will not cycle lights further down the registers
#define MAX_LIGHT_SINK 6    // last pin of the current sink chips for the entire panel
                            // controller will not cycle lights further down the registers
int active_light_sink = 0;  // currently on sink register pin

// STA panel array
byte STA_shields[20][2] = {
  {0, 0},
  {1, 0},
  {2, 0},
  {3, 0},
  {4, 0},
  {5, 0},
  {6, 0},
  {7, 0},
  {0, 1},
  {1, 1},
  {2, 1},
  {3, 1},
  {4, 1},
  {5, 1},
  {6, 1},
  {7, 1},
  {0, 2},
  {1, 2},
  {2, 2},
  {3, 2}  
};
byte STA_damage[20][2] = {
  {0, 4},
  {1, 4},
  {2, 4},
  {0, 3},
  {0, 5},
  {4, 2},
  {7, 2},
  {7, 3},
  {5, 6},
  {4, 3},
  {5, 2},
  {6, 2},
  {7, 5},
  {7, 6},
  {6, 6},
  {6, 3},
  {6, 3},
  {4, 6},
  {3, 5},
  {3, 6}  
};
byte STA_weapons[6][2] = {
  {3, 4},
  {4, 4},
  {5, 4},
  {1, 3},
  {2, 3},
  {3, 3}
};
byte STA_sensors[4][2] = {
  {6, 4},
  {1, 5},
  {2, 5},
  {7, 4}
};
byte STA_engines[6][2] = {
  {6, 5},
  {5, 5},
  {4, 5},
  {0, 6},
  {1, 6},
  {2, 6}
};
// torpedo tube loading lights array
byte tubes_lights[12][2] = {
  {0, 1},
  {1, 1},
  {2, 1},
  {3, 1},
  {4, 1},
  {5, 1},
  {0, 2},
  {1, 2},
  {2, 2},
  {3, 2},
  {4, 2},
  {5, 2}
};
unsigned long animate_shields_timer = 0;   // variable to store the time between shield animation frames
#define ANIMATE_SHIELDS_INTERVAL 150  // milliseconds between shield animation frames
byte animate_shields_frame = 0;
unsigned long animate_weapons_timer = 0;   // variable to store the time between shield animation frames
#define ANIMATE_WEAPONS_INTERVAL 500  // milliseconds between shield animation frames
byte animate_weapons_frame = 0;
byte animate_engines_frame = 0;
unsigned long animate_tubes_timer = 0;   // variable to store the time between tube animation frames
#define ANIMATE_TUBES_INTERVAL 500   // milliseconds between tube loading animation frames
byte animate_tubes_frame = 0;

// define shift register clock (serial clock), latch (register clock), and output enable pins
#define SHIFT_CLOCK_PIN 2
#define SHIFT_LATCH_PIN 3
#define OUTPUT_ENABLE_PIN 4

// times to cycle the shift registers and sample the buttons
unsigned long shift_register_timer = 0;   // variable to store the time between shift register cycles
#define SHIFT_REGISTER_INTERVAL 2        // milliseconds betwen shift register cycles

long status_time = 0;
#define UPDATE_STATUS 500

unsigned long test_light_delay = 0;
#define TEST_LIGHT_INTERVAL 1000   // milliseconds between test light changes
int active_test_light = 0;

unsigned long refresh_light_delay = 0;
#define REFRESH_LIGHT_INTERVAL 250   // milliseconds between refreshing all lights against the stored data states

// analog controls - general
unsigned long read_analog_next = 0;   // variable to store the time of the next analog reading
#define ANALOG_READ_RATE 50  // delay in milliseconds between warp slider readings
#define ANALOG_DIFFERENCE_THRESHOLD 10  // how different the moving average needs to be from the last change to trigger a command

// WEAPONS constants
#define JOYSTICK_X_MIN 0
#define JOYSTICK_Y_MIN 0
#define JOYSTICK_X_MAX 0
#define JOYSTICK_Y_MAX 0
#define JOYSTICK_DEADZONE 5
#define JOYSTICK_SCALE -30
#define ABS_JOYSTICK_SCALE -2
#define SCREEN_CENTER_X 640
#define SCREEN_CENTER_Y 360

// console-specific (WEAPONS) variables
#define MAX_TRANSMIT_QUEUE 30
int dmx_transmit_queue_pos = 0;
int dmx_transmit_queue_last = 0;
#define MY_CONTROLLER_ADDRESS 20
byte RS_485_TX[MAX_TRANSMIT_QUEUE][10];   // variable to store bytes for transmittal on the RS-485 bus
int RS_485_RX[10];  // variable to store received bytes
bool data_stream_on = false; // variable to store in currently transmitting data state
int data_stream_pos = 0;  // varaible to store where in the block of data we are currently transmitting/receiving
bool transmit_enable = false; // variable to turn ability to transmit on and off (token)
#define RS_485_TX_START 255 // symbol to signify start of transmitting stream
#define RS_485_TX_STOP 254 // symbol to stop transmitting stream

bool transmit_data = false;
bool control_enable = true;
int screen_mode = 2;   // start assuming that screen is view forward
bool CAM_mode = false;
bool joystick_enable = true;
int joystick_x_center = 512;
int joystick_y_center = 512;
unsigned long mouse_timer = 0;
int power_levels[8] = {20, 20, 20, 20, 20, 20, 20, 20};
bool system_status_light_on[8] = {false, false, false, false, false, false, false, false};
bool shields_on = false;
int damage_state = 0;
bool front_shield_low = false;
bool rear_shield_low = false;
int tube_state[4][2] = {   // -1 = n/a, 0 = empty, 1 = loading, 2 = unloading, 3 = ready
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0}
};
int panel_mode = 2;  // number of torpedo tubes
bool load_enable = true;
bool fire_enable = true;
bool abs_pos = false;
int autoload_on = false;
int autofire_on = true;
bool science_control_on = false;
int beam_freq = 1;
int torp_select = 1;



// SETUP CODE ***********************************************************************************************


void setup() {
  Serial.begin(9600);   // USB serial - used to debug

  Mouse.screenSize(1280, 720);   // setup mouse commands for 1280 by 720 resolution screen

  // setup RS-485 bus **************************************************************************************
  Serial1.begin(9600); //Enable serial TX/RX
  Serial1.transmitterEnable(13);  // Use pin 13 to automatically enable a RS-485 transceiver chip. This pin outputs a logic HIGH when transmitting data. It goes LOW after the last stop bit, to allow reception when not transmitting. 

 // setup transmit queue ******************************************************
  for (int i=0; i<MAX_TRANSMIT_QUEUE; i++) {
    for (int k=0; k<10; k++) {
      RS_485_TX[i][k] = 0;
    }
  }

  // setup RX buffer *****************************************************
  for (int i=0; i<10; i++) {
    RS_485_RX[i] = 0;
  }

  // setup button array ****************************
  for (int i=0; i<=7; i++) {
    for (int j=0; j<=2; j++) {
      for (int k=0; k<=3; k++) {
        button_array[i][j][k][0] = 0;    // set all button press counters to zero
        button_array[i][j][k][1] = 0;    // set all button codes to zero
      }
    }
  }
  // button code setup ****************************
  // simple commands first
  button_array[6][2][2][1] = 10;    // zoom -
  button_array[5][2][2][1] = 11;    // zoom +
  button_array[3][2][3][1] = 34;    // select EMP
  button_array[6][2][3][1] = 38;    // select probe
  button_array[2][2][3][1] = 33;    // select mine
  button_array[7][2][3][1] = 37;    // select tag
  button_array[1][2][3][1] = 32;    // select nuke
  button_array[5][2][3][1] = 36;    // select beacon
  button_array[0][2][3][1] = 31;    // select homing
  button_array[4][2][3][1] = 35;    // select pshock
  button_array[3][2][2][1] = 39;    // convert energy to torpedo
  button_array[4][2][2][1] = 40;    // convert torpedo to energy
  button_array[0][2][1][1] = 45;    // load/unload tube 1
  button_array[1][2][1][1] = 47;    // load/unload tube 2
  button_array[2][2][1][1] = 49;    // load/unload tube 3
  button_array[3][2][1][1] = 51;    // load/unload tube 4
  button_array[4][2][1][1] = 41;    // fire tube 1
  button_array[5][2][1][1] = 42;    // fire tube 2
  button_array[6][2][1][1] = 43;    // fire tube 3
  button_array[7][2][1][1] = 44;    // fire tube 4  
  button_array[0][1][3][1] = 53;    // auto beams button
  button_array[2][1][3][1] = 54;    // freq left
  button_array[3][1][3][1] = 55;    // freq right
  button_array[7][2][2][1] = 56;    // click mouse ("select")
  button_array[0][2][2][1] = 57;    // toggle load enable
  button_array[1][2][2][1] = 58;    // toggle fire enable
  button_array[1][1][3][1] = 59;    // toggle mouse pointer absolute position
  
  button_array[1][0][2][1] = 20;    // shields up
  button_array[0][0][2][1] = 21;    // shields down
  button_array[1][0][1][1] = 12;    // view forward
  button_array[3][0][1][1] = 14;    // view right
  button_array[2][0][1][1] = 19;    // view "CAM"
  button_array[0][0][1][1] = 13;    // view left
  button_array[4][0][1][1] = 15;    // view aft
  button_array[7][0][1][1] = 18;    // view INFO
  button_array[6][0][1][1] = 17;    // view LRS
  button_array[5][0][1][1] = 16;    // view TAC

  // complex "high-level" commands second
  button_array[2][2][2][1] = 126;   // cycle tube control mode
  
  // end button code setup

  // setup joystick *****************************
  read_analog_next = millis() + ANALOG_READ_RATE;
  

  // setup light array ****************************
  for (int i=0; i<=7; i++) {
    for (int j=0; j<=7; j++) {
      light_array[i][j][0] = false;    // turn all lights off
      light_array[i][j][1] = false;
      light_array[i][j][2] = false;
    }
  }

  // setup panel output and input pins ***********
  pinMode(12, INPUT);
  joystick_x_center = analogRead(16);
  joystick_y_center = analogRead(17);
  for (int i = 0; i<=2; i++) {
    if (connected_panels[i]) {
      pinMode(button_pins[i][0], OUTPUT);
      digitalWrite(button_pins[i][0], LOW);
      pinMode(button_pins[i][1], INPUT);
      pinMode(button_pins[i][2], INPUT);
      pinMode(button_pins[i][3], INPUT);
    }
  }

  // setup shift register pins ********************
  pinMode(SHIFT_CLOCK_PIN, OUTPUT);
  digitalWrite(SHIFT_CLOCK_PIN, LOW);
  pinMode(SHIFT_LATCH_PIN, OUTPUT);
  digitalWrite(SHIFT_LATCH_PIN, LOW);

  // setup shift register timers *******************
  shift_register_timer = millis();

  // setup dummy test light timer *****************
  test_light_delay = millis() + TEST_LIGHT_INTERVAL;

  // setup refresh light timer *****************
  refresh_light_delay = millis() + REFRESH_LIGHT_INTERVAL;

  // setup command queue
  for (int i = 0; i<MAX_COMMAND_QUEUE; i++) {
    command_queue[i][0] = 0;
    command_queue[i][1] = 0;
    command_queue[i][2] = 0;
  }

  // setup command timer
  command_timer = millis() + COMMAND_INTERVAL;

  // setup animation timers
  animate_shields_timer = millis() + ANIMATE_SHIELDS_INTERVAL;
  animate_weapons_timer = millis() + ANIMATE_WEAPONS_INTERVAL;
  animate_tubes_timer = millis() + ANIMATE_TUBES_INTERVAL;

  // cycle shift register for the first time to clear all data from the registers
  cycle_shift_register();

  // allow output enable pin to go low to enable output from shift registers
  // (there is a pullup resistor on the output enable pin to ensure the startup state
  // of the controllers does not allow lights to come on and over-current the 3V supply)
  pinMode(OUTPUT_ENABLE_PIN, OUTPUT);
  digitalWrite(OUTPUT_ENABLE_PIN, LOW);

      
}   // end SETUP ********************************************************************************************


// CYCLE SHIFT REGISTER CODE ************************************************************************************
void cycle_shift_register() {
  // shift register cycle bottom to top
  // top of the stack are the button registers
  // middle of the stack are the light supply pins
  // bottom of the stack are the light sink pins

  digitalWrite(SHIFT_LATCH_PIN, LOW);

  active_light_sink++;
  if (active_light_sink > MAX_LIGHT_SINK) {
    active_light_sink = 0;
  }
  active_button_pin++;
  if (active_button_pin > MAX_BUTTON_PIN) {
    active_button_pin = 0;
  }
  
  // read out the light sink pins first (in reverse order)
  for (int i=7; i>=0; i--) {
    digitalWrite(SHIFT_CLOCK_PIN, LOW);
    if (i == active_light_sink) {
      for (int k=0; k<= 2; k++) {
        if (connected_panels[k]) {
          digitalWrite(button_pins[k][0], HIGH);
        }
      }
    }
    else {
      for (int k=0; k<= 2; k++) {
        if (connected_panels[k]) {
          digitalWrite(button_pins[k][0], LOW);
        }
      }
    }
    digitalWrite(SHIFT_CLOCK_PIN, HIGH);
  }

  // read out the light supply pins second (in reverse order)
  for (int i=7; i>=0; i--) {
    digitalWrite(SHIFT_CLOCK_PIN, LOW);
    for (int k=0; k<=2; k++) {
      if (light_array[i][active_light_sink][k]) {
        if (connected_panels[k]) {
          digitalWrite(button_pins[k][0], HIGH);
        }
      }
      else {
        if (connected_panels[k]) {
          digitalWrite(button_pins[k][0], LOW);
        }
      }
    }
    digitalWrite(SHIFT_CLOCK_PIN, HIGH);
  }

  // read out the button pins last (in reverse order)
  for (int i=7; i>=0; i--) {
    digitalWrite(SHIFT_CLOCK_PIN, LOW);
    if (i == active_button_pin) {
      for (int k=0; k<= 2; k++) {
        if (connected_panels[k]) {
          digitalWrite(button_pins[k][0], HIGH);
        }
      }
    }
    else {
      for (int k=0; k<= 2; k++) {
        if (connected_panels[k]) {
          digitalWrite(button_pins[k][0], LOW);
        }
      }
    }
    digitalWrite(SHIFT_CLOCK_PIN, HIGH);
  }

  digitalWrite(SHIFT_LATCH_PIN, HIGH);   // turn everything on to the new registers
  
}


// ENTER COMMAND CODE ********************************************************************************************
void enter_command(int enter_code, int x, int y) {
  command_queue[last_command][0] = enter_code;
  command_queue[last_command][1] = x;
  command_queue[last_command][2] = y;
  last_command++;
  if (last_command == MAX_COMMAND_QUEUE) {
     last_command = 0;
  }
}

// EXECUTE COMMAND CODE ******************************************************************************************
void execute_command() {
  if (last_command != current_command) {
    switch (command_queue[current_command][0]) {
      case 0:
      break;

      // simple commands block ************
      case 10:                     // zoom console -
        Keyboard.press(KEY_Y);
        Keyboard.release(KEY_Y);
      break;
      case 11:                      // zoom console +
        Keyboard.press(KEY_T);
        Keyboard.release(KEY_T);
      break;      
      case 20:                     // raise shields
        Keyboard.press(KEY_K);
        Keyboard.release(KEY_K);
      break;
      case 21:                     // lower shields
        Keyboard.press(KEY_L);
        Keyboard.release(KEY_L);
      break;
      case 12:                    // view front
        Keyboard.press(KEY_F2); 
        Keyboard.release(KEY_F2);
        screen_mode = 2;
        add_data_to_transmit_queue(134, screen_mode, 0);
      break;
      case 13:                    // view left
        Keyboard.press(KEY_F3);
        Keyboard.release(KEY_F3);
        screen_mode = 3;
        add_data_to_transmit_queue(134, screen_mode, 0);
      break;
      case 14:                    // view right
        Keyboard.press(KEY_F4);
        Keyboard.release(KEY_F4);
        screen_mode = 4;
        add_data_to_transmit_queue(134, screen_mode, 0);
      break;
      case 15:                    // view aft
        Keyboard.press(KEY_F5);
        Keyboard.release(KEY_F5);
        screen_mode = 5;
        add_data_to_transmit_queue(134, screen_mode, 0);
      break;
      case 16:                    // view tac
        Keyboard.press(KEY_F6);
        Keyboard.release(KEY_F6);
        screen_mode = 6;
        add_data_to_transmit_queue(134, screen_mode, 0);
      break;
      case 17:                    // view lrs
        Keyboard.press(KEY_F7);
        Keyboard.release(KEY_F7);
        screen_mode = 7;
        add_data_to_transmit_queue(134, screen_mode, 0);
      break;
      case 18:                    // view info
        Keyboard.press(KEY_F8);
        Keyboard.release(KEY_F8);
        screen_mode = 8;
        add_data_to_transmit_queue(134, screen_mode, 0);
      break;
      case 19:
        // click on CAM button
        Mouse.moveTo(1180, 130);
        Mouse.click();
        if (CAM_mode) {
          CAM_mode = false;
        }
        else {
          CAM_mode = true;
        }
      break;
      
      case 56:
        // click mouse button
        Mouse.click();
      break;
      
      case 46:
        // unload 1
        if (panel_mode > 0 && load_enable) {
          Mouse.moveTo(45, 695);
          Mouse.click();
          if (tube_state[0][0] == 3) {
            tube_state[0][0] = 2;
          }          
        }
      break;
      case 48:
        // unload 2
        if (panel_mode > 1 && load_enable) {
          Mouse.moveTo(45, 645);
          Mouse.click();
          if (tube_state[1][0] == 3) {
            tube_state[1][0] = 2;
          }          
        }
      break;
      case 50:
        // unload 3
        if (panel_mode > 2 && load_enable) {
          Mouse.moveTo(45, 595);
          Mouse.click();
          if (tube_state[2][0] == 3) {
            tube_state[2][0] = 2;
          }          
        }
      break;
      case 52:
        // unload 4
        if (panel_mode > 3 && load_enable) {
          Mouse.moveTo(45, 545);
          Mouse.click();
          if (tube_state[3][0] == 3) {
            tube_state[3][0] = 2;
          }          
        }
      break;
      case 45:
        // load 1
        if (panel_mode > 0 && load_enable) {
          Mouse.moveTo(45, 695);
          Mouse.click();
          if (tube_state[0][0] == 0) {
            tube_state[0][0] = 1;
            tube_state[0][1] = torp_select;
            system_status_light_on[1] = true;
            
          }
          if (tube_state[0][0] == 3) {
            tube_state[0][0] = 2;
            system_status_light_on[1] = true;
          }           
        }
      break;
      case 47:
        // load 2
        if (panel_mode > 1 && load_enable) {
          Mouse.moveTo(45, 645);
          Mouse.click();
          if (tube_state[1][0] == 0) {
            tube_state[1][0] = 1;
            tube_state[1][1] = torp_select;
            system_status_light_on[1] = true;
          } 
          if (tube_state[1][0] == 3) {
            tube_state[1][0] = 2;
            system_status_light_on[1] = true;
          }          
        }
      break;
      case 49:
        // load 3
        if (panel_mode > 2 && load_enable) {
          Mouse.moveTo(45, 595);
          Mouse.click();
          if (tube_state[2][0] == 0) {
            tube_state[2][0] = 1;
            tube_state[2][1] = torp_select;
            system_status_light_on[1] = true;
          }
          if (tube_state[2][0] == 3) {
            tube_state[2][0] = 2;
            system_status_light_on[1] = true;
          }           
        }
      break;
      case 51:
        // load 4
        if (panel_mode > 3 && load_enable) {
          Mouse.moveTo(45, 545);
          Mouse.click();
          if (tube_state[3][0] == 0) {
            tube_state[3][0] = 1;
            tube_state[3][1] = torp_select;
            system_status_light_on[1] = true;
          }
          if (tube_state[3][0] == 3) {
            tube_state[3][0] = 2;
            system_status_light_on[1] = true;
          }           
        }
      break;
      case 41:
        // fire 1
        if (panel_mode > 0 && fire_enable) {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_1);
          Keyboard.release(KEY_1);
          Keyboard.set_modifier(0);
          if (tube_state[0][0] == 0) {
            tube_state[0][0] = 1;
            tube_state[0][1] = torp_select;
            system_status_light_on[1] = true;
          }
          if (tube_state[0][0] == 3) {
            tube_state[0][0] = 0;
            tube_state[0][1] = 0;
          }          
        }
      break;
      case 42:
        // fire 2
        if (panel_mode > 1 && fire_enable) {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_2);
          Keyboard.release(KEY_2);
          Keyboard.set_modifier(0);
          if (tube_state[1][0] == 0) {
            tube_state[1][0] = 1;
            tube_state[1][1] = torp_select;
            system_status_light_on[1] = true;
          }  
          if (tube_state[1][0] == 3) {
            tube_state[1][0] = 0;
            tube_state[1][1] = 0;
          }        
        }
      break;
      case 43:
        // fire 3
        if (panel_mode > 2 && fire_enable) {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_3);
          Keyboard.release(KEY_3);
          Keyboard.set_modifier(0);
          if (tube_state[2][0] == 0) {
            tube_state[2][0] = 1;
            tube_state[2][1] = torp_select;
            system_status_light_on[1] = true;
          }
          if (tube_state[2][0] == 3) {
            tube_state[2][0] = 0;
            tube_state[2][1] = 0;
          }           
        }
      break;
      case 44:
        // fire 4
        if (panel_mode > 3 && fire_enable) {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_4);
          Keyboard.release(KEY_4);
          Keyboard.set_modifier(0);
          if (tube_state[3][0] == 0) {
            tube_state[3][0] = 1;
            tube_state[3][1] = torp_select;
            system_status_light_on[1] = true;
          }  
          if (tube_state[3][0] == 3) {
            tube_state[3][0] = 0;
            tube_state[3][1] = 0;
          }         
        }
      break;
      case 53:
        // toggle autobeams
        Keyboard.press(KEY_B);
        Keyboard.release(KEY_B);
        if (autofire_on) {
          autofire_on = false;
        }
        else {
          autofire_on = true;
        }
      break;
      case 54:
        // frequency left
        Keyboard.press(KEY_LEFT);
        Keyboard.release(KEY_LEFT);
        beam_freq--;
        if (beam_freq < 1) {
          beam_freq = 1;
        }
      break;
      case 55:
        // frequency right
        Keyboard.press(KEY_RIGHT);
        Keyboard.release(KEY_RIGHT);
        beam_freq++;
        if (beam_freq > 5) {
          beam_freq = 5;
        }
      break;
      case 57:
        // load enable
        if (load_enable) {
          load_enable = false;
        }
        else {
          load_enable = true;
        }
      break;
      case 58:
        // fire enable
        if (fire_enable) {
          fire_enable = false;
        }
        else {
          fire_enable = true;
        }
      break;
      case 59:
        // toggle mouse pointer absolute position
        if (abs_pos) {
          abs_pos = false;
        }
        else {
          abs_pos = true;
        }
      break;

      case 31:
        // select homing
        Keyboard.press(KEY_1);
        Keyboard.release(KEY_1);
        torp_select = 1;
      break;
      case 32:
        // select nuke
        Keyboard.press(KEY_2);
        Keyboard.release(KEY_2);
        torp_select = 2;
      break;
      case 33:
        // select mine
        Keyboard.press(KEY_3);
        Keyboard.release(KEY_3);
        torp_select = 3;
      break;
      case 34:
        // select EMP
        Keyboard.press(KEY_4);
        Keyboard.release(KEY_4);
        torp_select = 4;
      break;
      case 35:
        // select PSHOCK
        Keyboard.press(KEY_5);
        Keyboard.release(KEY_5);
        torp_select = 5;
      break;
      case 36:
        // select beacon
        Keyboard.press(KEY_6);
        Keyboard.release(KEY_6);
        torp_select = 6;
      break;
      case 37:
        // select tag
        Keyboard.press(KEY_8);
        Keyboard.release(KEY_8);
        torp_select = 8;
      break;
      case 38:
        // select probe
        Keyboard.press(KEY_7);
        Keyboard.release(KEY_7);
        torp_select = 7;
      break;
      case 39:
        // convert energy to torpedo
        Keyboard.set_modifier(MODIFIERKEY_SHIFT);
        Keyboard.press(KEY_I);
        Keyboard.release(KEY_I);
        Keyboard.set_modifier(0);
      break;
      case 40:
        // convert torpedo to energy
        Keyboard.set_modifier(MODIFIERKEY_SHIFT);
        Keyboard.press(KEY_U);
        Keyboard.release(KEY_U);
        Keyboard.set_modifier(0);
      break;
      

      // complex command block ************
      case 126:
        // adjust torpedo tube mode
        panel_mode++;
        if (panel_mode > 4) {
          panel_mode = 0;
        }
      break;
      
      // mouse control bloc *******************************
      case 210:   // move mouse pointer
        Mouse.moveTo(command_queue[current_command][1],command_queue[current_command][2]);
      break;
      case 211:   // push mouse button
        Mouse.set_buttons(1, 0, 0);
      break;
      case 212:   // release mouse button
        Mouse.set_buttons(0, 0, 0);
      break;
    }

    command_queue[current_command][0] = 0;  //clear command from queue
    current_command++;  // increment current command to read next one in queue
    if (current_command == MAX_COMMAND_QUEUE) {
       current_command = 0;
    }
  }
}

// READ BUTTON CODE **********************************************************************************************
void read_buttons() {
  for (int j=0; j<=2; j++) {
    if (connected_panels[j]) {
      for (int k=1; k<=3; k++) {
        if (button_array[active_button_pin][j][k][1] != 0) {
          if (digitalRead(button_pins[j][k]) == HIGH) {
            button_array[active_button_pin][j][k][0]++;
            if (button_array[active_button_pin][j][k][0] == BUTTON_PUSH_THRESHOLD) {
              enter_command(button_array[active_button_pin][j][k][1], 0, 0);
            }
          }
          else {
            button_array[active_button_pin][j][k][0] = 0;
          }
        }
      }
    }
  }
}


// READ ANALOG CONTROLS CODE ************************************************************************************
void read_analog() {
  if (joystick_enable) {
    int readX = analogRead(16) - joystick_x_center;
    int readY = analogRead(17) - joystick_y_center;
    int read_distance = sqrt(readX * readX + readY * readY);

    Serial.print(readX);
    Serial.print(", ");
    Serial.println(readY);
    
    if (abs_pos) {
        if (read_distance > JOYSTICK_DEADZONE) {
          //readX = SCREEN_CENTER_X + (readX - JOYSTICK_DEADZONE * readX / read_distance) / ABS_JOYSTICK_SCALE;
          //readY = SCREEN_CENTER_Y + (readY - JOYSTICK_DEADZONE * readY / read_distance) / ABS_JOYSTICK_SCALE;
          readX = SCREEN_CENTER_X + readX / ABS_JOYSTICK_SCALE;
          readY = SCREEN_CENTER_Y + readY / ABS_JOYSTICK_SCALE;
        Mouse.moveTo(readX,readY);
        }
        else {
          Mouse.moveTo(SCREEN_CENTER_X,SCREEN_CENTER_Y);
        }
    }
    else {
      if (read_distance > JOYSTICK_DEADZONE) {
        //readX = (readX - JOYSTICK_DEADZONE * readX / read_distance) / JOYSTICK_SCALE;
        //readY = (readY - JOYSTICK_DEADZONE * readY / read_distance) / JOYSTICK_SCALE;
        readX = readX / JOYSTICK_SCALE;
        readY = readY / JOYSTICK_SCALE;
        Mouse.move(readX,readY);
      }
    }
  }
}


// ANIMATE SHIELDS CODE ***********************************************************************************
void animate_shields() {
  if (shields_on == false) {
    animate_shields_frame = 0;
    for (int i=0; i<20; i++) {
      light_array[STA_shields[i][0]][STA_shields[i][1]][1] = false;
    }
  }
  else {
    light_array[STA_shields[animate_shields_frame][0]][STA_shields[animate_shields_frame][1]][1] = true;
    animate_shields_frame++;
    if (animate_shields_frame >=19) {
      animate_shields_frame = 19;
    }
  }
  
}

// ANIMATE DAMAGE CODE ***********************************************************************************
void animate_damage() {
  for (int i=0; i<20; i++) {
      light_array[STA_damage[i][0]][STA_damage[i][1]][1] = random(0,40) < long(damage_state * 10);
    }  
}

// ANIMATE STATUS PANEL TORPEDOS CODE ******************************************************************
void animate_weapons() {
  if (system_status_light_on[1]) {
    for (int i=0; i<6; i++) {
      light_array[STA_weapons[i][0]][STA_weapons[i][1]][1] = true;
    }
    light_array[STA_weapons[animate_weapons_frame][0]][STA_weapons[animate_weapons_frame][1]][1] = false;
    light_array[STA_weapons[animate_weapons_frame + 3][0]][STA_weapons[animate_weapons_frame + 3][1]][1] = false;
    animate_weapons_frame++;
    if (animate_weapons_frame >= 3) {
      animate_weapons_frame = 0;
    }
  }
  else {
     for (int i=0; i<6; i++) {
      light_array[STA_weapons[i][0]][STA_weapons[i][1]][1] = false;
    }
  }
}

// ANIMATE STATUS PANEL ENGINES CODE ******************************************************************
void animate_engines() {
  if (system_status_light_on[5]) {
    for (int i=0; i<6; i++) {
      light_array[STA_engines[i][0]][STA_engines[i][1]][1] = true;
    }
    light_array[STA_engines[animate_engines_frame][0]][STA_engines[animate_engines_frame][1]][1] = false;
    light_array[STA_engines[animate_engines_frame + 3][0]][STA_engines[animate_engines_frame + 3][1]][1] = false;
    animate_engines_frame++;
    if (animate_engines_frame >= 3) {
      animate_engines_frame = 0;
    }
  }
  else {
     for (int i=0; i<6; i++) {
      light_array[STA_engines[i][0]][STA_engines[i][1]][1] = false;
    }
  }
}

// ANIMATE TUBES CODE ***********************************************************************************
void animate_tubes() {
  for (int i=0; i<12; i++) {
      light_array[tubes_lights[i][0]][tubes_lights[i][1]][2] = false;
    }
  animate_tubes_frame++;
  if (animate_tubes_frame > 2) {
    animate_tubes_frame = 0;
  }
  if (panel_mode > 0) {
    switch (tube_state[0][0]) {
      case 0:                           // tube empty  
      break;
      case 1:                           // tube loading  
        light_array[tubes_lights[0 + animate_tubes_frame][0]][tubes_lights[0 + animate_tubes_frame][1]][2] = true;
      break;
      case 2:                           // tube unloading
        light_array[tubes_lights[2 - animate_tubes_frame][0]][tubes_lights[2 - animate_tubes_frame][1]][2] = true;
      break;
      case 3:                           // tube ready 
        light_array[tubes_lights[0][0]][tubes_lights[0][1]][2] = true;
        light_array[tubes_lights[1][0]][tubes_lights[1][1]][2] = true;
        light_array[tubes_lights[2][0]][tubes_lights[2][1]][2] = true;
      break;
    }
  }
  if (panel_mode > 1) {
    light_array[6][3][2] = true;
    switch (tube_state[1][0]) {
      case 0:                           // tube empty   
      break;
      case 1:                           // tube loading  
        light_array[tubes_lights[3 + animate_tubes_frame][0]][tubes_lights[3 + animate_tubes_frame][1]][2] = true;
      break;
      case 2:                           // tube unloading
        light_array[tubes_lights[5 - animate_tubes_frame][0]][tubes_lights[5 - animate_tubes_frame][1]][2] = true;
      break;
      case 3:                           // tube ready
        light_array[tubes_lights[3][0]][tubes_lights[3][1]][2] = true;
        light_array[tubes_lights[4][0]][tubes_lights[4][1]][2] = true;
        light_array[tubes_lights[5][0]][tubes_lights[5][1]][2] = true;
      break;
    }
  }
  if (panel_mode > 2) {
    light_array[6][1][2] = true;
    switch (tube_state[2][0]) {
      case 0:                           // tube empty  
      break;
      case 1:                           // tube loading  
        light_array[tubes_lights[6 + animate_tubes_frame][0]][tubes_lights[6 + animate_tubes_frame][1]][2] = true;
      break;
      case 2:                           // tube unloading
        light_array[tubes_lights[8 - animate_tubes_frame][0]][tubes_lights[8 - animate_tubes_frame][1]][2] = true;
      break;
      case 3:                           // tube ready 
        light_array[tubes_lights[6][0]][tubes_lights[6][1]][2] = true;
        light_array[tubes_lights[7][0]][tubes_lights[7][1]][2] = true;
        light_array[tubes_lights[8][0]][tubes_lights[8][1]][2] = true;
      break;
    }
  }
  if (panel_mode > 3) {
    light_array[7][3][2] = true;
    switch (tube_state[3][0]) {
      case 0:                           // tube empty 
      break;
      case 1:                           // tube loading  
        light_array[tubes_lights[9 + animate_tubes_frame][0]][tubes_lights[9 + animate_tubes_frame][1]][2] = true;
      break;
      case 2:                           // tube unloading
        light_array[tubes_lights[11 - animate_tubes_frame][0]][tubes_lights[11 - animate_tubes_frame][1]][2] = true;
      break;
      case 3:                           // tube ready  
        light_array[tubes_lights[9][0]][tubes_lights[9][1]][2] = true;
        light_array[tubes_lights[10][0]][tubes_lights[10][1]][2] = true;
        light_array[tubes_lights[11][0]][tubes_lights[11][1]][2] = true;
      break;
    }
  }
}


// REFRESH LIGHTS CODE ************************************************************************************
void refresh_lights() {

  // torpedo tube status lights
  light_array[7][1][2] = false;
  light_array[6][3][2] = false;
  light_array[6][1][2] = false;
  light_array[7][3][2] = false;
  for (int i=0; i<8; i++) {
    light_array[i][0][2] = false;
  }
  if (panel_mode > 0) {
    light_array[7][1][2] = true;
    switch (tube_state[0][0]) {
      case 0:                           // tube empty
        light_array[0][0][2] = true;   
      break;
      case 1:                           // tube loading  
      break;
      case 2:                           // tube unloading
      break;
      case 3:                           // tube ready
        light_array[4][0][2] = true;   
      break;
    }
  }
  if (panel_mode > 1) {
    light_array[6][3][2] = true;
    switch (tube_state[1][0]) {
      case 0:                           // tube empty
        light_array[1][0][2] = true;   
      break;
      case 1:                           // tube loading  
      break;
      case 2:                           // tube unloading
      break;
      case 3:                           // tube ready
        light_array[5][0][2] = true;   
      break;
    }
  }
  if (panel_mode > 2) {
    light_array[6][1][2] = true;
    switch (tube_state[2][0]) {
      case 0:                           // tube empty
        light_array[2][0][2] = true;   
      break;
      case 1:                           // tube loading  
      break;
      case 2:                           // tube unloading
      break;
      case 3:                           // tube ready
        light_array[6][0][2] = true;   
      break;
    }
  }
  if (panel_mode > 3) {
    light_array[7][3][2] = true;
    switch (tube_state[3][0]) {
      case 0:                           // tube empty
        light_array[3][0][2] = true;   
      break;
      case 1:                           // tube loading  
      break;
      case 2:                           // tube unloading
      break;
      case 3:                           // tube ready
        light_array[7][0][2] = true;   
      break;
    }
  }

  if (autofire_on) {
    light_array[6][4][2] = true;
  }
  else {
    light_array[6][4][2] = false;
  }
  if (abs_pos) {
    light_array[7][4][2] = true;
  }
  else {
    light_array[7][4][2] = false;
  }
  if (load_enable) {
    light_array[6][2][2] = true;
  }
  else {
    light_array[6][2][2] = false;
  }
  if (fire_enable) {
    light_array[7][2][2] = true;
  }
  else {
    light_array[7][2][2] = false;
  }

  for (int i=0; i<5; i++) {
    light_array[i][6][2] = false;
  }
  switch (beam_freq) {
    case 1:
      light_array[0][6][2] = true;
    break;
    case 2:
      light_array[1][6][2] = true;
    break;
    case 3:
      light_array[2][6][2] = true;
    break;
    case 4:
      light_array[3][6][2] = true;
    break;
    case 5:
      light_array[4][6][2] = true;
    break;
  }

  for (int i=0; i<8; i++) {
    light_array[i][5][2] = false;
  }
  switch (torp_select) {
    case 1:                         // torp
      light_array[0][5][2] = true;
    break;
    case 2:                         // nuke
      light_array[1][5][2] = true;
    break;
    case 3:                         // mine
      light_array[2][5][2] = true;
    break;
    case 4:                         // emp
      light_array[3][5][2] = true;
    break;
    case 5:                         // pshock
      light_array[4][5][2] = true;
    break;
    case 6:                         // beacon
      light_array[5][5][2] = true;
    break;
    case 7:                         // probe
      light_array[6][5][2] = true;
    break;
    case 8:                         // tag
      light_array[7][5][2] = true;
    break;
  }

  for (int i=0; i<6; i++) {
    light_array[i][4][2] = false;
    light_array[i][3][2] = false;
    light_array[i][4][0] = false;
    light_array[i][3][0] = false;
  }
  // beam power level
  if (power_levels[0] > 5) {
    light_array[0][4][2] = true;
  }
  if (power_levels[0] > 15) {
    light_array[1][4][2] = true;
  }
  if (power_levels[0] > 25) {
    light_array[2][4][2] = true;
  }
  if (power_levels[0] > 35) {
    light_array[3][4][2] = true;
  }
  if (power_levels[0] > 45) {
    light_array[4][4][2] = true;
  }
  if (power_levels[0] > 55) {
    light_array[5][4][2] = true;
  }
  
  // torpedo power level
  if (power_levels[1] > 5) {
    light_array[0][3][2] = true;
  }
  if (power_levels[1] > 15) {
    light_array[1][3][2] = true;
  }
  if (power_levels[1] > 25) {
    light_array[2][3][2] = true;
  }
  if (power_levels[1] > 35) {
    light_array[3][3][2] = true;
  }
  if (power_levels[1] > 45) {
    light_array[4][3][2] = true;
  }
  if (power_levels[1] > 55) {
    light_array[5][3][2] = true;
  }

  // front shield power level
  if (power_levels[6] > 5) {
    light_array[0][4][0] = true;
  }
  if (power_levels[6] > 15) {
    light_array[1][4][0] = true;
  }
  if (power_levels[6] > 25) {
    light_array[2][4][0] = true;
  }
  if (power_levels[6] > 35) {
    light_array[3][4][0] = true;
  }
  if (power_levels[6] > 45) {
    light_array[4][4][0] = true;
  }
  if (power_levels[6] > 55) {
    light_array[5][4][0] = true;
  }

  // rear shield power level
  if (power_levels[7] > 5) {
    light_array[0][3][0] = true;
  }
  if (power_levels[7] > 15) {
    light_array[1][3][0] = true;
  }
  if (power_levels[7] > 25) {
    light_array[2][3][0] = true;
  }
  if (power_levels[7] > 35) {
    light_array[3][3][0] = true;
  }
  if (power_levels[7] > 45) {
    light_array[4][3][0] = true;
  }
  if (power_levels[7] > 55) {
    light_array[5][3][0] = true;
  }


  // shields status
  if (shields_on) {
    light_array[0][1][0] = false;
    light_array[1][1][0] = true;
    light_array[5][2][0] = true;
    light_array[0][2][0] = true;
  }
  else {
    light_array[0][1][0] = true;
    light_array[1][1][0] = false;
    light_array[5][2][0] = false;
    light_array[0][2][0] = false;
  }

  // shields low
  if (front_shield_low) {
    light_array[5][2][0] = false;
    light_array[4][2][0] = true;
  }
  else {
    light_array[4][2][0] = false;
  }
  if (rear_shield_low) {
    light_array[0][2][0] = false;
    light_array[1][2][0] = true;
  }
  else {
    light_array[1][2][0] = false;
  }

  // viewscreen status
  for (int i=0; i<8; i++) {
    light_array[i][0][0] = false;
  }
  switch(screen_mode) {
    case 2:
      light_array[1][0][0] = true;
    break;
    case 3:
      light_array[0][0][0] = true;
    break;
    case 4:
      light_array[3][0][0] = true;
    break;
    case 5:
      light_array[4][0][0] = true;
    break;
    case 6:
      light_array[5][0][0] = true;
    break;
    case 7:
      light_array[6][0][0] = true;
    break;
    case 8:
      light_array[7][0][0] = true;
    break;
  }
  if (CAM_mode) {
      light_array[2][0][0] = true;
  }
  
}


void add_data_to_transmit_queue (int code, int data0, int data1) {
  RS_485_TX[dmx_transmit_queue_last][0] = RS_485_TX_START; // number signifying the start of a new data stream
  RS_485_TX[dmx_transmit_queue_last][1] = byte(code);
  switch (code) {
    case 212:
      RS_485_TX[dmx_transmit_queue_last][2] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][3] = RS_485_TX_STOP;
    break;
    case 215:
      RS_485_TX[dmx_transmit_queue_last][2] = RS_485_TX_STOP;
    break;
    case 101:
      RS_485_TX[dmx_transmit_queue_last][2] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data1);
      RS_485_TX[dmx_transmit_queue_last][4] = RS_485_TX_STOP;
    break;
    case 102:
      RS_485_TX[dmx_transmit_queue_last][2] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data1);
      RS_485_TX[dmx_transmit_queue_last][4] = RS_485_TX_STOP;
    break;
    case 103:
      RS_485_TX[dmx_transmit_queue_last][2] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data1);
      RS_485_TX[dmx_transmit_queue_last][4] = RS_485_TX_STOP;
    break;
    case 133:
      RS_485_TX[dmx_transmit_queue_last][2] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data1);
      RS_485_TX[dmx_transmit_queue_last][4] = RS_485_TX_STOP;
    break;
    case 134:
      RS_485_TX[dmx_transmit_queue_last][2] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data1);
      RS_485_TX[dmx_transmit_queue_last][4] = RS_485_TX_STOP;
    break;
    case 135:
      RS_485_TX[dmx_transmit_queue_last][2] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data1);
      RS_485_TX[dmx_transmit_queue_last][4] = RS_485_TX_STOP;
    break;
  }
  dmx_transmit_queue_last++;
  if (dmx_transmit_queue_last == MAX_TRANSMIT_QUEUE) {
    dmx_transmit_queue_last = 0;
  }
}    // end add data to transmit queue *****************************************************


void transmit_receive() {   //  RS-485 tranceiver code ****************************************
  
  // code for sending data ****** 
  if (transmit_enable) {
    if (data_stream_on) {
      if (Serial1.availableForWrite()) {
        if (RS_485_TX[dmx_transmit_queue_pos][data_stream_pos] != RS_485_TX_STOP) {            
          Serial1.write(RS_485_TX[dmx_transmit_queue_pos][data_stream_pos]);
          if (RS_485_TX[dmx_transmit_queue_pos][data_stream_pos] == RS_485_TX_START) {
          }
        }
        data_stream_pos++;
        if (data_stream_pos == 10) {
          data_stream_pos = 0;
          data_stream_on = false;
          dmx_transmit_queue_pos++;
          if (dmx_transmit_queue_pos >= MAX_TRANSMIT_QUEUE) {
            dmx_transmit_queue_pos = 0;
          }
        }
        if (RS_485_TX[dmx_transmit_queue_pos][data_stream_pos] == RS_485_TX_STOP) {
          if (RS_485_TX[dmx_transmit_queue_pos][1] == 215) {
            transmit_enable = false;
          }
          data_stream_pos = 0;
          data_stream_on = false;
          dmx_transmit_queue_pos++;
          if (dmx_transmit_queue_pos >= MAX_TRANSMIT_QUEUE) {
            dmx_transmit_queue_pos = 0;
          }
        }
      }
    }
    else {
      if (dmx_transmit_queue_pos != dmx_transmit_queue_last) {
        data_stream_on = true;
        data_stream_pos = 0;
      }
    }
         
  }
  else {
    if (Serial1.available()) {
      int rx = Serial1.read();
      if (rx == RS_485_TX_START) {
        data_stream_pos = 0;
      }
      RS_485_RX[data_stream_pos] = rx;

      // sort data into appropriate places

      if (data_stream_pos > 1) {
        if (RS_485_RX[1] == 220) {
          if (RS_485_RX[2] == MY_CONTROLLER_ADDRESS) {
            transmit_enable = true;
            add_data_to_transmit_queue(212, MY_CONTROLLER_ADDRESS, 0);
            if (system_status_light_on[1]) {
              add_data_to_transmit_queue(133, 1, 1);  // torpedos loading
            }
            else {
              add_data_to_transmit_queue(133, 1, 0);            
            }
            add_data_to_transmit_queue(215, 0, 0);
          }
        }
      }

      if (data_stream_pos > 2) {
        if (RS_485_RX[1] == 101) {
          switch (RS_485_RX[2]) {
            case 7:                           // receive shields on cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                shields_on = true;
              }
              else {
                shields_on = false;
              }
            break;
            case 8:                           // receive front shield low cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                front_shield_low = true;
                system_status_light_on[6] = true;
              }
              else {
                front_shield_low = false;
                system_status_light_on[6] = false;
              }
            break;
            case 9:                           // receive rear shield low cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                rear_shield_low = true;
                system_status_light_on[7] = true;
              }
              else {
                rear_shield_low = false;
                system_status_light_on[7] = false;
              }
            break;
            case 32:                          // ship damage 20
              if (RS_485_RX[3] == 1) {
                damage_state = 1;
              }
              else {
                damage_state = 0;
              }
            break;
            case 33:                          // ship damage 40
              if (RS_485_RX[3] == 1) {
                damage_state = 2;
              }
            break;
            case 34:                          // ship damage 60
              if (RS_485_RX[3] == 1) {
                damage_state = 3;
              }
            break;
            case 58:                          // finished loading tube
              if (RS_485_RX[3] == 1) {
                tube_state[0][0] = 3;
                system_status_light_on[1] = false;
              }              
            break;
            case 59:                          // finished loading tube
              if (RS_485_RX[3] == 1) {
                tube_state[1][0] = 3;
                system_status_light_on[1] = false;
              }   
            break;
            case 60:                          // finished loading tube
              if (RS_485_RX[3] == 1) {
                tube_state[2][0] = 3;
                system_status_light_on[1] = false;
              }   
            break;
            case 61:                          // finished loading tube
              if (RS_485_RX[3] == 1) {
                tube_state[3][0] = 3;
                system_status_light_on[1] = false;
              }   
            break;
            case 62:                          // finished unloading tube
              if (RS_485_RX[3] == 1) {
                tube_state[0][0] = 0;
                system_status_light_on[1] = false;
              }
            break;
            case 63:                          // finished unloading tube
              if (RS_485_RX[3] == 1) {
                tube_state[1][0] = 0;
                system_status_light_on[1] = false;
              }
            break;
            case 64:                          // finished unloading tube
              if (RS_485_RX[3] == 1) {
                tube_state[2][0] = 0;
                system_status_light_on[1] = false;
              }
            break;
            case 65:                          // finished unloading tube
              if (RS_485_RX[3] == 1) {
                tube_state[3][0] = 0;
                system_status_light_on[1] = false;
              }
            break;
          }
        }
        if (RS_485_RX[1] == 133) {            // receive system active states from another controller
              if (RS_485_RX[3] == 1) {
                system_status_light_on[RS_485_RX[2]] = true;
              }
              else {
                system_status_light_on[RS_485_RX[2]] = false;
              }
        }
        if (RS_485_RX[1] == 134) {             // receive viewscreen state from another controller
          screen_mode = RS_485_RX[2];
        }
        if (RS_485_RX[1] == 135) {             // receive viewscreen CAM state from another controller
          switch (RS_485_RX[2]) {
            case 0:
              CAM_mode = false;
            break;
            case 1:
              CAM_mode = true;
            break;
          }
        }
      }

      if (data_stream_pos > 8) {             // receive power levels from another controller
        if (RS_485_RX[1] == 120) {
          for (int i=0; i<8; i++) {
            power_levels[i] = RS_485_RX[i+2];
          }
        }
      }
      
      data_stream_pos++;
      if (data_stream_pos > 10) {
        data_stream_pos = 2;
      }
    }
  }
}       // end transmit/receive code


// MAIN LOOP HERE ************************************************************************************************
void loop() {

  // check for instructions from master controller & transmit if it is this controller's turn

  transmit_receive();

  // cycle registers on control panels
  if (millis() >= shift_register_timer) {
    cycle_shift_register();
    shift_register_timer = millis() + SHIFT_REGISTER_INTERVAL;

  // read inputs from controls
    if (digitalRead(12) == HIGH) {
      read_buttons();
    }
  }

  // read analog controls
  if (millis() >= read_analog_next) {
    read_analog_next = millis() + ANALOG_READ_RATE;
    if (digitalRead(12) == HIGH) {
      read_analog();
    }
  }

  // execute commands from command queue
  if (millis() >= command_timer) {
    execute_command();
    command_timer = millis() + COMMAND_INTERVAL;
  }

  // animate lights
  if (millis() >= animate_shields_timer) {
    animate_shields();
    animate_damage();
    animate_shields_timer = millis() + ANIMATE_SHIELDS_INTERVAL;
  }
  if (millis() >= animate_weapons_timer) {
    animate_weapons();
    animate_engines();
    animate_weapons_timer = millis() + ANIMATE_WEAPONS_INTERVAL;
  }
  if (millis() >= animate_tubes_timer) {
    animate_tubes();
    animate_tubes_timer = millis() + ANIMATE_TUBES_INTERVAL;
  }

  // refresh lights
  if (millis() >= refresh_light_delay) {
    refresh_lights();
    
    refresh_light_delay = millis() + REFRESH_LIGHT_INTERVAL;
  }

  
}

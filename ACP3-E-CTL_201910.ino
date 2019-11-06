// Artemis SBS Controller Code
// ACP3 ENG Controller Prototype

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
  {5, 6, 7, 22},      // panel 0 "right": 5 - shift serial, 6&7 - button inputs
  {24, 25, 26, 23},     // panel 1 "left": 24 - shift serial, 25&26 - button inputs
  {8, 10, 11, 12}    // panel 2 "center": 8 - shift serial, 10&11 - button inputs
};

#define MAX_COMMAND_QUEUE 50
int command_queue[MAX_COMMAND_QUEUE][3];   // array to store commands to be executed by the button-push code
  // index 0: commmand number
  // index 1: x coord
  // index 2: y coord
int current_command = 0;    // variable that points to the current command in the queue
int last_command = 0;    // variable that points to the command after the last command enterred in the queue
unsigned long command_timer = 0;   // variable to store the time between consecutive keyboard commands
#define COMMAND_INTERVAL 20 // milliseconds in between consecutive keyboard commands to the connected PC

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
unsigned long animate_shields_timer = 0;   // variable to store the time between shield animation frames
#define ANIMATE_SHIELDS_INTERVAL 150  // milliseconds between shield animation frames
byte animate_shields_frame = 0;
unsigned long animate_weapons_timer = 0;   // variable to store the time between shield animation frames
#define ANIMATE_WEAPONS_INTERVAL 500  // milliseconds between shield animation frames
byte animate_weapons_frame = 0;
byte animate_engines_frame = 0;
byte animate_sensors_frame = 0;

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

// ENGINEERING constants
#define SLIDER_X 50
#define SLIDER_Y 700
#define SLIDER_SCALE 77
#define SLIDER_OFFSET 157
#define SLIDER_DEFAULT_Y 612
#define SLIDER_1_Y 668
#define SLIDER_2_Y 636
#define SLIDER_3_Y 604
#define SLIDER_4_Y 572
#define SLIDER_5_Y 540
#define SLIDER_6_Y 508
#define COOLANT_X 93
#define COOLANT_UP_Y 505
#define COOLANT_DN_Y 703
#define COOLANT_OFFSET 157

// console-specific (ENGINEERING) variables
#define MAX_TRANSMIT_QUEUE 30
int dmx_transmit_queue_pos = 0;
int dmx_transmit_queue_last = 0;
#define MY_CONTROLLER_ADDRESS 40
byte RS_485_TX[MAX_TRANSMIT_QUEUE][12];   // variable to store bytes for transmittal on the RS-485 bus
int RS_485_RX[10];  // variable to store received bytes
bool data_stream_on = false; // variable to store in currently transmitting data state
int data_stream_pos = 0;  // varaible to store where in the block of data we are currently transmitting/receiving
bool transmit_enable = false; // variable to turn ability to transmit on and off (token)
#define RS_485_TX_START 255 // symbol to signify start of transmitting stream
#define RS_485_TX_STOP 254 // symbol to stop transmitting stream
int errors = 0;

bool transmit_data = false;
bool control_enable = true;
bool shields_on = false;
int damage_state = 0;
int ship_hit = 0;
int power_levels[8] = {20, 20, 20, 20, 20, 20, 20, 20};
int power_matrix[8][11];  // array to store power settings
bool system_status_light_on[8] = {false, false, false, false, false, false, false, false};
int slider_reads = 0;
bool slider_command_on[8] = {false, false, false, false, false, false, false, false};
bool settings_match = true;
bool store_on = true;
int slider_state[2][8] = {
  {512, 512, 512, 512, 512, 512, 512, 512},
  {512, 512, 512, 512, 512, 512, 512, 512}
  };

// SETUP CODE ***********************************************************************************************
void setup() {
  Serial.begin(9600);   // USB serial - used to debug

  Mouse.screenSize(1280, 720);   // setup mouse commands for 1280 by 720 resolution screen
  
  // setup RS-485 bus **************************************************************************************
  Serial1.begin(9600); //Enable serial TX/RX
  // pinMode(13, OUTPUT);
  // digitalWrite(13, HIGH);
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
  button_array[0][2][1][1] = 92;    // coolant 1 up
  button_array[1][2][1][1] = 93;    // coolant 2 up
  button_array[2][2][1][1] = 94;    // coolant 3 up
  button_array[3][2][1][1] = 95;    // coolant 4 up
  button_array[4][2][1][1] = 96;    // coolant 5 up
  button_array[5][2][1][1] = 97;    // coolant 6 up
  button_array[6][2][1][1] = 98;    // coolant 7 up
  button_array[7][2][1][1] = 99;    // coolant 8 up
  button_array[0][2][2][1] = 100;    // coolant 1 dn
  button_array[1][2][2][1] = 101;    // coolant 2 dn
  button_array[2][2][2][1] = 102;    // coolant 3 dn
  button_array[3][2][2][1] = 103;    // coolant 4 dn
  button_array[4][2][2][1] = 104;    // coolant 5 dn
  button_array[5][2][2][1] = 105;    // coolant 6 dn
  button_array[6][2][2][1] = 106;    // coolant 7 dn
  button_array[7][2][2][1] = 107;    // coolant 8 dn

  button_array[2][0][2][1] = 70;    // preset 0
  button_array[0][0][2][1] = 71;    // preset 1
  button_array[3][0][2][1] = 72;    // preset 2
  button_array[5][0][2][1] = 73;    // preset 3
  button_array[1][0][2][1] = 74;    // preset 4  
  button_array[4][0][2][1] = 75;    // preset 5
  button_array[6][0][2][1] = 76;    // preset 6
  button_array[0][0][1][1] = 77;    // preset 7
  button_array[3][0][1][1] = 78;    // preset 8
  button_array[5][0][1][1] = 79;    // preset 9
  button_array[1][0][1][1] = 90;    // reset power
  button_array[6][0][1][1] = 91;    // reset coolant
  
  // complex "high-level" commands second
  button_array[7][0][2][1] = 160;    // str/rcl
  
  // end button code setup

  // setup sliders *****************************
  read_analog_next = millis() + ANALOG_READ_RATE;
  for (int i=0; i<=7; i++) {
    for (int j=0; j<=10; j++) {
      power_matrix[i][j] = SLIDER_DEFAULT_Y;
    }
  }

  // setup light array ****************************
  for (int i=0; i<=7; i++) {
    for (int j=0; j<=7; j++) {
      light_array[i][j][0] = false;    // turn all lights off
      light_array[i][j][1] = false;
      light_array[i][j][2] = false;
    }
  }

  // setup panel output and input pins ***********
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

  // cycle shift register for the first time to clear all data from the registers
  cycle_shift_register();

  // allow output enable pin to go low to enable output from shift registers
  // (there is a pullup resistor on the output enable pin to ensure the startup state
  // of the controllers does not allow lights to come on and over-current the 3V supply)
  pinMode(OUTPUT_ENABLE_PIN, OUTPUT);
  digitalWrite(OUTPUT_ENABLE_PIN, LOW);
    
}   // end setup code *******************************************************************************************

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
      
      // coolant buttons:
      case 92:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 0, COOLANT_UP_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 93:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 1, COOLANT_UP_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 94:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 2, COOLANT_UP_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 95:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 3, COOLANT_UP_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 96:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 4, COOLANT_UP_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 97:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 5, COOLANT_UP_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 98:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 6, COOLANT_UP_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 99:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 7, COOLANT_UP_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 100:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 0, COOLANT_DN_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 101:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 1, COOLANT_DN_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 102:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 2, COOLANT_DN_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 103:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 3, COOLANT_DN_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 104:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 4, COOLANT_DN_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 105:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 5, COOLANT_DN_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;;
      break;
      case 106:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 6, COOLANT_DN_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;
      case 107:
        Mouse.moveTo(COOLANT_X + COOLANT_OFFSET * 7, COOLANT_DN_Y);
        Mouse.click();
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;
      break;

      // preset buttons:
      case 70:
        set_power_setting(0, store_on);
        if (!store_on) {
          Keyboard.press(KEY_0);
          Keyboard.release(KEY_0);
        }
        else {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_0);
          Keyboard.release(KEY_0);
          Keyboard.set_modifier(0);
        }
      break;
      case 71:
        set_power_setting(1, store_on);
        if (!store_on) {
          Keyboard.press(KEY_1);
          Keyboard.release(KEY_1);
        }
        else {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_1);
          Keyboard.release(KEY_1);
          Keyboard.set_modifier(0);
        }
      break;
      case 72:
        set_power_setting(2, store_on);
        if (!store_on) {
          Keyboard.press(KEY_2);
          Keyboard.release(KEY_2);
        }
        else {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_2);
          Keyboard.release(KEY_2);
          Keyboard.set_modifier(0);
        }
      break;
      case 73:
        set_power_setting(3, store_on);
        if (!store_on) {
          Keyboard.press(KEY_3);
          Keyboard.release(KEY_3);
        }
        else {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_3);
          Keyboard.release(KEY_3);
          Keyboard.set_modifier(0);
        }
      break;
      case 74:
        set_power_setting(4, store_on);
        if (!store_on) {
          Keyboard.press(KEY_4);
          Keyboard.release(KEY_4);
        }
        else {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_4);
          Keyboard.release(KEY_4);
          Keyboard.set_modifier(0);
        }
      break;
      case 75:
        set_power_setting(5, store_on);
        if (!store_on) {
          Keyboard.press(KEY_5);
          Keyboard.release(KEY_5);
        }
        else {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_5);
          Keyboard.release(KEY_5);
          Keyboard.set_modifier(0);
        }
      break;
      case 76:
        set_power_setting(6, store_on);
        if (!store_on) {
          Keyboard.press(KEY_6);
          Keyboard.release(KEY_6);
        }
        else {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_6);
          Keyboard.release(KEY_6);
          Keyboard.set_modifier(0);
        }
      break;
      case 77:
        set_power_setting(7, store_on);
        if (!store_on) {
          Keyboard.press(KEY_7);
          Keyboard.release(KEY_7);
        }
        else {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_7);
          Keyboard.release(KEY_7);
          Keyboard.set_modifier(0);
        }
      break;
      case 78:
        set_power_setting(8, store_on);
        if (!store_on) {
          Keyboard.press(KEY_8);
          Keyboard.release(KEY_8);
        }
        else {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_8);
          Keyboard.release(KEY_8);
          Keyboard.set_modifier(0);
        }
      break;
      case 79:
        set_power_setting(9, store_on);
        if (!store_on) {
          Keyboard.press(KEY_9);
          Keyboard.release(KEY_9);
        }
        else {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_9);
          Keyboard.release(KEY_9);
          Keyboard.set_modifier(0);
        }
      break;
      case 90:      // reset power
        set_power_setting(10, store_on);
        Keyboard.press(KEY_SPACE);
        Keyboard.release(KEY_SPACE);        
      break;
      case 91:      // reset coolat
        Keyboard.press(KEY_ENTER);
        Keyboard.release(KEY_ENTER);
        settings_match = false;
        light_array[4][0][0] = true;
        light_array[2][0][0] = false;       
      break;

      // complex command block ************
      case 160:
        if (!store_on) {
          store_on = true;
          light_array[7][1][0] = true; 
        }
        else {
          store_on = false;
          light_array[7][1][0] = false;
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
      case 215:
        slider_command_on[command_queue[current_command][1]] = false;
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

// SET/RECALL POWER PRESET CODE **********************************************************
void set_power_setting (int setting, bool storing) {
  if (setting < 10) {
    settings_match = true;
    light_array[4][0][0] = false;
    light_array[2][0][0] = true;
    if (storing) {
      for (int i=0; i<=7; i++) {
        power_matrix[i][setting] = power_matrix[i][10];
      }
    }
    else {
      for (int i=0; i<=7; i++) {
        power_matrix[i][10] = power_matrix[i][setting];
        for (int j=0; j<=5; j++) {
          light_array[i][j][2] = false;
        }        
        if (power_matrix[i][10] < SLIDER_1_Y) {
          light_array[i][5][2] = true;
        }
        if (power_matrix[i][10] < SLIDER_2_Y) {
          light_array[i][4][2] = true;
        }
        if (power_matrix[i][10] < SLIDER_3_Y) {
          light_array[i][3][2] = true;
        }
        if (power_matrix[i][10] < SLIDER_4_Y) {
          light_array[i][2][2] = true;
        }
        if (power_matrix[i][10] < SLIDER_5_Y) {
          light_array[i][1][2] = true;
        }
        if (power_matrix[i][10] < SLIDER_6_Y) {
          light_array[i][0][2] = true;
        }
      }
    }
  }
  else {
    settings_match = false;
    light_array[4][0][0] = true;
    light_array[2][0][0] = false; 
    for (int i=0; i<=7; i++) {
      power_matrix[i][10] = SLIDER_DEFAULT_Y;
    }
  }

  light_array[0][0][0] = false;
  light_array[3][0][0] = false;
  light_array[5][0][0] = false;
  light_array[0][1][0] = false;
  light_array[1][1][0] = false;
  light_array[2][1][0] = false;
  light_array[3][1][0] = false;
  light_array[4][1][0] = false;
  light_array[5][1][0] = false;
  light_array[6][1][0] = false;

  switch (setting) {
    case 0:
      light_array[2][1][0] = true;
    break;
    case 1:
      light_array[0][1][0] = true;
    break;
    case 2:
      light_array[3][1][0] = true;
    break;
    case 3:
      light_array[5][1][0] = true;
    break;
    case 4:
      light_array[1][1][0] = true;
    break;
    case 5:
      light_array[4][1][0] = true;
    break;
    case 6:
      light_array[6][1][0] = true;
    break;
    case 7:
      light_array[0][0][0] = true;
    break;
    case 8:
      light_array[3][0][0] = true;
    break;
    case 9:
      light_array[5][0][0] = true;
    break;
  }
}


// SET SLIDER CODE ************************************************************************
void set_slider (int pos, int state) {
  int x = 0;
  int y = 0;

  settings_match = false;
  light_array[4][0][0] = true;
  light_array[2][0][0] = false; 

  x = SLIDER_X + pos * SLIDER_OFFSET;
  y = SLIDER_Y - state * 10 / SLIDER_SCALE;

  power_matrix[pos][10] = y;

  if (state < SLIDER_DEFAULT_Y) {
    enter_command(210, x, y - 5);
  }
  else {
    enter_command(210, x, y + 5);
  }
  enter_command(211, 0, 0);
  enter_command(210, x, y);
  enter_command(212, 0, 0);
  enter_command(215, pos, 0);  // release slider for next change

  for (int j=0; j<=5; j++) {
    light_array[pos][j][2] = false;
  }        
  if (power_matrix[pos][10] < SLIDER_1_Y) {
    light_array[pos][5][2] = true;
  }
  if (power_matrix[pos][10] < SLIDER_2_Y) {
    light_array[pos][4][2] = true;
  }
  if (power_matrix[pos][10] < SLIDER_3_Y) {
    light_array[pos][3][2] = true;
  }
  if (power_matrix[pos][10] < SLIDER_4_Y) {
    light_array[pos][2][2] = true;
  }
  if (power_matrix[pos][10] < SLIDER_5_Y) {
    light_array[pos][1][2] = true;
  }
  if (power_matrix[pos][10] < SLIDER_6_Y) {
    light_array[pos][0][2] = true;
  }

}

// READ ANALOG CONTROLS CODE ************************************************************************************
void read_analog() {
  slider_state[0][0] = slider_state[0][0] + analogRead(14);
  slider_state[0][1] = slider_state[0][1] + analogRead(15);
  slider_state[0][2] = slider_state[0][2] + analogRead(16);
  slider_state[0][3] = slider_state[0][3] + analogRead(17);
  slider_state[0][4] = slider_state[0][4] + analogRead(18);
  slider_state[0][5] = slider_state[0][5] + analogRead(19);
  slider_state[0][6] = slider_state[0][6] + analogRead(20);
  slider_state[0][7] = slider_state[0][7] + analogRead(21);
  slider_reads++;
  if (slider_reads == 3) {
    for(int i=0; i<=7; i++) {
      slider_state[0][i] = slider_state[0][i]/3;
      if(abs(slider_state[0][i] - slider_state[1][i]) > ANALOG_DIFFERENCE_THRESHOLD && !slider_command_on[i]) {
        slider_state[1][i] = slider_state[0][i];
        set_slider(i, slider_state[1][i]);
        slider_command_on[i] = true;
      }
    }
    slider_reads = 0;
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
    if (animate_shields_frame <= 19) {
      light_array[STA_shields[animate_shields_frame][0]][STA_shields[animate_shields_frame][1]][1] = true;
    }
    if (animate_shields_frame == 19 && system_status_light_on[6] == false) {
      for (int i=0; i<=4; i = i+2) {
         light_array[STA_shields[i][0]][STA_shields[i][1]][1] = true;
      }
      for (int i=16; i<=18; i = i+2) {
         light_array[STA_shields[i][0]][STA_shields[i][1]][1] = true;
      }
    }
    if (animate_shields_frame == 19 && system_status_light_on[7] == false) {
      for (int i=6; i<=14; i = i+2) {
         light_array[STA_shields[i][0]][STA_shields[i][1]][1] = true;
      }
    }
    if (animate_shields_frame == 19 && system_status_light_on[6]) {
      for (int i=0; i<=4; i = i+2) {
         light_array[STA_shields[i][0]][STA_shields[i][1]][1] = false;
      }
      for (int i=16; i<=18; i = i+2) {
         light_array[STA_shields[i][0]][STA_shields[i][1]][1] = false;
      }
    }
    if (animate_shields_frame == 19 && system_status_light_on[7]) {
      for (int i=6; i<=14; i = i+2) {
         light_array[STA_shields[i][0]][STA_shields[i][1]][1] = false;
      }
    }
    if (animate_shields_frame == 20 && system_status_light_on[6]) {
      for (int i=0; i<=4; i = i+2) {
         light_array[STA_shields[i][0]][STA_shields[i][1]][1] = true;
      }
      for (int i=16; i<=18; i = i+2) {
         light_array[STA_shields[i][0]][STA_shields[i][1]][1] = true;
      }
    }
    if (animate_shields_frame == 20 && system_status_light_on[7]) {
      for (int i=6; i<=14; i = i+2) {
         light_array[STA_shields[i][0]][STA_shields[i][1]][1] = true;
      }
    }
    animate_shields_frame++;
    if (animate_shields_frame >=21) {
      animate_shields_frame = 19;
    }
  }
  
}


// ANIMATE DAMAGE CODE ***********************************************************************************
void animate_damage() {
  for (int i=0; i<20; i++) {
      light_array[STA_damage[i][0]][STA_damage[i][1]][1] = random(0,40) < long(damage_state * 10);
      if (ship_hit != 0) {
        light_array[STA_damage[i][0]][STA_damage[i][1]][1] = random(0,40) < long(ship_hit * 10);
      }
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

// ANIMATE STATUS PANEL SENSORS CODE ******************************************************************
void animate_sensors() {
  if (system_status_light_on[2]) {
    for (int i=0; i<4; i++) {
      light_array[STA_sensors[i][0]][STA_sensors[i][1]][1] = true;
    }
    light_array[STA_sensors[animate_sensors_frame][0]][STA_sensors[animate_sensors_frame][1]][1] = false;
    light_array[STA_sensors[animate_sensors_frame + 2][0]][STA_sensors[animate_sensors_frame + 2][1]][1] = false;
    animate_sensors_frame++;
    if (animate_sensors_frame >= 2) {
      animate_sensors_frame = 0;
    }
  }
  else {
     for (int i=0; i<4; i++) {
      light_array[STA_sensors[i][0]][STA_sensors[i][1]][1] = false;
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

// REFRESH LIGHTS CODE ************************************************************************************
void refresh_lights() {
  // keep reset lights on
  light_array[1][0][0] = true;
  light_array[6][0][0] = true;
  
  // check preset lights
  if (settings_match) {
    light_array[4][0][0] = false;
    light_array[2][0][0] = true;
  }
  else {
    light_array[4][0][0] = true;
    light_array[2][0][0] = false;
  }
  if (store_on) {
    light_array[7][1][0] = true;
  }
  else {
    light_array[7][1][0] = false;
  }

  //system status lights
  for (int i=0; i<8; i++) {
    light_array[i][6][2] = system_status_light_on[i];
  }
  
}  // END REFRESH LIGHTS CODE


void add_data_to_transmit_queue (int code, int data0, int data1) {
  int pwr = 0;
  RS_485_TX[dmx_transmit_queue_last][0] = 0; // number to clear possible framing errors at the start of the packet
  RS_485_TX[dmx_transmit_queue_last][1] = RS_485_TX_START; // number signifying the start of a new data stream
  RS_485_TX[dmx_transmit_queue_last][2] = byte(code);
  switch (code) {
    case 212:                                                       // console "check-in code" - requested by master controller
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][4] = RS_485_TX_STOP;
    break;
    case 220:                                                       // commands console to report data
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][4] = RS_485_TX_STOP;
    break;
    case 215:                                                       // pass token to master controller
      RS_485_TX[dmx_transmit_queue_last][3] = RS_485_TX_STOP;
    break;
    case 101:                                                       // "top-down" game state from DMX
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][4] = byte(data1);
      RS_485_TX[dmx_transmit_queue_last][5] = RS_485_TX_STOP;
    break;
    case 102:                                                       // "top-down" game events from DMX
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][4] = byte(data1);
      RS_485_TX[dmx_transmit_queue_last][5] = RS_485_TX_STOP;
    break;
    case 103:                                                       // interpreted game state
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][4] = byte(data1);
      RS_485_TX[dmx_transmit_queue_last][5] = RS_485_TX_STOP;
    break;
    case 120: // transmit power settings
      for (int i=0; i<8; i++) {
        if (power_matrix[i][10] >= SLIDER_1_Y) {
          pwr = 0;
        }
        if (power_matrix[i][10] < SLIDER_1_Y) {
          pwr = 6;
        }
        if (power_matrix[i][10] < SLIDER_2_Y) {
          pwr = 16;
        }
        if (power_matrix[i][10] < SLIDER_3_Y) {
          pwr = 26;
        }
        if (power_matrix[i][10] < SLIDER_4_Y) {
          pwr = 36;
        }
        if (power_matrix[i][10] < SLIDER_5_Y) {
          pwr = 46;
        }
        if (power_matrix[i][10] < SLIDER_6_Y) {
          pwr = 56;
        }
        RS_485_TX[dmx_transmit_queue_last][i+3] = byte (pwr);
      }
    break;
    case 133:                                                       // system active code        
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][4] = byte(data1);
      RS_485_TX[dmx_transmit_queue_last][5] = RS_485_TX_STOP;
    break;
    case 134:                                                       // viewscreen state
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][4] = byte(data1);
      RS_485_TX[dmx_transmit_queue_last][5] = RS_485_TX_STOP;
    break;
    case 135:                                                       // viewscreen CAM state
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][4] = byte(data1);
      RS_485_TX[dmx_transmit_queue_last][5] = RS_485_TX_STOP;
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
        }
        data_stream_pos++;
        if (data_stream_pos == 12) {
          data_stream_pos = 0;
          data_stream_on = false;
          dmx_transmit_queue_pos++;
          if (dmx_transmit_queue_pos >= MAX_TRANSMIT_QUEUE) {
            dmx_transmit_queue_pos = 0;
          }
        }
        if (RS_485_TX[dmx_transmit_queue_pos][data_stream_pos] == RS_485_TX_STOP) {
          if (RS_485_TX[dmx_transmit_queue_pos][2] == 215) {
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
        if (RS_485_RX[1] == 220) {                 // code requesting data from controller when not DMX/MCP controller
          if (RS_485_RX[2] == MY_CONTROLLER_ADDRESS) {
            transmit_enable = true;
            add_data_to_transmit_queue(212, MY_CONTROLLER_ADDRESS, 0);
            add_data_to_transmit_queue(120, 0, 0);
            add_data_to_transmit_queue(215, 0, 0);
          }
        }
      }

      if (data_stream_pos > 2) {
        if (RS_485_RX[1] == 101) {
          switch (RS_485_RX[2]) {
            case 7:
              if (RS_485_RX[3] == 1) {
                shields_on = true;
              }
              else {
                shields_on = false;
              }
            break;
            case 8:
              if (RS_485_RX[3] == 1) {
                system_status_light_on[6] = true;
              }
              else {
                system_status_light_on[6] = false;
              }
            break;
            case 9:
              if (RS_485_RX[3] == 1) {
                system_status_light_on[7] = true;
              }
              else {
                system_status_light_on[7] = false;
              }
            break;
            case 28:                          // ship takes internal damage
              if (RS_485_RX[3] == 1) {
                ship_hit = 3;
              }
              else {
                ship_hit = 0;
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
          }
        }
        if (RS_485_RX[1] == 133) {
              if (RS_485_RX[3] == 1) {
                system_status_light_on[RS_485_RX[2]] = true;
              }
              else {
                system_status_light_on[RS_485_RX[2]] = false;
              }
        }
      }

      if (data_stream_pos > 8) {
        if (RS_485_RX[1] == 120) {
          for (int i=0; i<8; i++) {
            power_levels[i] = RS_485_RX[i+2];
          }
        }
      }
      
      data_stream_pos++;
      if (data_stream_pos > 11) {
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
  read_buttons();
}


// read analog controls
if (millis() >= read_analog_next && digitalRead(12) == HIGH) {
  read_analog_next = millis() + ANALOG_READ_RATE;
  read_analog();
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
    animate_sensors();
    animate_weapons_timer = millis() + ANIMATE_WEAPONS_INTERVAL;
  }

// refresh lights
  if (millis() >= refresh_light_delay) {
    refresh_lights();
    
    refresh_light_delay = millis() + REFRESH_LIGHT_INTERVAL;
  }
}

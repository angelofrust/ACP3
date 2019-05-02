// Artemis SBS Weapons Control Panel (Block III) Code
// NOTE: some input pins may misbehave if left open (disconnected from panel hardware)
// connected panel variables are available to exclude input from these disconnected pins

// code optimized for 1280 x 720 client resolution - will not work with other resolutions

// digital inputs from the keypad matrices are arranged active HIGH with a pull-down resistor to make use of the shift registers

// connected panels array - used to tell controller which pins should be read when reading controls - false disables input
bool connected_panels[4] = {true, true, true, false};
// 0 - "center" panel - pins 5, 6, 7, (23) - pin 23 available for additional buttons
// 1 - "right" panel - pins 8, 9, 10, (23) - pin 23 available for additional buttons
// 2 - "left" panel - pins 24, 25, 26, (23) - pin 23 available for additional buttons
// 3 - "center" panel - pins 14, 15, 16, 17, 18, 19, 20, 21 - intended for analog input

// control button functions array - stores button code and press state by relative pin reference
int button_array[8][3][4][2];
// first index is to shift register pin number - generally only one pin will be active at a time
// second index is to the input panel number (0 = "center", 1 = "right", 2 = "left")
// third index is to the input pin number - all input pins will generally be available simultaneously
// fourth index is to the button state and button code:
    // button state - index 0: number of cycles active
    // button code - index 1: numberic code for the command associated with the button
int active_button_pin = 0;  // stores which of the eight button pins on the shift register is on
#define MAX_BUTTON_PIN 7   // last pin of the button array
#define BUTTON_PUSH_THRESHOLD 3   // number of cycles the button switch must be closed to register as a button push

// control button pin references - stores pin numbers associated with each pin reference in the functions array
int button_pins[3][4] = {
  {5, 6, 7, 23},      // panel 0 "center": 5 - shift serial, 6&7 - button inputs
  {8, 9, 10, 10},     // panel 1 "right": 8 - shift serial, 9&10 - button inputs
  {24, 25, 26, 26}    // panel 2 "left": 24 - shift serial, 25&26 - button inputs
};

#define MAX_COMMAND_QUEUE 30
int command_queue[MAX_COMMAND_QUEUE][3];   // array to store commands to be executed by the button-push code
  // index 0: commmand number
  // index 1: x coord
  // index 2: y coord
int current_command = 0;    // variable that points to the current command in the queue
int last_command = 0;    // variable that points to the command after the last command enterred in the queue
unsigned long command_timer = 0;   // variable to store the time between consecutive keyboard commands
#define COMMAND_INTERVAL 10 // milliseconds in between consecutive keyboard commands to the connected PC

// panel indicator light array - stores light states -- true = on, false = off
bool light_array[8][8][3];  // 1st index = supply register 0 - 7
                            // 2nd index = sink register 0 - 7
                            // 3rd index = panel (0 = "center", 1 = "right", 2 = "left")
#define MAX_LIGHT_SUPPLY 7  // last pin of the current supply chips for the entire panel
                            // controller will not cycle lights further down the registers
#define MAX_LIGHT_SINK 5    // last pin of the current sink chips for the entire panel
                            // controller will not cycle lights further down the registers
int active_light_sink = 0;  // currently on sink register pin

// define shift register clock (serial clock), latch (register clock), and output enable pins
#define SHIFT_CLOCK_PIN 3
#define SHIFT_LATCH_PIN 4
#define OUTPUT_ENABLE_PIN 12

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
#define ANALOG_READ_RATE 10  // delay in milliseconds between touch pad readings
#define ANALOG_DIFFERENCE_THRESHOLD 10  // how different the moving average needs to be from the last change to trigger a command
#define MOUSE_SCALE_FACTOR_X 10    // controls sensitivity of mouse movements
#define MOUSE_SCALE_FACTOR_Y -10    // controls sensitivity of mouse movements
#define CLICK_MIN_TIME 100  // minimum hold time (in ms) for a click to register
#define CLICK_MAX_TIME 200  // maximum hold time (in ms) for a clock to register

// WEAPONS constants
#define PIN_Y_NEG 14
#define PIN_X_PLUS 15
#define PIN_Y_PLUS 16
#define PIN_X_NEG 17

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
unsigned long mouse_timer = 0;
int power_levels[8] = {20, 20, 20, 20, 20, 20, 20, 20};
bool shields_on = false;
bool front_shield_low = false;
bool rear_shield_low = false;
bool pad_active = false;
bool pad_done_reading = false;
bool pad_read_x = false;
int pad_reads = 0;
int pad_coords[2][2] = {
  {0, 0},
  {0, 0}
};
bool pad_move = false;
int tube_state[4][2] = {   // -1 = n/a, 0 = empty, 1 = loading, 2 = unloading, 3 = ready
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0}
};
int panel_mode = 2;  // number of torpedo tubes
int autoload_on = false;
int autofire_on = true;
bool science_control_on = false;
int beam_freq = 1;
int torp_select = 1;

// SETUP CODE ***********************************************************************************************
void setup() {
  Serial.begin(9600);   // USB serial - used to debug

  Mouse.screenSize(1280, 720);   // setup mouse commands for 1280 by 720 resolution screen
  
  // setup RS-485 bus *****************************
  Serial1.begin(9600); //Enable serial TX/RX
  Serial1.transmitterEnable(2);   // Use pin 2 to automatically enable a RS-485 transceiver chip. 
                                  // This pin outputs a logic HIGH when transmitting data. 
                                  // It goes LOW after the last stop bit, to allow reception when not transmitting. 

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
  
  // setup status light
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); 

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
  button_array[3][2][2][1] = 10;    // zoom -
  button_array[2][2][2][1] = 11;    // zoom +
  button_array[1][1][1][1] = 20;    // shields up
  button_array[3][1][1][1] = 21;    // shields down
  button_array[0][1][2][1] = 12;    // view forward
  button_array[1][1][2][1] = 14;    // view right
  button_array[2][1][2][1] = 19;    // view "CAM"
  button_array[3][1][2][1] = 13;    // view left
  button_array[4][1][2][1] = 15;    // view aft
  button_array[5][1][2][1] = 18;    // view INFO
  button_array[6][1][2][1] = 17;    // view LRS
  button_array[7][1][2][1] = 16;    // view TAC
  button_array[0][2][1][1] = 34;    // select EMP
  button_array[1][2][1][1] = 38;    // select probe
  button_array[2][2][1][1] = 33;    // select mine
  button_array[3][2][1][1] = 37;    // select tag
  button_array[4][2][1][1] = 32;    // select nuke
  button_array[5][2][1][1] = 36;    // select beacon
  button_array[6][2][1][1] = 31;    // select homing
  button_array[7][2][1][1] = 35;    // select pshock
  button_array[0][2][2][1] = 39;    // convert energy to torpedo
  button_array[1][2][2][1] = 40;    // convert torpedo to energy
  button_array[0][0][1][1] = 45;    // load tube 1
  button_array[1][0][1][1] = 46;    // unload tube 1
  button_array[2][0][1][1] = 47;    // load tube 2
  button_array[3][0][1][1] = 48;    // unload tube 2
  button_array[4][0][1][1] = 49;    // load tube 3
  button_array[5][0][1][1] = 50;    // unload tube 3
  button_array[6][0][1][1] = 51;    // load tube 4
  button_array[7][0][1][1] = 52;    // unload tube 4
  button_array[0][0][2][1] = 41;    // fire tube 1
  button_array[2][0][2][1] = 42;    // fire tube 2
  button_array[4][0][2][1] = 43;    // fire tube 3
  button_array[6][0][2][1] = 44;    // fire tube 4  
  button_array[2][0][3][1] = 53;    // auto beams button
  button_array[4][0][3][1] = 54;    // freq left
  button_array[3][0][3][1] = 55;    // freq right
  button_array[0][0][3][1] = 56;    // click mouse ("select")

  // complex "high-level" commands second
  button_array[3][0][2][1] = 126;   // cycle tube control mode
  button_array[5][0][2][1] = 127;   // toggle autoload
  button_array[7][0][2][1] = 128;   // adjust torpedo system power
  button_array[3][0][2][1] = 126;   // cycle tube control mode
  button_array[5][0][3][1] = 129;   // adjust beam system power
  button_array[6][0][3][1] = 130;   // science frequency control enable

  // end button code setup

  // setup touch pad *****************************
  read_analog_next = millis() + ANALOG_READ_RATE;
  pinMode(PIN_X_PLUS, OUTPUT);   // start in "standby mode" to detemrine if there is a touch
  digitalWrite(PIN_X_PLUS, LOW);
  pinMode(PIN_X_NEG, INPUT);
  pinMode(PIN_Y_NEG, INPUT_PULLUP);
  pinMode(PIN_Y_PLUS, INPUT);

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
    /* Serial.print("command = ");
    Serial.print(command_queue[current_command][0]);
    Serial.print(" * ");
    Serial.println(current_command); */
    switch (command_queue[current_command][0]) {
      case 0:
      break;

      // simple commands block ************
      case 10:
        Keyboard.press(KEY_Y);
        Keyboard.release(KEY_Y);
      break;
      case 11:
        Keyboard.press(KEY_T);
        Keyboard.release(KEY_T);
      break;      
      case 20:
        Keyboard.press(KEY_K);
        Keyboard.release(KEY_K);
      break;
      case 21:
        Keyboard.press(KEY_L);
        Keyboard.release(KEY_L);
      break;
      case 12:
        Keyboard.press(KEY_F2);
        Keyboard.release(KEY_F2);
      break;
      case 13:
        Keyboard.press(KEY_F3);
        Keyboard.release(KEY_F3);
      break;
      case 14:
        Keyboard.press(KEY_F4);
        Keyboard.release(KEY_F4);
      break;
      case 15:
        Keyboard.press(KEY_F5);
        Keyboard.release(KEY_F5);
      break;
      case 16:
        Keyboard.press(KEY_F6);
        Keyboard.release(KEY_F6);
      break;
      case 17:
        Keyboard.press(KEY_F7);
        Keyboard.release(KEY_F7);
      break;
      case 18:
        Keyboard.press(KEY_F8);
        Keyboard.release(KEY_F8);
      break;
      case 19:
        // click on CAM button
        Mouse.moveTo(1180, 130);
        Mouse.click();
      break;
      
      case 56:
        // click mouse button
        Mouse.click();
      break;
      
      case 46:
        // unload 1
        if (panel_mode > 0) {
          Mouse.moveTo(45, 695);
          Mouse.click();
          if (tube_state[0][0] == 3) {
            tube_state[0][0] = 2;
          }          
        }
      break;
      case 48:
        // unload 2
        if (panel_mode > 1) {
          Mouse.moveTo(45, 645);
          Mouse.click();
          if (tube_state[1][0] == 3) {
            tube_state[1][0] = 2;
          }          
        }
      break;
      case 50:
        // unload 3
        if (panel_mode > 2) {
          Mouse.moveTo(45, 595);
          Mouse.click();
          if (tube_state[2][0] == 3) {
            tube_state[2][0] = 2;
          }          
        }
      break;
      case 52:
        // unload 4
        if (panel_mode > 3) {
          Mouse.moveTo(45, 545);
          Mouse.click();
          if (tube_state[3][0] == 3) {
            tube_state[3][0] = 2;
          }          
        }
      break;
      case 45:
        // load 1
        if (panel_mode > 0) {
          Mouse.moveTo(45, 695);
          Mouse.click();
          if (tube_state[0][0] == 0) {
            tube_state[0][0] = 1;
            tube_state[0][1] = torp_select;
          }          
        }
      break;
      case 47:
        // load 2
        if (panel_mode > 1) {
          Mouse.moveTo(45, 645);
          Mouse.click();
          if (tube_state[1][0] == 0) {
            tube_state[1][0] = 1;
            tube_state[1][1] = torp_select;
          }          
        }
      break;
      case 49:
        // load 3
        if (panel_mode > 2) {
          Mouse.moveTo(45, 595);
          Mouse.click();
          if (tube_state[2][0] == 0) {
            tube_state[2][0] = 1;
            tube_state[2][1] = torp_select;
          }          
        }
      break;
      case 51:
        // load 4
        if (panel_mode > 3) {
          Mouse.moveTo(45, 545);
          Mouse.click();
          if (tube_state[3][0] == 0) {
            tube_state[3][0] = 1;
            tube_state[3][1] = torp_select;
          }          
        }
      break;
      case 41:
        // fire 1
        if (panel_mode > 0) {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_1);
          Keyboard.release(KEY_1);
          Keyboard.set_modifier(0);
          if (tube_state[0][0] == 3) {
            tube_state[0][0] = 0;
            tube_state[0][1] = 0;
          }          
        }
      break;
      case 42:
        // fire 2
        if (panel_mode > 1) {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_2);
          Keyboard.release(KEY_2);
          Keyboard.set_modifier(0);
          if (tube_state[1][0] == 3) {
            tube_state[1][0] = 0;
            tube_state[1][1] = 0;
          }          
        }
      break;
      case 43:
        // fire 3
        if (panel_mode > 2) {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_3);
          Keyboard.release(KEY_3);
          Keyboard.set_modifier(0);
          if (tube_state[2][0] == 3) {
            tube_state[2][0] = 0;
            tube_state[2][1] = 0;
          }          
        }
      break;
      case 44:
        // fire 4
        if (panel_mode > 3) {
          Keyboard.set_modifier(MODIFIERKEY_SHIFT);
          Keyboard.press(KEY_4);
          Keyboard.release(KEY_4);
          Keyboard.set_modifier(0);
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
      case 127:
        // toggle autoload
        if (autoload_on) {
          autoload_on = false;
        }
        else {
          autoload_on = true;
        }
      break;
      case 130:
        // toggle science frequence control
        if (science_control_on) {
          science_control_on = false;
        }
        else {
          science_control_on = true;
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
      case 216:
        Mouse.move(command_queue[current_command][1],command_queue[current_command][2]);
      break;
      case 217:
        pad_move = false;
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
  if (pad_reads == 0) {
    if (digitalRead(PIN_Y_NEG) == LOW) {
      if (!pad_active) {
        pad_coords[0][1] = 0;
        pad_coords[1][1] = 0;
        mouse_timer = millis();
      }
      else {
        // mouse moving code - verify that finger is down at beigning and end of move
        if (pad_coords[0][1] != 0) {
          int x = (pad_coords[0][0] - pad_coords[0][1]) * 10 / MOUSE_SCALE_FACTOR_X;
          int y = (pad_coords[1][0] - pad_coords[1][1]) * 10 / MOUSE_SCALE_FACTOR_Y;
          /* Serial.print("move: ");
          Serial.print(x);
          Serial.print(" -- ");
          Serial.println(y); */
          enter_command(216, x, y);
        }
        pad_coords[0][1] = pad_coords[0][0];
        pad_coords[1][1] = pad_coords[1][0];
      }
      pad_reads++;
      pad_coords[0][0] = 0;
      pad_coords[1][0] = 0;
      pinMode(PIN_Y_NEG, INPUT);
      pinMode(PIN_Y_PLUS, INPUT);
      pinMode(PIN_X_NEG, OUTPUT);
      pinMode(PIN_X_PLUS, OUTPUT);
      digitalWrite(PIN_X_NEG, HIGH);
      digitalWrite(PIN_X_PLUS, LOW);
      light_array[0][0][0] = true;
      light_array[1][0][0] = false;
      light_array[2][0][0] = true;
      light_array[3][0][0] = false;
    }
    else {
      if (pad_active == true) {
        if (millis() - mouse_timer > CLICK_MIN_TIME && millis() - mouse_timer < CLICK_MAX_TIME) {
          enter_command(56, 0, 0);
        }
      }
      pad_active = false;
      light_array[0][0][0] = false;
      light_array[1][0][0] = true;
      light_array[2][0][0] = false;
      light_array[3][0][0] = true;
    }
  }
  if (pad_reads > 0) {
    if (pad_reads > 3) {   // read y coords
      pad_coords[1][0] = pad_coords[1][0] + analogRead(PIN_X_PLUS);
      pad_reads++;
      if (pad_reads == 7) {
        pad_coords[1][0] = pad_coords[1][0] / 3;
        /*Serial.print("Y = ");
        Serial.println(pad_coords[1][0]);*/
        pad_reads = 0;
        pinMode(PIN_X_PLUS, OUTPUT);   // set to "standby mode" to detemrine if there is a touch
        digitalWrite(PIN_X_PLUS, LOW);
        pinMode(PIN_X_NEG, INPUT);
        pinMode(PIN_Y_NEG, INPUT_PULLUP);
        pinMode(PIN_Y_PLUS, INPUT);

        pad_active = true;
      }
    }
    else {  // read x coords      
      pad_coords[0][0] = pad_coords[0][0] + analogRead(PIN_Y_PLUS);
      pad_reads++;
      if (pad_reads == 4) {
        pad_coords[0][0] = pad_coords[0][0] / 3;
        /*Serial.print("X = ");
        Serial.print(pad_coords[0][0]);
        Serial.print(" -- ");*/
        pinMode(PIN_X_NEG, INPUT);
        pinMode(PIN_X_PLUS, INPUT);
        pinMode(PIN_Y_NEG, OUTPUT);
        pinMode(PIN_Y_PLUS, OUTPUT);
        digitalWrite(PIN_Y_NEG, HIGH);
        digitalWrite(PIN_Y_PLUS, LOW);
      }
    }
  }

}

// REFRESH LIGHTS CODE ************************************************************************************
void refresh_lights() {
  // turn all lights off first
  for (int i=0; i<=MAX_LIGHT_SUPPLY; i++) {
    for (int k=0; k<=MAX_LIGHT_SINK; k++) {
      light_array[i][k][0] = false;
      light_array[i][k][1] = false;
      light_array[i][k][2] = false;
    }
  }

  if (pad_active) {
    light_array[0][0][0] = true;
    light_array[1][0][0] = false;
    light_array[2][0][0] = true;
    light_array[3][0][0] = false;
  }
  else {
    light_array[0][0][0] = false;
    light_array[1][0][0] = true;
    light_array[2][0][0] = false;
    light_array[3][0][0] = true;
  }

  if (panel_mode > 0) {
    light_array[6][2][0] = true;
    switch (tube_state[0][0]) {
      case 0:
        light_array[7][0][0] = true;
      break;
      case 1:
        light_array[4][0][0] = true;
      break;
      case 2:
        light_array[6][0][0] = true;
      break;
      case 3:
        light_array[5][0][0] = true;
      break;
    }
  }
  if (panel_mode > 1) {
    light_array[5][2][0] = true;
    switch (tube_state[1][0]) {
      case 0:
        light_array[5][1][0] = true;
      break;
      case 1:
        light_array[6][1][0] = true;
      break;
      case 2:
        light_array[4][1][0] = true;
      break;
      case 3:
        light_array[7][1][0] = true;
      break;
    }
  }
  if (panel_mode > 2) {
    light_array[7][2][0] = true;
    switch (tube_state[2][0]) {
      case 0:
        light_array[3][1][0] = true;
      break;
      case 1:
        light_array[0][1][0] = true;
      break;
      case 2:
        light_array[2][1][0] = true;
      break;
      case 3:
        light_array[1][1][0] = true;
      break;
    }
  }
  if (panel_mode > 3) {
    light_array[4][2][0] = true;
    switch (tube_state[3][0]) {
      case 0:
        light_array[1][2][0] = true;
      break;
      case 1:
        light_array[2][2][0] = true;
      break;
      case 2:
        light_array[0][2][0] = true;
      break;
      case 3:
        light_array[3][2][0] = true;
      break;
    }
  }

  if (autoload_on) {
    light_array[6][4][0] = true;
  }
  else {
    light_array[7][4][0] = true;
  }

  if (autofire_on) {
    light_array[7][3][0] = true;
  }

  if (science_control_on) {
    light_array[0][3][0] = true;
  }

  switch (beam_freq) {
    case 1:
      light_array[1][3][0] = true;
    break;
    case 2:
      light_array[2][3][0] = true;
    break;
    case 3:
      light_array[3][3][0] = true;
    break;
    case 4:
      light_array[4][3][0] = true;
    break;
    case 5:
      light_array[5][3][0] = true;
    break;
  }

  switch (torp_select) {
    case 1:
      light_array[3][0][2] = true;
    break;
    case 2:
      light_array[2][0][2] = true;
    break;
    case 3:
      light_array[1][0][2] = true;
    break;
    case 4:
      light_array[0][0][2] = true;
    break;
    case 5:
      light_array[4][0][2] = true;
    break;
    case 6:
      light_array[5][0][2] = true;
    break;
    case 7:
      light_array[7][0][2] = true;
    break;
    case 8:
      light_array[6][0][2] = true;
    break;
  }

  // beam power level
  if (power_levels[0] > 5) {
    light_array[0][5][0] = true;
  }
  if (power_levels[0] > 15) {
    light_array[1][5][0] = true;
  }
  if (power_levels[0] > 25) {
    light_array[2][5][0] = true;
  }
  if (power_levels[0] > 35) {
    light_array[3][5][0] = true;
  }
  if (power_levels[0] > 45) {
    light_array[4][5][0] = true;
  }
  if (power_levels[0] > 55) {
    light_array[5][5][0] = true;
  }
  
  // torpedo power level
  if (power_levels[1] > 5) {
    light_array[0][4][0] = true;
  }
  if (power_levels[1] > 15) {
    light_array[1][4][0] = true;
  }
  if (power_levels[1] > 25) {
    light_array[2][4][0] = true;
  }
  if (power_levels[1] > 35) {
    light_array[3][4][0] = true;
  }
  if (power_levels[1] > 45) {
    light_array[4][4][0] = true;
  }
  if (power_levels[1] > 55) {
    light_array[5][4][0] = true;
  }

  // shields status
  if (shields_on) {
    light_array[0][0][1] = true;
    light_array[0][1][1] = true;
    light_array[0][2][1] = true;
    light_array[0][3][1] = true;
  }
  else {
    light_array[1][0][1] = true;
    light_array[1][1][1] = true;
    light_array[1][2][1] = true;
    light_array[1][3][1] = true;
  }

  // shields low
  if (front_shield_low) {
    light_array[2][0][1] = true;
    light_array[2][1][1] = true;
    light_array[2][2][1] = true;
    light_array[2][3][1] = true;
  }
  if (rear_shield_low) {
    light_array[3][0][1] = true;
    light_array[3][1][1] = true;
    light_array[3][2][1] = true;
    light_array[3][3][1] = true;
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
            Serial.println(" ");
            Serial.print("TX: ");
          }
          Serial.print(RS_485_TX[dmx_transmit_queue_pos][data_stream_pos]);
          Serial.print(" ");
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
      digitalWrite(13, HIGH);
      int rx = Serial1.read();
      if (rx == RS_485_TX_START) {
        data_stream_pos = 0;
        Serial.println(" ");
        Serial.print("RX -- ");
      }
      RS_485_RX[data_stream_pos] = rx;
      Serial.print(rx, DEC);
      Serial.print(" ");

      // sort data into appropriate places

      if (data_stream_pos > 1) {
        if (RS_485_RX[1] == 220) {
          if (RS_485_RX[2] == MY_CONTROLLER_ADDRESS) {
            transmit_enable = true;
            add_data_to_transmit_queue(212, MY_CONTROLLER_ADDRESS, 0);
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
                front_shield_low = true;
              }
              else {
                front_shield_low = false;
              }
            break;
            case 9:
              if (RS_485_RX[3] == 1) {
                rear_shield_low = true;
              }
              else {
                rear_shield_low = false;
              }
            break;
            case 58:
              tube_state[0][0] = 3;
            break;
            case 59:
              tube_state[1][0] = 3;
            break;
            case 60:
              tube_state[2][0] = 3;
            break;
            case 61:
              tube_state[3][0] = 3;
            break;
            case 62:
              tube_state[0][0] = 0;
            break;
            case 63:
              tube_state[1][0] = 0;
            break;
            case 64:
              tube_state[2][0] = 0;
            break;
            case 65:
              tube_state[3][0] = 0;
            break;
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
      if (data_stream_pos > 10) {
        data_stream_pos = 2;
      }
    }
    digitalWrite(13, LOW);
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
if (millis() >= read_analog_next) {
  read_analog_next = millis() + ANALOG_READ_RATE;
  read_analog();
}

// execute commands from command queue
  if (millis() >= command_timer) {
    execute_command();
    command_timer = millis() + COMMAND_INTERVAL;
  }

// refresh lights
  if (millis() >= refresh_light_delay) {
    refresh_lights();
    
    refresh_light_delay = millis() + REFRESH_LIGHT_INTERVAL;
  }

}

// Artemis SBS Controller Code
// ACP3 COMMS Controller Prototype

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
#define MAX_LIGHT_SINK 5    // last pin of the current sink chips for the entire panel
                            // controller will not cycle lights further down the registers
int active_light_sink = 0;  // currently on sink register pin

// define shift register clock (serial clock), latch (register clock), and output enable pins
#define SHIFT_CLOCK_PIN 2
#define SHIFT_LATCH_PIN 3
#define OUTPUT_ENABLE_PIN 4

// times to cycle the shift registers and sample the buttons
unsigned long shift_register_timer = 0;   // variable to store the time between shift register cycles
#define SHIFT_REGISTER_INTERVAL 2        // milliseconds betwen shift register cycles

unsigned long refresh_light_delay = 0;
#define REFRESH_LIGHT_INTERVAL 250   // milliseconds between refreshing all lights against the stored data states

// analog controls - general
unsigned long read_analog_next = 0;   // variable to store the time of the next analog reading
#define ANALOG_READ_RATE 50  // delay in milliseconds between warp slider readings
#define ANALOG_DIFFERENCE_THRESHOLD 10  // how different the moving average needs to be from the last change to trigger a command

// COMM constants
#define JOYSTICK_X_MIN 0
#define JOYSTICK_Y_MIN 0
#define JOYSTICK_X_MAX 0
#define JOYSTICK_Y_MAX 0
#define JOYSTICK_DEADZONE 5
#define JOYSTICK_SCALE -30
#define ABS_JOYSTICK_SCALE -2
#define SCREEN_CENTER_X 640
#define SCREEN_CENTER_Y 360
#define COMMS_SORT_ORIGIN_X 405
#define COMMS_SORT_ORIGIN_Y 110
#define COMMS_SORT_DELTA_X 160
#define COMMS_SORT_DELTA_Y 28

// console-specific (COMMS) variables
#define MAX_TRANSMIT_QUEUE 30
int dmx_transmit_queue_pos = 0;
int dmx_transmit_queue_last = 0;
#define MY_CONTROLLER_ADDRESS 50          // COMMS address (does not apply to DMX/MCP variant)
byte RS_485_TX[MAX_TRANSMIT_QUEUE][10];   // variable to store bytes for transmittal on the RS-485 bus
int RS_485_RX[10];  // variable to store received bytes
bool data_stream_on = false; // variable to store in currently transmitting data state
int data_stream_pos = 0;  // varaible to store where in the block of data we are currently transmitting/receiving
bool transmit_enable = false; // variable to turn ability to transmit on and off (token)
#define RS_485_TX_START 255 // symbol to signify start of transmitting stream
#define RS_485_TX_STOP 254 // symbol to stop transmitting stream

bool transmit_data = false;
bool control_enable = true;
bool joystick_enable = true;
int joystick_x_center = 512;
int joystick_y_center = 512;
int power_levels[8] = {20, 20, 20, 20, 20, 20, 20, 20};
bool message_filter[7] = {false, false, false, false, false, false, false};
bool red_alert_on = false;
bool normal_on = false;
bool shields_on = false;
bool front_shield_low = false;
bool rear_shield_low = false;
bool jump_on = false;
bool dock_state_on = false;
bool tractor_state_on = false;
bool alert_on = false;
bool shield_hit_on = false;
bool damage_on = false;
bool damcon_on = false;
int damage_state = 0;
int power_state = 5;
int slider_reads = 0;
bool slider_command_on[4] = {false, false, false, false};
unsigned long mission_timer = 0;
int computer_mode = 0;    // 0 = timer, 1 = monitor, 2 = program
int monitor_code = 40;
int monitor_value = 0;
bool enter_mode = false;
int enter_char = 0;
int enter_value = 0;

unsigned long animate_buttons_timer = 0;   // variable to store the time between tube animation frames
#define ANIMATE_BUTTONS_INTERVAL 500   // milliseconds between tube loading animation frames
byte animate_buttons_frame = 0;


// SETUP CODE ***********************************************************************************************


void setup() {
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
  button_array[1][2][3][1] = 110;    // 0 key
  button_array[4][2][2][1] = 109;    // 9 key
  button_array[2][2][2][1] = 108;    // 8 key
  button_array[0][2][2][1] = 107;    // 7 key
  button_array[5][2][2][1] = 106;    // 6 key
  button_array[3][2][2][1] = 105;    // 5 key
  button_array[1][2][2][1] = 104;    // 4 key
  button_array[6][2][2][1] = 56;    // select button (click)
  button_array[4][2][3][1] = 103;    // 3 key
  button_array[2][2][3][1] = 102;    // 2 key
  button_array[0][2][3][1] = 101;    // 1 key
  button_array[7][2][1][1] = 111;    // red alert

  button_array[0][2][1][1] = 161;    // sort alert
  button_array[2][2][1][1] = 162;    // sort side
  button_array[4][2][1][1] = 163;    // sort stat
  button_array[6][2][1][1] = 164;    // sort player
  button_array[1][2][1][1] = 165;    // sort station
  button_array[3][2][1][1] = 166;    // sort enemy
  button_array[5][2][1][1] = 167;    // sort friend

  // complex "high-level" commands second
  // COMPUTER panel
  button_array[1][1][1][1] = 152;   // select timer mode
  button_array[6][1][2][1] = 140;   // press comp '0'
  button_array[5][1][1][1] = 149;   // press comp '9'
  button_array[0][1][1][1] = 151;   // select monitor input mode
  button_array[4][1][1][1] = 148;   // press comp '8' 
  button_array[2][1][1][1] = 150;   // select program input mode
  button_array[3][1][1][1] = 147;   // press comp '7' 
  button_array[7][1][2][1] = 153;   // press comp enter key
  button_array[2][1][2][1] = 146;   // press comp '6'
  button_array[5][1][2][1] = 143;   // press comp '3'
  button_array[1][1][2][1] = 145;   // press comp '5'
  button_array[4][1][2][1] = 142;   // press comp '2'
  button_array[0][1][2][1] = 144;   // press comp '4'
  button_array[3][1][2][1] = 141;   // press comp '1'
 
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

  // placeholder indicator lights
  light_array[0][4][1] = true;         // timer button light
  light_array[6][4][1] = true;         // timer 'colon' light
  light_array[3][0][1] = true;         // timer digit underscore (temporary)
  light_array[3][1][1] = true;         // timer digit underscore (temporary)
  light_array[3][2][1] = true;         // timer digit underscore (temporary)
  light_array[3][3][1] = true;         // timer digit underscore (temporary)

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
  animate_buttons_timer = millis() + ANIMATE_BUTTONS_INTERVAL;

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


void set_led_number(int number, int pos) {
  if (pos < 0) {
    pos = 0;
  }
  if (pos > 5) {
    pos = 5;
  }
          
  switch (number){
    case 0:
      light_array[5][pos][1] = true; // UPPER LEFT segment
      light_array[0][pos][1] = true; // TOP segment
      light_array[1][pos][1] = true; // UPPER RIGHT segment
      light_array[6][pos][1] = false; // MIDDLE segment
      light_array[4][pos][1] = true; // LOWER LEFT segment
      light_array[3][pos][1] = true; // BOTTOM segment
      light_array[2][pos][1] = true; // LOWER RIGHT segment
    break;
    case 1:
      light_array[5][pos][1] = false; // UPPER LEFT segment
      light_array[0][pos][1] = false; // TOP segment
      light_array[1][pos][1] = true; // UPPER RIGHT segment
      light_array[6][pos][1] = false; // MIDDLE segment
      light_array[4][pos][1] = false; // LOWER LEFT segment
      light_array[3][pos][1] = false; // BOTTOM segment
      light_array[2][pos][1] = true; // LOWER RIGHT segment
    break;
    case 2:
      light_array[5][pos][1] = false; // UPPER LEFT segment
      light_array[0][pos][1] = true; // TOP segment
      light_array[1][pos][1] = true; // UPPER RIGHT segment
      light_array[6][pos][1] = true; // MIDDLE segment
      light_array[4][pos][1] = true; // LOWER LEFT segment
      light_array[3][pos][1] = true; // BOTTOM segment
      light_array[2][pos][1] = false; // LOWER RIGHT segment
    break;
    case 3:
      light_array[5][pos][1] = false; // UPPER LEFT segment
      light_array[0][pos][1] = true; // TOP segment
      light_array[1][pos][1] = true; // UPPER RIGHT segment
      light_array[6][pos][1] = true; // MIDDLE segment
      light_array[4][pos][1] = false; // LOWER LEFT segment
      light_array[3][pos][1] = true; // BOTTOM segment
      light_array[2][pos][1] = true; // LOWER RIGHT segment
    break;
    case 4:
      light_array[5][pos][1] = true; // UPPER LEFT segment
      light_array[0][pos][1] = false; // TOP segment
      light_array[1][pos][1] = true; // UPPER RIGHT segment
      light_array[6][pos][1] = true; // MIDDLE segment
      light_array[4][pos][1] = false; // LOWER LEFT segment
      light_array[3][pos][1] = false; // BOTTOM segment
      light_array[2][pos][1] = true; // LOWER RIGHT segment
    break;
    case 5:
      light_array[5][pos][1] = true; // UPPER LEFT segment
      light_array[0][pos][1] = true; // TOP segment
      light_array[1][pos][1] = false; // UPPER RIGHT segment
      light_array[6][pos][1] = true; // MIDDLE segment
      light_array[4][pos][1] = false; // LOWER LEFT segment
      light_array[3][pos][1] = true; // BOTTOM segment
      light_array[2][pos][1] = true; // LOWER RIGHT segment
    break;
    case 6:
      light_array[5][pos][1] = true; // UPPER LEFT segment
      light_array[0][pos][1] = true; // TOP segment
      light_array[1][pos][1] = false; // UPPER RIGHT segment
      light_array[6][pos][1] = true; // MIDDLE segment
      light_array[4][pos][1] = true; // LOWER LEFT segment
      light_array[3][pos][1] = true; // BOTTOM segment
      light_array[2][pos][1] = true; // LOWER RIGHT segment
    break;
    case 7:
      light_array[5][pos][1] = false; // UPPER LEFT segment
      light_array[0][pos][1] = true; // TOP segment
      light_array[1][pos][1] = true; // UPPER RIGHT segment
      light_array[6][pos][1] = false; // MIDDLE segment
      light_array[4][pos][1] = false; // LOWER LEFT segment
      light_array[3][pos][1] = false; // BOTTOM segment
      light_array[2][pos][1] = true; // LOWER RIGHT segment
    break;
    case 8:
      light_array[5][pos][1] = true; // UPPER LEFT segment
      light_array[0][pos][1] = true; // TOP segment
      light_array[1][pos][1] = true; // UPPER RIGHT segment
      light_array[6][pos][1] = true; // MIDDLE segment
      light_array[4][pos][1] = true; // LOWER LEFT segment
      light_array[3][pos][1] = true; // BOTTOM segment
      light_array[2][pos][1] = true; // LOWER RIGHT segment
    break;
    case 9:
      light_array[5][pos][1] = true; // UPPER LEFT segment
      light_array[0][pos][1] = true; // TOP segment
      light_array[1][pos][1] = true; // UPPER RIGHT segment
      light_array[6][pos][1] = true; // MIDDLE segment
      light_array[4][pos][1] = false; // LOWER LEFT segment
      light_array[3][pos][1] = true; // BOTTOM segment
      light_array[2][pos][1] = true; // LOWER RIGHT segment
    break;
  }
}   // end code to set numbers on display


void set_led_char(char character, int pos) {
  if (pos < 0) {
    pos = 0;
  }
  if (pos > 5) {
    pos = 5;
  }
      
  switch (character){
    case 'C':
      light_array[5][pos][1] = true; // UPPER LEFT segment
      light_array[0][pos][1] = true; // TOP segment
      light_array[1][pos][1] = false; // UPPER RIGHT segment
      light_array[6][pos][1] = false; // MIDDLE segment
      light_array[4][pos][1] = true; // LOWER LEFT segment
      light_array[3][pos][1] = true; // BOTTOM segment
      light_array[2][pos][1] = false; // LOWER RIGHT segment
    break;
    case '_':
      light_array[5][pos][1] = false; // UPPER LEFT segment
      light_array[0][pos][1] = false; // TOP segment
      light_array[1][pos][1] = false; // UPPER RIGHT segment
      light_array[6][pos][1] = false; // MIDDLE segment
      light_array[4][pos][1] = false; // LOWER LEFT segment
      light_array[3][pos][1] = true; // BOTTOM segment
      light_array[2][pos][1] = false; // LOWER RIGHT segment
    break;
    case '-':
      light_array[5][pos][1] = false; // UPPER LEFT segment
      light_array[0][pos][1] = false; // TOP segment
      light_array[1][pos][1] = false; // UPPER RIGHT segment
      light_array[6][pos][1] = true; // MIDDLE segment
      light_array[4][pos][1] = false; // LOWER LEFT segment
      light_array[3][pos][1] = false; // BOTTOM segment
      light_array[2][pos][1] = false; // LOWER RIGHT segment
    break;
    case ' ':
      light_array[5][pos][1] = false; // UPPER LEFT segment
      light_array[0][pos][1] = false; // TOP segment
      light_array[1][pos][1] = false; // UPPER RIGHT segment
      light_array[6][pos][1] = false; // MIDDLE segment
      light_array[4][pos][1] = false; // LOWER LEFT segment
      light_array[3][pos][1] = false; // BOTTOM segment
      light_array[2][pos][1] = false; // LOWER RIGHT segment
    break;
    case 'h':
      light_array[5][pos][1] = true; // UPPER LEFT segment
      light_array[0][pos][1] = false; // TOP segment
      light_array[1][pos][1] = false; // UPPER RIGHT segment
      light_array[6][pos][1] = true; // MIDDLE segment
      light_array[4][pos][1] = true; // LOWER LEFT segment
      light_array[3][pos][1] = false; // BOTTOM segment
      light_array[2][pos][1] = true; // LOWER RIGHT segment
    break;
    case 'd':
      light_array[5][pos][1] = false; // UPPER LEFT segment
      light_array[0][pos][1] = false; // TOP segment
      light_array[1][pos][1] = true; // UPPER RIGHT segment
      light_array[6][pos][1] = true; // MIDDLE segment
      light_array[4][pos][1] = true; // LOWER LEFT segment
      light_array[3][pos][1] = true; // BOTTOM segment
      light_array[2][pos][1] = true; // LOWER RIGHT segment
    break;
  }
}   // end code to set numbers on display ************************************************************************


void type_number(int number) { 
  if (computer_mode == 1 && enter_mode) {
    switch (enter_char) {
      case 0:
        enter_value = 100 * number;
      break;
      case 1:
        enter_value = enter_value + 10 * number;
      break;
      case 2:
        enter_value = enter_value + number;
        enter_mode = false;
        monitor_code = enter_value;
        monitor_value = 99;
      break;
      default:
        enter_mode = false;
      break;
    }
    set_led_number(number, enter_char);
    enter_char++;
    set_led_char('_', enter_char);
  }
}  // end type number code ********************************************************************************


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
      case 56:
        // click mouse button
        Mouse.click();
      break;
      
      case 110:
        // press 0 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_0);
        Keyboard.release(KEY_0);
        Keyboard.set_modifier(0);        
      break;
      case 109:
        // press 9 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_9);
        Keyboard.release(KEY_9);
        Keyboard.set_modifier(0);        
      break;
      case 108:
        // press 8 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_8);
        Keyboard.release(KEY_8);
        Keyboard.set_modifier(0);        
      break;
      case 107:
        // press 7 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_7);
        Keyboard.release(KEY_7);
        Keyboard.set_modifier(0);        
      break;
      case 106:
        // press 6 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_6);
        Keyboard.release(KEY_6);
        Keyboard.set_modifier(0);        
      break;
      case 105:
        // press 5 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_5);
        Keyboard.release(KEY_5);
        Keyboard.set_modifier(0);        
      break;
      case 104:
        // press 4 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_4);
        Keyboard.release(KEY_4);
        Keyboard.set_modifier(0);        
      break;
      case 103:
        // press 3 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_3);
        Keyboard.release(KEY_3);
        Keyboard.set_modifier(0);        
      break;
      case 102:
        // press 2 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_2);
        Keyboard.release(KEY_2);
        Keyboard.set_modifier(0);        
      break;
      case 101:
        // press 1 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_1);
        Keyboard.release(KEY_1);
        Keyboard.set_modifier(0);        
      break;

      case 111:
        // red alert
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_R);
        Keyboard.release(KEY_R);
        Keyboard.set_modifier(0);        
      break;

      case 161:
        // sort messages - alert
        Mouse.moveTo(COMMS_SORT_ORIGIN_X, COMMS_SORT_ORIGIN_Y);
        Mouse.click();
        if (message_filter[0] == true) {
          message_filter[0] = false;
        }
        else {
          message_filter[0] = true;
        }
      break;
      case 162:
        // sort messages - side
        Mouse.moveTo(COMMS_SORT_ORIGIN_X + COMMS_SORT_DELTA_X, COMMS_SORT_ORIGIN_Y);
        Mouse.click();
        if (message_filter[2] == true) {
          message_filter[2] = false;
        }
        else {
          message_filter[2] = true;
        }
      break;
      case 163:
        // sort messages - status
        Mouse.moveTo(COMMS_SORT_ORIGIN_X + 2 * COMMS_SORT_DELTA_X, COMMS_SORT_ORIGIN_Y);
        Mouse.click();
        if (message_filter[4] == true) {
          message_filter[4] = false;
        }
        else {
          message_filter[4] = true;
        }
      break;
      case 164:
        // sort messages - player
        Mouse.moveTo(COMMS_SORT_ORIGIN_X + 3 * COMMS_SORT_DELTA_X, COMMS_SORT_ORIGIN_Y);
        Mouse.click();
        if (message_filter[6] == true) {
          message_filter[6] = false;
        }
        else {
          message_filter[6] = true;
        }
      break;
      case 165:
        // sort messages - station
        Mouse.moveTo(COMMS_SORT_ORIGIN_X, COMMS_SORT_ORIGIN_Y + COMMS_SORT_DELTA_Y);
        Mouse.click();
        if (message_filter[1] == true) {
          message_filter[1] = false;
        }
        else {
          message_filter[1] = true;
        }
      break;
      case 166:
        // sort messages - enemy
        Mouse.moveTo(COMMS_SORT_ORIGIN_X + COMMS_SORT_DELTA_X, COMMS_SORT_ORIGIN_Y + COMMS_SORT_DELTA_Y);
        Mouse.click();
        if (message_filter[3] == true) {
          message_filter[3] = false;
        }
        else {
          message_filter[3] = true;
        }
      break;
      case 167:
        // sort messages - friend
        Mouse.moveTo(COMMS_SORT_ORIGIN_X + 2* COMMS_SORT_DELTA_X, COMMS_SORT_ORIGIN_Y + COMMS_SORT_DELTA_Y);
        Mouse.click();
        if (message_filter[5] == true) {
          message_filter[5] = false;
        }
        else {
          message_filter[5] = true;
        }
      break;

      // complex command block ************
      case 140:
        type_number(0);
      break;
      case 141:
        type_number(1);
      break;
      case 142:
        type_number(2);
      break;
      case 143:
        type_number(3);
      break;
      case 144:
        type_number(4);
      break;
      case 145:
        type_number(5);
      break;
      case 146:
        type_number(6);
      break;
      case 147:
        type_number(7);
      break;
      case 148:
        type_number(8);
      break;
      case 149:
        type_number(9);
      break;
      case 150:   // push PROG button:
        /*computer_mode = 0;
        nav_disp = NAV_DISP_HDG;
        enter_num = course_array[active_course][0];
        enter_num_pos = 0;
        set_led_char('h',0);
        set_led_char('_',1);
        set_led_char(' ',2);
        set_led_char(' ',3);*/
      break;
      case 151:   // push TIME button
        computer_mode = 0;
      break;
      case 152:   // push MNTR button
        computer_mode = 1;
      break;
      case 153:   // push ENTER button
        if(computer_mode == 1 && enter_mode){
          enter_mode = false;
        }
        else {
          enter_mode = true;
          enter_char = 0;
          set_led_char('_', 0);
          set_led_char(' ', 1);
          set_led_char(' ', 2);
          set_led_char(' ', 3);
        }
      break;
      
      // mouse control block *******************************
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
    
    if (read_distance > JOYSTICK_DEADZONE) {
      readX = readX / JOYSTICK_SCALE;
      readY = readY / JOYSTICK_SCALE;
      Mouse.move(readX,readY);
    }
  }
}


// ANIMATE BUTTONS CODE ***********************************************************************************
void animate_buttons() {
  for (int i=0; i<6; i++) {
      light_array[i][1][2] = random(0,40) < 36;
      light_array[i][2][2] = random(0,40) < 36;
    }  
}


// REFRESH LIGHTS CODE ************************************************************************************
void refresh_lights() {
  // turn all lights off first
  for (int i=0; i<=MAX_LIGHT_SUPPLY; i++) {
    light_array[i][0][2] = false;
    light_array[i][3][2] = false;
    light_array[i][4][2] = false;
    light_array[i][5][2] = false;
  }

  // shields low
  if (front_shield_low || rear_shield_low) {
    light_array[3][3][2] = true;
  }
  // red alert
  if (red_alert_on) {
    light_array[2][3][2] = true;
    light_array[7][0][2] = true;
  }  
  // normal
  if (normal_on) {
    light_array[0][3][2] = true;
  }
  // tractored for dock
  if (tractor_state_on) {
    light_array[4][3][2] = true;
  }
  // docked
  if (dock_state_on) {
    light_array[6][3][2] = true;
  }
  // jump
  if (jump_on) {
    light_array[0][4][2] = true;
  }
  // alert
  if (alert_on) {
    light_array[1][3][2] = true;
  }
  // shield hit
  if (shield_hit_on) {
    light_array[5][3][2] = true;
  }
  // damage
  if (damage_on) {
    light_array[7][3][2] = true;
  }
  // damcon loss
  if (damcon_on) {
    light_array[0][5][2] = true;
  }

  switch (damage_state) {
    case 0:
      light_array[4][4][2] = true;
    break;
    case 1:
      light_array[3][4][2] = true;
    break;
    case 2:
      light_array[2][4][2] = true;
    break;
    case 3:
      light_array[1][4][2] = true;
    break;
  }
  if (power_state > 0) {
    light_array[6][5][2] = true;
  }
  if (power_state > 1) {
    light_array[5][5][2] = true;
  }
  if (power_state > 2) {
    light_array[4][5][2] = true;
  }
  if (power_state > 3) {
    light_array[3][5][2] = true;
  }
  if (power_state > 4) {
    light_array[2][5][2] = true;
  }
  if (power_state > 5) {
    light_array[1][5][2] = true;
  }

  for (int i=0; i<=6; i++) {
    light_array[i][0][2] = message_filter[i];
  }

  if (computer_mode == 0) {
    // timer code
    light_array[0][4][1] = true;
    light_array[1][4][1] = false;
    light_array[7][5][1] = false;
    if (normal_on) {
      int mission_seconds = (millis() - mission_timer) / 1000;
      set_led_number(mission_seconds / 600, 0);
      mission_seconds = mission_seconds % 600;
      set_led_number(mission_seconds / 60, 1);
      mission_seconds = mission_seconds % 60;
      set_led_number(mission_seconds / 10, 2);
      set_led_number(mission_seconds % 10, 3);
    }
    else {
      for (int i=0; i<=3; i++) {
        set_led_char('_',i);
      }
    }
  }
  if (computer_mode == 1) {
    // monitor mode
    light_array[0][4][1] = false;
    light_array[1][4][1] = true;
    light_array[7][5][1] = true;
    light_array[3][4][1] = enter_mode;
    light_array[4][4][1] = enter_mode;
    light_array[5][4][1] = enter_mode;
    light_array[0][5][1] = enter_mode;
    light_array[1][5][1] = enter_mode;
    light_array[2][5][1] = enter_mode;
    light_array[3][5][1] = enter_mode;
    light_array[4][5][1] = enter_mode;
    light_array[5][5][1] = enter_mode;
    light_array[6][5][1] = enter_mode;
    if (!enter_mode) {
      set_led_number((monitor_code % 100) / 10, 0);
      set_led_number(monitor_code % 10, 1);
      set_led_number((monitor_value % 100) / 10, 2);
      set_led_number(monitor_value % 10, 3);
    }
  }
  
  
}


void add_data_to_transmit_queue (int code, int data0, int data1) {
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
        if (data_stream_pos == 10) {
          data_stream_pos = 0;
          data_stream_on = false;
          dmx_transmit_queue_pos++;
          if (dmx_transmit_queue_pos >= MAX_TRANSMIT_QUEUE) {
            dmx_transmit_queue_pos = 0;
          }
        }
        if (RS_485_TX[dmx_transmit_queue_pos][data_stream_pos] == RS_485_TX_STOP) {     
          if (RS_485_TX[dmx_transmit_queue_pos][2] == 215) {      // return token to master controller (basic controller code)
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
        if (RS_485_RX[1] == 220) {                // code requesting data from controller when not DMX/MCP controller
          if (RS_485_RX[2] == MY_CONTROLLER_ADDRESS) {
            transmit_enable = true;
            add_data_to_transmit_queue(212, MY_CONTROLLER_ADDRESS, 0);
            add_data_to_transmit_queue(215, 0, 0);   // return TX token to master controller
          }
        }
      }

      if (data_stream_pos > 2) {
        if (RS_485_RX[1] == 101) {
          if (RS_485_RX[2] == monitor_code) {
            monitor_value = RS_485_RX[3];
          }
          if (monitor_code == 98) {
            monitor_value = power_state;
          }
          switch (RS_485_RX[2]) {
            case 0:                           // receive normal state cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                if (normal_on != true) {
                  mission_timer = millis();
                  normal_on = true;
                }
                
              }
              else {
                normal_on = false;
              }
            break;
            case 3:                           // receive red alert state cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                red_alert_on = true;
              }
              else {
                red_alert_on = false;
              }
            break;
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
              }
              else {
                front_shield_low = false;
              }
            break;
            case 9:                           // receive rear shield low cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                rear_shield_low = true;
              }
              else {
                rear_shield_low = false;
              }
            break;
            case 11:                          // receive jump event cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                jump_on = true;
              }
              else {
                jump_on = false;
              }
            break;
            case 17:                           // receive tractor for dock cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                tractor_state_on = true;
              }
              else {
                tractor_state_on = false;
              }
            break;
            case 18:                           // receive docked cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                dock_state_on = true;
              }
              else {
                dock_state_on = false;
              }
            break;
            case 20:                          // receive shield hit event cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                shield_hit_on = true;
              }
              else {
                shield_hit_on = false;
              }
            break;
            case 28:                          // receive ship damaged event cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                damage_on = true;
              }
              else {
                damage_on = false;
              }
            break;
            case 37:                          // receive damcon loss event cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                damcon_on = true;
              }
              else {
                damcon_on = false;
              }
            break;
            case 32:                           // receive damage state cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                damage_state = 1;
              }
              else {
                damage_state = 0;
              }
            break;
            case 33:                           // receive damage state cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                damage_state = 2;
              }
            break;
            case 34:                           // receive damage state cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                damage_state = 3;
              }
            break;
            case 40:                           // receive energy state cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                power_state = 1;
              }
            break;
            case 41:                           // receive energy state cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                power_state = 2;
              }
            break;
            case 42:                           // receive energy state cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                power_state = 3;
              }
            break;
            case 43:                           // receive energy state cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                power_state = 4;
              }
            break;
            case 44:                           // receive energy state cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                power_state = 5;
              }
            break;
            default:
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

  // check for instructions from master controller & transmit if it is this controller's turn (regular code)

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
  if (millis() >= animate_buttons_timer) {
    animate_buttons();
    animate_buttons_timer = millis() + ANIMATE_BUTTONS_INTERVAL;
  }

  // refresh lights
  if (millis() >= refresh_light_delay) {
    refresh_lights();
    
    refresh_light_delay = millis() + REFRESH_LIGHT_INTERVAL;
  }
  
}

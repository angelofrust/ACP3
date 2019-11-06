// Artemis SBS Controller Code
// ACP3 SCIENCE Controller Prototype

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

#define MAX_COMMAND_QUEUE 30
int command_queue[MAX_COMMAND_QUEUE][3];   // array to store commands to be executed by the button-push code
  // index 0: commmand number
  // index 1: x coord
  // index 2: y coord
int current_command = 0;    // variable that points to the current command in the queue
int last_command = 0;    // variable that points to the command after the last command enterred in the queue
unsigned long command_timer = 0;   // variable to store the time between consecutive keyboard commands
#define COMMAND_INTERVAL 50 // milliseconds in between consecutive keyboard commands to the connected PC
#define COMMAND_INTERVAL_SLOW 100

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

// SCIENCE constants
#define JOYSTICK_X_MIN 0
#define JOYSTICK_Y_MIN 0
#define JOYSTICK_X_MAX 0
#define JOYSTICK_Y_MAX 0
#define JOYSTICK_DEADZONE 5
#define JOYSTICK_SCALE -30
#define ABS_JOYSTICK_SCALE -2
#define MAP_CENTER_X 640
#define MAP_CENTER_Y 360
#define MAP_FAST_DELTA_X 200
#define MAP_FAST_DELTA_Y 200
#define MAP_SCREEN_DELTA_X 600
#define MAP_SCREEN_DELTA_Y 320

#define SCANNER_DELAY 1800 // time in milliseconds between 1/6th scan cycle dots

// console-specific (SCIENCE) variables
#define MAX_TRANSMIT_QUEUE 30
int dmx_transmit_queue_pos = 0;
int dmx_transmit_queue_last = 0;
#define MY_CONTROLLER_ADDRESS 30          // SCIENCE address (does not apply to DMX/MCP variant
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
int slider_reads = 0;
bool slider_command_on[4] = {false, false, false, false};
int map_mode = 2;
int scan_counter = 0;
unsigned long scanner_timer = 0;
unsigned long nav_blink_timer = 0;
#define NAV_BLINK_INTERVAL 750
bool nav_blink_state = false;
int external_effects[6] = {0, 0, 0, 0, 0, 0};
unsigned long effects_decay_timer = 0;
#define DECAY_INTERVAL 2000     // milliseconds between external effects decay events
#define DECAY_AMOUNT 100
#define EFFECTS_MAX 800
int course_array[10][2] = {
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0},
  {0, 0}
};
int active_course = 0;
int nav_mode = 0;
int helm_course = 0;
#define NAV_MODE_NORMAL 0
#define NAV_MODE_HDG 1
#define NAV_MODE_DIST 2
#define NAV_MODE_LOCKED 3
#define NAV_MODE_TYPE 4
int nav_disp = 0;
#define NAV_DISP_HDG 0
#define NAV_DISP_DIST 1
int enter_num = 0;
int enter_num_pos = -1;

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
  button_array[0][2][2][1] = 61;    // zoom -
  button_array[1][2][2][1] = 60;    // zoom +
  button_array[7][2][1][1] = 62;    // scan
  button_array[2][2][2][1] = 65;    // select previous
  button_array[3][2][2][1] = 64;    // select next
  button_array[4][2][2][1] = 63;    // select nearest
  button_array[5][2][2][1] = 56;    // click

  // complex "high-level" commands second
  button_array[0][2][1][1] = 154;   // map mode change - auto
  button_array[1][2][1][1] = 155;   // map mode change - fast
  button_array[2][2][1][1] = 156;   // map mode change - screen
  button_array[3][2][1][1] = 132;   // map up
  button_array[4][2][1][1] = 133;   // map left
  button_array[5][2][1][1] = 134;   // map right
  button_array[6][2][1][1] = 135;   // map down

  // NAV panel
  button_array[1][1][1][1] = 152;   // select distance input mode
  button_array[6][1][2][1] = 140;   // press nav comp '0'
  button_array[5][1][1][1] = 149;   // press nav comp '9'
  button_array[0][1][1][1] = 151;   // select heading input mode
  button_array[4][1][1][1] = 148;   // press nav comp '8' 
  button_array[2][1][1][1] = 150;   // select program input mode
  button_array[3][1][1][1] = 147;   // press nav comp '7' 
  button_array[7][1][2][1] = 153;   // press nav comp enter key
  button_array[2][1][2][1] = 146;   // press nav comp '6'
  button_array[5][1][2][1] = 143;   // press nav comp '3'
  button_array[1][1][2][1] = 145;   // press nav comp '5'
  button_array[4][1][2][1] = 142;   // press nav comp '2'
  button_array[0][1][2][1] = 144;   // press nav comp '4'
  button_array[3][1][2][1] = 141;   // press nav comp '1'

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
  light_array[0][1][2] = true;   //map zoom
  light_array[1][1][2] = true;   //map zoom

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

  // setup animate lights timers *********************
  effects_decay_timer = millis() + DECAY_INTERVAL;

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

  // setup special scanner timer
  scanner_timer = millis() + SCANNER_DELAY;

  // setup animation timers
  animate_buttons_timer = millis() + ANIMATE_BUTTONS_INTERVAL;
  nav_blink_timer = millis() + NAV_BLINK_INTERVAL;

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
    if (nav_mode == NAV_MODE_NORMAL || nav_mode == NAV_MODE_LOCKED) {
      active_course = number;
      switch (nav_disp) {
        case NAV_DISP_HDG:
          set_led_char('h', 0);
          set_led_number(course_array[number][0]%10,3);
          set_led_number(course_array[number][0]%100/10,2);
          set_led_number(course_array[number][0]/100,1);
          break;
        case NAV_DISP_DIST:
          set_led_char('d', 0);
          set_led_number(course_array[number][1]/10,2);
          set_led_number(course_array[number][1]%10,3);
      }
    }
    if (nav_mode == NAV_MODE_TYPE && nav_disp == NAV_DISP_DIST) {
      switch (enter_num_pos) {
        case 0:
          set_led_number(number, enter_num_pos + 2);
          enter_num = 0 + 10 * number;
          enter_num_pos = 1;
          set_led_char('_', enter_num_pos + 2);
        break;
        case 1:
          set_led_number(number, enter_num_pos + 2);
          enter_num = enter_num + number;
          enter_num_pos = -1;
          nav_mode = NAV_MODE_NORMAL;
          nav_disp = NAV_DISP_HDG;
          if(enter_num >= 0 && enter_num <= 99){
            course_array[active_course][1] = enter_num;
            add_data_to_transmit_queue(123, active_course, 0);
          }
          type_number(active_course);
          enter_num = 0;
        break;
      }
    }
    if (nav_mode == NAV_MODE_TYPE && nav_disp == NAV_DISP_HDG) {
      switch (enter_num_pos) {
        case 0:
          set_led_number(number, enter_num_pos + 1);
          enter_num = 0 + 100 * number;
          enter_num_pos = 1;
          set_led_char('_', enter_num_pos + 1);
        break;
        case 1:
          set_led_number(number, enter_num_pos + 1);
          enter_num = enter_num + 10 * number;
          enter_num_pos = 2;
          set_led_char('_', enter_num_pos + 1);
        break;
        case 2:
          set_led_number(number, enter_num_pos + 1);
          enter_num = enter_num + number;
          enter_num_pos = 0;
          nav_disp = NAV_DISP_DIST;
          if(enter_num >= 0 && enter_num <= 360){
            course_array[active_course][0] = enter_num;
            add_data_to_transmit_queue(123, active_course, 0);
          }
          set_led_char('d', 0);
          set_led_char(' ', 1);
          set_led_char('_', 2);
          set_led_char(' ', 3);
          enter_num = course_array[active_course][1];
        break;
      }
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
    if (command_queue[current_command][1] != -1) {
      switch (command_queue[current_command][0]) {
        case 0:
        break;
  
        // simple commands block ************
          case 61:
            // zoom out
            Keyboard.press(KEY_DOWN);
            Keyboard.release(KEY_DOWN);
          break;
          case 60:
            // zoom in
            Keyboard.press(KEY_UP);
            Keyboard.release(KEY_UP);
          break;
          case 62:
            // scan
            scan_counter = 1;
            scanner_timer = millis() + SCANNER_DELAY;
            Keyboard.press(KEY_ENTER);
            Keyboard.release(KEY_ENTER);
          break;
          case 65:
            // select previous
            Keyboard.press(KEY_I);
            Keyboard.release(KEY_I);
          break;
          case 64:
            // select previous
            Keyboard.press(KEY_U);
            Keyboard.release(KEY_U);
          break;
          case 63:
            // select previous
            Keyboard.press(KEY_Y);
            Keyboard.release(KEY_Y);
          break;
                
          case 56:
            // click mouse button
            Mouse.click();
          break;
          
             
    
          // complex command block ************
          case 154:
            // map mode auto
            map_mode = 1;
          break;
          case 155:
            // map mode fast
            map_mode = 2;
          break;
          case 156:
            // map mode screen
            map_mode = 3;
          break;
          case 132:
            // move map up
            switch (map_mode) {
              case 1:  //auto
                Keyboard.press(KEY_W);
              break;
              case 2:   // fast
                enter_command(217, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y - MAP_FAST_DELTA_Y);
                enter_command(211, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y + MAP_FAST_DELTA_Y);
                enter_command(218, 0, 0);
                enter_command(212, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y);
              break;
              case 3:   // screen
                enter_command(217, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y - MAP_SCREEN_DELTA_Y);
                enter_command(211, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y + MAP_SCREEN_DELTA_Y);
                enter_command(218, 0, 0);
                enter_command(212, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y);
              break;
            }
          break;
          case 133:
            // move map left
            switch (map_mode) {
              case 1:  //auto
                Keyboard.press(KEY_A);
              break;
              case 2:   // fast
                enter_command(217, 0, 0);
                enter_command(210, MAP_CENTER_X - MAP_FAST_DELTA_X, MAP_CENTER_Y);
                enter_command(211, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X + MAP_FAST_DELTA_X, MAP_CENTER_Y);
                enter_command(218, 0, 0);
                enter_command(212, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y);
              break;
              case 3:   // screen
                enter_command(217, 0, 0);
                enter_command(210, MAP_CENTER_X - MAP_SCREEN_DELTA_X, MAP_CENTER_Y);
                enter_command(211, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X + MAP_SCREEN_DELTA_X, MAP_CENTER_Y);
                enter_command(218, 0, 0);
                enter_command(212, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y);
              break;
            }
          break;
          case 134:
            // move map right
            switch (map_mode) {
              case 1:  //auto
                Keyboard.press(KEY_D);
              break;
              case 2:   // fast
                enter_command(217, 0, 0);
                enter_command(210, MAP_CENTER_X + MAP_FAST_DELTA_X, MAP_CENTER_Y);
                enter_command(211, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X - MAP_FAST_DELTA_X, MAP_CENTER_Y);
                enter_command(218, 0, 0);
                enter_command(212, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y);
              break;
              case 3:   // screen
                enter_command(217, 0, 0);
                enter_command(210, MAP_CENTER_X + MAP_SCREEN_DELTA_X, MAP_CENTER_Y);
                enter_command(211, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X - MAP_SCREEN_DELTA_X, MAP_CENTER_Y);
                enter_command(218, 0, 0);
                enter_command(212, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y);
              break;
            }
          break;
          case 135:
            // move map down
            switch (map_mode) {
              case 1:  //auto
                Keyboard.press(KEY_S);
              break;
              case 2:   // fast
                enter_command(217, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y + MAP_FAST_DELTA_Y);
                enter_command(211, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y - MAP_FAST_DELTA_Y);
                enter_command(218, 0, 0);
                enter_command(212, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y);
              break;
              case 3:   // screen
                enter_command(217, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y + MAP_SCREEN_DELTA_Y);
                enter_command(211, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y - MAP_SCREEN_DELTA_Y);
                enter_command(218, 0, 0);
                enter_command(212, 0, 0);
                enter_command(218, 0, 0);
                enter_command(210, MAP_CENTER_X, MAP_CENTER_Y);
              break;
            }
          break;

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
          case 150:   // push CHNG button:
            nav_mode = NAV_MODE_TYPE;
            nav_disp = NAV_DISP_HDG;
            enter_num = course_array[active_course][0];
            enter_num_pos = 0;
            set_led_char('h',0);
            set_led_char('_',1);
            set_led_char(' ',2);
            set_led_char(' ',3);
          break;
          case 151:   // push HDG button
            if (nav_mode == NAV_MODE_NORMAL) {
              nav_disp = NAV_DISP_HDG;
              set_led_char('h', 0);
              set_led_number(course_array[active_course][0]%10,3);
              set_led_number(course_array[active_course][0]%100/10,2);
              set_led_number(course_array[active_course][0]/100,1);
            }
          break;
          case 152:   // push DIST button
            if (nav_mode == NAV_MODE_NORMAL) {
              nav_disp = NAV_DISP_DIST;
              set_led_char('d', 0);
              set_led_char(' ', 1);
              set_led_number(course_array[active_course][1]/10,2);
              set_led_number(course_array[active_course][1]%10,3);
            }
          break;
          case 153:   // push ENTER button
            if(nav_mode == NAV_MODE_TYPE && nav_disp == NAV_DISP_DIST){
              if(enter_num >= 0 && enter_num <= 99){
                course_array[active_course][1] = enter_num;
                add_data_to_transmit_queue(123, active_course, 0);
              }
              enter_num_pos = -1;
              nav_mode = NAV_MODE_NORMAL;
              nav_disp = NAV_DISP_HDG;
              type_number(active_course);
              enter_num = 0;
              add_data_to_transmit_queue(123, active_course, 0);
            }
            if(nav_mode == NAV_MODE_TYPE && nav_disp == NAV_DISP_HDG){
              if(enter_num >= 0 && enter_num <= 360){
                course_array[active_course][0] = enter_num;
              }
              enter_num_pos = 0;
              enter_num = course_array[active_course][1];
              nav_disp = NAV_DISP_DIST;
              set_led_char('d', 0);
              set_led_char(' ', 1);
              set_led_char('_', 2);
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
          case 218:
              command_timer = millis() + COMMAND_INTERVAL_SLOW;
          break;
        }
      }
      else {
        switch (command_queue[current_command][0]) {
          case 132:
            if (map_mode == 1) {
              Keyboard.release(KEY_W);
            }
          break; 
          case 133:
            if (map_mode == 1) {
              Keyboard.release(KEY_A);
            }
          break; 
          case 134:
            if (map_mode == 1) {
              Keyboard.release(KEY_D);
            }
          break; 
          case 135:
            if (map_mode == 1) {
              Keyboard.release(KEY_S);
            }
          break; 
        }
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


// ANIMATE EXTERNAL EFFECTS CODE ********************************************************************************
void decay_external_effects() {
  for (int i=0; i<=5; i++) {
    external_effects[i] = external_effects[i] - 100;
    if (external_effects[i] < 0) {
      external_effects[i] = 0;
    }
    if (external_effects[i] > EFFECTS_MAX) {
      external_effects[i] = EFFECTS_MAX;
    }
  }
}

// ANIMATE BLINKING NAV COURSE BUTTON ***********************************************************************
void nav_blink() {
  int i = 0;
  int k = 0;
  if (nav_mode == NAV_MODE_NORMAL) {
    light_array[3][4][1] = false;
    light_array[4][4][1] = false;
    light_array[5][4][1] = false;
    light_array[0][5][1] = false;
    light_array[1][5][1] = false;
    light_array[2][5][1] = false;
    light_array[3][5][1] = false;
    light_array[4][5][1] = false;
    light_array[5][5][1] = false;
    light_array[6][5][1] = false;
    light_array[7][5][1] = false;
    switch (active_course) {
      case 0:
        i = 6;
        k = 5;
      break;
      case 1:
        i = 3;
        k = 5;
      break;
      case 2:
        i = 4;
        k = 5;
      break;
      case 3:
        i = 5;
        k = 5;
      break;
      case 4:
        i = 0;
        k = 5;
      break;
      case 5:
        i = 1;
        k = 5;
      break;
      case 6:
        i = 2;
        k = 5;
      break;
      case 7:
        i = 3;
        k = 4;
      break;
      case 8:
        i = 4;
        k = 4;
      break;
      case 9:
        i = 5;
        k = 4;
      break;
    }
    light_array[i][k][1] = true;
    
    switch (helm_course) {
      case 0:
        i = 6;
        k = 5;
      break;
      case 1:
        i = 3;
        k = 5;
      break;
      case 2:
        i = 4;
        k = 5;
      break;
      case 3:
        i = 5;
        k = 5;
      break;
      case 4:
        i = 0;
        k = 5;
      break;
      case 5:
        i = 1;
        k = 5;
      break;
      case 6:
        i = 2;
        k = 5;
      break;
      case 7:
        i = 3;
        k = 4;
      break;
      case 8:
        i = 4;
        k = 4;
      break;
      case 9:
        i = 5;
        k = 4;
      break;
    }
    if (nav_blink_state) {
      nav_blink_state = false;
    }
    else {
      nav_blink_state = true;
    }
    light_array[i][k][1] = nav_blink_state;
  }  
}


// REFRESH LIGHTS CODE ************************************************************************************
void refresh_lights() {

  // map mode lights
  switch (map_mode) {
    case 1:
      light_array[0][0][2] = true;
      light_array[1][0][2] = false;
      light_array[2][0][2] = false;
    break;
    case 2:
      light_array[0][0][2] = false;
      light_array[1][0][2] = true;
      light_array[2][0][2] = false;
    break;
    case 3:
      light_array[0][0][2] = false;
      light_array[1][0][2] = false;
      light_array[2][0][2] = true;
    break;
  }

  // scanning status
  if (scan_counter > 0) {
    light_array[6][5][2] = true;
  }
  else {
    light_array[6][5][2] = false;
  }

  // sensor power level
  for (int i=0; i<=5; i++) {
    light_array[i][5][2] = false;
  }
  if (power_levels[2] > 5) {
    light_array[0][5][2] = true;
  }
  if (power_levels[2] > 15) {
    light_array[1][5][2] = true;
  }
  if (power_levels[2] > 25) {
    light_array[2][5][2] = true;
  }
  if (power_levels[2] > 35) {
    light_array[3][5][2] = true;
  }
  if (power_levels[2] > 45) {
    light_array[4][5][2] = true;
  }
  if (power_levels[2] > 55) {
    light_array[5][5][2] = true;
  }

  //reset external effects array
  for (int i=2; i<=4; i++) {
    for (int k=0; k<=7; k++) {
      light_array[k][i][2] = false;
    }
  }

  for (int i=0; i<=4; i=i+2) {
    light_array[0][i/2+2][2] = (external_effects[i] > 0);
    light_array[2][i/2+2][2] = (external_effects[i] > 100);
    light_array[4][i/2+2][2] = (external_effects[i] > 200);
    light_array[6][i/2+2][2] = (external_effects[i] > 300);
  }
  for (int i=1; i<=5; i=i+2) {
    light_array[1][i/2+2][2] = (external_effects[i] > 0);
    light_array[3][i/2+2][2] = (external_effects[i] > 100);
    light_array[5][i/2+2][2] = (external_effects[i] > 200);
    light_array[7][i/2+2][2] = (external_effects[i] > 300);
  }


  // navigation panel
  switch (nav_mode) {
    case NAV_MODE_NORMAL:
      light_array[2][4][1] = false;
      break;
    case NAV_MODE_HDG:
      break;
    case NAV_MODE_DIST:
      break;
    case NAV_MODE_TYPE:
      light_array[2][4][1] = true;
      light_array[3][4][1] = true;
      light_array[4][4][1] = true;
      light_array[5][4][1] = true;
      light_array[0][5][1] = true;
      light_array[1][5][1] = true;
      light_array[2][5][1] = true;
      light_array[3][5][1] = true;
      light_array[4][5][1] = true;
      light_array[5][5][1] = true;
      light_array[6][5][1] = true;
      light_array[7][5][1] = true;
      break;
  }

  switch (nav_disp) {
    case  NAV_DISP_HDG:
      light_array[0][4][1] = true;
      light_array[1][4][1] = false;
      break;
    case  NAV_DISP_DIST:
      light_array[0][4][1] = false;
      light_array[1][4][1] = true;
      break;
  }
  
}

// scanner and autoscan code (for science) *******************************************************************
void autoscan() {
  // check scanner state first
  if (scan_counter > 0) {
    if (millis() > scanner_timer) {
      scan_counter++;
      scanner_timer = millis() + SCANNER_DELAY;
      if (scan_counter > 6) {
        scan_counter = 0;
      }
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
    case 123:
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data0);
      if (course_array[data0][0] >= 180) {
        RS_485_TX[dmx_transmit_queue_last][4] = byte(course_array[data0][0] - 180);
        RS_485_TX[dmx_transmit_queue_last][5] = byte(180);
      }
      else {
        RS_485_TX[dmx_transmit_queue_last][4] = byte(0);
        RS_485_TX[dmx_transmit_queue_last][5] = byte(course_array[data0][0]);
      }      
      RS_485_TX[dmx_transmit_queue_last][6] = byte(course_array[data0][1]);
      RS_485_TX[dmx_transmit_queue_last][7] = RS_485_TX_STOP;
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
            if (scan_counter > 0) {
              add_data_to_transmit_queue(133, 2, 1);
            }
            else {
              add_data_to_transmit_queue(133, 2, 0);
            }
            add_data_to_transmit_queue(215, 0, 0);   // return TX token to master controller
          }
        }
        if (RS_485_RX[1] == 124) {
          helm_course = RS_485_RX[2];
        }
      }

      if (data_stream_pos > 2) {
        if (RS_485_RX[1] == 101) {
          switch (RS_485_RX[2]) {
            case 15:
              if (RS_485_RX[3] != 0) {
                external_effects[0] = external_effects[0] + 100;
              }              
            break;
            case 17:
              if (RS_485_RX[3] != 0) {
                external_effects[1] = external_effects[1] + 100;
              }
            break;
            case 21:
              if (RS_485_RX[3] != 0) {
                external_effects[2] = external_effects[2] + 100;
              }
            break;
            case 22:
              if (RS_485_RX[3] != 0) {
                external_effects[2] = external_effects[2] + 100;
              }
            break;
            case 25:
              if (RS_485_RX[3] != 0) {
                external_effects[2] = external_effects[2] + 300;
              }
            break;
            case 26:
              if (RS_485_RX[3] != 0) {
                external_effects[3] = external_effects[3] + 200;
              }
            break;
            case 23:
              if (RS_485_RX[3] != 0) {
                external_effects[4] = external_effects[4] + 100;
              }
            break;
            case 27:
              if (RS_485_RX[3] != 0) {
                external_effects[4] = external_effects[4] + 200;
              }
            break;
            case 24:
              if (RS_485_RX[3] != 0) {
                external_effects[5] = external_effects[5] + 200;
              }
            break;
            default:
            break;
          }
        }
      }

      if (data_stream_pos > 4) {             // receive nav course from another controller
        if (RS_485_RX[1] == 123) {
          course_array[RS_485_RX[2]][0] = RS_485_RX[3] + RS_485_RX[4];
          course_array[RS_485_RX[2]][1] = RS_485_RX[5];
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

  // check scanner state and enter autoscan commands
  autoscan();

  // execute commands from command queue
  if (millis() >= command_timer) {
    execute_command();
    command_timer = millis() + COMMAND_INTERVAL;
  }

  // animate lights
  if (millis() >= effects_decay_timer) {
    decay_external_effects();
    effects_decay_timer = millis() + DECAY_INTERVAL;
  }
  if (millis() >= nav_blink_timer) {
    nav_blink();
    nav_blink_timer = millis() + NAV_BLINK_INTERVAL;
  }

  // refresh lights
  if (millis() >= refresh_light_delay) {
    refresh_lights();
    
    refresh_light_delay = millis() + REFRESH_LIGHT_INTERVAL;
  }
  
}

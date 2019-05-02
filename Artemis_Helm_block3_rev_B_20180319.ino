// Artemis SBS Helm Control Panel (Block III) Code
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
  {5, 6, 7, 7},      // panel 0 "center": 5 - shift serial, 6&7 - button inputs
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
#define COMMAND_INTERVAL 50 // milliseconds in between consecutive keyboard commands to the connected PC

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
#define ANALOG_READ_RATE 50  // delay in milliseconds between warp slider readings
#define ANALOG_DIFFERENCE_THRESHOLD 10  // how different the moving average needs to be from the last change to trigger a command

// HELM constants
#define RUDDER_Y 705
#define RUDDER_X 590
#define RUDDER_CENTER 747
#define RUDDER_SCALE 77
#define RUDDER_L1 730
#define RUDDER_L2 511
#define RUDDER_L3 256
#define RUDDER_R1 770
#define RUDDER_R2 1023
#define RUDDER_R3 1278
#define SLIDER_N3 256
#define SLIDER_N2 511
#define SLIDER_N1 730
#define SLIDER_P1 770
#define SLIDER_P2 1023
#define SLIDER_P3 1278
#define SLIDER_MID 750
#define SLIDER_SCALE 50
#define IMPULSE_X 74
#define IMPULSE_CENTER 756
#define IMPULSE_BOTTOM 678
#define IMPULSE_SCALE 46
#define WARP_X 20
#define WARP_BOTTOM 678
#define WARP_1 283
#define WARP_2 685
#define WARP_3 1081
#define WARP_4 1463
#define WARP_0_Y 671
#define WARP_1_Y 626
#define WARP_2_Y 595
#define WARP_3_Y 543
#define WARP_4_Y 505
#define MANUEVER_X 590.0
#define MANUEVER_Y 361.0
#define MANUEVER_RADIUS 250.0
#define JUMP_HDG_X 234
#define JUMP_HDG_Y 434
#define JUMP_DIST_X 234
#define JUMP_DIST_Y 462
#define JUMP_INITIATE_X 157
#define JUMP_INITIATE_Y 516
#define JUMP_CONFIRM_X 158
#define JUMP_CONFIRM_Y 435
#define JUMP_CANCEL_X 158
#define JUMP_CANCEL_Y 497
#define JUMP_EMERG_F_X 30
#define JUMP_EMERG_F_Y 389
#define JUMP_EMERG_R_X 30
#define JUMP_EMERG_R_Y 444

// console-specific (HELM) variables
#define MAX_TRANSMIT_QUEUE 30
int dmx_transmit_queue_pos = 0;
int dmx_transmit_queue_last = 0;
#define MY_CONTROLLER_ADDRESS 10
byte RS_485_TX[MAX_TRANSMIT_QUEUE][10];   // variable to store bytes for transmittal on the RS-485 bus
int RS_485_RX[10];  // variable to store received bytes
bool data_stream_on = false; // variable to store in currently transmitting data state
int data_stream_pos = 0;  // varaible to store where in the block of data we are currently transmitting/receiving
bool transmit_enable = false; // variable to turn ability to transmit on and off (token)
#define RS_485_TX_START 255 // symbol to signify start of transmitting stream
#define RS_485_TX_STOP 254 // symbol to stop transmitting stream

bool transmit_data = false;
bool control_enable = true;
int power_levels[8] = {20, 20, 20, 20, 20, 20, 20, 20};
bool shields_on = false;
bool front_shield_low = false;
bool rear_shield_low = false;
bool warp_mode_on = true;
int warp_state = 0;
bool impulse_on = false;
bool rudder_on = false;
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
#define NAV_MODE_NORMAL 0
#define NAV_MODE_HDG 1
#define NAV_MODE_DIST 2
#define NAV_MODE_LOCKED 3
int enter_num = 0;
int enter_num_pos = -1;
bool jump_on = false;
bool dock_state_on = false;
bool tractor_state_on = false;
bool reverse_state_on = false;
int slider_state[2][3] = {
  {512, 0, 512},
  {512, 0, 512}
  };
int slider_reads = 0;
bool slider_command_on[3] = {false, false, false};

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
  button_array[1][0][1][1] = 10;    // zoom -
  button_array[2][0][1][1] = 11;    // zoom +
  button_array[5][0][1][1] = 22;    // req dock
  button_array[7][0][1][1] = 23;    // pitch up
  button_array[0][0][2][1] = 27;    // cancel jump
  button_array[1][0][2][1] = 26;    // confirm jump
  button_array[2][0][2][1] = 25;    // initiate jump
  button_array[4][0][2][1] = 29;    // emergency jump reverse
  button_array[5][0][2][1] = 28;    // emergency jump forward
  button_array[7][0][2][1] = 24;    // pitch down
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

  // complex "high-level" commands second
  button_array[0][0][1][1] = 120;   // request adjust impulse power level
  button_array[3][0][1][1] = 121;   // toggle warp system mode
  button_array[4][0][1][1] = 122;   // request adjust warp power level
  button_array[6][0][1][1] = 123;   // request adjust manuever power level
  button_array[3][0][2][1] = 124;   // enter jump course
  button_array[6][0][2][1] = 125;   // enter maneuver course
  button_array[0][2][1][1] = 152;   // select distance input mode
  button_array[1][2][1][1] = 140;   // press nav comp '0'
  button_array[2][2][1][1] = 149;   // press nav comp '9'
  button_array[3][2][1][1] = 151;   // select heading input mode
  button_array[4][2][1][1] = 148;   // press nav comp '8' 
  button_array[5][2][1][1] = 150;   // select program input mode
  button_array[6][2][1][1] = 147;   // press nav comp '7' 
  button_array[0][2][2][1] = 153;   // press nav comp enter key
  button_array[1][2][2][1] = 146;   // press nav comp '6'
  button_array[2][2][2][1] = 143;   // press nav comp '3'
  button_array[3][2][2][1] = 145;   // press nav comp '5'
  button_array[4][2][2][1] = 142;   // press nav comp '2'
  button_array[5][2][2][1] = 144;   // press nav comp '4'
  button_array[6][2][2][1] = 141;   // press nav comp '1'
  // end button code setup

  // setup sliders *****************************
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


void set_led_number(int number, int pos) {
  if (pos < 0) {
    pos = 0;
  }
  if (pos > 5) {
    pos = 5;
  }
          
  switch (number){
    case 0:
      light_array[0][pos][2] = true; // UPPER LEFT segment
      light_array[1][pos][2] = true; // TOP segment
      light_array[2][pos][2] = true; // UPPER RIGHT segment
      light_array[3][pos][2] = false; // MIDDLE segment
      light_array[4][pos][2] = true; // LOWER LEFT segment
      light_array[5][pos][2] = true; // BOTTOM segment
      light_array[6][pos][2] = true; // LOWER RIGHT segment
    break;
    case 1:
      light_array[0][pos][2] = false; // UPPER LEFT segment
      light_array[1][pos][2] = false; // TOP segment
      light_array[2][pos][2] = true; // UPPER RIGHT segment
      light_array[3][pos][2] = false; // MIDDLE segment
      light_array[4][pos][2] = false; // LOWER LEFT segment
      light_array[5][pos][2] = false; // BOTTOM segment
      light_array[6][pos][2] = true; // LOWER RIGHT segment
    break;
    case 2:
      light_array[0][pos][2] = false; // UPPER LEFT segment
      light_array[1][pos][2] = true; // TOP segment
      light_array[2][pos][2] = true; // UPPER RIGHT segment
      light_array[3][pos][2] = true; // MIDDLE segment
      light_array[4][pos][2] = true; // LOWER LEFT segment
      light_array[5][pos][2] = true; // BOTTOM segment
      light_array[6][pos][2] = false; // LOWER RIGHT segment
    break;
    case 3:
      light_array[0][pos][2] = false; // UPPER LEFT segment
      light_array[1][pos][2] = true; // TOP segment
      light_array[2][pos][2] = true; // UPPER RIGHT segment
      light_array[3][pos][2] = true; // MIDDLE segment
      light_array[4][pos][2] = false; // LOWER LEFT segment
      light_array[5][pos][2] = true; // BOTTOM segment
      light_array[6][pos][2] = true; // LOWER RIGHT segment
    break;
    case 4:
      light_array[0][pos][2] = true; // UPPER LEFT segment
      light_array[1][pos][2] = false; // TOP segment
      light_array[2][pos][2] = true; // UPPER RIGHT segment
      light_array[3][pos][2] = true; // MIDDLE segment
      light_array[4][pos][2] = false; // LOWER LEFT segment
      light_array[5][pos][2] = false; // BOTTOM segment
      light_array[6][pos][2] = true; // LOWER RIGHT segment
    break;
    case 5:
      light_array[0][pos][2] = true; // UPPER LEFT segment
      light_array[1][pos][2] = true; // TOP segment
      light_array[2][pos][2] = false; // UPPER RIGHT segment
      light_array[3][pos][2] = true; // MIDDLE segment
      light_array[4][pos][2] = false; // LOWER LEFT segment
      light_array[5][pos][2] = true; // BOTTOM segment
      light_array[6][pos][2] = true; // LOWER RIGHT segment
    break;
    case 6:
      light_array[0][pos][2] = true; // UPPER LEFT segment
      light_array[1][pos][2] = true; // TOP segment
      light_array[2][pos][2] = false; // UPPER RIGHT segment
      light_array[3][pos][2] = true; // MIDDLE segment
      light_array[4][pos][2] = true; // LOWER LEFT segment
      light_array[5][pos][2] = true; // BOTTOM segment
      light_array[6][pos][2] = true; // LOWER RIGHT segment
    break;
    case 7:
      light_array[0][pos][2] = false; // UPPER LEFT segment
      light_array[1][pos][2] = true; // TOP segment
      light_array[2][pos][2] = true; // UPPER RIGHT segment
      light_array[3][pos][2] = false; // MIDDLE segment
      light_array[4][pos][2] = false; // LOWER LEFT segment
      light_array[5][pos][2] = false; // BOTTOM segment
      light_array[6][pos][2] = true; // LOWER RIGHT segment
    break;
    case 8:
      light_array[0][pos][2] = true; // UPPER LEFT segment
      light_array[1][pos][2] = true; // TOP segment
      light_array[2][pos][2] = true; // UPPER RIGHT segment
      light_array[3][pos][2] = true; // MIDDLE segment
      light_array[4][pos][2] = true; // LOWER LEFT segment
      light_array[5][pos][2] = true; // BOTTOM segment
      light_array[6][pos][2] = true; // LOWER RIGHT segment
    break;
    case 9:
      light_array[0][pos][2] = true; // UPPER LEFT segment
      light_array[1][pos][2] = true; // TOP segment
      light_array[2][pos][2] = true; // UPPER RIGHT segment
      light_array[3][pos][2] = true; // MIDDLE segment
      light_array[4][pos][2] = false; // LOWER LEFT segment
      light_array[5][pos][2] = true; // BOTTOM segment
      light_array[6][pos][2] = true; // LOWER RIGHT segment
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
      light_array[0][pos][2] = true; // UPPER LEFT segment
      light_array[1][pos][2] = true; // TOP segment
      light_array[2][pos][2] = false; // UPPER RIGHT segment
      light_array[3][pos][2] = false; // MIDDLE segment
      light_array[4][pos][2] = true; // LOWER LEFT segment
      light_array[5][pos][2] = true; // BOTTOM segment
      light_array[6][pos][2] = false; // LOWER RIGHT segment
    break;
    case '_':
      light_array[0][pos][2] = false; // UPPER LEFT segment
      light_array[1][pos][2] = false; // TOP segment
      light_array[2][pos][2] = false; // UPPER RIGHT segment
      light_array[3][pos][2] = false; // MIDDLE segment
      light_array[4][pos][2] = false; // LOWER LEFT segment
      light_array[5][pos][2] = true; // BOTTOM segment
      light_array[6][pos][2] = false; // LOWER RIGHT segment
    break;
    case '-':
      light_array[0][pos][2] = false; // UPPER LEFT segment
      light_array[1][pos][2] = false; // TOP segment
      light_array[2][pos][2] = false; // UPPER RIGHT segment
      light_array[3][pos][2] = true; // MIDDLE segment
      light_array[4][pos][2] = false; // LOWER LEFT segment
      light_array[5][pos][2] = false; // BOTTOM segment
      light_array[6][pos][2] = false; // LOWER RIGHT segment
    break;
    case ' ':
      light_array[0][pos][2] = false; // UPPER LEFT segment
      light_array[1][pos][2] = false; // TOP segment
      light_array[2][pos][2] = false; // UPPER RIGHT segment
      light_array[3][pos][2] = false; // MIDDLE segment
      light_array[4][pos][2] = false; // LOWER LEFT segment
      light_array[5][pos][2] = false; // BOTTOM segment
      light_array[6][pos][2] = false; // LOWER RIGHT segment
    break;
  }
}   // end code to set numbers on display ************************************************************************


void type_number(int number) { 
    if (nav_mode == NAV_MODE_NORMAL || nav_mode == NAV_MODE_LOCKED) {
      active_course = number;
      set_led_number(number,3);
      set_led_number(course_array[number][0]%10,2);
      set_led_number(course_array[number][0]%100/10,1);
      set_led_number(course_array[number][0]/100,0);
      set_led_number(course_array[number][1]/10,4);
      set_led_number(course_array[number][1]%10,5);
    }
    if (enter_num_pos != -1) {
      switch (enter_num_pos) {
        case 0:
          set_led_number(number, enter_num_pos);
          enter_num = enter_num + 100 * number;
          enter_num_pos = 1;
          set_led_char('_', enter_num_pos);
        break;
        case 1:
          set_led_number(number, enter_num_pos);
          enter_num = enter_num + 10 * number;
          enter_num_pos = 2;
          set_led_char('_', enter_num_pos);
        break;
        case 2:
          set_led_number(number, enter_num_pos);
          enter_num = enter_num + number;
          enter_num_pos = -1;
          nav_mode = NAV_MODE_NORMAL;
          if(enter_num >= 0 && enter_num <= 360){
            course_array[active_course][0] = enter_num;
          }
          type_number(active_course);
        break;
        case 4:
          set_led_number(number, enter_num_pos);
          enter_num = enter_num + 10 * number;
          enter_num_pos = 5;
          set_led_char('_', enter_num_pos);
        break;
        case 5:
          set_led_number(number, enter_num_pos);
          enter_num = enter_num + number;
          enter_num_pos = -1;
          nav_mode = NAV_MODE_NORMAL;
          if(enter_num >= 0 && enter_num <= 99){
            course_array[active_course][1] = enter_num;
          }
          type_number(active_course);
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
    Serial.println(current_command);
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
      case 22:
        Keyboard.press(KEY_R);
        Keyboard.release(KEY_R);
      break;
      case 23:
        Keyboard.press(KEY_INSERT);
        Keyboard.release(KEY_INSERT);
      break;
      case 24:
        Keyboard.press(KEY_DELETE);
        Keyboard.release(KEY_DELETE);
      break;
      case 27:
        if (!warp_mode_on) {
          // click on jump cancel
          jump_on = false; 
          enter_command(210, JUMP_CANCEL_X, JUMP_CANCEL_Y);
          enter_command(211, 0, 0);
          enter_command(212, 0, 0); 
        }
      break;
      case 26:
        if (!warp_mode_on) {
          // click on jump confirm
          jump_on = false; 
          enter_command(210, JUMP_CONFIRM_X, JUMP_CONFIRM_Y);
          enter_command(211, 0, 0);
          enter_command(212, 0, 0);  
        }
      break;
      case 25:
        if (!warp_mode_on) {
          // click on initiate jump
          jump_on = true;
          enter_command(210, JUMP_INITIATE_X, JUMP_INITIATE_Y);
          enter_command(211, 0, 0);
          enter_command(212, 0, 0);   
        }
      break;
      case 29:
        if (!warp_mode_on) {
          // click on emergency jump reverse
          enter_command(210, JUMP_EMERG_R_X, JUMP_EMERG_R_Y);
          enter_command(211, 0, 0);
          enter_command(212, 0, 0); 
        }
      break;
      case 28:
      if (!warp_mode_on) {
          // click on emergency jump forward
          enter_command(210, JUMP_EMERG_F_X, JUMP_EMERG_F_Y);
          enter_command(211, 0, 0);
          enter_command(212, 0, 0); 
        }
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

      // complex command block ************
      case 121:
        if (!warp_mode_on) {
          warp_mode_on = true;
          jump_on = false;
          light_array[0][1][0] = true;
          light_array[1][1][0] = false;
        }
        else {
          warp_mode_on = false;
          warp_state = 0;
          light_array[0][1][0] = false;
          light_array[1][1][0] = true;
        }
      break;
      case 124:
        // enter jump course
        if (!warp_mode_on) {
          set_j_course(course_array[active_course][0], course_array[active_course][1]);
        }        
      break;
      case 125:
        // enter manuever course
        if (nav_mode == NAV_MODE_NORMAL) {
          set_m_course(course_array[active_course][0]);
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
      case 151:   // push HDG button
        nav_mode = NAV_MODE_HDG;
        enter_num = 0;
        enter_num_pos = 0;
        set_led_char('_',0);
        set_led_char(' ',1);
        set_led_char(' ',2);
      break;
      case 152:   // push DIST button
        nav_mode = NAV_MODE_DIST;
        enter_num = 0;
        enter_num_pos = 4;
        set_led_char('_',4);
        set_led_char(' ',5);
      break;
      case 153:   // push ENTER button
        if(nav_mode == NAV_MODE_HDG){
          if(enter_num >= 0 && enter_num <= 360){
            course_array[active_course][0] = enter_num;
          }
          enter_num_pos = -1;
          nav_mode = NAV_MODE_NORMAL;
          type_number(active_course);
          enter_num = 0;
        }
        if(nav_mode == NAV_MODE_DIST){
          if(enter_num >= 0 && enter_num <= 99){
            course_array[active_course][1] = enter_num;
          }
          enter_num_pos = -1;
          nav_mode = NAV_MODE_NORMAL;
          type_number(active_course);
          enter_num = 0;
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
      case 213:   // reverse on
        if (!reverse_state_on) {
          Keyboard.press(KEY_ESC);
          Keyboard.release(KEY_ESC);
          reverse_state_on = true;
        }
      break;
      case 214:   // reverse off
        if (reverse_state_on) {
          Keyboard.press(KEY_ESC);
          Keyboard.release(KEY_ESC);
          reverse_state_on = false;
        }
      break;
      case 215:
        slider_command_on[command_queue[current_command][1]] = false;
      break;
      case 220:     // type a number
        switch (command_queue[current_command][1]) {
          case 0:
            Keyboard.press(KEY_0);
            Keyboard.release(KEY_0);
          break;
          case 1:
            Keyboard.press(KEY_1);
            Keyboard.release(KEY_1);
          break;
          case 2:
            Keyboard.press(KEY_2);
            Keyboard.release(KEY_2);
          break;
          case 3:
            Keyboard.press(KEY_3);
            Keyboard.release(KEY_3);
          break;
          case 4:
            Keyboard.press(KEY_4);
            Keyboard.release(KEY_4);
          break;
          case 5:
            Keyboard.press(KEY_5);
            Keyboard.release(KEY_5);
          break;
          case 6:
            Keyboard.press(KEY_6);
            Keyboard.release(KEY_6);
          break;
          case 7:
            Keyboard.press(KEY_7);
            Keyboard.release(KEY_7);
          break;
          case 8:
            Keyboard.press(KEY_8);
            Keyboard.release(KEY_8);
          break;
          case 9:
            Keyboard.press(KEY_9);
            Keyboard.release(KEY_9);
          break;
        }
      break;
      case 230:
        Keyboard.press(KEY_BACKSPACE);
        Keyboard.release(KEY_BACKSPACE);
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

// SET MANUEVER COURSE CODE ***************************************************************
void set_m_course (int heading) {
  int x = 0;
  int y = 0;
  float hdg = heading / 180.0 * 3.14;

  x = MANUEVER_X + sin(hdg) * MANUEVER_RADIUS;
  y = MANUEVER_Y - cos(hdg) * MANUEVER_RADIUS;

  /*Serial.print("course: ");
  Serial.print(hdg);
  Serial.print(" x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.println(y);*/

  enter_command(210, x, y);
  enter_command(211, 0, 0);
  enter_command(212, 0, 0);  
}

// SET JUMP COURSE CODE ***************************************************************
void set_j_course (int heading, int distance) {
  enter_command(210, JUMP_HDG_X, JUMP_HDG_Y);
  enter_command(211, 0, 0);
  enter_command(212, 0, 0);
  enter_command(230, 0, 0);
  enter_command(230, 0, 0);
  enter_command(230, 0, 0);
  enter_command(220, heading/100, 0);
  enter_command(220, heading%100/10, 0);
  enter_command(220, heading%10, 0);

  enter_command(210, JUMP_DIST_X, JUMP_DIST_Y);
  enter_command(211, 0, 0);
  enter_command(212, 0, 0);
  enter_command(230, 0, 0);
  enter_command(230, 0, 0);
  enter_command(220, distance%100/10, 0);
  enter_command(220, distance%10, 0); 
}

// SET SLIDER CODE ************************************************************************
void set_slider (int pos, int state) {
  int x = 0;
  int y = 0;
  switch (pos) {
    case 0:   // impulse slider -- click on speed bar; toggle reverse if under threshold value
        if (warp_mode_on) {
          x = IMPULSE_X;
        }
        else {
          x = WARP_X;
        }
        impulse_on = false;
        light_array[2][0][0] = false;
        light_array[3][0][0] = false;
        light_array[4][0][0] = false;
        light_array[5][0][0] = false;
        light_array[6][0][0] = false;
        light_array[7][0][0] = false;
      if (state > SLIDER_N1) {
        enter_command(214, 0, 0);  // turn impulse reverse off
        y = IMPULSE_BOTTOM;
      }
      if (state > SLIDER_P1) {
        impulse_on = true;
        light_array[4][0][0] = true;
        if (state > SLIDER_P2) {
          light_array[3][0][0] = true;
        }
        if (state > SLIDER_P3) {
          light_array[2][0][0] = true;
        }
        y = IMPULSE_BOTTOM - (state - SLIDER_MID) * 10 / IMPULSE_SCALE;
      }
      if (state < SLIDER_N1) {
        enter_command(213, 0, 0);  // turn impulse reverse on
        impulse_on = true;
        light_array[5][0][0] = true;
        if (state < SLIDER_N2) {
          light_array[6][0][0] = true;
        }
        if (state < SLIDER_N3) {
          light_array[7][0][0] = true;
        }
        y = IMPULSE_BOTTOM - (SLIDER_MID - state) * 10 / IMPULSE_SCALE;
      }
      if (IMPULSE_BOTTOM - y > 20) {
        enter_command(210, x, y + 2);
      }
      else {
        enter_command(210, x, y - 10);
      }
      enter_command(211, 0, 0);
      enter_command(210, x, y);
      enter_command(212, 0, 0);
      enter_command(215, 0, 0);  // release slider for next impulse change    
    break;
    case 1:   // warp slider -- click on speed bar
      light_array[4][1][0] = false;
      light_array[5][1][0] = false;
      light_array[6][1][0] = false;
      light_array[7][1][0] = false;
      if (!reverse_state_on && warp_mode_on) {
        x = WARP_X;        
        if (state < WARP_1) {
          y = WARP_0_Y;
          warp_state = 0;
        }
        if (state > WARP_1) {
          y = WARP_1_Y;
          warp_state = 1;
          light_array[7][1][0] = true;
        }
        if (state > WARP_2) {
          y = WARP_2_Y;
          warp_state = 2;
          light_array[6][1][0] = true;
        }
        if (state > WARP_3) {
          y = WARP_3_Y;
          warp_state = 3;
          light_array[5][1][0] = true;
        }
        if (state > WARP_4) {
          y = WARP_4_Y;
          warp_state = 4;
          enter_command(210, x, y + 10);
          enter_command(211, 0, 0);
          light_array[4][1][0] = true;
        }
        enter_command(210, x, y);
        enter_command(211, 0, 0);
        enter_command(212, 0, 0);
      }      
      enter_command(215, 1, 0);  // release slider for next warp change
    break;
    case 2:   // manuever slider -- click on rudder bar
      rudder_on = false;
      light_array[0][2][0] = false;
      light_array[1][2][0] = false;
      light_array[2][2][0] = false;
      light_array[3][2][0] = false;
      light_array[4][2][0] = false;
      light_array[5][2][0] = false;
      if (state > RUDDER_R1) {
        rudder_on = true;
        light_array[3][2][0] = true;
      }
      if (state > RUDDER_R2) {
        light_array[4][2][0] = true;
      }
      if (state > RUDDER_R3) {
        light_array[5][2][0] = true;
      }
      if (state < RUDDER_L1) {
        rudder_on = true;
        light_array[2][2][0] = true;
      }
      if (state < RUDDER_L2) {
        light_array[1][2][0] = true;
      }
      if (state < RUDDER_L3) {
        light_array[0][2][0] = true;
      }
      x = RUDDER_X + (state - SLIDER_MID) * 10 / RUDDER_SCALE;
      y = RUDDER_Y;
      if (state > SLIDER_MID) {
        enter_command(210, x - 5, y);
      }
      else {
        enter_command(210, x + 5, y);
      }
      enter_command(211, 0, 0);
      enter_command(210, x, y);
      enter_command(212, 0, 0);
      enter_command(215, 2, 0);  // release slider for next rudder change      
    break;
  }
}

// READ ANALOG CONTROLS CODE ************************************************************************************
void read_analog() {
  slider_state[0][0] = slider_state[0][0] + analogRead(14);
  slider_state[0][1] = slider_state[0][1] + analogRead(15);
  slider_state[0][2] = slider_state[0][2] + analogRead(16);
  slider_reads++;
  if (slider_reads == 3) {
    for(int i=0; i<=2; i++) {
      slider_state[0][i] = slider_state[0][i]/3;
      if(abs(slider_state[0][i] - slider_state[1][i]) > ANALOG_DIFFERENCE_THRESHOLD && !slider_command_on[i]) {
        slider_state[1][i] = slider_state[0][i];
        set_slider(i, slider_state[1][i]);
        slider_command_on[i] = true;
      }
    }
    slider_reads = 0;
    /*Serial.print("Impulse ");
    Serial.println(slider_state[0][0]);
    Serial.print("Warp ");
    Serial.println(slider_state[0][1]);
    Serial.print("Rudder ");
    Serial.println(slider_state[0][2]);*/
  }
}

// REFRESH LIGHTS CODE ************************************************************************************
void refresh_lights() {
  if (warp_mode_on) {
    light_array[0][1][0] = true;
    light_array[1][1][0] = false;
  }
  else {
    light_array[0][1][0] = false;
    light_array[1][1][0] = true;
  }
  if (jump_on) {
    light_array[2][1][0] = true;
  }
  else {
    light_array[2][1][0] = false;
  }
  // reverse, dock, tractor
  if (reverse_state_on && warp_mode_on) {
    light_array[3][1][0] = true;
  }
  else {
    light_array[3][1][0] = false;
  }
  if (tractor_state_on) {
    light_array[6][2][0] = true;
  }
  else {
    light_array[6][2][0] = false;
  }
  if (dock_state_on) {
    light_array[7][2][0] = true;
  }
  else {
    light_array[7][2][0] = false;
  }

  for (int i=0; i<6; i++) {
    for (int k=3; k<6; k++) {
      light_array[i][k][0] = false;
    }
  }
  
  // manuever power level
  if (power_levels[3] > 5) {
    light_array[0][3][0] = true;
  }
  if (power_levels[3] > 15) {
    light_array[1][3][0] = true;
  }
  if (power_levels[3] > 25) {
    light_array[2][3][0] = true;
  }
  if (power_levels[3] > 35) {
    light_array[3][3][0] = true;
  }
  if (power_levels[3] > 45) {
    light_array[4][3][0] = true;
  }
  if (power_levels[3] > 55) {
    light_array[5][3][0] = true;
  }
  
  // impulse power level
  if (power_levels[4] > 5) {
    light_array[0][5][0] = true;
  }
  if (power_levels[4] > 15) {
    light_array[1][5][0] = true;
  }
  if (power_levels[4] > 25) {
    light_array[2][5][0] = true;
  }
  if (power_levels[4] > 35) {
    light_array[3][5][0] = true;
  }
  if (power_levels[4] > 45) {
    light_array[4][5][0] = true;
  }
  if (power_levels[4] > 55) {
    light_array[5][5][0] = true;
  }

  // warp power level
  if (power_levels[5] > 5) {
    light_array[0][4][0] = true;
  }
  if (power_levels[5] > 15) {
    light_array[1][4][0] = true;
  }
  if (power_levels[5] > 25) {
    light_array[2][4][0] = true;
  }
  if (power_levels[5] > 35) {
    light_array[3][4][0] = true;
  }
  if (power_levels[5] > 45) {
    light_array[4][4][0] = true;
  }
  if (power_levels[5] > 55) {
    light_array[5][4][0] = true;
  }

  // shields status
  if (shields_on) {
    light_array[0][0][1] = true;
    light_array[0][1][1] = true;
    light_array[0][2][1] = true;
    light_array[0][3][1] = true;
    light_array[1][0][1] = false;
    light_array[1][1][1] = false;
    light_array[1][2][1] = false;
    light_array[1][3][1] = false;
  }
  else {
    light_array[0][0][1] = false;
    light_array[0][1][1] = false;
    light_array[0][2][1] = false;
    light_array[0][3][1] = false;
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
  else {
    light_array[2][0][1] = false;
    light_array[2][1][1] = false;
    light_array[2][2][1] = false;
    light_array[2][3][1] = false;
  }
  if (rear_shield_low) {
    light_array[3][0][1] = true;
    light_array[3][1][1] = true;
    light_array[3][2][1] = true;
    light_array[3][3][1] = true;
  }
  else {
    light_array[3][0][1] = false;
    light_array[3][1][1] = false;
    light_array[3][2][1] = false;
    light_array[3][3][1] = false;
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
            if (jump_on || warp_state != 0) {
              add_data_to_transmit_queue(133, 5, 1);
            }
            else {
              add_data_to_transmit_queue(133, 5, 0);            
            }
            if (rudder_on) {
              add_data_to_transmit_queue(133, 3, 1);
            }
            else {
              add_data_to_transmit_queue(133, 3, 0);            
            }
            if (impulse_on) {
              add_data_to_transmit_queue(133, 4, 1);
            }
            else {
              add_data_to_transmit_queue(133, 4, 0);            
            }
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
            case 17:
              if (RS_485_RX[3] == 1) {
                tractor_state_on = true;
              }
              else {
                tractor_state_on = false;
              }
            break;
            case 18:
              if (RS_485_RX[3] == 1) {
                dock_state_on = true;
              }
              else {
                dock_state_on = false;
              }
            break;
            case 19:
              if (RS_485_RX[3] == 1) {
                reverse_state_on = true;
              }
              else {
                reverse_state_on = false;
              }
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

// Artemis SBS Controller Code
// ACP3 HELM Controller Prototype with DMX connection to Viewscreen

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

// HELM constants
#define RUDDER_Y 705
#define RUDDER_X 590
#define RUDDER_CENTER 512
#define RUDDER_SCALE -522
#define RUDDER_L1 462
#define RUDDER_R1 562
#define RUDDER_MIN 511
#define RUDDER_MAX 671
#define PITCH_DEADZONE 50
#define SLIDER_N3 85
#define SLIDER_N2 255
#define SLIDER_N1 425
#define SLIDER_P1 595
#define SLIDER_P2 765
#define SLIDER_P3 935
#define SLIDER_MID 512
#define SLIDER_SCALE 40
#define IMPULSE_X 74
#define IMPULSE_CENTER 512
#define IMPULSE_BOTTOM 680
#define IMPULSE_SCALE 296
#define IMPULSE_SCALE2 592
#define WARP_X 20
#define WARP_BOTTOM 678
#define WARP_1 205
#define WARP_2 410
#define WARP_3 614
#define WARP_4 819
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
#define MY_CONTROLLER_ADDRESS 10          // HELM address (does not apply to DMX/MCP variant
byte RS_485_TX[MAX_TRANSMIT_QUEUE][10];   // variable to store bytes for transmittal on the RS-485 bus
int RS_485_RX[10];  // variable to store received bytes
bool data_stream_on = false; // variable to store in currently transmitting data state
int data_stream_pos = 0;  // varaible to store where in the block of data we are currently transmitting/receiving
bool transmit_enable = true; // variable to turn ability to transmit on and off (token)
#define RS_485_TX_START 255 // symbol to signify start of transmitting stream
#define RS_485_TX_STOP 254 // symbol to stop transmitting stream

bool transmit_data = false;
bool control_enable = true;
int screen_mode = 2;   // start assuming that screen is view forward
bool CAM_mode = false;
int power_levels[8] = {20, 20, 20, 20, 20, 20, 20, 20};
bool shields_on = false;
bool front_shield_low = false;
bool rear_shield_low = false;
bool invert_pitch = false;     // start with the maneuver joystick in regular pitch mode
int pitch_mode = 0;           // 0 = neutral, 1 = up, -1 = down
bool impulse_mode_dual = true;  // start with impulse slider providing forward/reverse functionality
bool warp_mode_on = true;
int warp_state = 0;
int jump_state = 0;           // 0 = ready to enter course, 1 = course entered, 2 = jump initiated
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
#define NAV_MODE_TYPE 4
int nav_disp = 0;
#define NAV_DISP_HDG 0
#define NAV_DISP_DIST 1
int enter_num = 0;
int enter_num_pos = -1;
bool jump_on = false;
bool dock_state_on = false;
bool tractor_state_on = false;
bool reverse_state_on = false;
int slider_state[2][4] = {
  {0, 0, 0, 0},
  {512, 0, 512, 512}
  };
int slider_reads = 0;
bool slider_command_on[4] = {false, false, false, false};
unsigned long animate_shields_timer = 0;   // variable to store the time between shield animation frames
#define ANIMATE_SHIELDS_INTERVAL 150  // milliseconds between shield animation frames
int damage_state = 0;
int ship_hit = 0;


// DMX variables and definitions

volatile uint8_t  DmxRxField[71]; //array of DMX vals (raw)
volatile uint16_t DmxAddress;    //start address
volatile uint16_t BytesRead = 0;    //bytes read
volatile uint16_t Errors = 0;    //Frame errors
     
enum {DMX_IDLE, BREAK, STARTB, STARTADR}; //DMX states
     
volatile uint8_t gDmxState;

int dmx_array[71][2] = {
  {0,0}, // 0
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0}, // 10
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0}, // 20
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0}, // 30
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0}, // 40
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0}, // 50
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0}, // 60
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0},
  {0,0} // 70
};

#define CONTROLLER_MAX_POLL 4
bool controller_connected[11] = {false, false, false, false, false, false, false, false, false, false, false};
int controller_address[11] = {100, 20, 30, 40, 50, 20, 30, 40, 50, 20, 30};
int controller_poll_position = 0;
unsigned long RS_485_delay_time = 0;   // variable to store the time between transmissions
#define RS_485_TIMEOUT 50  // timeout time in milliseconds for master controller to wait for return signals from other controllers

unsigned long DMX_check_time = 0;   // variable to store the time between checks for DMX changes
#define DMX_CHECK_INTERVAL 100      // time between checks for DMX changes
#define DMX_CHECK_QUANTITY 5      // number of DMX cues to be read during each check cycle
int DMX_refresh_count = 0;          // current place in the DMX array t o be checked and transmitted to connected controllers


// SETUP CODE ***********************************************************************************************


void setup() {
  Mouse.screenSize(1280, 720);   // setup mouse commands for 1280 by 720 resolution screen

  // setup DMX reader **************************************************************************************
  Serial2.begin(250000, SERIAL_8N2); //Enable serial reception with a 250k rate
  gDmxState= DMX_IDLE; // initial state
  DmxAddress = 1; // The desired DMX Start Address

  attachInterruptVector(IRQ_UART1_STATUS, DMX_read);

  // setup RS-485 bus **************************************************************************************
  Serial1.begin(9600); //Enable serial TX/RX
  Serial1.transmitterEnable(13);  // Use pin 13 to automatically enable a RS-485 transceiver chip. This pin outputs a logic HIGH when transmitting data. It goes LOW after the last stop bit, to allow reception when not transmitting. 

  RS_485_delay_time = millis() + RS_485_TIMEOUT;
  DMX_check_time = millis() + DMX_CHECK_INTERVAL;

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
  button_array[0][2][1][1] = 10;    // zoom -
  button_array[1][2][1][1] = 11;    // zoom +
  button_array[6][2][1][1] = 22;    // req dock
  button_array[3][2][2][1] = 27;    // cancel jump
  button_array[2][2][2][1] = 26;    // confirm jump
  button_array[1][2][2][1] = 25;    // initiate jump
  button_array[5][2][2][1] = 29;    // emergency jump reverse
  button_array[4][2][2][1] = 28;    // emergency jump forward
  button_array[3][2][1][1] = 118;    // impulse reverse
  
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
  button_array[7][2][2][1] = 116;   // invert maneuver pitch
  button_array[5][2][1][1] = 117;   // enable jump system mode
  button_array[2][2][1][1] = 119;    // toggle impulse slider mode
  button_array[4][2][1][1] = 121;   // enable warp system mode
  button_array[0][2][2][1] = 124;   // enter jump course
  button_array[6][2][2][1] = 125;   // enter maneuver course

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

  // placeholder indicator lights
  light_array[6][0][2] = true;   //console zoom
  light_array[7][0][2] = true;   //console zoom
  light_array[6][4][2] = true;   //request dock
  light_array[6][5][2] = true;   //enter course

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

  // cycle shift register for the first time to clear all data from the registers
  cycle_shift_register();

  // allow output enable pin to go low to enable output from shift registers
  // (there is a pullup resistor on the output enable pin to ensure the startup state
  // of the controllers does not allow lights to come on and over-current the 3V supply)
  pinMode(OUTPUT_ENABLE_PIN, OUTPUT);
  digitalWrite(OUTPUT_ENABLE_PIN, LOW);

      
}   // end SETUP ********************************************************************************************


void DMX_read() {
        static  uint16_t DmxCount;
        uint8_t  USARTstate= UART1_S1;    //get state before data!
        uint8_t  DmxByte   = UART1_D;          //get data
        uint8_t  DmxState  = gDmxState; //just load once from SRAM to increase speed
 
        if (USARTstate & UART_S1_FE)               //check for break
        {
                DmxCount = DmxAddress;         //reset channel counter (count channels before start address)
                gDmxState= BREAK;
                Errors++;
                UART1_S1 = UART1_S1 & UART_S1_FE;
        }
 
        else if (DmxState == BREAK)
        {
                if (DmxByte == 0) gDmxState= STARTB;  //normal start code detected
                else {
                  gDmxState= DMX_IDLE;
                  controller_connected[0] = false;
                }
        }
 
        else if (DmxState == STARTB)
        {
                if (--DmxCount == 0)    //start address reached?
                {
                        DmxCount= 1;            //set up counter for required channels
                        DmxRxField[0]= DmxByte; //get 1st DMX channel of device
                        gDmxState= STARTADR;
                        BytesRead++;
                        controller_connected[0] = true;
                }
        }
 
        else if (DmxState == STARTADR)
        {
                DmxRxField[DmxCount++]= DmxByte;        //get channel
                BytesRead++;
                if (DmxCount >= sizeof(DmxRxField)) //all ch received?
                {
                        gDmxState= DMX_IDLE;        //wait for next break
                }
        }                                               
}   // end DMX_read code *****************************************************************************


void lookfor_dmx_changes() {       // DMX change code ***********************************************
  for (int channel = 0; channel < 71; channel++) {
    dmx_array[channel][0] = DmxRxField[channel];
    if (dmx_array[channel][0] != dmx_array[channel][1]) {
      dmx_array[channel][1] = dmx_array[channel][0];
      if (dmx_array[channel][0] != 0) {                     // code to transmit changes to other controllers
        add_data_to_transmit_queue(101, channel, 1);
      }
      else {
        add_data_to_transmit_queue(101, channel, 0);
      }
      add_data_to_transmit_queue(101, 99, channel);
    }

    switch (channel) {
      case 7:                           // receive shields on cue from Artemis DMX
        if (dmx_array[channel][0] != 0) {
          shields_on = true;
        }
        else {
          shields_on = false;
        }
      break;
      case 8:                           // receive front shield low cue from Artemis DMX
        if (dmx_array[channel][0] != 0) {
          front_shield_low = true;
        }
        else {
          front_shield_low = false;
        }
      break;
      case 9:                           // receive rear shield low cue from Artemis DMX
        if (dmx_array[channel][0] != 0) {
          rear_shield_low = true;
        }
        else {
          rear_shield_low = false;
        }
      break;
      case 17:                           // receive tractor for dock cue from Artemis DMX
        if (dmx_array[channel][0] != 0) {
          tractor_state_on = true;
        }
        else {
          tractor_state_on = false;
        }
      break;
      case 18:                           // receive docked cue from Artemis DMX
        if (dmx_array[channel][0] != 0) {
          dock_state_on = true;
        }
        else {
          dock_state_on = false;
        }
      break;
      case 19:                           // receive reverse on cue from Artemis DMX
        if (dmx_array[channel][0] != 0) {
          reverse_state_on = true;
        }
        else {
          reverse_state_on = false;
        }
      break;
      case 28:                          // ship takes internal damage
        if (dmx_array[channel][0] != 0) {
          ship_hit = 3;
        }
        else {
          ship_hit = 0;
        }
      break;
      case 32:                          // ship damage 20
        if (dmx_array[channel][0] != 0) {
          damage_state = 1;
        }
        else {
          damage_state = 0;
        }
      break;
      case 33:                          // ship damage 40
        if (dmx_array[channel][0] != 0) {
          damage_state = 2;
        }
      break;
      case 34:                          // ship damage 60
        if (dmx_array[channel][0] != 0) {
          damage_state = 3;
        }
      break; 
    }                                     // end HELM interpret DMX
              
  }
}


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
      add_data_to_transmit_queue(124, active_course, 0);
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
      case 22:                      // request dock
        Keyboard.press(KEY_R);
        Keyboard.release(KEY_R);
      break;
      case 23:                      // pitch up
        Keyboard.press(KEY_INSERT);
        Keyboard.release(KEY_INSERT);
      break;
      case 24:                      // pitch down
        Keyboard.press(KEY_DELETE);
        Keyboard.release(KEY_DELETE);
      break;
      case 27:
        if (!warp_mode_on) {
          // click on jump cancel
          jump_on = false; 
          jump_state = 0;
          enter_command(210, JUMP_CANCEL_X, JUMP_CANCEL_Y);
          enter_command(211, 0, 0);
          enter_command(212, 0, 0); 
        }
      break;
      case 26:
        if (!warp_mode_on) {
          // click on jump confirm
          jump_on = false; 
          jump_state = 0;
          enter_command(210, JUMP_CONFIRM_X, JUMP_CONFIRM_Y);
          enter_command(211, 0, 0);
          enter_command(212, 0, 0);  
        }
      break;
      case 25:
        if (!warp_mode_on) {
          // click on initiate jump
          jump_on = true;
          jump_state = 2;
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
          add_data_to_transmit_queue(135, 0, 0);
        }
        else {
          CAM_mode = true;
          add_data_to_transmit_queue(135, 1, 0);
        }
      break;

      // complex command block ************
      case 116:                 // toggle invert pitch mode for joystick
        if (!invert_pitch) {
          invert_pitch = true;
        }
        else {
          invert_pitch = false;
        }
      break;
      case 117:                 // press enable jump button
          warp_mode_on = false;
      break;
      case 118:                 // press reverse button
          if (!impulse_mode_dual) {
            Keyboard.press(KEY_ESC);
            Keyboard.release(KEY_ESC);
            if (reverse_state_on) {
              reverse_state_on = false;
            }
            else {
              reverse_state_on = true;
            }
          }
      break;
      case 119:                 // toggle impulse slider mode (full to stop vs. forward to reverse)
          if (impulse_mode_dual) {
            impulse_mode_dual = false;
          }
          else {
            impulse_mode_dual = true;
          }
      break;
      case 121:                 // press enable warp button
          warp_mode_on = true;
      break;
      case 124:
        // enter jump course
        if (!warp_mode_on) {
          jump_state = 1;
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
        light_array[0][1][2] = false;
        light_array[1][1][2] = false;
        light_array[2][1][2] = false;
        light_array[3][1][2] = false;
        light_array[4][1][2] = false;
        light_array[5][1][2] = false;
      if (impulse_mode_dual) {
        if (state > SLIDER_N1) {
          enter_command(214, 0, 0);  // turn impulse reverse off
          y = IMPULSE_BOTTOM;
        }
        if (state > SLIDER_P1) {
          impulse_on = true;
          light_array[3][1][2] = true;
          if (state > SLIDER_P2) {
            light_array[4][1][2] = true;
          }
          if (state > SLIDER_P3) {
            light_array[5][1][2] = true;
          }
          y = IMPULSE_BOTTOM - (state - SLIDER_MID) * 100 / IMPULSE_SCALE;
        }
        if (state < SLIDER_N1) {
          enter_command(213, 0, 0);  // turn impulse reverse on
          impulse_on = true;
          light_array[2][1][2] = true;
          if (state < SLIDER_N2) {
            light_array[1][1][2] = true;
          }
          if (state < SLIDER_N3) {
            light_array[0][1][2] = true;
          }
          y = IMPULSE_BOTTOM - (SLIDER_MID - state) * 100 / IMPULSE_SCALE;
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
      }
      else {
        if (state > SLIDER_N3) {
          light_array[0][1][2] = true;
          impulse_on = true;
        }
        if (state > SLIDER_N2) {
          light_array[1][1][2] = true;
        }
        if (state > SLIDER_N1) {
          light_array[2][1][2] = true;
        }
        if (state > SLIDER_P1) {
          light_array[3][1][2] = true;
        }
        if (state > SLIDER_P2) {
          light_array[4][1][2] = true;
        }
        if (state > SLIDER_P3) {
          light_array[5][1][2] = true;
        }
        y = IMPULSE_BOTTOM - state * 100 / IMPULSE_SCALE2;
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
      }
          
    break;
    case 1:   // warp slider -- click on speed bar
      light_array[0][2][2] = false;
      light_array[1][2][2] = false;
      light_array[2][2][2] = false;
      light_array[3][2][2] = false;
      if (!reverse_state_on && warp_mode_on) {
        x = WARP_X;        
        if (state < WARP_1) {
          y = WARP_0_Y;
          warp_state = 0;
        }
        if (state > WARP_1) {
          y = WARP_1_Y;
          warp_state = 1;
          light_array[0][2][2] = true;
        }
        if (state > WARP_2) {
          y = WARP_2_Y;
          warp_state = 2;
          light_array[1][2][2] = true;
        }
        if (state > WARP_3) {
          y = WARP_3_Y;
          warp_state = 3;
          light_array[2][2][2] = true;
        }
        if (state > WARP_4) {
          y = WARP_4_Y;
          warp_state = 4;
          enter_command(210, x, y + 10);
          enter_command(211, 0, 0);
          light_array[3][2][2] = true;
        }
        enter_command(210, x, y);
        enter_command(211, 0, 0);
        enter_command(212, 0, 0);
      }      
      enter_command(215, 1, 0);  // release slider for next warp change
    break;
    
    case 2:   // manuever axis -- click on rudder bar
      rudder_on = false;
      if (state > RUDDER_R1) {
        rudder_on = true;
      }
      if (state < RUDDER_L1) {
        rudder_on = true;
      }
      x = RUDDER_X + (state - SLIDER_MID) * 100 / RUDDER_SCALE;
      y = RUDDER_Y;
      if (x > RUDDER_MIN) {
        if (x > RUDDER_MAX) {
          enter_command(210, RUDDER_MAX, y);
          x = RUDDER_MAX + 40;
          enter_command(211, 0, 0);
          enter_command(210, x, y);
        }
        else {
          enter_command(210, x, y);
          enter_command(211, 0, 0);
        }
      }
      else {
        enter_command(210, RUDDER_MIN, y);
        x = RUDDER_MIN - 40;
        enter_command(211, 0, 0);
        enter_command(210, x, y);
      }

      enter_command(212, 0, 0);
      enter_command(215, 2, 0);  // release slider for next rudder change      
    break;

    case 3:   // pitch axis -- pitch ship up or down if position exceeds threshold
      x = state - SLIDER_MID;
      if (invert_pitch) {
        if (x > PITCH_DEADZONE && pitch_mode == 0) {
          enter_command(24, 0, 0);
          pitch_mode = -1;
        }
        if (x < -1 * PITCH_DEADZONE && pitch_mode == 0) {
          enter_command(23, 0, 0);
          pitch_mode = 1;
        }
        if (abs(x) < PITCH_DEADZONE) {
          if (pitch_mode == 1) {
            enter_command(24, 0, 0);
            pitch_mode = 0;
          }
          if (pitch_mode == -1) {
            enter_command(23, 0, 0);
            pitch_mode = 0;
          }
        }
      }
      else {
        if (x > PITCH_DEADZONE && pitch_mode == 0) {
          enter_command(23, 0, 0);
          pitch_mode = 1;
        }
        if (x < -1 * PITCH_DEADZONE && pitch_mode == 0) {
          enter_command(24, 0, 0);
          pitch_mode = -1;
        }
        if (abs(x) < PITCH_DEADZONE) {
          if (pitch_mode == 1) {
            enter_command(24, 0, 0);
            pitch_mode = 0;
          }
          if (pitch_mode == -1) {
            enter_command(23, 0, 0);
            pitch_mode = 0;
          }
        }
      }
      enter_command(215, 3, 0);  // release slider for next pitch change
    break;
  }
}

// READ ANALOG CONTROLS CODE ************************************************************************************
void read_analog() {
  slider_state[0][0] = slider_state[0][0] + analogRead(14);
  slider_state[0][1] = slider_state[0][1] + analogRead(15);
  slider_state[0][2] = slider_state[0][2] + analogRead(16);
  slider_state[0][3] = slider_state[0][3] + analogRead(17);
  slider_reads++;
  if (slider_reads == 3) {
    for(int i=0; i<=3; i++) {
      slider_state[0][i] = slider_state[0][i]/3;
      if(abs(slider_state[0][i] - slider_state[1][i]) > ANALOG_DIFFERENCE_THRESHOLD && !slider_command_on[i]) {
        slider_state[1][i] = slider_state[0][i];
        set_slider(i, slider_state[1][i]);
        slider_command_on[i] = true;
      }
      slider_state[0][i] = 0;
    }
    slider_reads = 0;
  }
}


// ANIMATE DAMAGE CODE ***********************************************************************************
void animate_SRS_damage() {
  for (int i=2; i<=3; i++) {
      light_array[i][2][0] = random(0,40) < long(damage_state * 10);
      if (ship_hit != 0) {
        light_array[i][2][0] = random(0,40) < long(ship_hit * 10);
      }
    }  
}

// REFRESH LIGHTS CODE ************************************************************************************
void refresh_lights() {
  // navigation panel
  for (int i=0; i<=7; i++) {
    for (int j=4; j<=7; j++) {
      light_array[i][j][1] = false;    // turn all button lights off
    }
  }
  switch (nav_mode) {
    case NAV_MODE_NORMAL:
      switch (active_course) {
        case 0:
          light_array[6][5][1] = true;
          break;
        case 1:
          light_array[3][5][1] = true;
          break;
        case 2:
          light_array[4][5][1] = true;
          break;
        case 3:
          light_array[5][5][1] = true;
          break;
        case 4:
          light_array[0][5][1] = true;
          break;
        case 5:
          light_array[1][5][1] = true;
          break;
        case 6:
          light_array[2][5][1] = true;
          break;
        case 7:
          light_array[3][4][1] = true;
          break;
        case 8:
          light_array[4][4][1] = true;
          break;
        case 9:
          light_array[5][4][1] = true;
          break;
      }
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
      break;
    case  NAV_DISP_DIST:
      light_array[1][4][1] = true;
      break;
  }

  // wrap & jump lights
  light_array[0][4][2] = false;
  light_array[1][4][2] = false;
  light_array[2][4][2] = false;
  light_array[3][4][2] = false;
  
  if (warp_mode_on) {
    light_array[6][3][2] = true;
    light_array[7][3][2] = false;
    light_array[4][4][2] = false;
    light_array[5][4][2] = false;
  }
  else {
    light_array[6][3][2] = false;
    light_array[7][3][2] = true;
    light_array[4][4][2] = true;
    light_array[5][4][2] = true;
    light_array[0][4][2] = true;
    if (jump_state > 0) {
      light_array[1][4][2] = true;
    }
    if (jump_state > 1) {
      light_array[2][4][2] = true;
      light_array[3][4][2] = true;
    }
  }

  // invert pitch
  if (invert_pitch) {
    light_array[7][5][2] = true;
  }
  else {
    light_array[7][5][2] = false;
  }
  
  // reverse, dock, tractor
  if (reverse_state_on) {
    light_array[5][2][2] = true;
  }
  else {
    light_array[5][2][2] = false;
  }
  if (tractor_state_on) {
    light_array[6][2][2] = true;
  }
  else {
    light_array[6][2][2] = false;
  }
  if (dock_state_on) {
    light_array[7][2][2] = true;
  }
  else {
    light_array[7][2][2] = false;
  }

  // impulse slider mode
  if (impulse_mode_dual) {
    light_array[4][2][2] = true;
    light_array[6][1][2] = true;
    light_array[7][1][2] = false;
  }
  else {
    light_array[4][2][2] = false;
    light_array[6][1][2] = false;
    light_array[7][1][2] = true;
  }

  for (int i=0; i<6; i++) {
      light_array[i][0][2] = false;
      light_array[i][3][2] = false;
      light_array[i][5][2] = false;
      light_array[i][4][0] = false;
      light_array[i][3][0] = false;
  }
  
  // manuever power level
  if (power_levels[3] > 5) {
    light_array[0][5][2] = true;
  }
  if (power_levels[3] > 15) {
    light_array[1][5][2] = true;
  }
  if (power_levels[3] > 25) {
    light_array[2][5][2] = true;
  }
  if (power_levels[3] > 35) {
    light_array[3][5][2] = true;
  }
  if (power_levels[3] > 45) {
    light_array[4][5][2] = true;
  }
  if (power_levels[3] > 55) {
    light_array[5][5][2] = true;
  }
  
  // impulse power level
  if (power_levels[4] > 5) {
    light_array[0][0][2] = true;
  }
  if (power_levels[4] > 15) {
    light_array[1][0][2] = true;
  }
  if (power_levels[4] > 25) {
    light_array[2][0][2] = true;
  }
  if (power_levels[4] > 35) {
    light_array[3][0][2] = true;
  }
  if (power_levels[4] > 45) {
    light_array[4][0][2] = true;
  }
  if (power_levels[4] > 55) {
    light_array[5][0][2] = true;
  }

  // warp power level
  if (power_levels[5] > 5) {
    light_array[0][3][2] = true;
  }
  if (power_levels[5] > 15) {
    light_array[1][3][2] = true;
  }
  if (power_levels[5] > 25) {
    light_array[2][3][2] = true;
  }
  if (power_levels[5] > 35) {
    light_array[3][3][2] = true;
  }
  if (power_levels[5] > 45) {
    light_array[4][3][2] = true;
  }
  if (power_levels[5] > 55) {
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
    case 123:                                                       // current course heading and distance
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
    case 124:                                                       // current course number
      RS_485_TX[dmx_transmit_queue_last][3] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][4] = RS_485_TX_STOP;
    break;
  }
  dmx_transmit_queue_last++;
  if (dmx_transmit_queue_last == MAX_TRANSMIT_QUEUE) {
    dmx_transmit_queue_last = 0;
  }
}    // end add data to transmit queue *****************************************************


void transmit_receive() {   //  RS-485 tranceiver code ****************************************

   // check to see if timeout time reached and waiting for data (used for DMX/master controllers ONLY)
  if (millis() > RS_485_delay_time && !transmit_enable) {
    transmit_enable = true;
    data_stream_on = false; // reset active transmission to start new transmission
    data_stream_pos = 0;
  }   // end timeout code
  
  
  // code for sending data ****** 
  if (transmit_enable) {

     if (dmx_transmit_queue_pos == dmx_transmit_queue_last) {  // poll next controller in queue (DMX/MCP code ONLY)
      controller_poll_position++;
      if (controller_poll_position > CONTROLLER_MAX_POLL) {
        controller_poll_position = 0;
      }
      /*for (int i=0; i<11; i++) {
        if (controller_connected[i]) {
          add_data_to_transmit_queue(220, controller_address[i], 0);
        }
      }*/
      add_data_to_transmit_queue(220, controller_address[controller_poll_position], 0);  
      controller_poll_position++;
      if (controller_poll_position > CONTROLLER_MAX_POLL) {
        controller_poll_position = 0;
      }
      add_data_to_transmit_queue(220, controller_address[controller_poll_position], 0); 
    }                                                       // end poll code
    
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
          if (RS_485_TX[dmx_transmit_queue_pos][2] == 220) {      // request data code for DMX/MCP controller
            transmit_enable = false;

            RS_485_delay_time = millis() + RS_485_TIMEOUT;        // timeout code for DMX/MCP controller
            switch (RS_485_TX[dmx_transmit_queue_pos][3]) {
              /*case 10:                            // HELM
                controller_connected[0] = false;
              break;*/
              case 20:                              // WEAPONS
                controller_connected[1] = false;
              break;
              case 30:                              // SCIENCE
                controller_connected[2] = false;
              break;
              case 40:                              // ENGINEERING
                controller_connected[3] = false;
              break;
              case 50:                              // COMMS
                controller_connected[4] = false;
              break;
            }                                                 // end timeout code
            
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

      if (data_stream_pos == 1 && rx == 215) {    // receive token from controller (DMX/MCP code ONLY)
        transmit_enable = true;
        if (jump_on || warp_state != 0) {
          add_data_to_transmit_queue(133, 5, 1);  // jump state
        }
        else {
          add_data_to_transmit_queue(133, 5, 0);            
        }
        if (rudder_on) {
          add_data_to_transmit_queue(133, 3, 1);  // maneuver in use
        }
        else {
          add_data_to_transmit_queue(133, 3, 0);            
        }
        if (impulse_on) {
          add_data_to_transmit_queue(133, 4, 1);  // impulse in use
        }
        else {
          add_data_to_transmit_queue(133, 4, 0);            
        }
        for (int i=0; i<DMX_CHECK_QUANTITY; i++) {
          if (DMX_refresh_count < 41 || DMX_refresh_count > 44) {
            if (dmx_array[DMX_refresh_count][0] != 0) {                     // code to cycke through DMX array and send state to other controllers
              add_data_to_transmit_queue(101, DMX_refresh_count, 1);
            }
            else {
              add_data_to_transmit_queue(101, DMX_refresh_count, 0);
            }
          }          
          if (DMX_refresh_count == 44) {                                  // code to handle error with power cues
            if (dmx_array[40][0] != 0 || dmx_array[41][0] != 0 || dmx_array[42][0] != 0 || dmx_array[43][0] != 0) {
              add_data_to_transmit_queue(101, 44, 0);
            }
            else {
              add_data_to_transmit_queue(101, 44, 1);
            }
          }
          if (DMX_refresh_count == 43) {                                  // code to handle error with power cues
            if (dmx_array[40][0] != 0 || dmx_array[41][0] != 0 || dmx_array[42][0] != 0) {
              add_data_to_transmit_queue(101, 43, 0);
            }
            else {
              if (dmx_array[43][0] != 0) {
                add_data_to_transmit_queue(101, 43, 1);
              }
            }
          }
          if (DMX_refresh_count == 42) {                                  // code to handle error with power cues
            if (dmx_array[40][0] != 0 || dmx_array[41][0] != 0) {
              add_data_to_transmit_queue(101, 42, 0);
            }
            else {
              if (dmx_array[42][0] != 0) {
                add_data_to_transmit_queue(101, 42, 1);
              }
            }
          }
          if (DMX_refresh_count == 41) {                                  // code to handle error with power cues
            if (dmx_array[40][0] != 0) {
              add_data_to_transmit_queue(101, 41, 0);
            }
            else {
              if (dmx_array[41][0] != 0) {
                add_data_to_transmit_queue(101, 41, 1);
              }
            }
          }
          DMX_refresh_count++;
          if (DMX_refresh_count > 70) {
            DMX_refresh_count = 0;
          }
        }
      }                                         // end receive token code

      // sort data into appropriate places

      if (data_stream_pos > 1) {
        if (RS_485_RX[1] == 212) {                // code receiving "check-in" from controller
          switch (RS_485_RX[2]) {
            /*case 10:                            // HELM
              controller_connected[0] = false;
            break;*/
            case 20:                              // WEAPONS
              controller_connected[1] = true;
            break;
            case 30:                              // SCIENCE
              controller_connected[2] = true;
            break;
            case 40:                              // ENGINEERING
              controller_connected[3] = true;
            break;
            case 50:                              // COMMS
              controller_connected[4] = true;
            break;
          }
        }
        if (RS_485_RX[1] == 220) {                // code requesting data from controller when not DMX/MCP controller
          if (RS_485_RX[2] == MY_CONTROLLER_ADDRESS) {
            transmit_enable = true;
            add_data_to_transmit_queue(212, MY_CONTROLLER_ADDRESS, 0);
            if (jump_on || warp_state != 0) {
              add_data_to_transmit_queue(133, 5, 1);  // jump state
            }
            else {
              add_data_to_transmit_queue(133, 5, 0);            
            }
            if (rudder_on) {
              add_data_to_transmit_queue(133, 3, 1);  // maneuver in use
            }
            else {
              add_data_to_transmit_queue(133, 3, 0);            
            }
            if (impulse_on) {
              add_data_to_transmit_queue(133, 4, 1);  // impulse in use
            }
            else {
              add_data_to_transmit_queue(133, 4, 0);            
            }
            add_data_to_transmit_queue(215, 0, 0);   // return TX token to master controller
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
            case 19:                           // receive reverse on cue from Artemis DMX
              if (RS_485_RX[3] == 1) {
                reverse_state_on = true;
              }
              else {
                reverse_state_on = false;
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

  if (millis() > DMX_check_time) {        // look for changes in DMX output from Artemis
    lookfor_dmx_changes();
    DMX_check_time = millis() + DMX_CHECK_INTERVAL;
  } 

  // check for instructions from master controller & transmit if it is this controller's turn (regular code)
  // send instructions to onther controllers and wait for response (DMX/MCP code)

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
    animate_SRS_damage();
    animate_shields_timer = millis() + ANIMATE_SHIELDS_INTERVAL;
  }

  // refresh lights
  if (millis() >= refresh_light_delay) {
    refresh_lights();
    
    refresh_light_delay = millis() + REFRESH_LIGHT_INTERVAL;
  }
  
}

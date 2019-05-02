// Artemis SBS Communications Control Panel (Block III) Code
// NOTE: some input pins may misbehave if left open (disconnected from panel hardware)
// connected panel variables are available to exclude input from these disconnected pins

// code optimized for 1280 x 720 client resolution - will not work with other resolutions

// digital inputs from the keypad matrices are arranged active HIGH with a pull-down resistor to make use of the shift registers

// connected panels array - used to tell controller which pins should be read when reading controls - false disables input
bool connected_panels[4] = {true, false, false, false};
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
#define PIN_Y_NEG 17
#define PIN_X_PLUS 16
#define PIN_Y_PLUS 15
#define PIN_X_NEG 14

// console-specific (COMMUNICATIONS) variables
#define MAX_TRANSMIT_QUEUE 30
int dmx_transmit_queue_pos = 0;
int dmx_transmit_queue_last = 0;
#define MY_CONTROLLER_ADDRESS 50
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
bool red_alert_on = false;
bool normal_on = false;
bool tractor_state_on = false;
bool dock_state_on = false;
bool jump_on = false;
bool alert_on = false;
bool shield_hit_on = false;
bool damage_on = false;
bool damcon_on = false;
int damage_state = 0;
int power_state = 5;
bool power_low_on = false;
bool panel_active[5] = {false, false, false, false, false};
bool panel_error_on = false;



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
  button_array[0][0][1][1] = 110;    // 0 key
  button_array[1][0][1][1] = 109;    // 9 key
  button_array[2][0][1][1] = 108;    // 8 key
  button_array[3][0][1][1] = 107;    // 7 key
  button_array[0][0][2][1] = 106;    // 6 key
  button_array[1][0][2][1] = 105;    // 5 key
  button_array[2][0][2][1] = 104;    // 4 key
  button_array[0][0][3][1] = 56;    // select button (click)
  button_array[1][0][3][1] = 103;    // 3 key
  button_array[2][0][3][1] = 102;    // 2 key
  button_array[3][0][3][1] = 101;    // 1 key
  button_array[4][0][3][1] = 111;    // red alert
  
  // complex "high-level" commands second

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
        // press 0 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_7);
        Keyboard.release(KEY_7);
        Keyboard.set_modifier(0);        
      break;
      case 106:
        // press 0 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_6);
        Keyboard.release(KEY_6);
        Keyboard.set_modifier(0);        
      break;
      case 105:
        // press 0 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_5);
        Keyboard.release(KEY_5);
        Keyboard.set_modifier(0);        
      break;
      case 104:
        // press 0 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_4);
        Keyboard.release(KEY_4);
        Keyboard.set_modifier(0);        
      break;
      case 103:
        // press 0 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_3);
        Keyboard.release(KEY_3);
        Keyboard.set_modifier(0);        
      break;
      case 102:
        // press 0 key
        Keyboard.set_modifier(MODIFIERKEY_CTRL);
        Keyboard.press(KEY_2);
        Keyboard.release(KEY_2);
        Keyboard.set_modifier(0);        
      break;
      case 101:
        // press 0 key
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

      // complex command block ************
      
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
      light_array[0][2][0] = false;
      light_array[1][2][0] = true;
      light_array[1][3][0] = false;
      light_array[0][3][0] = true;
    }
    else {
      if (pad_active == true) {
        if (millis() - mouse_timer > CLICK_MIN_TIME && millis() - mouse_timer < CLICK_MAX_TIME) {
          enter_command(56, 0, 0);
        }
      }
      pad_active = false;
      light_array[0][2][0] = true;
      light_array[1][2][0] = false;
      light_array[1][3][0] = true;
      light_array[0][3][0] = false;
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
    light_array[0][2][0] = false;
    light_array[1][2][0] = true;
    light_array[1][3][0] = false;
    light_array[0][3][0] = true;
  }
  else {
    light_array[0][2][0] = true;
    light_array[1][2][0] = false;
    light_array[1][3][0] = true;
    light_array[0][3][0] = false;
  }

  // shields low
  if (front_shield_low || rear_shield_low) {
    light_array[6][0][0] = true;
  }
  // red alert
  if (red_alert_on) {
    light_array[6][1][0] = true;
  }  
  // normal
  if (normal_on) {
    light_array[7][1][0] = true;
  }
  // tractored for dock
  if (tractor_state_on) {
    light_array[5][1][0] = true;
  }
  // docked
  if (dock_state_on) {
    light_array[4][1][0] = true;
  }
  // jump
  if (jump_on) {
    light_array[3][1][0] = true;
  }
  // alert
  if (alert_on) {
    light_array[7][0][0] = true;
  }
  // shield hit
  if (shield_hit_on) {
    light_array[5][0][0] = true;
  }
  // damage
  if (damage_on) {
    light_array[4][0][0] = true;
  }
  // damcon loss
  if (damcon_on) {
    light_array[3][0][0] = true;
  }

  switch (damage_state) {
    case 0:
      light_array[6][4][0] = true;
    break;
    case 1:
      light_array[5][4][0] = true;
    break;
    case 2:
      light_array[4][4][0] = true;
    break;
    case 3:
      light_array[3][4][0] = true;
    break;
  }
  if (power_state > 0) {
    light_array[2][2][0] = true;
  }
  if (power_state > 1) {
    light_array[7][2][0] = true;
  }
  if (power_state > 2) {
    light_array[6][2][0] = true;
  }
  if (power_state > 3) {
    light_array[5][2][0] = true;
  }
  if (power_state > 4) {
    light_array[4][2][0] = true;
  }
  if (power_state > 5) {
    light_array[3][2][0] = true;
  }
  if (power_low_on) {
    light_array[0][4][0] = true;
  }

    
}  // END REFRESH LIGHTS CODE ***************************************************************

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
            case 0:
              if (RS_485_RX[3] == 1) {
                normal_on = true;
              }
              else {
                normal_on = false;
              }
            break;
            case 3:
              if (RS_485_RX[3] == 1) {
                red_alert_on = true;
              }
              else {
                red_alert_on = false;
              }
            break;
            case 7:
              if (RS_485_RX[3] == 1) {
                shields_on = true;
              }
              else {
                shields_on = false;
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
            case 32:
              if (RS_485_RX[3] == 1) {
                damage_state = 1;
              }
              else {
                damage_state = 0;
              }
            break;
            case 33:
              if (RS_485_RX[3] == 1) {
                damage_state = 2;
              }
            break;
            case 34:
              if (RS_485_RX[3] == 1) {
                damage_state = 3;
              }
            break;
            case 40:
              if (RS_485_RX[3] == 1) {
                power_state = 1;
              }
            break;
            case 41:
              if (RS_485_RX[3] == 1) {
                power_state = 2;
              }
            break;
            case 42:
              if (RS_485_RX[3] == 1) {
                power_state = 3;
              }
            break;
            case 43:
              if (RS_485_RX[3] == 1) {
                power_state = 4;
              }
            break;
            case 44:
              if (RS_485_RX[3] == 1) {
                power_state = 5;
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
            case 11:
              if (RS_485_RX[3] == 1) {
                jump_on = true;
              }
              else {
                jump_on = false;
              }
            break;
            case 20:
              if (RS_485_RX[3] == 1) {
                shield_hit_on = true;
              }
              else {
                shield_hit_on = false;
              }
            break;
            case 28:
              if (RS_485_RX[3] == 1) {
                damage_on = true;
              }
              else {
                damage_on = false;
              }
            break;
            case 37:
              if (RS_485_RX[3] == 1) {
                damcon_on = true;
              }
              else {
                damcon_on = false;
              }
            break;
            case 39:
              if (RS_485_RX[3] == 1) {
                power_low_on = true;
              }
              else {
                power_low_on = false;
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

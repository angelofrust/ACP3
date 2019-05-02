// Artemis SBS Master Control (and DMX reader) Panel

#include <Keypad.h>

#define HWSERIAL Serial1
     
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

int dmx_display_queue[10] = {0,0,0,0,0,0,0,0,0};
int dmx_display_count = 0;
int dmx_display_max = 0;

#define MAX_TRANSMIT_QUEUE 30
int dmx_transmit_queue_pos = 0;
int dmx_transmit_queue_last = 0;
bool controller_connected[11] = {false, false, false, false, false, false, false, false, false, false, false};
int controller_address[11] = {100, 100, 100, 100, 100, 100, 10, 20, 30, 40, 50};
int controller_poll_position = 0;
byte RS_485_TX[MAX_TRANSMIT_QUEUE][10];   // variable to store bytes for transmittal on the RS-485 bus
int RS_485_RX[10];  // variable to store received bytes
bool data_stream_on = false; // variable to store in currently transmitting data state
int data_stream_pos = 0;  // varaible to store where in the block of data we are currently transmitting/receiving
bool transmit_enable = true; // variable to turn ability to transmit on and off (token)
unsigned long RS_485_delay_time = 0;   // variable to store the time between transmissions
#define RS_485_TIMEOUT 100  // timeout time in milliseconds for master controller to wait for return signals from other controllers
#define RS_485_TX_START 255 // symbol to signify start of transmitting stream
#define RS_485_TX_STOP 254 // symbol to stop transmitting stream

long status_time = 0;
#define UPDATE_STATUS 500

#define FRAME_RATE_SLOW 400  // delay in milliseconds between animation frames
#define FRAME_RATE_FAST 200  // delay in milliseconds between animation frames
#define FRAME_RATE_VFAST 100  // delay in milliseconds between animation frames
#define REFRESH_RATE 2  // delay in milliseconds between lighting columns

unsigned long refresh_last = 0;  // variable to store the time to the last light column switch
byte active_column = 0;  // variable to store the current lit column in the LED matrix

const int light_columns[4] = {5,16,17,21};
const int light_rows[10] = {14,15,18,19,20,22,23,24,25,26};

bool lights[10][4] = {      // initialize light matrix
  {true,true,true,true},
  {true,true,true,true},
  {true,true,true,true},
  {true,true,true,true},
  {true,true,true,true},
  {true,true,true,true},
  {true,true,true,true},
  {true,true,true,true},
  {true,true,true,true},
  {true,true,true,true}
};

// setup button matrix
const byte rows = 4; //four rows
const byte cols = 4; //four columns
char keys[rows][cols] = {      // rows, columns
  {10,11,12,13},
  {20,21,22,23},
  {30,31,32,33},
  {40,41,42,43}
};
byte colPins[cols] = {2, 3, 4, 6}; //connect to the column pinouts of the keypad
byte rowPins[rows] = {7, 8, 11, 12}; //connect to the row pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, rows, cols );

int input_mode = 0;
#define INPUT_MODE_STANDBY 0
#define INPUT_MODE_KEYPAD 1

int data_mode = 0;
#define DATA_MODE_STANDBY 0
#define DATA_MODE_DMX 1
#define DATA_MODE_DEMO 2

unsigned long data_frame_time1 = 0;
unsigned long data_frame_time[8];
int data_status[8][4] = {
  // code, light state (no of blinks), delay, priority
  {-1, -1, 0, 5},
  {-1, -1, 0, 5},
  {-1, -1, 0, 5},
  {-1, -1, 0, 5},
  {-1, -1, 0, 5},
  {-1, -1, 0, 5},
  {-1, -1, 0, 5},
  {-1, -1, 0, 5}
};

const int data_status_map[71][4] = {
  // light number, light state (no of blinks), delay, priority
  {-1,0,0,5}, // 0   normal condition
  {-1,0,0,5},        // game over
  {-1,0,0,5},       // war turn warning
  {3,10,500,1},     // red alert
  {-1,0,0,5},       // item collected
  {-1,0,0,5},       // player shields raised
  {-1,0,0,5},       // player shields lowered
  {1,-1,0,3},       // player shields on
  {1,5,200,2},    // front shield low
  {1,5,200,2},    // rear shield low
  {2,5,500,2}, // 10    jump initiated
  {2,5,200,1},     // jump executed
  {-1,0,0,5},       // jump fizzled
  {-1,0,0,5},       // entering nebula
  {-1,0,0,5},       // exiting nebula
  {2,-1,0,3},       // within nebula
  {-1,0,0,5},       // start docking
  {2,10,500,2},     // tractored for docked
  {2,-1,0,3},       // completely docked
  {2,11,200,2},    // helm in reverse
  {5,5,100,1}, // 20    something hits player
  {-1,0,0,5},       // NPC beam hits player
  {-1,0,0,5},       // Player beam hits player
  {-1,0,0,5},       // torpedo hits player
  {-1,0,0,5},       // mine hits player
  {-1,0,0,5},       // lightning hits player
  {-1,0,0,5},       // collision hits player
  {-1,0,0,5},       // drone hits player
  {6,5,100,1},      // player takes internal damage
  {1,5,100,1},      // player takes shield damage
  {-1,0,0,5}, // 30  player takes front shield damage
  {-1,0,0,5},       // player takes rear shield damage
  {6,9,750,3},     // ship damage 20
  {6,9,500,3},     // ship damage 40
  {6,9,250,3},     // ship damage 60
  {6,-1,0,3},       // player destroyed
  {-1,0,0,5},       // self destructed
  {4,9,750,2},     // damcon casualty
  {4,9,500,1},     // just killed damcon member
  {0,9,750,2},     // energy low
  {-1,0,0,5}, // 40   energy 20
  {-1,0,0,5},       // energy 40
  {-1,0,0,5},       // energy 60
  {-1,0,0,5},       // energy 80
  {0,-1,0,2},       // energy 100
  {-1,0,0,5},       // energy 200
  {7,9,500,2},     // unloading tube
  {-1,0,0,5},       // unloading tube 1
  {-1,0,0,5},       // unloading tube 2
  {-1,0,0,5},       // unloading tube 3
  {-1,0,0,5}, // 50    unloaidng tube 4
  {7,9,750,2},     // loading tube 
  {-1,0,0,5},       // loading tube 1
  {-1,0,0,5},       // loading tube 2
  {-1,0,0,5},       // loading tube 3
  {-1,0,0,5},       // loading tube 4
  {7,-1,0,3},       // any tube ready to fire
  {-1,0,0,5},       // any tube empty
  {-1,0,0,5},
  {-1,0,0,5},
  {-1,0,0,5}, // 60
  {-1,0,0,5},
  {-1,0,0,5},
  {-1,0,0,5},
  {-1,0,0,5},
  {-1,0,0,5},
  {-1,0,0,5},
  {-1,0,0,5},
  {-1,0,0,5},
  {-1,0,0,5},
  {-1,0,0,5} // 70
};

const int data_lights[8][2] = {
  {2,2},
  {5,2},
  {8,2},
  {9,0},
  {0,2},
  {3,2},
  {6,2},
  {9,2}
};

int display_mode = 0;
int display_frame = 0;
unsigned long display_frame_time = 0;
#define DISPLAY_MODE_STANDBY 0
#define DISPLAY_MODE_DMX 1
#define DISPLAY_MODE_DEMO 2
#define DISPLAY_MODE_KEYPAD 3

int connection_mode = 0;
unsigned long connection_frame_time = 0;
#define CONNECTION_MODE_STANDBY 0
#define CONNECTION_MODE_DMX 1
#define CONNECTION_MODE_DEMO 2
#define CONNECTION_REFRESH_RATE 10 // time in milliseconds between updates of connection lights

int active_program = 0;
int enter_program = 0;
int enter_character = 0;


void setup() {
  Serial.begin(9600);   // USB serial - used to debug

  // setup DMX reader **************************************************************************************
  HWSERIAL.begin(250000, SERIAL_8N2); //Enable serial reception with a 250k rate
  gDmxState= DMX_IDLE; // initial state
  DmxAddress = 1; // The desired DMX Start Address

  status_time = millis();
  display_frame_time = millis() + FRAME_RATE_VFAST;

  connection_frame_time = millis() + CONNECTION_REFRESH_RATE;

  attachInterruptVector(IRQ_UART0_STATUS, DMX_read);

  // setup RS-485 bus **************************************************************************************
  Serial2.begin(9600); //Enable serial TX/RX
  // pinMode(13, OUTPUT);
  // digitalWrite(13, HIGH);
  Serial2.transmitterEnable(13);  // Use pin 13 to automatically enable a RS-485 transceiver chip. This pin outputs a logic HIGH when transmitting data. It goes LOW after the last stop bit, to allow reception when not transmitting. 

  RS_485_delay_time = millis() + RS_485_TIMEOUT;

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

  // setup light matrix *****************************************************************************

      pinMode(light_columns[0], OUTPUT);
      pinMode(light_columns[1], OUTPUT);
      pinMode(light_columns[2], OUTPUT);
      pinMode(light_columns[3], OUTPUT);
      pinMode(light_rows[0], OUTPUT);
      pinMode(light_rows[1], OUTPUT);
      pinMode(light_rows[2], OUTPUT);
      pinMode(light_rows[3], OUTPUT);
      pinMode(light_rows[4], OUTPUT);
      pinMode(light_rows[5], OUTPUT);
      pinMode(light_rows[6], OUTPUT);
      pinMode(light_rows[7], OUTPUT);
      pinMode(light_rows[8], OUTPUT);
      pinMode(light_rows[9], OUTPUT);
      refresh_last = millis();      // initialize the light refresh time to the start of the program

      digitalWrite(light_columns[0], HIGH);     // turn off all columns in the LED matrix
      digitalWrite(light_columns[1], HIGH);
      digitalWrite(light_columns[2], HIGH);
      digitalWrite(light_columns[3], HIGH);

      for (int i = 0; i < 8; i++) {
        data_frame_time[i] = millis();
      }
      data_frame_time1 = millis();
      
}   // end SETUP ********************************************************************************************


void refresh_lights() {
  // update the lights with the current pattern  ********************************************
     
    digitalWrite(light_columns[active_column], HIGH);  // Turn whole previous column off
    active_column++;
    if (active_column == 4) {
      active_column = 0;
    }
    for (int row = 0; row < 10; row++) {
      if (lights[row][active_column] == true) {
        digitalWrite(light_rows[row], HIGH);  // Turn on this led
      }
      else {
        digitalWrite(light_rows[row], LOW); // Turn off this led
      }
    }
    digitalWrite(light_columns[active_column], LOW); // Turn whole column on at once (for equal lighting times)
    refresh_last = millis();

}  // end REFRESH_LIGHTS ************************************************************************************


void set_led_number(int number, int pos) {
  if (pos < 0) {
    pos = 0;
  }
  if (pos > 2) {
    pos = 2;
  }

  lights[0 + pos*3][0] = false;   // UPPER LEFT segment
  lights[1 + pos*3][0] = false;   // TOP segment
  lights[2 + pos*3][0] = false;   // UPPER RIGHT segment
  lights[0 + pos*3][1] = false;   // LOWER LEFT segment
  lights[1 + pos*3][1] = false;   // MIDDLE segment
  lights[2 + pos*3][1] = false;   // LOWER RIGHT segment
  lights[1 + pos*3][2] = false;   // BOTTOM segment
      
  switch (number){
    case 0:
      lights[0 + pos*3][0] = true;   // UPPER LEFT segment
      lights[1 + pos*3][0] = true;   // TOP segment
      lights[2 + pos*3][0] = true;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = true;   // LOWER LEFT segment
      lights[1 + pos*3][1] = false;   // MIDDLE segment
      lights[2 + pos*3][1] = true;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = true;   // BOTTOM segment
    break;
    case 1:
      lights[0 + pos*3][0] = false;   // UPPER LEFT segment
      lights[1 + pos*3][0] = false;   // TOP segment
      lights[2 + pos*3][0] = true;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = false;   // LOWER LEFT segment
      lights[1 + pos*3][1] = false;   // MIDDLE segment
      lights[2 + pos*3][1] = true;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = false;   // BOTTOM segment
    break;
    case 2:
      lights[0 + pos*3][0] = false;   // UPPER LEFT segment
      lights[1 + pos*3][0] = true;   // TOP segment
      lights[2 + pos*3][0] = true;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = true;   // LOWER LEFT segment
      lights[1 + pos*3][1] = true;   // MIDDLE segment
      lights[2 + pos*3][1] = false;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = true;   // BOTTOM segment
    break;
    case 3:
      lights[0 + pos*3][0] = false;   // UPPER LEFT segment
      lights[1 + pos*3][0] = true;   // TOP segment
      lights[2 + pos*3][0] = true;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = false;   // LOWER LEFT segment
      lights[1 + pos*3][1] = true;   // MIDDLE segment
      lights[2 + pos*3][1] = true;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = true;   // BOTTOM segment
    break;
    case 4:
      lights[0 + pos*3][0] = true;   // UPPER LEFT segment
      lights[1 + pos*3][0] = false;   // TOP segment
      lights[2 + pos*3][0] = true;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = false;   // LOWER LEFT segment
      lights[1 + pos*3][1] = true;   // MIDDLE segment
      lights[2 + pos*3][1] = true;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = false;   // BOTTOM segment
    break;
    case 5:
      lights[0 + pos*3][0] = true;   // UPPER LEFT segment
      lights[1 + pos*3][0] = true;   // TOP segment
      lights[2 + pos*3][0] = false;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = false;   // LOWER LEFT segment
      lights[1 + pos*3][1] = true;   // MIDDLE segment
      lights[2 + pos*3][1] = true;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = true;   // BOTTOM segment
    break;
    case 6:
      lights[0 + pos*3][0] = true;   // UPPER LEFT segment
      lights[1 + pos*3][0] = true;   // TOP segment
      lights[2 + pos*3][0] = false;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = true;   // LOWER LEFT segment
      lights[1 + pos*3][1] = true;   // MIDDLE segment
      lights[2 + pos*3][1] = true;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = true;   // BOTTOM segment
    break;
    case 7:
      lights[0 + pos*3][0] = false;   // UPPER LEFT segment
      lights[1 + pos*3][0] = true;   // TOP segment
      lights[2 + pos*3][0] = true;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = false;   // LOWER LEFT segment
      lights[1 + pos*3][1] = false;   // MIDDLE segment
      lights[2 + pos*3][1] = true;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = false;   // BOTTOM segment
    break;
    case 8:
      lights[0 + pos*3][0] = true;   // UPPER LEFT segment
      lights[1 + pos*3][0] = true;   // TOP segment
      lights[2 + pos*3][0] = true;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = true;   // LOWER LEFT segment
      lights[1 + pos*3][1] = true;   // MIDDLE segment
      lights[2 + pos*3][1] = true;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = true;   // BOTTOM segment
    break;
    case 9:
      lights[0 + pos*3][0] = true;   // UPPER LEFT segment
      lights[1 + pos*3][0] = true;   // TOP segment
      lights[2 + pos*3][0] = true;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = false;   // LOWER LEFT segment
      lights[1 + pos*3][1] = true;   // MIDDLE segment
      lights[2 + pos*3][1] = true;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = true;   // BOTTOM segment
    break;
  }
}   // end code to set numbers on display


void set_led_char(char character, int pos) {
  if (pos < 0) {
    pos = 0;
  }
  if (pos > 2) {
    pos = 2;
  }

  lights[0 + pos*3][0] = false;   // UPPER LEFT segment
  lights[1 + pos*3][0] = false;   // TOP segment
  lights[2 + pos*3][0] = false;   // UPPER RIGHT segment
  lights[0 + pos*3][1] = false;   // LOWER LEFT segment
  lights[1 + pos*3][1] = false;   // MIDDLE segment
  lights[2 + pos*3][1] = false;   // LOWER RIGHT segment
  lights[1 + pos*3][2] = false;   // BOTTOM segment
      
  switch (character){
    case 'C':
      lights[0 + pos*3][0] = true;   // UPPER LEFT segment
      lights[1 + pos*3][0] = true;   // TOP segment
      lights[2 + pos*3][0] = false;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = true;   // LOWER LEFT segment
      lights[1 + pos*3][1] = false;   // MIDDLE segment
      lights[2 + pos*3][1] = false;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = true;   // BOTTOM segment
    break;
    case '_':
      lights[0 + pos*3][0] = false;   // UPPER LEFT segment
      lights[1 + pos*3][0] = false;   // TOP segment
      lights[2 + pos*3][0] = false;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = false;   // LOWER LEFT segment
      lights[1 + pos*3][1] = false;   // MIDDLE segment
      lights[2 + pos*3][1] = false;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = true;   // BOTTOM segment
    break;
    case '-':
      lights[0 + pos*3][0] = false;   // UPPER LEFT segment
      lights[1 + pos*3][0] = false;   // TOP segment
      lights[2 + pos*3][0] = false;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = false;   // LOWER LEFT segment
      lights[1 + pos*3][1] = true;   // MIDDLE segment
      lights[2 + pos*3][1] = false;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = false;   // BOTTOM segment
    break;
    case ' ':
      lights[0 + pos*3][0] = false;   // UPPER LEFT segment
      lights[1 + pos*3][0] = false;   // TOP segment
      lights[2 + pos*3][0] = false;   // UPPER RIGHT segment
      lights[0 + pos*3][1] = false;   // LOWER LEFT segment
      lights[1 + pos*3][1] = false;   // MIDDLE segment
      lights[2 + pos*3][1] = false;   // LOWER RIGHT segment
      lights[1 + pos*3][2] = false;   // BOTTOM segment
    break;
  }
}   // end code to set numbers on display ************************************************************************


void animate_display() {
  if (millis() > display_frame_time) {
    display_frame++;
    if (display_frame > 30000) {
      display_frame = display_frame - 30000;
    }
    switch (display_mode) {
      case DISPLAY_MODE_STANDBY:
        switch (display_frame % 4) {
          case 0:
            set_led_char(' ', 0);
            set_led_char(' ', 1);
            set_led_char(' ', 2);
          break;
          case 1:
            set_led_char('-', 0);
            set_led_char(' ', 1);
            set_led_char(' ', 2);
          break;
          case 2:
            set_led_char(' ', 0);
            set_led_char('-', 1);
            set_led_char(' ', 2);
          break;
          case 3:
            set_led_char(' ', 0);
            set_led_char(' ', 1);
            set_led_char('-', 2);
          break;
        }
        display_frame_time = millis() + FRAME_RATE_SLOW;
      break;
      case DISPLAY_MODE_KEYPAD:
        if (enter_character < 3) {
          switch (display_frame % 2) {
            case 0:
              set_led_char(' ', enter_character);
            break;
            case 1:
              set_led_char('_', enter_character);
            break;
          }
        }
        if (enter_character > 2) {
          set_led_number(enter_program % 10, 2);
        }
        if (enter_character > 1) {
          set_led_number(enter_program % 100 / 10, 1);
        }
        if (enter_character > 0) {
          set_led_number(enter_program % 1000 / 100, 0);
        }
        display_frame_time = millis() + FRAME_RATE_SLOW;
      break;
      case DISPLAY_MODE_DEMO:
        // second counter code
        set_led_number(millis() % 10000 / 1000, 2);  //thousand millisecond place
        set_led_number(millis() % 100000 / 10000, 1);  //ten-thousand millisecond place
        set_led_number(millis() % 1000000 / 100000, 0);  //hundred-thousand millisecond place
        display_frame_time = millis() + FRAME_RATE_VFAST;
      break;
      case DISPLAY_MODE_DMX:
        if (dmx_display_count >= dmx_display_max) {
          set_led_char('-', 0);
          set_led_char('-', 1);
          set_led_char('-', 2);
        }
        else {
          dmx_display_count++;
          set_led_char('C', 0);
          set_led_number(dmx_display_queue[dmx_display_count % 10] % 100 / 10, 1);
          set_led_number(dmx_display_queue[dmx_display_count % 10] % 10, 2);
        }
        display_frame_time = millis() + FRAME_RATE_SLOW;
      break;
    }
  }
}  // end animate display code **************************************************************************************


void animate_data() {
  if (millis() > data_frame_time1) {

    switch (data_mode) {
      case DATA_MODE_STANDBY:
        for (int i = 0; i < 8; i++) {
          data_status[i][0] = -1;
          data_status[i][1] = -1;
          data_status[i][2] = 0;
          data_status[i][3] = 5;
          data_frame_time[i] = millis();
        }
      break;
      case DATA_MODE_DMX:
        for (int i = 0; i < 71; i++) {
          if (dmx_array[i][0] != 0) {
            if (data_status[data_status_map[i][0]][0] != i) {
              if (data_status[data_status_map[i][0]][3] > data_status_map[i][3]) {
                data_status[data_status_map[i][0]][0] = i;
                data_status[data_status_map[i][0]][1] = data_status_map[i][1];
                data_status[data_status_map[i][0]][2] = data_status_map[i][2];
                data_status[data_status_map[i][0]][3] = data_status_map[i][3];
                data_frame_time[data_status_map[i][0]] = millis();
              }
            }
          }
        }
      break;
    }

    for (int i = 0; i < 8; i++) {
      if (millis() >= data_frame_time[i]) {
        if (data_status[i][1] == -1) {
          if (data_status[i][0] != -1) {
            if (dmx_array[data_status[i][0]][0] == 0) {
              data_status[i][0] = -1;
              data_status[i][1] = 0;
              data_status[i][2] = 0;
              data_status[i][3] = 5;
              data_frame_time[i] = millis();
            }
            else {
              lights[data_lights[i][0]][data_lights[i][1]] = true;
              data_frame_time[i] = millis();
            }
          }
          else {
            lights[data_lights[i][0]][data_lights[i][1]] = true;
            data_frame_time[i] = millis();       
          }
        }
        if (data_status[i][1] == 0) {
          lights[data_lights[i][0]][data_lights[i][1]] = false;
          data_status[i][0] = -1;
          data_status[i][2] = 0;
          data_status[i][3] = 5;
          data_frame_time[i] = millis();
        }
        if (data_status[i][1] > 0) {
          if (data_status[i][1] % 2 == 1) {
            lights[data_lights[i][0]][data_lights[i][1]] = true;
          }
          else {
            lights[data_lights[i][0]][data_lights[i][1]] = false;
          }
          data_status[i][1]--;
          data_frame_time[i] = millis() + data_status[i][2];
        }
      }
    }
    
    data_frame_time1 = millis() + FRAME_RATE_VFAST;
  }

  
}  // end animate data code **************************************************************************************

void animate_connection() {     // animate connection lights code *****************************************
  if (millis() > connection_frame_time) {
    switch (connection_mode) {
      case CONNECTION_MODE_STANDBY:
        lights[0][3] = true;
        lights[3][3] = true;
        lights[5][3] = true;
        lights[7][3] = true;
        lights[9][3] = true;
        lights[1][3] = true;
        lights[2][3] = true;
        lights[4][3] = true;
        lights[6][3] = true;
        lights[8][3] = true;
        lights[9][1] = true;
      break;
      case CONNECTION_MODE_DMX:
        lights[0][3] = controller_connected[0];
        lights[3][3] = controller_connected[1];
        lights[5][3] = controller_connected[2];
        lights[7][3] = controller_connected[3];
        lights[9][3] = controller_connected[4];
        lights[1][3] = controller_connected[5];
        lights[2][3] = controller_connected[6];
        lights[4][3] = controller_connected[7];
        lights[6][3] = controller_connected[8];
        lights[8][3] = controller_connected[9];
        lights[9][1] = controller_connected[10];
      break;
    }
    connection_frame_time = millis() + CONNECTION_REFRESH_RATE;
  }
  
} // end animate connection code ******************************************************************


void read_keypad() {
  // Fills keypad.key[ ] array with up-to 10 active keys.
  // Returns true if there are ANY active keys.
    if (keypad.getKeys())
    {
        for (int i=0; i<LIST_MAX; i++)   // Scan the whole key list.
        {
            if ( keypad.key[i].stateChanged )   // Only find keys that have changed state.
            {
                switch (keypad.key[i].kchar) {  // check to see what button has changed and take appropraite action

                  //  ***************** keypad numbers ***************************
                    case 11:                    // type 0
                      if (keypad.key[i].kstate == PRESSED) {
                        type_number(0);
                      }
                    break;
                    case 42:                    // type 1
                      if (keypad.key[i].kstate == PRESSED) {
                        type_number(1);
                      }
                    break;  
                    case 32:                    // type 2
                      if (keypad.key[i].kstate == PRESSED) {
                        type_number(2);
                      }
                    break;  
                    case 22:                    // type 3
                      if (keypad.key[i].kstate == PRESSED) {
                        type_number(3);
                      }
                    break;  
                    case 41:                    // type 4
                      if (keypad.key[i].kstate == PRESSED) {
                        type_number(4);
                      }
                    break;  
                    case 31:                    // type 5
                      if (keypad.key[i].kstate == PRESSED) {
                        type_number(5);
                      }
                    break;  
                    case 21:                    // type 6
                      if (keypad.key[i].kstate == PRESSED) {
                        type_number(6);
                      }
                    break;  
                    case 40:                    // type 7
                      if (keypad.key[i].kstate == PRESSED) {
                        type_number(7);
                      }
                    break;  
                    case 30:                    // type 8
                      if (keypad.key[i].kstate == PRESSED) {
                        type_number(8);
                      }
                    break;  
                    case 20:                    // type 9
                      if (keypad.key[i].kstate == PRESSED) {
                        type_number(9);
                      }
                    break;

                    // ***************** keypad commands ***********************************
                    case 10:                    // CMD button
                      if (keypad.key[i].kstate == PRESSED) {
                        if (input_mode == INPUT_MODE_KEYPAD) {
                          input_mode = INPUT_MODE_STANDBY;
                          change_program(active_program);
                        }
                        else {
                          input_mode = INPUT_MODE_KEYPAD;
                          display_mode = DISPLAY_MODE_KEYPAD;
                          enter_program = 0;
                          enter_character = 0;
                          set_led_char(' ', 0);
                          set_led_char(' ', 1);
                          set_led_char(' ', 2);
                        }
                      }
                    break;
                    case 12:                    // ENTER button
                      if (keypad.key[i].kstate == PRESSED) {
                        if (input_mode == INPUT_MODE_KEYPAD) {
                          input_mode = INPUT_MODE_STANDBY;
                          if (enter_character > 2){
                            change_program(enter_program);
                          }
                          else {
                            change_program(active_program);
                          }
                        }
                      }
                    break;
                 }
            }
        }
     }          // end keypad block
}   // end keypad code *******************************************************************************************

void type_number(int number) {
  if (input_mode == INPUT_MODE_KEYPAD) {
    switch (enter_character) {
      case 0:
        enter_program = enter_program + 100 * number;
        enter_character++;
        display_frame_time = millis();
      break;
      case 1:
        enter_program = enter_program + 10 * number;
        enter_character++;
        display_frame_time = millis();
      break;
      case 2:
        enter_program = enter_program + number;
        enter_character++;
        display_frame_time = millis();
      break;
    }
  }
}  // end type number code ********************************************************************************


void change_program(int program) {
  switch (program) {
    case 0:
      active_program = program;
      input_mode = INPUT_MODE_STANDBY;
      data_mode = DATA_MODE_STANDBY;
      display_mode = DISPLAY_MODE_STANDBY;
      connection_mode = CONNECTION_MODE_STANDBY;
    break;
    case 101:
      active_program = program;
      input_mode = INPUT_MODE_STANDBY;
      data_mode = DATA_MODE_DMX;
      display_mode = DISPLAY_MODE_DMX;
      connection_mode = CONNECTION_MODE_DMX;
      for (int i = 0; i < 8; i++) {
        data_status[i][0] = -1;
        data_status[i][1] = 0;
        data_status[i][2] = 0;
        data_status[i][3] = 5;
      }      
    break;
    case 901:
      active_program = program;
      input_mode = INPUT_MODE_STANDBY;
      data_mode = DATA_MODE_STANDBY;
      display_mode = DISPLAY_MODE_DEMO;
      connection_mode = CONNECTION_MODE_STANDBY;
    break;
    default:
      switch (active_program) {
        case 0:
        input_mode = INPUT_MODE_STANDBY;
        data_mode = DATA_MODE_STANDBY;
        display_mode = DISPLAY_MODE_STANDBY;
        connection_mode = CONNECTION_MODE_STANDBY;
      break;
      case 101:
        active_program = program;
        input_mode = INPUT_MODE_STANDBY;
        data_mode = DATA_MODE_DMX;
        display_mode = DISPLAY_MODE_DMX;
        connection_mode = CONNECTION_MODE_DMX;
      break;
      case 901:
        input_mode = INPUT_MODE_STANDBY;
        data_mode = DATA_MODE_STANDBY;
        display_mode = DISPLAY_MODE_DEMO;
        connection_mode = CONNECTION_MODE_STANDBY;
      break;
      }
    break;
  }
}  // end change program code *********************************************************************************

void DMX_read() {
        static  uint16_t DmxCount;
        uint8_t  USARTstate= UART0_S1;    //get state before data!
        uint8_t  DmxByte   = UART0_D;          //get data
        uint8_t  DmxState  = gDmxState; //just load once from SRAM to increase speed
 
        if (USARTstate & UART_S1_FE)               //check for break
        {
                DmxCount = DmxAddress;         //reset channel counter (count channels before start address)
                gDmxState= BREAK;
                Errors++;
                UART0_S1 = UART0_S1 & UART_S1_FE;
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
      dmx_display_max++;
      dmx_display_queue[dmx_display_max % 10] = channel;
      dmx_array[channel][1] = dmx_array[channel][0];
      if (dmx_array[channel][0] != 0) {
        add_data_to_transmit_queue(101, channel, 1);
        add_data_to_transmit_queue(101, channel, 1);
      }
      else {
        add_data_to_transmit_queue(101, channel, 0);
        add_data_to_transmit_queue(101, channel, 0);
      }
    }
  }
}

void add_data_to_transmit_queue (int code, int data0, int data1) {
  RS_485_TX[dmx_transmit_queue_last][0] = RS_485_TX_START; // number signifying the start of a new data stream
  RS_485_TX[dmx_transmit_queue_last][1] = byte(code);
  switch (code) {
    case 220:
      RS_485_TX[dmx_transmit_queue_last][2] = byte(data0);
      RS_485_TX[dmx_transmit_queue_last][3] = RS_485_TX_STOP;
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
  
  controller_connected[1] = transmit_enable;
  
  // check to see if timeout time reached and waiting for data
  if (millis() > RS_485_delay_time && !transmit_enable) {
    transmit_enable = true;
    data_stream_on = false; // reset active transmission to start new transmission
    data_stream_pos = 0;
  }

  // code for sending data ****** 
  if (transmit_enable) {
    if (dmx_transmit_queue_pos == dmx_transmit_queue_last) {  // poll next controller in queue
      controller_poll_position++;
      if (controller_poll_position > 10) {
        controller_poll_position = 0;
      }
      for (int i=0; i<11; i++) {
        if (controller_connected[i]) {
          add_data_to_transmit_queue(220, controller_address[i], 0);
        }
      }
      add_data_to_transmit_queue(220, controller_address[controller_poll_position], 0);  
    }
    if (data_stream_on) {
      if (Serial2.availableForWrite()) {
        if (RS_485_TX[dmx_transmit_queue_pos][data_stream_pos] != RS_485_TX_STOP) {            
          Serial2.write(RS_485_TX[dmx_transmit_queue_pos][data_stream_pos]);
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
          if (RS_485_TX[dmx_transmit_queue_pos][1] == 220) {
            transmit_enable = false;
            RS_485_delay_time = millis() + RS_485_TIMEOUT;
            switch (RS_485_TX[dmx_transmit_queue_pos][2]) {
              case 10:
                controller_connected[6] = false;
              break;
              case 20:
                controller_connected[7] = false;
              break;
              case 30:
                controller_connected[8] = false;
              break;
              case 40:
                controller_connected[9] = false;
              break;
              case 50:
                controller_connected[10] = false;
              break;
            }
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
    if (Serial2.available()) {
      int rx = Serial2.read();
      if (rx == RS_485_TX_START) {
        data_stream_pos = 0;
      }
      RS_485_RX[data_stream_pos] = rx;
      if (data_stream_pos == 1 && rx == 215) {    // receive token from controller
        transmit_enable = true;
      }

      if (data_stream_pos > 1) {
        if (RS_485_RX[1] == 212) {
          switch (RS_485_RX[2]) {
            case 10:
              controller_connected[6] = true;
            break;
            case 20:
              controller_connected[7] = true;
            break;
            case 30:
              controller_connected[8] = true;
            break;
            case 40:
              controller_connected[9] = true;
            break;
            case 50:
              controller_connected[10] = true;
            break;
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


void loop() {

  read_keypad();

  lookfor_dmx_changes();

  transmit_receive();

  animate_data();
  animate_display();
  animate_connection();
  
  if (millis() - refresh_last > REFRESH_RATE) {
    refresh_lights();
  }

  
  /*
  if (millis() - status_time > UPDATE_STATUS) {
    Serial.print(DmxRxField[0]);
    Serial.print(DmxRxField[1]);
    Serial.print(DmxRxField[2]);
    Serial.print(DmxRxField[3]);
    Serial.print(DmxRxField[4]);
    Serial.print(DmxRxField[5]);
    Serial.print(Errors);
    Serial.println(BytesRead);
    status_time = millis();
  }
  */
  
}

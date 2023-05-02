// Board definition file
#include <EEPROM.h>
#include <Wire.h>                           // needed for I2C
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// ----------------------------------------------------------------------------------------------------------
// uncomment the following flags to enable features

// To enable temperature probe, uncomment next line
#define TEMPERATUREPROBE 1

// To enable backlash in this firmware, uncomment the next line
#define BACKLASH 1

// To use this controller locally via USB serial port
#define LOCALSERIAL 1

// Enable Serial Debug
//#define SERIALDEBUG 1

// Enable Wifi
#define WIFI 1

// Enable Webserver
#define WEBSERVER

#ifdef WIFI
// WIFI SSID & Password
const char* ssid = "Seattle";  // Enter your SSID here
const char* password = "ClearSkies!";  //Enter your Password here
#endif

#ifdef WEBSERVER
#define WEBPORT             80
#endif

// Define motor micro step mode
#define STEP1               1               // Full step
#define STEP2               2               // Half step
#define STEP4               4
#define STEP8               8
#define STEP16              16
#define STEP32              32

// There is no hardware jumper to change the step mode
byte USERSTEPMODE = STEP4;


// ----------------------------------------------------------------------------------------------------------
// GLOBAL DEFINES YOU CAN CHANGE - THESE CAN ONLY BE CONFIGURED AT COMPILE TIME
// CHANGE AT YOUR PERIL
#define BUFFERSIZE    64                    // maximum number of chars in a message will be Firmware Version String
#define ESPBUFFERSIZE 128
#define SERVERPORT    2020
#define LCDUPDATESTEPCOUNT  15              // the number of steps moved which triggers an lcd update when moving, do not make too small
#define TEMPREFRESHRATE     1300L           // refresh rate between temperature conversions unless an update is requested via serial command
#define SERIALPORTSPEED     115200          // 9600, 14400, 19200, 28800, 38400, 57600, 115200
#define IF_SERIAL           0               // Use Serial port for responses
#define IF_HTTP             1               // Use HTTP for responses
#define IF_IP               2               // Use IP TCP/UDP for responses
#define IF_BC               3               // Broadcast to all the interfaces

// Hardware pin definition
#define DRV8825DIR          13              // D7
#define DRV8825STEP         12              // D6
#define DRV8825ENABLE       14              // D5
#define TEMPPIN             10              // SD3
#define DRV8825M0           5               // D1
#define DRV8825M1           4               // D2
#define DRV8825M2           0               // D3
#define LEDPIN              16              // D0
#define I2CDATAPIN          5               // D1
#define I2CCLOCKPIN         4               // D2
#define OLED_ADDR           0x3C            // some OLED displays maybe at 0x3F, use I2Cscanner to find the correct address
#define EEPROMSIZE          512             // use 512 locations 
#define EEPROMWRITEINTERVAL 10000L          // interval in milliseconds to wait after a move before writing settings to EEPROM, 10s
#define VALIDDATAFLAG       99              // 01100011 valid eeprom data flag
#define SLOW                0               // motor speeds
#define MED                 1
#define FAST                2



#define TEMP_PRECISION      10              // Set the default DS18B20 precision to 0.25 of a degree 9=0.5, 10=0.25, 11=0.125, 12=0.0625
#define MOTORSPEEDCHANGETHRESHOLD   200
#define MOTORPULSETIME      10              // drv8825 requires minimum 2uS pulse to step
#define SETTLEMULTIPLIER    5
#define MOVINGIN            0
#define MOVINGOUT           1
#define FOCUSERUPPERLIMIT   55000L          // arbitary focuser limit up to 2000000000
#define FOCUSERLOWERLIMIT   5760L           // lowest value that maxsteps can be
#define DEFAULTSTEPSIZE     11.3            // This is the default setting for the step size in microns
#define PAGEDISPLAYTIME     5000

#define MOTORSLOWDELAY      8
#define MOTORMEDIUMDELAY    4
#define MOTORFASTDELAY      2

#ifdef TEMPERATUREPROBE
  #include <OneWire.h>                      // needed for DS18B20 temperature probe
  #include "myDallasTemperature.h"          // needed for DS18B20 temperature probe
  //#include <DallasTemperature.h>          // needed for DS18B20 temperature probe
  OneWire oneWirech1(TEMPPIN);              // setup temperature probe
  DallasTemperature sensor1(&oneWirech1);
  DeviceAddress tpAddress;                  // holds address of the temperature probe
#endif

#ifdef LOCALSERIAL
  #define MAXCOMMAND 15
  char serial_buffer[MAXCOMMAND];           // buffer for serial data
  byte idx = 0;                             // index of a char in the serial_buffer
  bool eoc = false;
#endif

#ifdef BACKLASH                             // backlash compensation
  byte movedirection;                       // holds direction of new planned move
#endif

char programName[]    = "myFP2E.DRV8825";
char programVersion[] = "999";
char ProgramAuthor[]  = "Bryan";
char ontxt[]          = "ON ";
char offtxt[]         = "OFF";
char coilpwrtxt[]     = "Coil power  =";
char revdirtxt[]      = "Reverse Dir =";


// to handle eeprom
int eeprom_current_addr;                  // will be address in eeprom of the data stored
bool eeprom_write_flag;                   // should we update values in eeprom
bool eeprom_found;
byte eeprom_data_size;
int eeprom_nlocations;
long eeprom_current_timestamp;
long eeprom_last_timestamp;               // used as a delay whenever the EEPROM settings need to be updated

long focuser_current_position;            // current focuser position
long focuser_target_position;             // target position
long focuser_new_position;                // used to calculate and validate target position requests
long temp_current_timestamp;
long temp_last_timestamp;                 // holds time of last conversion
double temp_current_value;                // temperature value for probe
double temp_tc_last_value;                // holds previous temperature value - used if ismoving and if temp request < 10s apart
double temp_tc_start_value;               // when using temperature compensation, holds the start temperature value
byte motor_step_delay;
byte focuser_motor_speed;
bool movestarted;
bool motor_turning;                       // is the motor currently moving
int motor_settle_time;               // determine the delay time after each motor move command
bool displayenabled;                      // used to enable and disable the display
bool temp_probe1;                         // indicate if there is a probe attached to myFocuserPro2
bool temp_tc_started;                     // indicates if temperature compensation is enabled
bool temp_request_flag;
bool motor_coil_power_state;
uint8_t MICROSTEPSPERMOVE = 1;

// these are stored in EEPROM
struct config_t {
  byte validdata;                         // if this is 99 then data is valid
  long last_position;                     // last focuser position
  long max_step;                          // max steps
  double motor_step_size;                 // the step size in microns, ie 7.2 - value * 10, so real stepsize = stepsize / 10 (maxval = 25.6)
  byte motor_delay_after_move;            // delay after movement is finished (maxval=256)
  byte motor_backlash_steps_in;           // number of backlash steps to apply for IN moves
  byte motor_backlash_steps_out;          // number of backlash steps to apply for OUT moves
  byte temp_coefficient;                  // steps per degree temperature coefficient value (maxval=256)
  bool motor_coil_power_enabled;
  bool motor_reversedirection;
  bool motor_backlash_in_enabled;         // enable or disable backlash compensation for IN
  bool motor_backlash_out_enabled;        // enable or disable backlash compensation for OUT
  bool motor_step_size_enabled;           // if 1, controller returns step size
  bool temp_mode;                         // temperature display mode, Celcius=1, Fahrenheit=0
  bool lcdupdateonmove;                   // update position on lcd when moving
  bool tc_enabled;                        // indicates if temperature compensation is enabled
  bool tc_direction;
  bool move_direction;                    // keeps track of last focuser move direction
  byte motor_speed;
} myfocuser;


#ifdef TEMPERATUREPROBE
  void temp_get_address();
  void temp_request();
  void temp_read();
#endif

void scheduled_tasks();
void software_reboot();
void eeprom_init ();
void write_EEPROM();
void eeprom_set_write_flag();
void send_str(String str, byte interface);
void send_new_line(byte interface);
void focuser_set_defaults();
void focuser_restore_settings();
void temp_init ();
void temp_compensation();
void motor_init();
void motor_set_delay();
void motor_enable_output();
void motor_disable_output();
void motor_set_step_mode();
void motor_turn_anti_clockwise();
void motor_turn_clockwise();
void motor_turn();
void motor_backlash_compensation();
void motor_set_target_position(long new_pos);
void motor_halt();
void command_processing(char *cmd_buffer, byte interface);
long convert_dec_str_to_long(String str);
uint8_t convert_dec_str_to_int(String str);

#ifdef WIFI
void wifi_init();
#endif

#ifdef WEBSERVER
void webserver_init();
void webserver_event();
String compose_HTML_body();
void webserver_handle_M_m500();
void webserver_handle_motor_turn(int steps);
void webserver_handle_M_m500();
void webserver_handle_M_m100();
void webserver_handle_M_m10();
void webserver_handle_M_m1();
void webserver_handle_M_p500();
void webserver_handle_M_p100();
void webserver_handle_M_p10();
void webserver_handle_M_p1();
void webserver_handle_M_Go();
void webserver_handle_M_Halt();
void webserver_handle_get_current_position();
void webserver_handle_get_current_temperature();
#endif

#ifdef LOCALSERIAL
void serial_send_str(String str);
void serial_send_new_line();
void serial_event();
#endif

// HTML & CSS contents for web server
String HTML_top = "<!DOCTYPE html>\
<html>\
<head>\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
<style>\
html {font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\
body {background-color: #cc0000;}\
.input {border: none; color: black; padding: 15px; width: 180px; font-size: 25px; margin: 5px; cursor: pointer;}\
.button {border: none; color: white; padding: 15px; width: 100px; font-size: 25px; margin: 5px; cursor: pointer; text-decoration: none;}\
.blue_button {background-color: #195B6A;}\
.red_button {background-color: #800000;}\
@media screen and (max-width: 600px){.button {float: none;}}</style>\
<script>\
setInterval(function() {getCurrentPosition(); getCurrentTemperature();}, 5000);\
function getCurrentPosition() {\
var xhttp = new XMLHttpRequest();\
xhttp.onreadystatechange = function() {if (this.readyState == 4 && this.status == 200) {document.getElementById(\"c_pos\").value = this.responseText;}};\
xhttp.open(\"GET\", \"G-C\", true); xhttp.send();\
}\
function getCurrentTemperature() {\
var xhttp = new XMLHttpRequest();\
xhttp.onreadystatechange = function() {if (this.readyState == 4 && this.status == 200) {document.getElementById(\"c_temp\").value = this.responseText;}};\
xhttp.open(\"GET\", \"G-T\", true); xhttp.send();\
}\
function goto() {var n_pos = document.getElementById(\"n_pos\").value; if (n_pos) {let xhttp = new XMLHttpRequest();\
xhttp.open(\"POST\", \"M-Go\", true);\
xhttp.setRequestHeader(\"Content-type\", \"application/x-www-form-urlencoded\");\
xhttp.send(\"n_pos=\" + n_pos); document.getElementById(\"n_pos\").value = '';}}\
function moveNSteps(element) {let xhttp = new XMLHttpRequest(); xhttp.open(\"GET\", element.id, true); xhttp.send();}\
</script>\
</head>\
<body>\
<h1>ESP8266 Electronic Focuser</h1>";

String HTML_body;

String HTML_bottom ="</body>\
</html>";


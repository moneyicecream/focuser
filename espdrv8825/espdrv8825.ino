// This ESP8266 Electronic focuser is an open soure project
// The implementation is based on the myFocuserPro2 project at: https://sourceforge.net/projects/arduinoascomfocuserpro2diy/
// The base firmware for this implementation is 2.25
// Please reach out to Mr. Robert Brown the obtain a license agreement before redistributing this implementation
// Modified by Bryan Phan
// Change logs:
// 2020-Dec-19: - Reorganized source code for better code reuse and execution
//              - Added web server implementation for remote control via the web interface


#include "espdrv8825.h"

#ifdef WEBSERVER
ESP8266WebServer server(WEBPORT);
#endif

// ----------------------------------------------------------------------------------------------------------
// FIRMWARE CODE START
void setup() {
  EEPROM.begin(EEPROMSIZE);

  // Start I2C
  //Wire.begin(I2CDATAPIN, I2CCLOCKPIN);
  Wire.begin();

  // open serial port if LOCALSERIAL
#ifdef LOCALSERIAL
  Serial.begin(SERIALPORTSPEED);
#endif

  motor_turning = 0;
  movestarted = 0;
  eeprom_init();

  // start temp probe
#ifdef TEMPERATUREPROBE
  temp_init();
#endif

  motor_init();

#ifdef SERIALDEBUG
 send_str("Focuser initialized", IF_SERIAL);
#endif

#ifdef WIFI
  wifi_init();
#endif

#ifdef WEBSERVER
  webserver_init();
#endif  

}


// Main program
void loop() {
  scheduled_tasks();

  // Check if the motor needs to turn
  if (focuser_current_position != focuser_target_position) {
    motor_turn();
  }

  if (temp_probe1 == 1) {
    temp_current_timestamp = millis();
    // see if the temperature needs updating - done automatically every 5s
    if (((temp_current_timestamp - temp_last_timestamp) > TEMPREFRESHRATE) || (temp_current_timestamp < temp_last_timestamp)) {
      temp_last_timestamp = millis();  // update
      if (temp_request_flag == 0) {
        temp_read();
        temp_request_flag = 1;
      } 
      else {
        temp_request();
        temp_request_flag = 0;
      }
    }

    // check for temperature compensation;
    if (myfocuser.tc_enabled == 1) {
      temp_compensation();
    }  // end of check for tc_enabled == 1)
  }

  // is it time to update EEPROM settings?
  if (eeprom_write_flag == 1) {
    // decide if we have waited 10s (value of EEPROMWRITEINTERVAL) after the last myfocuser key variable update, if so, update the EEPROM
    eeprom_current_timestamp = millis();
    if (((eeprom_current_timestamp - eeprom_last_timestamp) > EEPROMWRITEINTERVAL) || (eeprom_current_timestamp < eeprom_last_timestamp)) {
      myfocuser.validdata = VALIDDATAFLAG;
      myfocuser.last_position = focuser_current_position;
      write_EEPROM();  // update values in EEPROM
      eeprom_write_flag = 0;
      eeprom_last_timestamp = eeprom_current_timestamp;  // update the timestamp
    }
  }
}

void scheduled_tasks() {
// Handle commands via the serial interface
#ifdef LOCALSERIAL
  serial_event();
#endif

// Handle web client requests
#ifdef WEBSERVER
  server.handleClient();             
#endif
}


// Initializing EEPROM and read the focuser config
void eeprom_init () {
  eeprom_write_flag = 0;

  // find focuser data
  eeprom_current_addr = 0;
  eeprom_found = 0;
  eeprom_data_size = (byte)sizeof(myfocuser);
  eeprom_nlocations = EEPROMSIZE / eeprom_data_size;

  for (int lp1 = 0; lp1 < eeprom_nlocations; lp1++) {
    int addr = lp1 * eeprom_data_size;
    EEPROM.get(addr, myfocuser);
    delay(5);
    if (myfocuser.validdata == VALIDDATAFLAG)  // check to see if the data is valid
    {
      eeprom_current_addr = addr;  // data was erased so write some default values
      eeprom_found = 1;
      delay(5);
      break;
    }
  }

  if (eeprom_found == 1) {
    delay(5);
    EEPROM.get(eeprom_current_addr, myfocuser);
    myfocuser.validdata = 0;
    write_EEPROM();           // update values in EEPROM
    eeprom_current_addr += eeprom_data_size;  // goto next free address and write data
    // bound check the eeprom storage and if greater than last index [0-EEPROMSIZE-1] then set to 0
    if (eeprom_current_addr >= (eeprom_nlocations * eeprom_data_size))
      eeprom_current_addr = 0;
    myfocuser.validdata = VALIDDATAFLAG;
    write_EEPROM();  // update values in EEPROM
  } 
  else {
    delay(5);
    eeprom_current_addr = 0;
    focuser_set_defaults();
  }
  eeprom_last_timestamp = millis();
}

// Initializing Temperature Probe
void temp_init () {
  temp_current_value = 20.0;          // start temperature sensor DS18B20
  temp_tc_last_value = 20.0;
  temp_probe1 = 0;                    // set probe indicator NOT FOUND
  temp_tc_started = 0;                // temperature compensation is off
  temp_last_timestamp = millis();
  temp_request_flag = 1;
  // Configure GPIO pin for temperature probe
  pinMode(TEMPPIN, INPUT);
  sensor1.begin();  // start the temperature sensor1
  delay(5);
  temp_probe1 = sensor1.getDeviceCount();  // should return 1 if probe connected
  temp_get_address();
  delay(5);
  if (temp_probe1 == 1) {
    sensor1.setResolution(tpAddress, 10);  // set probe resolution
    temp_request();
    delay(600 / (1 << (12 - 10)));  // should enough time to wait
    temp_read();
  }
  temp_request_flag = 0;
}

// Initializing Motor Driver
void motor_init() {
    // Configure GPIO pins for driver board
  pinMode(DRV8825DIR, OUTPUT);
  pinMode(DRV8825STEP, OUTPUT);
  pinMode(DRV8825ENABLE, OUTPUT);
  pinMode(DRV8825M0, OUTPUT);
  pinMode(DRV8825M1, OUTPUT);
  pinMode(DRV8825M2, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, HIGH);

  // assign to current working values
  focuser_current_position = myfocuser.last_position;
  focuser_target_position = focuser_current_position;
  focuser_motor_speed = myfocuser.motor_speed;
  motor_set_step_mode();
  motor_set_delay();
  motor_settle_time = SETTLEMULTIPLIER * int(myfocuser.motor_delay_after_move);
  delay(5);

  if (myfocuser.motor_coil_power_enabled == 0)
    motor_disable_output();  // release the stepper coils to save power
  else
    motor_enable_output();

  myfocuser.tc_enabled = 0;  // tc disabled
  myfocuser.tc_direction = myfocuser.tc_direction & 0x01;
}


// set focuser settings to default values
void focuser_set_defaults() {
  myfocuser.validdata = VALIDDATAFLAG;
  myfocuser.max_step = 11520L;
  myfocuser.last_position = 0L;
  myfocuser.motor_coil_power_enabled = 1;
  myfocuser.motor_reversedirection = 0;
  myfocuser.motor_step_size_enabled = 1;                // default state is step size OFF
  myfocuser.motor_step_size = DEFAULTSTEPSIZE;
  myfocuser.motor_delay_after_move = 100;
  myfocuser.motor_backlash_steps_in = 1;
  myfocuser.motor_backlash_steps_out = 1;
  myfocuser.motor_backlash_in_enabled = 45;
  myfocuser.motor_backlash_out_enabled = 45;
  myfocuser.temp_coefficient = 0;
  myfocuser.tc_direction = 0;                           // tc direction 0: move in, tc direction 1 : move out
  myfocuser.temp_mode = 0;                              // default is celsius
  myfocuser.tc_enabled = 0;                             // tc disabled
  myfocuser.lcdupdateonmove = 1;
  myfocuser.move_direction = MOVINGIN;
  myfocuser.motor_speed = FAST;
  write_EEPROM();                                       // update values in EEPROM
  motor_turning = 0;
  movestarted = 0;
  delay(5);
}


void focuser_restore_settings() {
  EEPROM.get(eeprom_current_addr, myfocuser);
}


// Wifi initialization
void wifi_init() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
#ifdef SERIALDEBUG
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#endif  
}


// Webserver initialization
void webserver_init() {
  server.on("/", webserver_event);
  server.on("/M-500", webserver_handle_M_m500);
  server.on("/M-100", webserver_handle_M_m100);
  server.on("/M-10", webserver_handle_M_m10);
  server.on("/M-1", webserver_handle_M_m1);
  server.on("/M+500", webserver_handle_M_p500);
  server.on("/M+100", webserver_handle_M_p100);
  server.on("/M+10", webserver_handle_M_p10);
  server.on("/M+1", webserver_handle_M_p1);
  server.on("/M-Go", webserver_handle_M_Go);
  server.on("/M-Halt", webserver_handle_M_Halt);
  server.on("/G-C", webserver_handle_get_current_position);
  server.on("/G-T", webserver_handle_get_current_temperature);
  server.begin();
}


// Web event handler for Root "/"
void webserver_event() {
  HTML_body = compose_HTML_body();
  server.send(200, "text/html", HTML_top + HTML_body + HTML_bottom);
}


// Web event handler for the Move -500 command: id="/M-500"
void webserver_handle_M_m500() {
  webserver_handle_motor_turn(-500);
}


// Web event handler for the Move -100 command: id="/M-100"
void webserver_handle_M_m100() {
  webserver_handle_motor_turn(-100);
}


// Web event handler for the Move -10 command: id="/M-10"
void webserver_handle_M_m10() {
  webserver_handle_motor_turn(-10);
}


// Web event handler for the Move -1 command: id="/M-1"
void webserver_handle_M_m1() {
  webserver_handle_motor_turn(-1);
}


// Web event handler for the Move +500 command: id="/M+500"
void webserver_handle_M_p500() {
  webserver_handle_motor_turn(500);
}


// Web event handler for the Move +100 command: id="/M+100"
void webserver_handle_M_p100() {
  webserver_handle_motor_turn(100);
}


// Web event handler for the Move +10 command: id="/M+10"
void webserver_handle_M_p10() {
  webserver_handle_motor_turn(10);
}


// Web event handler for the Move +1 command: id="/M+1"
void webserver_handle_M_p1() {
  webserver_handle_motor_turn(1);
}


// Web event handler for the Goto command: id="/M+Go"
void webserver_handle_M_Go() {
  String new_pos_str = server.arg("n_pos");
#ifdef SERIALDEBUG
  serial_send_str(new_pos_str);
#endif
  long new_pos = convert_dec_str_to_long(new_pos_str);
  webserver_handle_motor_turn(new_pos - focuser_current_position);
}


// Web event handler for the Halt command: id="/M+Halt"
void webserver_handle_M_Halt() {
  motor_halt();
  webserver_event();
}


// Web event handler for the Move commands
void webserver_handle_motor_turn(int steps) {
  motor_set_target_position(focuser_current_position + steps);
  motor_turning = true;
  webserver_event();
}


// Web event handler for the current position requery
void webserver_handle_get_current_position() {
  server.send(200, "text/plane", String(focuser_current_position));
}


// Web event handler for the current temperature requery
void webserver_handle_get_current_temperature() {
  server.send(200, "text/plane", String(temp_current_value));
}


// Composing the body of the HTML page
String compose_HTML_body() {
  String html_body = "<button class=\"input\" style=\"width: 210px; color: black; background-color: #cc0000; font-weight: bold; text-align:left; cursor: default;\">Temperature:</button>";

  html_body += "<input class=\"input\" id=\"c_temp\" value=\"" + String(temp_current_value) + "\" disabled style=\"color: white; font-weight: bold; text-align: right; cursor: default;\"><br>";

  html_body += "<button class=\"input\" style=\"width: 210px; color: black; background-color: #cc0000; font-weight: bold; text-align:left; cursor: default;\">Position:</button>";

  html_body += "<input class=\"input\" id=\"c_pos\" value=\"" + String(focuser_current_position) + "\" disabled style=\"color: white; font-weight: bold; text-align: right; cursor: default;\"><br>";

  html_body += "<input class=\"input\" type=\"number\" id=\"n_pos\" name=\"n_pos\" placeholder=\"Target Position\">\
<button class=\"button blue_button\" id=\"M-Go\" onclick=\"goto()\">GOTO</button>\
<button class=\"button red_button\" id=\"M-Halt\" onclick=\"moveNSteps(this)\">Halt</button><br>\
<button class=\"button blue_button\" id=\"M-500\" onclick=\"moveNSteps(this)\">-500</button>\
<button class=\"button blue_button\" id=\"M-100\" onclick=\"moveNSteps(this)\">-100</button>\
<button class=\"button blue_button\" id=\"M-10\" onclick=\"moveNSteps(this)\">-10</button>\
<button class=\"button blue_button\" id=\"M-1\" onclick=\"moveNSteps(this)\">-1</button><br>\
<button class=\"button blue_button\" id=\"M+1\" onclick=\"moveNSteps(this)\">+1</button>\
<button class=\"button blue_button\" id=\"M+10\" onclick=\"moveNSteps(this)\">+10</button>\
<button class=\"button blue_button\" id=\"M+100\" onclick=\"moveNSteps(this)\">+100</button>\
<button class=\"button blue_button\" id=\"M+500\" onclick=\"moveNSteps(this)\">+500</button>";

  return html_body;
}


// Command processing
void command_processing(char cmd_buffer[], uint8_t interface) {
  uint8_t cmd_buffer_len;
  long pos;
  String buffer;
  String cmd_key_str;
  uint8_t cmd_key_num;
  String cmd_value_str;
  int cmd_value_num;


  buffer = String(cmd_buffer);
  cmd_buffer_len = (byte)buffer.length();

  // a valid command with no parameters, ie, :01#
  if (cmd_buffer_len == 2) {
    cmd_key_str = buffer;  
  } 

  // this command has parameters
  else if (cmd_buffer_len > 2) {   
    cmd_key_str = buffer.substring(0, 2);  
    cmd_value_str = buffer.substring(2, cmd_buffer_len);
  }
  else return;
  
  cmd_key_num = convert_dec_str_to_int(cmd_key_str);
  switch (cmd_key_num) {
    case 0:  // :00#     Pxxxx#    get current focuser position
      buffer = "P" + String(focuser_current_position) + "#";
      send_str(buffer, interface);
      break;
    case 1:  // :01#     Ixx#      get motor moving status - 01 if moving, 00 otherwise
      if (motor_turning == true)
        send_str("I01#", interface);
      else
        send_str("I00#", interface);
      break;
    case 2:  // :02#     EOK#    get motor controller status - Controller Response to "Are we connected"- also see Wifi
      send_str("EOK#", interface);
      break;
    case 3:  // :03#     Fxxx#   get firmware version string
      buffer = "F" + String(programVersion) + "#";
      send_str(buffer, interface);
      break;
    case 4:  // :04#     FString#  get firmware version string (Fprogram name, version, #)
      send_str("F", interface);
      send_str(String(programName), interface);
      send_new_line(interface);
      send_str((String(programVersion) + "#"), interface);
      break;
    case 6:  // :06#     Zxxxxxx#  get temperature as a double XXXX
      // Send the command to get temperatures from DS18B20 probe
      // is there a probe?
      // there is not a probe so just return 20
      buffer = "Z" + String(temp_current_value, 3) + "#";
      send_str(buffer, interface);
      break;
    case 8:  // :08#     Mxxxxxx#  Get MaxStep, returns XXXXXX
      buffer = "M" + String(myfocuser.max_step) + "#";
      send_str(buffer, interface);
      break;
    case 10:  // :10#    Yxxxxxx#  Get MaxIncrement, returns xxxxxx
      buffer = "Y" + String(myfocuser.max_step) + "#";
      send_str(buffer, interface);
      break;
    case 11:  // :11#   Oxx#      Get coil pwr setting (00 = coils released after move, 01 = coil pwr on after move)
      buffer = "O" + String(myfocuser.motor_coil_power_enabled) + "#";
      send_str(buffer, interface);
      break;
    case 13:  // :13#   Rxx#      Get reverse direction setting, 00 off, 01 on
      buffer = "R" + String(myfocuser.motor_reversedirection) + "#";
      send_str(buffer, interface);
      break;
    case 21:  // :21#    Qxx#    get temperature probe resolution setting (9, 10, 11 or 12)
      buffer = "Q" + String(TEMP_PRECISION) + "#";
      send_str(buffer, interface);
      break;
    case 24:  // :24#   1x#       Get state of Temperature Compensation, 0=disabled, 1=enabled
      buffer = "1" + String(myfocuser.tc_enabled) + "#";
      send_str(buffer, interface);
      break;
    case 25:  // :25#   Ax#       Get if Temperature Compensation available 0=No, 1=Yes
      if (temp_probe1 == 1)
        buffer = "A1#";  // this focuser supports temperature compensation
      else
        buffer = "A0#";  
      send_str(buffer, interface);
      break;
    case 26:  // :26#   Bxxx#   get Temperature Coefficient (in steps per degree)
      buffer = "B" + String(myfocuser.temp_coefficient) + "#";
      send_str(buffer, interface);
      break;
    case 29:  // :29#     Sxx#    get stepmode, returns XX#
      buffer = "S" + String(USERSTEPMODE) + "#";
      send_str(buffer, interface);
      break;
    case 32:  // :32#   Ux#       Get if step_size is enabled in controller (1 or 0, 0/1)
      buffer = "U" + String(myfocuser.motor_step_size_enabled) + "#";
      send_str(buffer, interface);
      break;
    case 33:  // :33#   Txxxxx#   Get step size in microns (if enabled by controller)
      buffer = "T" + String(myfocuser.motor_step_size) + "#";
      send_str(buffer, interface);
      break;
    case 34:  // :34#   Xxxxxx#   get the time that an LCD screen is displayed for (in milliseconds, eg 2500 = 2.5seconds
      buffer = "X" + String(PAGEDISPLAYTIME) + "#";
      send_str(buffer, interface);
      break;
    case 37:  // :37#   Dxx#      Get Display status 0=disabled, 1=enabled
      buffer = "D" + String(displayenabled) + "#";
      send_str(buffer, interface);
      break;
    case 38:  // :38#   Dxx#      Get Temperature mode 1=Celsius, 0=Fahrenheight
      buffer = "b" + String(myfocuser.temp_mode) + "#";
      send_str(buffer, interface);
      break;
    case 39:  // :39#   Nxxxxxx#  Get the new motor position (target) XXXXXX (not used yet)
      buffer = "N" + String(focuser_target_position) + "#";
      send_str(buffer, interface);
      break;
    case 43:  // :43#            Cx#          Get motorspeed (0-3)
      buffer = "C" + String(myfocuser.motor_speed) + "#";
      send_str(buffer, interface);
      break;
    case 45:  // :45#   Gxxx#     Get tswthreshold - value for which stepper slows down at end of its move
      buffer = "G" + String(MOTORSPEEDCHANGETHRESHOLD) + "#";
      send_str(buffer, interface);
      break;
    case 47:  // 47#    Jxxx#     Get if motorspeedchange enabled/disabled
      buffer = "J0#";
      send_str(buffer, interface);
      break;
    case 49:  // :49#         aXXXXX
      send_str("ab552efd25e454b36b35795029f3a9ba7#", interface);
      break;
    case 62:  // :62#   Lxx#      Get update of position on lcd when moving (00=disable, 01=enable)
      buffer = "L" + String(myfocuser.lcdupdateonmove) + "#";
      send_str(buffer, interface);
      break;
    case 63:  // :63#   Hxx#      Get status of home position switch (0=open, 1=closed)
      buffer = "H0#";
      send_str(buffer, interface);
      break;
    case 66:  // :66#   Kxx#      Get jogging state enabled/disabled
      buffer = "K0#";
      send_str(buffer, interface);
      break;
    case 68:  // :68#   Vxx#      Return jogging direction 0=IN, 1=OUT
      buffer = "V0#";
      send_str(buffer, interface);
      break;
    case 72:  // :72#            3xxx#   Gets motor_delay_after_move
      buffer = "3" + String(myfocuser.motor_delay_after_move) + "#";
      send_str(buffer, interface);
      break;
    case 74:  // get backlash in enabled status
      buffer = "4" + String(myfocuser.motor_backlash_in_enabled) + "#";
      send_str(buffer, interface);
      break;
    case 76:  // get backlash OUT enabled status
      buffer = "5" + String(myfocuser.motor_backlash_out_enabled) + "#";
      send_str(buffer, interface);
      break;
    case 78:  // get number of backlash steps IN
      buffer = "6" + String(myfocuser.motor_backlash_steps_in) + "#";
      send_str(buffer, interface);
      break;
    case 80:  // get number of backlash steps OUT
      buffer = "7" + String(myfocuser.motor_backlash_steps_out) + "#";
      send_str(buffer, interface);
      break;
    case 83:  // get if there is a temperature probe
      buffer = "c" + String(temp_probe1) + "#";
      send_str(buffer, interface);
      break;
    case 87:  // :87#    kx#     Get temp comp direction 1=IN
      buffer = "k" + String(myfocuser.tc_direction) + "#";
      send_str(buffer, interface);
      break;
    case 89:  // get stepper power
      buffer = "91#";
      send_str(buffer, interface);
      break;
    case 5:   // :05xxxxxx# None    Set new target position to xxxxxx (and focuser initiates immediate move to xxxxxx)
    case 28:  // :28#       None    home the motor to position 0
      if (cmd_key_num == 28) {
        focuser_target_position = 0;  // if this is a home then set target to 0
        if (focuser_current_position != 0)
          motor_turning = true;
      } 
      else {
        focuser_new_position = convert_dec_str_to_long(cmd_value_str);  // else set target to a move command
        // Range check
        motor_set_target_position(focuser_new_position);
        if (focuser_target_position != focuser_current_position)
          motor_turning = true;
      }
      break;
    case 7:  // :07xxxxxx# None    set MaxStep
      pos = convert_dec_str_to_long(cmd_value_str);
      if (pos > FOCUSERUPPERLIMIT)  // range check the new value for maxSteps
        pos = FOCUSERUPPERLIMIT;
      if (pos < FOCUSERLOWERLIMIT)  // avoid setting maxSteps too low
        pos = FOCUSERLOWERLIMIT;
      myfocuser.max_step = pos;
      eeprom_set_write_flag();
      break;
    case 12:  // :12xx#      None    set coil pwr 0=release pwr after move, 1=keep power on after move
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      myfocuser.motor_coil_power_enabled = (byte)cmd_value_num & 0x01;
      eeprom_set_write_flag();
      if (myfocuser.motor_coil_power_enabled == 1)
      if (myfocuser.motor_coil_power_enabled == 0)
        motor_disable_output();
      break;
    case 14:  // :14xx#     None    set reverse direction setting 0=normal, 1=reverse
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      myfocuser.motor_reversedirection = (byte)cmd_value_num & 0x01;
      eeprom_set_write_flag();
      break;
    case 15:  // :15XX#   None    Set MotorSpeed, 00 = Slow, 01 = Med, 02 = Fast
      //cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      //myfocuser.motor_speed = focuser_motor_speed = (byte) cmd_value_num & 0x03;
      //motor_set_delay();
      //eeprom_set_write_flag();
      break;
    case 16:  // :16#    None    Display in Celsius (LCD or TFT)
      myfocuser.temp_mode = 1;
      eeprom_set_write_flag();
      break;
    case 17:  // :17#    None    Display in Fahrenheit (LCD or TFT)
      myfocuser.temp_mode = 0;
      eeprom_set_write_flag();
      break;
    case 18:
      // :180#    None    set the return of user specified step_size to be OFF - default
      // :181#    None    set the return of user specified step_size to be ON - reports what user specified as step_size
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      myfocuser.motor_step_size_enabled = (byte)cmd_value_num & 0x01;
      eeprom_set_write_flag();
      break;
    case 19:  // :19xxxx# None    set the step size value - double type, eg 2.1
      {
        double tmp_step_size = (double)cmd_value_str.toFloat();
        if (tmp_step_size <= 0)
          tmp_step_size = DEFAULTSTEPSIZE;  // set default maximum stepsize
        myfocuser.motor_step_size = tmp_step_size;
        eeprom_set_write_flag();
      }
      break;
    case 20:  // :20xx#    None    set the temperature resolution setting for the DS18B20 temperature probe
      // ignore
      break;
    case 22:  // :22xxx#    None    set the temperature compensation value to xxx
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      myfocuser.temp_coefficient = (byte)cmd_value_num & 0xff;
      eeprom_set_write_flag();
      break;
    case 23:  // :23x#   None    Set the temperature compensation ON (1) or OFF (0)
      if (temp_probe1 == 1) {
        cmd_value_num = convert_dec_str_to_int(cmd_value_str);
        myfocuser.tc_enabled = (byte)cmd_value_num & 0x01;
        eeprom_set_write_flag();
      }
      break;
    case 27:  // :27#   None    stop a move - like a Halt
      motor_halt();
      break;
    case 30:  // :30xx#     None    set stepmode (1=Full, 2=Half)
      // full steps only
      // ignore
      break;
    case 31:  // :31xxxxxx# None  set current motor position to xxxxxx (does not move, updates currentpos and targetpos to xxxxxx)
      focuser_new_position = convert_dec_str_to_long(cmd_value_str);
      // rangecheck target
      motor_set_target_position(focuser_new_position);
      motor_turning = false;
      myfocuser.last_position = focuser_current_position = focuser_target_position;
      eeprom_set_write_flag();
      break;
    case 35:  // :35xxxx# None    Set length of time an LCD page is displayed for in milliseconds
      // ignore
      break;
    case 36:
      // :360#    None    Disable Display
      // :361#    None    Enable Display
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      displayenabled = (byte)cmd_value_num & 0x01;
      break;
    case 40:  // :40#      None   Reset Arduino myFocuserPro2 controller
      software_reboot();
      break;
    case 44:  // :44xxx#  None    Set motorspeed threshold when moving - switches to slowspeed when nearing destination
      // ignore
      //      delay(5);
      break;
    case 46:  // :46x#    None    Enable/Disable motorspeed change when moving
      // ignore
      //      delay(5);
      break;
    case 48:  // :48#          None        Save settings to EEPROM
      // copy current settings and write the data to EEPROM
      myfocuser.validdata = VALIDDATAFLAG;
      myfocuser.last_position = focuser_current_position;
      write_EEPROM();
      eeprom_write_flag = 0;
      break;
    case 61:  // :61xx#   None    set update of position on lcd when moving (00=disable, 01=enable)
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      myfocuser.lcdupdateonmove = (byte)cmd_value_num & 0x01;
      eeprom_set_write_flag();
      break;
    case 64:  // :64xxx#  None    Move a specified number of steps
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      focuser_new_position = focuser_current_position + cmd_value_num;
      // rangecheck target
      if (focuser_new_position < 0)
        focuser_new_position = 0;
      if (focuser_new_position > myfocuser.max_step)
        focuser_new_position = myfocuser.max_step;
      focuser_target_position = focuser_new_position;
      motor_turning = true;
      eeprom_set_write_flag();      
      break;
    case 65:  // :65xx#  None    Set jogging state enable/disable
      // ignore
      break;
    case 67:  // :67#     None    Set jogging direction, 0=IN, 1=OUT
      // ignore
      break;
    case 71:  // :71xxx#  None      Set motor_delay_after_move in milliseconds
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      myfocuser.motor_delay_after_move = (byte)(cmd_value_num & 0xff);
      eeprom_set_write_flag();
      break;
    case 73:  // Disable/enable backlash IN (going to lower focuser position)
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      myfocuser.motor_backlash_in_enabled = (byte)cmd_value_num & 0x01;
      eeprom_set_write_flag();
      break;
    case 75:  // Disable/enable backlash OUT (going to higher focuser position)
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      myfocuser.motor_backlash_out_enabled = (byte)cmd_value_num & 0x01;
      eeprom_set_write_flag();
      break;
    case 77:  // set backlash in steps
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      myfocuser.motor_backlash_steps_in = (byte)cmd_value_num & 0xff;
      eeprom_set_write_flag();
      break;
    case 79:  // set backlash OUT steps
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      myfocuser.motor_backlash_steps_out = (byte)cmd_value_num & 0xff;
      eeprom_set_write_flag();
      break;
    case 88:  //:88#          Set temp comp direction 0 =IN, 1: OUT
      cmd_value_num = convert_dec_str_to_int(cmd_value_str);
      myfocuser.tc_direction = (byte)cmd_value_num & 0x01;
      eeprom_set_write_flag();
      break;
  }  // end of switch
}  // end of processcmd()


// send a string to a specific interface
void send_str(String str, uint8_t interface) {
  if (interface == IF_SERIAL) {
    if (Serial) {
      Serial.print(str);
    }
  }
}

// send a new line string to a specific interface
void send_new_line(uint8_t interface) {
  if (interface == IF_SERIAL) {
    if (Serial) {
      Serial.println();
    }
  }
}


// serial_read occurs whenever new data comes in the serial RX.
void serial_event() {
  // : starts the command, # ends the command, do not store these in the command buffer
  // read the command until the terminating # character
  char serial_byte;
  while (Serial.available() && !eoc) {
    serial_byte = Serial.read();
    //Serial.println(serial_byte);
    // If is is the start of a command, clear the command receive buffer
    if (serial_byte == ':') {
      memset(&serial_buffer, 0, MAXCOMMAND);
      idx = 0;
    }
    if (serial_byte == '#') {
      eoc = true;
    }
    else if (serial_byte != '#' && serial_byte != ':') {
      serial_buffer[idx] = serial_byte;
      idx++;
    } 
  }

  if (eoc) {
#ifdef SERIALDEBUG
  serial_send_str(serial_buffer);
#endif
    command_processing(serial_buffer, IF_SERIAL);
    eoc = false;
  }
}


// send a string to the serial port
void serial_send_str(String str) {
  if (Serial) {
    Serial.print(str);
  }
}


// send a new line string to the serial port
void serial_send_new_line() {
  if (Serial) {
    Serial.println();
  }
}


void software_reboot() {
  ESP.restart();
}


void temp_request() {
  sensor1.requestTemperatures();
}


// Read temperature probe
void temp_read() {
  double result = sensor1.getTempCByIndex(0);  // get channel 1 temperature, always in celsius
  // sometimes sensor returns -127, not sure why, so check and use last reading if this is the case
  // range of sensor is -55c to +125c
  // limit to values of -40c to +80c (-40f to 176f)
  int tempval = (int)result;
  if ((tempval < -40) || (tempval > 80)) {
    temp_current_value = temp_tc_last_value;
  }
  else {
    temp_current_value = result;
    temp_tc_last_value = temp_current_value;  // remember last reading
  }
}


// find the address of the DS18B20 sensor probe
void temp_get_address() {
  // look for probes
  // Search the wire for address
  if (sensor1.getAddress(tpAddress, 0)) {
    temp_probe1 = 1;  // there is a probe1
  } else {
    temp_probe1 = 0;
  }
  // device address is now saved in tpAddress
}


// disable the stepper motor outputs - coil power off
void motor_disable_output() {
  digitalWrite(DRV8825ENABLE, HIGH);
  motor_coil_power_state = 0;
  // control via ENABLE pin, but this turns off indexer inside DRV8825
  // which means the stepper motor will loose power and move position to the nearest full step
  // not an issue if using full steps but major issue if using microstepping as will cause change
  // in focus position
  delay(10);
}


// Enable the stepper motor outputs - coil power on
void motor_enable_output() {
  digitalWrite(DRV8825ENABLE, LOW);
  motor_coil_power_state = 1;
  delay(10);  // need to wait 1ms before driver chip is ready for stepping
}


// Set motor step delay
void motor_set_delay() {
  switch (myfocuser.motor_speed) {
    case SLOW:
      motor_step_delay = MOTORSLOWDELAY;
      break;
    case MED:
      motor_step_delay = MOTORMEDIUMDELAY;
      break;
    case FAST:
      motor_step_delay = MOTORFASTDELAY;
      break;
    default:
      motor_step_delay = MOTORFASTDELAY;
      break;
  }
}


// Set motor step mode
void motor_set_step_mode() {
  switch (USERSTEPMODE) {
    case STEP1:
      digitalWrite(DRV8825M0, LOW);
      digitalWrite(DRV8825M1, LOW);
      digitalWrite(DRV8825M2, LOW);
      break;
    case STEP2:
      digitalWrite(DRV8825M0, HIGH);
      digitalWrite(DRV8825M1, LOW);
      digitalWrite(DRV8825M2, LOW);
      break;
    case STEP4:
      digitalWrite(DRV8825M0, LOW);
      digitalWrite(DRV8825M1, HIGH);
      digitalWrite(DRV8825M2, LOW);
      break;
    case STEP8:
      digitalWrite(DRV8825M0, HIGH);
      digitalWrite(DRV8825M1, HIGH);
      digitalWrite(DRV8825M2, LOW);
      break;
    case STEP16:
      digitalWrite(DRV8825M0, LOW);
      digitalWrite(DRV8825M1, LOW);
      digitalWrite(DRV8825M2, HIGH);
      break;
    case STEP32:
      digitalWrite(DRV8825M0, HIGH);
      digitalWrite(DRV8825M1, LOW);
      digitalWrite(DRV8825M2, HIGH);
      break;
    default:  // STEP32
      digitalWrite(DRV8825M0, HIGH);
      digitalWrite(DRV8825M1, LOW);
      digitalWrite(DRV8825M2, HIGH);
      break;
  }
}


// Turn stepper motor_turn_anti_clockwise
void motor_turn_anti_clockwise() {
  // Move MICROSTEPSPERMOVE micro steps for each step requested for smoother stepping
  for (uint8_t steps = 0; steps < MICROSTEPSPERMOVE; steps++) {
    digitalWrite(LEDPIN, LOW);
    digitalWrite(DRV8825DIR, LOW);
    digitalWrite(DRV8825STEP, 1);
    delayMicroseconds(MOTORPULSETIME);
    digitalWrite(DRV8825STEP, 0);
    delayMicroseconds(MOTORPULSETIME);
    digitalWrite(LEDPIN, HIGH);
    delay(motor_step_delay);
  }
}


// Turn stepper motor_turn_clockwise
void motor_turn_clockwise() {
  // Move MICROSTEPSPERMOVE micro steps for each step requested for smoother stepping
  for (uint8_t steps = 0; steps < MICROSTEPSPERMOVE; steps++) {
    digitalWrite(LEDPIN, LOW);
    digitalWrite(DRV8825DIR, HIGH);
    digitalWrite(DRV8825STEP, 1);
    delayMicroseconds(MOTORPULSETIME);
    digitalWrite(DRV8825STEP, 0);
    delayMicroseconds(MOTORPULSETIME);
    digitalWrite(LEDPIN, HIGH);
    delay(motor_step_delay);
  }
}


// Turn the motor
void motor_turn() {
  if (motor_coil_power_state == false) { // if board is not enabled, we need to enable it else it will not step
    motor_enable_output();  // have to enable driver board
  }

  long move_steps = abs(focuser_current_position - focuser_target_position);
  //----------------------------------moving in---------------------------------------------------------
  if (focuser_target_position < focuser_current_position)   // moving in
  {
    movestarted = 1;
    for (long steps = 0; steps < move_steps; steps++) {
      if (motor_turning == true) {
        if (myfocuser.motor_reversedirection == 1)          // revert direction
          motor_turn_clockwise();
        else
          motor_turn_anti_clockwise();

        focuser_current_position--;
        scheduled_tasks();
      }
      else
        break;
    }
  }

  //----------------------------------moving out--------------------------------------------------------
  else if (focuser_target_position > focuser_current_position)  // moving out
  {
    movestarted = 1;
    // move the actual distance
    for (long steps = 0; steps < move_steps; steps++) {
      if (motor_turning == true) {        
        if (myfocuser.motor_reversedirection == 1)            // revert direction
          motor_turn_anti_clockwise();
        else
          motor_turn_clockwise();

        focuser_current_position++;
        scheduled_tasks();
      }
      else
        break;
    }

#ifdef BACKLASH  
    motor_backlash_compensation();
#endif
  }
  motor_turning = false;
  movestarted = 0;
  myfocuser.last_position = focuser_current_position;
  write_EEPROM();                               // update values in EEPROM

  if (myfocuser.motor_coil_power_enabled == false){
    motor_disable_output();
  }
  scheduled_tasks(); 
  // Delay to allow the focuser to settle after the move    
  delay (motor_settle_time);

}


// Applying backlash compensation algorithm
void motor_backlash_compensation() {
  // apply backlash out
  for (int steps = 0; steps < myfocuser.motor_backlash_steps_out * USERSTEPMODE; steps++) {
    motor_turn_clockwise();
  }   
  delay (motor_settle_time);
  // apply backlash in to avoid focuser slip
  for (int steps = 0; steps < myfocuser.motor_backlash_steps_in * USERSTEPMODE; steps++) {
    motor_turn_anti_clockwise();
  }
}


// Validate and set target position
void motor_set_target_position(long new_pos) {
  if (new_pos > myfocuser.max_step)
    focuser_target_position = myfocuser.max_step;
  else if (new_pos < 0)
    focuser_target_position = 0;
  else
    focuser_target_position = new_pos;
}


void motor_halt(){
  motor_turning = false;
  focuser_target_position = focuser_current_position;
  myfocuser.last_position = focuser_current_position;
  eeprom_set_write_flag();
}

// set the eeprom write flag
void eeprom_set_write_flag() {
  //  delay(5);
  eeprom_write_flag = 1;                       // updating of EEPROM ON
  eeprom_last_timestamp = millis();
}


void write_EEPROM() {
  EEPROM.put(eeprom_current_addr, myfocuser);  // update values in EEPROM
  EEPROM.commit();
  delay(10);
}


// convert string to long
long convert_dec_str_to_long(String str) {
  char ch_arr[12];
  str.toCharArray(ch_arr, sizeof(ch_arr));
  return atol(ch_arr);
}


// convert string to int
uint8_t convert_dec_str_to_int(String str) {
  return (uint8_t)(str.toInt());
}


void temp_compensation() {
  if (temp_tc_started == 0) {
    temp_tc_started = 1;
    temp_tc_start_value = temp_current_value;
  }
 
  double tempchange =  temp_current_value - temp_tc_start_value;
  // temperature has dropped by 1 degree
  if ((tempchange < 0) && (abs(tempchange) >= 1))
  {
    // move the focuser by the required amount, this should move focuser inwards
    if (myfocuser.tc_direction == 0)
      focuser_new_position = focuser_current_position - myfocuser.temp_coefficient;
    else
      focuser_new_position = focuser_current_position + myfocuser.temp_coefficient;
  }

  // temperature has increased by 1 degree
  else if (tempchange >= 1) {
    // move the focuser by the required amount, this should move focuser inwards
    if (myfocuser.tc_direction == 0)
      focuser_new_position = focuser_current_position + myfocuser.temp_coefficient;
    else
      focuser_new_position = focuser_current_position - myfocuser.temp_coefficient;
  }

  // range check the new position
  if (focuser_new_position < 0)
    focuser_new_position = 0;
  if (focuser_new_position > myfocuser.max_step)
    focuser_new_position = myfocuser.max_step;
  
  focuser_target_position = focuser_new_position;
  motor_turning = true;
  motor_turn();
  temp_tc_started = 0;  // indicate that temp compensation was done

}

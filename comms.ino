//--------------------------------------------------------
// Evil Minion 5 Axis robot firmware
// dan@marginallyclever.com 
// 2015 September 3
// see http://evilminion.info/ for more information.
//--------------------------------------------------------
#include "pid.h"
#include "motor.h"
#include "sensor.h"
#include "flash.h"



char buffer[MAX_BUF+1];  // Serial buffer
int sofar;               // Serial buffer progress
static long last_cmd_time;    // prevent timeouts

uint32_t line_number;

char absolute_mode = DEFAULT_IS_ABSOLUTE_ON;





void comms_setup() {
  Serial.begin(BAUD_RATE);
  line_number=0;
}


/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parse_number(char code,float val) {
  char *ptr=buffer;  // start at the beginning of buffer
  while(ptr && *ptr && ptr<buffer+sofar) {  // walk to the end
    if(*ptr==code) {  // if you find code on your walk,
      return atof(ptr+1);  // convert the digits that follow into a float and return it
    }
    ptr=strchr(ptr,' ')+1;  // take a step from here to the letter after the next space
  }
  return val;  // end reached, nothing found, return default val.
}


/**
 * Look for character /code/ in the buffer.
 * @return /true/ if the code is found.
 * @input code the character to look for.
 **/
float has_code(char code) {
  char *ptr=buffer;  // start at the beginning of buffer
  while(ptr && *ptr && ptr<buffer+sofar) {  // walk to the end
    if(*ptr==code) {  // if you find code on your walk,
      return true;
    }
    ptr=strchr(ptr,' ')+1;  // take a step from here to the letter after the next space
  }
  return false;
}


/**
 * Print a helpful message to serial.  The first line must never be changed to play nice with the JAVA software.
 */
void help() {
  Serial.print(F("\n\nHELLO WORLD! I AM MINION #"));
  Serial.println(robot_uid,DEC);
  Serial.println(F("== http://www.marginallyclever.com/ =="));
  Serial.println(F("I understand the following commands:"));
  Serial.println(F("Nx *y (this is line number x, with checksum y)\n"\
"UID * (write robot UID * to EEPROM)\n"\
"M17 (disable motors)\n"\
"M18 (enable motors)\n"\
"M100 (help)\n"\
"M110 N* (set line number to *)\n"\
"M114 (where)\n"\
"G90 (absolute mode)\n"\
"G91 (relative mode)\n"\
"G92 [Aa] (teleport)\n"\
"R0 [Aa] [Bb] [Cc] [Dd] [Ed] (move motors so that sensors read [abcde])\n"\
"R1 (continuous angle reporting)\n"\
"R3 (toggle compliant mode)\n"\
"R5 P* (adjust compliance limit to *)\n"\
"R10 S* (adjust servo angle to *)\n"\
"R60 [Aa] [Bb] [Cc] [Dd] [Ed] (set sensors to new value [abcde])\n"\
"R61 (display sensor adjustment amounts)\n"\
"R70 (write sensor adjustments to EEPROM)\n"));
}


void where() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    Serial.print(motor_letters[i]);
    Serial.print(sensors_filtered[i]);
    Serial.print(' ');
  }
  
  Serial.print('S');
  Serial.print(servo_angle);
  
  Serial.print('\n');
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">\n"));  // signal ready to receive input
  last_cmd_time = millis();
}


/**
 * Prevent illegal moves that would obviously damage the machine
 * @input index the joint index
 * @input angle the desired angle
 * @return the corrected angle, clamped within an acceptable range.
 */
float software_angle_limits(int index,float angle) {
  switch(index) {
    case 0:  break;  // A has NO LIMIT.
    case 1: // B
      if( angle > ANGLE_B_MAX ) angle = ANGLE_B_MAX;
      if( angle < ANGLE_B_MIN ) angle = ANGLE_B_MIN;
      break;
    case 2: // C
      if( angle > ANGLE_C_MAX ) angle = ANGLE_C_MAX;
      if( angle < ANGLE_C_MIN ) angle = ANGLE_C_MIN;
      break;
    case 3: // D
      if( angle > ANGLE_D_MAX ) angle = ANGLE_D_MAX;
      if( angle < ANGLE_D_MIN ) angle = ANGLE_D_MIN;
      break;
    case 4: // E
      if( angle > ANGLE_E_MAX ) angle = ANGLE_E_MAX;
      if( angle < ANGLE_E_MIN ) angle = ANGLE_E_MIN;
      break;
  }
  
  return angle;
}


void process_move_motors() {
  int i;
  float v;
  
  for(i=0;i<NUM_AXIES;++i) {
    if(!has_code(motor_letters[i])) continue;
    
    if( absolute_mode==0 ) {
      v = parse_number(motor_letters[i], 0 );
      v += destination[i];
    } else {
      v = parse_number(motor_letters[i], destination[i] );
    }
    destination[i] = software_angle_limits(i,v);
    
//    Serial.print(F("Moving "));
//    Serial.print(absolute_mode==0?F("REL "):F("ABS "));
//    Serial.print(motor_letters[i]);
//    Serial.print(F(" to "));
//    Serial.print(v);
//    Serial.print(F(" ~ "));
//    Serial.println(destination[i]);
    
    PID_init(pid[i]);
    move_active[i]=1;
  }
  
}


void process_servo_command() {
  if(!has_code('S')) return;
  
  int v = parse_number('S',servo_angle);
  servo_move(v);
}


void process_sensors_adjust() {
  int i;
  
  for(i=0;i<NUM_AXIES;++i) {
    if(!has_code(motor_letters[i])) continue;
    
    // get the new angle
    float newAngle = parse_number(motor_letters[i], 0 );
    // get the original sensor angle (without adjustment)
    float original_angle = sensors_filtered[i] - sensors_adjust[i];
    // find the difference
    float diff = newAngle - original_angle;
//    Serial.print("i=");  Serial.println(i);
//    Serial.print("motor_letters[i]=");  Serial.println(motor_letters[i]);
//    Serial.print("sensors_adjust[i]=");  Serial.println(sensors_adjust[i]);
//    Serial.print("sensor_angle(i)=");  Serial.println(sensor_angle(i));
//    Serial.print("newAngle=");  Serial.println(newAngle);
//    Serial.print("diff=");  Serial.println(diff);
    // save it
    sensors_adjust[i] = diff;
  }
}


void print_sensors_adjust() {
  int i;
  
  for(i=0;i<NUM_AXIES;++i) {
    Serial.print(motor_letters[i]);
    Serial.print(sensors_adjust[i]);
    Serial.print(' ');
  }
  Serial.println();
}


void parse_teleport() {
  sensors_filtered[0] = parse_number('A',0);
}

/**
 * process commands in the serial receive buffer
 */
void process_command() {
  // blank lines
  if(buffer[0]==';') return;
  
  long cmd;
  
  // is there a line number?
  cmd=parse_number('N',-1);
  if(cmd!=-1 && buffer[0]=='N') {  // line number must appear first on the line
    if( cmd != line_number ) {
      // wrong line number error
      Serial.print(F("BADLINENUM "));
      Serial.println(line_number);
      return;
    }
  
    // is there a checksum?
    if(strchr(buffer,'*')!=0) {
      // yes.  is it valid?
      char checksum=0;
      int c=0;
      while(buffer[c]!='*' && c<MAX_BUF) checksum ^= buffer[c++];
      c++; // skip *
      int against = strtod(buffer+c,NULL);
      if( checksum != against ) {
        Serial.print(F("BADCHECKSUM "));
        Serial.println(line_number);
        return;
      } 
    } else {
      Serial.print(F("NOCHECKSUM "));
      Serial.println(line_number);
      return;
    }
    
    line_number++;
  }

  if(!strncmp(buffer,"UID",3) && robot_uid==0) {
    robot_uid=atoi(strchr(buffer,' ')+1);
    saveUID();
  }

  
  cmd=parse_number('M',-1);
  switch(cmd) {
  //case 6:  tool_change(parse_number('T',current_tool));  break;
  case 17:  motor_disable();  break;
  case 18:  motor_enable();  break;
  case 100:  help();  break;
  //case 101:  processconfig();  break;
  case 110:  line_number = parse_number('N',line_number);  break;
  case 114:  where();  break;
  }


  cmd=parse_number('G',-1);
  switch(cmd) {
/*
  case 0:
  case 1: {  // line
      Vector3 offset=get_end_plus_offset();
      acceleration = min(max(parse_number('A',acceleration),1),2000);
      line_safe( parse_number('X',(absolute_mode?offset.x:0)*10)*0.1 + (absolute_mode?0:offset.x),
                 parse_number('Y',(absolute_mode?offset.y:0)*10)*0.1 + (absolute_mode?0:offset.y),
                 parse_number('Z',(absolute_mode?offset.z:0)) + (absolute_mode?0:offset.z),
                 parse_number('F',feed_rate) );
      break;
    }
  case 2:
  case 3: {  // arc
      Vector3 offset=get_end_plus_offset();
      acceleration = min(max(parse_number('A',acceleration),1),2000);
      setFeedRate(parse_number('F',feed_rate));
      arc(parse_number('I',(absolute_mode?offset.x:0)*10)*0.1 + (absolute_mode?0:offset.x),
          parse_number('J',(absolute_mode?offset.y:0)*10)*0.1 + (absolute_mode?0:offset.y),
          parse_number('X',(absolute_mode?offset.x:0)*10)*0.1 + (absolute_mode?0:offset.x),
          parse_number('Y',(absolute_mode?offset.y:0)*10)*0.1 + (absolute_mode?0:offset.y),
          parse_number('Z',(absolute_mode?offset.z:0)) + (absolute_mode?0:offset.z),
          (cmd==2) ? -1 : 1,
          parse_number('F',feed_rate) );
      break;
    }
  case 4:  {  // dwell
      wait_for_empty_segment_buffer();
      pause(parse_number('S',0) + parse_number('P',0)*1000.0f);
      break;
    }
  case 28:  FindHome();  break;
  case 54:
  case 55:
  case 56:
  case 57:
  case 58:
  case 59: {  // 54-59 tool offsets
    int tool_id=cmd-54;
    set_tool_offset(tool_id,parse_number('X',tool_offset[tool_id].x),
                            parse_number('Y',tool_offset[tool_id].y),
                            parse_number('Z',tool_offset[tool_id].z));
    break;
    }*/
  case 90:  absolute_mode=1;  break;  // absolute mode
  case 91:  absolute_mode=0;  break;  // relative mode
  case 92:  parse_teleport();  break;
//  case 4:  SD_StartPrintingFile(strchr(buffer,' ')+1);  break;  // read file
  }


  cmd=parse_number('R',-1);
  switch(cmd) {
  case 0:  process_move_motors();  break;
  case 1:  continuous_reporting=1-continuous_reporting;  break;
  case 3:  
    compliant_mode=1-compliant_mode;
    Serial.print(F("Compliance "));
    Serial.println(compliant_mode==1?F("ON"):F("OFF"));
    break;
  case 5:  compliance_limit = parse_number('P',compliance_limit);  break;
  case 10: process_servo_command();  break;
  case 60: process_sensors_adjust();  break;
  case 61: print_sensors_adjust();  break;
  case 70: saveAdjustments();  break;
  }
}


// listen for serial commands
// See: http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-serial-monitor/
void tick_comms() {
  while(Serial.available() > 0) {
    char c = Serial.read();
    if(c=='\r') continue;
    if(sofar<MAX_BUF) buffer[sofar++]=c;
    if(c=='\n' || c=='\r') {
      buffer[sofar]=0;
      
      // echo confirmation
//      Serial.println(buffer);
   
      // do something with the command
      process_command();
      ready();
    }
  }
}



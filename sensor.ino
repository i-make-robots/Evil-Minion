#include "sensor.h"

float sensors_adjust[NUM_AXIES];
float sensors_raw[NUM_AXIES];
float sensors_expected[NUM_AXIES];


//
void setup_sensors() {
}


// from http://www.madscientisthut.com/forum_php/viewtopic.php?f=11&t=7
uint32_t sensor_update(int csel,int sdout) {
  uint32_t data = 0, inputStream;
  int x;
  
  // Sensor sends data when CLK goes high.
  // To choose a board, set the CSEL pin high, then tick the clock.
  digitalWrite(csel, HIGH);
  digitalWrite(PIN_SENSOR_CLK, HIGH);
  // We won't need CSEL again until the next sample, so set it low
  digitalWrite(csel, LOW);
  // Set the clock low.  On the next high sensor will start to deliver data.
  digitalWrite(PIN_SENSOR_CLK, LOW);

  for (x=0; x < SENSOR_TOTAL_BITS; x++) {
    digitalWrite(PIN_SENSOR_CLK, HIGH);
    // one bit of data is now waiting on sensor pin
    inputStream = digitalRead(sdout);
    data = ((data << 1) + inputStream); // left-shift summing variable, add pin value
    digitalWrite(PIN_SENSOR_CLK, LOW);
  }
  return data;
}


/**
 * @input data the raw sensor reading
 * @return the angle in degrees
 */
float sensor_angle(uint32_t data) {
  uint32_t angle = data >> SENSOR_STATUS_BITS; // shift 18-digit angle right 6 digits to form 12-digit value
  angle &= SENSOR_ANGLE_MASK >> SENSOR_STATUS_BITS;  // mask the 18 bits that form the angle
  return (angle * SENSOR_ANGLE_PER_BIT);
}


/**
 * @input data the raw sensor reading
 * @return the angle in degrees
 */
int sensor_error(uint32_t data,int beVerbose) {
#ifdef SENSOR_PARITY_TEST_ON
  // Parity test
  char parity = data & SENSOR_PARITY_MASK;
  int parity_test=0;
  int x;
  uint32_t v = data >> SENSOR_PARITY_BITS;
  for (x=0; x < (SENSOR_TOTAL_BITS-SENSOR_PARITY_BITS); ++x) {
    parity_test += (v & 0x1);
    v >>= 1;
  }
  if( (parity_test & 0x1) != parity ) {
    if(beVerbose) {
      Serial.print("Parity error ");
      Serial.print(parity_test&1,DEC);
      Serial.print('\t');
      Serial.print(parity,DEC);
      Serial.print('\t');
    }
    return 1;
  }
#endif
  
  // status tests
  int statusBits = data & SENSOR_ERROR_MASK;
  char DECn = statusBits & 2; // goes high if magnet moved away from IC
  char INCn = statusBits & 4; // goes high if magnet moved towards IC
  char LIN = statusBits & 8; // goes high for linearity alarm
  char COF = statusBits & 16; // goes high for cordic overflow: data invalid
  char OCF = 32 - (statusBits & 32); // this is 1 when the chip startup is finished.
  
  if (DECn && INCn) {
    if(beVerbose) Serial.println("magnet moved out of range");
    return (4 | 2);
  } else if (DECn) { 
    if(beVerbose) Serial.println("magnet moved away from chip");
    return 4;
  } else if (INCn) {
    if(beVerbose) Serial.println("magnet moved towards chip");
    return 2;
  }
  
  if (LIN) { 
    if(beVerbose) Serial.println("linearity alarm: magnet misaligned? Data questionable.");
    return 8;
  }
  if (COF) {
    if(beVerbose) Serial.println("cordic overflow: magnet misaligned? Data invalid.");
    return 16;
  }
  if(OCF) {
    if(beVerbose) Serial.println("Sensor not ready.");
    return 32;
  }

  return 0;
}



void test_one_sensor(int sensor_number,int csel_pin,int sdout_pin) {
  uint32_t d = sensor_update(csel_pin,sdout_pin);
//  Serial.print(d,BIN);
//  Serial.print('\t');
  
  int err = sensor_error(d,0);
  if(err != 0) {
    //Serial.print("ERR");
    //Serial.print(err,HEX);
  } else {
    float angle = sensor_angle(d) + sensors_adjust[sensor_number];
    if(angle<0) angle+=360;
    else if(angle>=360) angle-=360;
    sensors_raw[sensor_number] = angle;
//    Serial.print(sensors_raw[sensor_number]);
  }
}


void tick_sensors() {
  test_one_sensor(0,PIN_SENSOR_A_CSEL,PIN_SENSOR_SDOUT_A);
  test_one_sensor(1,PIN_SENSOR_B_CSEL,PIN_SENSOR_SDOUT_B);
  test_one_sensor(2,PIN_SENSOR_C_CSEL,PIN_SENSOR_SDOUT_C);
  test_one_sensor(3,PIN_SENSOR_D_CSEL,PIN_SENSOR_SDOUT_D);
  test_one_sensor(4,PIN_SENSOR_E_CSEL,PIN_SENSOR_SDOUT_E);
}



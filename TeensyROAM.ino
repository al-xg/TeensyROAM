
unsigned long report_time;
unsigned long work_time;
unsigned long RC_time;
unsigned long status_time;
unsigned long tmp_time;
unsigned long LastActivePPM;
unsigned long tmpPPMtime;
int flashRate;

//PID loops
#include "PID_ROAM.h"
//#include <PID_v1.h>

const double PIDSampleTime = 67; //Interval in ms
const double safe_distance = 45; //value in cm
const double OutMax = 550; //value in ms
const double maxRange = 770; //this is the range used by the program for initialisation and when reading the sonar fails
const double P = 15;
const double I = 15;
const double D = 0;
double Setpoint_right, right_sonar = maxRange, Output_right = 0;
double Setpoint_front, front_sonar = maxRange, Output_front = 0;
double Setpoint_left, left_sonar = maxRange, Output_left = 0;
double Setpoint_rear, rear_sonar = maxRange, Output_rear = 0;

//Specify the links and initial tuning parameters
PID PID_right(&right_sonar, &Output_right, &Setpoint_right, P, I, D, DIRECT);
PID PID_front(&front_sonar, &Output_front, &Setpoint_front, P, I, D, DIRECT);
PID PID_left(&left_sonar, &Output_left, &Setpoint_left, P, I, D, DIRECT);
PID PID_rear(&rear_sonar, &Output_rear, &Setpoint_rear, P, I, D, DIRECT);


//I2C sonars
#include <i2c_t3.h>
//The Arduino Wire library uses the 7-bit version of the address, so the code example uses 0x70 instead of the 8‑bit 0xE0
#define frontI2C byte(0x81) //A
#define rearI2C byte(0x82)  //B
#define leftI2C byte(0x70)  //C
#define rightI2C byte(0x71) //D

#define RangeCommand byte(0x51)//The Sensor ranging command has a value of 0x51


bool front_fn = 0;
bool rear_fn = 0;
bool left_fn = 0;
bool right_fn = 0;
size_t front_sz = 0;
size_t rear_sz = 0;
size_t left_sz = 0;
size_t right_sz = 0;

//Active sonars: 0= not in use, 1=in use
bool leftSonarActive = 1;
bool rightSonarActive = 1;
bool frontSonarActive = 0;
bool rearSonarActive = 0;

//Commands the I2C sensor to take a range reading
void takeRangeReading(byte Address) {
  Wire.beginTransmission(Address);             //Start addressing
  Wire.write(RangeCommand);  //send range command
  Wire.sendTransmission(I2C_STOP);  //Stop and do something else now
  Wire.finish();
}

//Returns the last range that the sensor determined in its last ranging cycle in centimeters. Returns 0 if there is no communication.
short readRange(byte Address) {
  size_t sz = Wire.requestFrom(Address, byte(2), I2C_STOP);

  if (Wire.available() >= 2) {                            //Sensor responded with the two bytes
    byte HighByte = Wire.read();                          //Read the high byte back
    byte LowByte = Wire.read();                           //Read the low byte back
    short range = (short)(LowByte + (HighByte << 8));
    return range;
  }
  else {
    return (short)(maxRange + 50);                        //Else nothing was received, return max range so that no reading= no obstacle rather than no reading= collision  and then + 50 for debuging
  }
}

void ReadI2CSonars() {
  
  
  
  front_fn = 0;
  rear_fn = 0;
  left_fn = 0;
  right_fn = 0;

  front_sz = 0;
  rear_sz = 0;
  left_sz = 0;
  right_sz = 0;

  front_sonar=maxRange;
  rear_sonar=maxRange;
  left_sonar=maxRange;
  right_sonar=maxRange;

  int timeout = millis() + 1;
  while ((front_fn != 1 & rear_fn != 1 & left_fn != 1 & right_fn != 1) & (timeout > millis())) {
    if ((front_sz == 2)) {
      if (front_fn == 0) {
        front_sonar = readRange(frontI2C);
        front_fn = 1;
      }
    }
    else {
      front_sz = Wire.requestFrom(frontI2C, byte(2)); //I2C_STOP,1000);
    }


    if ((rear_sz == 2)) {
      if (rear_fn == 0) {
        rear_sonar = readRange(rearI2C);
        rear_fn = 1;
      }
    }
    else {
      rear_sz = Wire.requestFrom(rearI2C, byte(2));
    }


    if ((left_sz == 2)) {
      if (left_fn == 0) {
        left_sonar = readRange(leftI2C);
        left_fn = 1;
      }
    }
    else {
      left_sz = Wire.requestFrom(leftI2C, byte(2));
    }
    if ((right_sz == 2)) {
      if (right_fn == 0) {
        right_sonar = readRange(rightI2C);
        right_fn = 1;
      }
    }
    else {
      right_sz = Wire.requestFrom(rightI2C, byte(2));
    }
  }
  if (front_fn == 0) {
    front_sonar = readRange(frontI2C);
  }
  if (rear_fn == 0) {
    rear_sonar = readRange(rearI2C);
  }
  if (left_fn == 0) {
    left_sonar = readRange(leftI2C);
  }
  if (right_fn == 0) {
    right_sonar = readRange(rightI2C);
  }
}


//PPM Read/Write
#include "PulsePosition.h"

PulsePositionOutput PPMout;
PulsePositionInput PPMin;
bool PPMactive = 0;

//RC Inputs/outputs
double channel[20];
double pitch_in; //channel 4
double roll_in;//channel 2
double throttle_in; //channel 3
double yaw_in; //channel 1
double mode_switch; //channel 5
double aux1; //channel 6
double aux2; //channel 7
double gear; //channel 8
int compd_pitch, compd_roll, compd_throttle;

int ConstrainPWM(int PWM_out, int MinPW, int MaxPW) {
  if (PWM_out < MinPW) PWM_out = MinPW;
  if (PWM_out > MaxPW) PWM_out = MaxPW;
  return PWM_out;
}

void readPPM() {
  int i, numCh;
  numCh = PPMin.available();
  if (numCh > 0) {
    PPMactive = 1;
    LastActivePPM = millis();
    for (i = 1; i <= 8; i++) {
      channel[i] = PPMin.read(i);
    }
  }
  else {
    //check how long it has been since last PPM signal
    tmpPPMtime = millis();
    if (tmpPPMtime > LastActivePPM + 108) { //DR4-II frame is 27ms
      PPMactive = 0; 
    }
  }
}

void writePPM() {
  PPMout.write(3, compd_throttle);
  PPMout.write(2, compd_pitch);
  PPMout.write(1, compd_roll);
  PPMout.write(4, yaw_in);
  PPMout.write(5, mode_switch);
  PPMout.write(6, aux1);
  PPMout.write(7, aux2);
  PPMout.write(8, gear);
}

void PPMdefaults() {
  pitch_in = 1500; //channel 4
  roll_in = 1500; //channel 2
  throttle_in = 990; //channel 3
  yaw_in = 1500; //channel 1
  mode_switch = 990; //channel 5
  aux1 = 990; //channel 6
  aux2 = 990; //channel 7
  gear = 990; //channel 8
}

//Spektrum Serial support
#include "SatelliteReceiver.h"
SatelliteReceiver Rx;

void readSpektrum() {
  Rx.getFrame();
  roll_in = Rx.getAile();
  pitch_in = Rx.getElev();
  throttle_in = Rx.getThro();
  yaw_in = Rx.getRudd();
  mode_switch = Rx.getFlap();
  aux1 = Rx.getGear();
  aux2 = Rx.getAux2();
}


void setup() {
  Serial.begin(115200);    //USB reporting
 // Serial1.begin(115200);   //Spektrum serial
 // Serial3.begin(115200);   //Telemetry

  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  flashRate=500;

  PPMout.begin(9);
  PulsePositionInput PPMout(RISING);
  PPMin.begin(10);
  PulsePositionInput PPMin(RISING);
  PPMdefaults(); // set default values for PPM channels until the Rx is turned on and correct read

  Wire.begin(I2C_MASTER, 0, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100); //Initiate Wire library for I2C communications with I2CXL‑MaxSonar‑EZ

  //Initiate sonar ranging ready for 1st loop
  ReadI2CSonars();
  takeRangeReading(frontI2C);
  takeRangeReading(rearI2C);
  takeRangeReading(leftI2C);
  takeRangeReading(rightI2C);

  //Initialise PID loops
  Setpoint_right = Setpoint_front = Setpoint_left = Setpoint_rear = safe_distance;

  PID_right.SetMode(AUTOMATIC);
  PID_right.SetSampleTime(PIDSampleTime);
  PID_right.SetOutputLimits(0, OutMax);

  PID_front.SetMode(AUTOMATIC);
  PID_front.SetSampleTime(PIDSampleTime);
  PID_front.SetOutputLimits(0, OutMax);

  PID_left.SetMode(AUTOMATIC);
  PID_left.SetSampleTime(PIDSampleTime);
  PID_left.SetOutputLimits(0, OutMax);

  PID_rear.SetMode(AUTOMATIC);
  PID_rear.SetSampleTime(PIDSampleTime);
  PID_rear.SetOutputLimits(0, OutMax);

  //pinMode(piezzo,OUTPUT); //Buzzer for PID output, find unused pin

  //Intialise loop timers
  report_time	= millis();
  work_time	= millis();
  RC_time       = millis();
  status_time   = millis();
}

void buzzer() {
  //buzzer
  /*   if (aux2>1600){
   if(right_sonar<(Setpoint_right+20)) analogWrite(piezzo,Output_right);
   else analogWrite(piezzo,0);
   }
   else analogWrite(piezzo,0);*/
}


  

void report() {
  report_time = millis();

  //PID varaibles
  MegunoFormat("Output_left",Output_left,"PID");
  MegunoFormat("Output_right",Output_right,"PID");
  //MegunoFormat("Output_front",Output_front,"PID");
  //MegunoFormat("Output_rear",Output_rear,"PID");
  MegunoFormat("safe_distance",safe_distance,"PID");
  MegunoFormat("left_sonar",left_sonar,"PID");
  MegunoFormat("right_sonar",right_sonar,"PID");
  //MegunoFormat("front_sonar",front_sonar,"PID");
  //MegunoFormat("rear_sonar",rear_sonar,"PID");
  MegunoFormat("roll_in",roll_in,"PID");
  MegunoFormat("compd_roll",compd_roll,"PID");
  MegunoFormat("pitch_in",pitch_in,"PID");
  MegunoFormat("compd_pitch",compd_pitch,"PID");
  
  //RC input varaibles
  MegunoFormat("roll_in",roll_in,"RC");
  MegunoFormat("pitch_in",pitch_in,"RC");
  MegunoFormat("yaw_in",yaw_in,"RC");
  MegunoFormat("throttle_in",throttle_in,"RC");
  MegunoFormat("mode_switch",mode_switch,"RC");
  MegunoFormat("aux1",aux1,"RC");
}


void RC() {
  RC_time = millis();
  //Refresh RC inputs
  //readSpektrum();
  readPPM();
  if (PPMactive == 0) {
    PPMdefaults();
  } else {
    pitch_in = channel[2];
    roll_in = channel[1];
    throttle_in = channel[3];
    yaw_in = channel[4];
    mode_switch = channel[5];
    aux1 = channel[6];
  }

  //Do we want obstacle avoidance on?
  if (aux1 > 1400) {
    flashRate=125;
    compd_roll = (roll_in - int(Output_right) + int(Output_left));
    compd_pitch = (pitch_in - int(Output_rear) + int(Output_front));
    compd_roll = ConstrainPWM(compd_roll, 990, 2015);
    compd_pitch = ConstrainPWM(compd_pitch, 990, 2015);
    compd_throttle = throttle_in; //throttle passthrough
  }
  else {
    flashRate=500;
    compd_roll = roll_in;       //passthrough
    compd_pitch = pitch_in;     //passthrough
    compd_throttle = throttle_in; //passthrough
  }

  writePPM(); //Send values to Flight Controller
}

void workloop() {
  work_time = millis();

  //Refresh sensor readings
   ReadI2CSonars();

  //Run PID loops for each sensor and start next ranging cycle on the I2C sonars
  if (frontSonarActive == 1) {
    PID_front.Compute();
    takeRangeReading(rearI2C);
  }
  if (rearSonarActive == 1) {
    PID_rear.Compute();
    takeRangeReading(frontI2C);
  }
  if (leftSonarActive == 1) {
    PID_left.Compute();
    takeRangeReading(leftI2C);
  }
  if (rightSonarActive == 1) {
    PID_right.Compute();
    takeRangeReading(rightI2C);
  }
}

void loop() {
  tmp_time = millis();

  if (tmp_time  > report_time + 10) {
    report();
  }
  if (tmp_time  > work_time + PIDSampleTime) {
    workloop();
  }
  if (tmp_time  > RC_time + 10) {
    RC();
  }
  if (tmp_time  > status_time + flashRate) {
    status_time = millis();  
    digitalWrite(13, !digitalRead(13));
  }
}
























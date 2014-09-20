unsigned long report_time;
unsigned long work_time;
unsigned long tmp_time;

#include "PID_ROAM.h"

//Define Variables we'll be connecting to
double Setpoint_right, right_sonar=100, Output_right=0;
double Setpoint_front, front_sonar=100, Output_front=0;
double Setpoint_left, left_sonar=100, Output_left=0;
double Setpoint_rear, rear_sonar=100, Output_rear=0;
const double PIDSampleTime=100; //interval in ms
const double safe_distance=30; //value in cm
const double OutMax=600; //value in ms
const double P=7;
const double I=3;
const double D=0;

//Specify the links and initial tuning parameters
PID PID_right(&right_sonar, &Output_right, &Setpoint_right,P,I,D, DIRECT);
PID PID_front(&front_sonar, &Output_front, &Setpoint_front,P,I,D, DIRECT);
PID PID_left(&left_sonar, &Output_left, &Setpoint_left,P,I,D, DIRECT);
PID PID_rear(&rear_sonar, &Output_rear, &Setpoint_rear,P,I,D, DIRECT);


#include <i2c_t3.h>
//The Arduino Wire library uses the 7-bit version of the address, so the code example uses 0x70 instead of the 8‑bit 0xE0
#define frontI2C byte(0x70) //A
#define rearI2C byte(0x71)  //B
#define leftI2C byte(0x81)  //C
#define rightI2C byte(0x82)  //D
//The Sensor ranging command has a value of 0x51
#define RangeCommand byte(0x51)

short rangeA=-2;
short rangeB=-2;
short rangeC=-2;
short rangeD=-2;

bool Afn=0;
bool Bfn=0;
bool Cfn=0;
bool Dfn=0;
size_t Asz=0;
size_t Bsz=0;
size_t Csz=0;
size_t Dsz=0;

//Commands the sensor to take a range reading
void takeRangeReading(byte Address){
  Wire.beginTransmission(Address);             //Start addressing 
  Wire.write(RangeCommand);                             //send range command 
  Wire.endTransmission(I2C_STOP);                                  //Stop and do something else now
}    

//Returns the last range that the sensor determined in its last ranging cycle in centimeters. Returns 0 if there is no communication. 
short readRange(byte Address){ 
  size_t sz = Wire.requestFrom(Address, byte(2),I2C_STOP);
  //Wire.beginTransmission(Address); 

  if(Wire.available() >= 2){                            //Sensor responded with the two bytes 
    byte HighByte = Wire.read();                        //Read the high byte back 
    byte LowByte = Wire.read();                        //Read the low byte back 
    //word range = word(HighByte, LowByte);         //Make a 16-bit word out of the two bytes for the range 
    short range = (short)(LowByte + (HighByte << 8));
    //ushort value2 = (ushort)(port1 + (port2 << 8));
    return range; 
  }
  else { 
    return (short)(-1);                                             //Else nothing was received, return 0 
  }
}

void sortOutI2C(){
        Afn=0;
        Bfn=0;
        Cfn=0;
        Dfn=0;
        Asz=0;
        Bsz=0;
        Csz=0;
        Dsz=0;        
        rangeA=-2;
        rangeB=-2;
        rangeC=-2;
        rangeD=-2;
       
        int timeout=millis()+PIDSampleTime;
        //(Afn!=1)and(Bfn!=1)and(Cfn!=1)
        while ((1==1)and(timeout>millis())) {                
                if ((Asz==2)){if(Afn==0){rangeA = readRange(frontI2C);Afn=1;}}else{Asz = Wire.requestFrom(frontI2C, byte(2));}               
                if ((Bsz==2)){if(Bfn==0){rangeB = readRange(rearI2C);Bfn=1;}}else{Bsz = Wire.requestFrom(rearI2C, byte(2));}                
                //if ((Csz==2)){if(Cfn==0){rangeC = readRange(leftI2C);Cfn=1;}}else{Csz = Wire.requestFrom(leftI2C, byte(2));}
                //if ((Dsz==2)){if(Dfn==0){rangeD = readRange(rightI2C);Dfn=1;}}else{Dsz = Wire.requestFrom(rightI2C, byte(2));}
        }          
        if (Afn){rangeA = readRange(frontI2C);}  
        if (Bfn){rangeB = readRange(rearI2C);}
        //if (Cfn){rangeC = readRange(leftI2C);}
        //if (Dfn){rangeD = readRange(rightI2C);}
        //Serial.print(" info:");Serial.print(Asz);Serial.print(Bsz);//Serial.println(Csz);
}

//Meguno Link
char channelName[ ] = "debug";

#include <PulsePosition.h>

PulsePositionOutput PPMout;
PulsePositionInput PPMin;

//RC Inputs/outputs
double channel[8];
double pitch_in=1500; //channel 4
double roll_in=1500;  //channel 2
double throttle_in=1000;  //channel 3
double yaw_in=1500;  //channel 1
double mode_switch=1000;  //channel 5
double aux1=1000;  //channel 6
double aux2=1000;  //channel 7
double gear=1000;  //channel 8
int compd_pitch, compd_roll, compd_throttle;

int ConstrainPWM(int PWM_out, int MinPW, int MaxPW){
  if (PWM_out<MinPW) PWM_out=MinPW;
  if (PWM_out>MaxPW) PWM_out=MaxPW;
  return PWM_out;
}

void readPPM(){
  int i, numCh;
  numCh = PPMin.available();
  if (numCh > 0) {
    for (i=1; i <= numCh; i++) {
      channel[i]= PPMin.read(i);
    }
  }
}

void writePPM(){
  PPMout.write(3, compd_throttle);
  PPMout.write(4, compd_pitch);
  PPMout.write(2, compd_roll);
  PPMout.write(1, yaw_in);
  PPMout.write(5, mode_switch);
  PPMout.write(6, aux1);
  PPMout.write(7, aux2);
  PPMout.write(8, gear);
}

#include "SatelliteReceiver.h"
SatelliteReceiver Rx;

void readSpektrum(){
  Rx.getFrame();
  //roll_in=Rx.getAile();
  //pitch_in=Rx.getElev();
  //throttle_in=Rx.getThro();
  //yaw_in=Rx.getRudd();
  //mode_switch=Rx.getFlap();
  //aux1= Rx.getGear();
  aux2= Rx.getAux2();
}


void setup() {
  Serial.begin(57600);
  //Serial1.begin(115200); //Spektrum serial
  
  PPMout.begin(5);
  PPMin.begin(6);

  Wire.begin(I2C_MASTER, 0,0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100); //Initiate Wire library for I2C communications with I2CXL‑MaxSonar‑EZ
  
  //Initiate sonar ranging ready for 1st loop
    sortOutI2C();
    takeRangeReading(frontI2C);
    takeRangeReading(rearI2C);
    //takeRangeReading(leftI2C);
    //takeRangeReading(rightI2C);

  //Initialise PID loops
  Setpoint_right= Setpoint_front = Setpoint_left = Setpoint_rear = safe_distance;

  PID_right.SetMode(AUTOMATIC);
  PID_right.SetSampleTime(PIDSampleTime);
  PID_right.SetOutputLimits(0,OutMax);

  PID_front.SetMode(AUTOMATIC);
  PID_front.SetSampleTime(PIDSampleTime);
  PID_front.SetOutputLimits(0,OutMax);

  PID_left.SetMode(AUTOMATIC);
  PID_left.SetSampleTime(PIDSampleTime);
  PID_left.SetOutputLimits(0,OutMax);

  PID_rear.SetMode(AUTOMATIC);
  PID_rear.SetSampleTime(PIDSampleTime);
  PID_rear.SetOutputLimits(0,OutMax);
  //pinMode(piezzo,OUTPUT); //Buzzer for PID output, find unused pin


  //Intialise loop timers
  report_time	= millis();
  work_time	= millis();
}

void buzzer(){
  //buzzer
  /*
     if (aux2>1600){
   if(right_sonar<(Setpoint_right+20)) analogWrite(piezzo,Output_right);
   else analogWrite(piezzo,0);
   }
   else analogWrite(piezzo,0);
   */
}

void report(){
  report_time = millis();
  Serial.print(F("{TIMEPLOT:PID|data|Output_right|T|"));
  Serial.print(Output_right);
  Serial.print(F("}"));
  
  Serial.print(F("{TIMEPLOT:PID|data|Output_front|T|"));
  Serial.print(Output_front);
  Serial.print(F("}"));
  
  Serial.print(F("{TIMEPLOT:PID|data|Output_rear|T|"));
  Serial.print(Output_rear);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|Output_left|T|"));
  Serial.print(Output_left);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|safe_distance|T|"));
  Serial.print(safe_distance);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|RightSonar|T|"));
  Serial.print(right_sonar);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|FrontSonar|T|"));
  Serial.print(front_sonar);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|RearSonar|T|"));
  Serial.print(rear_sonar);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|LeftSonar|T|"));
  Serial.print(left_sonar);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:PID|data|AileronOut|T|"));
  Serial.print(compd_roll);
  Serial.print(F("}"));
  
  Serial.print(F("{TIMEPLOT:PID|data|ElevonOut|T|"));
  Serial.print(compd_pitch);
  Serial.print(F("}"));

  Serial.print(F("{TIMEPLOT:RC|data|Aileron|T|"));
  Serial.print(roll_in);
  Serial.println(F("}"));
  
  Serial.print(F("{TIMEPLOT:RC|data|Elevon|T|"));
  Serial.print(pitch_in);
  Serial.println(F("}"));
  
  Serial.print(F("{TIMEPLOT:RC|data|Rudder|T|"));
  Serial.print(yaw_in);
  Serial.println(F("}"));
  
  Serial.print(F("{TIMEPLOT:RC|data|throttle|T|"));
  Serial.print(throttle_in);
  Serial.println(F("}"));

  Serial.print(F("{TIMEPLOT:RC|data|Mode|T|"));
  Serial.print(mode_switch);
  Serial.println(F("}"));
  
   Serial.print(F("{TIMEPLOT:RC|data|Aux1|T|"));
   Serial.print(aux1);
   Serial.println(F("}"));
   
   /*Serial.print(F("{TIMEPLOT:PIDsettings|data|Kd|T|"));
   Serial.print(PID_right.GetKd());
   Serial.println(F("}"));
   
   Serial.print(F("{TIMEPLOT:PIDsettings|data|Ki|T|"));
   Serial.print(PID_right.GetKi());
   Serial.println(F("}"));
   
   Serial.print(F("{TIMEPLOT:PIDsettings|data|Kp|T|"));
   Serial.print(PID_right.GetKp());
   Serial.println(F("}"));
   
   Serial.print(F("{TIMEPLOT:Variables|data|millis()|T|"));
   Serial.print(millis());
   Serial.println(F("}"));

   Serial.print(F("{TIMEPLOT:Variables|data|tmp_time|T|"));
   Serial.print(tmp_time);
   Serial.println(F("}"));
   
   
   Serial.print("{MESSAGE:");
   Serial.print(channelName);
   Serial.print("|data|");
   Serial.print(millis());
   Serial.print(",");
   Serial.print(TCNT1);
   Serial.print(",");
   Serial.print(right_sonar);
   Serial.println("}");
   
   */
  //Serial.print(" \n");

}

void workloop(){
  work_time = millis();

  //Refresh sensor readings
    sortOutI2C();
    front_sonar= rangeA;  //read I2C sonar range, Value in cm
    rear_sonar= rangeB; //read I2C sonar range, Value in cm
    //left_sonar= rangeC; //read I2C sonar range, Value in cm
    //right_sonar= rangeD; //read I2C sonar range, Value in cm

  //Refresh RC inputs  
  //readSpektrum();  
    readPPM();
    pitch_in=channel[4];
    roll_in=channel[2];
    throttle_in=channel[3];
    yaw_in=channel[1];
    mode_switch=channel[5];
    aux1=channel[6];
   
  //Run PID loops
  PID_rear.Compute();
  PID_right.Compute();
  PID_front.Compute();
  PID_left.Compute();
  

  //Start next ranging cycle
    takeRangeReading(frontI2C);
    takeRangeReading(rearI2C);
    //takeRangeReading(leftI2C);
    //takeRangeReading(rightI2C);


  //Do we want obstacle avoidance on?
  if(aux1>1400){
    compd_roll=(roll_in-int(Output_right)+int(Output_left));
    compd_pitch=(pitch_in-int(Output_rear)+int(Output_front)); //remember to check the direction of pitch before testing
    compd_roll=ConstrainPWM(compd_roll,1100,1950);
    compd_pitch=ConstrainPWM(compd_pitch,1100,1950);
    compd_throttle=throttle_in;
  }
  else {
    compd_roll=roll_in;
    compd_pitch=pitch_in;
    compd_throttle=throttle_in;
  }

  //buzzer();
  writePPM();
}


void loop() {
  tmp_time=millis();

  if (tmp_time  >report_time+ 66){
    report();
  } 
  if (tmp_time  >work_time + PIDSampleTime){
    workloop();
  }

}























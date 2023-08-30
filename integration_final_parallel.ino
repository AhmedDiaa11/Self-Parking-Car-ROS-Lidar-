#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#include <filters.h>
#include <filters_defs.h>
#include "Wire.h"
//#include <MPU6050_light.h>

//MPU6050 mpu(Wire);
unsigned long timer = 0;

volatile unsigned char pulse_left_motor=0;
volatile unsigned char pulse_right_motor=0;

#define PIN_SLIDE_POT_A A2  //steering
#define MotorIN1Pin 42   //steering
//IN 1>> PIN 8
//IN 2>> PIN 7
#define MotorIN2Pin 40   //steering
#define MotorEnablePin 6  //steering
//-30 RIGHT , 30 LEFT
//blue >>out1 , yellow >>out2

#define MotorSLIN1Pin 22   //Left motor
#define MotorSLIN2Pin 24   //Left motor
#define MotorSLEnablePin A0   //Left motor
#define MotorL 5        //Left motor
#define LEFT_MOTOR_SENSOR 19

//18 right 19 
#define MotorSRIN1Pin 28  //Right motor
#define MotorSRIN2Pin 30   //Right motor
#define MotorSREnablePin A1   //Right motor
#define MotorR 9        //Right motor
#define RIGHT_MOTOR_SENSOR 2
// BR trig = 10<grey>, echo=11<yellow>
//BL TRig =46<orange>,echo =44<yellow>
// FR TRIG=31  <GREEN>,ECHO=33 <YEELOW>

//front right
#define  FR_trig  31
#define FR_echo  33
//back right
#define  BR_trig  10
#define BR_echo  11
 

//std_msgs::Float64 imu_msg;



double error,value_slide_pot_a ,angle;   //steering

unsigned long start;
volatile long pulse;
volatile bool pinB, pinA, dir;
const byte ppr = 130, upDatesPerSec = 2;
const int fin = 1000 / upDatesPerSec;
const float konstant = 60.0 * upDatesPerSec / (ppr * 2);
double rps;


IIR::ORDER order = IIR::ORDER::OD2;
double filtered=0;
Filter f1(50, 0.001, order, TYPE::LOWPASS);

int arr[4]={0,0,0,0};
std_msgs::Float64 fr_data;
std_msgs::Float64 br_data;

std_msgs::Float32 Speed_msg;
ros::Publisher RealSpeed("Real_Speed", &Speed_msg);
ros::Publisher fr("fr", &fr_data);
ros::Publisher br("br", &br_data);

// defines variables
long duration;

ros::NodeHandle  nh;

void speed_control( const std_msgs::Int32& data){
  digitalWrite(MotorSREnablePin,HIGH);
  digitalWrite(MotorSLEnablePin,HIGH);
  
  if (data.data > 0){
    

    digitalWrite(MotorSLIN1Pin, HIGH);
    digitalWrite(MotorSLIN2Pin, LOW);
    digitalWrite(MotorSRIN1Pin, HIGH);
    digitalWrite(MotorSRIN2Pin, LOW);
  }

  else if(data.data < 0){

    digitalWrite(MotorSLIN1Pin, LOW);
    digitalWrite(MotorSLIN2Pin, HIGH);
    digitalWrite(MotorSRIN1Pin, LOW);
    digitalWrite(MotorSRIN2Pin, HIGH);
  }
  else{
  digitalWrite(MotorSREnablePin,LOW);
  digitalWrite(MotorSLEnablePin,LOW);
    digitalWrite(MotorSLIN1Pin, LOW);
    digitalWrite(MotorSLIN2Pin, LOW);
    digitalWrite(MotorSRIN1Pin, LOW);
    digitalWrite(MotorSRIN2Pin, LOW);
  }
  analogWrite(MotorR, abs(data.data) );
  analogWrite(MotorL, abs(data.data));
  

}

void steering_control( const std_msgs::Int32& setpoint){
  while(1){
 double value_slide_pot_a = analogRead(PIN_SLIDE_POT_A);
  double angle = map(value_slide_pot_a,9,935,-30,30);
 angle=f1.filterIn(angle);
  error=setpoint.data-angle;
  //  Serial.println(angle);

  if (abs(error)>4                                                                                                                                     ){
    if (error > 0)
    {
      //TURN LEFT
       Serial.println("TURN left");

      digitalWrite(MotorIN1Pin, LOW);
      digitalWrite(MotorIN2Pin, HIGH);
      analogWrite(MotorEnablePin, 200);
      if(setpoint.data==30){
        delay(300);
      }
    }
    else
    {
      //TURN RIGHT
//Serial.println("TURN right");

      digitalWrite(MotorIN1Pin, HIGH);
      digitalWrite(MotorIN2Pin, LOW);
      analogWrite(MotorEnablePin, 200);
    }
    if(setpoint.data==-30){
        delay(300);
      }

  }
  else
  {
   // Serial.println("TURN stop");
    digitalWrite(MotorIN1Pin, LOW);
    digitalWrite(MotorIN2Pin, LOW);
    delay(100);

    break;

  }
  }
}


ros::Subscriber<std_msgs::Int32> sub("PWM_Values_A", steering_control);
ros::Subscriber<std_msgs::Int32> sub1("PWM_Values_S", speed_control);


void setup() {
    Serial.begin(57600);
     nh.initNode();
 

  
  //byte status = mpu.begin();
 // Serial.print(F("MPU6050 status: "));
  //Serial.println(status);
  //while(status!=0){ } // stop everything if could not connect to MPU6050
  
 // Serial.println(F("Calculating offsets, do not move MPU6050"));
  //delay(1000);
    //Serial.println("Done!\n");

  nh.advertise(fr);
    nh.advertise(br);

  nh.subscribe(sub);    //STEERING
  nh.subscribe(sub1);   //SPEED
  //nh.subscribe(sub1);
  nh.advertise(RealSpeed);

  pinMode(FR_trig, OUTPUT); // Sets the trigPin as an Output
  pinMode(FR_echo, INPUT); // Sets the echoPin as an Input

  pinMode(BR_trig, OUTPUT); // Sets the trigPin as an Output
  pinMode(BR_echo, INPUT); // Sets the echoPin as an Input

  pinMode(PIN_SLIDE_POT_A, INPUT );
  pinMode(MotorIN1Pin, OUTPUT); //steering
  pinMode(MotorIN2Pin, OUTPUT);  //steering
  pinMode(MotorEnablePin, OUTPUT);  //steering
    

  //right motor
  pinMode(MotorSRIN1Pin, OUTPUT);
  pinMode(MotorSRIN2Pin, OUTPUT);
  pinMode(MotorSREnablePin, OUTPUT);
  pinMode(MotorR, OUTPUT);
    //left motor
  pinMode(MotorSLIN1Pin, OUTPUT);
  pinMode(MotorSLIN2Pin, OUTPUT);
  pinMode(MotorSLEnablePin, OUTPUT);
  pinMode(MotorL, OUTPUT);
  
  

}

void loop() {
  check_safety();  //ultrasonic

  value_slide_pot_a = analogRead(PIN_SLIDE_POT_A);

  //msg_arr.data = arr[4];    //ultrasonic
  //msg_arr.data_length =4;   //ultrasonic
  //Pub_Arr.publish( &msg_arr );   //ultrasonic
  
  //mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
  //Serial.print("X : ");
  //Serial.print(mpu.getAngleX());
  //Serial.print("\tY : ");
  //Serial.print(mpu.getAngleY());
  //Serial.print("\tZ : ");
  //Serial.println(mpu.getAngleZ());
  timer = millis();  
  }
 if(digitalRead(LEFT_MOTOR_SENSOR)==LOW)
  {
              pulse_right_motor++;
             // Serial.println(pulse_right_motor);
              delay(100);
  }
  if(millis() - start > 1000)
  { 
    start = millis();
    rps = pulse_right_motor;
    //Serial.print(" SPEED: ");
    //Serial.println(rps);
    Speed_msg.data = rps * 0.11;//(0.1*2*3.14*rpm)/60 ;
    pulse_right_motor = 0;
  }
   

 
    fr.publish( &fr_data );
  br.publish( &br_data );

  RealSpeed.publish( &Speed_msg);

  nh.spinOnce();
    delay(1);
    
  
}

void check_safety(void){  //ultrasonic
  
  fr_data.data=data(FR_trig,FR_echo);
  br_data.data=data(BR_trig,BR_echo);


}
float data( int trigPin, int echoPin){  //ultrasonic
 digitalWrite(trigPin,LOW);
 delayMicroseconds(2);
 digitalWrite(trigPin,HIGH);
 delayMicroseconds(10);
 digitalWrite(trigPin, LOW);
 duration=pulseIn (echoPin,HIGH);
 return duration*0.034/2;
}

#include <util/atomic.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <FastLED.h>
#include <math.h>


#define NUM_LEDS 7
CRGB leds[NUM_LEDS];

ros::NodeHandle nh;
// Pins

#define ENCRA 21
#define ENCRB 20
#define PWMR 12
#define IN3 10
#define IN4 11

// globals
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;

float eintegral = 0;

#define ENCLA 19
#define ENCLB 18
#define PWML 7
#define IN1 8
#define IN2 9

// globals
long lprevT = 0;
int lposPrev = 0;
volatile int lpos_i = 0;
volatile float lvelocity_i = 0;
volatile long lprevT_i = 0;

float lv1Filt = 0;
float lv1Prev = 0;

float leintegral = 0;

float right = 0.0;
float left = 0.0;

int dirr;
int dirl;

void messageCb( const geometry_msgs::Twist &cmd_vel){
  right = cmd_vel.linear.x+((cmd_vel.angular.z*0.210)/2);
  left = cmd_vel.linear.x-((cmd_vel.angular.z*0.210)/2);

//  m/s to rpm = ((60.0*right) / (wheel dia*pi))
  right = float((60.0*right) / (0.068*3.14159));
  left = float((60.0*left) / (0.068*3.14159));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

std_msgs::Float64 right_w;
ros::Publisher right_wheel_p("right_wheel", &right_w);
std_msgs::Float64 left_w;
ros::Publisher left_wheel_p("left_wheel", &left_w);

void setup() {
  pinMode(ENCRA,INPUT);
  pinMode(ENCRB,INPUT);
  pinMode(ENCLA,INPUT);
  pinMode(ENCLB,INPUT);
  
  pinMode(PWMR,OUTPUT);
  pinMode(PWML,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  FastLED.addLeds<NEOPIXEL, 14>(leds, NUM_LEDS);

  attachInterrupt(digitalPinToInterrupt(ENCRA),
                  readEncoderR,RISING);

  attachInterrupt(digitalPinToInterrupt(ENCLA),
                  readEncoderL,RISING);
                  
  nh.initNode();
  nh.advertise(right_wheel_p);
  nh.advertise(left_wheel_p);
  nh.subscribe(sub);
  Serial.begin(9600);
}

void loop() {

//  right = 88;
//  left = 88;
  
  if(right>0){
    leds[0] = CRGB::Green; FastLED.show();
    leds[1] = CRGB::Green; FastLED.show();
    leds[2] = CRGB::Green; FastLED.show();
    dirr = 1;
  }
  else if(right<0){
    leds[4] = CRGB(32,32,32); FastLED.show();
    leds[5] = CRGB(32,32,32); FastLED.show();
    leds[6] = CRGB(32,32,32); FastLED.show();
    dirr = -1;
  }
  else if(right==0){
    leds[0] = CRGB::Red; FastLED.show();
    leds[1] = CRGB::Red; FastLED.show();
    leds[2] = CRGB::Red; FastLED.show();
    dirr = 0;
  }

  
  if(left>0){
    leds[4] = CRGB::Green; FastLED.show();
    leds[5] = CRGB::Green; FastLED.show();
    leds[6] = CRGB::Green; FastLED.show();
    dirl = 1;
  }
  else if(left<0){
    leds[0] = CRGB(32,32,32); FastLED.show();
    leds[1] = CRGB(32,32,32); FastLED.show();
    leds[2] = CRGB(32,32,32); FastLED.show();
    dirl = -1;
  }
  else if(left==0){
    leds[4] = CRGB::Red; FastLED.show();
    leds[5] = CRGB::Red; FastLED.show();
    leds[6] = CRGB::Red; FastLED.show();
    dirl = 0;
  }

//  int right_rpm = abs(right);
//  right_rpm = map(right_rpm, 0, 300, 0, 255);
//  int left_rpm = abs(left);
//  left_rpm = map(left_rpm, 0, 300, 0, 255);

//  int rpm = 100;
//  int pwm = map(rpm, 0, 260, 0, 255);
//  setMotor(1, pwm, 12, 10, 11);
//  setMotor(dirl, left_rpm, 7, 9, 8);
  
  int pos = 0;
  int lpos=0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    lpos = lpos_i;
  }

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;

  long lcurrT = micros();
  float ldeltaT = ((float) (lcurrT-lprevT))/1.0e6;
  float lvelocity1 = (lpos - lposPrev)/ldeltaT;
  
  posPrev = pos;
  prevT = currT;
  
  lposPrev = lpos;
  lprevT = lcurrT;

  // Convert count/s to RPM
  float v1 = velocity1/825.0*60.0;
  float lv1 = lvelocity1/825.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;

  lv1Filt = 0.854*lv1Filt + 0.0728*lv1 + 0.0728*lv1Prev;
  lv1Prev = lv1;

  float vt =130;
//  float lvt =left*(sin(currT/1e6)>0);
  float lvt = 130;

  // Compute the control signal u
  float kp = 2.25;
  float ki = 4.2;
  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;

  float lkp = 2.25;
  float lki = 4.2;
  float le = lvt-lv1Filt;
  leintegral = leintegral + le*ldeltaT;
  
  float u = kp*e + ki*eintegral;
  float lu = lkp*le + lki*leintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }

  int ldir = 1;
  if (lu<0){
    ldir = -1;
  }
  int lpwr = (int) fabs(lu);
  if(lpwr > 255){
    lpwr = 255;
  }

  int right_rpm = abs(vt);
  pwr = map(right_rpm, 0, 160, 0, 255);
  int left_rpm = abs(lvt);
  lpwr = map(left_rpm, 0, 160, 0, 255);
  
//  setMotor(dirl, left_rpm, 7, 9, 8);
  setMotor(dir, pwr, 12, 10, 11);
  setMotor(ldir, lpwr, 7, 8, 9);

  Serial.print(v1Filt);
  Serial.print (" ");
  Serial.print(lv1Filt);
  Serial.print (" ");
  Serial.print(vt);
  Serial.println();
  

  right_w.data = right;
  right_wheel_p.publish( &right_w );

  left_w.data = v1Filt;
  left_wheel_p.publish( &left_w );

  nh.spinOnce();
  delay(1);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void readEncoderR(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCRB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;
}

void readEncoderL(){
  // Read encoder B when ENCA rises
  int lb = digitalRead(ENCLB);
  int lincrement = 0;
  if(lb>0){
    // If B is high, increment forward
    lincrement = 1;
  }
  else{
    // Otherwise, increment backward
    lincrement = -1;
  }
  lpos_i = lpos_i + lincrement;
}

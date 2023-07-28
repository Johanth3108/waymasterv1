#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <FastLED.h>
#include <geometry_msgs/Vector3Stamped.h>

// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
    // store previous error
    eprev = e;
  }
  
};

#define NUM_LEDS 7
CRGB leds[NUM_LEDS];

ros::NodeHandle nh;

// How many motors
#define NMOTORS 2

// Pins
const int enca[] = {21,19};
const int encb[] = {20,18};
const int pwm[] = {12,7};
const int in1[] = {10,8};
const int in2[] = {11,9};

// Globals
long prevT = 0;
volatile int posi[] = {0,0};

// PID class instances
SimplePID pid[NMOTORS];

float right = 0.0;
float left = 0.0;

int op_right = 0;
int op_left = 0;

float target_f[] = {0.0,0.0};
long target[] = {0,0};

int encPrevr = 0;
int encPrevl = 0;

double right_speed = 0.0;
double left_speed = 0.0;

void setTarget(float t, float deltat, int mot_num){
  float positionChange[2] = {0.2, 0.2};
  float pulsesPerTurn = 11*18.75;
  float pulsesPerMeter = pulsesPerTurn*4.7619;
  
//  mot_num 1 is right and mot_num 2 is left

  if(mot_num==1){
    float velocity = t;
    positionChange[0] = velocity*deltat*pulsesPerTurn;
    target_f[0] = target_f[0]+positionChange[0];
  }
  if(mot_num==2){
    float velocity = t;
    positionChange[1] = velocity*deltat*pulsesPerTurn;
    target_f[1] = target_f[1]+positionChange[1];
  }
//  float velocity = t;
//  for(int k =0; k<2; k++){
//    positionChange[k] = velocity*deltat*pulsesPerTurn;
//  }
//  for(int k=0; k<2; k++){
//    target_f[k] = target_f[k]+positionChange[k];
//  }
  target[0] = (long) target_f[0];
  target[1] = (long) target_f[1];
  
}

void messageCb( const geometry_msgs::Twist &cmd_vel){
  right = cmd_vel.linear.x+((cmd_vel.angular.z*0.210)/2);
  left = cmd_vel.linear.x-((cmd_vel.angular.z*0.210)/2);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

//geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
//ros::Publisher speed_pub("speed", &speed_msg);
std_msgs::Int16 right_w;
ros::Publisher right_wheel_p("right_wheel", &right_w);
std_msgs::Int16 left_w;
ros::Publisher left_wheel_p("left_wheel", &left_w);

void setup() {
//  Serial.begin(9600);

  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);

    pid[k].setParams(9.5,0.52,0,255);
  }

  FastLED.addLeds<NEOPIXEL, 14>(leds, NUM_LEDS);
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  
//  Serial.println("target pos");
  nh.initNode();
  nh.advertise(right_wheel_p);
  nh.advertise(left_wheel_p);
  nh.subscribe(sub);
}

void loop() {

  // set target position
//  int target[NMOTORS];
//  target[0] = 1200*sin(prevT/1e6);
//  target[1] = -1200*sin(prevT/1e6);

  if(right>0){
    leds[0] = CRGB::Green; FastLED.show();
    leds[1] = CRGB::Green; FastLED.show();
    leds[2] = CRGB::Green; FastLED.show();
  }
  else if(right<0){
    leds[4] = CRGB(255,255,0); FastLED.show();
    leds[5] = CRGB(255,255,0); FastLED.show();
    leds[6] = CRGB(255,255,0); FastLED.show();
  }
  else if(right==0){
    leds[0] = CRGB::Red; FastLED.show();
    leds[1] = CRGB::Red; FastLED.show();
    leds[2] = CRGB::Red; FastLED.show();
  }

  
  if(left>0){
    leds[4] = CRGB::Green; FastLED.show();
    leds[5] = CRGB::Green; FastLED.show();
    leds[6] = CRGB::Green; FastLED.show();
  }
  else if(left<0){
    leds[0] = CRGB(255,255,0); FastLED.show();
    leds[1] = CRGB(255,255,0); FastLED.show();
    leds[2] = CRGB(255,255,0); FastLED.show();
  }
  else if(left==0){
    leds[4] = CRGB::Red; FastLED.show();
    leds[5] = CRGB::Red; FastLED.show();
    leds[6] = CRGB::Red; FastLED.show();
  }

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

//  setTarget(currT/1.0e6, deltaT);
  setTarget(right, deltaT, 1);
  setTarget(left, deltaT, 2);
  
  
  // Read the position
  int pos[NMOTORS];
  noInterrupts(); // disable interrupts temporarily while reading
  for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  interrupts(); // turn interrupts back on
  
  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // evaluate the control signal
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
  }

//  for(int k = 0; k < NMOTORS; k++){
//    Serial.print(target[k]);
//    Serial.print(" ");
//    Serial.print(pos[k]);
//    Serial.print(" ");
//  }
//  Serial.println();

//  int encDiffr = op_right-encPrevr;
//  int encDiffl = op_left-encPrevl;
//  
//  right_speed = encDiffr/4.290;
//  left_speed = encDiffl/4.290;
//  double theta = (encDiffr-encDiffl)/0.21;
//  
//  encPrevr = op_right;
//  encPrevl = op_left;

  right_w.data = op_right;
  right_wheel_p.publish( &right_w );

  left_w.data = op_left;
  left_wheel_p.publish( &left_w );
//  publishSpeed(10, right_speed, left_speed, theta);
  nh.spinOnce();
  delay(1);
}

//void publishSpeed(double time, double rspeed, double lspeed, double theta) {
//  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
//  speed_msg.vector.x = rspeed;    //left wheel speed (in m/s)
//  speed_msg.vector.y = lspeed;   //right wheel speed (in m/s)
//  speed_msg.vector.z = theta;         //looptime, should be the same as specified in LOOPTIME (in s)
//  speed_pub.publish(&speed_msg);
////  nh.loginfo("Publishing odometry");
//}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
  
  if(j==0){
    op_right=posi[j];
  }
  else{
    op_left=posi[j];
  }
}
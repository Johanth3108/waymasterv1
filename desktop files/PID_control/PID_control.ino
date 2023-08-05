// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.

#define ENCA 20 // YELLOW
#define ENCB 21 // WHITE

#define ENCC 18 // YELLOW
#define ENCD 19 // WHITE
#define PWM 12
#define IN2 11
#define IN1 10

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int targetposi = 0; 
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(ENCC,INPUT);
  pinMode(ENCD,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCD),readEncoderC,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  int target = targetposi;
//  int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 9.5;
  float kd = 0.52;
  float ki = 0.001;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

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

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}

void readEncoderC(){
  int c = digitalRead(ENCC);
  if(c > 0){
    targetposi++;
  }
  else{
    targetposi--;
  }
}

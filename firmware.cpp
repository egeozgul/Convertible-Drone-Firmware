#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

#include<Servo.h>

#define PD2 B00000100 //4  PCINT18 ********
#define PD4 B00010000 //6  PCINT20
#define PB5 B00100000 //19 PCINT5
#define PB4 B00010000 //18 PCINT4
#define PC1 B00000010 //24 PCINT9
#define PC0 B00000001 //23 PCINT8

int pwmMax = 2000;
int pwmMin = 900;

struct pwmState
{
  int center = 1500;
  unsigned long timer = 0;
  bool last_state  = 0;
  int pwmValue = 0;
};
/*
int servoMove(int a, int b, long startTime,long duration_millis)
{
  double d = b - a;
  
  long t = millis();
  if(t>startTime && t < startTime + duration_millis)
  {
    double f = (double)(t - startTime);
    return (double)a + (d*f)/(double)duration);  
  }
}*/

int pwmPinRead(struct pwmState *pinInfo, bool pinState,unsigned long currentTime)
{
  if(pinState) //if pin high
  {
    if(pinInfo->last_state == 0)
    {                                   //Input 8 changed from 0 to 1
      pinInfo->last_state = 1;           //Remember current input state
      pinInfo->timer = currentTime;     //Set timer_1 to current_time
    }
  }
  else if(pinInfo->last_state == 1)
  {                                //Input 8 is not high and changed from 1 to 0
    pinInfo->last_state = 0; 
    //Remember current input state
    int val = currentTime - pinInfo->timer;   //Channel 1 is current_time - timer_1
    pinInfo->pwmValue = val;//map(val,pwmMin,pwmMax,0,255);
  }
}




void PCI_setup()
{
  PCMSK2 = B00010100; //mask register
  PCMSK1 = B00000011; //mask register
  PCMSK0 = B00110000; //mask register
  PCICR |= (1 << PCIE0)|(1 << PCIE1)|(1 << PCIE2);
  DDRD = 0;
  DDRC = 0;
  DDRB = 0;
}

pwmState PinInfo_1;
pwmState PinInfo_2;
pwmState PinInfo_3;
pwmState PinInfo_4;
pwmState PinInfo_5;
pwmState PinInfo_6;

Servo m1,m2,m3,m4;
Servo s1,s2;

double centerY;
double centerX;
double centerZ;

class pid
{
  unsigned long currentTime, previousTime;
  double elapsedTime;
  double error;
  double lastError;
  double input, output, setPoint;
  double cumError, rateError;
  public:
  
  //PID constants
  double kp = 0;
  double ki = 0;               
  double kd = 0;

  double computePID(double inp){     
          currentTime = millis();                //get current time
          elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
          
          error = inp;                                // determine error
          cumError += error * elapsedTime;                // compute integral
          rateError = (error - lastError)/elapsedTime;   // compute derivative
   
          double out = kp*error + ki*cumError + kd*rateError;                //PID output               
   
          lastError = error;                                //remember current error
          previousTime = currentTime;                        //remember current time
   
          return out;                                        //have function return the PID output
  }
};

void blinkLED(int period)
{
  if(millis()%period*2<period)
    digitalWrite(7,HIGH);
  else if(millis()%period*2>period)
    digitalWrite(7,LOW);
}


void calibrateGyro(double *centerX,double *centerY,double *centerZ)
{
  mpu6050.calcGyroOffsets(true);
  //mpu6050.setGyroOffsets(3.933,-2.77,1.07);
 
  Serial.println("calculating the center positions of remote sticks and gyro..."); 
  
  for(int i = 0; i<300; i++)
  {
    mpu6050.update();
    Serial.print(mpu6050.getAngleX());
    Serial.print(" ");
    Serial.print(mpu6050.getAngleY());
    Serial.print(" ");
    Serial.println(mpu6050.getAngleZ());
    delay(50);
  } 

  *centerX = 0;
  *centerY = 0;
  *centerZ = 0;
  double n = 20; 
  for(int i = 0; i<n; i++)
  {
    mpu6050.update();
    *centerX += mpu6050.getAngleX();
    *centerY += mpu6050.getAngleY();
    *centerZ += mpu6050.getAngleZ();
    
    Serial.print(mpu6050.getAngleX());
    Serial.print(" ");
    Serial.print(mpu6050.getAngleY());
    Serial.print(" ");
    Serial.println(mpu6050.getAngleZ());
    delay(10);
  } 

  int pwm1=0,pwm2=0,pwm3=0,pwm4=0,pwm5=0,pwm6=0;
  int k =50;
  for(int i = 0;i<k;i++)
  {
    delay(10);
    pwm1 += PinInfo_1.pwmValue;
    pwm2 += PinInfo_2.pwmValue;
    pwm3 += PinInfo_3.pwmValue;
    pwm4 += PinInfo_4.pwmValue;
    pwm5 += PinInfo_5.pwmValue;
    pwm6 += PinInfo_6.pwmValue;
  }
  
  PinInfo_1.center = pwm1/k;
  PinInfo_2.center = pwm2/k;
  PinInfo_3.center = pwm3/k;
  PinInfo_4.center = pwm4/k;
  PinInfo_5.center = pwm5/k;
  PinInfo_6.center = pwm6/k;

  PinInfo_1.center = PinInfo_1.pwmValue;
  PinInfo_2.center = PinInfo_2.pwmValue;
  PinInfo_3.center = PinInfo_3.pwmValue;
  PinInfo_4.center = PinInfo_4.pwmValue;
  PinInfo_5.center = PinInfo_5.pwmValue;
  PinInfo_6.center = PinInfo_6.pwmValue;
  
  /*PinInfo_1.center = 1500;
  PinInfo_2.center = 1500;
  PinInfo_3.center = 1500;
  PinInfo_4.center = 1500;
  PinInfo_5.center = 1500;
  PinInfo_6.center = 1500;
*/
  Serial.print(PinInfo_1.center);
  Serial.print(" ");
  Serial.print(PinInfo_2.center);
  Serial.print(" ");
  Serial.print(PinInfo_3.center);
  Serial.print(" ");
  Serial.print(PinInfo_4.center);
  Serial.print(" ");
  Serial.print(PinInfo_5.center);
  Serial.print(" ");
  Serial.println(PinInfo_6.center);

  *centerX /= n;
  *centerY /= n;
  *centerZ /= n;
}

pid pid_roll;
pid pid_pitch;
pid pid_yaw;

void setup() 
{ 
  PCI_setup();
  Serial.begin(9600);
 
  pinMode(7,OUTPUT);
  pinMode(8,INPUT_PULLUP);
  
  digitalWrite(7,HIGH);
  digitalRead(8);
  
  Wire.begin();
  mpu6050.begin();
  
  delay(100); 
  
  s1.attach(11);
  m2.attach(10);
  m3.attach(9);
  m4.attach(6);
  m1.attach(5);
  s2.attach(3);
  
  m1.writeMicroseconds(pwmMin);
  m2.writeMicroseconds(pwmMin);
  m3.writeMicroseconds(pwmMin);
  m4.writeMicroseconds(pwmMin);
  s1.writeMicroseconds(1500-700);
  s2.writeMicroseconds(1500+500);
  
  pid_roll.kp = 4.0;
  pid_roll.ki = 0.0008;
  pid_roll.kd = 500.0;
  
  pid_pitch.kp = pid_roll.kp;
  pid_pitch.ki = pid_roll.ki;
  pid_pitch.kd = pid_roll.kd;
  
  pid_yaw.kp = 3;
  pid_yaw.ki = 0.0001;
  pid_yaw.kd = 300;

  Serial.println("pres the button to calibrate");
  
  while(digitalRead(8) == HIGH && !Serial.available())
    blinkLED(200);
    
  digitalWrite(7,HIGH);
  calibrateGyro(&centerX,&centerY,&centerZ);
  digitalWrite(7,LOW);

  Serial.println("calibration is done, press the button to start the drone");
  
  while(digitalRead(8) == HIGH && !Serial.available())
    blinkLED(200);
  
  digitalWrite(7,HIGH);
  delay(1000);
  digitalWrite(7,LOW);
}

void loop() 
{
  if(digitalRead(8) == LOW)
  {
    m1.writeMicroseconds(pwmMin);
    m2.writeMicroseconds(pwmMin);
    m3.writeMicroseconds(pwmMin);
    m4.writeMicroseconds(pwmMin);
    
    digitalWrite(7,HIGH);
    calibrateGyro(&centerX,&centerY,&centerZ);
    digitalWrite(7,LOW);
    
    while(digitalRead(8) == HIGH)
      blinkLED(200);
      
    digitalWrite(7,HIGH);
    delay(1000);
    digitalWrite(7,LOW);
  }

  Serial.setTimeout(40);
  if(Serial.available())
  {
    String w = Serial.readStringUntil(' ');
    float number = -1.0f;
    
    if(w == "kp")
      number = pid_pitch.kp = pid_roll.kp = Serial.parseFloat();
    else if(w == "ki")
      number = pid_pitch.ki = pid_roll.ki = Serial.parseFloat();
    else if(w == "kd")
      number = pid_pitch.kd = pid_roll.kd = Serial.parseFloat();
    else if(w == "yaw_kp")
      number = pid_yaw.kp = Serial.parseFloat();
    else if(w == "yaw_ki")
      number = pid_yaw.ki = Serial.parseFloat();
    else if(w == "yaw_kd")
      number = pid_yaw.kd = Serial.parseFloat();
    else if(w == "?" || w == "?\n")
    {      
      Serial.print("roll,pitch kp = ");
      Serial.println(pid_roll.kp);
      
      Serial.print("roll,pitch ki = ");
      Serial.println(pid_roll.ki);
      
      Serial.print("roll,pitch kd = ");
      Serial.println(pid_roll.kd);
       
      Serial.print("yaw kp = ");
      Serial.println(pid_yaw.kp);
      
      Serial.print("yaw ki = ");
      Serial.println(pid_yaw.ki);
      
      Serial.print("yaw kd = "); 
      Serial.println(pid_yaw.kd);
    }
      
    if(number != -1.0f)
      {
        Serial.print(w);
        Serial.print(" is set to ");
        Serial.println(number);
      }
      
      Serial.println();
  }
  
  mpu6050.update();
  /*Serial.print("angleX : ");
  Serial.print(mpu6050.getAngleX());
  Serial.print("\tangleY : ");
  Serial.print(mpu6050.getAngleY());
  Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ()); 
  */
  
  float stickSensitivity = 0.008f;//70.0f/1000.0f; //1000 to 70
  int pitch = pid_pitch.computePID(mpu6050.getAngleY()-centerY + (int)(PinInfo_4.pwmValue - PinInfo_4.center)*stickSensitivity);
  int roll  = pid_roll.computePID( mpu6050.getAngleX()-centerX + (int)(PinInfo_3.pwmValue - PinInfo_3.center)*stickSensitivity);
  int yaw   = pid_yaw.computePID(  mpu6050.getAngleZ()-centerZ + (int)(PinInfo_2.pwmValue - PinInfo_2.center)*stickSensitivity);
/*
Serial.print((PinInfo_2.pwmValue - PinInfo_2.center)*stickSensitivity);
Serial.print(" ");
Serial.print((PinInfo_3.pwmValue - PinInfo_3.center)*stickSensitivity);
Serial.print(" ");
Serial.println((PinInfo_4.pwmValue - PinInfo_4.center)*stickSensitivity);
*/
  
  int throttle = PinInfo_1.pwmValue;
  int offset = 350;
  if(throttle <= 1350)
  {
    m1.writeMicroseconds(pwmMin);
    m2.writeMicroseconds(pwmMin);
    m3.writeMicroseconds(pwmMin);
    m4.writeMicroseconds(pwmMin);
  }
  else if(throttle > 1250)
  {
    int t1 = throttle-pitch - roll  -yaw -offset;
    if(t1<1000)t<=1000;
    if(t1>1600)t<=1600;
    m1.writeMicroseconds(t1);

    int t2 = throttle+pitch + roll  -yaw -offset;
    if(t2<1000)t2<=1000;
    if(t2>1600)t2<=1600;
    m2.writeMicroseconds(t2);
    
    int t3 = throttle-pitch + roll  +yaw -offset;
    if(t3<1000)t<=1000;
    if(t3>1600)t<=1600;
    m3.writeMicroseconds(t3);
    
    int t4 = throttle+pitch - roll  +yaw -offset;
    if(t4<1000)t4<=1000;
    if(t4>1600)t4<=1600;
    m4.writeMicroseconds(t4);
    
    //m2.writeMicroseconds(throttle+pitch + roll  -yaw -offset);
    //m3.writeMicroseconds(throttle-pitch + roll  +yaw -offset);
    //m4.writeMicroseconds(throttle+pitch - roll  +yaw -offset);
  }
  /* Serial.print(throttle-pitch - roll  -yaw -offset);
   Serial.print(" ");
   Serial.print(throttle+pitch + roll  -yaw -offset);
   Serial.print(" ");
   Serial.print(throttle-pitch + roll  +yaw -offset);
   Serial.print(" ");
   Serial.println(throttle+pitch - roll  +yaw -offset);
*/  /*
  Serial.print(mpu6050.getAngleX()-centerX);
  Serial.print(" ");
  Serial.print(mpu6050.getAngleY()-centerY);
  Serial.print(" ");
  Serial.println(mpu6050.getAngleZ()-centerZ);
 */
  Serial.print(PinInfo_1.pwmValue);
  Serial.print(" ");
  Serial.print(PinInfo_2.pwmValue);
  Serial.print(" ");
  Serial.print(PinInfo_3.pwmValue);
  Serial.print(" ");
  Serial.print(PinInfo_4.pwmValue);
  Serial.print(" ");
  Serial.print(PinInfo_5.pwmValue);
  Serial.print(" ");  
  Serial.println(PinInfo_6.pwmValue);

/*
  m1.writeMicroseconds(PinInfo_1.pwmValue - PinInfo_1.center);
  m2.writeMicroseconds(PinInfo_2.pwmValue - PinInfo_2.center);
  m3.writeMicroseconds(PinInfo_3.pwmValue - PinInfo_3.center);
  m4.writeMicroseconds(PinInfo_4.pwmValue - PinInfo_4.center);
  s1.writeMicroseconds(PinInfo_5.pwmValue);
  s2.writeMicroseconds(PinInfo_6.pwmValue);*/
}


ISR(PCINT0_vect)
{
  unsigned long current_time = micros();
  
  pwmPinRead(&PinInfo_1,PINB&PB4,current_time);
  pwmPinRead(&PinInfo_2,PINB&PB5,current_time);
}

ISR(PCINT1_vect)
{
  unsigned long current_time = micros();
  
  pwmPinRead(&PinInfo_3,PINC&PC0,current_time);
  pwmPinRead(&PinInfo_4,PINC&PC1,current_time); 
}

ISR(PCINT2_vect)
{
  unsigned long current_time = micros();
  pwmPinRead(&PinInfo_5,PIND&PD2,current_time);
  pwmPinRead(&PinInfo_6,PIND&PD4,current_time);
}

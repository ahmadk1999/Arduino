#include <QTRSensors.h>
#define kp 0.3
#define kd 3
#define k1 0
#define MaxSpeed 80
#define BaseSpeed 60

#define speedturn 20

#define rightMotor1 A1
#define rightMotor2 A2
#define rightMotorPWM 10
#define leftMotor1 A4
#define leftMotor2 A5
#define leftMotorPWM 11

   int x1= digitalRead(2);
   int x2= digitalRead(3);
   int x3= digitalRead(4);
   int x4= digitalRead(5);
   int x5= digitalRead(6);



QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues [SensorCount];

 
void setup()
{

  
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {2 , 3 , 4 , 5 , 6  }, SensorCount);
  qtr.setEmitterPin(12);

  pinMode(rightMotor1,OUTPUT);
  pinMode(rightMotor2,OUTPUT);
  pinMode(rightMotorPWM,OUTPUT);
  pinMode(leftMotor1,OUTPUT);
  pinMode(leftMotor2,OUTPUT);
  pinMode(leftMotorPWM,OUTPUT);
  
  delay(3000);
    Serial.begin(9600);
    Serial.println();

  int i;
  for(int i = 0 ; i<100 ; i++)
  {
    if(i < 25 || i >= 75 )
    {
    move(1 , 70 ,1);
    move(0, 70 ,0);
      }
    else
    {
      move(1,70,0);
      move(0,70,1);
      }
      qtr.calibrate();
      delay(20);  
    
    }
    
    wait();
    delay(3000);
      
    

}
int lastError = 0; 
uint16_t position = qtr.readLineBlack(sensorValues);

void loop()
 {
   
  int lastError = 0; 
   position = qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  if(position>6500){
    move(1,speedturn,1);
    move(0,speedturn,0);
Serial.println("0");    
  }
  if(position<500 ){
    move(1,speedturn,0);
    move(0,speedturn,1);
    Serial.println("1");
    return;
    }

    Serial.println("2");
    int error = position - 2500;
    int motorSpeed = kp * error + kd *  (error - lastError);
    lastError = error;

    int rightMotorSpeed = BaseSpeed + motorSpeed;
    int leftMotorSpeed = BaseSpeed - motorSpeed;

    if(rightMotorSpeed > MaxSpeed) {rightMotorSpeed = MaxSpeed;}
    if(leftMotorSpeed > MaxSpeed) {leftMotorSpeed = MaxSpeed;}
    if(rightMotorSpeed < 0 ) {rightMotorSpeed = 0;}
    if(leftMotorSpeed < 0) {leftMotorSpeed = 0;}

    Serial.println("3");
    
    move(1, rightMotorSpeed , 1);
    move(0,leftMotorSpeed , 1);
   Serial.println("4");
   
   if(x1==1 && x2==1 && x3==1 && x4==1 && x5== 1){turnleft();
   }
   else if(x1==0 && x2==0 && x3==0 && x4==0 && x5== 0){turnaround();
   }
   
  }
    

 
 


void wait(){
    analogWrite(leftMotorPWM, 0);
    analogWrite(rightMotorPWM, 0);
 }

void move (int motor, int speed, int direction){

    boolean inPin1;
    boolean inPin2;
    if (direction ==1){
        inPin1 = HIGH;
        inPin2 = LOW;
    }
    if(direction == 0){
        inPin1 = LOW;
        inPin2 = HIGH;
    }
    if(motor == 0){
        digitalWrite(leftMotor1, inPin1);
        digitalWrite(leftMotor2, inPin2);
        analogWrite(leftMotorPWM, speed);
    }
    if(motor == 1){
        digitalWrite(rightMotor1, inPin1);
        digitalWrite(rightMotor2, inPin2);
        analogWrite(rightMotorPWM, speed);
    }
    }    

   void sensor_calibration(){
   Serial.println("qsl");   
    
    for (uint8_t i = 0; i < 400; i++){
        Serial.println("qsl11"); 
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW);
    for(uint8_t i = 0; i < SensorCount; i++){
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print("  ");
    }
    Serial.println();
    
    for(uint8_t i=0; i < SensorCount; i++){
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print("  ");
    }
    Serial.println("qsl");
    Serial.println();
    delay(1000);
   }
   
   void readsensors(){
   x1= digitalRead(2);
   x2= digitalRead(3);
   x3= digitalRead(4);
   x4= digitalRead(5);
   x5= digitalRead(6);

   }
void turnleft(){

digitalWrite(A1,HIGH);
digitalWrite(A2,LOW);
digitalWrite(A3,HIGH);
digitalWrite(A4,LOW);

readsensors();
if(x1==1 && x2==1 && x3==1 && x4==1 && x5== 1){
  digitalWrite(10,HIGH);
  digitalWrite(11,LOW);
  readsensors();

  
}
else{
  loop();
}

  
}   
void turnaround(){
  digitalWrite(A1,HIGH);
  digitalWrite(A2,LOW);
  digitalWrite(A3,LOW);
  digitalWrite(A4,HIGH);
  readsensors();
  if(x1==1 && x2==1 && x3==1 && x4==1 && x5== 1){
  digitalWrite(10,HIGH);
  digitalWrite(11,LOW);
  
  readsensors();
  }
  else{loop();}
}
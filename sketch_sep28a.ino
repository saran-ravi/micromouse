#include <NewPing.h>
#define max_dis 20
NewPing sonar_l(A0,A1,max_dis);
NewPing sonar_m(A2,A3,100);
NewPing sonar_r(A5,A4,max_dis);
float dis_left;
float dis_middle;
float dis_right,leftSensor,rightSensor,frontSensor,oldFrontSensor,oldLeftSensor,oldRightSensor;

int a;
#define encl 2
#define encr 3
float leftrot=0;
float rightrot=0;
#define mr1 4
#define mr2 5
#define ml1 6
#define ml2 7
#define enr 10
#define enl 11


int wall_threshold = 50 ;
int front_threshold = 60 ;
float oldErrorP ;
float totalError ;
int BaseSpeed = 100 ;

boolean frontwall ;
boolean leftwall ;
boolean rightwall ;
boolean first_turn ;
boolean rightWallFollow ;
boolean leftWallFollow ;
int rightMiddleValue=34;
int leftMiddleValue=34;
float errorP,errorI,errorD;

void setup()
{
Serial.begin(9600);
 pinMode(encl,INPUT);
 pinMode(encr,INPUT);
 attachInterrupt(digitalPinToInterrupt(encl),leftpulse,CHANGE);
 attachInterrupt(digitalPinToInterrupt(encr),rightpulse, CHANGE);
 pinMode(ml1,OUTPUT);

 pinMode(ml2,OUTPUT);
  pinMode(mr2,OUTPUT);
  pinMode(mr1,OUTPUT);
  pinMode(enl,OUTPUT);
  pinMode(enr,OUTPUT);
  
}

void loop() 
{
  //Serial.println("entered");
 ultrasonic();
 // walls();
  //move_cell();
 //rightfollow();
forward();
Serial.println(leftwall);
Serial.println(frontwall);
Serial.println(rightwall);
 
}

void leftfollow()
{
 BaseSpeed=110;
 ultrasonic();
  walls();
 errorP=38-leftSensor;
 errorD = (errorP - oldErrorP);
 oldErrorP=errorP;

 totalError=6*errorP+2*errorD*abs(errorD);
 analogWrite(enr,BaseSpeed - totalError);
 analogWrite(enl,BaseSpeed + totalError);
 if (totalError>110)
 {
  
 analogWrite(enr,0);
 if(totalError>145)
 {
  analogWrite(enl,255);
 }
}
 
 forward();
 delay(5);
}
void rightfollow()
{ 
  BaseSpeed=110;
 ultrasonic();
 errorP=40-rightSensor;
 errorD = errorP - oldErrorP;
 oldErrorP=errorP;
 
 totalError=4*errorP+2*errorD*abs(errorD);
 analogWrite(enr,BaseSpeed + totalError);
 analogWrite(enl,BaseSpeed - totalError);
 if (totalError>110)
 {
  
 analogWrite(enl,0);
 if(totalError>145)
 {
  analogWrite(enr,255);
 }
 
 }
 forward();
 delay(5);
}

void move_cell()
{
  leftrot=0;
  rightrot=0;
  ultrasonic();
  walls();
  while(leftrot<70 && rightrot<70)
  {//forward();
  if(rightwall)
    {
      rightfollow();
    }
  else if(leftwall)
    {
      leftfollow();
    }
   else
    { stopm();
      analogWrite(enr,BaseSpeed);
      analogWrite(enl,BaseSpeed);
      forward();
      delay(5);
     }
    
   }
}

void forward()
{
  digitalWrite(ml2,HIGH);
  digitalWrite(ml1,LOW);
  digitalWrite(mr2,HIGH);
  digitalWrite(mr1,LOW);
}
void backward()
{
  digitalWrite(ml1,HIGH);
  digitalWrite(ml2,LOW);
  digitalWrite(mr1,HIGH);
  digitalWrite(mr2,LOW);
}
void stopm()
{
  digitalWrite(ml1,LOW);
  digitalWrite(ml2,LOW);
  digitalWrite(mr1,LOW);
  digitalWrite(mr2,LOW);
}
void right()
{
  digitalWrite(ml2,HIGH);
  digitalWrite(ml1,LOW);
  digitalWrite(mr2,LOW);
  digitalWrite(mr1,HIGH);
}
void soft_right()
{
  digitalWrite(ml2,HIGH);
  digitalWrite(ml1,LOW);
  digitalWrite(mr2,LOW);
  digitalWrite(mr1,LOW);
}
void left()
{
  digitalWrite(ml2,LOW);
  digitalWrite(ml1,HIGH);
  digitalWrite(mr2,HIGH);
  digitalWrite(mr1,LOW);
}
void soft_left()
{
  digitalWrite(ml2,LOW);
  digitalWrite(ml1,HIGH);
  digitalWrite(mr2,LOW);
  digitalWrite(mr1,LOW);
}
void left_turn()
{ analogWrite(enr,110);
analogWrite(enl,110);
  leftrot=0;
  rightrot=0;
  while(leftrot<0.59 and rightrot<0.59)
  { //Serial.println(leftrot);
    left();
//    delay(10);
//    stop();
//    delay(2);
  }
}
void right_turn()
{
   analogWrite(enr,120);
  analogWrite(enl,80);
  leftrot=0;
  rightrot=0;
  while( rightrot<0.78)
  {
    //Serial.println(leftrot);
    right();
  }
}
void u_turn()
{
  leftrot=0;
  rightrot=0;
  while(leftrot<2.6)   
  {
    ///Serial.println(leftrot);
    left();
  }
}
void encoder()
{//Serial.println(leftrot);
 //Serial.println(rightrot);
 //delay(100);
}

void leftpulse()
{
  leftrot+=1;
}
void rightpulse()
{
 rightrot+=1;
}

void ultrasonic()
{
//   dis_left=sonar_l.ping_cm(); 
//   dis_middle=sonar_m.ping_cm(); 
//   dis_right=sonar_r.ping_cm(); 

  int duration1 = sonar_l.ping();
 
  leftSensor = (duration1 / 2) * 0.343;
  int duration2 = sonar_m.ping();
  frontSensor = (duration2 / 2) * 0.343;
  int duration3 = sonar_r.ping();
  rightSensor = (duration3 / 2) * 0.343;

  //leftSensor = (dis_left + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
  //rightSensor = (dis_right + oldRightSensor) / 2;
  //frontSensor = (dis_middle + oldFrontSensor) / 2;


  oldLeftSensor = leftSensor; // save old readings for movment
  oldRightSensor = rightSensor;
  oldFrontSensor = frontSensor;
  if (leftSensor < wall_threshold && leftSensor>0) {
    leftwall = true ;
  }
  else{
    leftwall = false ;
  }


  if( rightSensor < wall_threshold && rightSensor>0) {
    rightwall = true ;
  }
  else {
    rightwall = false ;


  } if (( frontSensor < front_threshold) && (frontSensor>0 )) {
    frontwall = true ;
  }
  else {
    frontwall = false ;
  }
  Serial.print("Distances in MM: ");
  Serial.println(leftSensor   );
  Serial.println(frontSensor   );
  Serial.println(rightSensor   );
}

void walls() {


  if (leftSensor < wall_threshold && leftSensor>0) {
    leftwall = true ;
  }
  else{
    leftwall = false ;
  }


  if( rightSensor < wall_threshold && rightSensor>0) {
    rightwall = true ;
  }
  else {
    rightwall = false ;


  } if (( frontSensor < front_threshold) && (frontSensor>0 )) {
    frontwall = true ;
  }
  else {
    frontwall = false ;
  }

}


 
  
 

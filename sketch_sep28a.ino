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
 //right_turn();
 //stop();
 //stopm();
 //delay(2000);
// if(frontwall==true)
// { analogWrite(enr,BaseSpeed);
// analogWrite(enl,BaseSpeed );
// stop();                        
// Serial.print("rukooooooooooooooooooooooooooooooooooooooooooooo");
// }
//rightfollow();
//leftrot=0;
//  rightrot=0;
//  walls();
 
//  
// move_cell();
 //left_turn();
 //delay(2000);
}

 //;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
// PID();

//  //u_turn();
//  left_turn();
//  stop();
//  delay(2000);((dis_left<4)&&(dis_right<4))
//  {
//   if (dis_left<dis_right)
//       forward();
//   else if (dis_left<dis_right)
//      soft_right();
//   else if(dis_left>dis_right)
//       soft_left();
//   }
//
// encoder();
// ultrasonic();
// //motors();
// delay(1000);

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

//void move_cell()
//{
//  Serial.print("hhhhhhhhhhhhhhhhhhhhhh");
//  ultrasonic();
//  a=abs(millis());
//  Serial.print("a    ");
//  Serial.println(a);
//  
//  int m;
//  if(rightSensor<60)
//  {
//    m=2;
//  }
//  if(leftSensor<60)
//  {
//    m=1;
//  }
//  if(!(leftSensor<60 or rightSensor<60))
//  {
//    m=3;
//  }
//  Serial.println(m);
//  switch(m)
//  {
//    case 1:
//    {
//      while((abs(millis())-a)<5000)
//      {
//        Serial.println(abs(millis()));
//        leftfollow();
//      }
//      break;
//    }
//    case 2:
//    {
//      while((abs(millis())-a)<5000)
//      {
//        Serial.println(abs(millis()));
//        rightfollow();
//      }
//      break;
//    }
//    case 3:
//    {
//       attachInterrupt(digitalPinToInterrupt(encl),leftpulse,CHANGE);
//       attachInterrupt(digitalPinToInterrupt(encr),rightpulse, CHANGE);
//      while((abs(millis())-a)<2000)
//      { 
//        Serial.println(abs(millis()));
//        analogWrite(enr,BaseSpeed);
//        analogWrite(enl,BaseSpeed);
//        forward();
//        delay(5);
//      }
//      detachInterrupt(digitalPinToInterrupt(encl));
//      detachInterrupt(digitalPinToInterrupt(encr));
//      break;
//    }
//  }
//    
//
//   stopm();
//   Serial.println("one finished ............................................................-");
// }
//void move_cell()
//{
//  leftrot=0;
//  rightrot=0;
//  stopm();
//  delay(1000);
//
//  while(leftrot<70 and rightrot<70)
//  {
//    ultrasonic();
//    Serial.println(leftrot);
//    Serial.println(rightrot);
//    Serial.println("\n");
//    analogWrite(enl,110);
//    analogWrite(enr,110);
//    
//  
//        forward();
//        delay(10); 
//  
//        if((leftrot)<(rightrot-5))
//        {
//          analogWrite(enl,80);
//          analogWrite(enr,0);
//          soft_right();
//         delay(5);
//      
//        }
//        if((rightrot)<(leftrot-5))
//        {
//          analogWrite(enl,0);
//          analogWrite(enr,80);
//         soft_left();
//         delay(5);
//        }
//        if(leftSensor<3.5)
//        {
//          analogWrite(enl,80);
//          analogWrite(enr,0);
//          soft_right();
//          delay(25);
//          
//        }
//        else if(rightSensor<3.5)
//        {
//          analogWrite(enl,0);
//          analogWrite(enr,80);
//          soft_left();
//          delay(25);
//          
//        }
//        
//        
//       
//     }
//    
//}
//void move_cell()
//{ leftrot=0;
//  rightrot=0;
//  ultrasonic();
//  walls();
//  if(rightwall)
//  {
//    while(leftrot<1.25 && rightrot<1.25)
//    { walls();
//      if(rightwall)
//      {Serial.println("yes");
//        errorP=rightSensor-34;
//        totalError=0.5*errorP;
//      }
//      else
//      {Serial.println("no");
//        totalError=0;
//      }
//      analogWrite(enr,BaseSpeed - totalError);
//      analogWrite(enl,BaseSpeed + totalError);
//      forward();
//      ultrasonic();
//    }
//  }
//  
//  else if(leftwall)
//  {
//    while(leftrot<1.25 && rightrot<1.25)
//    { 
//      Serial.print("encoder:");
//      Serial.println(leftrot);
//      Serial.println(rightrot);
//      walls();
//      ultrasonic();
//      if(leftwall)
//      {
//        errorP=34-leftSensor;
//        totalError=0.2*errorP;
//      }
//      else
//      {
//        totalError=0;
//      }
//      analogWrite(enr,BaseSpeed - totalError);
//      analogWrite(enl,BaseSpeed + totalError);
//      forward();
//    }
//    Serial.println("stoppppppppppppppppppppp............................................................................................");
//  }
//  stop();
//}
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
//void read_walls()                                                         // reading values from sensors and assigning it to cwall
//{ cwall.front = 0;
//  cwall.right = 0;
//  cwall.left = 0;
//  if (dis_left>50)
//    {
//      cwall.left = 1;
//    }
//    if (dis_middle>50)
//    {
//      cwall.front = 1;
//    }
//    if (dis_right>50)
//    {
//      cwall.right = 1;
//    }
//}
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
// void PID(void)       
// {   
//     if((leftwall)&&(rightwall))//has both walls
//     {  //ccw direction is positive
//         //63 is the offset between left and right sensor when mouse in the middle of cell
//         errorP = rightSensor-leftSensor+33;
//         errorD=errorP-oldErrorP;
//         errorI = (2.0 / 3) * errorI + errorP ;
//     }        
//     else if(leftwall)//only has left wall
//     {   errorP=2*(34-leftSensor);
        
//         errorD=errorP-oldErrorP;
        
//         errorI = (2.0 / 3) * errorI + errorP ;
//     }
//     else if(rightwall)//only has right wall
//     {
//         errorP = 2*(rightSensor - rightMiddleValue);
//         errorD=errorP-oldErrorP;
//         errorI = (2.0 / 3) * errorI + errorP ;
//     }
//     else if((!leftwall)&&(!rightwall))//no wall, use encoder or gyro
//     {   stopm();
//         delay(5000);
//         errorP = 0;//(leftEncoder – rightEncoder*1005/1000)*3;
//         errorD = 0;
//         errorI=0;
//     }
//     totalError = P * errorP + D * errorD +I*errorI;
//     oldErrorP = errorP;
//    // Serial.print("errorrr:");
//    // Serial.println(totalError/2);
//  analogWrite(enr,BaseSpeed + totalError/2);
//  analogWrite(enl,BaseSpeed - totalError/2);
//     forward();
// }


 
  
 

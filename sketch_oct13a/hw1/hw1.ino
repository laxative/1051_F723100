#include <Wire.h>
#include <NewPing.h>


#define MAX_DISTANCE 500

const int LeftMotorF = 5;
const int LeftMotorB = 6;
const int RightMotorB = 9;
const int RightMotorF = 10;
const int F_trigger = 12;
const int L_trigger = 7; 
const int R_trigger = 3; 
const int F_echo = 11;
const int L_echo = 4;
const int R_echo = 2;

NewPing sonarF(F_trigger,F_echo,MAX_DISTANCE);
NewPing sonarL(L_trigger,L_echo,MAX_DISTANCE);
NewPing sonarR(R_trigger,R_echo,MAX_DISTANCE);

int dis_L, dis_R, dis_F;

void setup() {
  pinMode(LeftMotorF,OUTPUT);
  pinMode(LeftMotorB,OUTPUT);
  pinMode(RightMotorF,OUTPUT);
  pinMode(RightMotorB,OUTPUT);
  Serial.begin(9600);
}
int a = 89, b = 80;
int pre_L = 0, pre_R = 0, cur_L, cur_R, times=0;
void loop() {
   
   //偵測距離
   detect();
   //依照距離判斷車子行走方向
   int condition = situation();
   switch(condition) {
     case 0:
	   //go forward
       forward(a,b);
       break;
     case 1:
	   //go left
       backward();
       delay(200);
       Motorstop();
       turnleft();
       delay(100);
       Motorstop();
       a=89;
       b=80;
       break;
     case 2:
	   //go right
       backward();
       delay(200);
       Motorstop();
       turnright();
       delay(100);
       Motorstop();
       a=89;
       b=80;
       break;
     default:
       break; 
   }
   //line 70-90 卡牆判定
   cur_R = dis_R;
   cur_L = dis_L;
   if(cur_R == pre_R && cur_L == pre_L) {
    times++;
    if(times>=10) {
      backward();
      delay(100);
      if(dis_L >= dis_R) {
        turnleft();
        delay(20);
      }else{
        turnright();
        delay(20);
      }
      times = 0;
    }
   }
   pre_R = cur_R;
   pre_L = cur_L;
   
   delay(50);
}

void forward(int a, int b) {
  //line 95-101 行走校正
  if(dis_L < 30 && dis_R < 30) {
    if(dis_L > dis_R) {
      a -= 8;
    } else if(dis_L < dis_R) {
      a += 8;
    }
  }
  analogWrite(LeftMotorF,a);
  analogWrite(LeftMotorB,0);
  analogWrite(RightMotorF,b);
  analogWrite(RightMotorB,0);
}

void backward() {
  analogWrite(LeftMotorF,0);
  analogWrite(LeftMotorB,89);
  analogWrite(RightMotorF,0);
  analogWrite(RightMotorB,80);
}

void turnright() {
  analogWrite(LeftMotorF,80);
  analogWrite(LeftMotorB,0);
  analogWrite(RightMotorF,40);
  analogWrite(RightMotorB,0);
}

void turnleft() {
  analogWrite(LeftMotorF,40);
  analogWrite(LeftMotorB,0);
  analogWrite(RightMotorF,80);
  analogWrite(RightMotorB,0);
}

void Motorstop() {
  analogWrite(LeftMotorF,0);
  analogWrite(LeftMotorB,0);
  analogWrite(RightMotorF,0);
  analogWrite(RightMotorB,0);
}

void detect() {
  dis_F = sonarF.ping_cm();
  dis_L = sonarL.ping_cm();
  dis_R = sonarR.ping_cm();
}

int situation() {
  if(dis_F == 0 || dis_F > 10) {
    return 0; //go ahead
  }else {
    if(dis_L > dis_R) return 1; //turn left
    else if(dis_L < dis_R) return 2; //turn right
    else return 1;
  }
}

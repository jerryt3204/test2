#include <MegaEncoderCounter.h>
#include <Timer.h>
MegaEncoderCounter megaEncoderCounter(4);
Timer timer;

//逆時針為正轉、順時針反轉，跟運動學公式一樣

#define motor1_in1 8 //正
#define motor1_in2 9 //反
#define motor2_in3 10 //正
#define motor2_in4 11 //反
#define en_a 53
#define en_b 52
#define test_led 41    //中斷指示燈
#define pprx  15.17
#define ppry  29.55

char incomingByte;
float degree;
double Kp=1,Ki=0.001,Kd=0;
double Kp2=0.2,Ki2=0,Kd2=0;
float error;
float sample, lastSample;
float P, I=0, D, pid;
float setPoint;
long  lastProcess;
float deltatime = 0;
float u_x,u_y;         //控制量
signed long count_x = 0,lastcount_x = 0;
signed long count_y = 0,lastcount_y = 0;
int flag = 0;

//for kinematics
float x,y;
float sita1,sita2,sita11,sita22;
float sita_1,sita_2;
float axis1 = 15,axis2 = 5; //兩軸長度

//for T_curve
float Time;
float T;
float Vmax = 20.0;
float Amax = 15.0;
float Ta1, V1 = 0.0, a1 = 0.0;
float Ta2, V2 = 0.0, a2 = 0.0;
float X1 = 0, X2 = 0;
int flg1 = 1,flg2 = 1;
float A;
float B;

float error1,error_d1,error_old1 = 0;
float error2,error_d2,error_old2 = 0;

float sliding_value1 = 1;
float sliding_value2 = 1;

float fsmcvector1[11] = {0};
float fsmcvector2[11] = {0};

float u1 = 0, u2 = 0;
signed long encoderValue1 = 0;
signed long encoderValue2 = 0;
float gs1[3] = {10,10,10};
float gs2[3] = {10,10,10};
float gu1[3] = {60,70,100};
float gu2[3] = {40,50,60};

float cnt1,cnt2;
float _gu1;
float _gu2;

void setup() {
  Serial.begin(9600);
  UserInterface();
  timer.every(30, show);
  pinMode(motor1_in1,OUTPUT);
  pinMode(motor1_in2,OUTPUT);
  pinMode(motor2_in3,OUTPUT);
  pinMode(motor2_in4,OUTPUT);
  pinMode(en_a,OUTPUT);
  pinMode(en_b,OUTPUT);
  digitalWrite(en_a,HIGH);
  digitalWrite(en_b,HIGH);
  attachInterrupt(digitalPinToInterrupt(2), limit, HIGH);
}

void loop() {
  flag = 0;
  if(Serial.available() > 0){
    incomingByte = Serial.read();
    switch (incomingByte){
      case 'd':
        Serial.println(" 請輸入角度: ");
        while (Serial.available()==0) {}
        degree = Serial.parseFloat();
        Serial.println(degree);
        deg1(degree);
        break;
      case 'o':
        cali();
        break;
      case 't':
        Serial.print(" 請輸入第一軸 Kp: ");
        while (Serial.available()==0) {}
        Kp = Serial.parseFloat();
        Serial.println(Kp);
        Serial.print(" 請輸入第一軸 Ki: ");
        while (Serial.available()==0) {}
        Ki = Serial.parseFloat();
        Serial.println(Ki);
        Serial.print(" 請輸入第一軸 Kd: ");
        while (Serial.available()==0) {}
        Kd = Serial.parseFloat();
        Serial.println(Kd);
        Serial.print(" 請輸入第二軸 Kp: ");
        while (Serial.available()==0) {}
        Kp2 = Serial.parseFloat();
        Serial.println(Kp2);
        Serial.print(" 請輸入第二軸 Ki: ");
        while (Serial.available()==0) {}
        Ki2 = Serial.parseFloat();
        Serial.println(Ki2);
        Serial.print(" 請輸入第二軸 Kd: ");
        while (Serial.available()==0) {}
        Kd2 = Serial.parseFloat();
        Serial.println(Kd2);
        break;
      case 'x':
        Serial.println(" 請輸入(x,y)座標: ");
        Serial.print(" x: ");
        while (Serial.available()==0) {}
        x = Serial.parseFloat();
        Serial.println(x);
        Serial.print(" y: ");
        while (Serial.available()==0) {}
        y = Serial.parseFloat();
        Serial.println(y);
        flag = 0;
        calculate(x,y);
        break;
      case 'v':
        Serial.println(" 請輸入(x,y)座標: ");
        Serial.print(" x: ");
        while (Serial.available()==0) {}
        x = Serial.parseFloat();
        Serial.println(x);
        Serial.print(" y: ");
        while (Serial.available()==0) {}
        y = Serial.parseFloat();
        Serial.println(y);
        flag = 1;
        calculate(x,y);
        break;
      case 'f':
        Serial.println(" 請輸入(x,y)座標: ");
        Serial.print(" x: ");
        while (Serial.available()==0) {}
        x = Serial.parseFloat();
        Serial.println(x);
        Serial.print(" y: ");
        while (Serial.available()==0) {}
        y = Serial.parseFloat();
        Serial.println(y);
        flag = 2;
        calculate(x,y);
        break;
    }
  }
}

void calculate(float x,float y){
  sita2 = (acos((sq(x)+sq(y)-sq(axis1)-sq(axis2))/(2*axis1*axis2)))*57.3;      //第一組解
  sita1 = (atan(y/x) - atan(axis2*sin(sita2)/(axis1+axis2*cos(sita2))))*57.3;
  sita22 = -(acos((sq(x)+sq(y)-sq(axis1)-sq(axis2))/(2*axis1*axis2)));         //第二組解
  sita11 = (atan(y/x) - atan(axis2*sin(sita22)/(axis1+axis2*cos(sita22))))*57.3;
  sita22 = sita22*57.3;
  //都已經轉成角度了
  Serial.print(" sita1 : ");
  Serial.println(sita1);
  Serial.print(" sita2 : ");
  Serial.println(sita2);
  Serial.print(" sita11 : ");
  Serial.println(sita11);
  Serial.print(" sita22 : ");
  Serial.println(sita22);
  if(flag == 1){
    if(sita1<90 && sita1>-90){
      T_curve();
    }
//    else if(sita11<90 &&sita22>-90){
//      T_curve();
//    }
    else{
      Serial.println("無法執行");
    }
  }
  if(flag == 0){
    if((sita1 < 85 )&&(sita1 > -85)){
    deg3(sita1,sita2);
    }
  }

  if(flag == 2){
    if(sita1<90 && sita1>-90){
      T_curve_fsmc();
    }
    else{
      Serial.println("無法執行");
    }
  }
}

//PID
void addNewSample(float _sample) {
  sample = _sample;
}
void setSetPoint(float _setPoint) {
  setPoint = _setPoint;
}
float process() {
  I=0;
  error = setPoint - sample;
  deltatime = 0.01;
  P = error * Kp;
  I = I + (error * Ki) * deltatime;
  D = (lastSample - sample) * Kd / (deltatime);
  lastSample = sample;
  pid = P + I + D;
  if (pid <= -40) {
    pid = -40;
  }
  else if (pid >= 60) {
    pid = 100;
  }
  return pid;
}

float process2() {
  I=0;
  error = setPoint - sample;
  deltatime = 0.01;
  P = error * Kp2;
  I = I + (error * Ki2) * deltatime;
  D = (lastSample - sample) * Kd2 / (deltatime);
  lastSample = sample;
  pid = P + I + D;
  if (pid <= -40) {
    pid = -40;
  }
  else if (pid >= 40) {
    pid = 40;
  }
  return pid;
}

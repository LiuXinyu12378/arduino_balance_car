 int i=10000,j=10000;
 int n=0;
#include <AFMotor.h>
#include "Wire.h"
#include "SPI.h"  
#include "Mirf.h"
#include "nRF24L01.h"
#include "MirfHardwareSpiDriver.h"
#include "I2Cdev.h"
#include "MPU6050.h"
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
AF_DCMotor motor2(3, MOTOR12_8KHZ);
AF_DCMotor motor1(4, MOTOR12_8KHZ);
#define Gry_offset -20     // 陀螺仪偏移量
#define Gyr_Gain 0.00763358    //对应的1G
#define pi 3.14159
/*********** PID控制器参数 *********/
float kp, ki, kd,kI; 
float k_p,k_d,k_i,k_I;
float angleA,omega;
float LOutput,ROutput;   
char buffer [4];
float LSpeed_Need=0.0,RSpeed_Need=0.0;
int data,adata;

unsigned long now;
unsigned long preTime = 0;
float SampleTime = 0.08;  //-------------------互补滤波+PID 采样时间0.08 s
unsigned long lastTime;
float Input, Output,SET_POINT, Setpoint=0.0,setspeed=10000,Output2,Output3,setpoint=0.0;
float errSum,errSum_,dErr,error,lastErr,f_angle,error_,error__,errSum__;
int timeChange; 


//-------------------------------------
void setup() {
  Wire.begin();
  accelgyro.initialize();  
  delay(100);
  delay(100);        
  delay(500);
  Serial.begin(115200);
  delay(200); 
  attachInterrupt(0, blink0, HIGH);
  attachInterrupt(1, blink1, HIGH);      
  interrupts();      //使能中断  
}

void loop() {
    llD();//---------------------------接收----------------------
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angleA= atan2(ay , az) * 180 / pi+77.0;   // 根据加速度分量得到的角度(degree)加75.0偏移量
  omega=  Gyr_Gain * (gx +  Gry_offset); // 当前角速度(degree/s)
  if (abs(angleA)<45)
  {    // 角度小于45度 运行程序
PIDD();//---------------------互补滤波+PID--------------------------
PWMB(); //----------------------PWM调速输出--------------------------
   }
    else 
    {      // 角度大于45度 停止PWM输出
motor1.setSpeed(0);
motor2.setSpeed(0);       
  }
 }
        //---------------------------接收----------------------
 void llD(){
if(Serial.available()>0){   //接收到信号，开始 
       //接收数据  
     Serial.readBytes(buffer,4);
     adata=*buffer;
     Serial.println(adata);
     if (adata==49){LSpeed_Need=0;RSpeed_Need=0;setpoint=3.0;}
else if (adata==50){LSpeed_Need=0;RSpeed_Need=0;setpoint=-3.0;}  
else if (adata==51){LSpeed_Need=0;RSpeed_Need=0;setpoint=0;}
else if (adata==52){LSpeed_Need=0;RSpeed_Need=0;setpoint=0;} 
else if (adata==53){LSpeed_Need=0;RSpeed_Need=0;setpoint=0;} 
else {LSpeed_Need=0;RSpeed_Need=0;setpoint=0.0;}//停止
    }
     delay(2);
  }
//---------------------互补滤波+PID-----------------
 void  PIDD(){ 
 kp = 5000*0.03;  //(这些值是小车调试后得出，请按自己小车调试后修改)
 ki = 3000*0.0002;
 kI = 1;
 kd = 4500*1; 
 k_p=160*0.01;
 k_i=1*0.001;
 k_I=1; 
//------------------互补滤波 ------------------------
    unsigned long now = millis();                             // 当前时间(ms)
    float dt = (now - preTime) / 1000.0;                    // 微分时间(ms)
    preTime = now;  
    float K = 0.8;                    
    float A = K / (K + dt);                    
   f_angle = A * (f_angle + omega * dt) + (1-A) * angleA;   // 互补滤波算法 
   //Serial.println(f_angle);
   timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {    
      Setpoint=setpoint;
      Input =f_angle;
      error = Setpoint- Input;
      error_ = setspeed-i;
      error__ = setspeed-j;
      errSum += error* timeChange;
      errSum_+=error_*timeChange;
      errSum__+=error__*timeChange;
      dErr = (error - lastErr)/ timeChange;
      if(abs(error)>10) kI=0;    //积分分离式PID，避免开机启动产生过冲
      else kI=1;
      if(abs(error_)>20) k_I=0;
      else k_I=1;
      Output = kp * error +kI* ki * errSum + kd * dErr;
      Output2 =k_p*error_+k_I*k_i*errSum_;
      Output3 =k_p*error__+k_I*k_i*errSum__;
      LOutput=Output-Output2+RSpeed_Need;//左电机
      ROutput=Output+Output3-RSpeed_Need;//右电机
      lastErr = error;
      lastTime = now;
   }
 }
 //----------------------PWM调速输出--------------------------
void PWMB(){
  if(LOutput>0.3)//左电机-------或者取0
{
    motor1.run(FORWARD);
}
  else if(LOutput<-0.3)//-------或者取0
{
 motor1.run(BACKWARD);
}
  else  //刹车-------取0后可以不用
{
motor1.run(RELEASE);
}
if(ROutput>0.3)//右电机--------或者取0
{
 motor2.run(FORWARD);

}
  else if(ROutput<-0.3)//-------或者取0
{
 motor2.run(BACKWARD);  
}
  else//刹车-------取0后可以不用
{
motor2.run(RELEASE);  
}
    motor1.setSpeed(min(255,abs(LOutput)+10));
    motor2.setSpeed(min(255,abs(ROutput)));
}
void blink1()     //中断函数
{
    if (digitalRead(3) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(A0) == LOW)      i++;//根据另外一相电平判定方向
    else      i--;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(A0) == LOW)      i--; //根据另外一相电平判定方向
    else     i++;
  }
}
void blink0()     //中断函数
{
    if (digitalRead(2) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(A1) == LOW)      j++;//根据另外一相电平判定方向
    else      j--;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(A1) == LOW)      j--; //根据另外一相电平判定方向
    else     j++;
  }
}

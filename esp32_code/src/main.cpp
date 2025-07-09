//串口打印遥控通道数据
#include <Arduino.h>
#include "SF_Servo.h"
#include "SF_IMU.h"
#include "HardwareSerial.h"
#include "bipedal_data.h"
#include "lowpass_filter.h"
#include "pid.h"
#include "SF_BLDC.h"
#include "math.h"
//tockn
#include "MPU6050_tockn.h"

#define ROLL_OFFSET 0

SF_Servo servos = SF_Servo(Wire); //实例化舵机
//tockn
MPU6050 mpu6050(Wire, 0.03, 0.97); 
 // UART2 用于电机通信
SF_BLDC motors = SF_BLDC(Serial2);
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

void getRCValue();
void getMPUValue();
void getMotorValue();
void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear, uint16_t servoRightFront, uint16_t servoRightRear);
void setRobotparam();
void robotRun();
void inverseKinematics();
void legControl();
float selfCaliCentroid(float central);


robotposeparam robotPose;
robotmotionparam robotMotion;
robotmode robotMode;
motorstatus motorStatus;
controlparam controlTarget;
coordinate coordTarget;
IKparam IKParam;
motorstarget motorsTarget;
float robotLastHeight;
int RCLastCH3Value;
int RCLastCH2Value;
SF_BLDC_DATA  BLDCData;

float wheel_X_off=2.2; //轮子X坐标偏置

uint32_t currTime;
uint32_t prevTime;

//debug
int16_t alpha1ToAngle,beta1ToAngle,alpha2ToAngle,beta2ToAngle;

float targetVoltage;


PIDIncrement PID_Roll{ .Kp = 0, .Ki = 0, .Kd = 0 };   
PIDIncrement PID_Gyrox{ .Kp = 0, .Ki = 0, .Kd = 0 };  
PIDIncrement PID_Hegiht{ .Kp = 0.15, .Ki = 0, .Kd = 0 };        
PIDIncrement PID_Y{ .Kp = 0.4, .Ki = 0, .Kd = 0 };        
PIDIncrement PID_XCoord{ .Kp = 0.34, .Ki = 0, .Kd = 0 };     
PIDIncrement PID_Stb{ .Kp = 0, .Ki = 0, .Kd = 0 };     
PIDIncrement PID_Streeing{ .Kp = 0.05, .Ki = 0, .Kd = 0 }; 
PIDIncrement PID_Forward{ .Kp = -0.8, .Ki = 0, .Kd = 0 };
// PIDIncrement PID_Forward{ .Kp = -0.5, .Ki = 0, .Kd = 0 };
LowPassFilter LPFPitch{0.03 };  // 速度低通滤波
LowPassFilter LPFRoll{0.05 };
float GyroXPModify;

PIDController PID_VEL{ 0, 0, 0, 1000,50 };

int16_t x,yL,yR;

const float wheel_base = 0.14;
const float wheel_radius = 0.065;
unsigned long last_send_time = 0;
const unsigned long send_interval = 50; // ms
// 全局变量，用于旧的里程计更新
unsigned long last_odom_time = 0;

struct OdomData {
  float x;
  float y;
  float yaw;
  float linear_vel;
  float angular_vel;
};

OdomData odomData = {0.0, 0.0, 0.0, 0.0, 0.0};

void updateOdometry(OdomData &odom, float dt);


void read(){
  static String received_chars;
  String command = "";

  while (Serial.available() > 0) {
    // command = Serial.parseInt();
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if(inChar=='\n'){
      command = received_chars;
      const char* d=command.c_str();
      sscanf(d,"%f",&wheel_X_off);
      // sscanf(d,"%f",&Gyro_off);
      received_chars = "";
    }
  }
}

uint8_t cnt;
uint32_t now_time;
uint32_t last_time=micros();

void setup() {
  Serial.begin(115200);
  setRobotparam();
  Wire.begin(1,2,400000UL);
  //tockn
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  servos.init();
  servos.setAngleRange(0,300);
  servos.setPluseRange(500,2500);
  motors.init();
  motors.setModes(4,4);
  

  delay(6000);
  currTime = micros();
  coordTarget.x = 0;
  coordTarget.yLeft = ROBOT_LOWESR_FOR_CAL;
  coordTarget.yRight = ROBOT_LOWESR_FOR_CAL;
  cnt = ROBOT_LOWESR_FOR_CAL;

  for(int i = 0; i < 4; i++) {
    RCValue[i] = 1000;  // 前四个通道初始化为1000
  }
  RCValue[4] = 200;  // 通道4固定为200
  RCValue[5] = 200;  // 通道5固定为200

}


void loop() {
  //read();
  getRCValue();
  getMPUValue();
  getMotorValue();

  legControl();
  inverseKinematics();
  robotRun();
  
  // 计算当前时间差并更新里程计
  unsigned long now = millis();
  float dt = (now - last_odom_time) / 1000.0;  // 单位为秒
  last_odom_time = now;
  
  // 更新里程计数据，并发布里程计和 IMU 消息
  updateOdometry(odomData, dt);
  if (now - last_send_time >= send_interval) {
    last_send_time = now;
    // 发布里程计数据 (odom)
    Serial.printf(
      "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
      now,
      odomData.x, odomData.y, odomData.yaw,
      odomData.linear_vel, odomData.angular_vel,
      robotPose.roll, robotPose.pitch, robotPose.yaw,
      robotPose.GyroX, robotPose.GyroY, robotPose.GyroZ
    );
  }
}

void updateOdometry(OdomData &odom, float dt) {
  float left_speed  = motorStatus.M0SpdDir * motorStatus.M0Speed * 2*PI*wheel_radius/60.0;
  float right_speed = motorStatus.M1SpdDir * motorStatus.M1Speed * 2*PI*wheel_radius/60.0;
  
  odom.linear_vel  = (left_speed + right_speed) / 2.0;
  odom.angular_vel = (right_speed - left_speed) / wheel_base;

  // 使用简单的积分更新位姿（未融合 IMU 数据，仅为示例）
  odom.x   += odom.linear_vel * cos(odom.yaw) * dt;
  odom.y   += odom.linear_vel * sin(odom.yaw) * dt;
  odom.yaw += odom.angular_vel * dt;
  while (odom.yaw > PI) {
    odom.yaw -= 2 * PI;
  }
  while (odom.yaw < -PI) {
    odom.yaw += 2 * PI;
  }
}

void setRobotparam(){
  //设置机体的运动限制
  robotMotion.heightest = ROBOT_HIGHEST;
  robotMotion.lowest = ROBOT_LOWEST_FOR_MOT;
  robotMotion.forwardLimit = 15;
  robotMotion.rollLimit = 20;

  robotMode.motorEnable = false;
  robotMode.servoEnable = false;
  robotMode.printFlag = false;
  robotMode.mode = ROBOTMODE_DIABLE;

  //电机速度控制方向
  motorStatus.M0Dir = -1;
  motorStatus.M1Dir = 1;
  //电机速度反馈方向
  motorStatus.M0SpdDir = -1;
  motorStatus.M1SpdDir = 1;

}


// 从ROS节点获取通道数据
void getRCValue() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    char *token = strtok(const_cast<char*>(data.c_str()), ",");
    int index = 0;
    while (token != NULL && index < 4) {
      RCValue[index] = atoi(token);  // 更新通道值
      RCValue[index] = _constrain(RCValue[index], RCCHANNEL_MIN, RCCHANNEL_MAX);  // 限制范围
      token = strtok(NULL, ",");
      index++;
    }
  }
  RCValue[4] = 200;  // 通道4固定为200
  RCValue[5] = 200;  // 通道5固定为200
}

void getMPUValue(){
  mpu6050.update();
  //tockn
  robotPose.pitch = -mpu6050.getAngleX();// 摆放原因导致调换
  robotPose.roll = mpu6050.getAngleY()+ROLL_OFFSET;// 摆放原因导致调换
  robotPose.yaw = mpu6050.getAngleZ();
  robotPose.GyroX = mpu6050.getGyroY(); 
  robotPose.GyroY = -mpu6050.getGyroX();
  robotPose.GyroZ = -mpu6050.getGyroZ();
  // Serial.printf("%f,%f,%f,%f,%f,%f\n",robotPose.pitch,robotPose.roll,robotPose.yaw,robotPose.GyroX,robotPose.GyroY,robotPose.GyroZ);
}

void getMotorValue(){
  BLDCData = motors.getBLDCData();
  motorStatus.M0Speed = BLDCData.M0_Vel;
  motorStatus.M1Speed = BLDCData.M1_Vel;
}

void setServoAngle(uint16_t servoLeftFront, uint16_t servoLeftRear, uint16_t servoRightFront, uint16_t servoRightRear){
  // servos.setPWM(LFSERVO_CH, 0, servoLeftFront);
  // servos.setPWM(LRSERVO_CH, 0, servoLeftRear);
  // servos.setPWM(RFSERVO_CH, 0, servoRightFront);
  // servos.setPWM(RRSERVO_CH, 0, servoRightRear);

  //TEST
  // Serial.printf("%d,%d,%d,%d\n",servoLeftFront,servoLeftRear,servoRightFront,servoRightRear);
  // Serial.printf("%d,%d,%d,%d\n",90+servoLeftFront,90+servoLeftRear,270-servoRightFront,270-servoRightRear);
  // Serial.println();
  servos.setAngle(3, 90+LFSERVO_OFFSET+servoLeftFront);
  servos.setAngle(4, 90+LRSERVO_OFFSET+servoLeftRear);
  servos.setAngle(5, 270+RFSERVO_OFFSET-servoRightFront);
  servos.setAngle(6, 270+RRSERVO_OFFSET-servoRightRear);
}

void legControl(){
  float e_H;
  float E_H;

  //根据RC 设定足轮的运动参数
  robotMotion.turn = map(RCValue[0], RCCHANNEL_MIN, RCCHANNEL_MAX, -1*robotMotion.turnLimit, robotMotion.turnLimit);//转弯，右左右 --- R1
  robotMotion.forward = map(RCValue[1], RCCHANNEL_MIN, RCCHANNEL_MAX, -1*robotMotion.forwardLimit, robotMotion.forwardLimit);//前进后退，右上下  ----  R0
  // Serial.printf("%f,%d\n",robotMotion.turn,RCValue[0]);
  if(robotMotion.forward >= 30) robotMotion.forward = 30;//正数是车身前倾，腿向后
  else if(robotMotion.forward <= -15) robotMotion.forward = -15;//负数是车身后倾，腿向前


  robotMotion.updown = ((int)map(RCValue[2], RCCHANNEL3_MIN, RCCHANNEL3_MAX, robotMotion.lowest, robotMotion.heightest));//变腿高，左上下 --- R2
  robotMotion.roll = map(RCValue[3], RCCHANNEL_MIN, RCCHANNEL_MAX, -1*robotMotion.rollLimit, robotMotion.rollLimit);//横滚，左左右 --- R3
  



  //根据RC 设定足轮的电机与舵机使能状态
  if(RCValue[4] == RCCHANNEL3_MIN){
    robotMode.motorEnable = true;
    robotMode.servoEnable = true;
    // Serial.println(111);
  }else if (RCValue[4] == RCCHANNEL3_MID){
    // robotMode.motorEnable = true;
    // robotMode.servoEnable = false;
    robotMode.motorEnable = false;
    robotMode.servoEnable = true;
    // Serial.println(222);
  }else if (RCValue[4] == RCCHANNEL3_MAX){
    robotMode.motorEnable = false;
    robotMode.servoEnable = false;
    // Serial.println(333);
  }

  //根据RC 设定足轮的控制模式
  if(RCValue[5] == RCCHANNEL3_MIN){
    robotMode.mode = ROBOTMODE_MOTION;
    robotMotion.lowest = ROBOT_LOWEST_FOR_MOT;
    robotMode.printFlag = false;
  }else if (RCValue[5] == RCCHANNEL3_MID){
    robotMode.printFlag = true;
  }else if (RCValue[5] == RCCHANNEL3_MAX){
    robotMode.mode = ROBOTMODE_CALIBRATION;
    robotMotion.forward = 0;
    robotMotion.updown = ROBOT_LOWESR_FOR_CAL;
    robotMotion.lowest = ROBOT_LOWESR_FOR_CAL;
    robotMode.printFlag = false;
  }

  if(abs(RCValue[2] - RCLastCH3Value) >= 5){
    GyroXPModify = 0.05f;
    RCLastCH3Value = RCValue[2];
  }else{
    GyroXPModify = 0.05f;
  }



  // controlTarget.forward = PID_Forward.Kp*(robotMotion.forward - (motorStatus.M0SpdDir*motorStatus.M0Speed + motorStatus.M1SpdDir*motorStatus.M1Speed)/2);
  controlTarget.forward = PID_Forward.Kp*(robotMotion.forward - 0);
  controlTarget.forward = _constrain(controlTarget.forward, -20, 20);
  // coordTarget.x = coordTarget.x + PID_XCoord.Kp*(controlTarget.forward - coordTarget.x);


  if(robotMode.mode == ROBOTMODE_MOTION && robotMode.servoEnable == true){

    // e_H = PID_Roll.Kp * LPFRoll((robotMotion.roll - 3) - robotPose.roll);
    e_H = PID_Roll.Kp * LPFRoll(robotMotion.roll - robotPose.roll);
    E_H = PID_Gyrox.Kp * (e_H - robotPose.GyroX);
  }else{
    e_H = 0;
    E_H = 0;
  }
  // Serial.printf("%f,%f\n",e_H,E_H);

  controlTarget.legLeft = controlTarget.legLeft + PID_Hegiht.Kp * (robotMotion.updown - controlTarget.legLeft);
  controlTarget.legRight = controlTarget.legRight + PID_Hegiht.Kp * (robotMotion.updown - controlTarget.legRight);


  controlTarget.legRollLeft = _constrain(controlTarget.legRollLeft + E_H, -robotMotion.lowest, robotMotion.lowest);
  controlTarget.legRollRight = _constrain(controlTarget.legRollRight - E_H, -robotMotion.lowest, robotMotion.lowest);



  // coordTarget.yLeft = _constrain(coordTarget.yLeft + PID_Y.Kp * ((robotMotion.updown+controlTarget.legRollLeft)-coordTarget.yLeft),robotMotion.lowest, robotMotion.heightest);
  // coordTarget.yRight = _constrain(coordTarget.yRight + PID_Y.Kp * ((robotMotion.updown+controlTarget.legRollRight)-coordTarget.yRight),robotMotion.lowest, robotMotion.heightest);

  coordTarget.yLeft = _constrain(controlTarget.legRollLeft + controlTarget.legLeft, robotMotion.lowest, robotMotion.heightest);
  coordTarget.yRight = _constrain(controlTarget.legRollRight + controlTarget.legRight, robotMotion.lowest, robotMotion.heightest);

  coordTarget.x = coordTarget.x + PID_XCoord.Kp * (controlTarget.forward - coordTarget.x)+ wheel_X_off;

  robotPose.height = (coordTarget.yLeft + coordTarget.yRight)/2;

  //debug
  // coordTarget.x = 0;
  // coordTarget.yLeft = 100;
  // coordTarget.yRight = 100;

}


void inverseKinematics(){
  // coordTarget.xLeft = coordTarget.x;
  // coordTarget.xRight = coordTarget.xLeft;

  IKParam.XLeft = coordTarget.x;
  IKParam.YLeft = coordTarget.yLeft;
  IKParam.XRight = coordTarget.x;
  IKParam.YRight = coordTarget.yRight;

  float a1 = 2 * IKParam.XLeft * L1;
  float b1 = 2 * IKParam.YLeft * L1;
  float c1 = IKParam.XLeft * IKParam.XLeft + IKParam.YLeft * IKParam.YLeft + L1 * L1 - L2 * L2;
  float d1 = 2 * L4 * (IKParam.XLeft - L5);
  float e1 = 2 * L4 * IKParam.YLeft;
  float f1 = ((IKParam.XLeft - L5) * (IKParam.XLeft - L5) + L4 * L4 + IKParam.YLeft * IKParam.YLeft - L3 * L3);

  IKParam.alpha1 = 2 * atan((b1 + sqrt((a1 * a1) + (b1 * b1) - (c1 * c1))) / (a1 + c1));
  IKParam.beta1 = 2 * atan((e1 - sqrt((d1 * d1) + e1 * e1 - (f1 * f1))) / (d1 + f1));

  //限制解算角度的范围
  if (IKParam.alpha1 < 0)
    IKParam.alpha1 = IKParam.alpha1 + 2 * PI;

  if (IKParam.beta1 < 0)
    IKParam.beta1 = 0;
  
  alpha1ToAngle = (int)((IKParam.alpha1 / 6.28) * 360);//弧度转角度
  beta1ToAngle = (int)((IKParam.beta1 / 6.28) * 360);


  motorsTarget.servoLeftRear = (int)map(alpha1ToAngle, 0, 180, 103, 327);  // 1号舵机 500~1500us(500~2500)
  motorsTarget.servoLeftFront = (int)map(beta1ToAngle, 0, 180, 103, 327);  // 2号舵机
  
  float a2 = 2 * IKParam.XRight * L1;
  float b2 = 2 * IKParam.YRight * L1;
  float c2 = IKParam.XRight * IKParam.XRight + IKParam.YRight * IKParam.YRight + L1 * L1 - L2 * L2;
  float d2 = 2 * L4 * (IKParam.XRight - L5);
  float e2 = 2 * L4 * IKParam.YRight;
  float f2 = ((IKParam.XRight - L5) * (IKParam.XRight - L5) + L4 * L4 + IKParam.YRight * IKParam.YRight - L3 * L3);

  IKParam.alpha2 = 2 * atan((b2 + sqrt((a2 * a2) + (b2 * b2) - (c2 * c2))) / (a2 + c2));
  IKParam.beta2 = 2 * atan((e2 - sqrt((d2 * d2) + e2 * e2 - (f2 * f2))) / (d2 + f2));



  if (IKParam.alpha2 < 0)
    IKParam.alpha2 = IKParam.alpha2 + 2 * PI;

  if (IKParam.beta2 < 0)
    IKParam.beta2 = 0;

  alpha2ToAngle = (int)((IKParam.alpha2 / 6.28) * 360);//todo
  beta2ToAngle = (int)((IKParam.beta2 / 6.28) * 360);

  motorsTarget.servoRightFront = (int)map(alpha2ToAngle, 0, 180, 103, 327);  // 1号舵机 500~1500us(500~2500)
  motorsTarget.servoRightRear = (int)map(beta2ToAngle, 0, 180, 103, 327);  // 2号舵机

  // motorsTarget.servoRightRear = (int)map(alpha2ToAngle, 0, 180, 103, 327);  // 1号舵机 500~1500us(500~2500)
  // motorsTarget.servoRightFront = (int)map(beta2ToAngle, 0, 180, 103, 327);  // 2号舵机




  if(robotMode.servoEnable){
    //debug
    setServoAngle(beta1ToAngle,alpha1ToAngle,beta2ToAngle,alpha2ToAngle);
    // setServoAngle(motorsTarget.servoLeftFront, motorsTarget.servoLeftRear, motorsTarget.servoRightFront, motorsTarget.servoRightRear);
    // Serial.printf("%d,%d,%d,%d\n",motorsTarget.servoLeftFront, motorsTarget.servoLeftRear, motorsTarget.servoRightFront, motorsTarget.servoRightRear);
  }

}




void robotRun(){
  float velLeft,velRight;
  float wheelControl;
  
  if(robotPose.height != robotLastHeight){
    PID_VEL.P = (0.00002 * robotPose.height *  robotPose.height - 0.007 *  robotPose.height + 0.669) * 1.8;
    PID_VEL.I = 0.00153 * 0.6;  //1
    PID_VEL.D = (0.0000002 * robotPose.height * robotPose.height - 0.00001 * robotPose.height - 0.01) * 0.1;

    PID_Stb.Kp = (0.0003 * robotPose.height * robotPose.height - 0.0488 * robotPose.height + 3.5798) * 0.8;     //0.7
    PID_Stb.Kd = (-0.000002 * robotPose.height * robotPose.height + 0.0005 * robotPose.height - 0.0043) * 1.6;  //1.6

    PID_Roll.Kp = (0.001 * robotPose.height * robotPose.height - 0.2281 * robotPose.height + 17.495) * 1.3;

    PID_Gyrox.Kp = (0.0000009 * robotPose.height * robotPose.height - 0.0005 * robotPose.height + 0.091) * GyroXPModify;


    robotMotion.turnLimit = (0.000009 * robotPose.height * robotPose.height - 0.005 * robotPose.height + 120.5104) * 1;

    robotLastHeight = robotPose.height;
  }
  
  // 初始代码
  robotPose.speedAvg = (motorStatus.M0SpdDir*motorStatus.M0Speed + motorStatus.M1SpdDir*motorStatus.M1Speed)/2;

  // 调试代码
  // robotPose.speedAvg = (motorStatus.M0Dir*motorStatus.M0Speed*+ motorStatus.M1Dir*motorStatus.M1Speed)/2;


  if(RCLastCH2Value == RCValue[1]){
    controlTarget.centerAngleOffset = selfCaliCentroid(controlTarget.centerAngleOffset);
  }
  RCLastCH2Value = RCValue[1];

  // 调试代码
  // controlTarget.velocity = PID_VEL(robotPose.speedAvg);//速度环

  // 初始代码
  controlTarget.velocity = PID_VEL(robotMotion.forward*0.5f - robotPose.speedAvg);//速度环   forward是遥控器前进后退参数
  controlTarget.differVel = PID_Streeing.Kp*(robotMotion.turn-robotPose.GyroZ);//转向        turn是遥控器左右控制参数
  targetVoltage = PID_Stb.Kp*(controlTarget.velocity + controlTarget.centerAngleOffset - robotPose.pitch) - PID_Stb.Kd * robotPose.GyroY;//直立环 输出控制的电机的目标速度

  // 调试代码
  // motorsTarget.motorLeft = motorStatus.M0Dir * (targetVoltage);
  // motorsTarget.motorRight = -motorStatus.M1Dir * (targetVoltage);

  // 初始代码
  motorsTarget.motorLeft = motorStatus.M0Dir * (targetVoltage + controlTarget.differVel);
  motorsTarget.motorRight = motorStatus.M1Dir * (targetVoltage - controlTarget.differVel);

  if (robotMode.motorEnable == 1 && robotPose.pitch <= 40 && robotPose.pitch >= -35) {
    motorsTarget.motorLeft = _constrain(motorsTarget.motorLeft, -5.7, 5.7);
    motorsTarget.motorRight = _constrain(motorsTarget.motorRight, -5.7, 5.7);
    motors.setTargets(motorsTarget.motorLeft,motorsTarget.motorRight);
    // motors.setTargets(2,2);
  } else {
    motors.setTargets(0,0);
  }
}

float selfCaliGain = 0.8;
float selfcaliOffset = 0;
#define SELF_CALI_RANGE 7
#define CENTER_ANGLE_OFFSET 3
float selfCaliCentroid(float central){
  static int i = 0;
  if(i == 40){
    if(fabs(robotPose.speedAvg) > 1){
      selfcaliOffset = selfCaliGain * -1 * robotPose.speedAvg;
      selfcaliOffset = _constrain(selfcaliOffset, -0.5, 0.5);
      central += selfcaliOffset;
    }
    i = 0;
  }else{
    ++i;   
  }
  central = _constrain(central, CENTER_ANGLE_OFFSET-SELF_CALI_RANGE, CENTER_ANGLE_OFFSET+SELF_CALI_RANGE);

  // central = _constrain(central, -10, 10);

  return central;
}





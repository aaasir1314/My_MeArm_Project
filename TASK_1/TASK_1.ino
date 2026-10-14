#include <Servo.h>
#include <stdlib.h>
#include <math.h>

#define M_PI 3.1415926
#define RAD2DEG (180.0f / (float)M_PI)

#define PIN_BASE       9        //底座舵机
#define PIN_SHOULDER   8       //大臂舵机
#define PIN_ELBOW      7      //小臂舵机
#define PIN_CLAW       6        //爪子舵机

//占位
#define CLAW_OPEN_DEG  30       //爪子张开
#define CLAW_CLOSE_DEG 110      //爪子闭合

#define BASE_SPEED_DPS   90.0f
#define GRAB_SPEED_DPS   30.0f
#define SPEED_STEP_UP    1.25f
#define SPEED_STEP_DOWN  0.80f
#define SPEED_SCALE_MIN  0.25f
#define SPEED_SCALE_MAX  4.00f
#define TICK_MS          20

#define PIN_JOY_SHOULDER A0
#define PIN_JOY_BASE     A1
#define PIN_JOY_ELBOW    A2
#define PIN_JOY_CLAW     A3
#define JOY_CENTER    512
#define JOY_DEADZONE  30
#define JOY_RANGE     512
#define JOY_MAX_DPS  120.0f

#define CMD_LEN  32

#define L1_MM           93.0f //大臂长
#define L2_MM           123.0f//小臂长
#define L3_MM           65.0f//前伸量

//占位
#define TH1_OFFSET_DEG  90.0f
#define TH2_OFFSET_DEG   90.0f
#define TH3_OFFSET_DEG  90.0f
#define TH1_GAIN        (+1.0f)
#define TH2_GAIN        (+1.0f)
#define TH3_GAIN        (+1.0f)

#define APPROACH_H_MM    40.0f
#define WAIT_GRIP_MS      500 

#define FRAME_HEAD      '$'
#define FRAME_KEY_MARK  'K'
#define FRAME_TAIL      '#'

#define HOME_SERVO1  90
#define HOME_SERVO2  90
#define HOME_SERVO3  90
#define HOME_CLAW_DEG  CLAW_OPEN_DEG

#define REC_BUFFER_BYTES   1020
#define REC_FRAME_BYTES    6
#define REC_FRAME_CAPACITY (REC_BUFFER_BYTES / REC_FRAME_BYTES)
#define REC_SAMPLE_MS     100

Servo servos[3];
const uint8_t servoPins[3] = { PIN_BASE, PIN_SHOULDER, PIN_ELBOW };
Servo claw;

float targetAngle[3]  = { 90, 90, 90 };
float currentAngle[3] = { 90, 90, 90 };
float clawTarget  = CLAW_OPEN_DEG;
float clawCurrent = CLAW_OPEN_DEG;
float speedScale = 1.0f;
unsigned long lastTick = 0;
unsigned long lastJoy  = 0;

char cmd[CMD_LEN];
uint8_t cmdLen = 0;//计数器

bool grabActive = false;

int g_placeX, g_placeY, g_placeZ;

bool recStopRequest = false;
bool recording = false; 

//占位
struct Target { int grabX, grabY, grabZ, placeX, placeY, placeZ, clawDeg; };//抓取点x, 抓取点y, 抓取点z,  放置点x, 放置点y, 放置点z,  夹持角
Target g_targetA = { 90, 0, 50,   90, -50, 75,  115 };
Target g_targetB = {  90, 0, 0,   50,  60, 65,  110 };
Target g_targetC = {  90, 0, 50,    40, -30, 70,  105 };

enum FrameParseState { FP_WAIT_HEAD = 0, FP_WAIT_K, FP_WAIT_DIGIT, FP_WAIT_TAIL };

FrameParseState fpState = FP_WAIT_HEAD;
uint8_t fpPendingKey = 0;

struct MotionFrame {
  uint16_t dt;//距离上一帧相隔的时间
  int8_t   d1, d2, d3, dclaw;//距离初始位置的偏移值
};

MotionFrame recBuf[REC_FRAME_CAPACITY];
uint16_t recCount = 0;//计数器
unsigned long recHoldMs = 0;

uint8_t t3Counter = 0; //夹取物体循环计数


void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++)
  {
    servos[i].attach(servoPins[i]);
    servos[i].write(90);
  }
  claw.attach(PIN_CLAW);
  clawTarget  = CLAW_OPEN_DEG;
  clawCurrent = CLAW_OPEN_DEG;
  claw.write(CLAW_OPEN_DEG);
  Serial.println(F("初始化完成"));
}

void loop() {
  unsigned long now = millis();
  readSerial();//读字符
  updateJoystick(now);//处理摇杆
  tick(now);//平滑运动
}

void readSerial() {
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == '\n')
    {
        if (cmdLen > 0)
        {
          cmd[cmdLen] = '\0';
          handleLine(cmd);
        }
        cmdLen = 0;
    }
    else if (c != '\r' && cmdLen < CMD_LEN - 1)
    {
      cmd[cmdLen++] = c;
    }
    fpFeed(c);
  }
}

void handleLine(char* line) {
  if (line[0] == FRAME_HEAD) {
    return;
  }

  if (line[1] == '\0'){
    switch (line[0])
     {
      case 'O':
        clawMove(CLAW_OPEN_DEG);
        Serial.println(F("成功打开"));
        return;
      case 'S':
        clawMove(CLAW_CLOSE_DEG);
        Serial.println(F("成功关闭"));
        return;
      case 'H':
        speedScale *= SPEED_STEP_UP;
        if (speedScale > SPEED_SCALE_MAX)
        {
          speedScale = SPEED_SCALE_MAX;
        }
        printSpeed();
        return;
      case 'L':
        speedScale *= SPEED_STEP_DOWN;
        if (speedScale < SPEED_SCALE_MIN)
        {
          speedScale = SPEED_SCALE_MIN;
        }
        printSpeed();
        return;

//目前没有拓展板，暂时用1234替代按钮输入。
      case '1': case '2': case '3': case '4':
        execKey((uint8_t)(line[0] - '0'));
        return;

    }
  }

  if (line[0] == 'A' || line[0] == 'B' || line[0] == 'C') {
    Target* t; 
    if(line[0]=='A'){
      t=&g_targetA;
    }
    else if(line[0]=='B'){
      t=&g_targetB;
    }
    else{
      t=&g_targetC;
    }
    
    Serial.print(F("成功，抓取"));
    Serial.println(line[0]);
    grabFull(t);
    return;
  }

   int v[3] = { -1, -1, -1 };//记录数值
  char *seg = strtok(line, ",");
  while (seg != NULL) {
    if (seg[0] == 'x')
    {
       v[0] = atoi(seg + 1);
    }
    if (seg[0] == 'y')
    {
       v[1] = atoi(seg + 1);
    }
    if (seg[0] == 'z')
    {
       v[2] = atoi(seg + 1);
    }
    seg = strtok(NULL, ",");
  }

  bool any = false;//判断数值是否有效
  for (int i = 0; i < 3; i++)
   {
    if (v[i] >= 0)
      {
       targetAngle[i] = v[i];
        any = true;
      }
   }
  if (any)
  {
     Serial.println(F("成功移动"));
  }
}

void printSpeed() {
  Serial.print(F("成功，当前速度比例为"));
  Serial.print(speedScale, 2);
  Serial.println(F("x"));
}

void updateJoystick(unsigned long now) {

  if (now - lastJoy < 20)
  {
     return;
  }

  lastJoy = now;
  float dt = 0.02f;

  targetAngle[0] += joySpeed(analogRead(PIN_JOY_BASE))     * speedScale * dt;//底座
  targetAngle[1] += joySpeed(analogRead(PIN_JOY_SHOULDER)) * speedScale * dt;//大臂
  targetAngle[2] += joySpeed(analogRead(PIN_JOY_ELBOW))    * speedScale * dt;//小臂
  clawTarget      -= joySpeed(analogRead(PIN_JOY_CLAW))    * speedScale * dt;//爪子

  for (int i = 0; i < 3; i++) {//限制范围
    if (targetAngle[i] <   0)
    {
      targetAngle[i] =   0;
    }
    if (targetAngle[i] > 180)
    {
      targetAngle[i] = 180;
    }
  }

  int clawLower = (CLAW_OPEN_DEG < CLAW_CLOSE_DEG) ? CLAW_OPEN_DEG : CLAW_CLOSE_DEG;
  int clawUpper = (CLAW_OPEN_DEG < CLAW_CLOSE_DEG) ? CLAW_CLOSE_DEG : CLAW_OPEN_DEG;
  if (clawTarget < clawLower)
  {
    clawTarget = clawLower;
  }
  if (clawTarget > clawUpper)
  {
    clawTarget = clawUpper;
  }
}

float joySpeed(int raw) {
  int centered = raw - JOY_CENTER;
  if (centered > -JOY_DEADZONE && centered < JOY_DEADZONE)
  {
    return 0.0f;
  }
  float norm = (float)centered / JOY_RANGE;
  if (norm >  1.0f)
  {
    norm =  1.0f;
  }
  if (norm < -1.0f)
  {
    norm = -1.0f;
  }

  return norm * JOY_MAX_DPS;
}

void clawMove(int deg) {
  clawTarget  = deg;
  clawCurrent = deg;
  claw.write(deg);
}

void tick(unsigned long now) {

  if (now - lastTick < TICK_MS)
  {
  return;
  }

  float dt = (now - lastTick) / 1000.0f;
  lastTick = now;
  float scale = grabActive ? (GRAB_SPEED_DPS / BASE_SPEED_DPS) : speedScale;
  for (int i = 0; i < 3; i++) {
    float diff = targetAngle[i] - currentAngle[i];
    if (diff == 0.0f)
    {
      continue;
    }
    float step = BASE_SPEED_DPS * scale * dt;
    if (step >= abs(diff))
    {
      currentAngle[i] = targetAngle[i];
    }
    else
    {
      currentAngle[i] += (diff > 0) ? step : -step;
    }
    servos[i].write((int)(currentAngle[i] + 0.5f));
  }

  float cdiff = clawTarget - clawCurrent;
  if (cdiff != 0.0f)
  {
    float cstep = BASE_SPEED_DPS * scale * dt;
    if (cstep >= abs(cdiff))
    {
      clawCurrent = clawTarget;
    }
    else
    {
      clawCurrent += (cdiff > 0) ? cstep : -cstep;
    }
    claw.write((int)(clawCurrent + 0.5f));
  }
}

bool grabArrived() {
  for (int i = 0; i < 3; i++) {
    if (abs(targetAngle[i] - currentAngle[i]) > 0.50f)
    {
      return false;
      }
  }
  return true;
}

void waitArrived() {
  while (!grabArrived()) {
    tick(millis());
  }
}

void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    tick(millis());
  }
}

void moveTo(int x, int y, int z) {
  int s1, s2, s3;
  ikToServo(x, y, z, &s1, &s2, &s3);
  targetAngle[0] = s1;
  targetAngle[1] = s2;
  targetAngle[2] = s3;
}

void grabRun(int x, int y, int z, int clawDeg) {
  
  grabActive = true;

  int aboveZ = z + (int)APPROACH_H_MM;
  int safeZ  = z + (int)APPROACH_H_MM; 

  moveTo(x, y, aboveZ);
  waitArrived();  

  clawMove(CLAW_OPEN_DEG);
  moveTo(x, y, z);
  waitArrived();

  clawMove(clawDeg);
  waitMs(WAIT_GRIP_MS);

  moveTo(x, y, safeZ);
  waitArrived();

  moveTo(g_placeX, g_placeY, g_placeZ + (int)APPROACH_H_MM);
  waitArrived();

  moveTo(g_placeX, g_placeY, g_placeZ);
  waitArrived();

  clawMove(CLAW_OPEN_DEG);
  waitMs(WAIT_GRIP_MS);

  //回中占位
  targetAngle[0] = 90; targetAngle[1] = 90; targetAngle[2] = 90;
  clawMove(CLAW_OPEN_DEG);
  waitArrived();

  grabActive = false;
  Serial.println(F("完成"));
}

void grabFull(Target* t) {
  g_placeX = t->placeX; g_placeY = t->placeY; g_placeZ = t->placeZ;
  grabRun(t->grabX, t->grabY, t->grabZ, t->clawDeg);
}

float safeAcos(float c) {
  if (c >  1.0f)
  {
    return 0.0f;
    }
  if (c < -1.0f)
  {
    return (float)M_PI;
    }
  return acosf(c);
}

bool ikSolve(int xMm, int yMm, int zMm,float* t1Out, float* t2Out, float* t3Out) {
  float x = (float)xMm, y = (float)yMm, z = (float)zMm;

  float t1 = atan2f(y, x) * RAD2DEG;

  float rho = sqrtf(x * x + y * y);
  float r   = rho - L3_MM;  
  float R2 = r * r + z * z;
  float R  = sqrtf(R2);

  float psi = atan2f(z, r); 

  float beta  = safeAcos((L1_MM * L1_MM + L2_MM * L2_MM - R2)/ (2.0f * L1_MM * L2_MM));
  float gamma = safeAcos((L1_MM * L1_MM + R2 - L2_MM * L2_MM)/ (2.0f * L1_MM * R));

  float t2Rad = psi + gamma;
  float t3Rad = t2Rad - ((float)M_PI - beta);

  *t1Out = t1;
  *t2Out = t2Rad * RAD2DEG;
  *t3Out = t3Rad * RAD2DEG;
  return true;
}

void jointToServo(float t1, float t2, float t3,int* s1Out, int* s2Out, int* s3Out) {
  *s1Out = (int)(TH1_OFFSET_DEG + TH1_GAIN * t1 + 0.5f);
  *s2Out = (int)(TH2_OFFSET_DEG + TH2_GAIN * t2 + 0.5f);
  *s3Out = (int)(TH3_OFFSET_DEG + TH3_GAIN * t3 + 0.5f);
}

void ikToServo(int x, int y, int z, int* s1, int* s2, int* s3) {
  float t1, t2, t3;
  ikSolve(x, y, z, &t1, &t2, &t3);
  jointToServo(t1, t2, t3, s1, s2, s3);
}

void fpFeed(char c) {
  switch (fpState) {
    case FP_WAIT_HEAD:
      if (c == FRAME_HEAD) {
        fpState = FP_WAIT_K;
      break;
      }
    case FP_WAIT_K:
      if (c == FRAME_HEAD) {
        break;
        }
      if (c == FRAME_KEY_MARK) {
        fpState = FP_WAIT_DIGIT;
      }
      else {
        fpState = FP_WAIT_HEAD;
      break;
      }
    case FP_WAIT_DIGIT:
      if (c >= '1' && c <= '4') {
        fpPendingKey = (uint8_t)(c - '0');
        fpState = FP_WAIT_TAIL;
      } else {
        fpState = (c == FRAME_HEAD) ? FP_WAIT_K : FP_WAIT_HEAD;
      }
      break;
    case FP_WAIT_TAIL:
      if (c == FRAME_TAIL) {
        if (recording && fpPendingKey == 2) {
          recStopRequest = true;
        } else {
          execKey(fpPendingKey);
        }
        fpState = FP_WAIT_HEAD;
        cmdLen = 0;
      } else {
        fpState = (c == FRAME_HEAD) ? FP_WAIT_K : FP_WAIT_HEAD;
      }
      break;
  }
}

int8_t toOffset(int angle, int home) {
  int d = angle - home;
  if (d >  127) d =  127;
  if (d < -128) d = -128;
  return (int8_t)d;
}

void recRun() {
  recCount = 0;
  recStopRequest = false;
  recording = true;
  Serial.println(F("录制开始"));

  unsigned long lastSample = millis();
  unsigned long lastRec    = millis();
  while (!recStopRequest && recCount < REC_FRAME_CAPACITY) {
    unsigned long now = millis();

    updateJoystick(now);
    tick(now);
    readSerial();

    if (now - lastSample >= REC_SAMPLE_MS) {
      lastSample = now;
      int a1 = (int)(targetAngle[0] + 0.5f);
      int a2 = (int)(targetAngle[1] + 0.5f);
      int a3 = (int)(targetAngle[2] + 0.5f);
      int ac = claw.read();

      MotionFrame* f = &recBuf[recCount];
      unsigned long gap = (recCount == 0) ? 0 : (now - lastRec);
      if (gap > 65535) gap = 65535;
      f->dt = (uint16_t)gap;
      f->d1 = toOffset(a1, HOME_SERVO1);
      f->d2 = toOffset(a2, HOME_SERVO2);
      f->d3 = toOffset(a3, HOME_SERVO3);
      f->dclaw = toOffset(ac, HOME_CLAW_DEG);
      recCount++;
      lastRec = now;
    }
  }
    if (recCount == 0) {
    MotionFrame* f = &recBuf[0];
    f->dt = 0;
    f->d1 = toOffset((int)(targetAngle[0] + 0.5f), HOME_SERVO1);
    f->d2 = toOffset((int)(targetAngle[1] + 0.5f), HOME_SERVO2);
    f->d3 = toOffset((int)(targetAngle[2] + 0.5f), HOME_SERVO3);
    f->dclaw = toOffset(claw.read(), HOME_CLAW_DEG);
    recCount = 1;
  }
  recHoldMs = millis() - lastRec;
  recording = false;

  Serial.print(F("录制结束，录制帧数为："));
  Serial.println(recCount);
}

void applyFrame(const MotionFrame& f) {
  targetAngle[0] = HOME_SERVO1 + f.d1;
  targetAngle[1] = HOME_SERVO2 + f.d2;
  targetAngle[2] = HOME_SERVO3 + f.d3;
  clawMove(HOME_CLAW_DEG + f.dclaw);
}

void playRun() {
  Serial.print(F("开始播放，总帧数为："));
  Serial.println(recCount);

  if (recCount == 0) { 
    Serial.println(F("播放完成")); 
    return; 
    }

  applyFrame(recBuf[0]);

  for (uint16_t i = 1; i < recCount; i++) {
    waitMs(recBuf[i].dt);        
    applyFrame(recBuf[i]);
  }

  waitArrived();
  if (recHoldMs > 0) waitMs(recHoldMs);

  Serial.println(F("播放完成"));
}

void goHome() {
  targetAngle[0] = HOME_SERVO1;
  targetAngle[1] = HOME_SERVO2;
  targetAngle[2] = HOME_SERVO3;
  clawMove(HOME_CLAW_DEG);
  waitArrived();
  Serial.println(F("成功回中"));
}

void execKey(uint8_t k) {
  switch (k) {

    case 1: {
      uint8_t idx = t3Counter % 3;
      t3Counter++;
      Target* t = (idx == 0) ? &g_targetA
                : (idx == 1) ? &g_targetB : &g_targetC;
      Serial.print(F("正在移动"));
      Serial.println((idx == 0) ? 'A' : (idx == 1) ? 'B' : 'C');
      grabFull(t);
      return;
    }

    case 2: {
      if (recording) { 
        recStopRequest = true;
        return;
      }
      recRun();
      return;
    }

    case 3: {
      playRun();
      return;
    }

    case 4:
      goHome();
      return;
  }
}

#include <Servo.h>
#include <stdlib.h>
#include <math.h>

#define M_PI 3.1415926
#define RAD2DEG (180.0f / (float)M_PI)

//占位
#define PIN_BASE       9        //底座舵机
#define PIN_SHOULDER   8       //大臂舵机
#define PIN_ELBOW      7      //小臂舵机
#define PIN_CLAW       6        //爪子舵机

//占位
#define CLAW_OPEN_DEG  120      //爪子张开
#define CLAW_CLOSE_DEG 20       //爪子闭合

#define BASE_SPEED_DPS   90.0f
#define SPEED_STEP_UP    1.25f
#define SPEED_STEP_DOWN  0.80f
#define SPEED_SCALE_MIN  0.25f
#define SPEED_SCALE_MAX  4.00f
#define TICK_MS          20


//占位
#define PIN_JOY_X    A0
#define PIN_JOY_Y    A2
#define JOY_CENTER    512
#define JOY_DEADZONE  30
#define JOY_RANGE     500
#define JOY_MAX_DPS  120.0f

#define CMD_LEN  32

//占位
#define L1_MM           80.0f //大臂长
#define L2_MM           80.0f//小臂长
#define L3_MM           68.0f//前伸量

//占位
#define TH1_OFFSET_DEG  90.0f
#define TH2_OFFSET_DEG   0.0f
#define TH3_OFFSET_DEG  90.0f
#define TH1_GAIN        (+1.0f)
#define TH2_GAIN        (+1.0f)
#define TH3_GAIN        (+1.0f)

#define APPROACH_H_MM    40.0f
#define WAIT_GRIP_MS      500 

Servo servos[3];
const uint8_t servoPins[3] = { PIN_BASE, PIN_SHOULDER, PIN_ELBOW };
Servo claw;

float targetAngle[3]  = { 90, 90, 90 };
float currentAngle[3] = { 90, 90, 90 };
float speedScale = 1.0f;
unsigned long lastTick = 0;
unsigned long lastJoy  = 0;

char cmd[CMD_LEN];
uint8_t cmdLen = 0;//计数器

bool grabActive = false;

int g_placeX, g_placeY, g_placeZ;

//占位
struct Target { int grabX, grabY, grabZ, placeX, placeY, placeZ, clawDeg; };//抓取点x, 抓取点y, 抓取点z,  放置点x, 放置点y, 放置点z,  夹持角
Target g_targetA = { 120, 30, 75,   60, -50, 75,  35 };
Target g_targetB = {  90, -60, 65,   50,  60, 65,  40 };
Target g_targetC = {  70, 80, 70,    40, -30, 70,  45 };

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 3; i++)
  {
    servos[i].attach(servoPins[i]);
    servos[i].write(90);
  }
  claw.attach(PIN_CLAW);
  claw.write(CLAW_OPEN_DEG);
  Serial.println("初始化完成");
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
        cmd[cmdLen] = '\0';
        handleLine(cmd);
        cmdLen = 0;
    }
    else if (c != '\r' && cmdLen < CMD_LEN - 1)
    {
      cmd[cmdLen++] = c;
    }
  }
}

void handleLine(char* line) {

  if (line[1] == '\0')//判断单个指令
  {
    switch (line[0])
     {
      case 'O':
        claw.write(CLAW_OPEN_DEG);
        Serial.println("成功打开");
        return;
      case 'S':
        claw.write(CLAW_CLOSE_DEG);
        Serial.println("成功关闭");
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
    
    Serial.print("成功，抓取");
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
     Serial.println("成功移动");
  }
}

void printSpeed() {
  Serial.print("成功，当前速度比例为");
  Serial.print(speedScale, 2);
  Serial.println("x");
}

void updateJoystick(unsigned long now) {

  if (now - lastJoy < 20)
  {
     return;
  }

  lastJoy = now;
  float dt = 0.02f;
  float vx = joySpeed(analogRead(PIN_JOY_X)) * speedScale;
  float vy = joySpeed(analogRead(PIN_JOY_Y)) * speedScale;

  targetAngle[1] += vx * dt;
  targetAngle[2] += vy * dt;
  for (int i = 1; i < 3; i++) {
    if (targetAngle[i] <   0)
    {
      targetAngle[i] =   0;
    }
    if (targetAngle[i] > 180)
    {
      targetAngle[i] = 180;
    }
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

void tick(unsigned long now) {

  if (now - lastTick < TICK_MS)
  {
  return;
  }

  float dt = (now - lastTick) / 1000.0f;
  lastTick = now;
  float scale = grabActive ? 1.0f : speedScale;
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
}
bool grabArrived() {
  for (int i = 0; i < 3; i++) {
    if (abs(targetAngle[i] - currentAngle[i]) > 1.00f)
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

  claw.write(CLAW_OPEN_DEG);
  moveTo(x, y, z);
  waitArrived();

  claw.write(clawDeg);
  waitMs(WAIT_GRIP_MS);

  moveTo(x, y, safeZ);
  waitArrived();

  moveTo(g_placeX, g_placeY, g_placeZ + (int)APPROACH_H_MM);
  waitArrived();

  moveTo(g_placeX, g_placeY, g_placeZ);
  waitArrived();

  claw.write(CLAW_OPEN_DEG);
  waitMs(WAIT_GRIP_MS);

  //占位
  targetAngle[0] = 90; targetAngle[1] = 90; targetAngle[2] = 90;
  claw.write(CLAW_OPEN_DEG);
  waitArrived();

  grabActive = false;
  Serial.println("完成");
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

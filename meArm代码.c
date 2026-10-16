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
#define CLAW_OPEN_DEG  50       //爪子张开
#define CLAW_CLOSE_DEG  110     //爪子闭合

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

#define L1_MM           83.0f //大臂长
#define L2_MM           80.0f//小臂长
#define L3_MM           35.0f//前伸量

//占位
#define TH1_OFFSET_DEG  90.0f
#define TH2_OFFSET_DEG   0.0f
#define TH3_OFFSET_DEG  90.0f
#define TH1_GAIN        (+1.0f)
#define TH2_GAIN        (+1.0f)
#define TH3_GAIN        (+1.0f)

#define APPROACH_H_MM    40.0f
#define WAIT_GRIP_MS      500

#define LIFT_DEG    30
#define LIFT_SIGN    1

#define FRAME_HEAD      '$'
#define FRAME_GES_MARK  'G'
#define FRAME_TAIL      '#'

//占位
#define HOME_SERVO1  90
#define HOME_SERVO2  90
#define HOME_SERVO3  90
#define HOME_CLAW_DEG  CLAW_OPEN_DEG

#define REC_BUFFER_BYTES   1020
#define REC_FRAME_BYTES    6
#define REC_FRAME_CAPACITY (REC_BUFFER_BYTES / REC_FRAME_BYTES)
#define REC_SAMPLE_MS     100

//占位
#define PAPER_Z_MM        50
#define PEN_UP_MM         25     //抬笔高度

#define PLOT_STEP_MM      4.0f
#define PLOT_STEP_DEG     3.0f
#define PLOT_STEP_MS      40
#define PLOT_SPEED_SCALE  1.0f
#define SPLINE_STEPS      20     //平滑曲线每段的采样点数

#define TEACH_MAX          5

#define DEMO_LINE_X1       0
#define DEMO_LINE_Y1     -50
#define DEMO_LINE_X2       0
#define DEMO_LINE_Y2      50

#define DEMO_N_X1          0
#define DEMO_N_Y1        -50
#define DEMO_N_X2          0
#define DEMO_N_Y2         50
#define DEMO_N_X3         40
#define DEMO_N_Y3        -50
#define DEMO_N_X4         40
#define DEMO_N_Y4         50

#define KEY1        1
#define KEY2        2
#define KEY3        3
#define KEY4        4
#define KEY5        5
#define KEY_PANIC   6

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

int8_t g_liftSign = LIFT_SIGN;

bool recStopRequest = false;
bool recording = false;

struct PlotPoint { float s1, s2, s3; };

bool plotActive    = false;
bool plotPauseReq  = false;
bool plotResumeReq = false;
bool plotCancelReq = false;
bool teachMode     = false;
uint8_t teachCount = 0;//已记录的示教点数

PlotPoint teachPts[TEACH_MAX];
float plotSavedScale = 0.6f;

struct Target { int grabS1, grabS2, grabS3, placeS1, placeS2, placeS3, clawDeg; };
Target g_targetA = { 94,0,140,   162,25,150,  100 };
Target g_targetB = { 94,0,140,   56,28,150,  100 };
Target g_targetC = { 94,0,140,   10,30,150,  100 };

enum FrameParseState { FP_WAIT_HEAD = 0, FP_WAIT_G, FP_WAIT_DIGIT, FP_WAIT_TAIL };

FrameParseState fpState = FP_WAIT_HEAD;
uint8_t fpNum = 0;

uint8_t drawMode  = 0;
bool    drawReady = false;
bool    plotPaused = false;

struct MotionFrame {
  uint16_t dt;
  int8_t   d1, d2, d3, dclaw;
};

MotionFrame recBuf[REC_FRAME_CAPACITY];
uint16_t recCount = 0;//计数器
unsigned long recHoldMs = 0;

uint8_t t3Counter = 0;

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
  Serial.println(F("初始化完成 [五按键版 K1~K5]"));
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

  if (plotActive) {
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

      case 'f': printFk();         return;
      case 'g': printAngles();     return;
      case 'a': plotRunLine();     return;
      case 'n': plotRunGraph();    return;
      case 'c': plotRunSpline();   return;

    }
  }

  if (line[0] == 'k' && line[2] == '\0') {
    if      (line[1] == '+') { g_liftSign =  1; }
    else if (line[1] == '-') { g_liftSign = -1; }
    else if (line[1] == '0') { g_liftSign = LIFT_SIGN; }
    Serial.print(F("抬升方向 = "));
    Serial.println(g_liftSign);
    return;
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
    printTarget(line[0], t);
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

  for (int i = 0; i < 3; i++) {
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

float endZAt(int s1, int s2, int s3) {
  float t1, t2, t3, x, y, z;
  servoToJoint(s1, s2, s3, &t1, &t2, &t3);
  fkSolve(t1, t2, t3, &x, &y, &z);
  return z;
}

int liftS2(int s1, int s2, int s3) {
  int up = s2 + LIFT_DEG;
  int dn = s2 - LIFT_DEG;
  if (up > 180) { up = 180; }
  if (dn <   0) { dn = 0;   }

  if (g_liftSign >= 0) {
    if (up == s2) { return dn; }
    return up;
  }
  if (dn == s2) { return up; }
  return dn;
}

void moveAngles(int s1, int s2, int s3) {
  targetAngle[0] = s1;
  targetAngle[1] = s2;
  targetAngle[2] = s3;
}

void grabRun(int gs1, int gs2, int gs3, int ps1, int ps2, int ps3, int clawDeg) {

  grabActive = true;

  int gUp = liftS2(gs1, gs2, gs3);
  int pUp = liftS2(ps1, ps2, ps3);

  moveAngles(gs1, gUp, gs3);
  waitArrived();

  clawMove(CLAW_OPEN_DEG);
  moveAngles(gs1, gs2, gs3);
  waitArrived();

  clawMove(clawDeg);
  waitMs(WAIT_GRIP_MS);

  moveAngles(gs1, gUp, gs3);
  waitArrived();

  moveAngles(ps1, pUp, ps3);
  waitArrived();

  moveAngles(ps1, ps2, ps3);
  waitArrived();

  clawMove(CLAW_OPEN_DEG);
  waitMs(WAIT_GRIP_MS);

  targetAngle[0] = HOME_SERVO1;
  targetAngle[1] = HOME_SERVO2;
  targetAngle[2] = HOME_SERVO3;
  clawMove(CLAW_OPEN_DEG);
  waitArrived();

  grabActive = false;
  Serial.println(F("完成"));
}

void grabFull(Target* t) {
  grabRun(t->grabS1, t->grabS2, t->grabS3,
          t->placeS1, t->placeS2, t->placeS3,
          t->clawDeg);
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

bool ikSolveNear(float xMm, float yMm, float zMm,
                 float refS1, float refS2, float refS3,
                 int* s1Out, int* s2Out, int* s3Out)
{
  float t1  = atan2f(yMm, xMm);
  float rho = sqrtf(xMm * xMm + yMm * yMm);
  float r   = rho - L3_MM;
  float R2  = r * r + zMm * zMm;
  float R   = sqrtf(R2);

  if (R > (L1_MM + L2_MM)) {
    return false;
  }

  float psi   = atan2f(zMm, r);
  float beta  = safeAcos((L1_MM * L1_MM + L2_MM * L2_MM - R2) / (2.0f * L1_MM * L2_MM));
  float gamma = safeAcos((L1_MM * L1_MM + R2 - L2_MM * L2_MM) / (2.0f * L1_MM * R));
  float elbow = (float)M_PI - beta;

  float bestD = 1.0e9f;
  int   bs1 = 0, bs2 = 0, bs3 = 0;

  for (int i = 0; i < 2; i++) {
    float t2 = (i == 0) ? (psi + gamma) : (psi - gamma);
    float t3 = (i == 0) ? (t2 - elbow)   : (t2 + elbow);
    int a1, a2, a3;
    jointToServo(t1 * RAD2DEG, t2 * RAD2DEG, t3 * RAD2DEG, &a1, &a2, &a3);
    float d = fabsf((float)a1 - refS1) + fabsf((float)a2 - refS2) + fabsf((float)a3 - refS3);
    if (d < bestD) {
      bestD = d;
      bs1 = a1; bs2 = a2; bs3 = a3;
    }
  }

  *s1Out = bs1; *s2Out = bs2; *s3Out = bs3;
  return true;
}

void fpFeed(char c) {
  switch (fpState) {
    case FP_WAIT_HEAD:
      if (c == FRAME_HEAD) {
        fpState = FP_WAIT_G;
      }
      break;
    case FP_WAIT_G:
      if (c == FRAME_HEAD) {
        break;
      }
      if (c == FRAME_GES_MARK) {
        fpNum = 0;
        fpState = FP_WAIT_DIGIT;
        break;
      }
      fpState = FP_WAIT_HEAD;
      break;
    case FP_WAIT_DIGIT:
      if (c >= '0' && c <= '9') {
        fpNum = (uint8_t)(c - '0');
        break;
      }
      if (c == FRAME_TAIL) {
        if (fpNum >= 1 && fpNum <= 6) {
          keyDispatch(fpNum);
        }
        fpNum = 0;
        cmdLen = 0;
        fpState = FP_WAIT_HEAD;
        break;
      }
      fpNum = 0;
      fpState = (c == FRAME_HEAD) ? FP_WAIT_G : FP_WAIT_HEAD;
      break;
    case FP_WAIT_TAIL:
      fpState = (c == FRAME_HEAD) ? FP_WAIT_G : FP_WAIT_HEAD;
      break;
  }
}

void drawModeEnter() {
  drawMode   = 1;
  drawReady  = false;
  teachMode  = true;
  teachCount = 0;
  Serial.println(F("DRAW_MODE"));
  Serial.println(F("  摇杆摆点 -> K1记点；K4=清空重点；K5=结束示教(再按一次=退出)"));
}

void drawReadyHint() {
  drawReady = true;
  Serial.println(F("TEACH_READY"));
  Serial.print(F("  已记 "));
  Serial.print(teachCount);
  Serial.println(F(" 点：K3=直线(要2点) K4=图形N(要4点) K1=折线 K2=曲线 K5=退出"));
}

void drawModeExit() {
  drawMode  = 0;
  drawReady = false;
  Serial.println(F("BACK_TO_ACTION"));
  goHome();
}

void keyDispatch(uint8_t g) {

  if (g == KEY_PANIC) {
    Serial.println(F("EMERGENCY_STOP"));
    plotPauseReq  = false;
    plotResumeReq = false;
    plotPaused    = false;
    if (recording) {
      recStopRequest = true;
    }
    drawMode  = 0;
    drawReady = false;
    teachMode = false;
    if (plotActive) {
      plotCancelReq = true;
    } else {
      goHome();
    }
    return;
  }

  if (plotActive) {
    if (g == KEY1) {
      if (!plotPaused) {
        Serial.println(F("暂停请求"));
        plotPauseReq = true;
      }
    } else if (g == KEY2) {
      if (plotPaused) {
        Serial.println(F("继续请求"));
        plotResumeReq = true;
      }
    } else if (g == KEY3) {
      Serial.println(F("取消绘图"));
      plotCancelReq = true;
    }
    return;
  }

  if (recording) {
    if (g == KEY2) {
      recStopRequest = true;
    } else {
      Serial.println(F("录制中：K2=结束录制"));
    }
    return;
  }

  if (drawMode) {
    if (teachMode) {
      if (g == KEY1) {
        teachRecord();
        if (!teachMode) {
          drawReadyHint();
        }
      } else if (g == KEY4) {
        teachCount = 0;
        Serial.println(F("已清空示教点，重新从第1点开始"));
      } else if (g == KEY5) {
        teachMode = false;
        drawReadyHint();
      } else {
        Serial.println(F("示教中：K1=记点 K4=清空 K5=结束示教"));
      }
      return;
    }

    if (drawReady) {

      if (teachCount == 0) {
        Serial.println(F("还没示教点：K1=记点"));
        drawReady = false;
        teachMode = true;
        return;
      }
      switch (g) {
        case KEY1: plotRunPolyline(); return;
        case KEY2: plotRunSpline();   return;
        case KEY3: plotRunLine();     return;
        case KEY4: plotRunGraph();    return;
        case KEY5: drawModeExit();    return;
      }
      return;
    }

    drawModeExit();
    return;
  }

  switch (g) {
    case KEY1: {
      uint8_t idx = t3Counter % 3;
      t3Counter++;
      Target* t = (idx == 0) ? &g_targetA
                : (idx == 1) ? &g_targetB : &g_targetC;
      Serial.print(F("正在移动"));
      Serial.println((idx == 0) ? 'A' : (idx == 1) ? 'B' : 'C');
      grabFull(t);
      return;
    }
    case KEY2:
      recRun();
      return;
    case KEY3:
      playRun();
      return;
    case KEY4:
      goHome();
      return;
    case KEY5:
      drawModeEnter();
      return;
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

  void servoToJoint(int s1, int s2, int s3, float* t1Out, float* t2Out, float* t3Out) {
  *t1Out = (s1 - TH1_OFFSET_DEG) / TH1_GAIN;
  *t2Out = (s2 - TH2_OFFSET_DEG) / TH2_GAIN;
  *t3Out = (s3 - TH3_OFFSET_DEG) / TH3_GAIN;
}

void fkSolve(float t1, float t2, float t3, float* xOut, float* yOut, float* zOut) {
  float t1r = t1 / RAD2DEG;
  float t2r = t2 / RAD2DEG;
  float t3r = t3 / RAD2DEG;

  float rw  = L1_MM * cosf(t2r) + L2_MM * cosf(t3r);   //腕点的水平距离
  float zz  = L1_MM * sinf(t2r) + L2_MM * sinf(t3r);   //腕点的高度
  float rho = rw + L3_MM;

  *xOut = rho * cosf(t1r);
  *yOut = rho * sinf(t1r);
  *zOut = zz;
}

void printFk() {
  float t1, t2, t3, x, y, z;
  servoToJoint((int)(targetAngle[0] + 0.5f),(int)(targetAngle[1] + 0.5f),(int)(targetAngle[2] + 0.5f), &t1, &t2, &t3);
  fkSolve(t1, t2, t3, &x, &y, &z);
  Serial.print(F("FK:"));
  Serial.print((int)(x + 0.5f));
  Serial.print(',');
  Serial.print((int)(y + 0.5f));
  Serial.print(',');
  Serial.println((int)(z + 0.5f));
}

void printAngles() {
  Serial.print(F("目标角:"));
  Serial.print((int)(targetAngle[0] + 0.5f));
  Serial.print(',');
  Serial.print((int)(targetAngle[1] + 0.5f));
  Serial.print(',');
  Serial.println((int)(targetAngle[2] + 0.5f));
  Serial.print(F("实际角:"));
  Serial.print((int)(currentAngle[0] + 0.5f));
  Serial.print(',');
  Serial.print((int)(currentAngle[1] + 0.5f));
  Serial.print(',');
  Serial.println((int)(currentAngle[2] + 0.5f));
  printFk();
}

void printTarget(char which, Target* t) {
  Serial.print(F("目标"));
  Serial.print(which);
  Serial.print(F(": 抓("));
  Serial.print(t->grabS1);  Serial.print(','); Serial.print(t->grabS2);  Serial.print(','); Serial.print(t->grabS3);
  Serial.print(F(") 放("));
  Serial.print(t->placeS1); Serial.print(','); Serial.print(t->placeS2); Serial.print(','); Serial.print(t->placeS3);
  Serial.print(F(") 爪"));
  Serial.println(t->clawDeg);
}

void plotPump() {
  readSerial();
  tick(millis());

  if (plotPauseReq) {
    plotPauseReq = false;
    plotPaused   = true;
    Serial.println(F("暂停"));
    while (!plotResumeReq && !plotCancelReq) {
      readSerial();
      tick(millis());
    }
    plotPaused    = false;
    plotPauseReq  = false;
    plotResumeReq = false;
    if (!plotCancelReq) {
      Serial.println(F("继续"));
    }
  }
}

void plotWaitArrived() {
  while (!grabArrived() && !plotCancelReq) plotPump();
}

void plotWaitMs(unsigned long ms) {
  unsigned long start = millis();
  while ((millis() - start) < ms) {
    plotPump();
    if (plotCancelReq) return;
  }
}

void plotPenUp() {
  int s1 = (int)(targetAngle[0] + 0.5f);
  int s2 = (int)(targetAngle[1] + 0.5f);
  int s3 = (int)(targetAngle[2] + 0.5f);
  moveAngles(s1, liftS2(s1, s2, s3), s3);
  plotWaitArrived();
}

void plotBegin(const char* tag) {
  plotSavedScale = speedScale;
  speedScale     = PLOT_SPEED_SCALE;      //绘制速度锁定，保证轨迹平稳
  plotActive     = true;
  plotPauseReq   = false;
  plotResumeReq  = false;
  plotCancelReq  = false;
  Serial.print(F("PLOT_START:"));
  Serial.println(tag);
}

void plotEnd(bool cancelled) {
  plotCancelReq = false;
  plotPauseReq  = false;
  plotResumeReq = false;

  plotPenUp();
  targetAngle[0] = HOME_SERVO1;
  targetAngle[1] = HOME_SERVO2;
  targetAngle[2] = HOME_SERVO3;
  plotWaitArrived();

  speedScale = plotSavedScale;
  plotActive = false;

  Serial.println(cancelled ? F("PLOT_CANCEL") : F("PLOT_DONE"));
}

void plotRunLine() {
  if (teachCount < 2) {
    Serial.println(F("PLOT_ERR:NEED_2PT"));
    return;
  }
  plotRunTeachSegs(1, "LINE");
}

void plotRunGraph() {
  if (teachCount < 4) {
    Serial.println(F("PLOT_ERR:NEED_4PT"));
    return;
  }
  plotRunTeachSegs(3, "GRAPH_N");
}

void teachRecord() {
  if (!teachMode) {
    teachMode  = true;
    teachCount = 0;
    Serial.println(F("TEACH_START"));
    return;
  }

  if (teachCount >= TEACH_MAX) return;

  teachPts[teachCount].s1 = targetAngle[0];
  teachPts[teachCount].s2 = targetAngle[1];
  teachPts[teachCount].s3 = targetAngle[2];
  teachCount++;

  Serial.print(F("TEACH_PT"));
  Serial.print(teachCount);
  Serial.print(':');
  Serial.print((int)(targetAngle[0] + 0.5f));
  Serial.print(',');
  Serial.print((int)(targetAngle[1] + 0.5f));
  Serial.print(',');
  Serial.println((int)(targetAngle[2] + 0.5f));

  if (teachCount >= TEACH_MAX) {
    teachMode = false;
    Serial.println(F("TEACH_DONE"));
  }
}

bool teachGotoAbove(uint8_t idx) {
  int s1 = (int)(teachPts[idx].s1 + 0.5f);
  int s2 = (int)(teachPts[idx].s2 + 0.5f);
  int s3 = (int)(teachPts[idx].s3 + 0.5f);
  moveAngles(s1, liftS2(s1, s2, s3), s3);
  plotWaitArrived();
  return !plotCancelReq;
}

bool teachDescend(uint8_t idx) {
  targetAngle[0] = teachPts[idx].s1;
  targetAngle[1] = teachPts[idx].s2;
  targetAngle[2] = teachPts[idx].s3;
  plotWaitArrived();
  return !plotCancelReq;
}

bool angleSegTo(uint8_t a, uint8_t b) {
  float d1 = teachPts[b].s1 - teachPts[a].s1;
  float d2 = teachPts[b].s2 - teachPts[a].s2;
  float d3 = teachPts[b].s3 - teachPts[a].s3;

  float dmax = fmaxf(fmaxf(fabsf(d1), fabsf(d2)), fabsf(d3));
  int steps = (int)(dmax / PLOT_STEP_DEG + 0.5f);
  if (steps < 1){
    steps = 1;
  }

  for (int i = 1; i <= steps; i++) {
    float u = (float)i / (float)steps;
    targetAngle[0] = teachPts[a].s1 + d1 * u;
    targetAngle[1] = teachPts[a].s2 + d2 * u;
    targetAngle[2] = teachPts[a].s3 + d3 * u;
    plotWaitMs(PLOT_STEP_MS);
    if (plotCancelReq){
      return false;
    }
  }
  return true;
}

void plotRunTeachSegs(uint8_t segs, const char* tag) {
  plotBegin(tag);

  targetAngle[0] = HOME_SERVO1;
  targetAngle[1] = HOME_SERVO2;
  targetAngle[2] = HOME_SERVO3;
  plotWaitArrived();
  if (plotCancelReq) {
    plotEnd(true);
    return;
  }

  if (!teachGotoAbove(0)) {
    plotEnd(true);
    return;
  }
  if (!teachDescend(0)) {                //落笔
    plotEnd(true);
    return;
  }

  for (uint8_t i = 0; i < segs; i++) {
    if (plotCancelReq) {
      break;
    }
    if (!angleSegTo(i, i + 1)) {
      break;
    }
  }

  plotEnd(plotCancelReq);
}

void plotRunPolyline() {
  if (teachCount < 2) {
    Serial.println(F("PLOT_ERR:NO_TEACH"));
    return;
  }
  plotRunTeachSegs((uint8_t)(teachCount - 1), "POLYLINE");
}

float crAxis(float p0, float p1, float p2, float p3, float u) {
  float u2 = u * u, u3 = u2 * u;
  return 0.5f * ((2.0f * p1)+ (-p0 + p2) *u + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * u2+ (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * u3);
}

void crPoint(int idx, float* s1Out, float* s2Out, float* s3Out) {
  if (idx < 0) idx = 0;
  if (idx >= (int)teachCount) idx = (int)teachCount - 1;
  *s1Out = teachPts[idx].s1;
  *s2Out = teachPts[idx].s2;
  *s3Out = teachPts[idx].s3;
}

void plotRunSpline() {
  if (teachCount < 2) {
    Serial.println(F("PLOT_ERR:NO_TEACH"));
    return;
  }
  plotBegin("SPLINE");

  targetAngle[0] = HOME_SERVO1;
  targetAngle[1] = HOME_SERVO2;
  targetAngle[2] = HOME_SERVO3;
  plotWaitArrived();
  if (plotCancelReq) { plotEnd(true); return; }

  if (!teachGotoAbove(0)) { plotEnd(true); return; }
  if (!teachDescend(0))   { plotEnd(true); return; }

  for (uint8_t j = 0; j + 1 < teachCount; j++) {
    float a1, a2, a3, b1, b2, b3, c1, c2, c3, e1, e2, e3;
    crPoint((int)j - 1, &a1, &a2, &a3);
    crPoint((int)j,     &b1, &b2, &b3);
    crPoint((int)j + 1, &c1, &c2, &c3);
    crPoint((int)j + 2, &e1, &e2, &e3);

    for (int s = 1; s <= SPLINE_STEPS; s++) {
      float u = (float)s / (float)SPLINE_STEPS;
      targetAngle[0] = crAxis(a1, b1, c1, e1, u);
      targetAngle[1] = crAxis(a2, b2, c2, e2, u);
      targetAngle[2] = crAxis(a3, b3, c3, e3, u);
      plotWaitMs(PLOT_STEP_MS);
      if (plotCancelReq){
         plotEnd(true);
         return;
        }
    }
  }

  plotEnd(false);
}

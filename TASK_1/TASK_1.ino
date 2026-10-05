#include <Servo.h>
#include <stdlib.h>

#define PIN_BASE       9        //底座舵机
#define PIN_SHOULDER  10        //大臂舵机
#define PIN_ELBOW     11        //小臂舵机
#define PIN_CLAW       6        //爪子舵机

#define CLAW_OPEN_DEG  120      //爪子张开
#define CLAW_CLOSE_DEG 20       //爪子闭合

#define BASE_SPEED_DPS   90.0f
#define SPEED_STEP_UP    1.25f
#define SPEED_STEP_DOWN  0.80f
#define SPEED_SCALE_MIN  0.25f
#define SPEED_SCALE_MAX  4.00f 
#define TICK_MS          20

#define PIN_JOY_X    A0
#define PIN_JOY_Y    A1
#define JOY_CENTER    512
#define JOY_DEADZONE  30
#define JOY_RANGE     500
#define JOY_MAX_DPS  120.0f

#define CMD_LEN  32

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
 
  targetAngle[0] += vx * dt;
  targetAngle[1] += vy * dt;
  for (int i = 0; i < 2; i++) {
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
  for (int i = 0; i < 3; i++) {
    float diff = targetAngle[i] - currentAngle[i];
    if (diff == 0.0f) 
    {
      continue;
    }
    float step = BASE_SPEED_DPS * speedScale * dt;
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
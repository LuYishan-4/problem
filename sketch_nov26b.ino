







#include <hcsr04.h>

#define TRIG_PIN_FRONT1 13
#define ECHO_PIN_FRONT1 12

// 左側(前)
#define TRIG_PIN_LEFT3 7
#define ECHO_PIN_LEFT3 6

// 左側(後)
#define TRIG_PIN_LEFT4 10
#define ECHO_PIN_LEFT4 11

// 與前車距離 12 公分 ( 120mm )
#define KEEP_DIST_FRONT 120

// 與左方牆壁距離 4 公分 ( 40mm )
#define KEEP_DIST_RIGHT 40
// 與左方牆壁距離超出 7 公分 ( 70mm ) ( 22 軌寬 - 15 車寬) 算沒有牆壁, 改用 KY010 轉速偵測直線前進
#define LOST_DIST_RIGHT 70
#define DISTAP 10
// 右側 (前) - 右側 (後) (距離牆壁差 單位: mm)
#define US_SHIFT_MM 8

#define MAX_DIST 400 // 超音波最大距離 mm
#define MIN_DIST 10 // 超音波最大距離 mm

//==== 馬達接腳定義 ====
#define MR_B1 26 // 5  // 左馬達輸入 Forward
#define MR_F1 27 // 6  // 左馬達輸入 Backward
#define ML_B1 22  // 11 // 右馬達輸入 Forward
#define ML_F1 23  // 10 // 右馬達輸入 Backward

#define KY010_L 34 // 左馬達轉速編碼 中斷腳
#define KY010_R 3 // 右馬達轉速編碼 中斷腳

// 左右馬達轉速修正比例 L * RL_RATE = R


#define SPEED_L 200   //馬達速度 低
#define SPEED_H 400   //馬達速度 高
#define SPEED_RD 50





HCSR04 hcsr04_f1(TRIG_PIN_FRONT1, ECHO_PIN_FRONT1, MIN_DIST, MAX_DIST);
HCSR04 hcsr04_l3(TRIG_PIN_LEFT3, ECHO_PIN_LEFT3, MIN_DIST, MAX_DIST);
HCSR04 hcsr04_l4(TRIG_PIN_LEFT4, ECHO_PIN_LEFT4, MIN_DIST, MAX_DIST);


volatile int leftTicks = 0;  // 左轮转速计数
volatile int rightTicks = 0; // 右轮转速计数


void setup() {
   Serial.begin(115200); 
    pinMode(MR_F1,OUTPUT);
    pinMode(MR_B1,OUTPUT);
    pinMode(ML_F1,OUTPUT);
    pinMode(ML_B1,OUTPUT);
    pinMode(TRIG_PIN_FRONT1,OUTPUT);
    pinMode(ECHO_PIN_FRONT1,INPUT);
    pinMode(TRIG_PIN_LEFT3,OUTPUT);
    pinMode(ECHO_PIN_LEFT3,INPUT);
    pinMode(TRIG_PIN_LEFT4,OUTPUT);
    pinMode(ECHO_PIN_LEFT4,INPUT);
    pinMode(KY010_L,INPUT);
    pinMode(KY010_R,INPUT);
     // 與電腦序列埠連線
attachInterrupt(digitalPinToInterrupt(KY010_L),  countLeftTicks, RISING);
attachInterrupt(digitalPinToInterrupt(KY010_R),  countRightTicks, RISING);
  
    

  //  pinMode(B1,INPUT);
 

    // 前右二組超音波雷達


  // 慣性導航初始化, 需靜置到蜂鳴器響一長聲
  // ins.setup(false);


  

}

void loop() {
int distanceFront = hcsr04_f1.distanceInMillimeters();
int distancel1 = hcsr04_l3.distanceInMillimeters();
int distancel2 = hcsr04_l4.distanceInMillimeters();
int aLeft = (distancel1 + distancel2) / 2;
int distanceError = aLeft - KEEP_DIST_RIGHT;
  Serial.print("Front Distance: ");
  Serial.print(distanceFront);
  Serial.print("mm,left:");
  Serial.print(aLeft);
  Serial.println("mm");
  delay(500);


  if (distanceFront < KEEP_DIST_FRONT) {
    stopMotors();
  } else if (distanceFront > KEEP_DIST_FRONT) {
    chaseObject(distanceFront);  
  }  else if (abs(distanceError) > DISTAP) {
        adjustToWall(distanceError);
    }
}
void chaseObject(int distanceFront) {
  
  int distanceToGo = distanceFront - KEEP_DIST_FRONT;

  
  int speed =map(distanceToGo,0,300,SPEED_L,SPEED_H);

  
  analogWrite(ML_B1, 0);
  analogWrite(ML_F1, speed);  
  analogWrite(MR_B1, 0);
  analogWrite(MR_F1, speed);  
}
void adjustToWall(int error) {
    int leftSpeed = SPEED_H;
    int rightSpeed = SPEED_H;

    if (error > 0) {
        
        leftSpeed += SPEED_RD;
        rightSpeed -= SPEED_RD;
    } else {
        
        leftSpeed -= SPEED_RD;
        rightSpeed += SPEED_RD;
    }

    
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    analogWrite(ML_F1, leftSpeed);
    analogWrite(MR_F1, rightSpeed);
}
void countLeftTicks() {
    leftTicks++;
}


void countRightTicks() {
    rightTicks++;
}










void stopMotors() {
  analogWrite(ML_B1, LOW);
  analogWrite(ML_F1, LOW);
  analogWrite(MR_B1, LOW);
  analogWrite(MR_F1, LOW);
  Serial.println("Emergency Stop!");
}





 


  
#include <NewPing.h>

// 超音波模組引腳定義
#define TRIG_PIN_FRONT 13
#define ECHO_PIN_FRONT 12
#define TRIG_PIN_LEFT 7
#define ECHO_PIN_LEFT 6
#define TRIG_PIN_RIGHT 11
#define ECHO_PIN_RIGHT 10

// 超音波模組最大距離
#define LOW_DISTANCE 100


// 馬達控制引腳
#define MR1 27 // 右馬達輸入 Forward
#define MR2 26 // 右馬達輸入 Backward
#define ML1 23 // 左馬達輸入 Forward
#define ML2 22 // 左馬達輸入 Backward

// 急停距離
#define STOP_DISTANCE 10

// 安全距離
#define SAFE_DISTANCE 30

// 建立超音波模組物件
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, LOW_DISTANCE);
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, LOW_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, LOW_DISTANCE);

// 車速控制變數
int speedLeft = 150;  // 左馬達初始速度 (PWM)
int speedRight = 150; // 右馬達初始速度 (PWM)

void setup() {
  pinMode(ML2, OUTPUT);
  pinMode(ML1, OUTPUT);
  pinMode(MR2, OUTPUT);
  pinMode(ML1, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // 讀取距離
  int distanceFront = sonarFront.ping_cm();
  int distanceLeft = sonarLeft.ping_cm();
  int distanceRight = sonarRight.ping_cm();

  // Debug: 打印距離
  Serial.print("Front: ");
  Serial.print(distanceFront);
  Serial.print(" cm, Left: ");
  Serial.print(distanceLeft);
  Serial.print(" cm, Right: ");
  Serial.println(distanceRight);

  // 判斷急停
  if  (distanceFront < STOP_DISTANCE) {
    stopMotors();} // 急停
   else {
    maintainSpeed(); // 維持速度
  }

  // 根據側面距離調整方向
  adjustDirection(distanceLeft, distanceRight);

  delay(50); // 減少感測頻率，避免干擾
}

void stopMotors() {
  analogWrite(ML2, LOW);
  analogWrite(ML1, LOW);
  analogWrite(MR2, LOW);
  analogWrite(MR1, LOW);
  Serial.println("Emergency Stop!");
}

void slowDown() {
  analogWrite(ML2, speedLeft / 2);
  analogWrite(ML1, speedLeft / 2);
  analogWrite(MR2, speedRight / 2);
  analogWrite(MR1, speedRight / 2);
  Serial.println("Slowing Down...");
}

void maintainSpeed() {
  analogWrite(ML2, speedLeft);
  analogWrite(ML1, speedLeft);
  analogWrite(MR2, speedRight);
  analogWrite(MR1, speedRight);

  Serial.println("Maintaining Speed...");
}

void adjustDirection(int left, int right) {
  if (left > 0 && left < SAFE_DISTANCE) {
    // 向右微調
    analogWrite(ML2, speedLeft);
    analogWrite(MR2, speedRight - 100);

    Serial.println("Adjusting Right...");
  } else if (right > 0 && right < SAFE_DISTANCE) {
    // 向左微調
    analogWrite(ML2, speedLeft - 100);
    analogWrite(MR2, speedRight);
    Serial.println("Adjusting Left...");
  } else {
    // 保持直行
    analogWrite(ML2, speedLeft);
    analogWrite(ML1, speedLeft);
    analogWrite(MR2, speedRight);
    analogWrite(MR1, speedRight);
  }
}


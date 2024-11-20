// 馬達驅動模組引腳定義
#define MR1 27 // 右馬達輸入 Forward
#define MR2 26 // 右馬達輸入 Backward
#define ML1 23 // 左馬達輸入 Forward
#define ML2 22 // 左馬達輸入 Backward

#define SPEED 150       // 馬達速度 (0~255)
#define SAFE_DISTANCE 10 // 安全距離 (厘米)

// 超聲波感測器引腳定義
#define FRONT_TRIG 13
#define FRONT_ECHO 12
#define LEFT_TRIG 7
#define LEFT_ECHO 6
#define RIGHT_TRIG 11
#define RIGHT_ECHO 10

void setup() {
  // 初始化序列監視器
  Serial.begin(115200);
  Serial.println("Ultrasonic Avoidance with 3 Sensors");

  // 設置馬達引腳為輸出
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);

  // 設置超聲波感測器引腳
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  // 初始馬達停止
  stopMotors();
}

void loop() {
  // 讀取三個方向的距離
  int frontDistance = getDistance(FRONT_TRIG, FRONT_ECHO);
  int leftDistance = getDistance(LEFT_TRIG, LEFT_ECHO);
  int rightDistance = getDistance(RIGHT_TRIG, RIGHT_ECHO);

  // 在序列監視器中顯示距離
  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.print(" cm, Left: ");
  Serial.print(leftDistance);
  Serial.print(" cm, Right: ");
  Serial.println(rightDistance);

  // 根據距離判斷動作
  if (frontDistance < SAFE_DISTANCE) {
    if (leftDistance > rightDistance) {
      // 前方有障礙物，左側較寬，左轉
      turnLeft();
    } else {
      // 前方有障礙物，右側較寬，右轉
      turnRight();
    }
  } else {
    // 前方無障礙物，直行
    moveForward(SPEED);
  }

  delay(100); // 避免超聲波感測器過快觸發
}

// 控制馬達向前
void moveForward(int speed) {
  analogWrite(MR1, speed);
  analogWrite(MR2, 0);
  analogWrite(ML1, speed);
  analogWrite(ML2, 0);
  Serial.println("Moving Forward");
}

// 控制馬達向左轉
void turnLeft() {
  analogWrite(MR1, SPEED);
  analogWrite(MR2, 0);
  analogWrite(ML1, 0);
  analogWrite(ML2, SPEED);
  Serial.println("Turning Left");
}

// 控制馬達向右轉
void turnRight() {
  analogWrite(MR1, 0);
  analogWrite(MR2, SPEED);
  analogWrite(ML1, SPEED);
  analogWrite(ML2, 0);
  Serial.println("Turning Right");
}

// 停止馬達
void stopMotors() {
  analogWrite(MR1, 0);
  analogWrite(MR2, 0);
  analogWrite(ML1, 0);
  analogWrite(ML2, 0);
  Serial.println("Motors Stopped");
}

// 讀取超聲波距離
int getDistance(int trigPin, int echoPin) {
  // 發送超聲波信號
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 接收回波時間
  long duration = pulseIn(echoPin, HIGH);

  // 計算距離（厘米）
  int distance = duration * 0.034 / 2;
  return distance;
}


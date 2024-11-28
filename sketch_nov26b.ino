#include <hcsr04.h>



#include <Motor.h>
#include <MotorKy010.h>
#include <SensorService.h>
#include <const.h>
#include <ultra_sonic.h>

// HC-SR04 超音波測距模組
// 前方
#define TRIG_PIN_FRONT 13
#define ECHO_PIN_FRONT 12

// 左側(前)
#define TRIG_PIN_LEFT1 7
#define ECHO_PIN_LEFT1 6

// 左側(後)
#define TRIG_PIN_LEFT2 10
#define ECHO_PIN_LEFT2 11

// 與前車距離 12 公分 ( 120mm )
#define KEEP_DIST_FRONT 120

// 與左方牆壁距離 4 公分 ( 40mm )
#define KEEP_DIST_RIGHT 40
// 與左方牆壁距離超出 7 公分 ( 70mm ) ( 22 軌寬 - 15 車寬) 算沒有牆壁, 改用 KY010 轉速偵測直線前進
#define LOST_DIST_RIGHT 70

// 右側 (前) - 右側 (後) (距離牆壁差 單位: mm)
#define US_SHIFT_MM 8

#define MAX_DIST 400 // 超音波最大距離 mm

//==== 馬達接腳定義 ====
#define MR_B 26 // 5  // 左馬達輸入 Forward
#define MR_F 27 // 6  // 左馬達輸入 Backward
#define ML_B 22  // 11 // 右馬達輸入 Forward
#define ML_F 23  // 10 // 右馬達輸入 Backward

#define M_KY010_L_PIN 34 // 左馬達轉速編碼 中斷腳
#define M_KY010_R_PIN 3 // 右馬達轉速編碼 中斷腳

// 左右馬達轉速修正比例 L * RL_RATE = R
#define RL_RATE 1.4

// 蜂鳴器 註解掉就不會響
#define BUZZER 9

#define B1 2 // 按鈕1
#define SPEED_L  100   //馬達速度 低
#define SPEED_H 200   //馬達速度 高

int speed[] = {0, SPEED_L, SPEED_H, SPEED_L, 0, -SPEED_L, -SPEED_H, -SPEED_L };
int i = 0;
ultra_sonic radar;
// INS_MPU6050 ins;
Motor motor;
bool sensor_ky010 = false;
#ifdef ARDUINO_SENSORSERVICE_H
SoftwareSerial monitorBT(PIN_BT_RX, PIN_BT_TX);
SensorService monitor(&monitorBT);
#endif








void readSensors(); 


void setup() {
    pinMode(MR_F,OUTPUT);
    pinMode(MR_B,OUTPUT);
    pinMode(ML_F,OUTPUT);
    pinMode(ML_B,OUTPUT);
    pinMode(TRIG_PIN_FRONT,OUTPUT);
    pinMode(ECHO_PIN_FRONT,INPUT);
    pinMode(TRIG_PIN_LEFT1,OUTPUT);
    pinMode(ECHO_PIN_LEFT1,INPUT);
    pinMode(TRIG_PIN_LEFT2,OUTPUT);
    pinMode(ECHO_PIN_LEFT2,INPUT);
    Serial.begin(115200);   // 與電腦序列埠連線
  Serial.println("HCSR04 test ready!\n");

  
    

  //  pinMode(B1,INPUT);
    digitalWrite(MR_F, LOW);
    digitalWrite(MR_B, LOW);
    digitalWrite(ML_F, LOW);
    digitalWrite(ML_B, LOW);

    // 前右二組超音波雷達
  radar.setup(false);

  // 慣性導航初始化, 需靜置到蜂鳴器響一長聲
  // ins.setup(false);

  // KY010 光遮斷計數器
  KY010_setup();

  // 馬達初始化, 需先定義 MR_F, MR_B, ML_F, ML_B 針腳常數於 const.h
  motor.setup(true);
  #ifdef ARDUINO_SENSORSERVICE_H
  monitorBT.begin(BT_BAUD_RATE);
#endif
  

}

void loop() {
 int s = speed[i];
  Serial.print( i ); Serial.print(" speed="); Serial.println(s);

  Motor( s );
  i++;
  if( i>7 ) i=0;
  delay(1000);
  
  }


void Motor(int s) {
  if (s > 255) s = 255;
  else if (s < -255) s = -255;

  if (s > 0) {
    analogWrite(ML_F, s);
    analogWrite(ML_B, 0);
    analogWrite(MR_F, s);
    analogWrite(MR_B, 0);
  } else if (s == 0) {
    // 停止所有馬達
    analogWrite(ML_F, 0);
    analogWrite(ML_B, 0);
    analogWrite(MR_F, 0);
    analogWrite(MR_B, 0);  // 確保所有馬達引腳設為 0
    digitalWrite(ML_F, LOW);  // 強制設為 LOW
    digitalWrite(ML_B, LOW);  // 強制設為 LOW
    digitalWrite(MR_F, LOW);  // 強制設為 LOW
    digitalWrite(MR_B, LOW);  // 強制設為 LOW
  }
}
void readSensors()
{
   int count_L = 0, count_R = 0;

  radar.read();  // 讀取超音波感測器數據

  // 讀取前方距離
  int front_dist = radar.getFront();  // 假設 radar.getFront() 可以返回前方距離（以毫米為單位）

    int speed_f = motor.calc_speed(KEEP_DIST_FRONT, front_dist);
    KY010_read(count_L, count_R);

    // 使用超音波與牆壁保持 KEEP_DIST_RIGHT mm
    String data_json = "{";
#ifdef ARDUINO_SENSORSERVICE_H
    monitor.listen();
    data_json += "\"df\":" + String(front_dist, 1);
    data_json += ",\"dl1\":" + String(radar.getLeft1(), 1);
    data_json += ",\"dl2\":" + String(radar.getLeft2(), 1);
    data_json += ",\"cL\":" + String(count_L);
    data_json += ",\"cR\":" + String(count_R);
#endif
    if (radar.in_wall()) {
      // 超音波感測到牆壁
      if (speed_f > MIN_SPEED || speed_f < -MIN_SPEED) {
        int sL = 0, sR = 0;
        int speed = motor.convert_speed(speed_f);
        radar.getSpeed(speed, &sL, &sR);

        // 驅動車輛
        motor.driveUS(speed, sL, sR, data_json);
      } else {
        motor.driveUS(0, 0, 0, data_json);
      }

      KY010_reset_pid();
    } else {
      // 沒有牆壁，使用光遮斷計數器保持直線
      int speed_diff = KY010_pid(count_L, count_R);
      Serial.print("NOT in wall, speed_diff=");
      Serial.println(speed_diff);
      // 驅動車輛
      motor.drive(speed_f, speed_diff, data_json);
    }
  }

  


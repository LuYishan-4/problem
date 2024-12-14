#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

#include <SoftwareSerial.h>

#include <JsonService.h>
#include <Motor.h>
#include <const.h>
#define PIN_BT_RX 51
#define PIN_BT_TX 50

#define SR_BAUD_RATE 9600
#define BT_BAUD_RATE 9600
#define USE_PCA9685_SERVO_EXPANDER
#define ENABLE_EASE_CUBIC
#define MR_F 38
#define MR_B 34
#define ML_F 31
#define ML_B 53

#define PIN_SERVO_2 11

#define PIN_SERVO_4 8
#define SERVO_DEGREE_PER_SECOND 45

#define INIT_ANG1 120
#define INIT_ANG2 150

#define INIT_ANG3 60
#define INIT_ANG4 130

SoftwareSerial appBT(PIN_BT_RX, PIN_BT_TX);
// 建立一個 JSON 物件
DynamicJsonDocument json(JSON_BUFFER_LEN);
JsonService btJson(&appBT, &json);
Motor motor;

#include <Servo.h>
#include <ServoEasing.hpp>

Servo _servo2;

Servo _servo4;


ServoEasing servo2(PCA9685_DEFAULT_ADDRESS);

ServoEasing servo4(PCA9685_DEFAULT_ADDRESS);

int ang1 = INIT_ANG1;
int ang2 = INIT_ANG2;
int ang3 = INIT_ANG3;
int ang4 = INIT_ANG4;
void attach_servos() {

  servo2.attach(PIN_SERVO_2, INIT_ANG2);
  
  servo4.attach(PIN_SERVO_4, INIT_ANG4);
  Serial.println("attach_servos");
}
void json_callback() {
  const char *pType = (const char *)json["type"];  // Ex: C1
  const char type0 = *pType;                       // Ex: C
  const char type1 = *(pType + 1);                 // Ex: 1

  Serial.print("type =");
  Serial.print(pType);
  Serial.print(", type0 =");
  Serial.print(type0);
  Serial.print(", type1 =");
  Serial.println(type1);

  if (type0 == 'M')  // App Controll Message
  {
    
    
    if (type1 == '6') {
       
      servo2.write(20);

      Serial.println("iukij");
    }
    if (type1 == '7') {
      servo2.write(130);

      
      Serial.println("sfrgre");
    }
    if (type1 == '2') {
      ang4 += 30;  // 增加1度
      servo4.write(ang4);
      Serial.println("oefie");
    }
    if (type1 == '3') {
      ang4 -= 30;  // 增加1度
      servo4.write(ang4);
      Serial.println("sifeji");
    }
  }

  if (type0 == 'C') {


    float mL = (float)(json["data"]["ml"]);
    float mR = (float)(json["data"]["mr"]);

    Serial.print("\tmL=");
    Serial.print(mL);
    Serial.print("\tmR=");
    Serial.println(mR);
    motor.drive2(mL, mR);
  }

  else if (type0 == 'R') {  // Response
  }
}



void setup() {
  Serial.begin(SR_BAUD_RATE);
  appBT.begin(BT_BAUD_RATE);
  Serial.println("Controller2 PCA9685:");

  servo2.InitializeAndCheckI2CConnection(&Serial);

  servo4.InitializeAndCheckI2CConnection(&Serial);

  attach_servos();

  servo2.startEaseTo(INIT_ANG2, SERVO_DEGREE_PER_SECOND, START_UPDATE_BY_INTERRUPT);  // Non blocking call
 
  servo4.startEaseTo(INIT_ANG4, SERVO_DEGREE_PER_SECOND, START_UPDATE_BY_INTERRUPT);  // Non blocking call
}

void loop() {
  btJson.listen(&json_callback);


  servo2.update();

  servo4.update();
}

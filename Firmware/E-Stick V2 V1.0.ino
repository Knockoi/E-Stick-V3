#include "DFRobot_BMI160.h"
#include <FastLED.h>
#include <BleMouse.h>
#include "OneButton.h"

#define MODE_TAP   5           // 模式切换按钮
#define LEFT_KEY   3           // 左侧自定义按钮
#define RIGHT_KEY  4           // 右侧自定义按钮
#define I2C_SDA    7           // SDA
#define I2C_SCL    6           // SCL

#define RGB_D       8          // WS2812B RGB LED引脚
#define NUM_LEDS    2          // LED數量
#define BRIGHTNESS  255        // LED亮度 (0-255之間)可以調整

#define X_AXIS_J   1           // 搖桿X轴
#define Y_AXIS_J   2           // 搖桿Y轴

#define CALIBRATION_DURATION 2000 

float X_AXIS_M = 0;
float Y_AXIS_M = 0;
float Z_AXIS_M = 0;

int MODE = 1;
int prevX = 0;
int prevY = 0;
int prevZ = 0;                  // 存儲鼠標前一個 X 、Z和 Y 位置的變量

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69;

BleMouse bleMouse;
CRGB leds[NUM_LEDS];
//---------------------------------------------------------------------------程序設定

void setup(){

  Serial.begin(115200);
  delay(100);

Serial.println("Starting BLE work!"); //藍芽開始使用
  bleMouse.begin();

  pinMode(LEFT_KEY, INPUT);  //左按鍵
  pinMode(RIGHT_KEY, INPUT); //右按鍵
  pinMode(MODE_TAP, INPUT);  //模式切換
  pinMode(X_AXIS_J, INPUT);  //搖桿X軸
  pinMode(Y_AXIS_J, INPUT);  //搖桿Y軸
  pinMode(RGB_D, OUTPUT);      //WS2812B

  FastLED.addLeds<WS2812B, RGB_D, RGB>(leds, NUM_LEDS);  // 初始化FastLED庫
  FastLED.setBrightness(BRIGHTNESS);                        // 設置亮度

  //init the hardware bmin160
  if (bmi160.softReset() != BMI160_OK){
    Serial.println("reset false");
    while(1);
  }

  //set and init the bmi160 i2c address
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK){
    Serial.println("init false");
    while(1);
  }
}
//---------------------------------------------------------------------------程序循環

void loop(){  
  //-------------------------------------------------------------------------陀螺儀、加速度數值
  int rslt;
  int16_t accelGyro[6]={0}; 
  int16_t gX, gY, gZ; // 單獨的陀螺儀數據
  float aX, aY, aZ; // 單獨的加速度數據

  // 從 BMI160 獲取陀螺儀和加速度數據
  // 參數 accelGyro 是存儲數據的指針
  rslt = bmi160.getAccelGyroData(accelGyro);

  if(rslt == 0){
    // 提取陀螺儀數據 (單位: 弧度/秒)
    gX = accelGyro[0] * 3.14 / 180.0;
    gY = accelGyro[1] * 3.14 / 180.0;
    gZ = accelGyro[2] * 3.14 / 180.0;

    // 提取加速度數據 (單位: g)
    aX = accelGyro[3] / 16384.0;
    aY = accelGyro[4] / 16384.0;
    aZ = accelGyro[5] / 16384.0;

    // 輸出陀螺儀數據
    Serial.print("GyroX: "); Serial.print(gZ); Serial.print("\t");
    Serial.print("GyroY: "); Serial.print(gY); Serial.print("\t");
    Serial.print("GyroZ: "); Serial.print(gX); Serial.println();

    // 輸出加速度數據
    Serial.print("AccelX: "); Serial.print(aX); Serial.print("\t");
    Serial.print("AccelY: "); Serial.print(aY); Serial.print("\t");
    Serial.print("AccelZ: "); Serial.print(aZ); Serial.println();
  } else {
    Serial.println("Error reading data from BMI160");
  }
  delay(100);

    // 根據加速度計值計算鼠標移動
  int deltaX = aX - prevX;
  int deltaY = aY - prevY;
  int deltaZ = aZ - prevZ;

  // 通過平均前一個和當前值來平滑鼠標移動
  int newX = prevX + (deltaX >> 1);
  int newY = prevY + (deltaY >> 1);
  int newZ = prevZ + (deltaZ >> 1);     // 更新鼠標位置Mouse.move(newX, newY, 0);
                                                       
  // 存儲前一個鼠標位置以供下一次迭代使用
  prevX = newX;
  prevY = newY;
  prevZ = newZ;
  //------------------------------------------------------------------------模式切換設定(預設5模式)
    if (digitalRead(MODE_TAP) == 1) {       // 如果偵測到按鍵按下
      delay(500);
      MODE ++; delay(500);                  // 模式變數 +1
    if (MODE>=5) {                          // 如果超過設定模式數量，那麼就回到 1 號模式
      MODE = 1;
      }
    }

  switch(MODE){
  //------------------------------------------------------------------------模式切換設定(模式1)
    case 1:                  
    Serial.println("MODE1");break;
     // 兩顆LED漸變為全亮
  for (int i = 0; i < 256; i++) {
    leds[0] = CRGB(i, 255, i);  // 第一顆LED為紅綠色漸變
    leds[1] = CRGB(i, 255, i);  // 第二顆LED為紅綠色漸變
    FastLED.show();
    delay(10);
  }

  // 兩顆LED常亮綠色
    leds[0] = CRGB(0, 255, 0);  // 第一顆LED為綠色
    leds[1] = CRGB(0, 255, 0);  // 第二顆LED為綠色
    FastLED.show();

  delay(3000);  // 等待3秒

  // 兩顆LED慢慢熄滅
  for (int i = 255; i >= 0; i--) {
    leds[0] = CRGB(i, 255, i);  // 第一顆LED緩慢熄滅
    leds[1] = CRGB(i, 255, i);  // 第二顆LED緩慢熄滅
    FastLED.show();
    delay(10);
  }

  // 將兩顆LED關閉
    leds[0] = CRGB::Black;  // 第一顆LED關閉
    leds[1] = CRGB::Black;  // 第二顆LED關閉
    FastLED.show();



















  //------------------------------------------------------------------------模式切換設定(模式2)
    case 2:
    Serial.println("MODE2");break;
    // 兩顆LED漸變為全亮
  for (int i = 0; i < 256; i++) {
    leds[0] = CRGB(i, 255, i);  // 第一顆LED為紅綠色漸變
    leds[1] = CRGB(i, 255, i);  // 第二顆LED為紅綠色漸變
    FastLED.show();
    delay(10);
  }

  // 兩顆LED常亮綠色
    leds[0] = CRGB(0, 255, 0);  // 第一顆LED為綠色
    leds[1] = CRGB(0, 255, 0);  // 第二顆LED為綠色
    FastLED.show();

  delay(3000);  // 等待3秒

  // 兩顆LED慢慢熄滅
  for (int i = 255; i >= 0; i--) {
    leds[0] = CRGB(i, 255, i);  // 第一顆LED緩慢熄滅
    leds[1] = CRGB(i, 255, i);  // 第二顆LED緩慢熄滅
    FastLED.show();
    delay(10);
  }

  // 將兩顆LED關閉
    leds[0] = CRGB::Black;  // 第一顆LED關閉
    leds[1] = CRGB::Black;  // 第二顆LED關閉
    FastLED.show();















  //------------------------------------------------------------------------模式切換設定(模式3)
    case 3:
    Serial.println("MODE3");break;
    // 兩顆LED漸變為全亮
  for (int i = 0; i < 256; i++) {
    leds[0] = CRGB(i, 255, i);  // 第一顆LED為紅綠色漸變
    leds[1] = CRGB(i, 255, i);  // 第二顆LED為紅綠色漸變
    FastLED.show();
    delay(10);
  }

  // 兩顆LED常亮綠色
    leds[0] = CRGB(0, 255, 0);  // 第一顆LED為綠色
    leds[1] = CRGB(0, 255, 0);  // 第二顆LED為綠色
    FastLED.show();

  delay(3000);  // 等待3秒

  // 兩顆LED慢慢熄滅
  for (int i = 255; i >= 0; i--) {
    leds[0] = CRGB(i, 255, i);  // 第一顆LED緩慢熄滅
    leds[1] = CRGB(i, 255, i);  // 第二顆LED緩慢熄滅
    FastLED.show();
    delay(10);
  }

  // 將兩顆LED關閉
    leds[0] = CRGB::Black;  // 第一顆LED關閉
    leds[1] = CRGB::Black;  // 第二顆LED關閉
    FastLED.show();















  //------------------------------------------------------------------------模式切換設定(模式4) 
    case 4:
    Serial.println("MODE4");break;
    // 兩顆LED漸變為全亮
  for (int i = 0; i < 256; i++) {
    leds[0] = CRGB(i, 255, i);  // 第一顆LED為紅綠色漸變
    leds[1] = CRGB(i, 255, i);  // 第二顆LED為紅綠色漸變
    FastLED.show();
    delay(10);
  }

  // 兩顆LED常亮綠色
    leds[0] = CRGB(0, 255, 0);  // 第一顆LED為綠色
    leds[1] = CRGB(0, 255, 0);  // 第二顆LED為綠色
    FastLED.show();

  delay(3000);  // 等待3秒

  // 兩顆LED慢慢熄滅
  for (int i = 255; i >= 0; i--) {
    leds[0] = CRGB(i, 255, i);  // 第一顆LED緩慢熄滅
    leds[1] = CRGB(i, 255, i);  // 第二顆LED緩慢熄滅
    FastLED.show();
    delay(10);
  }

  // 將兩顆LED關閉
    leds[0] = CRGB::Black;  // 第一顆LED關閉
    leds[1] = CRGB::Black;  // 第二顆LED關閉
    FastLED.show();


















  //------------------------------------------------------------------------模式切換設定(模式5)  
    case 5:
    Serial.println("MODE5");break;
    // 兩顆LED漸變為全亮
  for (int i = 0; i < 256; i++) {
    leds[0] = CRGB(i, 255, i);  // 第一顆LED為紅綠色漸變
    leds[1] = CRGB(i, 255, i);  // 第二顆LED為紅綠色漸變
    FastLED.show();
    delay(10);
  }

  // 兩顆LED常亮綠色
    leds[0] = CRGB(0, 255, 0);  // 第一顆LED為綠色
    leds[1] = CRGB(0, 255, 0);  // 第二顆LED為綠色
    FastLED.show();

  delay(3000);  // 等待3秒

  // 兩顆LED慢慢熄滅
  for (int i = 255; i >= 0; i--) {
    leds[0] = CRGB(i, 255, i);  // 第一顆LED緩慢熄滅
    leds[1] = CRGB(i, 255, i);  // 第二顆LED緩慢熄滅
    FastLED.show();
    delay(10);
  }

  // 將兩顆LED關閉
    leds[0] = CRGB::Black;  // 第一顆LED關閉
    leds[1] = CRGB::Black;  // 第二顆LED關閉
    FastLED.show();



















    

    default: Serial.println("ERROR");break;

  }

}


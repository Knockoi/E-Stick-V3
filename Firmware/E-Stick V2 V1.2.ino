#include <DFRobot_BMI160.h>
#include <FastLED.h>
#include <BleMouse.h>
#include <BleKeyboard.h>
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
#define Acce_SENS 2.0  // 陀螺儀靈敏度
#define JOYSTICK_SENS 1.0  // 搖桿靈敏度
#define JOYSTICK_MIN 0
#define JOYSTICK_MAX 1023


float X_AXIS_M = 0; // 鼠標移動的 X 軸
float Y_AXIS_M = 0; // 鼠標移動的 Y 軸
float Z_AXIS_M = 0; // 鼠標移動的 Z 軸

int MODE = 1; // 模式變數

int prevX = 0;
int prevY = 0;
int prevZ = 0;                  // 存儲鼠標前一個 X 、Z和 Y 位置的變量

int xValue = 0;
int yValue = 0;                 // 搖桿數值

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

FastLED.addLeds<WS2812, RGB_D>(leds, NUM_LEDS); // 初始化 WS2812B RGB LED

  pinMode(LEFT_KEY, INPUT);  //左按鍵
  pinMode(RIGHT_KEY, INPUT); //右按鍵
  pinMode(MODE_TAP, INPUT);  //模式切換
  pinMode(X_AXIS_J, INPUT);  //搖桿X軸
  pinMode(Y_AXIS_J, INPUT);  //搖桿Y軸
  

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
    aX = accelGyro[3] / 16384.0 * Acce_SENS;
    aY = accelGyro[4] / 16384.0 * Acce_SENS;
    aZ = accelGyro[5] / 16384.0 * Acce_SENS;

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
  prevX = newX;  // 更新前一個 X 位置
  prevY = newY;  // 更新前一個 Y 位置
  prevZ = newZ;  // 更新前一個 Z 位置
  
  //-------------------------------------------------------------------------搖桿數值

  int xValue = constrain(analogRead(X_AXIS_J) * JOYSTICK_SENS, JOYSTICK_MIN, JOYSTICK_MAX); // 通過搖桿值計算鼠標移動
  int yValue = constrain(analogRead(Y_AXIS_J) * JOYSTICK_SENS, JOYSTICK_MIN, JOYSTICK_MAX); // 通過搖桿值計算鼠標移動

  //------------------------------------------------------------------------模式切換設定(預設5模式)
    if (digitalRead(MODE_TAP) == 1) {       // 如果偵測到按鍵按下
      delay(500);
      MODE ++; delay(500);                  // 模式變數 +1
    if (MODE>=5) {                          // 如果超過設定模式數量，那麼就回到 1 號模式
      MODE = 1;
      }
    }

  switch(MODE){
  //------------------------------------------------------------------------模式切換設定(模式1)橘色(陀螺儀+滑鼠左右鍵)
    case 1:                  
    Serial.println("MODE1");
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(255, 165, 0); // 橘色
    }
    bleMouse.move(newX, newY); 

      if (bleMouse.isConnected()) {
    if (digitalRead(LEFT_KEY) == LOW) {  // 如果左按鍵被按下
      bleMouse.press(BLE_MOUSE_LEFT);    // 模擬左鍵按下
    } else {
      bleMouse.release(BLE_MOUSE_LEFT);  // 模擬左鍵釋放
    }

    if (digitalRead(RIGHT_KEY) == LOW) { // 如果右按鍵被按下
      bleMouse.press(BLE_MOUSE_RIGHT);   // 模擬右鍵按下
    } else {
      bleMouse.release(BLE_MOUSE_RIGHT); // 模擬右鍵釋放
    }
  }
  break;

  //------------------------------------------------------------------------模式切換設定(模式2)綠色(陀螺儀+切頁數)
    case 2:
    Serial.println("MODE2");
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(0, 255, 0); // 綠色
    }
    bleMouse.move(newX, newY);

      if (bleKeyboard.isConnected()) {
    if (digitalRead(LEFT_KEY) == LOW) {  // 如果左按鍵被按下
      bleKeyboard.write(KEY_LEFT_ARROW); // 模擬按下左箭頭
      delay(100);                        // 等待一段時間以防止連續輸入
    }

    if (digitalRead(RIGHT_KEY) == LOW) { // 如果右按鍵被按下
      bleKeyboard.write(KEY_RIGHT_ARROW); // 模擬按下右箭頭
      delay(100);                         // 等待一段時間以防止連續輸入
    }
  }
  break;

  //------------------------------------------------------------------------模式切換設定(模式3)藍色(搖桿+滑鼠左右鍵)
    case 3:
    Serial.println("MODE3");
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(0, 0, 255); // 藍色
    }
    bleMouse.move(xValue, yValue); 

      if (bleMouse.isConnected()) {
    if (digitalRead(LEFT_KEY) == LOW) {  // 如果左按鍵被按下
      bleMouse.press(BLE_MOUSE_LEFT);    // 模擬左鍵按下
    } else {
      bleMouse.release(BLE_MOUSE_LEFT);  // 模擬左鍵釋放
    }

    if (digitalRead(RIGHT_KEY) == LOW) { // 如果右按鍵被按下
      bleMouse.press(BLE_MOUSE_RIGHT);   // 模擬右鍵按下
    } else {
      bleMouse.release(BLE_MOUSE_RIGHT); // 模擬右鍵釋放
    }
  }
  break;

  //------------------------------------------------------------------------模式切換設定(模式4)白色(搖桿+切頁數)
    case 4:
    Serial.println("MODE4");
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(255, 255, 255); // 白色
    }
    bleMouse.move(xValue, yValue);

      if (bleKeyboard.isConnected()) {
    if (digitalRead(LEFT_KEY) == LOW) {  // 如果左按鍵被按下
      bleKeyboard.write(KEY_LEFT_ARROW); // 模擬按下左箭頭
      delay(100);                        // 等待一段時間以防止連續輸入
    }

    if (digitalRead(RIGHT_KEY) == LOW) { // 如果右按鍵被按下
      bleKeyboard.write(KEY_RIGHT_ARROW); // 模擬按下右箭頭
      delay(100);                         // 等待一段時間以防止連續輸入
    }
  }
  break;

  //------------------------------------------------------------------------模式切換設定(模式5)紅色(遊戲模式)  
    case 5:
    Serial.println("MODE5");
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(255, 0, 0); // 紅色
    }

    break;

    default: Serial.println("ERROR");break;

  }

}


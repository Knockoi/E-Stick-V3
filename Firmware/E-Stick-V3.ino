#include <Wire.h>
#include "SparkFun_BMI270_Arduino_Library.h"                 //https://github.com/sparkfun/SparkFun_BMI270_Arduino_Library
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>    //https://github.com/sparkfun/SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library
#include <BleConnectionStatus.h>                             //https://github.com/Mystfit/ESP32-BLE-CompositeHID
#include <BleCompositeHID.h>                                 //..
#include <KeyboardDevice.h>                                  //..
#include <MouseDevice.h>                                     //..
#include <Adafruit_NeoPixel.h>                               //https://github.com/adafruit/Adafruit_NeoPixel
#include <driver/rtc_io.h>                       
#include <PushButton.h>                                      //https://github.com/pololu/pushbutton-arduino


#define MODE_TAP   39          // 模式切換按鈕  
#define LEFT_KEY   37          // 左侧自定义按钮
#define RIGHT_KEY  38          // 右侧自定义按钮
#define I2C_SDA    21          // SDA
#define I2C_SCL    22          // SCL
#define Razer      23          // 雷射引脚          
#define Motor      25          // 震動馬達引脚


#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex


#define X_AXIS_J   34          // 搖桿X轴
#define Y_AXIS_J   35          // 搖桿Y轴

#define CALIBRATION_DURATION 2000 
#define Acce_SENS 2.0          // 陀螺儀靈敏度

double JOYSTICK_SENS = 1.0;      // 搖桿靈敏度
#define JOYSTICK_MIN 0
#define JOYSTICK_MAX 1024

#define RGB1      26  // 第1條燈帶連接的輸入引腳
#define RGB_NUM1  2   // 第1條燈帶的 LED 數量
#define RGB2      19  // 第2條燈帶連接的輸入引腳
#define RGB_NUM2  2   // 第2條燈帶的 LED 數量

Adafruit_NeoPixel strip1(RGB_NUM1, RGB1, NEO_GRB + NEO_KHZ800); // 創建第一條燈帶對象
Adafruit_NeoPixel strip2(RGB_NUM1, RGB1, NEO_GRB + NEO_KHZ800); // 創建第二條燈帶對象

void BMI270DATA(void *pvParameters);
void MAX17048DATA(void *pvParameters);
void BLEHID(void *pvParameters);
void NVM(void *pvParameters);
void SWM(void *pvParameters);
void SLEEP(void *pvParameters);
void SPOT(void *pvParameters);


int batteryLevel = 100;
int Calibration = 0;

int X_AXIS_M = 0; // 鼠標移動的 X 軸
int Y_AXIS_M = 0; // 鼠標移動的 Y 軸
int Z_AXIS_M = 0; // 鼠標移動的 Z 軸

int TAP = 1 ;  //模式切換方式1=按鈕2=搖桿
int MODE = 1; // 模式變數
int Mouse_Pointer = 0; // 滑鼠指針

float prevX = 0;
float prevY = 0;
float prevZ = 0;                  // 存儲鼠標前一個 X 、Z和 Y 位置的變量

float xValue = 0;
float yValue = 0;                 // 搖桿數值

SFE_MAX1704X lipo(MAX1704X_MAX17048); // Create a MAX17048
// 建立一個新的感測器對象
BMI270 imu;

//I2C地址選擇
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR;  // 0x68    
//uint8_t i2cAddress = BMI2_I2C_SEC_ADDR; // 0x69

// 中斷指令腳位
int interruptPin = 18;

// 判斷中斷何時發生的標誌
volatile bool interruptOccurred = false;

KeyboardDevice* keyboard; 
MouseDevice* mouse;
BleCompositeHID compositeHID("EVO-Stick V3", "OOC.tw", 100);

unsigned long startTime = 0; // 變數用於存儲計時開始的時間

PushButton myButton(MODE_TAP);       // 模式切换按钮

const unsigned long maxButtonHoldTime = 40000;  // 最大按鈕按住時間（40秒）

//-------------------------------------------------------------------------------------------------------設定
void setup() {

  Serial.begin(115200);

  Wire.begin();

  pinMode(LEFT_KEY, INPUT);  //左按鍵
  pinMode(RIGHT_KEY, INPUT); //右按鍵
  pinMode(MODE_TAP, INPUT);  //模式切換
  pinMode(X_AXIS_J, INPUT);  //搖桿X軸
  pinMode(Y_AXIS_J, INPUT);  //搖桿Y軸

  myButton.setHoldTime(500); // Set time (ms) for a hold-event to trigger [default: 500]
  myButton.setDoubleClickTime(500);
  
  strip1.begin();  // 初始化第一條燈帶
  strip1.show();   // 顯示第一條燈帶的初始狀態

  strip2.begin();  // 初始化第二條燈帶
  strip2.show();   // 顯示第二條燈帶的初始狀態


  xTaskCreate(BMI270DATA,"BMI270", 1024, NULL, 6, NULL);     
  xTaskCreate(MAX17048DATA,"MAX17048", 1024, NULL, 5, NULL);
  xTaskCreate(BLEHID,"BLEHID", 4096, NULL, 8, NULL);
  xTaskCreate(NVM,"NVM", 1024, NULL, 6, NULL);
  xTaskCreate(SWM,"SWM", 1024, NULL, 10, NULL);
  xTaskCreate(SPOT,"SPOT", 1024, NULL, 9, NULL);
  


 //-----------------------------------------------------------------------------------------------------------電量顯示
  while (!Serial); //等待使用者開啟終端
  Serial.println(F("MAX17048 Example"));
  lipo.enableDebugging(); 

  // 設定 MAX17048 LiPo 電量計：
  if (lipo.begin() == false)    // 使用預設有線連接埠連接到MAX17048
  {
    Serial.println(F("MAX17048 not detected. Please check wiring. Freezing."));
    while (1);
  }

  // 因為我們可以，所以讓我們重設 MAX17048
  Serial.println(F("Resetting the MAX17048..."));
  vTaskDelay(1000);    // 給它時間讓它恢復正常

  // 讀取並列印重設指示符
  Serial.print(F("Reset Indicator was: "));
  bool RI = lipo.isReset(true);    // 讀取RI標誌，若設定則自動清除
  Serial.println(RI);              // 列印 RI
                                   // 如果 RI 已設置，請檢查它現在是否已清除
  if (RI){
    Serial.print(F("Reset Indicator is now: "));
    RI = lipo.isReset();   // 讀取RI標誌
    Serial.println(RI);    // 列印 RI
  }
  // 讀取並列印裝置ID
  Serial.print(F("Device ID: 0x"));
  uint8_t id = lipo.getID();              //讀取裝置ID
  if (id < 0x10) Serial.print(F("0"));    // 若需要則列印前導零
  Serial.println(id, HEX);                // 將 ID 列印為十六進位

  // 讀取並列印裝置版本
  Serial.print(F("Device version: 0x"));
  uint8_t ver = lipo.getVersion();         // 讀取裝置版本
  if (ver < 0x10) Serial.print(F("0"));    // 若需要則列印前導零
  Serial.println(ver, HEX);                // 將版本列印為十六進位

  // 讀取並列印電池閾值
  Serial.print(F("Battery empty threshold is currently: "));
  Serial.print(lipo.getThreshold());
  Serial.println(F("%"));

	// 我們可以設定一個中斷，以便在電池 SoC 電量過低時發出警報。我們可以在 1% 到 32% 之間的任何位置發出警報：
	lipo.setThreshold(20);    // 將警報閾值設定為 20%。

  // 讀取並列印電池電量耗盡閾值
  Serial.print(F("Battery empty threshold is now: "));
  Serial.print(lipo.getThreshold());
  Serial.println(F("%"));
  // 讀取並列印高電壓閾值
  Serial.print(F("High voltage threshold is currently: "));
  float highVoltage = ((float)lipo.getVALRTMax()) * 0.02;    // 1 LSb 為 20mV。轉換為伏特。
  Serial.print(highVoltage, 2);
  Serial.println(F("V"));

  // 設定高壓閾值
  lipo.setVALRTMax((float)4.2); // 設定高電壓閾值（伏特）

  // 讀取並列印高電壓閾值
  Serial.print(F("High voltage threshold is now: "));
  highVoltage = ((float)lipo.getVALRTMax()) * 0.02;    // 1 LSb 為 20mV。轉換為伏特。
  Serial.print(highVoltage, 2);
  Serial.println(F("V"));

 // 讀取並列印低電壓閾值
  Serial.print(F("Low voltage threshold is currently: "));
  float lowVoltage = ((float)lipo.getVALRTMin()) * 0.02;    // 1 LSb 為 20mV。轉換為伏特。
  Serial.print(lowVoltage, 2);
  Serial.println(F("V"));

  // 設定低電壓閾值
  lipo.setVALRTMin((float)3.7); // 設定低電壓閾值（伏特）

  // 讀取並列印低電壓閾值
  Serial.print(F("Low voltage threshold is now: "));
  lowVoltage = ((float)lipo.getVALRTMin()) * 0.02;    // 1 LSb 為 20mV。轉換為伏特。
  Serial.print(lowVoltage, 2);
  Serial.println(F("V"));

  // 啟用狀態變更警報
  Serial.print(F("Enabling the 1% State Of Change alert: "));
  if (lipo.enableSOCAlert()){
    Serial.println(F("success."));
  }
  else{
    Serial.println(F("FAILED!"));
  }
  
  // 讀取並列印 HIBRT 活動閾值
  Serial.print(F("Hibernate active threshold is: "));
  float actThr = ((float)lipo.getHIBRTActThr()) * 0.00125;    // 1 LSb 為 20mV。轉換為伏特。
  Serial.print(actThr, 5);
  Serial.println(F("V"));

  // 讀取並列印 HIBRT 休眠閾值
  Serial.print(F("Hibernate hibernate threshold is: "));
  float hibThr = ((float)lipo.getHIBRTHibThr()) * 0.208;    // 1 LSb 為 20mV。轉換為伏特。
  Serial.print(hibThr, 3);
  Serial.println(F("%/h"));
 //-----------------------------------------------------------------------------------------------------------加速度計
 Serial.println("BMI270 Example 9 - Low Power");
 // 檢查感測器是否連接並初始化
 // 位址是可選的（預設為0x68）
  while (imu.beginI2C(i2cAddress) != BMI2_OK) {
    
  Serial.println("Error: BMI270 not connected, check wiring and I2C address!");   // 未連接，通知用戶
  vTaskDelay(1000);                                                               //等一下看看連線是否建立
  }

  Serial.println("BMI270 connected!");
  
  imu.setAccelPowerMode(BMI2_POWER_OPT_MODE);                       // 我們可以透過設定各個電源模式來降低功耗
  imu.setGyroPowerMode(BMI2_POWER_OPT_MODE, BMI2_POWER_OPT_MODE);   // 感測器進入功率最佳化模式

  imu.setAccelODR(BMI2_ACC_ODR_25HZ);    // 較低的 ODR 值會減少耗電量。以確保感測器
  imu.setGyroODR(BMI2_GYR_ODR_25HZ);     // 已同步，我們將選擇兩個感測器都支援的最小 ODR

  // BMI270 具有先進的省電模式，可用於最大限度地減少目前的耗電。請注意，這有一些限制，請參閱
  imu.enableAdvancedPowerSave();

  // 配置感應器後，我們可以透過以下方式進入掛起模式
  // 停用加速度計和陀螺儀。這可以降低BMI270的電流消耗降至3.5uA
  imu.disableFeature(BMI2_ACCEL);
  imu.disableFeature(BMI2_GYRO);
 //-----------------------------------------------------------------------------------------------------------中斷設定
  imu.mapInterruptToPin(BMI2_DRDY_INT, BMI2_INT1);

  bmi2_int_pin_config intPinConfig;
    intPinConfig.pin_type = BMI2_INT1;
    intPinConfig.int_latch = BMI2_INT_NON_LATCH;
    intPinConfig.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    intPinConfig.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    intPinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    intPinConfig.pin_cfg[0].input_en = BMI2_INT_INPUT_ENABLE;
    imu.setInterruptPinConfig(intPinConfig);

     // Setup interrupt handler
   interruptOccurred = true;
 //-----------------------------------------------------------------------------------------------------------BLE-HID
  Serial.println("Starting BLE work!"); //藍芽開始使用

  // 初始化 BLE HID 裝置
  KeyboardConfiguration keyboardConfig;
  keyboardConfig.setAutoReport(false);
  keyboard = new KeyboardDevice(keyboardConfig);

  // 初始化 BLE 鼠標裝置
  MouseConfiguration mouseConfig;
  mouseConfig.setAutoReport(false);
  mouse = new MouseDevice(mouseConfig);

  // 將鍵盤和鼠標裝置添加到複合 HID 裝置中
  compositeHID.addDevice(keyboard);
  compositeHID.addDevice(mouse);

  // 開始 BLE 裝置
  compositeHID.begin();


}
//-------------------------------------------------------------------------------------------------------滑鼠點擊
void Pointer(){

 myButton.update();

 if(myButton.isDoubleClicked()){
  Mouse_Pointer ++;
  } 

  if(Mouse_Pointer == 1){
    mouse->mouseMove(prevX, prevY);
    mouse->sendMouseReport();
  } 
  if(Mouse_Pointer == 2){
    mouse->mouseMove(xValue, yValue);
    mouse->sendMouseReport();
  }
  
  if(Mouse_Pointer>2){
    Mouse_Pointer = 0;
  }

}
//-------------------------------------------------------------------------------------------------------滑鼠點擊
void BLE_MOUSE_KEY(){

 if(analogRead(LEFT_KEY) != HIGH){
    mouse->mousePress(MOUSE_LOGICAL_LEFT_BUTTON);
    strip2.setPixelColor(0, strip2.Color(100, 100, 100));  
    vTaskDelay(100);
      } else {
    mouse->mouseRelease(MOUSE_LOGICAL_LEFT_BUTTON);
    strip2.setPixelColor(0, strip2.Color(0, 0, 0)); 
      }
    if(analogRead(LEFT_KEY) != HIGH){
    mouse->mousePress(MOUSE_LOGICAL_RIGHT_BUTTON);
    strip2.setPixelColor(1, strip2.Color(100, 100, 100));
    vTaskDelay(100);
      } else {
    mouse->mouseRelease(MOUSE_LOGICAL_RIGHT_BUTTON);
    strip2.setPixelColor(1, strip2.Color(0, 0, 0));
      }

}
//-------------------------------------------------------------------------------------------------------左右
void BLE_KEYBOARD_POINT(){

    if(analogRead(LEFT_KEY) != HIGH){
    keyboard->keyPress(KEY_LEFT);
    strip2.setPixelColor(0, strip2.Color(100, 100, 100));
    vTaskDelay(100);
      } else {
    keyboard->keyRelease(KEY_LEFT);
    strip2.setPixelColor(0, strip2.Color(0, 0, 0)); 
      }
    if(analogRead(RIGHT_KEY) != HIGH){
    keyboard->keyPress(KEY_RIGHT);
    strip2.setPixelColor(1, strip2.Color(100, 100, 100));
    vTaskDelay(100);
      } else {
    keyboard->keyRelease(KEY_RIGHT);
    strip2.setPixelColor(1, strip2.Color(0, 0, 0)); 
      }

}
//-------------------------------------------------------------------------------------------------------上下
void BLE_KEYBOARD_POINT2(){

    if(analogRead(LEFT_KEY) != HIGH){
    keyboard->keyPress(KEY_UP);
    strip2.setPixelColor(0, strip2.Color(100, 100, 100));
    vTaskDelay(100);
      } else {
    keyboard->keyRelease(KEY_UP);
    strip2.setPixelColor(0, strip2.Color(0, 0, 0)); 
      }
    if(analogRead(RIGHT_KEY) != HIGH){
    keyboard->keyPress(KEY_DOWN);
    strip2.setPixelColor(1, strip2.Color(100, 100, 100));
    vTaskDelay(100);
      } else {
    keyboard->keyRelease(KEY_DOWN);
    strip2.setPixelColor(1, strip2.Color(0, 0, 0)); 
      }

}
//-------------------------------------------------------------------------------------------------------虛擬SPOTLIGHT
void BLE_KEYBOARD_SPOTLIGHT(){

    if(myButton.isHeld()) {    // 如果偵測到按鍵按下
    keyboard->keyPress(KEY_MOD_LCTRL);
    keyboard->keyPress(KEY_Q);
    } else {
    keyboard->keyRelease(KEY_MOD_LCTRL);
    keyboard->keyRelease(KEY_Q);
    vTaskDelay(100);
    }

}
//-------------------------------------------------------------------------------------------------------音量調整
void BLE_KEYBOARD_VOICE(){

    if(analogRead(LEFT_KEY) != HIGH){
    keyboard->keyPress(KEY_VOLUMEDOWN);
    strip2.setPixelColor(0, strip2.Color(100, 100, 100));
    vTaskDelay(100);
      } else {
    keyboard->keyRelease(KEY_VOLUMEDOWN);
    strip2.setPixelColor(0, strip2.Color(0, 0, 0)); 
      }
    if(analogRead(RIGHT_KEY) != HIGH){
    keyboard->keyPress(KEY_VOLUMEUP);
    strip2.setPixelColor(1, strip2.Color(100, 100, 100));
    vTaskDelay(100);
      } else {
    keyboard->keyRelease(KEY_VOLUMEUP);
    strip2.setPixelColor(1, strip2.Color(0, 0, 0)); 
      }
}


//-------------------------------------------------------------------------------------------------------主迴圈
void loop() {
 //not for use
}


//-------------------------------------------------------------------------------------------------------切換模式切換方式
void SWM(void *pvParameters){
  
 myButton.update();

 switch(TAP){
  case 1:
   if(myButton.isClicked()) {    // 如果偵測到按鍵按下
      MODE ++; 
    vTaskDelay(50);                    // 模式變數 +1
    if (MODE>=5) {                     // 如果超過設定模式數量，那麼就回到 1 號模式
      MODE = 1;
      }
    }
   break;

  case 2:
  if (xValue < JOYSTICK_MIN + 50){      // 如果搖桿滑到最左边
   MODE--;                              // MODE 减一
  if (MODE < 1){                       // 如果 MODE 小于 1，回到最大模式数
    MODE = 5;                           // 设置最大模式数
  }
  } else 
  if (xValue > JOYSTICK_MAX - 50) { // 如果搖桿滑到最右边
  MODE++;                              // MODE 加一
  if (MODE > 5){                       // 如果 MODE 大于最大模式数
    MODE = 1;                           // 回到 1 号模式
  }
  }
  break;
  }
   
}

//-------------------------------------------------------------------------------------------------------SPOTLIGHT的縮放使用設定
void SPOT(void *pvParameters){
   
   myButton.update();

 if (myButton.isDoubleClicked()){
   Mouse_Pointer ++;
    }
 if(Mouse_Pointer < 2){
  Mouse_Pointer == 1;
 }

  switch(Mouse_Pointer){
  case 1:
  while (1)
  {
    mouse->mouseMove(prevX, prevY);
    mouse->sendMouseReport();
  }
  
  case 2:
  while (1)
  {
   mouse->mouseMove(xValue, yValue);
    mouse->sendMouseReport();
  }

  default: 
    Serial.println("Mouse Error Code");
    break;
  }

}

//-------------------------------------------------------------------------------------------------------藍牙HID
void BLEHID(void *pvParameters){
  while(1){
  //-----------------------------------------------------------------------------------------------------滑鼠移動
  int16_t gX, gY, gZ; // 單獨的陀螺儀數據
  int16_t aX, aY, aZ; // 單獨的加速度數據

  aX = imu.data.accelX * Acce_SENS;
  aY = imu.data.accelY * Acce_SENS;
  aZ = imu.data.accelZ * Acce_SENS;

  // 根據加速度計值計算鼠標移動
  int16_t deltaX = aX - prevX;
  int16_t deltaY = aY - prevY;
  int16_t deltaZ = aZ - prevZ;

  // 通過平均前一個和當前值來平滑鼠標移動
  int16_t newX = prevX + (deltaX >> 1);
  int16_t newY = prevY + (deltaY >> 1);
  int16_t newZ = prevZ + (deltaZ >> 1);     // 更新鼠標位置Mouse.move(newX, newY);
                                                       
  // 存儲前一個鼠標位置以供下一次迭代使用
  prevX = newX;  // 更新前一個 X 位置
  prevY = newY;  // 更新前一個 Y 位置
  prevZ = newZ;  // 更新前一個 Z 位置
  
  //-------------------------------------------------------------------------搖桿數值

  float xValue = constrain(analogRead(X_AXIS_J) * JOYSTICK_SENS, JOYSTICK_MIN, JOYSTICK_MAX); // 通過搖桿值計算鼠標移動
  float yValue = constrain(analogRead(Y_AXIS_J) * JOYSTICK_SENS, JOYSTICK_MIN, JOYSTICK_MAX); // 通過搖桿值計算鼠標移動

  //-------------------------------------------------------------------------模式切換
 
  switch(MODE){
  //------------------------------------------------------------------------模式切換設定(模式1)橘色(滑鼠左右鍵)                 
  case 1:                  
    Serial.println("MODE1");
   for (int brightness = 0; brightness <= 255; brightness += 5) {
    strip1.setPixelColor(0, strip1.Color(brightness, brightness / 2, 0));  // 控制第一顆 LED (橘色)
    strip1.setPixelColor(1, strip1.Color(brightness, brightness / 2, 0));  // 控制第二顆 LED (橘色)
    strip1.show();  // 顯示設置的顏色
    break;
   }
    vTaskDelay(2500);  // 等待 2.5 秒
    strip1.clear();  // 清除第一條燈帶的顏色
    strip1.show();   // 顯示燈帶熄滅

   if (digitalRead(interruptPin) == HIGH) {
    // 当 interruptPin 为高电平时，执行以下代码
    digitalWrite(interruptPin, LOW); // 将 interruptPin 设为低电平
   }
   if(compositeHID.isConnected()){

    BLE_MOUSE_KEY();

   } 
   break;
  //------------------------------------------------------------------------模式切換設定(模式2)綠色(切頁數)
  case 2:                  
    Serial.println("MODE2");
   for (int brightness = 0; brightness <= 255; brightness += 5) {
    strip1.setPixelColor(0, strip1.Color(0, brightness, 0));  // 控制第一顆 LED (綠色)
    strip1.setPixelColor(1, strip1.Color(0, brightness, 0));  // 控制第二顆 LED (綠色)
    strip1.show();  // 顯示設置的顏色
    break;
   }
    vTaskDelay(2500);  // 等待 2.5 秒
    strip1.clear();  // 清除第一條燈帶的顏色
    strip1.show();   // 顯示燈帶熄滅

   if (digitalRead(interruptPin) == HIGH) {
    // 当 interruptPin 为高电平时，执行以下代码
    digitalWrite(interruptPin, LOW); // 将 interruptPin 设为低电平
   }

   if(compositeHID.isConnected()){

    BLE_KEYBOARD_POINT();
   
   }
   break;
  //------------------------------------------------------------------------模式切換設定(模式3)藍色(音量控制)
  case 3:                  
    Serial.println("MODE3");
   for (int brightness = 0; brightness <= 255; brightness += 5) {
    strip1.setPixelColor(0, strip1.Color(brightness, 0, 0));  // 控制第一顆 LED (紅色)
    strip1.setPixelColor(1, strip1.Color(brightness, 0, 0));  // 控制第二顆 LED (紅色)
    strip1.show();  // 顯示設置的顏色
    break;
   }
    vTaskDelay(2500);  // 等待 2.5 秒
    strip1.clear();  // 清除第一條燈帶的顏色
    strip1.show();   // 顯示燈帶熄滅

   if(compositeHID.isConnected()){

    BLE_KEYBOARD_VOICE();

   } 
   break;
  //------------------------------------------------------------------------模式切換設定(模式4)白色(聚光燈設定)
  case 4:                  
    Serial.println("MODE4");
  for (int brightness = 0; brightness <= 255; brightness += 5) {
    strip1.setPixelColor(0, strip1.Color(0, brightness/2, brightness));  // 控制第一顆 LED (紅色)
    strip1.setPixelColor(1, strip1.Color(0, brightness/2, brightness));  // 控制第二顆 LED (紅色)
    strip1.show();  // 顯示設置的顏色
    break;
   }
    vTaskDelay(2500);  // 等待 2.5 秒
    strip1.clear();  // 清除第一條燈帶的顏色
    strip1.show();   // 顯示燈帶熄滅


    if(analogRead(LEFT_KEY) != HIGH){
    keyboard->keyPress(KEY_X);
    vTaskDelay(100);
    } else {
    keyboard->keyRelease(KEY_X);
    }
    if(analogRead(RIGHT_KEY) != HIGH){
    keyboard->keyPress(KEY_Z);
    vTaskDelay(100);
    } else {
    keyboard->keyRelease(KEY_Z);
    }


  break;
  //------------------------------------------------------------------------模式切換設定(錯誤)
  default: 
    Serial.println("Unknow Error Code");
    break;
  }
 }
}

//-------------------------------------------------------------------------------------------------------陀螺儀數據
void BMI270DATA(void *pvParameters){

  while(1){
      // 等待下一次測量。對於低功耗應用，這可能是透過將微控制器設定為睡眠狀態來替換 
  vTaskDelay(1000);

  // 重新啟用感測器以獲得新的測量結果。目前消耗取決於啟用了哪些感測器：
  // 
  // 僅加速度計 - 低至 4uA
  // 僅陀螺儀 - 低至 400uA
  // 加速計和陀螺儀 - 低至 420uA
  imu.enableFeature(BMI2_ACCEL);
  imu.enableFeature(BMI2_GYRO);

  // 等待測量完成。這由 DRDY 指示
  // 在感測器狀態暫存器中設定位
  bool accelDRDY = false;
  bool gyroDRDY = false;
  while ((accelDRDY == false) || (gyroDRDY == false)) {
    // 取得感測器狀態
    uint8_t status = 0;
    imu.getStatus(&status);

    // 更新資料 DRDY 標誌
    if (status & BMI2_DRDY_ACC) {
      accelDRDY = true;
    }
    if (status & BMI2_DRDY_GYR) {
      gyroDRDY = true;
    }
  }

  // 從感測器取得測量值。在訪問之前必須呼叫此方法
  // 感測器數據，否則永遠不會更新
  imu.getSensorData();

 // 透過停用感應器返回掛起模式。這必須完成
 // 呼叫 getSensorData() 後，否則資料將為零
  imu.disableFeature(BMI2_ACCEL);
  imu.disableFeature(BMI2_GYRO);

  // 列印加速度數據
  Serial.print("Acceleration in g's");
  Serial.print("\t");
  Serial.print("X: ");
  Serial.print(imu.data.accelX, 3);
  Serial.print("\t");
  Serial.print("Y: ");
  Serial.print(imu.data.accelY, 3);
  Serial.print("\t");
  Serial.print("Z: ");
  Serial.print(imu.data.accelZ, 3);

  Serial.print("\t");

  // 列印陀螺儀數據
  Serial.print("Rotation in deg/sec");
  Serial.print("\t");
  Serial.print("X: ");
  Serial.print(imu.data.gyroX, 3);
  Serial.print("\t");
  Serial.print("Y: ");
  Serial.print(imu.data.gyroY, 3);
  Serial.print("\t");
  Serial.print("Z: ");
  Serial.println(imu.data.gyroZ, 3);
  }
 
}

//-------------------------------------------------------------------------------------------------------電量顯示
void MAX17048DATA(void *pvParameters){
  while(1){
   if (compositeHID.isConnected()){
    compositeHID.setBatteryLevel(lipo.getSOC());
    vTaskDelay(5000);
   }
  Serial.print("Voltage: ");
  Serial.print(lipo.getVoltage()); // 列印電池電壓
  Serial.print("V");

  Serial.print(" Percentage: ");
  Serial.print(lipo.getSOC(), 2); // 列印電池充電狀態，保留 2 位小數
  Serial.print("%");

  Serial.print(" Change Rate: ");
  Serial.print(lipo.getChangeRate(), 2);// 列印電池更換率，保留 2 位小數
  Serial.print("%/hr");

  Serial.print(" Alert: ");
  Serial.print(lipo.getAlert()); // 列印通用警報標誌

  Serial.print(" Voltage High Alert: ");
  Serial.print(lipo.isVoltageHigh()); // 列印警報標誌

  Serial.print(" Voltage Low Alert: ");
  Serial.print(lipo.isVoltageLow()); // 列印警報標誌

  Serial.print(" Empty Alert: ");
  Serial.print(lipo.isLow()); // 列印警報標誌

  Serial.print(" SOC 1% Change Alert: ");
  Serial.print(lipo.isChange()); // 列印警報標誌

  Serial.print(" Hibernating: ");
  Serial.print(lipo.isHibernating()); // 列印警報標誌
  
  Serial.println();

  vTaskDelay(500);
  }
}
 
//-------------------------------------------------------------------------------------------------------校準
void NVM(void *pvParameters){
     
  if(Calibration == 1 && imu.beginI2C(i2cAddress) == BMI2_OK){
    Serial.println("Place the sensor on a flat surface and leave it stationary.");
    Serial.println("Enter any key to begin calibration.");

    // Throw away any previous inputs
    while(Serial.available() != 0) {Serial.read();}
    // Wait for user input
    while(Serial.available() == 0) {}

    Serial.println();
    Serial.println("Average sensor values before calibration:");
    printAverageSensorValues();
    Serial.println();

    // Perform component retrim for the gyroscope. According to the datasheet,
    // the gyroscope has a typical error of 2%, but running the CRT can reduce
    // that error to 0.4%
    Serial.println("Performing component retrimming...");
    imu.performComponentRetrim();

    // Perform offset calibration for both the accelerometer and IMU. This will
    // automatically determine the offset of each axis of each sensor, and
    // that offset will be subtracted from future measurements. Note that the
    // offset resolution is limited for each sensor:
    // 
    // Accelerometer offset resolution: 0.0039 g
    // Gyroscope offset resolution: 0.061 deg/sec
    Serial.println("Performing acclerometer offset calibration...");
    imu.performAccelOffsetCalibration(BMI2_GRAVITY_POS_Z);
    Serial.println("Performing gyroscope offset calibration...");
    imu.performGyroOffsetCalibration();
    
    Serial.println();
    Serial.println("Calibration complete!");
    Serial.println();
    Serial.println("Average sensor values after calibration:");
    printAverageSensorValues();
    Serial.println();

    Serial.println("These calibration values can be stored in the sensor's non-volatile memory (NVM).");
    Serial.println("Would you like to save these values to the NVM? If so, enter 'Y'");

    // Throw away any previous inputs
    while(Serial.available() != 0) {Serial.read();}
    // Wait for user input
    while(Serial.available() == 0) {}

    // Check to see if user wants to save values to NVM
    if(Serial.read() == 'Y')
    {
        Serial.println();
        Serial.println("!!! WARNING !!! WARNING !!! WARNING !!! WARNING !!! WARNING !!!");
        Serial.println();
        Serial.println("The BMI270's NVM only supports 14 write cycles TOTAL!");
        Serial.println("Are you sure you want to save to the NVM? If so, enter 'Y' again");

      // Throw away any previous inputs
      while(Serial.available() != 0) {Serial.read();}
      // Wait for user input
      while(Serial.available() == 0) {}

      // Check to see if user *really* wants to save values to NVM
      if(Serial.read() == 'Y'){
            // Save NVM contents
            int8_t err = imu.saveNVM();
            
            // Check to see if the NVM saved successfully
            if(err == BMI2_OK)
            {
                Serial.println();
                Serial.println("Calibration values have been saved to the NVM!");
            }
            else
            {
                Serial.print("Error saving to NVM, error code: ");
                Serial.println(err);
            }
        }
    }

    Serial.println();
    Serial.println("done!");
  }
}

//-------------------------------------------------------------------------------------------------------感測器進行多次取樣
void printAverageSensorValues(){
    // Variables to store the sum of each sensor axis
    float accXSum = 0;
    float accYSum = 0;
    float accZSum = 0;

    float gyrXSum = 0;
    float gyrYSum = 0;
    float gyrZSum = 0;

    // Collect 50 measurements at 50Hz
    int numSamples = 50;
    for(int i = 0; i < numSamples; i++)
    {
        // Get measurements from the sensor
        imu.getSensorData();

        // Add this measurement to the running total
        accXSum += imu.data.accelX;
        accYSum += imu.data.accelY;
        accZSum += imu.data.accelZ;

        gyrXSum += imu.data.gyroX;
        gyrYSum += imu.data.gyroY;
        gyrZSum += imu.data.gyroZ;

        // Wait for next measurement
       vTaskDelay(20);
    }
}

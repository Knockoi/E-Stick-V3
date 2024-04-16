#include <Wire.h>
#include <FastLED.h>
#include <NimBLEDevice.h>
#include <bmi160.h>
#include <Mouse.h>
#include <Keyboard.h>
#include <AnalogJoystick.h>

#define MODE_SWITCH_PIN   5  // 模式切换按钮
#define LEFT_BUTTON_PIN   3  // 左侧自定义按钮
#define RIGHT_BUTTON_PIN  4  // 右侧自定义按钮
#define LED_PIN           8  // WS2812B RGB LED引脚
#define X_AXIS_PIN        1  // 搖桿X轴
#define Y_AXIS_PIN        2  // 搖桿Y轴
#define I2C_SDA           7  // SDA
#define I2C_SCL           6  // SCL

#define BMI160_ADDR       0x68  // BMI160 的 I2C 地址

#define NUM_LEDS          2  // RGB LED数量
CRGB leds[NUM_LEDS];

BMI160 mySensor;  // 创建一个 BMI160 对象
AnalogJoystick joystick(X_AXIS_PIN, Y_AXIS_PIN);

const int BUTTON_THRESHOLD = 1000;
int currentMode = 0;
CRGB currentColor;

unsigned long previousMillis = 0;
const long interval = 50;  // 每隔 50 毫秒更新一次传感器数据

void setup() {
    Serial.begin(115200);
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
  
    // 初始化 BMI160 传感器
    if (!mySensor.begin(BMI160_ADDR, Wire)) {
        Serial.println("BMI160 initialization failed!");
        while (1);  // 如果初始化失败，停止程序
    }

    // 配置 BMI160 传感器设置
    mySensor.setAccelerometerRange(2);  // 设置加速度计量程为 ±2g
    mySensor.setGyroscopeRange(125);    // 设置陀螺仪量程为 125 degrees/second

    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);  // 配置 WS2812B RGB LED

    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MODE_SWITCH_PIN), switchMode, FALLING);  // 设置模式切换中断

    // 初始化 NimBLEDevice
    NimBLEDevice::init("MyDevice");
    NimBLEDevice::startAdvertising();
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        updateSensorData();  // 每隔一定时间更新传感器数据
        previousMillis = currentMillis;
    }

    switch (currentMode) {
        case 0:
            handleMouseMode();                     // 处理鼠标模式 (滑鼠左右鍵，陀螺仪)
            break;
        case 1:
            handleJoystickMouseMode();             // 处理搖桿滑鼠模式 (滑鼠左右鍵，搖桿)
            break;
        case 2:
            handlePresentationMode();              // 处理演示模式 (切換頁數，陀螺仪)
            break;
        case 3:
            handleJoystickPresentationMode();      // 处理搖桿演示模式 (切換頁數，搖桿)
            break;
        default:
            break;
    }

    NimBLEDevice::loop();  // 处理 NimBLEDevice 事件循环
}

void switchMode() {
    currentMode = (currentMode + 1) % 4;

    switch (currentMode) {
        case 0:
            currentColor = CRGB::Red;
            break;
        case 1:
            currentColor = CRGB::Green;
            break;
        case 2:
            currentColor = CRGB::Blue;
            break;
        case 3:
            currentColor = CRGB::White;
            break;
        default:
            currentColor = CRGB::Red;
            break;
    }

    fadeOutLeds();  // LED渐暗效果
    delay(500);
    fadeInLeds();  // LED渐亮效果
}

void handleMouseMode() {
    Mouse.begin();  // 初始化鼠标模式
    Mouse.releaseAll();  // 释放所有鼠标按钮

    if (analogRead(LEFT_BUTTON_PIN) < BUTTON_THRESHOLD) {
        Mouse.press(MOUSE_LEFT);  // 模拟按下鼠标左键
    }
    if (analogRead(RIGHT_BUTTON_PIN) < BUTTON_THRESHOLD) {
        Mouse.press(MOUSE_RIGHT);  // 模拟按下鼠标右键
    }

    // 根据陀螺仪数据更新鼠标移动量
    updateMouseMovement();
}

void handleJoystickMouseMode() {
    Mouse.begin();  // 初始化鼠标模式
    Mouse.releaseAll();  // 释放所有鼠标按钮

    if (analogRead(LEFT_BUTTON_PIN) < BUTTON_THRESHOLD) {
        Mouse.press(MOUSE_LEFT);  // 模拟按下鼠标左键
    }
    if (analogRead(RIGHT_BUTTON_PIN) < BUTTON_THRESHOLD) {
        Mouse.press(MOUSE_RIGHT);  // 模拟按下鼠标右键
    }

    // 使用搖桿控制移动鼠标
    handleJoystickControl();
}

void handlePresentationMode() {
    Keyboard.begin();  // 初始化键盘模式

    if (analogRead(LEFT_BUTTON_PIN) < BUTTON_THRESHOLD) {
        Keyboard.press(KEY_LEFT_ARROW);  // 模拟按下左箭头键
    }
    if (analogRead(RIGHT_BUTTON_PIN) < BUTTON_THRESHOLD) {
        Keyboard.press(KEY_RIGHT_ARROW);  // 模拟按下右箭头键
    }

    // 根据陀螺仪数据更新鼠标移动量
    updateMouseMovement();
}

void handleJoystickPresentationMode() {
    Keyboard.begin();  // 初始化键盘模式

    if (analogRead(LEFT_BUTTON_PIN) < BUTTON_THRESHOLD) {
        Keyboard.press(KEY_LEFT_ARROW);  // 模拟按下左箭头键
    }
    if (analogRead(RIGHT_BUTTON_PIN) < BUTTON_THRESHOLD) {
        Keyboard.press(KEY_RIGHT_ARROW);  // 模拟按下右箭头键
    }

    // 使用搖桿控制移动鼠标
    handleJoystickControl();
}

void updateSensorData() {
    // 更新 BMI160 传感器数据
    float ax, ay, az, gx, gy, gz;
    mySensor.getMotionSensorData(ax, ay, az, gx, gy, gz);

    // 处理获取的传感器数据
}

void handleJoystickControl() {
    joystick.update();  // 更新搖桿状态

    // 获取搖桿的X和Y轴值
    int xAxisValue = joystick.getX();
    int yAxisValue = joystick.getY();

    // 映射到鼠标移动量
    int mouseX = map(xAxisValue, joystick.getMinX(), joystick.getMaxX(), -MOUSE_MAX_SPEED, MOUSE_MAX_SPEED);
    int mouseY = map(yAxisValue, joystick.getMinY(), joystick.getMaxY(), -MOUSE_MAX_SPEED, MOUSE_MAX_SPEED);

    // 更新鼠标移动量
    Mouse.move(mouseX, mouseY);
}

void fadeInLeds() {
    for (int brightness = 0; brightness <= 255; brightness++) {
        for (int i = 0; i < NUM_LEDS; i++) {
            leds[i] = currentColor;
            FastLED.showColor(leds[i].fadeToBlackBy(255 - brightness));
        }
        delay(10);  // 控制亮度递增速度
    }
}

void fadeOutLeds() {
    for (int brightness = 255; brightness >= 0; brightness--) {
        for (int i = 0; i < NUM_LEDS; i++) {
            leds[i] = currentColor;
            FastLED.showColor(leds[i].fadeToBlackBy(255 - brightness));
        }
        delay(10);  // 控制亮度递减速度
    }
}

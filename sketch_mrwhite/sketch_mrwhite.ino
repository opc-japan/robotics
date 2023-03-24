// 1. Set sketchbook location to ~/opc-japan/robotics/sketch_mrwhite
// 2. Add this to Settings > Additional boards manager urls:
//   https://dl.espressif.com/dl/package_esp32_index.json
// 3. Install the following boards from boards manager:
//   esp32
// 4. Install the following libraries from library manager:
//   ESP32Servo
// 5. If you get an error saying that python executable is found on path, run this command and restart Arduino IDE:
//   sed -i -e 's/=python /=python3 /g' ~/Library/Arduino15/packages/esp32/hardware/esp32/*/platform.txt

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>

const char* ssid = "Okazaki_Hope_Chapel";
const char* password = "0001234567890";

enum Direction {
  Left,
  Right,
  Forward,
  Stop
};

class LeftMotor {
public:
  void init() {
    servo.attach(13);
  }

  void setSpeed(int speed) {
    servo.write(90 + speed * 180 / 200);
  }

private:
  Servo servo;
};

class RightMotor {
public:
  void init() {
    servo.attach(12);
  }

  void setSpeed(int speed) {
    servo.write(90 - speed * 180 / 200);
  }

private:
  Servo servo;
};

class Sonar {
public:
  Sonar(int e, int t) {
    echoPin = e;
    triggerPin = t;
  }

  void init() {
    pinMode(echoPin, INPUT);
    pinMode(triggerPin, OUTPUT);
  }

  double getDistance() {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);  //超音波を出力
    delayMicroseconds(10);           //
    digitalWrite(triggerPin, LOW);

    double duration = pulseIn(echoPin, HIGH);  //センサからの入力
    double distance = 0;

    if (duration > 0) {
      duration = duration / 2;                    //往復距離を半分にする
      distance = duration * 340 * 100 / 1000000;  // 音速を340m/sに設定
    }

    return distance;
  }

private:
  int echoPin;
  int triggerPin;
};

LeftMotor leftMotor;
RightMotor rightMotor;
Sonar frontSensor = Sonar(26, 27);
Sonar leftSensor = Sonar(25, 33);
Sonar rightSensor = Sonar(4, 16);

void initOTA() {
  Serial.begin(9600);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  initOTA();

  leftMotor.init();
  rightMotor.init();

  frontSensor.init();
  leftSensor.init();
  rightSensor.init();
}

void navigate(Direction direction) {
  int leftSpeed = 0;
  int rightSpeed = 0;

  switch (direction) {
    case Forward:
      leftSpeed = 100;
      rightSpeed = 30;
      break;
    case Left:
      leftSpeed = -30;
      rightSpeed = 50;
      break;
    case Right:
      leftSpeed = 50;
      rightSpeed = -30;
      break;
    case Stop:
      leftSpeed = 0;
      rightSpeed = 0;
      break;
  }

  // rightSpeed *= 0.5;

  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
}

Direction getDirection(double ld, double rd, double fd) {
  if (fd < 4 && ld < 4 && fd < 4) {
    return Stop;
  }

  if (fd < 10) {
    return ld > rd ? Left : Right;
  }

  if (ld < 8) {
    return Right;
  }

  if (rd < 8) {
    return Left;
  }

  return Forward;
}

char* toString(Direction direction) {
  switch (direction) {
    case Forward:
      return "Forward";
    case Left:
      return "Left";
    case Right:
      return "Right";
    case Stop:
      return "Stop";
  }
}

void loop() {
  ArduinoOTA.handle();

  double leftDistance = leftSensor.getDistance();
  double rightDistance = rightSensor.getDistance();
  double frontDistance = frontSensor.getDistance();
  Direction direction = getDirection(leftDistance, rightDistance, frontDistance);

  // Serial.printf("Left: %.2lf Front: %.2lf Right: %.2lf Direction: %s\n", leftDistance, frontDistance, rightDistance, toString(direction));

  navigate(direction);

  //delay(50);
}

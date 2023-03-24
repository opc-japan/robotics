#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>

const char* ssid = "MESH-WIFI-kkJHnF";
const char* password = "RcfnfgEnDpDXNu";

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

  int convert(int speed) {
    return 90 + speed * 180 / 200;
  }
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

  int convert(int speed) {
    return 90 - speed * 180 / 200;
  }
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

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

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
  switch (direction) {
    case Forward:
      leftMotor.setSpeed(100);
      rightMotor.setSpeed(100);
      break;
    case Left:
      leftMotor.setSpeed(50);
      rightMotor.setSpeed(100);
      break;
    case Right:
      leftMotor.setSpeed(100);
      rightMotor.setSpeed(50);
      break;
    case Stop:
      leftMotor.setSpeed(0);
      rightMotor.setSpeed(0);
      break;
  }
}

Direction getDirection(double ld, double rd, double fd) {
  if (fd < 4 && ld < 4 && fd < 4) {
    return Stop;
  }

  if (ld > fd) {
    if (ld > rd) {
      return Left;
    } else {
      return Right;
    }
  } else {
    if (fd > rd) {
      return Forward;
    } else {
      return Right;
    }
  }
}

void printDirection(Direction direction) {
  switch(direction) {
    case Forward:
      Serial.println("Direction: Forward");
      break;
    case Left:
      Serial.println("Direction: Left");
      break;
    case Right:
      Serial.println("Direction: Right");
      break;
    case Stop:
      Serial.println("Direction: Stop");
      break;
  }
}

void loop() {
  ArduinoOTA.handle();

  double leftDistance = leftSensor.getDistance();
  double rightDistance = rightSensor.getDistance();
  double frontDistance = frontSensor.getDistance();

  Serial.printf("Left: %lf Front: %lf Right: %lf\n", leftDistance, frontDistance, rightDistance);

  Direction direction = getDirection(leftDistance, rightDistance, frontDistance);
  printDirection(direction);

  // navigate(direction);

  delay(50);
}

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Servo.h>

enum Direction {
  Left,
  Right,
  Forward,
  Stop
}

class Sonar {
  int echoPin;
  int triggerPin;

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
      Serial.print("distance:");
      Serial.print(distance);
      Serial.println(" cm");
    }

    return distance;
  }
}

Servo leftMotor;   //Servoオブジェクトを作成
Servo rightMotor;  //Servoオブジェクトを作成
Sonar frontSensor = Sonar(4, 16);
Sonar leftSensor = Sonar(27, 26);
Sonar rightSensor = Sonar(33, 25);

const char* ssid = "MESH-WIFI-kkJHnF";
const char* password = "RcfnfgEnDpDXNu";

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

void initMotors() {
  leftMotor.attach(13);   //13番ピンにサーボ制御線（オレンジ）を接続
  rightMotor.attach(12);  //13番ピンにサーボ制御線（オレンジ）を接続
}

void initSensors() {
  frontSensor.init();
  leftSensor.init();
  rightSensor.init();
}

void setup() {
  initMotors();

  initSensors();

  initOTA();
}

Direction getDirection(double ld, double rd, double fd) {
  if (fd < 6 && ld < 6 && fd < 6) {
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

void navigate(Direction direction) {
  switch (direction) {
    case Forward:
      leftMotor.write(180);
      rightMotor.write(0);
    case Left:
      leftMotor.write(180);
      rightMotor.write(0);
    case Right:
      leftMotor.write(180);
      rightMotor.write(0);
    case Stop:
      leftMotor.write(90);
      rightMotor.write(90);
  }
}

void loop() {
  ArduinoOTA.handle();

  double leftDistance = leftSensor.getDistance();
  double rightDistance = rightSensor.getDistance();
  double frontDistance = frontSensor.getDistance();

  Direction direction = getDirection(leftDistance, rightDistance, frontDistance);

  navigate(direction);

  delay(50);
}
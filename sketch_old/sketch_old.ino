#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Servo.h>

const int left = 0;
const int right = 1;
const int forward = 2;
const int halt = 3;

const int frontSensorEchoPin = 26;
const int frontSensorTriggerPin = 27;

const int rightSensorEchoPin = 4;
const int rightSensorTriggerPin = 16;

const int leftSensorEchoPin = 25;
const int leftSensorTriggerPin = 33;

struct sonar
{
  int echoPin;
  int triggerPin;

};

typedef struct sonar Sonar;

Servo leftMotor;   //Servoオブジェクトを作成
Servo rightMotor;  //Servoオブジェクトを作成

Sonar frontSensor;
Sonar rightSensor;
Sonar leftSensor;

const char* ssid = "MESH-WIFI-kkJHnF";
const char* password = "RcfnfgEnDpDXNu";

void initOTA() {
  
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

void initSensor(int echoPin, int triggerPin) {
  pinMode(echoPin, INPUT);
  pinMode(triggerPin, OUTPUT);
}

double getDistance(int echoPin, int triggerPin) {
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

void initSensors() {
  initSensor(frontSensorEchoPin, frontSensorTriggerPin);
  initSensor(leftSensorEchoPin, leftSensorTriggerPin);
  initSensor(rightSensorEchoPin, rightSensorTriggerPin);
}

void setup() {
  Serial.begin(9600);
  
  initMotors();

  initSensors();

  // initOTA();
}

int getDirection(double ld, double rd, double fd) {
  if (fd < 4 && ld < 4 && fd < 4) {
    return halt;
  }

  if (ld > fd) {
    if (ld > rd) {
      return left;
    } else {
      return right;
    }
  } else {
    if (fd > rd) {
      return forward;
    } else {
      return right;
    }
  }
}

void navigate(int direction) {
  if (direction == forward) {
    leftMotor.write(180);
    rightMotor.write(0);
    return;
  }

  if (direction == left) {
    leftMotor.write(0);
    rightMotor.write(0);
    return;
  }

  if (direction == right) {
    leftMotor.write(180);
    rightMotor.write(180);
    return;
  }

  if (direction == halt) {
    leftMotor.write(90);
    rightMotor.write(90);
    return;
  }
}

void loop() {
  //ArduinoOTA.handle();

  double leftDistance = getDistance(leftSensorEchoPin, leftSensorTriggerPin);
  double rightDistance = getDistance(rightSensorEchoPin, rightSensorTriggerPin);
  double frontDistance = getDistance(frontSensorEchoPin, frontSensorTriggerPin);

  //Serial.print("Left Distance: ");
  //Serial.println(leftDistance);

  int direction = getDirection(leftDistance, rightDistance, frontDistance);

  Serial.printf("Direction: %d\n", direction);

  navigate(direction);

  delay(50);
}
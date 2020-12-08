#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>

#define MPU 0x68
#define MQTT_PORT 17827
#define MQTT_PWD "password"
#define MQTT_SERVER "farmer.cloudmqtt.com"
#define MQTT_USER "user"

char temp[7];
const char* ssid = "hotspot";
const char* password = "password";
double integral = 0;
double prev_pidError = 0;
int adjust = 0;
int deg = 0;
int deltaTheta = 0;
int event = 1;
int prevDeg = 0;
int unstability = 0;
int servoRotation_Y = 0;
int theta = 0;
int16_t x, y, z;

WiFiClient guest;
PubSubClient client(guest);
Servo servo_Y;

/* Setup function: gyroscope intentions
are declared here thanks to Wire.h. */
void setup() {
  Serial.begin(115200);

  servo_Y.attach(18);
  pinMode(5, OUTPUT);

  wifiStart();
  wireStart();

  mqttStart();
}

/* Loop function */
void loop() {
  client.loop();

  readGyro();
  wireConnection();

  doServo();
  doLed();

  mqttWrite();
}

/* Avoiding servo interruptions
or instability */
int deductServo(int x) {
  if (x < 20) {
    x = 17;
  } else if (x > 160) {
    x = 173;
  }

  return x;
}

/* LED blinking if the plane
is in a dangerous position */
void doLed() {
  if(deg > 20 || deg < -20) {
    digitalWrite(5, HIGH);
  } else {
    digitalWrite(5, LOW);
  }
}

/* Updating servo with gyroscope data */
void doServo() {
  getData();

  servoRotation_Y = deductServo(map(deg, -90, 90, 0, 179));
  servo_Y.write(servoRotation_Y);

  pidController();

  delay(50);
}

/* Gets inclination (-90° to +90°) and
computes angular velocity (theta/time)
where time is negligible since the
rotation happens istantly */
void getData() {
  deg = translator(-x, y, z);
  if (deg != prevDeg) {
    theta = abs(prevDeg - deg);
  } else {
    theta = 0;
  }

  prevDeg = deg;
  deltaTheta += theta;

  if (deg <= -20 || deg >= 20) {
    unstability++;
  }
}

/* Casting int16 values */
char* int16toString(int16_t i) {
  sprintf(temp, "%d", i);
  return temp;
}

/* Setting up a MQTT connection between
the ESP32 and a server hosted on cloudmqtt.
The messages will be viewable on "WEBSOCKET UI" */
void mqttStart() {
  client.setServer(MQTT_SERVER, MQTT_PORT);

  while (!client.connected()) {
    if (client.connect("ESP32Client", MQTT_USER, MQTT_PWD)) {
      break;
    } else {
      delay(2000);
    }
  }
}

/* Real-time data is sent to the MQTT
server to keep values updated. A message
is sent every 100 evalutation (nearly once
every 10 seconds) */
void mqttWrite() {
  if (event == 100) {
    char result[100];
    char* first = "Unstability: ";
    char* second = "; Degrees right now: ";
    char* third = "; Average angular velocity: ";
    char* textUnst;
    char* textPerc = "";

    if (unstability > 100) {                  //max is 202
      textUnst = "Too much unstable";
    } else {
      textUnst = int16toString(unstability);
      textPerc = "%";
    }

    strcpy(result, first);
    strcat(result, textUnst);
    strcat(result, textPerc);
    strcat(result, second);
    strcat(result, int16toString((int) deg));
    strcat(result, "°");
    strcat(result, third);
    strcat(result, int16toString((int) (deltaTheta/100)));
    strcat(result, "°/100rps");

    client.publish("normaloide.ino", result);
    deltaTheta = 0;
    unstability = 0;
    event = 0;
  } else {
    event++;
  }
}

/* PID controller */
void pidController() {
  getData();

  double weight = 1.3;
  int servoRotation_Now = deductServo(map(deg, -90, 90, 0, 179));
  double pidError = servoRotation_Now - servoRotation_Y;

  //proportional
  double pError = pidError * weight;

  //integral
  integral += pidError * 1;
  double iError = integral * weight;

  //derivative
  double derivative = (pidError - prev_pidError) / 100;
  double dError = derivative * weight;

  adjust = (int) (pError + iError + dError);
  if (servoRotation_Y + adjust > 90) {
    pError = 90;
  } else if (servoRotation_Y - adjust < -90) {
    pError = -90;
  }

  if (adjust > 0) {
    adjust = (int) (servoRotation_Y + adjust) % 180;
    servo_Y.write(map(adjust, -90, 90, 0, 179));
  } else if (adjust < 0) {
    adjust = (int) (servoRotation_Y - adjust) % (-180);
    servo_Y.write(map(adjust, -90, 90, 0, 179));
  }

  prev_pidError = pidError;;
}

/* Updating gyroscope values.
Data is 16bit, the first computed iteraction
is 8 bits, then it shifts by other 8 bits
(8 bits occupied, 8 bits free) so thanks
to the OR we can occupy the other
empty 8 bits */
void readGyro() {
  x = Wire.read()<<8 | Wire.read();
  y = Wire.read()<<8 | Wire.read();
  z = Wire.read()<<8 | Wire.read();
}

/* atan2(x, r) is the angle created from
the intersection of r with X. If y = 0,
I'll have a vector like (-x, 0, z) and
I'm interested in Y and Z. We use -x since
it's the axis that has to be inverted to
obtain a reversed rotation */
int translator(int x, int y, int z){
  double r, val;

  r = sqrt((y*y) + (z*z));

  val = atan2(x, r);
  val = val * 180/3.14;

  return (int) val;
}

/* Allows Wi-Fi connection (look at SSID and Password) */
void wifiStart() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  randomSeed(micros());
  Serial.println("WiFi connesso a " + WiFi.localIP());
}

/* Reading the gyroscope.
0x3B is the ACCEL_XOUT_H register, 3*2
are the registers we are interested in and
true allows us to stop further requests */
void wireConnection() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU, 3*2, true);
}

/* Starting the gyroscope */
void wireStart() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(false);
}

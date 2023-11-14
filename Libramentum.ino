/* Inclusions */
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
/* Definitions */
#define M_PI 3.14159
#define MPU 0x68
#define MQTT_PORT 17827
#define MQTT_PWD "password"
#define MQTT_SERVER "farmer.cloudmqtt.com"
#define MQTT_USER "user"
/* Variables */
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
Servo servoMotorY;

/* Setup */
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  // Attach the servo motor to pin 18
  servoMotorY.attach(18);
  // Set pin 5 as output
  pinMode(5, OUTPUT);
  // Initialize WiFi connection
  initializeWiFi();
  // Start Wire communication for sensor
  initializeWire();
  // Initialize MQTT client
  initializeMQTT();
}

/* Loop */
void loop() {
  // Handle incoming messages
  client.loop();
  // Read data from the gyroscope
  readGyro();
  wireConnection();
  // Control the servo motor
  doServo();
  // Control the LED
  doLed();
  // Publish sensor data
  mqttWrite();
}

/* Ensure servo motor stability */
int deductServo(int x) {
  return (x < 20) ? 17 : (x > 160) ? 173 : x;
}

/* LED indication for dangerous inclination */
void doLed() {
  if (deg > 20 || deg < -20) {
    digitalWrite(5, HIGH);
  } else {
    digitalWrite(5, LOW);
  }
}

/* Update the servo motor with gyroscope data */
void doServo() {
  // Get inclination data
  getData();
  // Calculate servo rotation
  servoRotation_Y = deductServo(map(deg, -90, 90, 0, 179));
  // Write to servo motor
  servoMotorY.write(servoRotation_Y);
  // Apply PID control
  pidController();
  // Small delay
  delay(50);
}

/*
  Gets inclination (-90° to +90°) and
  computes angular velocity (theta/time)
  where time is negligible since the
  rotation is instantaneous
*/
void getData() {
  deg = translator(-x, y, z);
  theta = (deg != prevDeg) ? abs(prevDeg - deg) : 0;
  prevDeg = deg;
  deltaTheta += theta;
  if (deg <= -20 || deg >= 20) {
    unstability++;
  }
}

/* Convert int16 to String */
char* int16toString(int16_t i) {
  sprintf(temp, "%d", i);
  return temp;
}

/* Setting up a MQTT connection between
the ESP32 and a server hosted on cloudmqtt.
The messages will be viewable on "WEBSOCKET UI" */
void initializeMQTT() {
  client.setServer(MQTT_SERVER, MQTT_PORT);

  while (!client.connected()) {
    if (client.connect("ESP32Client", MQTT_USER, MQTT_PWD)) {
      break;
    } else {
      delay(2000);
    }
  }
}

/*
  Real-time data is sent to the MQTT
  server to keep values updated.
  A message is sent every 100 evalutations
  (nearly once every 10 seconds)
*/
void mqttWrite() {
  if (event == 100) {
    char result[202];
    // Format the result string (max instability is 202)
    snprintf(
      result,
      sizeof(result),
      "Unstability: %s%s; Degrees right now: %d°; Average angular velocity: %d°/100rps",
      (unstability > 100) ? "Too much unstable" : int16ToString(unstability),
      (unstability > 100) ? "" : "%",
      (int) deg,
      (int) (deltaTheta / 100)
    );

    // Publish the result to MQTT
    client.publish("libramentum.ino", result);
    // Reset counters
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
  // Proportional
  double pError = pidError * weight;
  // Integral
  integral += pidError;
  double iError = integral * weight;
  // Derivative
  double derivative = (pidError - prev_pidError);
  double dError = derivative * weight;

  adjust = (int) (pError + iError + dError);
  // Limit the adjust value to prevent exceeding servo limits
  if (servoRotation_Y + adjust > 90) {
    pError = 90;
  } else if (servoRotation_Y - adjust < -90) {
    pError = -90;
  }

  // Apply adjustment to the servo motor
  if (adjust > 0) {
    adjust = (int) (servoRotation_Y + adjust) % 180;
  } else if (adjust < 0) {
    adjust = (int) (servoRotation_Y - adjust) % (-180);
  }

  servoMotorY.write(map(adjust, -90, 90, 0, 179));

  prev_pidError = pidError;;
}

/*
  Updating gyroscope values.
  Data is 16bit, the first computed iteraction
  is 8 bits, then it shifts by other 8 bits
  (8 bits occupied, 8 bits free) so thanks
  to the OR we can occupy the other
  empty 8 bits
*/
void readGyro() {
  x = (Wire.read() << 8) | Wire.read();
  y = (Wire.read() << 8) | Wire.read();
  z = (Wire.read() << 8) | Wire.read();
}

/*
  This function calculates the angle (in degrees)
  formed by a vector in three-dimensional space
  represented by Cartesian coordinates (x, y, z).
  It uses the atan2(x, r) function to determine the
  angle of rotation around the Y-axis, created from
  the intersection of r with X.

  If the y-component of the vector is zero (y = 0),
  the vector lies in the XZ-plane, and we are
  interested in the rotation around the Y-axis.
  To achieve this, we use -x in the atan2 function
  since inverting the x-axis results in a reversed
  rotation.
*/
int translator(int x, int y, int z){
  double r, val;

  r = sqrt((y*y) + (z*z));

  val = atan2(x, r);
  val = val * 180 / M_PI;

  return (int) val;
}

/* Allows Wi-Fi connection (look at SSID and Password) */
void initializeWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  randomSeed(micros());
  Serial.println("WiFi connesso a " + WiFi.localIP());
}

/*
  Reading the gyroscope.
  0x3B is the ACCEL_XOUT_H register, 3*2
  are the registers we are interested in and
  true allows us to stop further requests
*/
void wireConnection() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU, 3*2, true);
}

/* Starting the gyroscope */
void initializeWire() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(false);
}

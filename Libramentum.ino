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
const char* password =  "password";
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

/**
 * Metodo di setup: vengono dichiarate le intenzioni del
 * giroscopio/accelerometro tramite la libreria Wire.
 */
void setup() {
  Serial.begin(115200);
  
  servo_Y.attach(18);
  pinMode(5, OUTPUT);
  
  wifiStart();
  wireStart();

  mqttStart();
}

/**
 * Qui vengono posizionati i metodi che
 * devono essere ripetuti ciclicamente.
 */
void loop() {
  client.loop();
  
  readGyro();
  wireConnection();
  
  doServo();
  doLed();

  mqttWrite();
}

/*
 * Operazioni per evitare che il servo-motore si inceppi e alteri
 * la sua stabilita'
 */
int deductServo(int x) {
  if(x < 20) {
    x = 17;
  } else if(x > 160) {
    x = 173;
  }

  return x;
}

/*
 * Semplice funzione che accende il LED quando il piano
 * si trova in una situazione critica
 */
void doLed() {
  if(deg > 20 || deg < -20) {
    digitalWrite(5, HIGH);
  } else {
    digitalWrite(5, LOW);
  }
}

/**
 * Viene semplicemento chiamato ad ogni loop per aggiornare
 * il servo motore con i dati del giroscopio.
 */
void doServo() {
  getData();
  
  servoRotation_Y = deductServo(map(deg, -90, 90, 0, 179));
  servo_Y.write(servoRotation_Y);

  pidController();
    
  delay(50);
}

/*
 * Preleva i dati per l'inclinazione del servo motore, che a causa
 * del giroscopio è solo da -90° a +90° e calcolare i vari delta
 * per calcolare la velocità angolare media (theta/tempo dove il tempo
 * e' trascurabile dato che la rotazione è praticamente istantanea).
 */
void getData() {
  deg = translator(-x, y, z);
  if(deg != prevDeg) {
    theta = abs(prevDeg - deg);
  } else {
    theta = 0;
  }

  prevDeg = deg;
  deltaTheta += theta;
  
  if(deg <= -20 || deg >= 20) {
    unstability++;
  }
}

/*
 * Permette di castare i valori numerici di i
 * nel char array "temp", per poi farne il return.
 */
char* int16toString(int16_t i) {
  sprintf(temp, "%d", i);
  return temp;
}

/**
 * Stabilisce una connessione MQTT tra l'ESP32 e il server hostato
 * su piattaforma apposita. I messaggi ricevuti verranno mostrati
 * nell'apposita box "WEBSOCKET UI" presente sul sito.
 */
void mqttStart() {
  client.setServer(MQTT_SERVER, MQTT_PORT);

  while(!client.connected()) {
    if(client.connect("ESP32Client", MQTT_USER, MQTT_PWD)) {
      break;
    } else {
      delay(2000);
    }
  }
}

/**
 * Riferisce in tempo reale, tramite MQTT, le modifiche effettuate
 * sull'accelerometro e sul servo motore, in modo tale da tenere
 * il sistema sempre aggiornato. Stampa una volta su 100, per non
 * intasare la queue (approssimativamente 1 messaggio ogni 10s).
 */
void mqttWrite() {
  if(event == 100) {
    char result[100];
    char* first = "Unstability: ";
    char* second = "; Degrees right now: ";
    char* third = "; Average angular velocity: ";
    char* textUnst;
    char* textPerc = "";
    
    if(unstability > 100) { //202 e' il massimo
      textUnst = "Too much unstable";
    } else {
      textUnst = int16toString(unstability);
      textPerc = "%";
    }
    
    strcpy(result, first);
    strcat(result, textUnst);
    strcat(result, textPerc);
    strcat(result, second);
    strcat(result, int16toString((int)deg));
    strcat(result, "°");
    strcat(result, third);
    strcat(result, int16toString((int)(deltaTheta/100)));
    strcat(result, "°/100rps");
  
    client.publish("normaloide.ino", result);
    deltaTheta = 0;
    unstability = 0;
    event = 0;
  } else {
    event++;
  }
}

/**
 * Funzione che implementa il PID controller per regolare
 * la rotazione del motore attraverso i relativi dati.
 */
void pidController() {
  getData(); //faccio ricalcolare deg

  double weight = 1.3;
  int servoRotation_Now = deductServo(map(deg, -90, 90, 0, 179));
  double pidError = servoRotation_Now - servoRotation_Y; //errore

  //proportional
  double pError = pidError * weight;

  //integral
  integral += pidError * 1;
  double iError = integral * weight;

  //derivative
  double derivative = (pidError - prev_pidError) / 100;
  double dError = derivative * weight;
  
  adjust = (int)(pError + iError + dError);
  if(servoRotation_Y + adjust > 90) {
    pError = 90;
  } else if(servoRotation_Y - adjust < -90) {
    pError = -90;
  }
  
  if(adjust > 0) {
    adjust = (int)(servoRotation_Y + adjust) % 180;
    servo_Y.write(map(adjust, -90, 90, 0, 179));
  } else if(adjust < 0) {
    adjust = (int)(servoRotation_Y - adjust) % (-180);
    servo_Y.write(map(adjust, -90, 90, 0, 179));
  }

  prev_pidError = pidError;;
}

 /**
 * Aggiorna lo stato del valore dell'asse x e y del giroscopio.
 * Il dato è di 16bit, la prima iterazione calcola 8bit, e
 * shifta di 8bit (8bit occupati, 8 vuoti), l'or permette
 * al secondo read() di occupare gli 8bit rimanenti.
 */
void readGyro() {
  x = Wire.read()<<8 | Wire.read();
  y = Wire.read()<<8 | Wire.read();
  z = Wire.read()<<8 | Wire.read();
}

/**
 * Dai complessi, atan2(x, r) sarà l'angolo dato dall'intersezione di r con x.
 * Nel caso in cui y = 0 (quindi il servo_Y), avrò un vettore del tipo
 * (-x, 0, z) e mi interesserà la rotazione sulla composizione degli assi Y e Z.
 * Usiamo -x in quanto e' l'asse che deve essere invertito per ottenere 
 * una rotazione opposta.
 */
int translator(int x, int y, int z){
  double r, val;
  
  r = sqrt((y*y) + (z*z));

  val = atan2(x, r);
  val = val * 180/3.14;
  
  return (int)val;
}

/**
 * Metodo per permettere al WiFi di collegarsi ad una rete
 * precedentemente definita (guardo SSID e Password).
 */
void wifiStart() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
  randomSeed(micros());
  Serial.println("WiFi connesso a " + WiFi.localIP());
}

/**
 * Metodo per la trasmissione del giroscopio/accelerometro.
 * 0x3B è l'ACCEL_XOUT_H register mentre in requestFrom(),
 * 3*2 sono i registri da richiedere e true serve per fermare richieste ulteriori.
 */
void wireConnection() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU, 3*2, true);
}

/**
 * Metodo che raggruppa le azioni di Wire una volta che l'Arduino
 * è avviato e i processi devono essere eseguiti.
 */
void wireStart() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(false);
}

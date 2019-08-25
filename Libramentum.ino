#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>

#define MPU 0x68
#define MQTT_PORT 000000
#define MQTT_PWD ""
#define MQTT_SERVER ""
#define MQTT_USER ""

char temp[7];
const char* ssid = "";
const char* password =  "";
int event = 1;
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

  mqttWrite();
}

/**
 * Viene semplicemento chiamato ad ogni loop per aggiornare
 * il servo motore con i dati del giroscopio.
 */
void doServo() {
  int servoRotation_Y = map(translator(-x, y, z), -90, 90, 0, 179);
  servo_Y.write(servoRotation_Y);
    
  delay(100);
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

  client.publish("normaloide.ino", "Sono sveglio!");
}

/**
 * Riferisce in tempo reale, tramite MQTT, le modifiche effettuate
 * sull'accelerometro e sul servo motore, in modo tale da tenere
 * il sistema sempre aggiornato. Stampa una volta su 100, per non
 * intasare la queue (approssimativamente 1 messaggio ogni 10s).
 */
 void mqttWrite() {
  if(event == 100) {
    client.publish("normaloide.ino", int16toString(x));
    event = 0;
  } else {
    event++;
  }
 }

/**
 * Dai complessi, atan2(x, r) sarà l'angolo dato dall'intersezione di r con x.
 * Nel caso in cui y = 0 (quindi il servo_Y), avrò un vettore del tipo
 * (-x, 0, z) e mi interesserà la rotazione sulla composizione degli assi X e Z.
 * Usiamo -x in quanto e' l'asse che deve essere invertito per ottenere 
 * una rotazione opposta.
 */
double translator(int x, int y, int z){
  double r, val;
  r = sqrt((y*y) + (z*z));
  
  val = atan2(x, r);
  val = val * 180/M_PI;
  
  return (int)val;
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
 * 2 sono i byte da richiedere e true serve per fermare richieste ulteriori.
 */
void wireConnection() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU, 2, true);
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

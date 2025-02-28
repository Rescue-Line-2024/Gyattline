#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <ArduinoJson.h>  // Include la libreria ArduinoJson

// Inizializzazione del driver PWM (indirizzo di default 0x40)
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

// --------------------------
// COSTANTI E DEFINIZIONI
// --------------------------
#define SERVOMIN  100   // Valore minimo per il servo (da regolare)
#define SERVOMAX  600   // Valore massimo per il servo (da regolare)

// --------------------------
// CONFIGURAZIONE DEI SENSORI ULTRASONICI
// I sensori verranno letti nei seguenti pin:
// - Sensore Frontale:  trig = 3,  echo = 4
// - Sensore Destro:     trig = 8,  echo = 7
// - Sensore Sinistro:   trig = 5,  echo = 6
// --------------------------
#define FRONT_TRIG 3
#define FRONT_ECHO 4

#define RIGHT_TRIG 8
#define RIGHT_ECHO 7

#define LEFT_TRIG 5
#define LEFT_ECHO 6

#define IN_A0  A1  // (Non modificato; verificare se è effettivamente usato)

// --------------------------
// VARIABILI PER IL MONITORAGGIO DEL SENSORE FRONTANALE
// --------------------------
unsigned long frontBelowThresholdStart = 0;
bool frontBelowThresholdNotified = false;

// --------------------------
// FUNZIONI DI CONTROLLO DEI MOTORI
// --------------------------
void AvviaServoAvanti(int speed, int speed2) {
  // Imposta il PWM per controllare i servomotori a rotazione continua
  board1.setPWM(0, 0, map(speed, -100, 100, SERVOMIN, SERVOMAX));
  board1.setPWM(1, 0, map(speed2, -100, 100, SERVOMIN, SERVOMAX));
}

void AvviaServoIndietro(int speed, int speed2) {
  board1.setPWM(2, 0, map(speed, -100, 100, SERVOMIN, SERVOMAX));
  board1.setPWM(3, 0, map(speed2, -100, 100, SERVOMIN, SERVOMAX));
}

void AvviaMotori(int DX, int SX, int lim = 100) {
  int potenzaDX = constrain(DX, -lim, lim)*-1;
  int potenzaSX = constrain(SX, -lim, lim)*-1;

  // Avvia i motori anteriori (aggiunge un offset di 10 per compensare eventuali disallineamenti)
  AvviaServoAvanti(potenzaDX + 10, -potenzaSX + 10);

  // Avvia i motori posteriori: qui si impone che la velocità sia non negativa
  potenzaDX = constrain(potenzaDX, -lim, lim);
  potenzaSX = constrain(potenzaSX, -lim, lim);
  AvviaServoIndietro(potenzaDX + 10, -potenzaSX + 10);
}

void stop() {
  AvviaServoAvanti(10, 10);
  AvviaServoIndietro(10, 10);
}

// --------------------------
// FUNZIONI PER IL CONTROLLO DEI SERVOMOTORI (BRACCIO, TILT, ecc.)
// --------------------------
void impostaservo(int gradi, int pin) {
  board1.setPWM(pin, 0, angleToPulse(gradi));
}

void gestiscibraccio(int gradi) {
  impostaservo(180 - gradi, 9);
  impostaservo(gradi, 8);
}

void apribracci() {
  impostaservo(20, 13);   // 20: braccio aperto (70: chiuso)
  impostaservo(180, 14);  // 180: aperto (135: chiuso)
}

void chiudibracci() {
  impostaservo(60, 13);   // 60: posizione per chiudere (rispetto a 20 aperto e 70 chiuso)
  impostaservo(60, 14);
}

// Converte un angolo (0-180) in un valore di pulse per il servo
int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}

// --------------------------
// FUNZIONE PER LA LETTURA DEI SENSORI ULTRASONICI
// --------------------------
int CheckDistanza(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long durata = pulseIn(echo, HIGH);
  int distanza = durata / 58;  // Conversione in centimetri (approssimativa)
  return distanza;
}

// --------------------------
// VARIABILI PER LA COMUNICAZIONE SERIAL
// --------------------------
String Message;
String Action;
int MotoreDX, MotoreSX;

// --------------------------
// SETUP
// --------------------------
void setup() {
  Serial.begin(115200);
  
  // Inizializza il driver PWM
  board1.begin();
  board1.setPWMFreq(60);      
  AvviaServoAvanti(10, 10);
  AvviaServoIndietro(10, 10);
 
  // Configurazione del pin analogico (se necessario)
  pinMode(IN_A0, INPUT);

  // Configura i pin dei sensori ad ultrasuoni
  // Sensore frontale
  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  // Sensore destro
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);
  // Sensore sinistro
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);

  // Inizializzazione di alcuni servomotori (es. TILT e braccio)
  impostaservo(145, 4); // Esempio: 145 gradi per il TILT (altri valori possibili in base all'applicazione)
  impostaservo(95, 15);
 
  gestiscibraccio(200); // Posiziona il braccio in alto
  apribracci();        // Apre i bracci
  
}

// --------------------------
// LOOP PRINCIPALE
// --------------------------
void loop() {
  // ----- Gestione dei comandi in arrivo via Serial -----
  if (Serial.available() > 0) {
    Message = Serial.readStringUntil('\n');

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, Message);
    if (error) {
      Serial.print("Errore nella deserializzazione: ");
      Serial.println(error.c_str());
      return;
    }

    Action = doc["action"].as<String>();
    
    // Comando per controllare i motori
    if (Action == "motors") {
      MotoreDX = doc["data"][0];
      MotoreSX = doc["data"][1];
      AvviaMotori(MotoreDX, MotoreSX);
    }
    
    if (Action == "stop") {
      AvviaMotori(0, 0);
    }
    
    // Comando per richiedere i dati dei sensori
    if (Action == "get_sensors") {
      int frontDistance = CheckDistanza(FRONT_TRIG, FRONT_ECHO);
      int rightDistance = CheckDistanza(RIGHT_TRIG, RIGHT_ECHO);
      int leftDistance  = CheckDistanza(LEFT_TRIG, LEFT_ECHO);
      
      StaticJsonDocument<200> response;
      response["front"] = frontDistance;
      response["right"] = rightDistance;
      response["left"]  = leftDistance;
      
      serializeJson(response, Serial);
      Serial.println();
    }
  }

}

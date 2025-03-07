+#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

// Inizializzazione del driver PWM per il controllo di motori e servomotori
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Definizione del pin collegato al sensore SilverLine
#define SILVERLINE 10  

// Definizione degli stati della macchina a stati per gestire la missione
enum State {
  WAIT_FOR_SILVER,  // In attesa che il robot entri nell'area (sensore attivo)
  SEARCH_BALL,      // Ricerca della palla mediante movimenti "scattosi"
  COLLECT_BALL,     // Raccolta della palla (movimenti del braccio e pinza)
  SEARCH_BIN,       // Ricerca del cestino corretto per depositare la palla
  RELEASE_BALL,     // Avvicinamento e rilascio della palla nel cestino
  EXIT_NAV,         // Avvicinamento al muro per iniziare la fase di uscita
  EXIT_CHECK,       // Richiesta del frame per verificare la presenza della linea nera
  EXIT_ALIGN,       // Allineamento con la linea nera tramite PID
  IDLE              // Stato di inattività (missione completata)
};
State currentState = WAIT_FOR_SILVER;

// Variabili per gestire gli intervalli dei movimenti "scattosi"
unsigned long lastScanTime = 0;
const unsigned long scanInterval = 500; // intervallo in millisecondi tra uno scatto

// Contatori per il numero di palline raccolte
int silverBallCount = 0;
int blackBallCount = 0;

// Variabile per memorizzare il colore dell'ultima palla rilevata (usato per scegliere il cestino)
String currentBallColor = "";

// Funzione per mappare il valore dei motori da un range (-100 a +100) al range accettato dal driver PWM (0-4095)
int mapMotor(int val) {
  return map(val, -100, 100, 0, 4095);
}

// Funzione per mappare il valore dei servomotori da un range (0 a 180°) al range utilizzato dal driver PWM
int mapServo(int angle) {
  return map(angle, 0, 180, 150, 600);
}

// Funzione per comandare i motori e i servomotori inviando i valori mappati al driver PWM
void AvviaMotori(int dxF, int sxF, int dxB, int sxB, int dxS, int sxS, int dxGrab, int sxGrab) {
  pwm.setPWM(0, 0, mapMotor(dxF));    // Motore anteriore destro
  pwm.setPWM(1, 0, mapMotor(sxF));    // Motore anteriore sinistro
  pwm.setPWM(2, 0, mapMotor(dxB));    // Motore posteriore destro
  pwm.setPWM(3, 0, mapMotor(sxB));    // Motore posteriore sinistro
  pwm.setPWM(4, 0, mapServo(dxS));    // Servo del braccio destro (shoulder)
  pwm.setPWM(5, 0, mapServo(sxS));    // Servo del braccio sinistro (shoulder)
  pwm.setPWM(6, 0, mapServo(dxGrab)); // Servo della pinza destra
  pwm.setPWM(7, 0, mapServo(sxGrab)); // Servo della pinza sinistra
}

// Funzione per fermare tutti i motori
void stopMotors() {
  AvviaMotori(0, 0, 0, 0, 0, 0, 0, 0);
}

void setup() {
  Serial.begin(115200);          // Inizializzazione della comunicazione seriale con il Raspberry
  pinMode(SILVERLINE, INPUT);     // Configurazione del pin del sensore come input
  pwm.begin();                   // Inizializzazione del driver PWM
  pwm.setPWMFreq(60);            // Imposta la frequenza PWM a 60 Hz
}

void loop() {
  // Gestione dei messaggi in arrivo dalla parte di visione (Raspberry)
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n'); // Legge il messaggio JSON
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
    if (!error) { // Se il messaggio è stato parsificato correttamente
      String action = doc["action"];

      // Gestione del messaggio relativo alla rilevazione della palla
      if (action == "BALL_DETECTED") {
        bool vicino = doc["data"]["vicino"];         // Flag che indica se la palla è vicina
        int correzione = doc["data"]["correzione"];     // Valore di correzione calcolato dal PID
        currentBallColor = String((const char*)doc["data"]["color"]); // Aggiorna il colore rilevato della palla
        if (vicino) {
          stopMotors();         // Se la palla è vicina, ferma il robot
          currentState = COLLECT_BALL; // Passa allo stato di raccolta della palla
        } else {
          // Applica una correzione girando sul posto
          AvviaMotori(correzione, -correzione, -correzione, correzione, 0, 0, 0, 0);
        }
      }
      // Gestione del messaggio relativo al rilevamento del cestino
      else if (action == "BIN_DETECTED") {
        bool vicino = doc["data"]["vicino"];
        int correzione = doc["data"]["correzione"];
        if (vicino) {
          stopMotors();
          currentState = RELEASE_BALL; // Se il cestino è vicino, passa allo stato di rilascio della palla
        } else {
          AvviaMotori(correzione, -correzione, -correzione, correzione, 0, 0, 0, 0);
        }
      }
      // Gestione del messaggio relativo al rilevamento della linea d'uscita
      else if (action == "EXIT_DETECTED") {
        bool vicino = doc["data"]["vicino"];
        int correzione = doc["data"]["correzione"];
        // Gestione in stato EXIT_CHECK: se la linea non è rilevata, esegue la rotazione
        if (currentState == EXIT_CHECK) {
          if (vicino) {
            stopMotors();
            currentState = EXIT_ALIGN; // Se la linea è rilevata, passa allo stato di allineamento
          } else {
            stopMotors();
            Serial.println("EXIT_DETECTED: Linea non trovata, ruoto 90 gradi sinistra");
            // Rotazione a sinistra per circa 90° (tempo predefinito per simulare il giro)
            AvviaMotori(-100, 100, -100, 100, 0, 0, 0, 0);
            delay(500);
            stopMotors();
            // Movimento in avanti per acquisire un nuovo frame
            AvviaMotori(100, 100, 100, 100, 0, 0, 0, 0);
            delay(500);
            stopMotors();
            Serial.println("REQUEST_EXIT_FRAME");
            lastScanTime = millis();
          }
        }
        // Gestione in stato EXIT_ALIGN: applica il PID per allinearsi con la linea
        else if (currentState == EXIT_ALIGN) {
          if (!vicino) {
            AvviaMotori(correzione, -correzione, -correzione, correzione, 0, 0, 0, 0);
          } else {
            stopMotors();
            Serial.println("Linea d'uscita allineata, esco dalla stanza.");
            // Movimento in avanti per uscire dalla stanza
            AvviaMotori(100, 100, 100, 100, 0, 0, 0, 0);
            delay(1500);
            stopMotors();
            currentState = IDLE;
            Serial.println("MISSIONE_COMPLETATA");
          }
        }
      }
    }
  }
  
  // Gestione della macchina a stati tramite switch-case
  switch (currentState) {
    case WAIT_FOR_SILVER:
      // In attesa che il sensore SilverLine venga attivato
      if (digitalRead(SILVERLINE) == HIGH) {
        Serial.println("SILVER_IS_TOUCHED");
        // Movimento in avanti per assicurarsi di essere all'interno
        AvviaMotori(100, 100, 100, 100, 0, 0, 0, 0);
        delay(1000);
        stopMotors();
        currentState = SEARCH_BALL;  // Passa allo stato di ricerca della palla
        lastScanTime = millis();
      }
      break;
      
    case SEARCH_BALL:
      // Esegue movimenti "scattosi" a intervalli regolari per cercare la palla
      if (millis() - lastScanTime >= scanInterval) {
        // Movimento rotazionale (es. gira a sinistra per 200 ms)
        AvviaMotori(50, -50, -50, 50, 0, 0, 0, 0);
        delay(200);
        stopMotors();
        // Richiede un frame al Raspberry per il rilevamento della palla
        Serial.println("REQUEST_BALL_FRAME");
        lastScanTime = millis();
      }
      break;
      
    case COLLECT_BALL:
      // Sequenza per raccogliere la palla:
      Serial.println("COLLECTING_BALL");
      // Abbassa il braccio (imposta l'angolo a 90°)
      AvviaMotori(0, 0, 0, 0, 90, 90, 0, 0);
      delay(500);
      // Chiude la pinza (imposta l'angolo a 180°)
      AvviaMotori(0, 0, 0, 0, 0, 0, 180, 180);
      delay(300);
      // Rialza il braccio tornando alla posizione iniziale (0°)
      AvviaMotori(0, 0, 0, 0, 0, 0, 0, 0);
      delay(500);
      stopMotors();
      Serial.println("BALL_COLLECTED");
      currentState = SEARCH_BIN;  // Dopo la raccolta passa alla ricerca del cestino
      lastScanTime = millis();
      break;
      
    case SEARCH_BIN:
      // Ricerca del cestino mediante movimenti "scattosi"
      if (millis() - lastScanTime >= scanInterval) {
        AvviaMotori(50, -50, -50, 50, 0, 0, 0, 0);
        delay(200);
        stopMotors();
        Serial.println("REQUEST_BIN_FRAME");
        lastScanTime = millis();
      }
      break;
      
    case RELEASE_BALL:
      // Rilascio della palla nel cestino:
      Serial.println("RELEASING_BALL");
      // Movimento in avanti per avvicinarsi al cestino
      AvviaMotori(100, 100, 100, 100, 0, 0, 0, 0);
      delay(1000);
      stopMotors();
      // Apri la pinza per rilasciare la palla (valore 0 = aperto)
      AvviaMotori(0, 0, 0, 0, 0, 0, 0, 0);
      delay(300);
      stopMotors();
      // Aggiorna i contatori in base al colore della palla raccolta
      if (currentBallColor == "silver") {
        silverBallCount++;
      } else if (currentBallColor == "black") {
        blackBallCount++;
      }
      Serial.print("Contatori - Silver: ");
      Serial.print(silverBallCount);
      Serial.print("  Black: ");
      Serial.println(blackBallCount);
      
      // Se la missione (2 palline silver e 1 nera) è completata, passa alla fase di uscita
      if (silverBallCount >= 2 && blackBallCount >= 1) {
        currentState = EXIT_NAV;
        Serial.println("MISSIONE_COMPLETA: Avvio uscita");
      } else {
        Serial.println("BALL_RELEASED");
        currentState = SEARCH_BALL;
        lastScanTime = millis();
      }
      break;
      
    case EXIT_NAV:
      // Avvicinamento al muro per iniziare la fase di uscita
      Serial.println("EXIT_NAV: Avvicinamento al muro");
      AvviaMotori(100, 100, 100, 100, 0, 0, 0, 0);
      delay(1000);  // Movimento in avanti per un tempo fisso
      stopMotors();
      currentState = EXIT_CHECK; // Passa alla fase di verifica della linea nera
      lastScanTime = millis();
      break;
      
    case EXIT_CHECK:
      // Richiede periodicamente un frame per verificare la presenza della linea nera
      if (millis() - lastScanTime >= scanInterval) {
        Serial.println("REQUEST_EXIT_FRAME");
        lastScanTime = millis();
      }
      break;
      
    case EXIT_ALIGN:
      // In stato di allineamento: continua a richiedere frame per aggiustare la direzione
      if (millis() - lastScanTime >= scanInterval) {
        Serial.println("REQUEST_EXIT_FRAME");
        lastScanTime = millis();
      }
      break;
      
    case IDLE:
      // Stato di inattività: la missione è completata
      stopMotors();
      break;
      
    default:
      break;
  }
}

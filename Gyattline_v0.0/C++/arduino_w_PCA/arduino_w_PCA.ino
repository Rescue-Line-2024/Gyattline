#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <ArduinoJson.h>  // Include la libreria ArduinoJson
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);  // called this way, it uses the default address 0x40  


#define SERVOMIN  100   // Adjust according to your servo
#define SERVOMAX  600   // Adjust according to your servo
#define Muro_trig 5
#define Muro_echo 6
#define Ostacolo_trig 3
#define Ostacolo_echo 4
#define IN_A0  A1

long durata,cm,durata2,cm2,rifrazione,err;
int Deviazione,c=0,last_error = 0;
int timer = millis();
bool ag = false;

void stop()
{
    AvviaServoAvanti(10,10);
    AvviaServoIndietro(10,10);
}
void setup() {
  Serial.begin(9600);
  cm = 1000;
  board1.begin();
  board1.setPWMFreq(60);      
  AvviaServoAvanti(10,10);
  AvviaServoIndietro(10,10);
 
  pinMode (IN_A0,INPUT);


  pinMode(Muro_trig, OUTPUT);
  pinMode(Muro_echo, INPUT);

  pinMode(Ostacolo_trig,OUTPUT);
  pinMode(Ostacolo_echo,INPUT);

  impostaservo(145,4); //TILT (160 per il seguilinea,115 per le palline)
  impostaservo(95,15);
 
  gestiscibraccio(200); //braccio sopra
  apribracci();

}





void apribracci()
{
  impostaservo(20,13);  //20 aperto 70 chiuso
  impostaservo(180,14); //180 aperto 135 chiuso
}
void chiudibracci()
{
  impostaservo(60,13);  //20 aperto 70 chiuso
  impostaservo(60,14); //180 aperto 135 chiuso
}

void impostaservo(int gradi,int pin){board1.setPWM(pin, 0, angleToPulse(gradi) );}

void gestiscibraccio(int gradi)
{
  impostaservo(180-gradi,9);
  impostaservo(gradi,8);
}


String Message;







int timer1 = millis();

String Action;
String Data;
int MotoreDX,MotoreSX;
void loop() {


  if (Serial.available() > 0) {

    Message = Serial.readStringUntil('\n');
    Serial.println("Messaggio ricevuto: " + Message);

    StaticJsonDocument<200> doc;  // Creazione di un buffer JSON

    DeserializationError error = deserializeJson(doc, Message);

    if (error) {
      Serial.print("Errore nella deserializzazione: ");
      Serial.println(error.c_str());
      return;  // Esci se c'è un errore
    }

    Action = doc["action"].as<String>();  // Ottiene il valore associato alla chiave "comando"
    Data = doc["data"];

    
    if(Action == "motors")
        {
        MotoreDX = Data[0];
        MotoreSX = Data[1];
        AvviaMotori(MotoreDX,MotoreSX);
        }

    
    
  }
}



//MOTORI ANTERIORI
void AvviaServoAvanti(int speed,int speed2) {
  // Set PWM to control the speed and direction of the continuous rotation servo
  board1.setPWM(0, 0, map(speed, -100, 100, SERVOMIN, SERVOMAX));
  board1.setPWM(1, 0, map(speed2, -100, 100, SERVOMIN, SERVOMAX));
}
//MOTORI POSTERIORI
void AvviaServoIndietro(int speed,int speed2)
{
  board1.setPWM(2, 0, map(speed, -100, 100, SERVOMIN, SERVOMAX));
  board1.setPWM(3, 0, map(speed2, -100, 100, SERVOMIN, SERVOMAX));
}

void AvviaMotori(int DX,int SX,int lim = 100)
{

  int potenzaDX = DX;
  int potenzaSX = SX;

  potenzaDX = constrain(potenzaDX, -lim, lim);
  potenzaSX = constrain(potenzaSX,-lim,lim);


  AvviaServoAvanti(potenzaDX+10,-1*(potenzaSX)+10);


  potenzaDX = constrain(potenzaDX, 0, lim);
  potenzaSX = constrain(potenzaSX,0,lim);


  AvviaServoIndietro(potenzaDX+10,-1*(potenzaSX)+10);
}

int CheckDistanza(int trig,int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  int durata = pulseIn(echo, HIGH);
  int cm = durata / 58;
  return cm;
}



int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max
   return pulse;
}


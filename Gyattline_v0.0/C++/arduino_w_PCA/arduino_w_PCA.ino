#include <HCSR04.h> //ultrasonic library
#include <Wire.h> //i2c library
#include <Adafruit_PWMServoDriver.h> //servo hub library 
#include <ArduinoJson.h> //arduino json library

//ServoDriver
Adafruit_PWMServoDriver servohub = Adafruit_PWMServoDriver(0x40); //object servo hub, uses the 0x40 adress
#define servomin 100 //Adjust according to your servo
#define servomax 600 //Adjust according to your servo


int time_elapsed;

int angleToPulse(int ang){
    int pulse = map(ang,0, 180, servomin,servomax);// map angle of 0 to 180 to Servo min and Servo max
    return pulse;
  }
  
  /*
  Se velocita vale -100, la funzione restituirà il valore servomin (100).
Se velocita vale 100, restituirà servomax (600).
Se velocita vale 0, restituirà un valore intermedio, tipicamente il punto medio tra servomin e servomax (ad esempio, circa 350).
  */

//motori anteriori
void avviaservoavanti(int velocita, int velocita2){ //move front servos
  servohub.setPWM(0, 0, map(velocita, -100, 100, servomin, servomax));  //slot 0
  servohub.setPWM(1, 0, map(velocita2, -100, 100, servomin, servomax)); //slot 1
}

//motori posteriori
void avviaservodietro(int velocita, int velocita2){ //move back servos
  servohub.setPWM(2, 0, map(velocita, -100, 100, servomin, servomax));  //slot 2
  servohub.setPWM(3, 0, map(velocita, -100, 100, servomin, servomax));  //slot 3

}

//avviamento dei motori
void avviomotori(int dx, int sx, int lim = 100){  //start servos
  dx = constrain(dx, -lim, lim);
  sx = constrain(sx, -lim, lim);
  avviaservoavanti(dx+10, -1*sx+10);
  avviaservodietro(dx+10, -1*sx+10);
}

//omniwheels
void avvio_omni(int dx, int sx, int lim = 100){
  dx = constrain(dx, -lim, lim);
  sx = constrain(sx, -lim, lim);
  avviaservoavanti(dx+10, -1*sx+10);
  dx = constrain(dx, 0, lim);
  sx = constrain(sx, 0, lim);
  avviaservodietro(dx+10, -1*sx+10);
}

//imposta i servo
void impostaservo(int gradi,int pin){servohub.setPWM(pin, 0, angleToPulse(gradi) );}



//HCSR04 sensore(trigger, echo);
UltraSonicDistanceSensor sensore1(13, 12); //inizializzo sensore ultrasuoni n.1
UltraSonicDistanceSensor sensore2(8, 7); //inizializzo sensore ultrasuoni n.2
UltraSonicDistanceSensor sensore3(4, 2); //inizializzo sensore ultrasuoni n.3

//esempio uso:  int u2 = sensore2.measureDistanceCm();


void invia_sensori()
{
      String output;
      StaticJsonDocument<200> responseDoc;
      responseDoc["action"] = "sensors";
      responseDoc["data"]["front"] = sensore1.measureDistanceCm();
      responseDoc["data"]["left"] = sensore2.measureDistanceCm();
      responseDoc["data"]["right"] = sensore3.measureDistanceCm();
      serializeJson(responseDoc, output);
      Serial.println(output);
}

void setup() {
  Serial.begin(115200);
  servohub.begin();        //starts the servohub
  servohub.setPWMFreq(60); //frequenza servohb (60ms)
  avviomotori(0, 0);        //starts to stop
  time_elapsed = millis();
}



void loop() {
  if(Serial.available()>0){ 

    //messages
    String Message = Serial.readStringUntil('\n'); //read until new line


    StaticJsonDocument<200> doc;  // buffer json
    DeserializationError error = deserializeJson(doc, Message);
    if(error){
      Serial.print("Errore nella deserializzazione: "); Serial.println(error.c_str()); //error debuggin
      return;
    }

    //action
    String Action = doc["action"].as<String>(); //starts the command line after the world 'action'

    //logic
    if(Action == "stop") avviomotori(0, 0); //completly stops servos
    else if(Action == "motors") avvio_omni(doc["data"][0], doc["data"][1]); //(motoredx, motoresx) reads directly the data by ArduinoJson
    else if( (Action == "get_sensors") )  {
        invia_sensori();
    }

  }
  //controllo sensori
  int front_distance = sensore1.measureDistanceCm();
  if(front_distance > 15 || front_distance == 0)
      time_elapsed = millis();

    
  if ((millis()-time_elapsed)>1000) //mille millisecondi
      invia_sensori();
}


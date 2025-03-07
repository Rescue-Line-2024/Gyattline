import cv2
import numpy as np
import time
import logging
from PID import gpPID
from ric_colori import RiconosciColori

logging.basicConfig(level=logging.DEBUG)


#############################################
#           Arduino Manager               #
#############################################
class ArduinoManager:
    """
    Gestisce la comunicazione con l'Arduino:
    - Invio comandi ai motori
    - Richiesta aggiornamento sensori
    - Gestione della schivata ostacoli
    """
    message = None
    front_sensor = None
    left_sensor = None
    right_sensor = None
    def __init__(self, motor_limit=30):
        self.motor_limit = motor_limit
        ArduinoManager.message = None
        # Valori aggiornati dal thread seriale
        self.last_obstacle_position = None
        self.pid_wall = gpPID(10, 0, 0, -1, 7)

    def limit_motor(self, motor):
        if motor > self.motor_limit:
            return self.motor_limit
        if motor < -self.motor_limit:
            return -self.motor_limit
        return motor

    def send_motor_commands(self, motor_dx, motor_sx):
        motor_dx = self.limit_motor(motor_dx)
        motor_sx = self.limit_motor(motor_sx)
        ArduinoManager.message = {"action": "motors", "data": [motor_dx, motor_sx]}
        #logging.debug(f"Invio comandi motori: DX={motor_dx}, SX={motor_sx}")

    def request_sensor_data(self):
        ArduinoManager.message = {"action": "get_sensors", "data": " "}
        
        
    def handle_obstacle(self, obstacle_sleep=1):
        """
        Se il sensore frontale rileva un ostacolo (ad esempio, distanza < 15 cm),
        esegue la procedura di schivata.
        """
        if self.front_sensor is not None and 0 < self.front_sensor < 10:
            logging.info("Ostacolo rilevato!")
            # Ferma i motori
            self.send_motor_commands(0, 0)
            time.sleep(0.05)
            self.send_motor_commands(0, 0)
            time.sleep(0.05)
            
            # Richiedi aggiornamento dei sensori
            self.request_sensor_data()
            time.sleep(0.1)
            
            if self.left_sensor is None or self.right_sensor is None:
                logging.warning("Sensori laterali non disponibili!")
                return False
            
            # Scegli la direzione in base allo spazio laterale
            direction = "right" if self.right_sensor > self.left_sensor else "left"
            if self.right_sensor < 1:
                direction = "left"
            elif self.left_sensor < 1:
                direction = "right"
            
            logging.info(f"Schivata ostacolo: giro verso {direction}")
            # Muovi all'indietro per un attimo
            self.send_motor_commands(-self.motor_limit, -self.motor_limit)
            print("INDIETRO!")
            time.sleep(obstacle_sleep)
            # Ruota in base alla direzione scelta
            if direction == "right":
                self.send_motor_commands(-self.motor_limit, self.motor_limit)
                print("DESTRA!!")
                self.pid_wall.inverted = -1
            else:
                self.send_motor_commands(self.motor_limit, -self.motor_limit)
                print("SINISTRA!!!")
                self.pid_wall.inverted = 1

            time.sleep(obstacle_sleep*2)

            self.send_motor_commands(self.motor_limit, self.motor_limit)
            time.sleep(obstacle_sleep/2)
             #in seguilinea procedi a fare il pid con il muro ed esci quando vedi il nero dal cilo
            
            # In un ciclo di correzione potremmo verificare il ripristino della linea
            # (qui semplificato: si esce subito)
            ArduinoManager.front_sensor = None
            ArduinoManager.left_sensor = None
            ArduinoManager.right_sensor = None

            #IMPLEMENTARE PID CON MURO ALTEOVE!!!!
            self.last_obstacle_position = direction
            return True 


    def pass_obstacle(self):
            if ArduinoManager.right_sensor is not None or ArduinoManager.left_sensor is not None:
                self.motor_limit = 25
                sensor = ArduinoManager.right_sensor if self.last_obstacle_position == "left" else ArduinoManager.left_sensor
                print("sensore usato:",sensor)
                dev = self.pid_wall.calcolopid(sensor)
                dev = np.clip(dev,-100,100)
                motor_dx, motor_sx = self.pid_wall.calcolapotenzamotori(dev)
                print("motori:",motor_dx,motor_sx)
                self.send_motor_commands(motor_dx, motor_sx)
                

            
            
        


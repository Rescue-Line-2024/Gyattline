import cv2
import numpy as np
import time
import logging
from PID import gpPID
from ric_colori import RiconosciColori
from threading import Lock

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

    Tutti gli attributi e i metodi sono definiti a livello di classe,
    per cui non è necessario (e non si deve) creare istanze.
    """
    message = None
    message_lock = Lock()  # Lock per proteggere l'accesso a `message`

    front_sensor = None
    left_sensor = None
    right_sensor = None
    motor_limit = 30
    last_obstacle_position = None
    motor_state = True
    obstacle_counter = 0
    camera_grad = 160
    pid_wall = gpPID(10, 0, 0, -1, 5)

    @classmethod
    def limit_motor(cls, motor):
        if motor > cls.motor_limit:
            return cls.motor_limit
        if motor < -cls.motor_limit:
            return -cls.motor_limit
        return motor

    @classmethod
    def send_motor_commands(cls, motor_dx, motor_sx):
        motor_dx = cls.limit_motor(motor_dx)
        motor_sx = cls.limit_motor(motor_sx)

        with cls.message_lock:  # Acquisisce il lock
            cls.message = {"action": "motors", "data": [motor_dx, motor_sx]}
        #logging.debug(f"Invio comandi motori: DX={motor_dx}, SX={motor_sx}")

    @classmethod
    def request_sensor_data(cls):

        with cls.message_lock:  # Acquisisce il lock
            cls.message = {"action": "get_sensors", "data": " "}
        #logging.debug("Richiesta dati sensori.")


    @classmethod
    def detect_obstacle(cls):
        if cls.front_sensor is not None and cls.front_sensor < 15:
            return True
        
    @classmethod
    def handle_obstacle(cls, obstacle_sleep=1):
        
            logging.info("Ostacolo rilevato!")
            # Ferma i motori
            cls.send_motor_commands(0, 0)
            time.sleep(0.05)
            cls.send_motor_commands(0, 0)
            time.sleep(0.05)
            
            # Richiedi aggiornamento dei sensori
            cls.request_sensor_data()
            time.sleep(0.1)
            
            if cls.left_sensor is None or cls.right_sensor is None:
                logging.warning("Sensori laterali non disponibili!")
                return False
            
            # Scegli la direzione in base allo spazio laterale
            direction = "right" if cls.right_sensor > cls.left_sensor else "left"
            if cls.right_sensor < 1:
                direction = "left"
            elif cls.left_sensor < 1:
                direction = "right"
            
            logging.info(f"Schivata ostacolo: giro verso {direction}")
            # Muovi all'indietro per un attimo
            cls.send_motor_commands(-cls.motor_limit, -cls.motor_limit)
            print("INDIETRO!")
            time.sleep(obstacle_sleep*0.5)
            # Ruota in base alla direzione scelta
            if direction == "right":
                cls.send_motor_commands(cls.motor_limit, -cls.motor_limit)
                print("DESTRA!!")
                cls.pid_wall.inverted = -1
            else:
                cls.send_motor_commands(-cls.motor_limit, cls.motor_limit)
                print("SINISTRA!!!")
                cls.pid_wall.inverted = 1

            time.sleep(obstacle_sleep * 1.7)

            cls.send_motor_commands(cls.motor_limit, cls.motor_limit)
            time.sleep(obstacle_sleep * 1)
            # In un ciclo di correzione si potrebbe verificare il ripristino della linea
            # (qui semplificato: si esce subito)
            cls.front_sensor = None
            cls.left_sensor = None
            cls.right_sensor = None

            cls.last_obstacle_position = direction

    @classmethod
    def pass_obstacle(cls):
        if cls.right_sensor is not None or cls.left_sensor is not None:
            cls.motor_limit = 25
            sensor = cls.right_sensor if cls.last_obstacle_position == "left" else cls.left_sensor
            print("sensore usato:", sensor)
            dev = cls.pid_wall.calcolopid(sensor)
            dev = np.clip(dev, -100, 100)
            
            motor_dx, motor_sx = (10,25) if sensor > 4 else (25,0)
            if sensor > 10:
                motor_dx, motor_sx = (0,25)

            if cls.last_obstacle_position == "left":
                temp = motor_dx
                motor_dx = motor_sx
                motor_sx = temp
                
        
                
            print("motori:", motor_dx, motor_sx)
            cls.send_motor_commands(motor_dx, motor_sx)
            time.sleep(0.1)
            cls.request_sensor_data()
            time.sleep(0.1)

    @classmethod
    def set_camera(cls,grad):
        cls.camera_grad = grad
        with cls.message_lock:
            print("modificando posizione telecamera dentro la f set camera")
            cls.message = {"action" : "set_camera" , "data" : int(grad)}

    @classmethod
    def send_message(cls,action,data):
        with cls.message_lock:
            cls.message = {"action" : action , "data" : data}




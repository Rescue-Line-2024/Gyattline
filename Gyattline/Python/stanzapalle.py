import time
import cv2
import numpy as np
from ArduinoManager import ArduinoManager
from ric_colori import RiconosciColori
from ultralytics import YOLO
import os,sys
from PID import gpPID

class BallsController:
    def __init__(self):
        self.sensor_timer = time.time()

        current_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
        model_dir =  os.path.join(current_dir, "my_model.pt")
        self.model_path = model_dir
        print(self.model_path)

        self.width = 320  # Valore di default se il frame non viene catturatoas
        self.height = 240
        
        self.model = YOLO(self.model_path)
        # PID per la correzione durante la raccolta o il deposito
        self.pid = gpPID(P=1.4, I=0, D=0, setpoint=self.width / 2)
        self.wall_pid = gpPID(P=12, I=0, D=0, setpoint=12)

        self.sensor_request_interval = 0.2
        
        # Flag per gestire la modalità: True => raccolta palline, False => deposito
        self.inseguendo_pallina = True  
        # Tempo dell'ultima scansione per eseguire una rotazione
        self.last_scan_time = time.time()
        # Intervallo di tempo (in secondi) fra una scansione e l'altra
        self.scan_interval = 5  
        # Durata della rotazione di scansione

        self.scan_duration = 6 
        '''
        Se il muro è a destra: 3 secondi a sinistra e poi 3 a destra
        else: 3 secondi a destra e poi 3 a sinistra
        '''
        self.intervallo_verdi = ([40, 80, 80], [85, 255, 255])
        self.ric_cassonetto_verde = RiconosciColori(self.intervallo_verdi[0],self.intervallo_verdi[1],10)

        self.scanning_active = False
        self.ball_found = False
        self.bin_found = False

    def riconosci_palla_argento(self, frame):
        """
        Rileva la pallina (argento o nera) nel frame.
        Se rilevata, esegue la logica di inseguimento.
        Restituisce True se la pallina è stata individuata.
        """
        results = self.model(frame)
        for result in results:
            for box in result.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                if conf > 0.7 and (cls == 1 or cls == 0):
                    x1, y1, x2, y2 = box.xyxy[0]
                    center_x = (x1 + x2) / 2.0
                    # Calcola la correzione PID in base alla posizione
                    correzione = int(self.pid.calcolopid(center_x))
                    # Disegna la bounding box per debug
                    pt1 = (int(x1), int(y1))
                    pt2 = (int(x2), int(y2))
                    cv2.rectangle(frame, pt1, pt2, (0, 255, 0), 2)
                    cv2.putText(frame, f"Deviazione: {correzione}", (pt1[0], pt1[1]-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # Se la pallina è "vicina" (ad esempio, se la parte inferiore della bbox supera una soglia)
                    if y2 > 220:
                        print("Pallina vicina: la prendo!")
                        # Esegue la procedura per prendere la pallina
                        self.prendi_pallina()
                        self.inseguendo_pallina = False
                        return True
                    else:
                        # Regola i motori per inseguire la pallina
                        DX, SX = self.pid.calcolapotenzamotori(correzione)
                        print(f"seguendo pallina ; MOTORI : DX-{DX} SX-{SX}")
                        ArduinoManager.send_motor_commands(DX, SX)
                        return True
        return False

    def insegui_cassonetto_verde(self, frame):
        """
        Rileva il cassonetto verde nel frame.
        Se rilevato, esegue la procedura di deposito della pallina.
        Restituisce True se il deposito è stato effettuato.
        """
        mask_verde = self.ric_cassonetto_verde.riconosci_colore_mask(frame)
        contours, _ = cv2.findContours(mask_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            print("Nessun cassonetto verde trovato")
            return False
        
        # Unisce i contorni e crea un rettangolo
        contours = np.vstack(contours).astype(np.int32)
        x, y, w, h = cv2.boundingRect(contours)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Se il cassonetto è sufficientemente vicino (ad esempio, in base a w e posizione verticale)
        if y + h > 230 and w > 280:
            print("Cassonetto vicino: deposito in corso!")
            self.inseguendo_pallina = True
            self.deposita_pallina()
            return True
        else:
            # Regola i motori per inseguire il cassonetto
            cx = (x + x  + w) // 2
            error = self.pid.calcolopid(cx)
            DX, SX = self.pid.calcolapotenzamotori(error, 15)
            ArduinoManager.send_motor_commands(DX, SX)
            return True

    def prendi_pallina(self):
        
        """
        Procedura per prendere la pallina.
        """
        ArduinoManager.send_motor_commands(-15, -15)
        time.sleep(1.5)
        ArduinoManager.send_message("pinza", "chiudi_braccia")
        print("Abbasso braccia")
        time.sleep(2)
        ArduinoManager.send_motor_commands(15, 15)
        time.sleep(6)
        ArduinoManager.send_message("pinza", "chiudi_mani")
        print("Chiudo mani")
        time.sleep(1)
        ArduinoManager.send_message("pinza", "apri_braccia")
        print("Alzo braccia")
        time.sleep(1)

    def deposita_pallina(self):
        """
        Procedura per depositare la pallina.
        """
        print("Deposito della pallina: apro le mani")
        ArduinoManager.send_motor_commands(15, 15)
        time.sleep(3)
        ArduinoManager.send_message("pinza", "chiudi_braccia")
        time.sleep(0.2)
        ArduinoManager.send_message("pinza", "apri_mani")
        time.sleep(0.5)
        ArduinoManager.send_message("pinza", "apri_braccia")
        time.sleep(1)

    def scan(self,direction):
        if ArduinoManager.last_obstacle_position == "right":
            ArduinoManager.send_motor_commands(motor_dx=20*direction, motor_sx=-20*direction)
        else:
            ArduinoManager.send_motor_commands(motor_dx=-20*direction, motor_sx=20*direction)

    def turn(self):
        print("ho un muro d'avanti! mi giro ")
        if ArduinoManager.last_obstacle_position == "right":
            ArduinoManager.send_motor_commands(motor_dx=20, motor_sx=-20) #gira a destra
        else:
            ArduinoManager.send_motor_commands(motor_dx=-20, motor_sx=20) #gira a sinistra
        time.sleep(2)



    def main(self, frame, dimensions):
            self.width, self.height_frame = dimensions
            current_time = time.time()

            if time.time() - self.sensor_timer > self.sensor_request_interval: #ogni tanto richiedi i sensori
                ArduinoManager.request_sensor_data()
                print(f"Front : {ArduinoManager.front_sensor} Left : {ArduinoManager.left_sensor} Right : {ArduinoManager.right_sensor}")
                self.sensor_timer = time.time()

            # Gestione della scansione non bloccante
            if not self.scanning_active and (current_time - self.last_scan_time > self.scan_interval):
                self.scanning_active = True
                self.scan_start_time = current_time

            #fai lo scan,ma se hai trovato o pallina o cassonetto non ne hai motivo di farlo
            if self.scanning_active and not self.ball_found and not self.bin_found:

                if current_time - self.scan_start_time < 3:
                    print("allontanandomi dal muro")
                    self.scan(1) #mi allontano dal muro
                else:
                    print("riavvicinandomi")
                    self.scan(-1) #inverto direzione e torno verso il muro
                    
                # Se la durata di scansione è terminata, ferma la rotazione
                if current_time - self.scan_start_time >= self.scan_duration:
                    self.scanning_active = False
                    self.last_scan_time = current_time

            if self.inseguendo_pallina:
                self.bin_found = False
                self.ball_found = self.riconosci_palla_argento(frame)
                if self.ball_found:
                    pass #bho ci pensa la funzione ad inseguire la pallina e prenderla
                elif not self.scanning_active: # se non sto scannerizzando, pid col muro
                    print("pid muro")
                    if (ArduinoManager.left_sensor is not None and ArduinoManager.right_sensor is not None):

                        value =( 
                         ArduinoManager.left_sensor if ArduinoManager.last_obstacle_position == "left"
                         else ArduinoManager.right_sensor)
                        
                        correction = self.wall_pid.calcolopid(value)
                        right_motor,left_motor = self.wall_pid.calcolapotenzamotori(correction)
                        print(f"PID MURO value = {value} MOTORI : DX-{right_motor} SX-{left_motor}")
                        ArduinoManager.send_motor_commands(right_motor, left_motor)
                    else:
                        ArduinoManager.send_motor_commands(20, 20)
            else:
                self.ball_found = False
                self.bin_found = self.insegui_cassonetto_verde(frame)
                if self.bin_found:
                    pass # ci pensa la funzione sopra ad inseguire il cassonetto e depositare pallina
                elif not self.scanning_active: #se non sto scannerizzando, pid col muro
                    print("pid muro")
                    if (ArduinoManager.left_sensor is not None and ArduinoManager.right_sensor is not None):
                        value =( 
                         ArduinoManager.left_sensor if ArduinoManager.last_obstacle_position == "left"
                         else ArduinoManager.right_sensor)
                                                
                        correction = self.wall_pid.calcolopid(value)
                        right_motor,left_motor = self.wall_pid.calcolapotenzamotori(correction)
                        print(f"PID MURO value = {value} MOTORI : DX-{right_motor} SX-{left_motor}")
                        ArduinoManager.send_motor_commands(right_motor, left_motor)
                    else:
                        ArduinoManager.send_motor_commands(20, 20)

            time.sleep(0.1)

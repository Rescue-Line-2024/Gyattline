import time
import json
import cv2
import os
import sys
import numpy as np
from ultralytics import YOLO  # Libreria per il rilevamento oggetti (YOLO)
from PID import gpPID       # Classe PID per il controllo dell'allineamento
from ArduinoManager import ArduinoManager
from ric_colori import RiconosciColori

class BallsController:
    def __init__(self):
        """
        Inizializza il sistema:
         - cap: oggetto VideoCapture (es. cv2.VideoCapture(0))
         - model_path: percorso del modello YOLO
        """

        current_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
        model_dir =  os.path.join(current_dir, "my_model.pt")
        self.model_path = model_dir
        print(self.model_path)
        self.width = 320  # Valore di default se il frame non viene catturatoas
        self.height = 240
        
        self.model = YOLO(self.model_path)
        # Inizializza il PID con il setpoint al centro del frame
        self.pid = gpPID(P=1.4, I=0, D=0, setpoint=self.width / 2)
        
        # Soglia per determinare se la palla è "vicino" (in percentuale della w della telecamera)
        self.ball_threshold = 0.5 
        self.green_threshold = 0.9

        self.inseguendo_pallina = True #se è false allora 
        self.intervallo_verdi = ([35, 50, 30], [85, 255, 255])

        self.timer_obiettivo = 0


        self.ric_cassonetto_verde = RiconosciColori(self.intervallo_verdi[0],self.intervallo_verdi[1],10)


    def prendi_pallina(self):
        ArduinoManager.send_motor_commands(-15,-15)
        time.sleep(1.5)
        ArduinoManager.send_message("pinza","chiudi_braccia")
        print("abbasso braccia")
        time.sleep(2)
        ArduinoManager.send_motor_commands(15,15)
        time.sleep(6)
        ArduinoManager.send_message("pinza","chiudi_mani")
        print("chiudo_mani")
        time.sleep(1)
        ArduinoManager.send_message("pinza","apri_braccia")
        print("alzo braccia")
        time.sleep(1)

    def deposita_pallina(self):
        print("Rilascio della pallina: apro le mani")
        ArduinoManager.send_message("pinza", "chiudi_braccia")
        time.sleep(0.2)
        ArduinoManager.send_message("pinza", "apri_mani")
        time.sleep(0.5)
        ArduinoManager.send_message("pinza","apri_braccia")
        time.sleep(1)

    def riconosci_palla_argento(self, frame):
        """
        Analizza il frame per rilevare la palla argentata (classe 1).
        Se la palla viene rilevata:
         - Disegna la bounding box attorno alla palla
         - Calcola la deviazione tramite il PID
         - Ritorna un dizionario con il colore, se la palla è vicina e la correzione PID
        """
        results = self.model(frame)
        for result in results:
            for box in result.boxes:
                self.timer_obiettivo = time.time()
                cls = int(box.cls[0])      # Classe 0 = palla nera, 1 = palla argentata
                conf = float(box.conf[0])
                if conf > 0.7 and (cls == 1 or cls == 0):
                    x1, y1, x2, y2 = box.xyxy[0]
                    center_x = (x1 + x2) / 2.0
                    bbox_width = x2 - x1
                    # Calcola la correzione PID in base alla posizione orizzontale
                    correzione = int(self.pid.calcolopid(center_x))
                    
                    # Disegna la bounding box attorno alla palla (in verde)
                    pt1 = (int(x1), int(y1))
                    pt2 = (int(x2), int(y2))
                    cv2.rectangle(frame, pt1, pt2, (0, 255, 0), 2)
                    # Visualizza la correzione sul frame
                    cv2.putText(frame, f"Deviazione: {correzione}", (pt1[0], pt1[1]-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # La palla è considerata "vicina" se la larghezza della bbox supera la soglia
                    vicino = y2 > 220
                    print("y2:",y2)
                    if vicino:
                        if ArduinoManager.camera_grad == 200:
                            print("SONO VICINOOOO!!!!!!!")
                            '''
                            if center_x > self.width//2 + 30:
                                ArduinoManager.send_motor_commands(-7,7)
                            elif center_x < self.width//2 - 30:
                                ArduinoManager.send_motor_commands(7,-7)
                            else:

                            '''
                            self.prendi_pallina()
                            self.inseguendo_pallina = False
                            ArduinoManager.set_camera(205)
                            time.sleep(0.1)
                            return
                        ArduinoManager.set_camera(200)
                        ArduinoManager.send_motor_commands(0,0)
                        time.sleep(0.2)
                        
                    

                    DX,SX = self.pid.calcolapotenzamotori(correzione)
                    print("motori:",DX,"  ",SX)
                    ArduinoManager.send_motor_commands(DX,SX)
                    return
                
        
                    

    def insegui_cassonetto_verde(self, frame):
        mask_verde = self.ric_cassonetto_verde.riconosci_colore_mask(frame)

        # Trova i contorni nella maschera
        contours, _ = cv2.findContours(mask_verde, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:  # Se non trova contorni, esce dalla funzione
            print("Nessun cassonetto verde trovato")
            return

        self.timer_obiettivo = time.time()

        # **Combina i contorni e trova un rettangolo unico**
        contours = np.vstack(contours).astype(np.int32)  # Converte in interi
        x, y, w, h = cv2.boundingRect(contours)

        # Disegna la bounding box unica
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Calcola il centro del rettangolo
        cx = x + w // 2

        # Se è vicino abbastanza, deposita la pallina
        vicino = y + h > 230 and w > 280
        if vicino:
            if ArduinoManager.camera_grad == 200:
                print("Deposita pallina!!!")
                self.deposita_pallina()
                self.inseguendo_pallina = True
                ArduinoManager.set_camera(200)
                time.sleep(0.1)
                return
            ArduinoManager.set_camera(200)
            ArduinoManager.send_motor_commands(0,0)
            time.sleep(0.2)
            
        # Regola il movimento con PID
        error = self.pid.calcolopid(cx)
        DX, SX = self.pid.calcolapotenzamotori(error, 15)
        ArduinoManager.send_motor_commands(DX, SX)


    def main(self,frame,dimensions):
        self.width,self.height_frame = dimensions
        if self.inseguendo_pallina == True:
            self.riconosci_palla_argento(frame)
        else:
            self.insegui_cassonetto_verde(frame)

        if time.time() - self.timer_obiettivo > 3: #se hai perso la pallina/cassonetto da 3 sec
            print("cercando palline(o cassonetto) . . .")
            ArduinoManager.send_motor_commands(-25,25)

        if time.time() - self.timer_obiettivo > 6: #se da 10 sec,abbassa la telecamera
            ArduinoManager.send_motor_commands(25,25)
            time.sleep(0.5)
            self.timer_obiettivo = time.time() -3 
                
        time.sleep(0.1)           
        

    


        



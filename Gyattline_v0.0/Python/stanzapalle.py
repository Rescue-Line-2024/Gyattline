import time
import json
import cv2
import os
import sys
from ultralytics import YOLO  # Libreria per il rilevamento oggetti (YOLO)
from PID import gpPID       # Classe PID per il controllo dell'allineamento

class RobotController:
    def __init__(self, cap):
        """
        Inizializza il sistema:
         - cap: oggetto VideoCapture (es. cv2.VideoCapture(0))
         - model_path: percorso del modello YOLO
        """
        current_dir = os.path.dirname(os.path.abspath(sys.argv[0]))
        model_dir =  os.path.join(current_dir, "my_model.pt")
        self.model_path = model_dir
        print(self.model_path)
        self.cap = cap
        ret, frame = self.cap.read()
        if ret:
            self.width = frame.shape[1]
        else:
            self.width = 640  # Valore di default se il frame non viene catturato
        self.model = YOLO(self.model_path)
        # Inizializza il PID con il setpoint al centro del frame
        self.pid = gpPID(P=2, I=0.01, D=0.1, setpoint=self.width / 2)
        
        # Soglia per determinare se la palla è "vicino"
        self.ball_threshold = 200

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
                cls = int(box.cls[0])      # Classe 0 = palla nera, 1 = palla argentata
                conf = float(box.conf[0])
                if conf > 0.5 and (cls == 1 or cls == 0):
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
                    vicino = bbox_width > self.ball_threshold
                    return {"color": "silver", "vicino": vicino, "correzione": correzione}
        return None

    def run(self):
        """
        Ciclo principale che:
         - Cattura continuamente i frame dalla webcam
         - Analizza il frame per rilevare la palla argentata
         - Visualizza il frame aggiornato con le bounding box disegnate
        """
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            ball_info = self.riconosci_palla_argento(frame)
            if ball_info:
                print("Palla rilevata:", ball_info)
            else:
                print("Palla non rilevata")
            
            cv2.imshow("Frame", frame)
            # Premere 'q' per uscire dal loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

# Esempio di utilizzo:
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    robot = RobotController(cap)
    robot.run()

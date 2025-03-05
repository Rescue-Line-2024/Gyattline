import cv2
import numpy as np
import time
import logging
from PID import gpPID
from ric_colori import RiconosciColori
from ArduinoManager import ArduinoManager
from LineGreenAnalyzer import LineGreenAnalyzer
from PID_manager import PIDManager

#############################################
#              SeguiLinea                 #
#############################################
class Seguilinea:
    """
    Classe principale che coordina:
     - L'analisi immagine (linea e marker verdi)
     - I calcoli PID per il follow della linea
     - La comunicazione con l'Arduino per comandare i motori
    """
    def __init__(self, cam, pid_params, P2, pen_multiplier, cam_resolution,
                 min_area=200, cut_percentage=0.6, motor_limit=30):
        self.cam = cam
        self.cut_percentage = cut_percentage
        self.cam_x = int(cam_resolution[0])
        self.cam_y = int(cam_resolution[1])
        self.motor_limit = motor_limit
        self.cut_y = None
        # Inizializza i manager
        self.arduino_manager = ArduinoManager(motor_limit=motor_limit)
        P, I, D = pid_params
        self.pid_manager = PIDManager(P, I, D, setpoint=int(self.cam_x/2), P2=P2, pen_multiplier=pen_multiplier)
        self.line_analyzer = LineGreenAnalyzer(min_area=min_area)
        self.last_green_direction = None
        self.avoiding_obstacle = False

        self.sensor_timer = time.time()
        self.sensor_request_interval = 0.5
        self.w = 0
    def follow_line(self, frame):
        frame_height, frame_width = frame.shape[:2]
        # Rileva la linea nell'immagine completa (o in quella tagliata)
        line_bboxes = self.line_analyzer.detect_line(frame, frame_height, self.cut_percentage)
        if line_bboxes is not None:
            x, y, self.w, h = line_bboxes[0]


        
        if time.time() - self.sensor_timer > self.sensor_request_interval: #ogni tanto richiedi i sensori
            self.arduino_manager.request_sensor_data()
            #print(f"Front : {ArduinoManager.front_sensor} Left : {ArduinoManager.left_sensor} Right : {ArduinoManager.right_sensor}")
            self.sensor_timer = time.time()
            return
        
        if(self.arduino_manager.handle_obstacle(1.3) == True) and self.avoiding_obstacle == False:
                #incomincia schivata ostacolo
                self.avoiding_obstacle = True

        if self.avoiding_obstacle == True:
            self.sensor_request_interval = 0.1
            self.arduino_manager.pass_obstacle()

            if self.w > 400:
                print("ostacolo schivato!")
                ArduinoManager.motor_limit = 35
                self.sensor_request_interval = 0.5
                self.avoiding_obstacle = False
            else:
                return
            

            
        if line_bboxes is not None:
            # Aggiorno l'altezza (cut_y) della mask ottenuta dal rilevamento della linea
            self.cut_y = self.line_analyzer.binary_mask.shape[0]
            
            # Prendi la prima bounding box rilevata
            x, y, w, h = line_bboxes[0]
            # Ripristino la coordinata y nel sistema completo
            y_original = y + int(self.cam_y * self.cut_percentage)
            adjusted_bbox = (x, y_original, w, h)
            nero_coords = (x, y, w, h)  # coordinate relative per ulteriori elaborazioni

            # Gestione ostacoli (se i sensori li segnalano)
            
            


            # Rileva eventuali marker verdi
            green_positions = self.line_analyzer.detect_green(frame,200)
            green_decision = self.line_analyzer.analyze_green_markers(green_positions, self.cam_y, self.cam_x)
            if green_decision is not None:
                # Aggiorna l'ultima direzione nota per il verde
                self.last_green_direction = green_decision
                if green_decision == "DX":
                    logging.info("Marker verde: gira a destra")
                    self.last_green_direction = "DX"
                    deviation = self.pid_manager.compute_deviation(x+w, h, self.cut_y)
                    motor_dx, motor_sx = self.pid_manager.compute_motor_commands(deviation)
                    self.arduino_manager.send_motor_commands(motor_dx, motor_sx)
                    return
                elif green_decision == "SX":
                    logging.info("Marker verde: gira a sinistra")
                    self.last_green_direction = "SX"
                    deviation = self.pid_manager.compute_deviation(x, h, self.cut_y)
                    motor_dx, motor_sx = self.pid_manager.compute_motor_commands(deviation)
                    self.arduino_manager.send_motor_commands(motor_dx, motor_sx)
                    return
                elif green_decision == "DOPPIO":
                    logging.info("Marker verdi doppi: gestione speciale da implementare")
                    self.last_green_direction = "DOPPIO"
                    deviation = self.pid_manager.compute_deviation(0, h, self.cut_y) #girerà su se stesso
                    motor_dx, motor_sx = self.pid_manager.compute_motor_commands(deviation)
                    self.arduino_manager.send_motor_commands(motor_dx, motor_sx)
                    return
                    # Inserisci qui la logica specifica se necessario.

            # **Nuova logica per le intersezioni**
            # Se non c'è più il verde, controlla se la linea tocca i bordi (intersezioni)
            # - Se x == 0 o x+w coincide con la larghezza del frame, oppure se y == 0
            if (x <= 10) or ((x + w) >= self.cam_x-10): #questo quando c'è un indecisione
                if self.last_green_direction is not None:
                    if self.last_green_direction == "DX":
                        # Se l'ultima direzione verde era a destra, usa il bordo destro della bbox
                        target_x = x + w
                        logging.info("Intersezione: seguo il ramo destro")
                    elif self.last_green_direction == "SX":
                        # Se era a sinistra, usa il bordo sinistro
                        target_x = x
                        logging.info("Intersezione: seguo il ramo sinistro")
                    else:
                        target_x = 0 #doppio verde,quindi gira su te stesso ;) 
               

                    deviation = self.pid_manager.compute_deviation(target_x, h, self.cut_y)
                    motor_dx, motor_sx = self.pid_manager.compute_motor_commands(deviation)
                    self.arduino_manager.send_motor_commands(motor_dx, motor_sx)
                    return
            else:
                self.last_green_direction = None

            # Se non sono presenti intersezioni, prova a ricavare due centri della linea per il PID avanzato
            points = self.pid_manager.find_line_centers(
                binary_mask=self.line_analyzer.binary_mask, 
                cut_y=self.cut_y, 
                offset=10, 
                start=0,
                cam_y=self.cam_y, 
                cut_percentage=self.cut_percentage
            )
            if points is not None and points not in [0, 3]:
                esito, center_line_x, center_line_y = self.pid_manager.advanced_pid(points, frame, nero_coords)
                if esito == "DESTRA":
                    self.arduino_manager.send_motor_commands(-self.motor_limit, self.motor_limit)
                    return
                elif esito == "SINISTRA":
                    self.arduino_manager.send_motor_commands(self.motor_limit, -self.motor_limit)
                    return
                # In caso di esito "NIENTE", usa il centro della bbox come fallback
                center_line_x = (x + x + w) // 2
                deviation = self.pid_manager.compute_deviation(center_line_x, h, self.cut_y)
            else:
                # Fallback: usa il centro della bbox
                center_line_x = (x + x + w) // 2
                center_line_y = (y_original + y_original + h) // 2
                cv2.circle(frame, (center_line_x, center_line_y), 10, (255, 0, 0), -1)
                deviation = self.pid_manager.compute_deviation(center_line_x, h, self.cut_y)
            
            motor_dx, motor_sx = self.pid_manager.compute_motor_commands(deviation)
            self.arduino_manager.send_motor_commands(motor_dx, motor_sx)

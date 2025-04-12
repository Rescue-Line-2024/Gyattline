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
        ArduinoManager.motor_limit = motor_limit
        P, I, D = pid_params
        self.pid_manager = PIDManager(P, I, D, setpoint=int(self.cam_x/2), P2=P2, pen_multiplier=pen_multiplier)
        self.line_analyzer = LineGreenAnalyzer(min_area=min_area)
        self.last_green_direction = None
        self.avoiding_obstacle = False

        self.sensor_timer = time.time()
        self.sensor_request_interval = 0.5
        self.sensor_counter = 0
        self.w = 0

    def follow_line(self, frame):
        frame_height, frame_width = frame.shape[:2]
        # Rileva la linea nell'immagine completa (o in quella tagliata)
        line_bboxes = self.line_analyzer.detect_line(frame, frame_height, self.cut_percentage)
        if line_bboxes is not None:
            x, y, self.w, h = line_bboxes[0]


        
        if time.time() - self.sensor_timer > self.sensor_request_interval: #ogni tanto richiedi i sensori
            ArduinoManager.request_sensor_data()
            print(f"Front : {ArduinoManager.front_sensor} Left : {ArduinoManager.left_sensor} Right : {ArduinoManager.right_sensor}")
            if ArduinoManager.front_sensor is not None:
                if ArduinoManager.front_sensor < 15:
                    self.sensor_counter +=1
                else:
                    self.sensor_counter = 0

            self.sensor_timer = time.time()
            return
        
        
        if self.avoiding_obstacle == False and self.sensor_counter >= 2:
            if(ArduinoManager.detect_obstacle() == True):
                ArduinoManager.handle_obstacle(0.7)
                #incomincia schivata ostacolo
                print("sto schivando ostacolo")
                self.avoiding_obstacle = True
        
        if ArduinoManager.motor_state == False:
            self.avoiding_obstacle = False
            ArduinoManager.motor_limit = 30
            
        if self.avoiding_obstacle == True:
            print("nel loop dell'ostacolo...")
            self.sensor_request_interval = 0.1
            ArduinoManager.pass_obstacle()
            print(self.w)
            if self.w > 200:
                print("ostacolo schivato!")
                ArduinoManager.motor_limit = 30
                if ArduinoManager.last_obstacle_position == "right":
                    ArduinoManager.send_motor_commands(-25,25)
                else:
                    ArduinoManager.send_motor_commands(25,-25)
                time.sleep(0.5)

                self.sensor_request_interval = 0.5
                self.avoiding_obstacle = False
            else:
                return
        
        

            
        if line_bboxes is not None:
            
            # Aggiorno l'altezza (cut_y) della mask ottenuta dal rilevamento della linea
            self.cut_y = self.line_analyzer.binary_mask.shape[0]

            
            # Prendi la prima bounding box rilevata
            x, y, w, h = line_bboxes[0]

            is_line_centered = x > 100 and x+w < self.cam_x-100 #per vedere se la linea si trova più o meno al centro
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
                    deviation = self.pid_manager.compute_deviation_h((x+w) , h, self.cut_y,is_line_centered)//2
                    cv2.circle(frame, (x+w, self.cut_y), 10, (255, 255, 0), -1)
                    motor_dx, motor_sx = self.pid_manager.compute_motor_commands(deviation)
                    ArduinoManager.send_motor_commands(motor_dx, motor_sx)
                    return
                elif green_decision == "SX":
                    logging.info("Marker verde: gira a sinistra")
                    self.last_green_direction = "SX"
                    deviation = self.pid_manager.compute_deviation_h(x, h, self.cut_y,is_line_centered)//2
                    cv2.circle(frame, (x, self.cut_y), 10, (255, 255, 0), -1)
                    motor_dx, motor_sx = self.pid_manager.compute_motor_commands(deviation)
                    ArduinoManager.send_motor_commands(motor_dx, motor_sx)
                    return
                elif green_decision == "DOPPIO!":
                    logging.info("Marker verdi doppi: gestione speciale da implementare")
                    self.last_green_direction = "DOPPIO"
                    deviation = self.pid_manager.compute_deviation_h(0, h, self.cut_y,is_line_centered) #girerà su se stesso
                    motor_dx, motor_sx = self.pid_manager.compute_motor_commands(deviation)
                    ArduinoManager.send_motor_commands(motor_dx, motor_sx)
                    time.sleep(1.3)
                    return
                    # Inserisci qui la logica specifica se necessario.

            #**Nuova logica per le intersezioni**
            # Se non c'è più il verde, controlla se la linea tocca i bordi (intersezioni)
            # - Se x == 0 o x+w coincide con la larghezza del frame, oppure se y == 0
            
            '''
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
               

                    deviation = self.pid_manager.compute_deviation_h(target_x, h, self.cut_y,is_line_centered)
                    motor_dx, motor_sx = self.pid_manager.compute_motor_commands(deviation)
                    ArduinoManager.send_motor_commands(motor_dx, motor_sx)
                    return
            else:
                self.last_green_direction = None
            '''
            
            # Se non sono presenti intersezioni, prova a ricavare due centri della linea per il PID avanzato
            points = self.pid_manager.find_line_centers(
                binary_mask=self.line_analyzer.binary_mask, 
                cut_y=self.cut_y, 
                offset=5, 
                start=0,
                cam_y=self.cam_y, 
                cut_percentage=self.cut_percentage
            )
            if points is not None and points not in [0, 3]:
                esito, center_line_x, center_line_y = self.pid_manager.advanced_pid(points, frame, nero_coords)
                if esito == "DESTRA":
                    ArduinoManager.send_motor_commands(-self.motor_limit, self.motor_limit)
                    return
                elif esito == "SINISTRA":
                    ArduinoManager.send_motor_commands(self.motor_limit, -self.motor_limit)
                    return
                # In caso di esito "NIENTE", usa il centro della bbox come fallback
                center_line_x = (x + x + w) // 2
                center_line_y = (y_original + y_original + h) // 2
                cv2.circle(frame, (center_line_x, center_line_y), 5, (255, 0, 0), -1)
                deviation = self.pid_manager.compute_deviation_h(center_line_x, h, self.cut_y,is_line_centered)
            else:
                # Fallback: usa il centro della bbox
                center_line_x = (x + x + w) // 2
                center_line_y = (y_original + y_original + h) // 2
                cv2.circle(frame, (center_line_x, center_line_y), 5, (255, 0, 0), -1)
                deviation = self.pid_manager.compute_deviation_h(center_line_x, h, self.cut_y,is_line_centered)
                print("deviazione: ",deviation,"centro:",center_line_x)
            motor_dx, motor_sx = self.pid_manager.compute_motor_commands(deviation)
            ArduinoManager.send_motor_commands(motor_dx, motor_sx)
            

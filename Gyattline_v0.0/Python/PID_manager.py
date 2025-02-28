import cv2
import numpy as np
import time
import logging
from PID import gpPID
from ric_colori import RiconosciColori

#############################################
#             PID Manager                 #
#############################################
class PIDManager:
    """
    Incapsula i controller PID per il follow della linea e per la regolazione in prossimità di un muro.
    Si occupa anche di ricavare i centri della linea (da una maschera binaria) e di calcolare la deviazione.
    """
    def __init__(self, P, I, D, setpoint=0, P2=1.0, pen_multiplier=1.0):
        self.pid_follow = gpPID(P, I, D, 1, setpoint)  # Il setpoint (centro) verrà aggiornato
        self.pid_wall = gpPID(20, 0, 0, -1, 12)
        self.P2 = P2           # Fattore di moltiplicazione per la deviazione
        self.pen_multiplier = pen_multiplier  # Fattore per la “pendenza”

    def compute_deviation(self, center_line_x, bbox_height, total_height):
        deviation = self.pid_follow.calcolopid(center_line_x)
        multiplicator_h = max(0.01, bbox_height / total_height)
        deviation /= multiplicator_h
        logging.debug(f"Deviazione calcolata: {deviation} (moltiplicatore: {multiplicator_h})")
        return deviation * self.P2

    def compute_motor_commands(self, deviation):
        motor_dx, motor_sx = self.pid_follow.calcolapotenzamotori(deviation)
        return motor_dx, motor_sx

    def compute_wall_deviation(self, lateral_distance):
        return self.pid_wall.calcolopid(lateral_distance)

    def find_line_centers(self, binary_mask, cut_y, offset, start, cam_y, cut_percentage):
        """
        Ricava i centri (punti A e B) della linea da una maschera binaria.
        Restituisce una tupla (A, B) oppure 0/3 in caso di rilevamenti particolari.
        """
        if binary_mask is None:
            logging.error("Maschera binaria assente in find_line_centers")
            return None

        # Ritaglia le parti inferiore e superiore della maschera
        down_part = binary_mask[cut_y - offset:cut_y, :]
        up_part = binary_mask[start:start + offset, :] if (start + offset) < cut_y else binary_mask[start:cut_y, :]

        if down_part.size == 0 or up_part.size == 0:
            return None

        # Trova i contorni nelle due parti
        contours_down, _ = cv2.findContours(down_part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_up, _ = cv2.findContours(up_part, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_down and contours_up:
            bboxes_down = [cv2.boundingRect(c) for c in contours_down if cv2.contourArea(c) > 10]
            bboxes_up = [cv2.boundingRect(c) for c in contours_up if cv2.contourArea(c) > 10]

            if bboxes_down and bboxes_up and (len(bboxes_down) + len(bboxes_up)) == 2:
                # Usa la prima bounding box trovata in ciascuna parte
                x1, y1, w1, h1 = bboxes_down[0]
                x2, y2, w2, h2 = bboxes_up[0]

                # Aggiusta la coordinata y per tenere conto della parte tagliata
                y1 += int(cam_y * (1-cut_percentage))
                y2 += int(cam_y * (1-cut_percentage))

                y1 += cut_y+offset
                y2 += offset
                

                # Calcola i centri delle bounding box
                A = (int((x1 + x1 + w1) / 2), int((y1 + y1 + h1) / 2))
                B = (int((x2 + x2 + w2) / 2), int((y2 + y2 + h2) / 2))
                
                return A, B
            elif len(bboxes_down) + len(bboxes_up) == 3:
                return 3
            else:
                return 0  # Nessun punto valido trovato
        else:
            return None


    def advanced_pid(self, points, frame, nero_coords):
        """
        Utilizza i due punti rilevati per stabilire se la pendenza (slope) ha la precedenza sulla deviazione.
        Restituisce:
         - esito: "DESTRA", "SINISTRA" oppure "NIENTE"
         - centro della linea (x,y)
        """
        esito = "NIENTE"
        Cinf, Csup = points
        
        center_line_x = (Cinf[0] + Csup[0]) // 2
        center_line_y = (Cinf[1] + Csup[1]) // 2
        cv2.circle(frame, (center_line_x, center_line_y), 5, (255, 0, 0), -1)
        
        deviation = self.pid_follow.calcolopid(center_line_x)
        try:
            cv2.circle(frame, Cinf, 5, (0, 255, 0), -1)
            cv2.circle(frame, Csup, 5, (0, 0, 255), -1)
        except Exception as e:
            logging.error(f"Errore nel disegnare i cerchi: {e}")
        
        # Calcola la “pendenza” come differenza in x moltiplicata per il fattore
        slope = abs(Csup[0] - Cinf[0]) * self.pen_multiplier
        logging.debug(f"Pendenza calcolata: {slope}")
        
        if abs(deviation) > slope:
            # La deviazione ha la precedenza
            pass
        else:
            if Cinf[0] < Csup[0]:
                logging.info("Ruota a destra (pendenza ha la precedenza)")
                esito = "DESTRA"
            else:
                logging.info("Ruota a sinistra (pendenza ha la precedenza)")
                esito = "SINISTRA"
        
        return esito, center_line_x, center_line_y
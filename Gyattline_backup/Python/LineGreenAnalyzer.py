import cv2
import numpy as np
import time
import logging
from PID import gpPID
from ric_colori import RiconosciColori
#############################################
#         Line & Green Analyzer           #
#############################################
class LineGreenAnalyzer:
    """
    Utilizza la classe RiconosciColori per analizzare l'immagine:
     - Rileva la linea (di solito in nero o colorata)
     - Rileva eventuali marker verdi per indicare svolte
    """
    def __init__(self, min_area=200):
        self.BLUE = ([100, 150, 0], [140, 255, 255])
        self.GREEN = ([35, 100, 50], [85, 255, 255])
        self.min_area = min_area
        self.color_detector = RiconosciColori(self.GREEN[0], self.GREEN[1], self.min_area)
        self.binary_mask = None
        self.timer_doppio = time.time()
        self.last_verde = None
        self.timer_verdi = time.time()

    def detect_line(self, image, frame_height, cut_percentage):
        """
        Rileva la linea nell'area tagliata del frame.
        Restituisce la bounding box (o le bbox) rilevata/e.
        """
        bboxes = self.color_detector.riconosci_nero_tagliato(
            image=image, frame_height=frame_height, cut_percentage=cut_percentage
        )
        # Salva la maschera binaria per eventuali elaborazioni successive.
        self.binary_mask = RiconosciColori.thresh[int(frame_height * cut_percentage):frame_height, :].copy()
        return bboxes

    def detect_green(self, image,minim = 200):
        """
        Rileva i marker verdi nell'immagine.
        """
        green_positions = self.color_detector.riconosci_verdi(image=image,minim=minim)
        return green_positions

    def analyze_green_markers(self, green_positions, cam_y, cam_x):
        """
        Analizza i marker verdi rilevati e, se presenti nell'area bassa, restituisce
        un'indicazione (ad esempio "DX", "SX" o "DOPPIO") per indicare una svolta.
        """


        if green_positions is None:
            self.timer_verdi = time.time()
            return None

        if len(green_positions) != 2: 
            self.timer_verdi = time.time()

        valid_greens = []
        for green in green_positions:
            x, y, w, h = green["coords"]
            if (y + h) > cam_y * 0.7:  # Considera solo quelli nella parte inferiore
                valid_greens.append(green)
        if len(valid_greens) == 1:
            self.last_verde = valid_greens[0]["position"]
            return valid_greens[0]["position"]
        elif len(valid_greens) == 2:
            if time.time()-self.timer_verdi > 1:
                return "DOPPIO!"
            else:
                return self.last_verde


        else:
            return None
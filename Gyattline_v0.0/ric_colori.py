import numpy as np
import cv2

class RiconosciColori:
    def __init__(self, bottom_value,upper_value):
        self.bottom_value = np.array(bottom_value)
        self.upper_value = np.array(upper_value)

        #dal main io passerò un array a parametro:quello dell'inizio dell intervallo
        #e quello dela fine dell'intervallo esempio:rosso = RiconosciColori([170,0,0],[255,70,70])
        self.interval = (np.array(bottom_value),np.array(upper_value))

    def riconosci_colore(self, image,min_area=500):
        """Riconosce i colori nell'immagine e restituisce le coordinate dei bounding box."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.bottom_value, self.upper_value)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        array = []
        for contour in contours:
            if cv2.contourArea(contour) > min_area:
                x, y, w, h = cv2.boundingRect(contour)    
                array.append([x, y, w, h])
        
        return array if array else None  # Restituisce None se non ci sono contorni
    
    def disegna_bbox(self,array,image):
        coordinate = array
        for x,y,w,h in coordinate:
            cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,0),2)
        
    def riconosci_nero(self,image,min_area):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        ret, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

        contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

        #filtro eventuali punti neri che danno fastidio
        significant_contours = [contour for contour in contours if cv2.contourArea(contour) > min_area]

        if len(significant_contours) > 0:
            for contour in significant_contours:
                x , y , w, h = cv2.boundingRect(contour)

            bounding_boxes = [(cv2.boundingRect(contour)) for contour in significant_contours]
            return bounding_boxes
        else:
            return None
    
        

                
        
        
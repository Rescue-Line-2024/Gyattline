import numpy as np
import cv2
class COLORIUWU:
    def __init__(self,colore):
        self.colore = colore
        self.colors ={
            
         "red" :   ( np.array([0, 100, 100]),np.array([10, 255, 255]) ),
         
         "red2" :  ( np.array([160, 100, 100]), np.array([180, 255, 255]) ),
         
         "green" : ( np.array([40, 40, 40]), np.array([80, 255, 255]) ),
         
         "black" : ( np.array([0, 0, 0]),np.array([180, 255, 100]) ) 
         
         
        }
        

    
    def RiconosciColore(self,image):
        Min,Max = self.colors[self.colore]
    
            
        
         #è qui il problema!!domani correggere
        hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
   
            
            
        mask = cv2.inRange(hsv,Min,Max)
        array = []
        contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                x,y,w,h = cv2.boundingRect(contour)
                
                if self.colore != "black":
                    cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
                    
                array.append([x,y,w,h])
        return array
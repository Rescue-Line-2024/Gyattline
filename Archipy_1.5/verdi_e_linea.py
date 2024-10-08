import cv2
import numpy as np
from PID import gpPID
import time


cx1 = 160
cx2 = 60

wd = 320
hd = 240

pidnero2 = gpPID(P=1.2, I=0, D=0, setpoint=wd/2)
pidverde = gpPID(P=15 , I=0, D=0, setpoint=cx1)


def treshnero(crop_img,mode,dev1):

    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    ret, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

    contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
    
    for contour in contours:
            # Calcola l'area del contorno
        area = cv2.contourArea(contour)
        if area > 0:
            x, y, w, h = cv2.boundingRect(contour)
        else:
            contours = []

    obiettivo = 0

    if len(contours) > 0:
        cy = (y + (y + h)) / 2
      
        cx = (x+(x+w))/2
       
        
        if w >= 160 and h == hd:
            return 6969
        
        cx = int(cx)
        cy = int(cy)
        cv2.line(crop_img, (cx, 0), (cx, 720), (255, 0, 0), 1)
        cv2.line(crop_img, (0, cy), (1280, cy), (255, 0, 0), 1)

        cv2.drawContours(crop_img, contours, -1, (0, 255, 0), 1)
        
        if mode == "linea":
            return cx
        else:
            return w
    
    else:
        return dev1
    
def RicLinea(crop_img, dev1):
    global cx1, cx2, pidnero2
    crop_img1 = crop_img[hd-70:hd, 0:wd]
    cx1 = treshnero(crop_img1, "linea", dev1)
    controllo_T = treshnero(crop_img.copy(), "linea", 0)
    if controllo_T == 6969:
        cx1 = 0
        
    if cx1 == dev1:  # Se la linea non è rilevata, restituisci un valore speciale per indicarlo
        return dev1
    else:
        deviazione = int(pidnero2.calcolopid(cx1))
        return deviazione

tiradritto = False

def RiconosciVerde(image):
    global tiradritto
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Definisci il range di colore per il verde
    Min = np.array([36, 60, 70])
    Max = np.array([89, 255, 255])

    # Applica una maschera per isolare il verde
    mask = cv2.inRange(hsv, Min, Max)

    # Applica una chiusura per rimuovere i piccoli buchi nel verde
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Trova i contorni
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    arrayverdi = []
    for contour in contours:
        # Calcola l'area del contorno
        area = cv2.contourArea(contour)
        if area > 500:  # Considera solo i contorni con un'area sufficientemente grande
            # Trova il rettangolo delimitatore del contorno
            x, y, w, h = cv2.boundingRect(contour)
            
            crop_img = image.copy()
            crop_img_nero = crop_img[y + h:y + h + 10, x:x + w]
            
            #se il verde è al fondo
            
            if y+h < hd-2:
                cx_nero = treshnero(crop_img_nero,"verde",0)
            else:
                cx_nero = -1
                
            
            if y+h > 120:        
                if cx_nero > 50:
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    continue
                else:
                    arrayverdi.append((x, y, w, h))
                    
                cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)


    if len(arrayverdi) == 0:
        return None
    else:
        return arrayverdi

def GestisciVerdi(image, crop_img):
    arrayverdi = RiconosciVerde(image)
    if arrayverdi == None:
        return None
    
    if len(arrayverdi) > 0:
        if len(arrayverdi) == 1:
            crop_img1 = crop_img[hd-20:hd,0:wd]
            cx1 = treshnero(crop_img1,"linea",0)
            x, y, w, h = arrayverdi[0]
            gx = int((x + x + w) / 2)
            
            #il setpoint è il centro della linea
            Deviazione_verde = pidverde.calcolopid(gx)
                 
            gy= int((y+y+h)/2)
            print("ALTEZZA VERDE",y+h)
            if y+h > 210:
                return Deviazione_verde
            else:
                return None
       
            
        elif len(arrayverdi) == 2:
            x1,y1,w1,h1 = arrayverdi[0]
            x2,y2,w2,h2 = arrayverdi[1]
            
            if x1 < 20 or x1+w1 > wd -20:
                return None
            if x2 < 20 or x2+w2 > wd -20:
                return None
            
            return "DOPPIO"
    else:
        return None
        
    


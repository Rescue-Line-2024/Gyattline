#NEXUS ANTARES Zenith
#OH FRA SE LAGGA IN CODA AL CICLO C'E' UNO SLEEP,TOGLILO

from ric_colori import RiconosciColori
from verdi_e_linea import Seguilinea
import cv2
import time


cam = cv2.VideoCapture(0)
if not cam.isOpened():
    print("Errore nell'apertura della camera.")
else:
    # Ottieni la risoluzione della camera
    width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

#DICHIARAZIONE DEGLI OGGETTI PRINCIPALI DEL SEGUILINEA
Line_follower = Seguilinea(P=3,I=0,D=0,PEN=0.5,min_area=500,cam_resolution=(width,height))
Riconosci_verde = RiconosciColori([35,40,40],[75,255,255])
#DICHIARAZIONE DEGLI OGGETTI PRINCIPALI DEL SEGUILINEAq

while True:
    ret, frame = cam.read()
    frame_colori = frame.copy()
    
    coordinate_nero = Line_follower.segui_linea(frame)
    coordinate_verde = Riconosci_verde.riconosci_colore(frame_colori)
    if coordinate_verde is not None:
        Riconosci_verde.disegna_bbox(coordinate_verde,frame_colori,(0,255,0))

    cv2.imshow("suca",frame)
    cv2.imshow("suca2",frame_colori)
    if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    time.sleep(0.1) #toglierlo in futuro

cam.release()
cv2.destroyAllWindows()


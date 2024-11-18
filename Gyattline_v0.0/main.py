#NEXUS ANTARES Zenit
from ric_colori import RiconosciColori
import cv2

Riconosci_verde = RiconosciColori([35,40,40],[75,255,255])

cam = cv2.VideoCapture(0)
while True:
    ret, frame = cam.read()
    coordinate = Riconosci_verde.riconosci_colore(frame)
    Riconosci_verde.disegna_bbox(coordinate,frame)
    cv2.imshow("suca",frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
                break

cam.release()
cv2.destroyAllWindows()


import cv2
import numpy as np
from PID import gpPID

class pallinadetector():
    def __init__(self, hd, wd):
        self.PIDy = gpPID(P=0.2, I=0, D=0, setpoint=hd/2)
        self.PIDx = gpPID(P=1, I=0, D=0, setpoint=wd/2)
        self.contapalline = 0
        self.hd = hd
        self.wd = wd
        self.green = (np.array([36, 60, 70]), np.array([89, 255, 255]))

    def riconosci_verde(self, image, color):
        Min, Max = color
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, Min, Max)
        array = []
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 255), 2)
                array.append([x, y, w, h])
        return array

    def detectnero(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Limiti per il colore nero
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 45])
        mask = cv2.inRange(hsv_image, lower_black, upper_black)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def detectpallina(self, image):
        arr = []
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Applica un blur per ridurre i falsi positivi (sia Gaussian che mediano)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)
        gray = cv2.medianBlur(gray, 5)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=40,
                                   param1=40, param2=30, minRadius=0, maxRadius=0)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                center = (circle[0], circle[1])
                radius = circle[2]
                cv2.circle(image, center, radius, (0, 255, 0), 2)
                arr.append((center, radius))
            return arr
        else:
            return None


    def distinguipallina(self, image):
        pallineargentate = []
        # Converti l'immagine in HSV per eventuali controlli ROI sui riflessi
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        palline = self.detectpallina(image)
        neri = self.detectnero(image)

        if palline is not None:
            for i in range(len(palline)):
                Cp, r = palline[i]
                Cx, Cy = Cp
                trovato_nero = False
                # Controlla se il centro della pallina coincide con una regione nera
                for j in range(len(neri)):
                    X, Y, W, H = cv2.boundingRect(neri[j])
                    Cn = int((X + X + W) / 2)
                    CHn = int((Y + Y + H) / 2)
                    if Cx - 10 < Cn < Cx + 10 and Cy - 10 < CHn < Cy + 10:
                        trovato_nero = True
                        break

                if trovato_nero:
                    print("Sembra che la pallina sia nera")
                else:
                    print("Sembra che la pallina sia argentata")
                    pallineargentate.append((Cp, r))

                    
        return pallineargentate

    def calcoloerrorepallinay(self, palla):
        try:
            center, radius = palla
            h = center[1]
            deviazione = self.PIDy.calcolopid(h)
            return int(deviazione)
        except Exception as e:
            print("Errore calcolo y:", e)
            return -400  # -400 vuol dire errore

    def calcoloerrorepallinax(self, palla):
        try:
            center, radius = palla
            w = center[0]
            deviazione = self.PIDx.calcolopid(w)
            return int(deviazione)
        except Exception as e:
            print("Errore calcolo x:", e)
            return -400  # -400 vuol dire errore

    def ritornaltezza(self, palla):
        try:
            center, radius = palla
            h = center[1]
            return h
        except Exception as e:
            print("Errore altezza:", e)
            return -400  # -400 vuol dire errore

    def vai_verso_cassonetto(self, image):
        array_verdi = self.riconosci_verde(image, self.green)
        if array_verdi == -1 or len(array_verdi) == 0:
            return 300  # Se non ci sono verdi, gira e cercalo
        x, y, w, h = array_verdi[0]
        if y+h > 230:  # Se il cassonetto è vicino, ritorna 1000 (deposita la pallina)
            return 1000
        Cx = int((x + x + w) / 2)
        deviazione = self.PIDx.calcolopid(Cx)
        return deviazione

def main():
    cap = cv2.VideoCapture(0)
    wd = 320
    hd = 240
    ballz = pallinadetector(hd, wd)
    cap.set(3, wd)
    cap.set(4, hd)
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Impossibile acquisire il frame dalla videocamera.")
            break

        image = frame.copy()
        pallineargentate = ballz.distinguipallina(frame)
        ballz.riconosci_verde(image, ballz.green)
        if pallineargentate is not None and len(pallineargentate) > 0:
            devy = ballz.calcoloerrorepallinay(pallineargentate[0])
            devx = ballz.calcoloerrorepallinax(pallineargentate[0])
            print("DEVIAZIONI:", devy, devx)

        cv2.imshow("frame", frame)
        cv2.imshow("a_image", image)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

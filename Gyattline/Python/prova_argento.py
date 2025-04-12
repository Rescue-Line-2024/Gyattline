import cv2
import numpy as np

# Inizializza la telecamera (0 è di solito la telecamera predefinita)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Errore: Impossibile accedere alla telecamera")
    exit()

# Imposta la risoluzione (opzionale, regola se necessario)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

while True:
    # Leggi il frame dalla telecamera
    ret, frame = cap.read()
    if not ret:
        print("Errore: Impossibile leggere il frame")
        break

    # Converti in HSV per l'analisi del colore
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Converti in scala di grigi e applica Canny per rilevare i bordi
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # gray = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(gray, 10, 45)  # Soglie Canny regolabili
    # edges = cv2.Canny(gray, 30, 100)

    # Trova le linee con HoughLinesP
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, minLineLength=20, maxLineGap=15)
    # cv2.drawContours(frame, edges, -1, (0,255,255), 1)

    # Analizza ogni linea
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]  # Coordinate della linea
            # Crea una maschera per la linea
            mask = np.zeros_like(gray)
            cv2.line(mask, (x1, y1), (x2, y2), 255, 2)  # Spessore 2 per più pixel
            # Estrai i pixel della linea dall'immagine HSV
            pixels = hsv[mask == 255]
            if len(pixels) > 0:
                mean_s = np.mean(pixels[:, 1])  # Saturazione media
                mean_v = np.mean(pixels[:, 2])  # Valore medio
                # Verifica se è argentata (soglie regolabili)
                if mean_s < 30 and mean_v > 200:
                # if mean_s > 5 and mean_s < 40 and mean_v > 100 and mean_v < 230:
                    cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Disegna in verde
                   
                    text = f"{len(lines)} linee"
                    org = (50, 50)  # posizione del testo (x, y)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 2
                    color = (0, 0, 255)  # rosso in BGR
                    thickness = 3
                    cv2.putText(frame, text, org, font, font_scale, color, thickness, cv2.LINE_AA)


    # Mostra il frame con le linee rilevate
    cv2.imshow('Linee argentate', frame)

    # Esci premendo 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Rilascia le risorse
cap.release()
cv2.destroyAllWindows()
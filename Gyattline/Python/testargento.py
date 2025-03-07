import cv2 

def riconosci_mask(image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        ret, threshold = cv2.threshold(blur, 225, 255, cv2.THRESH_BINARY_INV)

        return threshold



# Apri la webcam (indice 0 è generalmente la webcam predefinita)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Errore: Impossibile accedere alla webcam.")
    exit()

print("Premi 'q' per uscire.")

while True:
    # Legge un frame dalla webcam
    ret, frame = cap.read()
    tresh = riconosci_mask(frame)
    
    if not ret:
        print("Errore: Impossibile leggere il frame dalla webcam.")
        break
    
    # Mostra il frame in una finestra
    cv2.imshow('Webcam',frame)
    cv2.imshow("tresh",tresh)
    
    # Esce se premi 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Rilascia le risorse e chiudi la finestra
cap.release()
cv2.destroyAllWindows()

        


import cv2

# Apri la webcam (indice 0 è generalmente la webcam predefinita)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Errore: Impossibile accedere alla webcam.")
    exit()

print("Premi 'q' per uscire.")

while True:
    # Legge un frame dalla webcam
    ret, frame = cap.read()
    
    if not ret:
        print("Errore: Impossibile leggere il frame dalla webcam.")
        break
    
    # Mostra il frame in una finestra
    cv2.imshow('Webcam', frame)
    
    # Esce se premi 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Rilascia le risorse e chiudi la finestra
cap.release()
cv2.destroyAllWindows()

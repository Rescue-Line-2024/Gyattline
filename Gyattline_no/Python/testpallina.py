from ultralytics import YOLO
import cv2

# Carica il modello YOLO
model = YOLO("my_model.pt")  # Sostituisci con il percorso del tuo modello

# Apri la webcam (o un video con cv2.VideoCapture("video.mp4"))
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Esegui il rilevamento con YOLO
    results = model(frame)

    # Itera su tutti i risultati
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0]  # Coordinate del bounding box
            class_id = int(box.cls[0])  # ID della classe
            conf = box.conf[0]  # Confidenza

            # Calcola il centro del bounding box
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # Disegna il bounding box e il centro
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            # cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # Mostra il nome della classe e la confidenza
            if class_id == 0:
                classe = "Argento"
            elif class_id == 1:
                classe = "Nero"
            
            cv2.putText(frame, f"{classe}: {conf:.2f}, {center_x}, {center_y}", (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Mostra il frame con le rilevazioni
    cv2.imshow("YOLO Ball Detection", frame)

    # Premi 'q' per uscire
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
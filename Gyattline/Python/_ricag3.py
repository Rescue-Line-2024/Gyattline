import time
import cv2
import numpy as np
from ultralytics import YOLO

def main():
    # Dimensioni per il ridimensionamento del frame
    camera_x = 448
    camera_y = 252
    text_pos = (int(camera_x * 0.96), int(camera_y * 0.04))

    # Inizializza la cattura video da una telecamera USB (device 0)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Errore: impossibile aprire la telecamera USB.")
        return

    # Carica il modello di classificazione
    model = YOLO('silver_classify_s.onnx', task='classify')

    fps_time = time.perf_counter()
    counter = 0
    fps = 0

    fps_limit = 1 / 30  # intervallo minimo tra i frame
    fps_limit_time = time.perf_counter()

    while True:
        # Controlla il limite di fps
        if time.perf_counter() - fps_limit_time <= fps_limit:
            continue
        fps_limit_time = time.perf_counter()

        ret, frame = cap.read()
        if not ret:
            print("Errore: impossibile acquisire il frame dalla telecamera.")
            break

        # Ridimensiona il frame
        frame = cv2.resize(frame, (camera_x, camera_y))

        # Effettua la predizione col modello YOLO
        results = model.predict(frame, imgsz=(128, 96), conf=0.4, workers=4, verbose=True)
        annotated_frame = results[0].plot()

        # Calcola gli fps
        counter += 1
        if time.perf_counter() - fps_time > 1:
            fps = int(counter / (time.perf_counter() - fps_time))
            fps_time = time.perf_counter()
            counter = 0

        # Mostra il contatore fps sul frame
        #cv2.putText(annotated_frame, str(fps), text_pos, cv2.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 0), 1, cv2.LINE_AA)

        # Visualizza il frame annotato
        cv2.imshow('Frame', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Rilascia le risorse
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

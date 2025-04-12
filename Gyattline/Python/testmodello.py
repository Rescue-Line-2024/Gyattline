import cv2
from ultralytics import YOLO
import time

def main():
    # Carica il modello YOLO (sostituisci il percorso con il tuo modello)
    model = YOLO("ball_model.pt")  # Puoi usare altri modelli: yolov8s.pt, yolov8m.pt, etc.
    
    # Apri la webcam (camera index 0)
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("Errore nell'apertura della webcam.")
        return

    # Impostazione opzionale della risoluzione (modifica secondo le tue necessità)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Errore nella lettura del frame.")
            break
        
        # Avvia il timer per misurare il tempo di inferenza
        start_time = time.time()
        
        # Esegue l'inferenza: utilizza parametri opzionali per ottimizzare (es. imgsz, conf)
        results = model.predict(frame, imgsz=320, conf=0.25, verbose=False)
        
        # Misura il tempo di inferenza
        elapsed = time.time() - start_time
        fps = 1 / elapsed if elapsed > 0 else 0

        # Visualizza i risultati (disegna le bounding box sugli oggetti rilevati)
        annotated_frame = results[0].plot()
        
        # Aggiunge informazioni sul tempo di inferenza (FPS)
        cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.imshow("Inferenza YOLO", annotated_frame)
        
        # Premere 'q' per uscire dal ciclo
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
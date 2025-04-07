import cv2
from ultralytics import YOLO

def test_yolo_confidenza(model_path):
    model = YOLO(model_path)

    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print("Impossibile aprire la videocamera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Errore nella cattura del frame.")
            break

        results = model(frame)
        confidence = results[0].probs.data[1].item()

        # Stampa il riepilogo tipo "0: 128x128 Silver 0.71, Line 0.29, 77.6ms"
        print("Confidenza argento",confidence)

        # Mostra il frame annotato
        annotated_frame = results[0].plot()
        cv2.imshow("YOLO - Inference", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_yolo_confidenza("silver_classify_s.pt")

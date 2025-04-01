import cv2
import multiprocessing
import numpy as np
from ultralytics import YOLO

# Caricamento del modello YOLO
model = YOLO("my_model.pt")

def riconosci_argento(frame_queue, argento_queue):
    while True:
        if not frame_queue.empty():
            print("Ciao")
            punti = [100]
            frame = frame_queue.get()
            results = model(frame)
            for resul in results:
                for box in resul.boxes:
                    punti = box.xywh[0]
                    print(punti)
            try:
                argento_queue.put(punti.numpy())
            except:
                argento_queue.put(punti)

def main():
    # Creazione delle code
    camera_queue_nero = multiprocessing.Queue(maxsize=5)  # Limite per evitare accumulo
    camera_queue_argento = multiprocessing.Queue(maxsize=5)
    visualizza_queue_nero = multiprocessing.Queue(maxsize=5)  # Limite per evitare accumulo
    visualizza_queue_argento = multiprocessing.Queue(maxsize=5)
    nero_queue = multiprocessing.Queue()
    argento_queue = multiprocessing.Queue()

    # Creazione dei processi
    processo_camera = multiprocessing.Process(target=cattura_video, args=(camera_queue_nero, camera_queue_argento, visualizza_queue_nero, visualizza_queue_argento))
    processo_nero = multiprocessing.Process(target=riconosci_nero, args=(camera_queue_nero, nero_queue))
    processo_argento = multiprocessing.Process(target=riconosci_argento, args=(camera_queue_argento, argento_queue))

    # Avvio dei processi
    processo_camera.start()
    processo_nero.start()
    processo_argento.start()

    try:
        while True:
            frame_nero = visualizza_queue_nero.get()
            frame_argento = visualizza_queue_argento.get()

            # Ottieni i risultati dai processi
            if not nero_queue.empty():
                contours = nero_queue.get()
                for contour in contours:
                    cv2.drawContours(frame_nero, [contour], -1, (0, 255, 0), 2)

            if not argento_queue.empty():
                argento_trovato = argento_queue.get()
                print(argento_trovato)
                
                if len(argento_trovato) == 4:
                    x, y, w, h = map(int, argento_trovato)

                    if y > hd / 2:  # Condizione per "argento trovato"
                        cv2.rectangle(frame_argento, (x, y), (x + w, y + h), (0, 0, 255), 2)
                        print("ARGENTO TROVATO!!")
                    else:
                        cv2.rectangle(frame_argento, (x, y), (x + w, y + h), (255, 0, 0), 2)
                        print("ARGENTO NON TROVATO ma c'è!!")
                else: 
                    print("ARGENTO NON TROVATO!!")

            # Mostra il frame
            cv2.imshow("Linea", frame_nero)
            cv2.imshow("Argento", frame_argento)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Termina i processi
        processo_camera.terminate()
        processo_nero.terminate()
        processo_argento.terminate()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
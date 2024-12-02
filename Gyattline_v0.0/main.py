from multiprocessing import Process, Queue
from Serial import SerialConnection  # Assumendo che la classe per la seriale sia salvata qui
from ric_colori import RiconosciColori
from verdi_e_linea import Seguilinea
import cv2
import time


def serial_communication(queue):
    """
    Funzione che gestisce la comunicazione seriale in un processo separato.
    """
    conn = SerialConnection(port='/dev/ttyUSB0', baudrate=115200)
    conn.open_connection()

    while True:
        # Controlla se ci sono messaggi nella coda da inviare
        if not queue.empty():
            message = queue.get()
            if message == "STOP":  # Comando per terminare il processo
                break
            conn.send_message(message)

        # Legge messaggi dalla seriale
        response = conn.read_message()
        if response:
            print(f"Ricevuto dall'ESP32: {response}")

    conn.close_connection()


# Funzione principale con videocamera
def main():
    # Inizializza la videocamera
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        print("Errore nell'apertura della camera.")
        return

        # Imposta la risoluzione desiderata (esempio: 640x480)
    desired_width = 320
    desired_height = 240

    cam.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
    # Ottieni la risoluzione della camera
    # Ottieni la risoluzione della camera
    width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Inizializza gli oggetti principali
    Line_follower = Seguilinea(P=3, I=0, D=0, PEN=0.25, min_area=500, cam_resolution=(width, height))
    Riconosci_verde = RiconosciColori([35, 40, 40], [75, 255, 255])

    # Inizializza la coda per la comunicazione con il processo seriale
    queue = Queue()
    serial_process = Process(target=serial_communication, args=(queue,))
    serial_process.start()

    try:
        while True:
            ret, frame = cam.read()
            if not ret:
                print("Errore nella lettura del frame.")
                break

            frame_colori = frame.copy()

            # Riconoscimento linea e verde
            coordinate_nero = Line_follower.segui_linea(frame)
            coordinate_verde = Riconosci_verde.riconosci_colore(frame_colori)

            # Disegna bounding box
            if coordinate_verde is not None:
                Riconosci_verde.disegna_bbox(coordinate_verde, frame_colori, (0, 255, 0))

            # Invia messaggi alla ESP32
            if coordinate_nero:
                queue.put(f"Linea trovata: {coordinate_nero}")  # Invio informazioni della linea
            if coordinate_verde:
                queue.put(f"Verde trovato: {coordinate_verde}")  # Invio informazioni del verde

            # Mostra i frame
            cv2.imshow("Camera principale", frame)
            cv2.imshow("Rilevamento colori", frame_colori)

            # Interrompi con il tasto 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Chiudi tutto
        cam.release()
        cv2.destroyAllWindows()
        queue.put("STOP")  # Invia il comando di stop al processo seriale
        serial_process.join()


if __name__ == "__main__":
    main()

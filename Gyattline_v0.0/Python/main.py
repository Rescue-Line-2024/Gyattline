from threading import Thread, Lock
from Serial import SerialConnection  # Assumendo che la classe per la seriale sia salvata qui
from ric_colori import RiconosciColori
from Seguilinea import Seguilinea
import cv2
import time


class Robot:
    def __init__(self):
        self.stop_signal = False  # Segnale per terminare i thread
        self.lock = Lock()  # Lock per sincronizzare l'accesso ai dati condivisi
        self.shared_message = None  # Messaggio condiviso tra i thread

        # Inizializza i thread
        self.serial_thread = Thread(target=self.serial_communication)
        self.camera_thread = Thread(target=self.camera_main)

        # Avvia i thread
        self.serial_thread.start()
        self.camera_thread.start()

    def serial_communication(self):
        """
        Funzione che gestisce la comunicazione seriale.
        """
        conn = SerialConnection(port='/dev/ttyACM0', baudrate=115200)

        try:
            conn.open_connection()

            while not self.stop_signal:
                # Leggi messaggi dalla seriale
                response = conn.read_message()
                if response:
                    pass
                    #print(f"Ricevuto dall'arduino: {response}")

                # Invia messaggi se presenti
                with self.lock:
                    if self.shared_message:
                        conn.send_message(self.shared_message)
                        self.shared_message = None

            conn.close_connection()
        except Exception as e:
            print("Errore nella comunicazione seriale:", e)
            conn.close_connection()

    def camera_main(self):
        """
        Funzione principale con videocamera.
        """
        cam = cv2.VideoCapture(0)
        if not cam.isOpened():
            print("Errore nell'apertura della camera.")
            return

        desired_width = 160
        desired_height = 120
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

        width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

        Line_follower = Seguilinea(P=3, I=0, D=0, P2=1.5,PEN=0.5, min_area=50, cam_resolution=(width, height),motor_limit = 20)
        
        
        
        try:
            while not self.stop_signal:
                ret, frame = cam.read()
                if not ret:
                    print("Errore nella lettura del frame.")
                    break

                frame_colori = frame.copy()

                #IL seguilinea tornerà un json con l'azione e il dato
                
                instruction = Line_follower.segui_linea(frame)
                    

                # Condividi messaggi con il thread seriale
                with self.lock:
                    self.shared_message = instruction

                # Mostra i frame
                cv2.imshow("Camera principale", frame)
                cv2.imshow("Rilevamento colori", frame_colori)

            

                # Interrompi con il tasto 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.shared_message = {"action" : "stop"}
                    self.stop_signal = True
                    break
        finally:
            cam.release()
            cv2.destroyAllWindows()

    def join_threads(self):
        """
        Attende che i thread terminino.
        """
        self.serial_thread.join()
        self.camera_thread.join()


if __name__ == "__main__":
    robot = Robot()
    robot.join_threads()

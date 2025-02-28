from threading import Thread, Lock
from Serial import SerialConnection  # Assumendo che la classe per la seriale sia salvata qui
from ric_colori import RiconosciColori
from Seguilinea import Seguilinea
from ArduinoManager import ArduinoManager
import cv2
import time


class Robot:
    def __init__(self):
        self.stop_signal = False  # Segnale per terminare i thread
        self.lock = Lock()  # Lock per sincronizzare l'accesso ai dati condivisi

        # Inizializza i thread
        self.serial_thread = Thread(target=self.serial_communication)
        self.camera_thread = Thread(target=self.camera_main)

        # Avvia i thread
        self.serial_thread.start()
        self.camera_thread.start()

    def serial_communication(self):
        
        conn = SerialConnection(port='/dev/ttyACM0', baudrate=115200)
        try:
            conn.open_connection()
            
            while not self.stop_signal:
                # Lettura di eventuali messaggi in arrivo dall'Arduino
                #ArduinoManager.message = {"action": "motors", "data": [i, -i]}
                
                response = conn.read_message()
                if response:
                    try:
                        # Se il messaggio contiene i dati dei sensori (Arduino non include "action" in questi messaggi)
                        if "front" in response and "left" in response and "right" in response:
                            front_sensor = response["front"]
                            left_sensor = response["left"]
                            right_sensor = response["right"]
                            print(f"Sensori: Front={front_sensor}, Left={left_sensor}, Right={right_sensor}")
                            ArduinoManager.front_sensor = front_sensor
                            ArduinoManager.left_sensor = left_sensor
                            ArduinoManager.right_sensor = right_sensor
                        # Gestione di altri tipi di messaggi (ad esempio, comandi o notifiche)
                        elif "action" in response:
                            if response["action"] == "stop":
                                print("Ricevuto comando di stop dall'Arduino")
                            else:
                                print(f"Messaggio ricevuto: {response}")
                    except Exception as e:
                        print("Errore nel parsing del messaggio:", e)
                
                
                # Se è presente un messaggio impostato dal modulo Seguilinea, invialo all'Arduino
                if ArduinoManager.message is not None:
                    conn.send_message(ArduinoManager.message)
                    ArduinoManager.message = None

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

            # Imposta i parametri del PID, ad esempio (P, I, D)
        pid_params = (3, 0, 0)
        # Crea l'istanza del SeguiLinea
        Line_follower = Seguilinea(
            cam=cam,
            pid_params=pid_params,
            P2=1,
            pen_multiplier=0.1,
            cam_resolution=(width, height),
            min_area=50,
            cut_percentage=0.6,
            motor_limit=30
        )
        
        
        
        try:
            while not self.stop_signal:
                ret, frame = cam.read()
                if not ret:
                    print("Errore nella lettura del frame.")
                    break

                frame_colori = frame.copy()

                #IL seguilinea tornerà un json con l'azione e il dato
                
                Line_follower.follow_line(frame)

                # Mostra i frame
                try:
                    cv2.imshow("Camera principale", frame)
                    cv2.imshow("Rilevamento colori", frame_colori)
                except:
                    pass

            

                # Interrompi con il tasto 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    ArduinoManager.message = {"action" : "stop"}
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
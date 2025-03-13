

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
            
            while not self.stop_signal:  # Loop infinito finché non viene inviato il segnale di stop
                response = conn.read_message()
                if response:
                    try:
                        # Se il messaggio contiene i dati dei sensori
                        if "front" in response:
                            ArduinoManager.front_sensor = response["front"]
                        if "left" in response:
                            ArduinoManager.left_sensor = response["left"]
                        if "right" in response:
                            ArduinoManager.right_sensor = response["right"]
                        
                        
                        # Gestione di altri tipi di messaggi
                        if "action" in response:
                            if response["action"] == "stop":
                                print("Ricevuto comando di stop dall'Arduino")
                                self.stop_signal = True
                            else:
                                print(f"Messaggio ricevuto: {response}")
                    
                    except Exception as e:
                        print("Errore nel parsing del messaggio:", e)
                
                # Invio di comandi all'Arduino se presenti
                if ArduinoManager.message is not None:
                    conn.send_message(ArduinoManager.message)
                    time.sleep(0.1)  # Piccola pausa per evitare di sovraccaricare la CPU
                    ArduinoManager.message = None

                

        except Exception as e:
            print("Errore nella comunicazione seriale:", e)

        finally:
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

        pid_params = (1, 0, 0)
        # Crea l'istanza del SeguiLinea
        Line_follower = Seguilinea(
            cam=cam,
            pid_params=pid_params,
            P2=1.5,
            pen_multiplier=0.1,
            cam_resolution=(width, height),
            min_area=50,
            cut_percentage=0.6,
            motor_limit=25
        )
        
        
        
        try:
            while not self.stop_signal:
                ret, frame = cam.read()
                if not ret:
                    print("Errore nella lettura del frame.")
                    break

                frame_colori = frame.copy()

                #IL seguilinea tornerà un json con l'azione e il dato
                zoom_factor = 1.3  # Puoi aumentare o diminuire questo valore per modificare lo zoom
                h, w, _ = frame.shape
                new_w, new_h = int(w / zoom_factor), int(h / zoom_factor)

                x1 = (w - new_w) // 2
                y1 = (h - new_h) // 2
                x2 = x1 + new_w
                y2 = y1 + new_h

                # Ritaglio e ridimensionamento
                frame = frame[y1:y2, x1:x2]  # Ritaglia la parte centrale
                frame = cv2.resize(frame, (w, h))  # Ridimensiona alla risoluzione originale
                    
                Line_follower.follow_line(frame)

                # Mostra i frame
                try:
                    pass
                    cv2.imshow("Camera principale", frame)
                    cv2.imshow("Rilevamento colori", frame_colori)
                except:
                    pass

            

                # Interrompi con il tasto 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    Seguilinea.messaggio = {"action" : "stop"}
                    self.stop_signal = True
                    break

        except KeyboardInterrupt:
            print("keyboard interrupt rilevato")
            cam.release()
            cv2.destroyAllWindows()
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
    #robot.join_threads()
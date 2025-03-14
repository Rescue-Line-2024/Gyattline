from threading import Thread, Lock
from Serial import SerialConnection  # Assumendo che la classe per la seriale sia salvata qui
from ric_colori import RiconosciColori
from Seguilinea import Seguilinea
from ArduinoManager import ArduinoManager
from stanzapalle import BallsController
import cv2
import time
class Robot:
    def __init__(self):
        self.stop_signal = False  # Segnale per terminare i thread
        self.lock = Lock()  # Lock per sincronizzare l'accesso ai dati condivisi
        ArduinoManager.motor_limit = 30
        # Inizializza i thread
        self.serial_thread = Thread(target=self.serial_communication)
        self.camera_thread = Thread(target=self.camera_main)

        self.zonapalle = BallsController()
        self.raccogliendo_palle = False
        self.riconoscendo_argento = False
        ArduinoManager.motor_state = True
        # Istanza per il riconoscimento del rosso.
        self.riconosci_rosso = RiconosciColori([0, 150, 150], [10, 255, 255], min_area=500)

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

                            elif response["action"] == "ARGENTO":
                                print("ARGENTOOOO!!!")
                                time.sleep(1)
                                self.raccogliendo_palle = True
                                ArduinoManager.set_camera(205)
                                

                            if response["action"] == "Motori_spenti":
                                print("motori spenti")
                                ArduinoManager.motor_state = False
                                self.raccogliendo_palle = False

                    
                    except Exception as e:
                        print("Errore nel parsing del messaggio:", e)

                else:
                    ArduinoManager.motor_state = True
                
                # Invio di comandi all'Arduino se presenti
                with ArduinoManager.message_lock:
                    if ArduinoManager.message is not None:
                        conn.send_message(ArduinoManager.message)
                        time.sleep(0.05)  # Piccola pausa per evitare di sovraccaricare la CPU
                        ArduinoManager.message = None
                        
        except Exception as e:
            print("Errore nella comunicazione seriale:", e)
        finally:
            ArduinoManager.send_motor_commands(0, 0)
            conn.close_connection()

    def camera_main(self):
        """
        Funzione principale con videocamera.
        """
        cam = cv2.VideoCapture(0)
        
        if not cam.isOpened():
            print("Errore nell'apertura della camera.")
            return

        desired_width = 320
        desired_height = 240
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

        width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print("Risoluzione corrente:", width, height)
        
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
                
                # Se si sta riconoscendo l'argento, vengono applicate alcune modifiche
                                # *** Riconoscimento del ROSSO ***
                red_boxes = self.riconosci_rosso.riconosci_colore(frame_colori)
                if red_boxes is not None:
                    print("Rilevato rosso: fermo i motori!")
                    # Disegna le bounding box sul frame (colore rosso)
                    self.riconosci_rosso.disegna_bbox(red_boxes, frame, (0, 0, 255))
                    ArduinoManager.send_motor_commands(0, 0)
                    # Mostra i frame aggiornati e passa alla prossima iterazione
                    cv2.imshow("Camera principale", frame)
                    cv2.imshow("Rilevamento colori", frame_colori)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        Seguilinea.messaggio = {"action": "stop"}
                        self.stop_signal = True
                    continue
                


                    

                    
                if self.raccogliendo_palle:
                    ArduinoManager.motor_limit = 15
                    self.zonapalle.main(frame, (width, height))
                else:
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
                    cv2.imshow("Camera principale", frame)
                    cv2.imshow("Rilevamento colori", frame_colori)
                except Exception as e:
                    print("Errore nell'apertura delle finestre:", e)
                
                # Interrompi con il tasto 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    Seguilinea.messaggio = {"action": "stop"}
                    self.stop_signal = True
                    break

        except KeyboardInterrupt:
            print("Keyboard interrupt rilevato")
            ArduinoManager.send_motor_commands(0, 0)
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
    # robot.join_threads()  # Puoi chiamare join_threads() se desideri attendere la terminazione dei thread

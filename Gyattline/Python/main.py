from threading import Thread, Lock
from Serial import SerialConnection  # Assumendo che la classe per la seriale sia salvata qui
from ric_colori import RiconosciColori
from Seguilinea import Seguilinea
from ArduinoManager import ArduinoManager
from stanzapalle import BallsController
from ultralytics import YOLO
import logging

logging.getLogger("ultralytics").setLevel(logging.ERROR)

import cv2
import time
import multiprocessing
import numpy as np

# Funzione globale per il riconoscimento dell'argento
def riconosci_argento_process(frame_queue, argento_queue):
    # Carica il modello YOLO all'interno del processo
    model = YOLO("my_model.pt")
    while True:
        if not frame_queue.empty():
            #print("riconoscendo argento ...")
            frame = frame_queue.get()
            punti = []  # Valore di default nel caso non vengano trovate box
            results = model(frame)
            for res in results:
                for box in res.boxes:
                    conf = float(box.conf[0])
                    if conf > 0.75:
                        punti = box.xywh[0]
                        print("Punti trovati:", punti)
            try:
                argento_queue.put(punti.numpy())
            except Exception as e:
                argento_queue.put(punti)

def zonapalle_process_func(frame_queue, pause_event, resolution, lock):
    balls = BallsController()
    while True:
        if pause_event.is_set():
            if not frame_queue.empty():
                with lock:  # Blocca l'accesso finché il frame attuale non è stato elaborato
                    frame = frame_queue.get()
                    balls.main(frame, resolution)
                    cv2.imshow("Palle", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        else:
            time.sleep(0.1)


class Robot:
    def __init__(self):
        self.stop_signal = False  # Segnale per terminare i thread
        self.lock = Lock()  # Lock per sincronizzare l'accesso ai dati condivisi
        ArduinoManager.motor_limit = 30
        # Inizializza i thread
        self.serial_thread = Thread(target=self.serial_communication)
        self.camera_thread = Thread(target=self.camera_main)
                # Code per il riconoscimento dell'argento (YOLO)
        self.frame_queue_argento = multiprocessing.Queue(maxsize=5)
        self.argento_queue = multiprocessing.Queue(maxsize=5)
        self.argento_process = multiprocessing.Process(
            target=riconosci_argento_process,
            args=(self.frame_queue_argento, self.argento_queue)
        )

        self.zonapalle_frame_queue = multiprocessing.Queue(maxsize=1)
        self.zonapalle_pause_event = multiprocessing.Event()
        self.zonapalle_lock = multiprocessing.Lock()
        self.zonapalle_process = multiprocessing.Process(
            target=zonapalle_process_func,
            args=(self.zonapalle_frame_queue, self.zonapalle_pause_event, (320, 240),self.zonapalle_lock)
        )

        
        

        self.ag_timer = time.time()-5
        self.zonapalle = BallsController()
        self.raccogliendo_palle = False
        self.riconoscendo_argento = False
        ArduinoManager.motor_state = True
        # Istanza per il riconoscimento del rosso.
        self.riconosci_rosso = RiconosciColori([0, 150, 150], [10, 255, 255], min_area=500)
        self.ag_visto = False
        # Avvia i thread
        self.serial_thread.start()
        self.camera_thread.start()
        self.argento_process.start()
        self.zonapalle_process.start()

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

                                
                                
                                

                            if response["action"] == "Motori_spenti":
                                print("motori spenti")
                                self.ag_timer = time.time()
                                ArduinoManager.motor_state = False
                                self.raccogliendo_palle = False
                                #zonapalle.andato_avanti = False
                                ArduinoManager.motor_limit = 30
                                ArduinoManager.set_camera(160)
                                time.sleep(0.1)

                    
                    except Exception as e:
                        print("Errore nel parsing del messaggio:", e)

                else:
                    ArduinoManager.motor_state = True
                    self.ag_visto = False
                
                # Invio di comandi all'Arduino se presenti
                with ArduinoManager.message_lock:
                    if ArduinoManager.message is not None:
                        conn.send_message(ArduinoManager.message)
                        print(ArduinoManager.message)
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
        
        pid_params = (2, 0, 0) # (Kp, Ki, Kd)
        # Crea l'istanza del SeguiLinea
        Line_follower = Seguilinea(
            cam=cam,
            pid_params=pid_params,
            P2=1.5,
            pen_multiplier=0.1,
            cam_resolution=(width, height),
            min_area=50,
            cut_percentage=0.5,
            motor_limit=30
        )
                # Inizializza il processo per zonapalle dopo aver ottenuto la risoluzione



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
                if red_boxes is not None and self.raccogliendo_palle == False:
                    x,y,w,h = red_boxes[0]
                    print("Rilevato rosso: fermo i motori!")
                    # Disegna le bounding box sul frame (colore rosso)
                    self.riconosci_rosso.disegna_bbox(red_boxes, frame, (0, 0, 255))
                    if w > 200 and y+h > 200:
                        ArduinoManager.send_motor_commands(0, 0)
                        continue
                        #ROSSO VISTO!!! STAI FERMOOOOO!!!

                    
                    cv2.imshow("Camera principale", frame)
                    cv2.imshow("Rilevamento colori", frame_colori)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        ArduinoManager.message = {"action": "stop"}
                        self.stop_signal = True
                    


                if self.raccogliendo_palle:
                    with self.zonapalle_lock:  # Blocca l'inserimento finché il processo non libera il lock
                        if not self.zonapalle_frame_queue.full():
                            self.zonapalle_frame_queue.put(frame)
                        
                       
                         # Invia il frame al processo del riconoscimento dell'argento
                if not self.frame_queue_argento.full():
                    # Se necessario, invia una copia o una versione ridimensionata
                    if self.raccogliendo_palle == False and ArduinoManager.motor_state == True:
                        self.frame_queue_argento.put(frame.copy())

                # Gestione del risultato di YOLO (riconoscimento dell'argento)
                if not self.argento_queue.empty():
                    argento_trovato = self.argento_queue.get()
                    print("Risultato argento:", argento_trovato)
                    if isinstance(argento_trovato, (list, np.ndarray)) and len(argento_trovato) == 4:
                        x, y, w, h = map(int, argento_trovato)
                        # Esempio: se l'argento è nella metà inferiore dell'immagine
                        if y > height / 2:
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                            print("ARGENTO TROVATO!! attendi un po")
                            if self.ag_timer < time.time()-0.5:
                                self.ag_visto = True
                                self.raccogliendo_palle = True
                                print("ARGENTOOOO!!!")
                        else:
                            self.ag_timer = time.time()
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                            print("ARGENTO NON TROVATO ma c'è!!")
                            self.ag_visto = False
                    else:
                        self.ag_timer = time.time()
                        print("ARGENTO NON TROVATO!!")
                        self.ag_visto = False


                if self.ag_visto and self.raccogliendo_palle == False:
                    #questo serve a fare andare avanti DI POCO il robot
                    ArduinoManager.send_motor_commands(25,25)
                    
                    print("argento! andando avanti")
                    time.sleep(1) 
                    for i in range (10):
                        ArduinoManager.request_sensor_data()
                        print(ArduinoManager.front_sensor,ArduinoManager.left_sensor,ArduinoManager.right_sensor)
                        time.sleep(0.1)


                    self.raccogliendo_palle = True
                    
                    #trattiamo il muro come un ostacolo da schivare, solo che useremo un pid differente
                    #last_obstacle position lo uso per tenermi traccia di quale sensore uso per distanziarmi dal muro
                    
                    
                    if ArduinoManager.left_sensor is not None and ArduinoManager.right_sensor is not None:
                        ArduinoManager.last_obstacle_position = (
                            "left" if ArduinoManager.left_sensor < ArduinoManager.right_sensor
                            else "right") 
                    else:
                        if ArduinoManager.left_sensor is not None:
                            ArduinoManager.last_obstacle_position = "left"
                        elif ArduinoManager.right_sensor is not None:
                            ArduinoManager.last_obstacle_position = "right"
                        else:
                            print("i sensori non funzionano,impossibile prendere le palle.")
                            self.raccogliendo_palle = False

                    if ArduinoManager.last_obstacle_position == "left":
                        self.zonapalle.wall_pid.inverted *= -1

                    ArduinoManager.set_camera(190)
                    time.sleep(0.1)
                    self.ag_visto = False
                    

                    
                if self.raccogliendo_palle:
                    # Attiva il processo zonapalle
                    ArduinoManager.motor_limit = 15
                    self.zonapalle_pause_event.set()
                    if not self.zonapalle_frame_queue.full():
                        self.zonapalle_frame_queue.put(frame)
                else:
                    self.zonapalle_pause_event.clear()
                    zoom_factor = 1.4  # Puoi aumentare o diminuire questo valore per modificare lo zoom
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
        self.argento_process.join()

if __name__ == "__main__":
    multiprocessing.freeze_support()
    robot = Robot()
    #robot.join_threads()  # Puoi chiamare join_threads() se desideri attendere la terminazione dei thread 
import serial  # Libreria per la comunicazione seriale
import time    # Per ritardi, se servono
import json

class SerialConnection:
    def __init__(self, port, baudrate=9600, timeout=0.1):
        """
        Configura la connessione seriale.
        :param port: La porta seriale (es: '/dev/ttyUSB0' o 'COM3').
        :param baudrate: Velocità della comunicazione (default 9600).
        :param timeout: Tempo di attesa per la lettura (default 1 secondo).
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None  # Qui mettiamo la connessione quando la apriamo

    def open_connection(self):
            self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            print(f"Connessione seriale aperta su {self.port} a {self.baudrate} baud.")
            time.sleep(2)  # Attendi che la connessione sia stabile 
  

    def close_connection(self):
        """Chiude la connessione seriale."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Connessione seriale chiusa.")

    def send_message(self, message):
        """
        Invia un messaggio all'ESP32.
        :param message: Il messaggio da inviare (stringa o dizionario).
        """
        # Se il messaggio è un dizionario, serializziamolo in JSON
        if isinstance(message, dict):
            message = json.dumps(message)
        
        if self.serial and self.serial.is_open:
            try:
                # Invia il messaggio convertendolo in bytes, seguito da un newline
                self.serial.write((message + '\n').encode('utf-8'))
                #print(f"Inviato: {message}")
            except Exception as e:
                print(f"Errore nell'invio del messaggio: {e}")
        else:
            print("Connessione seriale non aperta. Impossibile inviare.")

    def read_message(self):
        """
        Legge un messaggio dall'ESP32.
        :return: Il messaggio ricevuto (come dizionario o stringa) o None.
        """
        if self.serial and self.serial.is_open:
            try:
                # Legge una linea (fino a '\n') e decodifica da bytes a stringa
                message = self.serial.readline().decode('utf-8').strip()
                
                if message:  # Se c'è qualcosa di valido
                    pass
                    #print(f"Ricevuto: {message}")
                    
                    # Tentiamo di decodificare il messaggio come JSON
                    try:
                        json_message = json.loads(message)
                        return json_message  # Se è JSON, ritorna il dizionario
                    except json.JSONDecodeError:
                        # Se non è un JSON valido, ritorna la stringa come tale
                        #print("Messaggio ricevuto non è un JSON valido.")
                        return message

            except Exception as e:
                print(f"Errore nella lettura del messaggio: {e}")
        else:
            print("Connessione seriale non aperta. Impossibile leggere.")
        
        return None


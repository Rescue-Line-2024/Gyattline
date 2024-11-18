import cv2
import time
import threading
import serial
import numpy as np
from verdi_e_linea import RicLinea,GestisciVerdi,treshnero
from Teachablemachine import vediargento
from PallinaDetector import pallinadetector
from PID import gpPID
from Interruttore import rpInterruttore
#LIBRERIE

class Robot:
    #COSTRUTTORE
    def __init__(self,mot):
        self.switch = rpInterruttore(16)
        self.video_capture = cv2.VideoCapture(0)
        self.video_capture.set(3, 320)
        self.video_capture.set(4, 240)
        ret , self.frame = self.video_capture.read()
        self.MotoriAccesi = mot
        self.deviazione = 0
        self.balldetect = pallinadetector(hd=240,wd=320)
        self.agvisto = False
        self.avviato = False
        
    #FUNZIONE CHE RICONOSCE IL ROSSO!
    def riconosci_rosso(self,image,color): 
        Min,Max = color
        hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,Min,Max)
        array = []
        contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                x,y,w,h = cv2.boundingRect(contour)
                cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,255),2)
                    
                array.append([x,y,w,h])
        return array
    
    #UNO SLEEP FATTO MEGLIO
    def modsleep(self,tempo):
        sleeptemp = time.time()+tempo
        while(time.time()<sleeptemp):
            pass
        
            #ZONA PALLINE!!!!
    def ZonaPalline(self):
            '''
            QUANDO LA DEVIAZIONE E' >1000,GLI DICO ALL ARDUINO DI IMPOSTARE IL SERVO DEL
            PAN TILT A (DEVIAZIONE-1000)
            '''
            
            timer_verde = time.time()
            time.sleep(1)
            self.MotoriAccesi = False
            for i in range(10):
                self.deviazione = 5000
                time.sleep(0.1)
            self.MotoriAccesi = True
        
            cv2.destroyAllWindows()
            pallinapresa = False
            self.MotoriAccesi = True
            
            cv2.destroyAllWindows()
            pallinapresa = False
            c = 0
            self.deviazione = 300
            dev2 = 0
            while c<3:
                if self.switch.pulsantepremuto() == False:
                    self.deviazione = 0
                    self.agvisto = False
                    break
                ret , self.frame = self.video_capture.read()
                
                #SE LA PALLINA NON È STATA PRESA,RICERCALA,ALTRIMENTI CERCA IL VERDE(CASSONETTO)
                if not pallinapresa:
                    pallineargentate = self.balldetect.distinguipallina(self.frame)
                    if len(pallineargentate) > 0:
                        Ph = self.balldetect.ritornaltezza(pallineargentate[0])
                        devy = self.balldetect.calcoloerrorepallinay(pallineargentate[0])
                        devx = self.balldetect.calcoloerrorepallinax(pallineargentate[0])
                        print("DEVIAZIONE palla:",devx)
                        print("ALTEZZA:",devy)
                        self.deviazione = devx
                        
                            
                        if self.deviazione >-5 and self.deviazione < 5 and Ph>150:
                                self.deviazione = 0
                                print("Prendo la pallina")
                                self.deviazione = 3000
                                time.sleep(0.1)
                                h = 0
                                pallinapresa = True
                                self.deviazione = 300
                                timer_verde = time.time()
                else:
                    dev2 = self.balldetect.vai_verso_cassonetto(self.frame)
                      
                    
                    if dev2 != 1000:
                        self.deviazione = dev2
                        
                    if dev2 == 1000: #SE IL CASSONETTO E' ABBASTANZA VICINO
                        print("prendi la pallina")
                        pallinapresa = False
                        c+=1
                        self.deviazione = 4000
                        time.sleep(1)
                        self.deviazione = 0
                        #4000 E UN CODICE PER DIRE ALL ARDUINO DI DEPOSITARE LA PALLA
                        
                    print(f"dev verde {self.deviazione}")
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.deviazione = 1000
                    break
                cv2.imshow("frame",self.frame)
            cv2.destroyAllWindows()
            
    #CICLO PRINCIPALE(PRIMO THREAD)
    def SeguiLinea(self):
        #------------------------init di alcune variabili della funzione-------------------------#
        timerv = time.time()
        last_deviazione = 0
        red_color = (np.array([0, 100, 100]), np.array([10, 255, 255])) 
        #----------------------------------------------------------------------------------------#

        #----------------------------------CICLO PRINCIPALE--------------------------------------#
        while True:
            self.avviato = self.switch.pulsantepremuto()
            if self.avviato == True:
                self.MotoriAccesi = True
            elif self.avviato == False:
                self.MotoriAccesi = False
                time.sleep(0.1)
                continue
            
                
            arrayRED = []
            #image e usato per il seguilinea,self.frame è usato per i colori
            ret, self.frame = self.video_capture.read()
            image = self.frame.copy()
            verdeix = GestisciVerdi(self.frame,image)
            if self.agvisto == True:
                self.deviazione = 0
                self.ZonaPalline()
                
            #MI CALCOLO LA DEVIAZIONE IN BASE ALLA POSIZIONE DELLA LINEA
            
            
            '''
            la funzione GestisciVerdi torna:
              0 se non ci sono verdi
              -1 se ci sono solo verdi dietro la linea
              1 se c'è un verde a destra
              2 se c'è un verde a sinistra
              3 se c'è un doppio verde
            '''
            last_deviazione = self.deviazione
            self.deviazione = RicLinea(image,last_deviazione)    
            #---------gestione verdi---------------#
            if time.time()>timerv:
                if verdeix == -1:
                    self.deviazione = 0
                if verdeix == "DOPPIO":
                    print("DOPPIO!")
                    time.sleep(1)
                    self.deviazione = -300
                    time.sleep(2.5)
                    timerv = time.time()+1
                elif verdeix is not None:
                    self.deviazione = verdeix
                    print("DEVIAZIONE VERDE",verdeix)
            #---------gestione verdi---------------#
                
            #print("DEVIAZIONE:+",self.deviazione)
  
  
            
            #---------gestione rosso---------------#
           
            arrayRED = self.riconosci_rosso(image,red_color)
            if len(arrayRED) > 0 and False: #togliere l' and false per attivare
                    print("ROSSO RILEVATO")
                    print(arrayRED)
                    Rx,Ry,Rw,Rh = arrayRED[0]
                    if Ry > 230:
                        time.sleep(0.1)
                        print("FERMATI!")
                        self.MotoriAccesi = False
            else:
                    self.MotoriAccesi = True
            #--------------------------------------#
                                                
       
            
            if self.MotoriAccesi:  
                pass
            else:
                self.deviazione = 1000
          
            cv2.imshow('Seguilinea', image)
            cv2.imshow('frame',self.frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.deviazione = 1000
                break
        #----------------------------------------------------------------------------------------#
        self.video_capture.release()
        cv2.destroyAllWindows()
        

        
    def ConnessioneSeriale(self):
        global lock
        devgraduale = 0
        
        ser = serial.Serial("/dev/ttyACM0",9600,timeout=0.1)
        time.sleep(3)
        ser.reset_input_buffer()
        print("Serial OK")
        line = 10
        while True:
            time.sleep(0.1)
            #!!!GESTIONE OSTACOLO
                
            if ser.in_waiting > 0 and self.agvisto == False: #se l arduino ha mandato un messaggio
                messaggio = int(ser.readline().decode('utf-8').rstrip())
                print(messaggio)
                if messaggio == 1000:  #mille vuol dire che ha visto l ostacolo
                    presenza_linea = 0
                    time.sleep(0.5)
                    while presenza_linea < 30:
                        presenza_linea = treshnero(self.frame.copy(),"muro",0 )
                        print("LARGHEZZA"+str(presenza_linea))
                        time.sleep(0.1)
                        if self.switch.pulsantepremuto() == False:
                            break
                        
                    for i in range (2):
                        ser.write((str(1000)+"\n").encode("utf-8"))
                        time.sleep(0.1)
        
            lock = 0
            if devgraduale < self.deviazione-25:
                devgraduale+=20
            elif devgraduale > self.deviazione+25:
                devgraduale-=20
            else:
                devgraduale = self.deviazione
            #AVVIO DEI MOTORI IN BASE ALLA DEVIAZIONE
            if self.MotoriAccesi:
                try:
                    ser.write((str(-1 * int(devgraduale)) + "\n").encode("utf-8"))
                    #print("success")
                except serial.SerialException as e:
                    print(f"Serial write failed: {e}")
                except Exception as e:
                    print(f"An unexpected error occurred: {e}")
            
            else:
                ser.write((str(1000)+"\n").encode("utf-8"))
                #1000 STA A SIGNIFICARE DI FERMARE I MOTORI
            lock = 1
        
            #SE GLI ARRIVA 1000 DAL MAIN COME DEVIAZIONE,CHIUDI LA CONN SERIALE
            if self.deviazione == 1000:
                print("Close Serial communication")
                ser.write((str(1000)+"\n").encode("utf-8"))
                time.sleep(0.1)
                ser.close()
                
            #3000 = PRENDI PALLINA
            elif self.deviazione == 3000:
                ser.write((str(3000)+"\n").encode("utf-8"))
                time.sleep(0.1)
            #4000 = RILASCIA PALLINA
            elif self.deviazione == 4000:
                ser.write((str(4000)+"\n").encode("utf-8"))
                time.sleep(0.1)
                    
            #>1000 = IMPOSTA SERVO CAM A DEVIAZIONE -1000
            elif self.deviazione > 1000:
                for i in range(2):
                    ser.write((str(self.deviazione)+"\n").encode("utf-8"))
                    time.sleep(0.1)
            
            elif self.deviazione == 2000:
                ser.write((str(2000)+"\n").encode("utf-8"))
                time.sleep(0.1)
                


    def avviathreads(self):
    
        thread_teachablemachine = threading.Thread(target=self.vediag)
        thread_seguilinea = threading.Thread(target=self.SeguiLinea)
        thread_seriale = threading.Thread(target=self.ConnessioneSeriale)
    
        #thread_teachablemachine.start()
        thread_seguilinea.start()
        thread_seriale.start()

    def vediag(self):
        Timerarg = time.time()
        while True:
                
            rs = vediargento(self.frame)
            if not rs:
                Timerarg = time.time()
            
            if Timerarg + 0.1 < time.time(): 
                print("ARGENTOOOO!")
                self.agvisto = True
                break
            


robot = Robot(True)
robot.avviathreads()


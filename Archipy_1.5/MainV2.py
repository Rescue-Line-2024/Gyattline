import cv2
import time
import threading
import serial
from verdi_e_linea import RicLinea,GestisciVerdi

video_capture = cv2.VideoCapture(-1)
ret , frame = video_capture.read()
image = frame.copy()



MotoriAccesi = False

deviazione = 0


lock = 1 #0=occupato 1=libero
 
def modsleep(tempo):
    sleeptemp = time.time()+tempo
    while(time.time()<sleeptemp):
        pass
    
        
def SeguiLinea():
    #-------------------------------------------------------#
    global frame,deviazione,MotoriAccesi
    video_capture.set(3, 320)
    video_capture.set(4, 240)
    timerv = time.time()
    last_deviazione = 0
    #-------------------------------------------------------#
    while True:
        arrayRED = 0
        ret, frame = video_capture.read()
        image = frame.copy()
        
        deviazione = RicLinea(image,last_deviazione)
        last_deviazione = deviazione
        verdeix = GestisciVerdi(frame,image)
        
        if time.time()>timerv:
            if verdeix == 1:
                deviazione = 300
                time.sleep(0.4)
                timerv = time.time()+0.5
                
            elif verdeix == 2:
                deviazione = -300
                time.sleep(0.4)
                timerv = time.time()+0.5
            
        
        if arrayRED > 0:
                print("ROSSO RILEVATO")
                MotoriAccesi = False
        else:
                MotoriAccesi = True
                        
            
    
        '''
        if time.time()>timerarg:
            timerarg = time.time()+0.3
            if(vediargento(frame.copy()) == True):
                deviazione = 2000
                time.sleep(0.5)
        '''
        
        
                                
        print("DEVIAZIONE : "+ str(deviazione))
        
        if MotoriAccesi:  
            pass

        else:
            deviazione = 1000
        
            #print("I don't see the line")
        cv2.imshow('Seguilinea', image)
        cv2.imshow('frame',frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            deviazione = 1000
            break
        
def ConnessioneSeriale():
    global lock
    devgraduale = 0
    
    ser = serial.Serial("/dev/ttyACM0",9600,timeout=1.0)
    time.sleep(3)
    ser.reset_input_buffer()
    print("Serial OK")

    
    while True:
                    time.sleep(0.1)
                
                    lock = 0
                    if devgraduale < deviazione-25:
                        devgraduale+=20
                    elif devgraduale > deviazione+25:
                        devgraduale-=20
                    else:
                        devgraduale = deviazione
                        
                    if MotoriAccesi:
                        try:
                            ser.write((str(-1 * int(devgraduale)) + "\n").encode("utf-8"))
                            print("success")
                        except serial.SerialException as e:
                            print(f"Serial write failed: {e}")
                        except Exception as e:
                            print(f"An unexpected error occurred: {e}")

                    else:
                        ser.write((str(1000)+"\n").encode("utf-8"))
                    lock = 1
                
                            
                    if deviazione == 1000:
                        print("Close Serial communication")
                        for i in range(2):
                            ser.write((str(1000)+"\n").encode("utf-8"))
                            time.sleep(0.1)
                        ser.close()
                    
                    if deviazione == 2000:
                        for i in range(2):
                            ser.write((str(2000)+"\n").encode("utf-8"))
                            time.sleep(0.1)

thread_seguilinea = threading.Thread(target=SeguiLinea )
thread_seriale = threading.Thread(target=ConnessioneSeriale )

thread_seriale.start()
thread_seguilinea.start()


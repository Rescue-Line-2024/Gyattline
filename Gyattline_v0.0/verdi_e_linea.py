import cv2
import numpy as np
from PID import gpPID
from ric_colori import RiconosciColori
import time

class Seguilinea:
    def __init__(self,P,I,D,cam_resolution,min_area=200,cut_percentage = 0.5):
        

        self.P,self.I,self.D = P , I , D

        self.cut_percentage = cut_percentage
        self.cam_x = int(cam_resolution[0])
        self.cam_y = int(cam_resolution[1])

        self.frame_x = None
        self.frame_y = None

        self.Pid_follow = gpPID(P,I,D, int(self.cam_x/2) ) 

        self.bottom_value_green = [35,40,40]
        self.upper_value_green = [75,255,255]

        self.Colors_detector = RiconosciColori(self.bottom_value_green,self.upper_value_green)
        self.frame = None
        self.pendenza = None
        self.min_area = min_area

    def segui_linea(self,frame):

            self.frame = frame  # Leggi il frame dalla camera
            frame_height = frame.shape[0]
            frame_line = frame.copy()[int(frame_height*self.cut_percentage):frame_height, :]

            self.frame_y = frame_line.shape[0]
            self.frame_x = frame_line.shape[1]
            posizione_linea = self.Colors_detector.riconosci_nero(image=frame_line, min_area=self.min_area)
            nero_coords = None

            if posizione_linea is not None:
                #----------CALCOLO PRIMO PID---------------#


                x, y, w, h = posizione_linea[0]
                y = y + int(self.cam_y*self.cut_percentage)
                posizione_linea[0] = (x,y,w,h)
                nero_coords = (x,y,w,h)
                self.Colors_detector.disegna_bbox(posizione_linea,frame,(0,0,0))

                A = (x,y)
                B = (x+w,y+h)
                        # Calcola centro in modo sicuro
                try:
                    Centerx = int((x + x + w) / 2)
                    Centery = int((y + y + h) / 2)
                    
                    Center = (Centerx, Centery)
                    
                    # Disegna il cerchio solo se il punto è valido
                    cv2.circle(frame, Center, 10, (255,0,0), -1)
            
                except Exception as e:
                    print(f"Errore nel calcolo del centro: {e}")

                centro_linea = (x + x + w )/ 2  

                #PRIMO PID : DEVIAZIONE TRA CENTRO DEL FRAME E LA LINEA#
                deviazione = self.Pid_follow.calcolopid(centro_linea)
                #PRIMO PID : DEVIAZIONE TRA CENTRO DEL FRAME E LA LINEA#

                #SECONDO PID : DISTANZA DAL PUNTO Csup E DAL PUNTO Cinf
                #Cinf è il punto in cui inizia la linea,Csup dove finisce

                

                '''
                la pendenza ci serve per capire se è piu urgente
                #raddrizzare la posizione della linea o seguirla
                '''
                #Guarda calcola_urgenza per capire come funziona
                points = self.TrovaCentriLinea(frame_line,10)
                if points is not None:
                    Cinf,Csup = points
                    try:
                        cv2.circle(frame,Cinf,10,(0,255,0),-1)
                        cv2.circle(frame,Csup,10,(0,0,255),-1)
                    except Exception as e:
                        print("errore nel disegnare i cerchi:",e)

                
                
                    self.pendenza = self.calcola_pendenza(Cinf, Csup, 1)*self.P
                    #si moltiplic per self.P per rendere equo il confronto
                    #tra il valore generato dal PID e il valore generato
                    #dal calcolo pendenza
                    
                    #abs : valore assoluto

                    if(abs(deviazione) > abs(self.pendenza)):
                        pass
                        #FAI SEGUILINEA
                    
                    else:
                        pass
                        #AGGIUSTA LINEA

                #print(f"DEVIAZIONE : {deviazione} PENDENZA LINEA: {self.pendenza}")
            else:
                pass

            return nero_coords

    def applica_movimento(self, deviazione):
        # Funzione per regolare il movimento dei motori in base alla deviazione calcolata
        potenzaDX, potenzaSX = self.Pid_follow.calcolapotenzamotori(deviazione)
        # Invia potenza ai motori (implementazione necessaria)
        print(f"Potenza Motore Destro: {potenzaDX}, Potenza Motore Sinistro: {potenzaSX}")

    def calcola_pendenza(self, A, B,moltiplicator):
        # Calcola la distanza tra x e x+w
        distanza_x = B[0] - A[0]  # distanza tra x e x+w
        h = float( (B[1] - A[1]) / self.cam_y )  # altezza h

        #PIU' H E' PICCOLA,PIU LA PENDENZA SARA ALTA E URGENTE
        #E ANDRA AD AUMENTARE NOTEVOLMENTE LA DISTANZA TRA X E X+W

        # Modifica la distanza in base all'altezza h
        if h > 0:  # Assicurati che h sia maggiore di zero per evitare divisione per zero
            distanza_modificata = (distanza_x / h) * moltiplicator   
        else:
            distanza_modificata = distanza_x  

        return distanza_modificata


    def TrovaCentriLinea(self, frame, offset):
        # Suddividi il frame nelle due parti
        
        down_part = frame.copy()[self.frame_y-offset:self.frame_y, :]
        up_part = frame.copy()[0:offset, :]

        # Riconosci le linee nelle due parti
        Bboxes1 = self.Colors_detector.riconosci_nero(down_part, 10)
        Bboxes2 = self.Colors_detector.riconosci_nero(up_part, 10)

        if Bboxes1 and Bboxes2:  # Controlla che entrambi non siano None
            x1, y1, w1, h1 = Bboxes1[0] #down
            x2, y2, w2, h2 = Bboxes2[0] #up
            y1+=self.cam_y-offset
            y2+=self.cam_y*self.cut_percentage

            print(f"y parte inferiore:{y1}")
            print(f"y parte superiore:{y2}")
            
            # Calcola i centri A e B
            A = (int((x1 + x1 + w1) / 2), int((y1 + y1 + h1) / 2))
            #SICCOME IL FRAME RITAGLIATO HA LE COORDINATE RELATIVE INDIPENDENTI DAL FRAME ORIGINALE
            #DOBBIAMO "PORTARE" IL CERCHIO SU NEL FRAME ORIGINALE
            B = (int((x2 + x2 + w2) / 2), int((y2 + y2 + h2) / 2))
            #qui non ce bisogno perchè y parte da 0 e non da un numero alto come in a

            print(f"DOWN:{A},UP:{B}")
            return (A,B)
        else:
            return None



        


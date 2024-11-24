import cv2
import numpy as np
from PID import gpPID
from ric_colori import RiconosciColori
import time

class Seguilinea:
    def __init__(self,P,I,D,cam_resolution,min_area = 5,):
        

        self.P,self.I,self.D = P , I , D

        self.cam_x = int(cam_resolution[0])
        self.cam_y = int(cam_resolution[1])

        self.Pid_follow = gpPID(P,I,D, int(self.cam_x/2) ) 

        self.bottom_value_green = [35,40,40]
        self.upper_value_green = [75,255,255]

        self.Colors_detector = RiconosciColori(self.bottom_value_green,self.upper_value_green)
        self.frame = None

        self.min_area = min_area

    def segui_linea(self,frame):

            self.frame = frame  # Leggi il frame dalla camera
            posizione_linea = self.Colors_detector.riconosci_nero(frame, self.min_area)
            

            if posizione_linea is not None:
                #----------CALCOLO PRIMO PID---------------#
                self.Colors_detector.disegna_bbox(posizione_linea,frame)


                x, y, w, h = posizione_linea[0]
                A = (x,y)
                B = (x+w,y+h)

                centro_linea = (x + x + w )/ 2  
                print("CENTRO LINEA:",centro_linea)

                #PRIMO PID : DEVIAZIONE TRA CENTRO DEL FRAME E LA LINEA#
                deviazione = self.Pid_follow.calcolopid(centro_linea)
                #PRIMO PID : DEVIAZIONE TRA CENTRO DEL FRAME E LA LINEA#

                #SECONDO PID : DISTANZA DAL PUNTO A(X,Y) E DAL PUNTO B(X+W,Y+H)

                

                '''
                la pendenza ci serve per capire se è piu urgente
                #raddrizzare la posizione della linea o seguirla
                '''
                #Guarda calcola_urgenza per capire come funziona

                pendenza = self.calcola_pendenza(A, B,   1)*self.P
                #si moltiplic per self.P per rendere equo il confronto
                #tra il valore generato dal PID e il valore generato
                #dal calcolo pendenza
                
                #abs : valore assoluto

                if(abs(deviazione) > abs(pendenza)):
                    pass
                    #FAI SEGUILINEA
                
                else:
                    pass
                    #AGGIUSTA LINEA

                print(f"DEVIAZIONE : {deviazione} PENDENZA LINEA: {pendenza}")
                

               

                
            else:
                pass

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



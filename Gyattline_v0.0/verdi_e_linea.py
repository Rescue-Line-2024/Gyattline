import cv2
import numpy as np
from PID import gpPID
from ric_colori import RiconosciColori
import time

class Seguilinea:
    def __init__(self,P,I,D,PEN,cam_resolution,min_area=200,cut_percentage=0.5):
        

        self.P,self.I,self.D = P , I , D #primo pid:calcolo distanza dalla linea
        self.PEN = PEN

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
        self.pendenza,self.deviazione = None,None
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

                posizione_linea = sorted(posizione_linea, key=lambda box: box[2] * box[3], reverse=True)

                x, y, w, h = posizione_linea[0]
                #la y del frame originale
                y_original = y + int(self.cam_y*self.cut_percentage)
                posizione_linea[0] = (x,y_original,w,h) #queste sono le coordinate assolute,le useremo per disegnare
                nero_coords = (x,y,w,h)#queste sono le coordinate relative con cui lavoreremo
                
                self.Colors_detector.disegna_bbox((posizione_linea[0],),frame,(0,0,0))

                A = (x,y)
                B = (x+w,y+h)
                        # Calcola centro in modo sicuro
                try:
                    Centerx_Abs = (x + x + w) // 2
                    Centery_Abs = (y + y + h) // 2
                    
                    
                    
            
                except Exception as e:
                    print(f"Errore nel calcolo del centro assoluto: {e}")


                #PRIMO PID : (guarda giu nell if)

                #SECONDO PID : DISTANZA DAL PUNTO Csup E DAL PUNTO Cinf
                #Cinf è il punto in cui inizia la linea,Csup dove finisce

                

                '''
                la pendenza ci serve per capire se è piu urgente
                #raddrizzare la posizione della linea o seguirla
                '''
                #Guarda calcola_urgenza per capire come funziona
                points = self.TrovaCentriLinea(frame_line,10,y)

                #trovacentrilinea torna 0 quando sono stati rilevati una quantità diversa da due punti
                if points is not None and points  != 0:
                    
                    Cinf,Csup = points
                    centro_linea_x = (Cinf[0]+Csup[0])//2
                    centro_linea_y = (Cinf[1]+Csup[1])//2
                    cv2.circle(frame,(centro_linea_x,centro_linea_y),10,(255,0,0),-1)
                    #media tra punto inferiore e punto superiore della linea
                    #per trovarci la x centrale

                
                    self.deviazione = self.Pid_follow.calcolopid(centro_linea_x)
                    try:
                        cv2.circle(frame,Cinf,10,(0,255,0),-1)
                        cv2.circle(frame,Csup,10,(0,0,255),-1)
                    except Exception as e:
                        print("errore nel disegnare i cerchi:",e)

                
                
                    self.pendenza = int(self.calcola_pendenza(A=Cinf, B=Csup, moltiplicator=self.PEN)*self.P)
                    #piu il moltiplicatore è alto,più la linea sarà riconosciuta pendente

                    #si moltiplic per self.P per rendere equo il confronto
                    #tra il valore generato dal PID e il valore generato
                    #dal calcolo pendenza
                    
                    
                    self.trovata_t(Cinf,Csup,nero_coords)

                    if(abs(self.deviazione) > abs(self.pendenza)):
                        print(f"deviazione ha la priorità con: {self.deviazione}")
                    
                    else:
                        print(f"pendenza ha la priorità con {self.pendenza}")
                        if Cinf[0] < Csup[0]:
                            print("DESTRA!")
                            #muovi i motori 100,-100 per andare a destra
                        else:
                            print("SINISTRA!")
                            #muovi i motori -100,100 per andare a sinistra
                else:
                    #se uno dei due punti per un qualsiasi motivo non è stato trovato
                    #il centro linea sarà il centro della bounding box
                    centro_linea_x = (x+x+w)//2
                    centro_linea_y = (y_original+y_original+h)//2
                    cv2.circle(frame,(centro_linea_x,centro_linea_y),10,(255,0,0),-1)
                    if points != 0:
                        self.deviazione = self.Pid_follow.calcolopid(centro_linea_x)
                        print("Deviazione normale : ",self.deviazione)       

            else:
                pass

            return nero_coords

    def trovata_t(self,Cinf,Csup,posiz_linea):
            x,y,w,h = posiz_linea
            #verifichiamo che punto superiore e inferiore siano piu o meno allineati al centro
            if( (Cinf[0] > self.cam_x//2 - self.cam_x//10 and Cinf[0] < self.cam_x//2 + self.cam_x//10)
            or(Csup[0] > self.cam_x//2 - self.cam_x//10 and Csup[0] < self.cam_x//2 + self.cam_x//10) ):
                if(h >= self.frame_y - 3 and( x<3 or x+w>self.cam_x-3)):
                    pass 
                    #da completare

    def applica_movimento(self, deviazione):
        # Funzione per regolare il movimento dei motori in base alla deviazione calcolata
        potenzaDX, potenzaSX = self.Pid_follow.calcolapotenzamotori(deviazione)
        # Invia potenza ai motori (implementazione necessaria)
        print(f"Potenza Motore Destro: {potenzaDX}, Potenza Motore Sinistro: {potenzaSX}")
        return potenzaDX,potenzaSX

    def calcola_pendenza(self, A, B,moltiplicator):
        
        distanza_x = abs(B[0] - A[0] )
        h = float( abs(B[1] - A[1]) / self.frame_y )  # altezza h,
        #la dividiamo per la massima altezza del frame cosi h parte da 1

        #PIU' H E' PICCOLA,PIU LA PENDENZA SARA ALTA E URGENTE,che fa da moltiplicatore

        # Modifica la distanza in base all'altezza h
        if h > 0:  
            distanza_modificata = (distanza_x / h) * moltiplicator   
        else:
            distanza_modificata = distanza_x  

        return distanza_modificata


    def TrovaCentriLinea(self, frame, offset,start):
        # Suddividi il frame nelle due parti
        
        down_part = frame.copy()[self.frame_y-offset:self.frame_y, :]
        up_part = frame.copy()[0:offset, :]

        if (start+offset) < self.frame_y:
            up_part = frame.copy()[start:start+offset, :] #possibilità di uscire dal frame originale
        else:
            up_part = frame.copy()[start:self.frame_y, :] #cosi non esce dal frame
      
        if down_part.size == 0 or up_part.size == 0:
            return None

        # Riconosci le linee nelle due parti
        Bboxes1 = self.Colors_detector.riconosci_nero(down_part, 10)
        Bboxes2 = self.Colors_detector.riconosci_nero(up_part, 10)

        if Bboxes1 and Bboxes2:  # Controlla che entrambi non siano None
            if (len(Bboxes1)+len(Bboxes2)) == 2:
                x1, y1, w1, h1 = Bboxes1[0] #down
                x2, y2, w2, h2 = Bboxes2[0] #up
                y1+=self.cam_y-offset#aggiustiamo la sfasatura pure qui(y del pallino inferiore)
                y2+=start+(self.cam_y*self.cut_percentage)#sfasatura del pallino superiore

                
                # Calcola i centri A e B
                A = (int((x1 + x1 + w1) / 2), int((y1 + y1 + h1) / 2))
                #SICCOME IL FRAME RITAGLIATO HA LE COORDINATE RELATIVE INDIPENDENTI DAL FRAME ORIGINALE
                #DOBBIAMO "PORTARE" IL CERCHIO SU NEL FRAME ORIGINALE
                B = (int((x2 + x2 + w2) / 2), int((y2 + y2 + h2) / 2))
                #qui non ce bisogno perchè y parte da 0 e non da un numero alto come in a

            
                return (A,B) #rispettivamente down e up
            else:
                return 0 
                #non siamo riusciti a stabilire due punti specifici
                #quindi vuol dire che la linea è urgentemente disallineata
                #nel main faremo una curvatura totale a destra o a sinistra
            
        else:
            return None



        


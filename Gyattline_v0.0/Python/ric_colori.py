import numpy as np
import cv2

class RiconosciColori:
    thresh = None # Variabile per conservare la maschera binaria

    def __init__(self, bottom_value,upper_value,min_area):
        self.min_area = min_area
        self.bottom_value = np.array(bottom_value)
        self.upper_value = np.array(upper_value)

        #dal main io passerò un array a parametro:quello dell'inizio dell intervallo
        #e quello dela fine dell'intervallo esempio:rosso = RiconosciColori([170,0,0],[255,70,70])
        self.interval = (np.array(bottom_value),np.array(upper_value))
        

    def riconosci_colore_mask(self, image):
        """Riconosce i colori nell'immagine e restituisce le coordinate dei bounding box."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.bottom_value, self.upper_value)

        return mask
    
    def riconosci_colore(self,image):

        mask = self.riconosci_colore_mask(image)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        array = []
        for contour in contours:
            if cv2.contourArea(contour) > self.min_area:
                x, y, w, h = cv2.boundingRect(contour)    
                array.append([x, y, w, h])
        
        return array if array else None  # Restituisce None se non ci sono contorni
    
    def disegna_bbox(self,array,image,color):
        coordinate = array
        for x,y,w,h in coordinate:
            cv2.rectangle(image,(x,y),(x+w,y+h),color,1)
        

    def riconosci_mask(self,image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        ret, threshold = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY_INV)

        RiconosciColori.thresh = threshold

        return threshold

    def riconosci_nero(self, image):
        threshold = self.riconosci_mask(image)

        contours, hierarchy = cv2.findContours(threshold.copy(), 1, cv2.CHAIN_APPROX_NONE)

        # Filtro eventuali punti neri che danno fastidio
        # Filtraggio e creazione dei bounding box
        bounding_boxes = [
            cv2.boundingRect(contour)  
            for contour in contours #itera in ogni contorno,si trova l'area e vede
            if cv2.contourArea(contour) > self.min_area        #se l'area è sufficiente,se si genera la BBox 
        ]


        return bounding_boxes if bounding_boxes else None
    
    def riconosci_nero_tagliato(self,image,frame_height,cut_percentage):
        threshold = self.riconosci_mask(image)

        cut_tresh = threshold.copy()[int(frame_height*cut_percentage):frame_height, :]

        contours, hierarchy = cv2.findContours(cut_tresh,1,cv2.CHAIN_APPROX_NONE)
    
        bounding_boxes = [
            cv2.boundingRect(contour)  
            for contour in contours #itera in ogni contorno,si trova l'area e vede
            if cv2.contourArea(contour) > self.min_area        #se l'area è sufficiente,se si genera la BBox 
        ]

        bounding_boxes = sorted(bounding_boxes, key=lambda box: box[2] * box[3], reverse=True)#prima le BB più grandi

        return bounding_boxes if bounding_boxes else None
    
    def riconosci_verdi(self,image):
        #qui ora si lavora con le due mask confrontandole
        #le mask dovrebbero essere entrambi delle stesse dimensioni del frame originale
        mask_verde = self.riconosci_colore_mask(image).copy()
        
        if RiconosciColori.thresh is None:
            return None
        
        mask_nero = RiconosciColori.thresh.copy()


        # Conta il numero di `1` in ciascuna riga
        sums = np.sum(mask_nero, axis=1)

        # Trova l'indice della riga con il massimo numero di `1`
        max_row_index = np.argmax(sums)


        #tagliamo la parte superiore dell'incrocio
        try:
            mask_verde_cutted = mask_verde[max_row_index + 1:, :]
            mask_nero_cutted = mask_nero[max_row_index +1:,:]
        except:
            print("impossibile tagliare la mask in valutazione verdi")
            return 0

        contours, _ = cv2.findContours(mask_verde_cutted, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        

        if len(contours) > 0:


            bounding_boxes = [
                cv2.boundingRect(contour)  
                for contour in contours #itera in ogni contorno, si trova l'area e vede
                if cv2.contourArea(contour) > self.min_area  #se l'area è sufficiente, genera la BBox 
            ]

            bounding_boxes = sorted(bounding_boxes, key=lambda box: box[2] * box[3], reverse=True)  # prima le BB più grandi

            # Correggi le coordinate delle bounding box
            bounding_boxes = [(x, y + max_row_index + 1, w, h) for x, y, w, h in bounding_boxes]

            self.disegna_bbox(bounding_boxes, image, (0, 255, 0))

            verdi_valutati = [] #SARA' UN ARRRAY DI DICT
            for box in bounding_boxes:
                x,y,w,h = box
                center_x = (x+x+w)//2
                center_y = (y+y+h)//2-max_row_index 
                #sopra abbiamo sfasato le coordinate del verde di +max_row_index per farle vedere
                #bene a schermo,quindi ora dobbiamo toglierli max_row_index alla y
                #per far combaciare le coordinate di nero e verde

                

                # Dividi la maschera nera tagliata a sinistra e a destra di center_x
                nero_sinistra = np.sum(mask_nero_cutted[center_y:, :center_x].copy())
                nero_destra = np.sum(mask_nero_cutted[center_y:, center_x:].copy())

                

                # Determina dove c'è più nero
                if nero_destra > nero_sinistra:
                    posizione = {'position': "SX", 'coords': (x, y, w, h)} #quindi verde a sinistra
                elif nero_destra < nero_sinistra:
                    posizione = {'position': "DX", 'coords': (x, y, w, h)} #quindi verde a destra
                else:
                    posizione = {'position': "CENTER", 'coords': (x, y, w, h)}

                verdi_valutati.append(posizione)
            return verdi_valutati

        # Se non ci sono contorni verdi validi
        return None
    
    
        

                
        
        
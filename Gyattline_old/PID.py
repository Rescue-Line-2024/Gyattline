import time
class gpPID:
    def __init__(self,P,I,D,setpoint):
            self.motoreDX = None
            self.motoreSX = None
   
            
            self.__KP = P
            self.__KI = I
            self.__KD = D

            self.__setpoint = setpoint

            self.__error = 0
            self.lastgradi = 0

            self.__integral = 0
            self.__last_error = 0
            
    def set_setpoint(self,value):
        self.__setpoint = value

    def calcolopid(self,value):
        error = (int(self.__setpoint) - int(value))
        
        integral = 0
        derivative = 0

        proportional = self.__KP * error
        integral += self.__KI*error
        
        try:
            derivative = self.__KD *(error-last_error)
        except:
            pass

        last_error = error

        turn_rate = proportional + integral + derivative

        return turn_rate

    def initmotori(self,DX,SX):
        self.motoreDX = DX
        self.motoreSX = SX
        
    def limitamotori(self,DX,limite):
        if DX > limite:
              return limite
            
        elif DX < -1*limite:
              return -1*limite
            
        else:
              return DX
        
    def calcolapotenzamotori(self,valore):
        deviazione = self.calcolopid(valore)
        potenzaDX = 100+deviazione
        potenzaSX = 100-deviazione
        potenzaDX = self.limitamotori(potenzaDX,100)
        potenzaSX = self.limitamotori(potenzaSX,100)
      
        print(f"MOTORE DESTRO:{potenzaDX} MOTORE SINISTRO:{potenzaSX} DEVIAZIONE:{deviazione}")
        return (int(potenzaDX/2.2),int(potenzaSX/2.2))
    
    def calcolopotenzaservopicarx(self,valore):
        deviazione = self.calcolopid(valore) #tengo traccia della posizione del servo
        Xc = self.__setpoint
        poservo = 0
        poservo = poservo - deviazione 
        poservo = self.limitamotori(poservo,45)-10
        
        self.ServoDir.angle(poservo)
        self.lastgradi = poservo
        print("graduazione del servo:",int(poservo),",equivarrebbe a una curvatura di:",int(poservo+10))



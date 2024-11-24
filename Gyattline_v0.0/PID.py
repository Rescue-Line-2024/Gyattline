import time

class gpPID:
    def __init__(self, P, I, D, setpoint = 0):
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

    def set_setpoint(self, value):
        self.__setpoint = value

    def calcolopid(self, value):
        error = int(self.__setpoint) - int(value)
        
        # Accumula l'integrale
        self.__integral += self.__KI * error
        
        # Calcola la derivata
        derivative = self.__KD * (error - self.__last_error)

        # Aggiorna l'errore precedente
        self.__last_error = error

        # Calcola il valore di controllo
        turn_rate = (self.__KP * error) + self.__integral + derivative

        return turn_rate

    def initmotori(self, DX, SX):
        self.motoreDX = DX
        self.motoreSX = SX

    def limitamotori(self, DX, limite):
        return max(min(DX, limite), -limite)

    def calcolapotenzamotori(self, valore):
        deviazione = self.calcolopid(valore)
        potenzaDX = self.limitamotori(100 + deviazione, 100)
        potenzaSX = self.limitamotori(100 - deviazione, 100)

        print(f"MOTORE DESTRO:{potenzaDX} MOTORE SINISTRO:{potenzaSX} DEVIAZIONE:{deviazione}")
        return (int(potenzaDX / 2.2), int(potenzaSX / 2.2))

    def calcolopotenzaservopicarx(self, valore):
        deviazione = self.calcolopid(valore)
        poservo = self.limitamotori(-deviazione - 10, 45)

        self.ServoDir.angle(poservo)
        self.lastgradi = poservo
        print("graduazione del servo:", int(poservo), ",equivarrebbe a una curvatura di:", int(poservo + 10))
import cv2
import numpy as np
from PID import gpPID
from ric_colori import RiconosciColori
import time

class Seguilinea:
    def __init__(self,P,I,D):
        self.Pid_follow = gpPID(P,I,D) 
        self.Pid_adjust = gpPID(P,0,0)


class Verdi:
    pass
import serial
import config
import warnings
import numpy as np

NUM_SERVO = 7; #should not be changed
class Flag:
    begin = 0xFE
    status = 0xF5
    speed = 0xF7
    angle = 0xF9
    IK6 = 0xFA
    IK5 = 0xFB
    IK3 = 0xFC
    read = 0xF9
    mask = 0x7F

class Arm:
    def __init__(self):
        try:
            self.port = serial.Serial(config.PORT_NAME, config.BAUD_RATE, timeout=1);
        except x:
            warnings.warn("can't open the serial port to control the robot arm");
            raise x;

        #to be set to control the arm
        self.fluent = np.array([True]*NUM_SERVO, dtype=np.bool);
        self.speed = np.array([50]*NUM_SERVO, dtype=np.int);
        self.angle = np.array([90,145,70,90,90,90,45],dtype=np.float32);
        assert(len(self.angle) == NUM_SERVO);
        self._doFluentSpeed();
        self._doAngle();
        self.isAllConverged = True;
        #read from the arm
        self._force = np.array([0]*NUM_SERVO, dtype=np.int);
        self._pos = np.array([0]*NUM_SERVO, dtype=np.float32);

    def __del__(self):
        self.port.close()
    
    def _do(self, command):
        self.port.write(command)

    def _doFluentSpeed(self):
        s = np.clip(self.speed,0,250)//10;
        s[self.fluent] += 64;
        command = bytes([Flag.begin,Flag.speed] + [Flag.mask & x for x in s]);
        self._do(command)

    def _doAngle(self): 
        self.isAllConverged = False
        c = np.array(self.angle * 50/9, dtype = np.int16)
        command = [Flag.begin,Flag.angle]
        for x in c:
            command += [(x//128) & Flag.mask, x & Flag.mask]
        command = bytes(command)
        self._do(command)

    def setForceStatus(self, status):#0-forceless, 1-normal, 2-protected
        c = bytes([Flag.begin, Flag.status, Flag.mask & status]);
        self._do(c)

    def setSpeed(self,servo_speed):  #set servo speed , 0-250
        for servo,sp in servo_speed.items():
            self.speed[servo] = sp;
        self._doFluentSpeed();
    
    def setFluent(self,servo_fluent): #set servo fluent or not, true or false
        for servo,fl in servo_fluent.items():
            self.fluent[servo] = fl;
        self._doFluentSpeed();
    
    def setAngle(self,servo_angle): #set servo angles
        for servo,ang in servo_angle.items():
            self.angle[servo] = ang;
        ##shoudn't we limit the range of the angels here!!
        self._doAngle();

    def read(self):#get force and position, needs to keep updating or else it won't return the laste force and position!!
        data = bytes(2*NUM_SERVO+1);
        #read bytes
        while True:
            c = self.port.read()[0]
            if c == Flag.begin:
                c = self.port.read()[0]
                if c == Flag.read:
                    break

        data = self.port.read(len(data))

        if data[-1] == 1:
            self.isAllConverged = True
        else:
            self.isAllConverged = False
        for i in range(NUM_SERVO):
            f = data[i*2]>>3;
            # -7 to 7
            self._force[i] = (f & 0x07) * (1 if (f>>3)==0 else 0);
            # 0 to 180 degree
            self._pos[i] = (9.0/50)*((data[i*2] & 0x07) * 128 + data[i*2+1]);

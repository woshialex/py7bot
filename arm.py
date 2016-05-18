import serial
import config
import warnings
import numpy as np

NUM_SERVO = 7; #should not be changed
class Arm:
    def __init__(self):
        try:
            self.port = serial.Serial(config.PORT_NAME, config.BAUD_RATE);
        except x:
            warnings.warn("can't open the serial port to control the robot arm");
            raise x;

        self.fluent = np.array([True]*NUM_SERVO, dtype=np.bool);
        self.speed = np.array([50]*NUM_SERVO, dtype=np.int);
        self._doFluentSpeed();
        self.angle = np.array([90,145,70,90,90,90,45],dtype=np.float32);
        assert(len(self.angle) == NUM_SERVO);
        self.isAllConverged = True;
        self._doAngle();

    def __del__(self):
        self.port.close()
    
    def _do(self, command):
        self.port.write(command)

    def _doFluentSpeed(self):#can we do one at a time?? 
        s = np.clip(self.speed,0,250)//10;
        s[self.fluent] += 64;
        command = bytes([0xFE,0xF7] + [0x7F & x for x in s]);
        self._do(command)

    def _doAngle(self): #can we do one at a time?
        self.isAllConverged = False
        c = np.array(self.angle * 50/9, dtype = np.int16)
        command = [0xFE,0xF9]
        for x in c:
            command += [(x//128) & 0x7F, x & 0x7F]
        command = bytes(command)
        self._do(command)

    def setForceStatus(self, status):#0-forceless, 1-normal, 2-protected
        c = bytes([0xFE, 0xF5, 0x7F & status]);
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

    ## need to add serialEvent to read from the servos to check if the move is done or not

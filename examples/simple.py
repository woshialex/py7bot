import sys
sys.path.append("..");
from arm import Arm;

if __name__=="__main__":
    arm = Arm();
    angles = {1:95, 2:65};
    arm.setAngle(angles);
    del arm


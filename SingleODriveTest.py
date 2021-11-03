import sys
import time
import odrive
from odrive.enums import *

class GearBox:

    def __init__(self):
        print("Trying to find drive")
        self.drive = odrive.find_any()
        self.drvAxis1 = self.drive.axis0
        self.drvAxis2 = self.drive.axis1

    def mode_idle(self):

        self.drvAxis1.requested_state = AXIS_STATE_IDLE
        self.drvAxis2.requested_state = AXIS_STATE_IDLE
    
    def move_input_pos(self, angle):
        
        self.drvAxis1.controller.input_pos = angle/360.0
        self.drvAxis2.controller.input_pos = angle/360.0
    def circularMove(self, increment):
        self.drvAxis1.controller.input_pos = increment
        self.drvAxis2.controller.input_pos = increment

if __name__ == "__main__":
    gearbox = GearBox()
    print("Motor test time")
    gearbox.circularMove(0.25)
    gearbox.mode_idle()
                                         
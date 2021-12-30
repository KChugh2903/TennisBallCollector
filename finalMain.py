from finalRobot import Robot
from pyPS4Controller.controller import Controller

class MyController(Controller):
    def __init__(self, great):
        Controller.__init__(self, great)
        self.rbt = Robot(0,1)
    def on_L3_up(self):
        self.rbt.forward()

    def on_L3_down(self):
        self.rbt.backward()

    def on_R3_left(self):
        self.rbt.turnLeft()

    def on_R3_left(self):
        self.rbt.turnRight()

    def on_L3_x_at_rest(self):
        self.rbt.driveRest()

    def on_L3_y_at_rest(self):
        self.rbt.driveRest()

    def on_R3_x_at_rest(self):
        self.rbt.driveRest()

    def on_R3_y_at_rest(self):
        self.rbt.driveRest()
        
    def on_x_press(self):
        self.rbt.intake()

    def on_x_release(self):
        self.rbt.intakeRest()

controller = MyController(interface = "/dev/input/js0",connecting_using_ds4drv = False)
controller.listen(timeout = 60)
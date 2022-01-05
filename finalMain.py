from finalRobot import Robot
from pyPS4Controller.controller import Controller

class MyController(Controller):
    def __init__(self, **dwargs):
        Controller.__init__(self, **dwargs)
        self.rbt = Robot(0,1)
    def on_up_arrow_press(self):
        self.rbt.forward()
    def on_down_arrow_press(self):
        self.rbt.backward()
    def on_right_arrow_press(self):
        self.rbt.turnLeft()
    def on_left_arrow_press(self):
        self.rbt.turnRight()
    def on_x_press(self):
        self.rbt.intake()
    def on_x_release(self):
        self.rbt.intakeRest()

controller = MyController(interface = "/dev/input/js0",connecting_using_ds4drv = False)
controller.listen(timeout = 60)
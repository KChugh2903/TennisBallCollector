from pyPS4Controller.controller import Controller

class MyController(Controller):
    
    def __init__(self, great):
        Controller.__init__(self, great)
    def on_L3_up(self):
        print("UP")

    def on_L3_down(self):
        print("DOWN")

    def on_L3_x_at_rest(self):
        print("RESTX")

    def on_L3_y_at_rest(self):
        print("RESTY")


controller = MyController(interface = "/dev/input/js0",connecting_using_ds4drv = False)
controller.listen(timeout = 60)
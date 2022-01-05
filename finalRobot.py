import threading
import time
import odrive
from odrive.enums import *


class Robot:
    KV_SMALL = 150
    KV_LARGE = 270


    def __init__(self, axis1, axis2):

        self.axis1 = axis1
        self.axis2 = axis2
        self.KV_SMALL = 150
        self.KV_LARGE = 270
        self.event = threading.Event()
        print("Looking for ODrives: ")
        self.findDrives()
        print("Found it")

    def findDrives(self):

        self.drive0 = odrive.find_any(serial_number="206C39974D4D")
        self.drive1 = odrive.find_any(serial_number="208D39934D4D")
        

        print(self.drive0.vbus_voltage)
        print(self.drive1.vbus_voltage)

        self.left = self.drive0.axis0
        self.right = self.drive0.axis1
        self.frontIntake = self.drive1.axis0
        self.backIntake = self.drive1.axis1

        self.left.motor.config.currrent_lim = 40 #no cooling
        self.right.motor.config.currrent_lim = 40 #no cooling
        self.frontIntake.motor.config.currrent_lim = 60 #idle air cooling
        self.backIntake.motor.config.currrent_lim = 60 #idle air cooling


        self.left.motor.config.pole_pairs = 7
        self.right.motor.config.pole_pairs = 7
        self.frontIntake.motor.config.pole_pairs = 7
        self.backIntake.motor.config.pole_pairs = 7

        self.left.motor.config.torque_constant = 8.27 / self.KV_SMALL
        self.right.motor.config.torque_constant = 8.27 / self.KV_SMALL
        self.frontIntake.motor.config.torque_constant = 8.27/ self.KV_SMALL
        self.backIntake.motor.config.torque_constant = 8.27 / self.KV_LARGE

        self.left.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.left.encoder.config.mode = ENCODER_MODE_HALL
        #feedforward implementation
        self.left.controller.config.input_mode = INPUT_MODE_POS_FILTER
        self.left.controller.config.input_filter_bandwidth = 2

        self.right.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.right.encoder.config.mode = ENCODER_MODE_HALL
        #feedforward implementation
        self.right.controller.config.input_mode = INPUT_MODE_POS_FILTER
        self.right.controller.config.input_filter_bandwidth = 2

        self.frontIntake.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.frontIntake.encoder.config.mode = ENCODER_MODE_HALL
        self.backIntake.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.backIntake.encoder.config.mode = ENCODER_MODE_HALL

        self.left.encoder.config.cpr = 8192
        self.right.encoder.config.cpr = 8192
        self.frontIntake.encoder.config.cpr = 8192
        self.backIntake.encoder.config.cpr = 8192


        self.calibration_sequence()
        time.sleep(10)
        self.closed_control()
        self.left.controller.input_pos = 0
        self.right.controller.input_pos = 0

        self.tuneForPosition(self.left)
        self.tuneForPosition(self.right)

    def calibration_sequence(self):
        self.left.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        time.sleep(15)
        self.right.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        time.sleep(15)
        self.frontIntake.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        time.sleep(15)
        self.backIntake.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    def closed_control(self):
        self.left.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(5)
        self.right.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(5)
        self.frontIntake.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(5)
        self.backIntake.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    def forward(self):
        self.left.controller.input_pos += 20
        self.right.controller.input_pos -= 20

    def backward(self):
        self.left.controller.input_pos -= 20
        self.right.controller.input_pos += 20

    def turnRight(self):
        self.left.controller.input_pos += 20
        self.right.controller.input_pos += 20
        
    def turnLeft(self):
        self.left.controller.input_pos -= 20
        self.right.controller.input_pos -= 20

    def intake(self):
        self.frontIntake.controller.input_vel = -15
        self.backIntake.controller.input_vel = 10

    def intakeRest(self):
        self.frontIntake.controller.input_vel = 0
        self.backIntake.controller.input_vel = 0
    """
    This part is the tuning function that tunes the drive motors.
    It was inspired by the Ziegler-Nichols process and follows the ODrive documentation.
    """

    def tuneForPosition(self,motor):
        motor.motor.config.current_control_bandwidth = 1000 #autotunes current_gain and current_integrator_gain
        motor.controller.config.pos_gain = 1
        motor.controller.config.vel_gain = 0.01
        motor.controller.config.vel_integrator_gain = 0
        

        first_current = motor.motor.current_control.Iqmeasured
        self.event.wait(1)
        second_current = motor.motor.current_control.Iqmeasured
        current_error = second_current - first_current

        while current_error < 7: #increases vel_gain until motor starts to vibrate. Motor current jumps a lot once vibrates
            first_current = second_current
            motor.controller.config.vel_gain *= 1.3
            motor.controller.input_pos += 1
            self.event.wait(1)
            second_current = motor.motor.current_control.Iqmeasured
            current_error = second_current - first_current

        motor.controller.config.vel_gain *= 0.5
        first_reading = motor.encoder.count_in_cpr 
        second_reading = motor.encoder.count_in_cpr + 8192
        error = second_reading - first_reading
        while error < 500: #increases pos_gain until motor overshoots
            motor.controller.config.pos_gain *= 1.5
            motor.controller.input_pos += 2
            first_reading = second_reading
            second_reading = motor.encoder.count_in_cpr
            error = second_reading - first_reading

        motor.controller.config.pos_gain -= 1 #prevents overshooting
        motor.controller.config.vel_integrator_gain = 0.5 * 1/0.03 * motor.controller.config.vel_gain #bandwidth was 30ms in this system



        

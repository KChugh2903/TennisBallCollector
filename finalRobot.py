import sys
import time
import odrive
from odrive.enums import *
from fibre.protocol import ChannelBrokenException

class Robot:
    KV_DRIVE = 150
    KV_INTAKE = 270

    MIN_PHASE_INDUCTANCE = 0
    MAX_PHASE_INDUCTANCE = 0.001

    MIN_PHASE_RESISTANCE = 0
    MAX_PHASE_RESISTANCE = 0.5

    ENCODER_OFFSET_FLOAT_TOLERANCE = 0.05

    def __init__(self, axis1, axis2):

        self.axis1 = axis1
        self.axis2 = axis2

        print("Looking for ODrives: ")
        self.findDrives()
        print("Found it")
    def findDrives(self):
        allOdrives = list(odrive.find_all(...))

        self.drive0 = allOdrives[0]
        self.drive1 = allOdrives[1]


        print(self.drive0.vbus_voltage)
        print(self.drive1.vbus_voltage)

        self.left = self.drive0.axis0
        self.right = self.drive0.axis1
        self.frontIntake = self.drive1.axis0
        self.backIntake = self.drive1.axis1

        self.left.motor.config.pole_pairs = 7
        self.right.motor.config.pole_pairs = 7
        self.frontIntake.motor.config.pole_pairs = 7
        self.backIntake.motor.config.pole_pairs = 7

        self.left.motor.config.torque_constant = 8.27 * self.left.motor.current_control.Iq_measured/ 150
        self.right.motor.config.torque_constant = 8.27 * self.left.motor.current_control.Iq_measured/ 150
        self.frontIntake.motor.config.torque_constant = 8.27 * self.left.motor.current_control.Iq_measured/ 150
        self.backIntake.motor.config.torque_constant = 8.27 * self.left.motor.current_control.Iq_measured/ 270

        self.left.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.left.encoder.config.mode = ENCODER_MODE_HALL
        self.right.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.right.encoder.config.mode = ENCODER_MODE_HALL
        self.frontIntake.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.frontIntake.encoder.config.mode = ENCODER_MODE_HALL
        self.backIntake.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.backIntake.encoder.config.mode = ENCODER_MODE_HALL

        self.left.encoder.config.cpr = 8192
        self.right.encoder.config.cpr = 8192
        self.frontIntake.encoder.config.cpr = 8192
        self.backIntake.encoder.config.cpr = 8192

    def forward(self):

        self.left.controller.input_vel = 20
        self.right.controller.input_vel = -20
    def backward(self):

        self.left.controller.input_vel = -20
        self.right.controller.input_vel = 20
    def turnRight(self):

        self.left.controller.input_vel = 20
        self.right.controller.input_vel = 20
        
    def turnLeft(self):
        self.left.controller.input_vel = -20
        self.right.controller.input_vel = -20
    def driveRest(self):
        self.left.controller.input_vel = 0
        self.right.controller.input_vel = 0
    def intake(self):
        self.frontIntake.controller.input_vel = -15
        self.backIntake.controller.input_vel = 10
    def intakeRest(self):
        self.frontIntake.controller.input_vel = 0
        self.backIntake.controller.input_vel = 0
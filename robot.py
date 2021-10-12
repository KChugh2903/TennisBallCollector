import sys
import time
import odrive
from odrive.enums import *
from fibre.protocol import ChannelBrokenException

class Robot:
    #General constants for the motors. Constants were found in the motor guide in ODrive
    KV = 150

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

        self.drive1 = allOdrives[0]
        self.drive2 = allOdrives[1]
        self.drive3 = allOdrives[2]

        self.drvAxis1 = getattr(self.drive1, "axis".format(self.axis1))
        self.drvAxis2 = getattr(self.drive1, "axis".format(self.axis2))
        self.drvAxis3 = getattr(self.drive2, "axis".format(self.axis1))
        self.drvAxis4 = getattr(self.drive2, "axis".format(self.axis2))
        self.drvAxis5 = getattr(self.drive3, "axis".format(self.axis1))

    def configure(self):
        #configuration
        print("Erasing pre-existing configuration")
        try:
            self.drive1.erase_configuration()
            self.drive2.erase_configuration()
            self.drive3.erase_configuration()

        except ChannelBrokenException:
            pass

        self.findDrives()

        self.drvAxis1.motor.config.pole_pairs = 7
        self.drvAxis2.motor.config.pole_pairs = 7
        self.drvAxis3.motor.config.pole_pairs = 7
        self.drvAxis4.motor.config.pole_pairs = 7
        self.drvAxis5.motor.config.pole_pairs = 7
        

        self.drvAxis1.motor.config.resistance_calib_max_voltage = 4
        self.drvAxis1.motor.config.requested_current_range      = 25
        self.drvAxis1.motor.config.current_control_bandwidth    = 100

        self.drvAxis2.motor.config.resistance_calib_max_voltage = 4
        self.drvAxis2.motor.config.requested_current_range      = 25
        self.drvAxis2.motor.config.current_control_bandwidth    = 100

        self.drvAxis3.motor.config.resistance_calib_max_voltage = 4
        self.drvAxis3.motor.config.requested_current_range      = 25
        self.drvAxis3.motor.config.current_control_bandwidth    = 100

        self.drvAxis4.motor.config.resistance_calib_max_voltage = 4
        self.drvAxis4.motor.config.requested_current_range      = 25
        self.drvAxis4.motor.config.current_control_bandwidth    = 100

        self.drvAxis5.motor.config.resistance_calib_max_voltage = 4
        self.drvAxis5.motor.config.requested_current_range      = 25
        self.drvAxis5.motor.config.current_control_bandwidth    = 100

        self.drvAxis1.motor.config.torque_constant = 8.27 * self.drvAxis1.motor.current_control.Iq_measured/ self.KV
        self.drvAxis2.motor.config.torque_constant = 8.27 * self.drvAxis2.motor.current_control.Iq_measured/ self.KV
        self.drvAxis3.motor.config.torque_constant = 8.27 * self.drvAxis3.motor.current_control.Iq_measured/ self.KV
        self.drvAxis4.motor.config.torque_constant = 8.27 * self.drvAxis4.motor.current_control.Iq_measured/ self.KV
        self.drvAxis5.motor.config.torque_constant = 8.27 * self.drvAxis5.motor.current_control.Iq_measured/ self.KV

        
        self.drvAxis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.drvAxis2.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.drvAxis3.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.drvAxis4.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.drvAxis5.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        self.drvAxis1.encoder.config.mode = ENCODER_MODE_HALL
        self.drvAxis2.encoder.config.mode = ENCODER_MODE_HALL
        self.drvAxis3.encoder.config.mode = ENCODER_MODE_HALL
        self.drvAxis4.encoder.config.mode = ENCODER_MODE_HALL
        self.drvAxis5.config.enable_sensorless_mode = True

        self.drvAxis1.encoder.config.cpr = 8192
        self.drvAxis2.encoder.config.cpr = 8192
        self.drvAxis3.encoder.config.cpr = 8192
        self.drvAxis4.encoder.config.cpr = 8192

        self.drvAxis1.encoder.config.calib_scan_distance = 9000
        self.drvAxis2.encoder.config.calib_scan_distance = 9000
        self.drvAxis3.encoder.config.calib_scan_distance = 9000
        self.drvAxis4.encoder.config.calib_scan_distance = 9000

        print("Saving manual configuration and rebooting...")

        self.drive1.save_configuration()
        self.drive2.save_configuration()
        self.drive3.save_configuration()

        print("Manual configuration saved")
        try:
            self.drive1.reboot()
            self.drive2.reboot()
            self.drive3.reboot()

        except ChannelBrokenException:
            pass

        self.findDrives()
        input("Make sure the motor is free to move, then press enter...")
        print("Calibrating Odrive for all the motors (you should hear a "
        "beep)...")

        self.drvAxis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        self.drvAxis2.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        self.drvAxis3.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        self.drvAxis4.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        self.drvAxis5.requested_state = AXIS_STATE_MOTOR_CALIBRATION

        time.sleep(10)

        if self.drvAxis2.motor.error != 0:
            print("Error: Odrive reported an error of {} while in the state " 
            "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
            "debug:\n{}".format(self.drvAxis2.motor.error, 
                                self.drvAxis2.motor))
            
            sys.exit(1)

        if self.drvAxis1.motor.error != 0:
            print("Error: Odrive reported an error of {} while in the state " 
            "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
            "debug:\n{}".format(self.drvAxis1.motor.error, 
                                self.drvAxis1.motor))
            
            sys.exit(1)

        if self.drvAxis3.motor.error != 0:
            print("Error: Odrive reported an error of {} while in the state " 
            "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
            "debug:\n{}".format(self.drvAxis3.motor.error, 
                                self.drvAxis3.motor))
            
            sys.exit(1)

        if self.drvAxis4.motor.error != 0:
            print("Error: Odrive reported an error of {} while in the state " 
            "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
            "debug:\n{}".format(self.drvAxis4.motor.error, 
                                self.drvAxis4.motor))
            
            sys.exit(1)
        
        if self.drvAxis5.motor.error != 0:
            print("Error: Odrive reported an error of {} while in the state " 
            "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
            "debug:\n{}".format(self.drvAxis5.motor.error, 
                                self.drvAxis5.motor))
            
            sys.exit(1)
        
        if self.drvAxis1.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE or \
        self.drvAxis1.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE:
            print("Error: After odrive motor calibration, the phase inductance "
            "is at {}, which is outside of the expected range. Either widen the "
            "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
            "is between {} and {} respectively) or debug/fix your setup. Printing "
            "out Odrive motor data for debug:\n{}".format(self.drvAxis1.motor.config.phase_inductance, 
                                                          self.MIN_PHASE_INDUCTANCE,
                                                          self.MAX_PHASE_INDUCTANCE, 
                                                          self.drvAxis1.motor))
            
            sys.exit(1)
        
        if self.drvAxis2.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE or \
        self.drvAxis2.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE:
            print("Error: After odrive motor calibration, the phase inductance "
            "is at {}, which is outside of the expected range. Either widen the "
            "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
            "is between {} and {} respectively) or debug/fix your setup. Printing "
            "out Odrive motor data for debug:\n{}".format(self.drvAxis2.motor.config.phase_inductance, 
                                                          self.MIN_PHASE_INDUCTANCE,
                                                          self.MAX_PHASE_INDUCTANCE, 
                                                          self.drvAxis2.motor))
            
            sys.exit(1)

        if self.drvAxis3.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE or \
        self.drvAxis3.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE:
            print("Error: After odrive motor calibration, the phase inductance "
            "is at {}, which is outside of the expected range. Either widen the "
            "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
            "is between {} and {} respectively) or debug/fix your setup. Printing "
            "out Odrive motor data for debug:\n{}".format(self.drvAxis3.motor.config.phase_inductance, 
                                                          self.MIN_PHASE_INDUCTANCE,
                                                          self.MAX_PHASE_INDUCTANCE, 
                                                          self.drvAxis3.motor))
            
            sys.exit(1)

        if self.drvAxis4.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE or \
        self.drvAxis4.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE:
            print("Error: After odrive motor calibration, the phase inductance "
            "is at {}, which is outside of the expected range. Either widen the "
            "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
            "is between {} and {} respectively) or debug/fix your setup. Printing "
            "out Odrive motor data for debug:\n{}".format(self.drvAxis4.motor.config.phase_inductance, 
                                                          self.MIN_PHASE_INDUCTANCE,
                                                          self.MAX_PHASE_INDUCTANCE, 
                                                          self.drvAxis4.motor))
            
            sys.exit(1)
        
        if self.drvAxis5.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE or \
        self.drvAxis5.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE:
            print("Error: After odrive motor calibration, the phase inductance "
            "is at {}, which is outside of the expected range. Either widen the "
            "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
            "is between {} and {} respectively) or debug/fix your setup. Printing "
            "out Odrive motor data for debug:\n{}".format(self.drvAxis5.motor.config.phase_inductance, 
                                                          self.MIN_PHASE_INDUCTANCE,
                                                          self.MAX_PHASE_INDUCTANCE, 
                                                          self.drvAxis5.motor))
            
            sys.exit(1)
        
        if self.drvAxis1.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE or \
        self.drvAxis1.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE:
            print("Error: After odrive motor calibration, the phase resistance "
            "is at {}, which is outside of the expected range. Either raise the "
            "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
            "debug/fix your setup. Printing out Odrive motor data for " 
            "debug:\n{}".format(self.drvAxis1.motor.config.phase_resistance, 
                                self.MIN_PHASE_RESISTANCE,
                                self.MAX_PHASE_RESISTANCE, 
                                self.drvAxis1.motor))
            
            sys.exit(1)

        if self.drvAxis2.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE or \
        self.drvAxis2.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE:
            print("Error: After odrive motor calibration, the phase resistance "
            "is at {}, which is outside of the expected range. Either raise the "
            "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
            "debug/fix your setup. Printing out Odrive motor data for " 
            "debug:\n{}".format(self.drvAxis2.motor.config.phase_resistance, 
                                self.MIN_PHASE_RESISTANCE,
                                self.MAX_PHASE_RESISTANCE, 
                                self.drvAxis2.motor))
            
            sys.exit(1)

        if self.drvAxis3.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE or \
        self.drvAxis3.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE:
            print("Error: After odrive motor calibration, the phase resistance "
            "is at {}, which is outside of the expected range. Either raise the "
            "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
            "debug/fix your setup. Printing out Odrive motor data for " 
            "debug:\n{}".format(self.drvAxis3.motor.config.phase_resistance, 
                                self.MIN_PHASE_RESISTANCE,
                                self.MAX_PHASE_RESISTANCE, 
                                self.drvAxis3.motor))
            
            sys.exit(1)

        if self.drvAxis4.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE or \
        self.drvAxis4.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE:
            print("Error: After odrive motor calibration, the phase resistance "
            "is at {}, which is outside of the expected range. Either raise the "
            "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
            "debug/fix your setup. Printing out Odrive motor data for " 
            "debug:\n{}".format(self.drvAxis4.motor.config.phase_resistance, 
                                self.MIN_PHASE_RESISTANCE,
                                self.MAX_PHASE_RESISTANCE, 
                                self.drvAxis4.motor))
            
            sys.exit(1)

        if self.drvAxis5.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE or \
        self.drvAxis5.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE:
            print("Error: After odrive motor calibration, the phase resistance "
            "is at {}, which is outside of the expected range. Either raise the "
            "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
            "debug/fix your setup. Printing out Odrive motor data for " 
            "debug:\n{}".format(self.drvAxis5.motor.config.phase_resistance, 
                                self.MIN_PHASE_RESISTANCE,
                                self.MAX_PHASE_RESISTANCE, 
                                self.drvAxis5.motor))
            
            sys.exit(1)
        
        self.drvAxis1.motor.config.pre_calibrated = True
        self.drvAxis2.motor.config.pre_calibrated = True
        self.drvAxis3.motor.config.pre_calibrated = True
        self.drvAxis4.motor.config.pre_calibrated = True
        self.drvAxis5.motor.config.pre_calibrated = True

        print("Calibrating Odrive for encoder...")

        self.drvAxis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        self.drvAxis2.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        self.drvAxis3.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        self.drvAxis4.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        self.drvAxis5.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

        time.sleep(10)

        if self.drvAxis1.encoder.error != 0:
            print("Error: Odrive reported an error of {} while in the state "
            "AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
            "data for debug:\n{}".format(self.drvAxis1.encoder.error, 
                                         self.drvAxis1.encoder))
            
            sys.exit(1)
        
        if self.drvAxis2.encoder.error != 0:
            print("Error: Odrive reported an error of {} while in the state "
            "AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
            "data for debug:\n{}".format(self.drvAxis2.encoder.error, 
                                         self.drvAxis2.encoder))
            
            sys.exit(1)

        if self.drvAxis3.encoder.error != 0:
            print("Error: Odrive reported an error of {} while in the state "
            "AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
            "data for debug:\n{}".format(self.drvAxis3.encoder.error, 
                                         self.drvAxis3.encoder))
            
            sys.exit(1)

        if self.drvAxis4.encoder.error != 0:
            print("Error: Odrive reported an error of {} while in the state "
            "AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
            "data for debug:\n{}".format(self.drvAxis4.encoder.error, 
                                         self.drvAxis4.encoder))
            
            sys.exit(1)
        
        if self.drvAxis5.encoder.error != 0:
            print("Error: Odrive reported an error of {} while in the state "
            "AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
            "data for debug:\n{}".format(self.drvAxis5.encoder.error, 
                                         self.drvAxis5.encoder))
            
            sys.exit(1)
        
        if not ((self.drvAxis1.encoder.config.offset_float > 0.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.drvAxis1.encoder.config.offset_float < 0.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE) or \
        (self.drvAxis1.encoder.config.offset_float > 1.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.drvAxis1.encoder.config.offset_float < 1.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE)):
            print("Error: After odrive encoder calibration, the 'offset_float' "
            "is at {}, which is outside of the expected range. 'offset_float' "
            "should be close to 0.5 or 1.5 with a tolerance of {}. Either "
            "increase the tolerance or debug/fix your setup. Printing out "
            "Odrive encoder data for debug:\n{}".format(self.drvAxis1.encoder.config.offset_float, 
                                                        self.ENCODER_OFFSET_FLOAT_TOLERANCE, 
                                                        self.drvAxis1.encoder))
                       
            sys.exit(1)

        if not ((self.drvAxis2.encoder.config.offset_float > 0.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.drvAxis2.encoder.config.offset_float < 0.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE) or \
        (self.drvAxis2.encoder.config.offset_float > 1.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.drvAxis2.encoder.config.offset_float < 1.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE)):
            print("Error: After odrive encoder calibration, the 'offset_float' "
            "is at {}, which is outside of the expected range. 'offset_float' "
            "should be close to 0.5 or 1.5 with a tolerance of {}. Either "
            "increase the tolerance or debug/fix your setup. Printing out "
            "Odrive encoder data for debug:\n{}".format(self.drvAxis2.encoder.config.offset_float, 
                                                        self.ENCODER_OFFSET_FLOAT_TOLERANCE, 
                                                        self.drvAxis2.encoder))
                       
            sys.exit(1)
        
        if not ((self.drvAxis3.encoder.config.offset_float > 0.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.drvAxis3.encoder.config.offset_float < 0.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE) or \
        (self.drvAxis3.encoder.config.offset_float > 1.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.drvAxis3.encoder.config.offset_float < 1.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE)):
            print("Error: After odrive encoder calibration, the 'offset_float' "
            "is at {}, which is outside of the expected range. 'offset_float' "
            "should be close to 0.5 or 1.5 with a tolerance of {}. Either "
            "increase the tolerance or debug/fix your setup. Printing out "
            "Odrive encoder data for debug:\n{}".format(self.drvAxis3.encoder.config.offset_float, 
                                                        self.ENCODER_OFFSET_FLOAT_TOLERANCE, 
                                                        self.drvAxis3.encoder))
                       
            sys.exit(1)

        if not ((self.drvAxis4.encoder.config.offset_float > 0.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.drvAxis4.encoder.config.offset_float < 0.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE) or \
        (self.drvAxis4.encoder.config.offset_float > 1.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.drvAxis4.encoder.config.offset_float < 1.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE)):
            print("Error: After odrive encoder calibration, the 'offset_float' "
            "is at {}, which is outside of the expected range. 'offset_float' "
            "should be close to 0.5 or 1.5 with a tolerance of {}. Either "
            "increase the tolerance or debug/fix your setup. Printing out "
            "Odrive encoder data for debug:\n{}".format(self.drvAxis4.encoder.config.offset_float, 
                                                        self.ENCODER_OFFSET_FLOAT_TOLERANCE, 
                                                        self.drvAxis4.encoder))
                       
            sys.exit(1)
        
        if not ((self.drvAxis5.encoder.config.offset_float > 0.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.drvAxis5.encoder.config.offset_float < 0.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE) or \
        (self.drvAxis5.encoder.config.offset_float > 1.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        self.drvAxis5.encoder.config.offset_float < 1.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE)):
            print("Error: After odrive encoder calibration, the 'offset_float' "
            "is at {}, which is outside of the expected range. 'offset_float' "
            "should be close to 0.5 or 1.5 with a tolerance of {}. Either "
            "increase the tolerance or debug/fix your setup. Printing out "
            "Odrive encoder data for debug:\n{}".format(self.drvAxis5.encoder.config.offset_float, 
                                                        self.ENCODER_OFFSET_FLOAT_TOLERANCE, 
                                                        self.drvAxis5.encoder))
                       
            sys.exit(1)

        self.drvAxis1.encoder.config.pre_calibrated = True
        self.drvAxis2.encoder.config.pre_calibrated = True
        self.drvAxis3.encoder.config.pre_calibrated = True
        self.drvAxis4.encoder.config.pre_calibrated = True
        self.drvAxis5.encoder.config.pre_calibrated = True

        print("Saving calibration configuration and rebooting...")
        self.drive1.save_configuration()
        self.drive2.save_configuration()
        self.drive3.save_configuration()

        print("Calibration configuration saved.")

        try:
            self.drive1.reboot()
            self.drive2.reboot()
            self.drive3.reboot()

        except ChannelBrokenException:
            pass

        self.findDrives()
        print("CONFIGURATION FINISHED. LET'S GO MOFO")

    def mode_idle(self):

        self.drvAxis1.requested_state = AXIS_STATE_IDLE
        self.drvAxis2.requested_state = AXIS_STATE_IDLE
        self.drvAxis3.requested_state = AXIS_STATE_IDLE
        self.drvAxis4.requested_state = AXIS_STATE_IDLE
        self.drvAxis5.requested_state = AXIS_STATE_IDLE
    
    def mode_close_loop_control(self):

        self.drvAxis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.drvAxis2.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.drvAxis3.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.drvAxis4.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.drvAxis5.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        
    def move_input_pos(self, angle):
        
        self.drvAxis1.controller.input_pos = angle/360.0
        self.drvAxis2.controller.input_pos = angle/360.0
        self.drvAxis3.controller.input_pos = angle/360.0
        self.drvAxis4.controller.input_pos = angle/360.0
        self.drvAxis5.controller.input_pos = angle/360.0

    def forward(self):

        self.drvAxis1.controller.input_vel = 10
        self.drvAxis2.controller.input_vel = 10
        self.drvAxis3.controller.input_vel = -10
        self.drvAxis4.controller.input_vel = -10
    
    def backward(self):

        self.drvAxis1.controller.input_vel = -10
        self.drvAxis2.controller.input_vel = -10
        self.drvAxis3.controller.input_vel = 10
        self.drvAxis4.controller.input_vel = 10
    
    def right(self):

        self.drvAxis1.controller.input_vel = 10
        self.drvAxis2.controller.input_vel = 10
        self.drvAxis3.controller.input_vel = 10
        self.drvAxis4.controller.input_vel = 10

    def left(self):

        self.drvAxis1.controller.input_vel = -10
        self.drvAxis2.controller.input_vel = -10
        self.drvAxis3.controller.input_vel = -10
        self.drvAxis4.controller.input_vel = -10
    
    def intake(self):

        self.drvAxis5.controller.input_vel = 40
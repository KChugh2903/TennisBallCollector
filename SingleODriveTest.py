import sys
import time
import odrive
from odrive.enums import *

class GearBox:
    KV = 150

    MIN_PHASE_INDUCTANCE = 0
    MAX_PHASE_INDUCTANCE = 0.001

    MIN_PHASE_RESISTANCE = 0
    MAX_PHASE_RESISTANCE = 0.5

    ENCODER_OFFSET_FLOAT_TOLERANCE = 0.05

    def __init__(self):
        print("Trying to find drive")
        self.drive = odrive.find_any()
        self.drvAxis1 = self.drive.axis0
        self.drvAxis2 = self.drive.axis1
        
        self.drvAxis1.controller.config.vel_limit = 22000.0
        self.drvAxis1.controller.config.current_lim = 11.0

        self.drvAxis2.controller.config.vel_limit = 22000
        self.drvAxis1.controller.config.current_lim = 11.0
    
    def configure(self):
        print("Erasing pre-existing configuration")
        self.drive.erase_configuration()

        self.drvAxis1.motor.config.pole_pairs = 7
        self.drvAxis2.motor.config.pole_pairs = 7

        self.drvAxis1.motor.config.resistance_calib_max_voltage = 4
        self.drvAxis1.motor.config.requested_current_range      = 25
        self.drvAxis1.motor.config.current_control_bandwidth    = 100

        self.drvAxis2.motor.config.resistance_calib_max_voltage = 4
        self.drvAxis2.motor.config.requested_current_range      = 25
        self.drvAxis2.motor.config.current_control_bandwidth    = 100

        self.drvAxis1.motor.config.torque_constant = 8.27 * self.drvAxis1.motor.current_control.Iq_measured/ self.KV
        self.drvAxis2.motor.config.torque_constant = 8.27 * self.drvAxis2.motor.current_control.Iq_measured/ self.KV

        self.drvAxis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.drvAxis2.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        self.drvAxis1.encoder.config.mode = ENCODER_MODE_HALL
        self.drvAxis2.encoder.config.mode = ENCODER_MODE_HALL

        self.drvAxis1.encoder.config.cpr = 8192
        self.drvAxis2.encoder.config.cpr = 8192

        self.drvAxis1.encoder.config.calib_scan_distance = 9000
        self.drvAxis2.encoder.config.calib_scan_distance = 9000

        print("Saving manual configuration and rebooting...")

        self.drive.save_configuration()

        print("Manual configuration saved")

        self.drive.reboot()

        self.findODrive()
        input("Make sure the motor is free to move. Now press enter...")
        print("Calibrating ODrive for motors")

        self.drvAxis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        self.drvAxis2.requested_state = AXIS_STATE_MOTOR_CALIBRATION

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
        
    
        self.drvAxis1.motor.config.pre_calibrated = True
        self.drvAxis2.motor.config.pre_calibrated = True

        print("Calibrating Odrive for encoder...")

        self.drvAxis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        self.drvAxis2.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

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
        
        self.drvAxis1.encoder.config.pre_calibrated = True
        self.drvAxis2.encoder.config.pre_calibrated = True
        print("Saving calibration configuration and rebooting...")
        self.drive.save_configuration()
        print("Calibration configuration saved. ")
        self.drive.reboot()

        self.findODrive()
        print("CONFIGURATION FINISHED!!")
    def mode_idle(self):

        self.drvAxis1.requested_state = AXIS_STATE_IDLE
        self.drvAxis2.requested_state = AXIS_STATE_IDLE
    
    def mode_close_loop_control(self):

        self.drvAxis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.drvAxis2.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    def move_input_pos(self, angle):
        
        self.drvAxis1.controller.input_pos = angle/360.0
        self.drvAxis2.controller.input_pos = angle/360.0

if __name__ == "__main__":
    gearbox = GearBox()
    print("Motor test time")
    print("Closed looped control rn")
    gearbox.mode_close_loop_control()
    for angle in range(0, 390, 30):
        print("Setting motor to {} degrees. ".format(angle))
        gearbox.move_input_pos(angle)
        time.sleep(5)
    print("Placing motor in idle. Free time. ")
    gearbox.mode_idle()   
                                         
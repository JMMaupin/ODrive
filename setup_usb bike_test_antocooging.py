

import odrive
from odrive.enums import *
import odrive.enums as oenums
import time
import math

odrv = odrive.find_any()


odrv.config.dc_bus_overvoltage_trip_level = 35
odrv.config.dc_bus_undervoltage_trip_level = 24
odrv.config.dc_max_positive_current = 10
odrv.config.dc_max_negative_current = -10
odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrv.axis0.config.motor.pole_pairs = 10
odrv.axis0.config.motor.torque_constant = 1.1814285714285713
odrv.axis0.config.motor.current_soft_max = 10
odrv.axis0.config.motor.current_hard_max = 23
odrv.axis0.config.motor.calibration_current = 5
odrv.axis0.config.motor.resistance_calib_max_voltage = 10
odrv.axis0.config.calibration_lockin.current = 5
odrv.axis0.motor.motor_thermistor.config.enabled = False
odrv.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv.axis0.controller.config.input_mode = InputMode.POS_FILTER
odrv.axis0.controller.config.vel_limit = 6
odrv.axis0.controller.config.vel_limit_tolerance = 1.6666666666666667
odrv.axis0.config.torque_soft_min = -10
odrv.axis0.config.torque_soft_max = 10
odrv.axis0.controller.config.input_filter_bandwidth = 3
odrv.can.config.protocol = Protocol.NONE
odrv.axis0.config.enable_watchdog = False
odrv.axis0.config.encoder_bandwidth = 100
odrv.hall_encoder0.config.enabled = True
odrv.axis0.config.load_encoder = EncoderId.HALL_ENCODER0
odrv.axis0.config.commutation_encoder = EncoderId.HALL_ENCODER0
odrv.config.enable_uart_a = False


odrv.axis0.controller.config.spinout_mechanical_power_threshold = -100
odrv.axis0.controller.config.spinout_electrical_power_threshold = 100


odrv.axis0.controller.config.vel_gain = 0.5
odrv.axis0.config.startup_encoder_index_search = True

odrv.axis0.config.anticogging.calib_vel_threshold = 5  # Seuil de vitesse
odrv.axis0.config.anticogging.calib_pos_threshold = 5  # Seuil de position
odrv.axis0.config.anticogging.max_torque = 0.5         # Couple max autorisé

odrv.axis0.requested_state = AxisState.ANTICOGGING_CALIBRATION


None
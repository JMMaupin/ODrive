

import odrive
from odrive.enums import *
import odrive.enums as oenums
import time
import math

odrv0 = odrive.find_any()


odrv0.config.dc_bus_overvoltage_trip_level = 17
odrv0.config.dc_bus_undervoltage_trip_level = 10.5
odrv0.config.dc_max_positive_current = math.inf
odrv0.config.dc_max_negative_current = -math.inf
odrv0.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrv0.axis0.config.motor.pole_pairs = 7
odrv0.axis0.config.motor.torque_constant = 0.05513333333333333
odrv0.axis0.config.motor.current_soft_max = 70
odrv0.axis0.config.motor.current_hard_max = 90
odrv0.axis0.config.motor.calibration_current = 10
odrv0.axis0.config.motor.resistance_calib_max_voltage = 2
odrv0.axis0.config.calibration_lockin.current = 10
odrv0.axis0.motor.motor_thermistor.config.enabled = False
odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv0.axis0.controller.config.input_mode = InputMode.VEL_RAMP
odrv0.axis0.controller.config.vel_limit = 10
odrv0.axis0.controller.config.vel_limit_tolerance = 1.2
odrv0.axis0.config.torque_soft_min = -1
odrv0.axis0.config.torque_soft_max = 1
odrv0.axis0.trap_traj.config.accel_limit = 10
odrv0.axis0.controller.config.vel_ramp_rate = 10
odrv0.can.config.protocol = Protocol.SIMPLE
odrv0.can.config.baud_rate = 1000000
odrv0.axis0.config.can.node_id = 0
odrv0.axis0.config.can.heartbeat_msg_rate_ms = 100
odrv0.axis0.config.can.encoder_msg_rate_ms = 10
odrv0.axis0.config.can.iq_msg_rate_ms = 10
odrv0.axis0.config.can.torques_msg_rate_ms = 10
odrv0.axis0.config.can.error_msg_rate_ms = 10
odrv0.axis0.config.can.temperature_msg_rate_ms = 10
odrv0.axis0.config.can.bus_voltage_msg_rate_ms = 10
odrv0.axis0.config.enable_watchdog = False
odrv0.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
odrv0.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0
odrv0.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_EVENT_DRIVEN
odrv0.config.enable_uart_a = False



# Définition des positions (en tours de moteur)
pos0 = 0.0
pos1 = 100.0

# Paramètres de mouvement (en tours/s et tours/s²)
vitesse = 30.0      # vitesse en tours/seconde
acc_dec = 20.0      # accélération/décélération en tours/s²

# Configuration de l'axe 0 en mode position avec trajectoire planifiée (Trap Traj)
odrv0.axis0.controller.config.input_mode = oenums.INPUT_MODE_TRAP_TRAJ
odrv0.axis0.controller.config.control_mode = oenums.CONTROL_MODE_POSITION_CONTROL

# Les limites de mouvement sont définies à la fois pour le contrôleur et la trajectoire
odrv0.axis0.controller.config.vel_limit = vitesse
odrv0.axis0.trap_traj.config.vel_limit = vitesse
odrv0.axis0.trap_traj.config.accel_limit = acc_dec
odrv0.axis0.trap_traj.config.decel_limit = acc_dec

# Tolérance de position (en tours) pour vérifier l'arrivée
tolerance = 0.01

odrv0.axis0.requested_state = oenums.AXIS_STATE_CLOSED_LOOP_CONTROL

while True:
    # Déplacement vers pos1
    print(f"Déplacement vers pos1 = {pos1} tours")
    odrv0.axis0.controller.input_pos = pos1
    # Attente active jusqu'à atteindre pos1
    while abs(odrv0.axis0.pos_estimate - pos1) > tolerance:
        time.sleep(0.01)
    print("Position pos1 atteinte.")
    time.sleep(0.1)  # Pause de 1 seconde

    # Déplacement vers pos0
    print(f"Déplacement vers pos0 = {pos0} tours")
    odrv0.axis0.controller.input_pos = pos0
    # Attente active jusqu'à atteindre pos0
    while abs(odrv0.axis0.pos_estimate - pos0) > tolerance:
        time.sleep(0.01)
    print("Position pos0 atteinte.")
    time.sleep(0.1)  # Pause de 1 seconde
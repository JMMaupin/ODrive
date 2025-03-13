

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



# Définition des positions (en tours de moteur)
pos0 = 0.0
pos1 = 60.0

# Paramètres de mouvement (en tours/s et tours/s²)
vitesse = 12.0      # vitesse en tours/seconde
acc_dec = 10.0      # accélération/décélération en tours/s²

# Configuration de l'axe 0 en mode position avec trajectoire planifiée (Trap Traj)
odrv.axis0.controller.config.input_mode = oenums.INPUT_MODE_TRAP_TRAJ
odrv.axis0.controller.config.control_mode = oenums.CONTROL_MODE_POSITION_CONTROL

# Les limites de mouvement sont définies à la fois pour le contrôleur et la trajectoire
odrv.axis0.controller.config.vel_limit = vitesse
odrv.axis0.trap_traj.config.vel_limit = vitesse
odrv.axis0.trap_traj.config.accel_limit = acc_dec
odrv.axis0.trap_traj.config.decel_limit = acc_dec

# Tolérance de position (en tours) pour vérifier l'arrivée
tolerance = 0.1

odrv.axis0.requested_state = oenums.AXIS_STATE_CLOSED_LOOP_CONTROL

while True:
    # Déplacement vers pos1
    print(f"Déplacement vers pos1 = {pos1} tours")
    odrv.axis0.controller.input_pos = pos1
    # Attente active jusqu'à atteindre pos1
    while abs(odrv.axis0.pos_estimate - pos1) > tolerance:
        time.sleep(0.01)
    print("Position pos1 atteinte.")
    time.sleep(0.1)  # Pause de 1 seconde

    # Déplacement vers pos0
    print(f"Déplacement vers pos0 = {pos0} tours")
    odrv.axis0.controller.input_pos = pos0
    # Attente active jusqu'à atteindre pos0
    while abs(odrv.axis0.pos_estimate - pos0) > tolerance:
        time.sleep(0.01)
    print("Position pos0 atteinte.")
    time.sleep(0.1)  # Pause de 1 seconde
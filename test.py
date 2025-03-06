import odrive
from odrive.enums import *
import can
import usb
import time
import struct

# Fonction pour vider le bus CAN
def flush_bus(bus):
    while True:
        msg = bus.recv(timeout=0.01)
        if msg is None:
            break

# Configuration de l'interface CAN (ici "can0" via SocketCAN)
dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F)
bus = can.Bus(interface="gs_usb", channel=dev.product, index=0, bitrate=1000000)
flush_bus(bus)

# Le node_id doit correspondre à celui configuré dans l'ODrive
node_id = 0

# Envoyer une commande CAN pour passer en "Closed Loop Control"
# La commande Set_Axis_State (cmd_id 0x07) avec la valeur 8 active le mode closed loop.
axis_state_cmd_id = 0x07
axis_state_arb_id = (node_id << 5) | axis_state_cmd_id
state_value = 8  # AXIS_STATE_CLOSED_LOOP_CONTROL
state_data  = struct.pack('<I', state_value)
state_msg   = can.Message(arbitration_id=axis_state_arb_id, data=state_data, is_extended_id=False)
bus.send(state_msg)
print("Axe mis en Closed Loop Control via CAN.")
time.sleep(0.2)

# Définitions des positions cibles (en tours) et de la vitesse (en tours/seconde)
pos0 = 0.0
pos1 = 100.0
tolerance = 0.1
vitesse = 10.0  # Vitesse en tours/seconde

# Fonction pour envoyer la commande de position via CAN (Set_Input_Pos, cmd_id 0x0C)
def send_set_input_pos(target_pos, velocity_limit):
    set_input_pos_cmd_id = 0x0C
    arb_id = (node_id << 5) | set_input_pos_cmd_id
    # Envoie la position cible et la limite de vitesse sous forme de floats (8 octets)
    data = struct.pack('<ff', target_pos, velocity_limit)
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)
    print(f"Commande envoyée: position cible = {target_pos} tours, vitesse = {velocity_limit} tours/s")

# Fonction pour lire l'estimation d'encodeur (position et vitesse)
def get_encoder_estimates(timeout=0.1):
    # L'ODrive envoie les messages avec cmd_id 0x09 pour les estimations
    expected_arb_id = (node_id << 5) | 0x09
    end_time = time.time() + timeout
    while time.time() < end_time:
        msg = bus.recv(timeout=0.01)
        if msg and msg.arbitration_id == expected_arb_id:
            if len(msg.data) >= 8:
                pos, vel = struct.unpack('<ff', msg.data)
                return pos, vel
    return None, None

# Boucle principale : alterner entre pos1 et pos0
while True:
    # Déplacement vers pos1
    target = pos1
    print(f"Déplacement vers pos1 = {target} tours")
    send_set_input_pos(target, vitesse)
    reached = False
    while not reached:
        pos_est, vel_est = get_encoder_estimates(timeout=0.1)
        if pos_est is not None:
            print(f"Estimation de position : {pos_est:.3f} tours, vitesse : {vel_est:.3f} tours/s")
            if abs(pos_est - target) <= tolerance:
                reached = True
        time.sleep(0.05)
    print("Position pos1 atteinte.")
    time.sleep(1)

    # Déplacement vers pos0
    target = pos0
    print(f"Déplacement vers pos0 = {target} tours")
    send_set_input_pos(target, vitesse)
    reached = False
    while not reached:
        pos_est, vel_est = get_encoder_estimates(timeout=0.1)
        if pos_est is not None:
            print(f"Estimation de position : {pos_est:.3f} tours, vitesse : {vel_est:.3f} tours/s")
            if abs(pos_est - target) <= tolerance:
                reached = True
        time.sleep(0.05)
    print("Position pos0 atteinte.")
    time.sleep(1)

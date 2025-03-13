import usb.core
import usb.util
import can
import asyncio
import math
import time
import struct
import numpy as np
from enum import Enum

# Définition des commandes CAN pour ODrive
class AxisState(Enum):
    UNDEFINED = 0
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_INDEX_SEARCH = 5
    ENCODER_OFFSET_CALIBRATION = 6
    CLOSED_LOOP_CONTROL = 8
    LOCKIN_SPIN = 9
    ENCODER_DIR_FIND = 10
    HOMING = 11
    ENCODER_HALL_POLARITY_CALIBRATION = 12
    ENCODER_HALL_PHASE_CALIBRATION = 13

class ControlMode(Enum):
    VOLTAGE_CONTROL = 0
    TORQUE_CONTROL = 1
    VELOCITY_CONTROL = 2
    POSITION_CONTROL = 3

class InputMode(Enum):
    INACTIVE = 0
    PASSTHROUGH = 1
    VEL_RAMP = 2
    POS_FILTER = 3
    MIX_CHANNELS = 4
    TRAP_TRAJ = 5
    TORQUE_RAMP = 6
    MIRROR = 7
    TUNING = 8

class CANCommand:
    HEARTBEAT = 0x001
    ESTOP = 0x002
    GET_MOTOR_ERROR = 0x003
    GET_ENCODER_ERROR = 0x004
    GET_SENSORLESS_ERROR = 0x005
    SET_AXIS_NODE_ID = 0x006
    SET_AXIS_STATE = 0x007
    GET_ENCODER_ESTIMATES = 0x009
    GET_ENCODER_COUNT = 0x00A
    SET_CONTROLLER_MODE = 0x00C
    SET_INPUT_POS = 0x00D
    SET_INPUT_VEL = 0x00E
    SET_INPUT_TORQUE = 0x00F
    SET_LIMITS = 0x011
    SET_TRAJ_VEL_LIMIT = 0x019
    SET_TRAJ_ACCEL_LIMIT = 0x01A
    SET_TRAJ_DECEL_LIMIT = 0x01B
    SET_POS_GAIN = 0x01C
    SET_VEL_GAINS = 0x01D
    CLEAR_ERRORS = 0x01E
    SET_INPUT_MODE = 0x012

# Classe pour contrôler l'ODrive via CAN
class ODriveCANControl:
    def __init__(self, can_bus, node_id=0):
        self.bus = can_bus
        self.node_id = node_id
        print(f"ODrive CAN Control initialisé avec Node ID {node_id}")
    
    def send_command(self, command_id, data=None, timeout=1.0):
        """Envoie une commande CAN à l'ODrive"""
        if data is None:
            data = []
        
        # Création du message CAN
        msg = can.Message(
            arbitration_id=(self.node_id << 5 | command_id),
            data=data,
            is_extended_id=False
        )
        
        try:
            self.bus.send(msg)
            print(f"Message envoyé: ID=0x{msg.arbitration_id:X}, Data={[hex(b) for b in msg.data]}")
            return True
        except Exception as e:
            print(f"Erreur lors de l'envoi du message: {e}")
            return False
    
    def receive_message(self, expected_id=None, timeout=1.0):
        """Reçoit un message CAN de l'ODrive avec un timeout"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.bus.recv(timeout=0.1)
            if msg is not None:
                # Si un ID est attendu, vérifier qu'il correspond
                if expected_id is None or (msg.arbitration_id & 0x1F) == expected_id:
                    return msg
        return None
    
    def set_axis_state(self, state):
        """Configure l'état de l'axe (ex: CLOSED_LOOP_CONTROL)"""
        print(f"Configuration de l'état de l'axe: {state.name}")
        return self.send_command(CANCommand.SET_AXIS_STATE, [state.value, 0, 0, 0, 0, 0, 0, 0])
    
    def set_controller_mode(self, control_mode, input_mode):
        """Configure le mode de contrôle et le mode d'entrée"""
        print(f"Configuration du mode de contrôle: {control_mode.name}, mode d'entrée: {input_mode.name}")
        return self.send_command(CANCommand.SET_CONTROLLER_MODE, 
                              [control_mode.value, input_mode.value, 0, 0, 0, 0, 0, 0])
    
    def set_input_pos(self, position, velocity_ff=0, torque_ff=0):
        """Définit la position cible avec feed-forward optionnels"""
        # Conversion en format IEEE 754 (float32)
        pos_bytes = list(struct.pack('<f', float(position)))
        vel_bytes = list(struct.pack('<f', float(velocity_ff)))
        torque_bytes = list(struct.pack('<f', float(torque_ff)))
        
        # Ne prend que les 2 premiers octets pour vel_ff et 2 premiers pour torque_ff
        data = pos_bytes + vel_bytes[:2] + torque_bytes[:2]
        
        print(f"Déplacement vers la position: {position} tours")
        return self.send_command(CANCommand.SET_INPUT_POS, data)
    
    def set_limits(self, velocity_limit, current_limit):
        """Définit les limites de vitesse et de courant"""
        vel_bytes = list(struct.pack('<f', float(velocity_limit)))
        current_bytes = list(struct.pack('<f', float(current_limit)))
        
        data = vel_bytes + current_bytes
        
        print(f"Configuration des limites: vitesse={velocity_limit} tours/s, courant={current_limit}A")
        return self.send_command(CANCommand.SET_LIMITS, data)
    
    def set_traj_vel_limit(self, vel_limit):
        """Définit la limite de vitesse pour les trajectoires"""
        vel_bytes = list(struct.pack('<f', float(vel_limit)))
        
        print(f"Configuration de la limite de vitesse de trajectoire: {vel_limit} tours/s")
        return self.send_command(CANCommand.SET_TRAJ_VEL_LIMIT, vel_bytes)
    
    def set_traj_accel_limit(self, accel_limit):
        """Définit la limite d'accélération pour les trajectoires"""
        accel_bytes = list(struct.pack('<f', float(accel_limit)))
        
        print(f"Configuration de la limite d'accélération de trajectoire: {accel_limit} tours/s²")
        return self.send_command(CANCommand.SET_TRAJ_ACCEL_LIMIT, accel_bytes)
    
    def set_traj_decel_limit(self, decel_limit):
        """Définit la limite de décélération pour les trajectoires"""
        decel_bytes = list(struct.pack('<f', float(decel_limit)))
        
        print(f"Configuration de la limite de décélération de trajectoire: {decel_limit} tours/s²")
        return self.send_command(CANCommand.SET_TRAJ_DECEL_LIMIT, decel_bytes)
    
    def set_pos_gain(self, pos_gain):
        """Définit le gain de position"""
        gain_bytes = list(struct.pack('<f', float(pos_gain)))
        
        print(f"Configuration du gain de position: {pos_gain}")
        return self.send_command(CANCommand.SET_POS_GAIN, gain_bytes)
    
    def set_vel_gains(self, vel_gain, vel_integrator_gain):
        """Définit les gains de vitesse"""
        vel_gain_bytes = list(struct.pack('<f', float(vel_gain)))
        vel_int_bytes = list(struct.pack('<f', float(vel_integrator_gain)))
        
        data = vel_gain_bytes + vel_int_bytes
        
        print(f"Configuration des gains de vitesse: gain={vel_gain}, intégrateur={vel_integrator_gain}")
        return self.send_command(CANCommand.SET_VEL_GAINS, data)
    
    def set_input_mode(self, input_mode):
        """Configure le mode d'entrée"""
        print(f"Configuration du mode d'entrée: {input_mode.name}")
        return self.send_command(CANCommand.SET_INPUT_MODE, [input_mode.value, 0, 0, 0, 0, 0, 0, 0])
    
    def clear_errors(self):
        """Efface les erreurs de l'ODrive"""
        print("Effacement des erreurs")
        return self.send_command(CANCommand.CLEAR_ERRORS, [0, 0, 0, 0, 0, 0, 0, 0])

# Fonction principale pour initialiser la communication et contrôler l'ODrive
async def main():
    # Configuration de l'adaptateur USB-CAN
    print("Recherche de l'adaptateur USB-CAN...")
    
    # Importation du backend libusb1
    import usb.backend.libusb1
    backend = usb.backend.libusb1.get_backend()
    
    # Recherche de l'adaptateur candleLight
    dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F, backend=backend)
    
    if dev is None:
        print("Adaptateur USB-CAN non trouvé!")
        return
    
    print(f"Adaptateur trouvé: {dev.product}")
    
    try:
        # Configuration du bus CAN
        print("Configuration de l'interface CAN...")
        bus = can.Bus(
            interface="gs_usb", 
            channel=dev.product,
            bus=dev.bus,
            address=dev.address,
            bitrate=250000
        )
        
        print("Interface CAN configurée avec succès!")
        
        # Création du contrôleur ODrive
        node_id = 0  # L'ID du nœud configuré sur votre ODrive Pro
        odrv = ODriveCANControl(bus, node_id)
        
        # Configuration initiale
        print("Configuration initiale de l'ODrive...")
        
        # Effacement des erreurs potentielles
        odrv.clear_errors()
        await asyncio.sleep(0.1)
        
        # Configuration des paramètres de contrôle
        odrv.set_controller_mode(ControlMode.POSITION_CONTROL, InputMode.POS_FILTER)
        await asyncio.sleep(0.1)
        
        # Configuration des gains
        odrv.set_pos_gain(20.0)  # Gain de position
        await asyncio.sleep(0.1)
        odrv.set_vel_gains(0.16, 0.32)  # Gain de vitesse et intégrateur
        await asyncio.sleep(0.1)
        
        # Configuration des limites
        odrv.set_limits(10.0, 10.0)  # Limite de vitesse et de courant
        await asyncio.sleep(0.1)
        
        # Passage en mode contrôle en boucle fermée
        print("Passage en mode contrôle en boucle fermée...")
        odrv.set_axis_state(AxisState.CLOSED_LOOP_CONTROL)
        await asyncio.sleep(1.0)  # Attendre que le mode soit activé
        
        print("Début des mouvements de démonstration...")
        
        # Exemple 1: Mouvement sinusoïdal
        print("Mouvement sinusoïdal pendant 10 secondes...")
        start_time = time.time()
        duration = 10.0  # secondes
        while time.time() - start_time < duration:
            t = time.time() - start_time
            position = 2.0 * math.sin(t * math.pi)  # Amplitude de 2 rotations
            odrv.set_input_pos(position)
            await asyncio.sleep(0.01)  # 10ms de délai
        
        # Exemple 2: Mouvements par étapes
        print("Mouvements par étapes...")
        positions = [2.0, -2.0, 1.0, -1.0, 0.5, -0.5, 0.0]
        for pos in positions:
            print(f"Déplacement vers la position {pos}...")
            odrv.set_input_pos(pos)
            await asyncio.sleep(2.0)  # Attente de 2 secondes à chaque position
        
        # Exemple 3: Profil trapézoïdal
        print("Configuration du profil trapézoïdal...")
        odrv.set_traj_vel_limit(5.0)  # Vitesse maximale
        await asyncio.sleep(0.1)
        odrv.set_traj_accel_limit(10.0)  # Accélération maximale
        await asyncio.sleep(0.1)
        odrv.set_traj_decel_limit(10.0)  # Décélération maximale
        await asyncio.sleep(0.1)
        
        # Passage en mode trajectoire trapézoïdale
        odrv.set_input_mode(InputMode.TRAP_TRAJ)
        await asyncio.sleep(0.5)
        
        # Mouvements avec profil trapézoïdal
        print("Mouvements avec profil trapézoïdal...")
        for pos in [5.0, -5.0, 2.5, -2.5, 0.0]:
            print(f"Déplacement trapézoïdal vers {pos}...")
            odrv.set_input_pos(pos)
            # Attente que le mouvement soit terminé (approximativement)
            await asyncio.sleep(3.0)
        
        # Retour à la position zéro
        print("Retour à la position initiale...")
        odrv.set_input_pos(0.0)
        await asyncio.sleep(2.0)
        
        print("Démonstration terminée!")
        
        # Fermeture propre du bus CAN
        bus.shutdown()
        print("Interface CAN fermée.")
        
    except Exception as e:
        print(f"Erreur lors de l'exécution: {e}")
        try:
            bus.shutdown()
        except:
            pass

if __name__ == "__main__":
    asyncio.run(main())

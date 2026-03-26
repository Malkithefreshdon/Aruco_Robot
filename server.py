#!/usr/bin/env python3
# File name   : robot_server.py
# Description : Serveur TCP de navigation — extrait de LineFollowerFin.py
#               Compatible Adeept (PCA9685 + servo direction + 2 moteurs DC)
# À lancer sur le Raspberry Pi

import time
import socket
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor, servo

# ─────────────────────────────────────────────────────────────────────────────
#  PIN & CHANNEL CONFIGURATION  (repris de LineFollowerFin.py)
# ─────────────────────────────────────────────────────────────────────────────

MOTOR_LEFT_IN1    = 15
MOTOR_LEFT_IN2    = 14
MOTOR_RIGHT_IN1   = 12
MOTOR_RIGHT_IN2   = 13
SERVO_STEERING_CH = 0

# ─────────────────────────────────────────────────────────────────────────────
#  TUNING PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────

DRIVE_SPEED      = 60    # Vitesse avant (0–100 %)
REVERSE_SPEED    = 50    # Vitesse arrière pour pivot sur place

SERVO_CENTER     = 90    # Tout droit (°)
SERVO_LEFT       = 60    # Virage gauche max (°)
SERVO_RIGHT      = 120   # Virage droite max (°)

# ─────────────────────────────────────────────────────────────────────────────
#  HARDWARE INITIALISATION  (identique à LineFollowerFin.py)
# ─────────────────────────────────────────────────────────────────────────────

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5f)
pca.frequency = 50

motor_left  = motor.DCMotor(pca.channels[MOTOR_LEFT_IN1],  pca.channels[MOTOR_LEFT_IN2])
motor_right = motor.DCMotor(pca.channels[MOTOR_RIGHT_IN1], pca.channels[MOTOR_RIGHT_IN2])
motor_left.decay_mode  = motor.SLOW_DECAY
motor_right.decay_mode = motor.SLOW_DECAY

steering = servo.Servo(
    pca.channels[SERVO_STEERING_CH],
    min_pulse=500,
    max_pulse=2400,
    actuation_range=180
)

# ─────────────────────────────────────────────────────────────────────────────
#  LOW-LEVEL HELPERS  (extraits de LineFollowerFin.py)
# ─────────────────────────────────────────────────────────────────────────────

def _pct_to_throttle(speed_pct: float) -> float:
    return max(0.0, min(1.0, speed_pct / 100.0))

def set_drive(speed_pct: float) -> None:
    """Avance les deux moteurs arrière à la vitesse donnée (0–100 %)."""
    throttle = -_pct_to_throttle(speed_pct)   # Négatif = forward physique
    motor_left.throttle  = throttle
    motor_right.throttle = throttle

def set_reverse(speed_pct: float) -> None:
    """Recule les deux moteurs."""
    throttle = _pct_to_throttle(speed_pct)
    motor_left.throttle  = throttle
    motor_right.throttle = throttle

def set_steering(angle: float) -> None:
    """Oriente les roues avant à l'angle servo donné (0–180 °)."""
    steering.angle = angle

def stop() -> None:
    """Arrêt complet + recentrage du servo."""
    motor_left.throttle  = 0
    motor_right.throttle = 0
    set_steering(SERVO_CENTER)

def destroy() -> None:
    stop()
    pca.deinit()

# ─────────────────────────────────────────────────────────────────────────────
#  COMMANDES DE NAVIGATION
#
#  Ce robot est de type VOITURE (servo avant + 2 moteurs arrière).
#  Pour tourner sur place (pivot), on utilise marche avant + marche arrière
#  sur les deux moteurs simultanément avec le servo braqué.
#
#  Commandes TCP disponibles :
#   FORWARD  <durée>   — avance tout droit
#   BACKWARD <durée>   — recule tout droit
#   LEFT     <durée>   — pivot gauche sur place
#   RIGHT    <durée>   — pivot droite sur place
#   STOP               — arrêt immédiat
# ─────────────────────────────────────────────────────────────────────────────

def cmd_forward(duration: float) -> None:
    set_steering(SERVO_CENTER)
    set_drive(DRIVE_SPEED)
    time.sleep(duration)
    stop()

def cmd_backward(duration: float) -> None:
    set_steering(SERVO_CENTER)
    set_reverse(REVERSE_SPEED)
    time.sleep(duration)
    stop()

def cmd_turn_left(duration: float) -> None:
    """
    Pivot gauche sur place :
    - Servo braqué à gauche max
    - Moteur gauche recule / Moteur droit avance
    """
    set_steering(SERVO_LEFT)
    motor_left.throttle  = _pct_to_throttle(REVERSE_SPEED)   # recule
    motor_right.throttle = -_pct_to_throttle(DRIVE_SPEED)    # avance
    time.sleep(duration)
    stop()

def cmd_turn_right(duration: float) -> None:
    """
    Pivot droit sur place :
    - Servo braqué à droite max
    - Moteur droit recule / Moteur gauche avance
    """
    set_steering(SERVO_RIGHT)
    motor_left.throttle  = -_pct_to_throttle(DRIVE_SPEED)    # avance
    motor_right.throttle = _pct_to_throttle(REVERSE_SPEED)   # recule
    time.sleep(duration)
    stop()

# ─────────────────────────────────────────────────────────────────────────────
#  SERVEUR TCP
# ─────────────────────────────────────────────────────────────────────────────

DISPATCH = {
    "FORWARD":  cmd_forward,
    "BACKWARD": cmd_backward,
    "LEFT":     cmd_turn_left,
    "RIGHT":    cmd_turn_right,
}

HOST = "0.0.0.0"
PORT = 9999

print("Robot server prêt. En attente de connexion sur le port 9999...")
set_steering(SERVO_CENTER)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()
    print(f"PC connecté : {addr}")

    try:
        with conn:
            while True:
                data = conn.recv(64).decode().strip()
                if not data:
                    break

                parts = data.split()
                cmd   = parts[0].upper()
                val   = float(parts[1]) if len(parts) > 1 else 0.0

                print(f"[CMD] {cmd} {val:.2f}s")

                if cmd in DISPATCH:
                    DISPATCH[cmd](val)
                elif cmd == "STOP":
                    stop()
                else:
                    print(f"[WARN] Commande inconnue : {cmd}")

                conn.sendall(b"OK\n")

    except (ConnectionResetError, BrokenPipeError):
        print("Connexion perdue.")
    finally:
        destroy()
        print("Hardware libéré.")
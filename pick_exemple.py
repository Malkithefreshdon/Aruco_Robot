#!/usr/bin/env python3
# File name   : pick_motion.py
# Description : Séquence "pick" pour le bras robot Adeept
#
# Channels:
#   0 → Roues (non utilisé ici)
#   1 → Rotation base du bras
#   2 → Premier joint du bras (épaule)
#   3 → Deuxième joint du bras (coude)
#   4 → Pince (gripper)

import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# ─── Init ────────────────────────────────────────────────────────────────────
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x5f)
pca.frequency = 50

# ─── Config ──────────────────────────────────────────────────────────────────
CHANNEL_BASE     = 1   # Rotation base
CHANNEL_SHOULDER = 2   # Premier joint (épaule)
CHANNEL_ELBOW    = 3   # Deuxième joint (coude)
CHANNEL_GRIPPER  = 4   # Pince

# Angles de repos (position neutre)
REST = {
    CHANNEL_BASE:     90,
    CHANNEL_SHOULDER: 90,
    CHANNEL_ELBOW:    90,
    CHANNEL_GRIPPER:  90,   # pince mi-ouverte
}

# Angles pour la séquence pick
GRIPPER_OPEN   = -120    # pince ouverte
GRIPPER_CLOSED = 120   # pince fermée (saisie objet)

PICK_BASE      = 90    # face à l'objet
PICK_SHOULDER  = 140    # descend vers l'objet
PICK_ELBOW     = 90   # plie le coude vers l'objet

LIFT_SHOULDER  = 90   # remonte avec l'objet
LIFT_ELBOW     = 110    # étend le bras vers le haut

# ─── Helpers ─────────────────────────────────────────────────────────────────
def set_angle(channel: int, angle: float):
    """Positionne un servo à l'angle donné (0–180)."""
    s = servo.Servo(
        pca.channels[channel],
        min_pulse=500,
        max_pulse=2400,
        actuation_range=180,
    )
    s.angle = max(0, min(180, angle))


def smooth_move(channel: int, start: float, end: float, steps: int = 40, delay: float = 0.015):
    """Mouvement fluide d'un servo de start° à end°."""
    step_size = (end - start) / steps
    for i in range(steps + 1):
        set_angle(channel, start + step_size * i)
        time.sleep(delay)


def go_to_rest():
    """Remet le bras en position de repos."""
    print("→ Position repos")
    smooth_move(CHANNEL_ELBOW,    REST[CHANNEL_ELBOW],    REST[CHANNEL_ELBOW],    steps=1)
    smooth_move(CHANNEL_SHOULDER, REST[CHANNEL_SHOULDER], REST[CHANNEL_SHOULDER], steps=1)
    smooth_move(CHANNEL_BASE,     REST[CHANNEL_BASE],     REST[CHANNEL_BASE],     steps=1)
    smooth_move(CHANNEL_GRIPPER,  REST[CHANNEL_GRIPPER],  REST[CHANNEL_GRIPPER],  steps=1)


# ─── Séquence Pick ───────────────────────────────────────────────────────────
def pick(target_base: int = PICK_BASE):
    """
    Séquence complète de pick :
      1. Ouvre la pince
      2. Oriente la base vers la cible
      3. Descend le bras (épaule + coude)
      4. Ferme la pince (saisie)
      5. Remonte le bras avec l'objet
    """
    print("─── Début séquence PICK ───")

    # 1. Ouvrir la pince
    print("  [1/5] Ouverture pince")
    smooth_move(CHANNEL_GRIPPER, REST[CHANNEL_GRIPPER], GRIPPER_OPEN)
    time.sleep(0.3)

    # 2. Orienter la base
    print(f"  [2/5] Rotation base → {target_base}°")
    smooth_move(CHANNEL_BASE, REST[CHANNEL_BASE], target_base)
    time.sleep(0.2)

    # 3. Descendre vers l'objet (épaule puis coude simultané approché)
    print("  [3/5] Descente vers l'objet")
    smooth_move(CHANNEL_SHOULDER, REST[CHANNEL_SHOULDER], PICK_SHOULDER, steps=50)
    smooth_move(CHANNEL_ELBOW,    REST[CHANNEL_ELBOW],    PICK_ELBOW,    steps=50)
    time.sleep(0.3)

    # 4. Saisir l'objet
    print("  [4/5] Fermeture pince (saisie)")
    smooth_move(CHANNEL_GRIPPER, GRIPPER_OPEN, GRIPPER_CLOSED, steps=30)
    time.sleep(0.4)

    # 5. Remonter avec l'objet
    print("  [5/5] Remontée avec l'objet")
    smooth_move(CHANNEL_ELBOW,    PICK_ELBOW,    LIFT_ELBOW,    steps=50)
    smooth_move(CHANNEL_SHOULDER, PICK_SHOULDER, LIFT_SHOULDER, steps=50)
    time.sleep(0.3)

    print("─── Pick terminé ✓ ───\n")


# ─── Main ─────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    try:
        # Partir d'une position connue
        go_to_rest()
        time.sleep(0.5)

        print("Lancement de la séquence pick (Ctrl+C pour arrêter)\n")

        # Effectuer 3 picks consécutifs à titre de démonstration
        for i in range(3):
            print(f"=== Pick #{i + 1} ===")
            pick(target_base=PICK_BASE)
            go_to_rest()
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nArrêt demandé — retour en position repos.")
        go_to_rest()

    finally:
        pca.deinit()
        print("PCA9685 libéré.")
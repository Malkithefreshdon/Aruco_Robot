#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
#  2_aruco_verify.py — Interface de vérification de la détection ArUco
#  Usage : python3 2_aruco_verify.py
#  Nécessite : camera_calibration.npz (généré par 1_calibration.py)
#              (optionnel mais recommandé pour la correction de distorsion)
# ─────────────────────────────────────────────────────────────────────────────
#  Contrôles :
#   [U]       — activer/désactiver la correction de distorsion (undistort)
#   [A]       — afficher/masquer les axes 3D sur chaque marqueur
#   [I]       — afficher/masquer les infos détaillées (ID, angle, distance)
#   [D]       — changer le dictionnaire ArUco (4x4, 5x5, 6x6)
#   [Q/ESC]   — quitter
# ─────────────────────────────────────────────────────────────────────────────

import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import os
import time
import math

# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────

CALIB_FILE       = "camera_calibration.npz"
MARKER_SIZE_MM   = 50.0    # taille physique de tes marqueurs imprimés (mm)

# IDs des marqueurs du projet
MARKER_ROLES = {
    0: ("BASE",    (200, 200, 200)),   # gris   — repère fixe
    1: ("ROBOT",   (80,  220, 100)),   # vert   — robot
    2: ("TARGET",  (60,  60,  220)),   # rouge  — destination
}

# Dictionnaires ArUco disponibles (touche D pour cycler)
DICTS = [
    ("DICT_4X4_50",   aruco.DICT_4X4_50),
    ("DICT_5X5_100",  aruco.DICT_5X5_100),
    ("DICT_6X6_250",  aruco.DICT_6X6_250),
    ("DICT_ARUCO_ORIGINAL", aruco.DICT_ARUCO_ORIGINAL),
]

# Couleurs UI (BGR)
C_WHITE   = (240, 240, 235)
C_DARK    = (12,  12,  18 )
C_ACCENT  = (0,   200, 160)    # cyan-vert
C_WARN    = (0,   180, 255)    # orange
C_OK      = (80,  220, 100)    # vert
C_BAD     = (60,  60,  220)    # rouge
C_PANEL   = (18,  18,  28 )

FONT     = cv2.FONT_HERSHEY_SIMPLEX
FONT_M   = cv2.FONT_HERSHEY_DUPLEX

# ─────────────────────────────────────────────────────────────────────────────
#  DÉTECTION CAMÉRA
# ─────────────────────────────────────────────────────────────────────────────

def find_camera() -> cv2.VideoCapture:
    print("Recherche de la caméra USB...")
    for idx in range(6):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                print(f"  ✓ Caméra index {idx}  ({w}×{h})")
                return cap
            cap.release()
    print("  ✗ Aucune caméra détectée.")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
#  CHARGEMENT DE LA CALIBRATION
# ─────────────────────────────────────────────────────────────────────────────

def load_calibration(path: str):
    if not os.path.exists(path):
        print(f"  ⚠ Fichier calibration non trouvé : {path}")
        print("    Lancer 1_calibration.py d'abord. Continuité sans correction.")
        return None, None
    data = np.load(path)
    mtx  = data["camera_matrix"]
    dist = data["dist_coeffs"]
    print(f"  ✓ Calibration chargée depuis {path}")
    return mtx, dist

# ─────────────────────────────────────────────────────────────────────────────
#  HELPERS GÉOMÉTRIE
# ─────────────────────────────────────────────────────────────────────────────

def marker_center(corners) -> tuple:
    c = corners[0]
    return (int(np.mean(c[:, 0])), int(np.mean(c[:, 1])))

def marker_angle_deg(corners) -> float:
    c = corners[0]
    dx = c[1][0] - c[0][0]
    dy = c[1][1] - c[0][1]
    return math.degrees(math.atan2(dy, dx))

def pixel_distance(p1, p2) -> float:
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def draw_arrow(img, center, angle_deg, length=40, color=(255,255,255), thickness=2):
    """Dessine une flèche indiquant l'orientation du marqueur."""
    rad = math.radians(angle_deg)
    end = (
        int(center[0] + length * math.cos(rad)),
        int(center[1] + length * math.sin(rad))
    )
    cv2.arrowedLine(img, center, end, color, thickness, tipLength=0.35)

# ─────────────────────────────────────────────────────────────────────────────
#  PANNEAU LATÉRAL
# ─────────────────────────────────────────────────────────────────────────────

PANEL_W = 280

def draw_panel(canvas: np.ndarray, markers: dict, fps: float,
               dict_name: str, undistort: bool, show_axes: bool,
               show_info: bool, calib_loaded: bool):
    h = canvas.shape[0]

    # Fond panneau
    cv2.rectangle(canvas, (canvas.shape[1] - PANEL_W, 0),
                  (canvas.shape[1], h), C_PANEL, -1)
    x0 = canvas.shape[1] - PANEL_W + 12
    y  = 30

    def txt(text, cy, color=C_WHITE, scale=0.48, bold=False):
        thick = 2 if bold else 1
        cv2.putText(canvas, text, (x0, cy), FONT, scale, color, thick, cv2.LINE_AA)

    def sep(cy):
        cv2.line(canvas, (x0 - 4, cy), (canvas.shape[1] - 8, cy), (40, 40, 55), 1)

    # ── Titre ──────────────────────────────────────────────────────────────
    txt("ARUCO VERIFY", y, C_ACCENT, 0.62, bold=True); y += 26
    txt(f"FPS : {fps:.1f}", y, C_WHITE, 0.45); y += 22
    sep(y); y += 12

    # ── Statut calibration ─────────────────────────────────────────────────
    cal_c = C_OK if calib_loaded else C_WARN
    cal_t = "Calibration : OUI" if calib_loaded else "Calibration : NON"
    txt(cal_t, y, cal_c, 0.45); y += 18
    txt(f"Undistort   : {'ON' if undistort else 'OFF'}", y,
        C_OK if undistort else (180,180,180), 0.45); y += 22
    sep(y); y += 12

    # ── Dictionnaire ──────────────────────────────────────────────────────
    txt("Dictionnaire :", y, (160,160,160), 0.43); y += 18
    txt(dict_name, y, C_ACCENT, 0.46); y += 22
    sep(y); y += 12

    # ── Marqueurs du projet ────────────────────────────────────────────────
    txt("MARQUEURS PROJET :", y, (160,160,160), 0.43); y += 18
    for mid, (role, col) in MARKER_ROLES.items():
        present = mid in markers
        dot_col = col if present else (60, 60, 60)
        status  = "DETECTE" if present else "absent"
        # Rond coloré
        cv2.circle(canvas, (x0 + 6, y - 6), 6, dot_col, -1)
        txt(f"  ID {mid} — {role}", y, col if present else (80,80,80), 0.46)
        if present:
            cx, cy2 = marker_center(markers[mid])
            ang = marker_angle_deg(markers[mid])
            txt(f"     pos ({cx}, {cy2})  {ang:.1f}°",
                y + 14, (140, 140, 140), 0.38)
            y += 14
        y += 22
    sep(y); y += 12

    # ── Tous les marqueurs détectés ───────────────────────────────────────
    txt(f"DETECTES : {len(markers)}", y, C_WHITE, 0.48, bold=True); y += 20
    for mid in sorted(markers.keys()):
        role_info = MARKER_ROLES.get(mid, ("", C_WHITE))
        col = role_info[1]
        txt(f"  ID {mid}", y, col, 0.45); y += 17

    sep(y); y += 12

    # ── Vecteur robot→cible ───────────────────────────────────────────────
    if 1 in markers and 2 in markers:
        rp  = marker_center(markers[1])
        tp  = marker_center(markers[2])
        ra  = marker_angle_deg(markers[1])
        dist_px = pixel_distance(rp, tp)
        dx   = tp[0] - rp[0]
        dy   = tp[1] - rp[1]
        angle_to_target = math.degrees(math.atan2(dy, dx))
        error = ((angle_to_target - ra) + 180) % 360 - 180
        txt("NAVIGATION :", y, (160,160,160), 0.43); y += 18
        txt(f"  Dist  : {dist_px:.0f} px", y, C_WHITE, 0.44); y += 16
        txt(f"  Cap   : {ra:.1f} deg", y, C_WHITE, 0.44); y += 16
        txt(f"  Cible : {angle_to_target:.1f} deg", y, C_WHITE, 0.44); y += 16
        err_col = C_OK if abs(error) < 15 else C_WARN
        txt(f"  Erreur: {error:.1f} deg", y, err_col, 0.44); y += 22

    sep(y); y += 12

    # ── Raccourcis ────────────────────────────────────────────────────────
    shortcuts = [
        ("[U] Undistort", C_ACCENT if undistort else (140,140,140)),
        ("[A] Axes 3D",   C_ACCENT if show_axes  else (140,140,140)),
        ("[I] Infos",     C_ACCENT if show_info  else (140,140,140)),
        ("[D] Dico",      (140,140,140)),
        ("[Q] Quitter",   (140,140,140)),
    ]
    for s, c in shortcuts:
        txt(s, y, c, 0.41); y += 16

# ─────────────────────────────────────────────────────────────────────────────
#  OVERLAY MARQUEURS
# ─────────────────────────────────────────────────────────────────────────────

def draw_marker_overlays(frame: np.ndarray, markers: dict,
                          show_axes: bool, show_info: bool,
                          mtx, dist):
    for mid, corners in markers.items():
        role, col = MARKER_ROLES.get(mid, (f"ID {mid}", C_WHITE))
        center = marker_center(corners)
        angle  = marker_angle_deg(corners)

        # Contour du marqueur
        pts = corners[0].astype(np.int32)
        cv2.polylines(frame, [pts], True, col, 2)

        # Point central
        cv2.circle(frame, center, 5, col, -1)

        # Flèche d'orientation
        draw_arrow(frame, center, angle, length=50, color=col, thickness=2)

        # Label
        if show_info:
            label = f"{role}  #{mid}"
            cv2.putText(frame, label,
                        (center[0] + 12, center[1] - 12),
                        FONT_M, 0.55, col, 2, cv2.LINE_AA)
            cv2.putText(frame, f"{angle:.1f}deg",
                        (center[0] + 12, center[1] + 8),
                        FONT, 0.4, col, 1, cv2.LINE_AA)

        # Axes 3D (nécessite calibration)
        if show_axes and mtx is not None and dist is not None:
            size = MARKER_SIZE_MM / 2
            obj_pts = np.array([
                [0, 0, 0], [size, 0, 0], [0, size, 0], [0, 0, -size]
            ], dtype=np.float32)
            corners_norm = corners.astype(np.float32)
            ret, rvec, tvec = cv2.solvePnP(
                np.array([[-size,-size,0],[size,-size,0],
                           [size,size,0],[-size,size,0]], dtype=np.float32),
                corners_norm[0], mtx, dist
            )
            if ret:
                cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, size * 0.8)

    # Ligne robot → cible
    if 1 in markers and 2 in markers:
        rp = marker_center(markers[1])
        tp = marker_center(markers[2])
        cv2.line(frame, rp, tp, (100, 200, 255), 1)
        mid_pt = ((rp[0]+tp[0])//2, (rp[1]+tp[1])//2)
        dist_px = pixel_distance(rp, tp)
        cv2.putText(frame, f"{dist_px:.0f}px",
                    (mid_pt[0]+5, mid_pt[1]-5), FONT, 0.45, (100,200,255), 1)

# ─────────────────────────────────────────────────────────────────────────────
#  BOUCLE PRINCIPALE
# ─────────────────────────────────────────────────────────────────────────────

def main():
    cap           = find_camera()
    mtx, dist     = load_calibration(CALIB_FILE)
    calib_loaded  = mtx is not None

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    dict_idx    = 0
    undistort   = calib_loaded     # activé par défaut si calibration disponible
    show_axes   = calib_loaded
    show_info   = True

    # Map de distorsion précalculée pour les performances
    new_mtx, roi = None, None
    if calib_loaded:
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(
            mtx, dist, (frame_w, frame_h), 1, (frame_w, frame_h)
        )

    fps_counter  = 0
    fps_time     = time.time()
    fps_display  = 0.0

    print("\nInterface lancée. Contrôles : [U] undistort  [A] axes  [I] infos  [D] dico  [Q] quitter\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Compteur FPS
        fps_counter += 1
        if time.time() - fps_time >= 1.0:
            fps_display  = fps_counter / (time.time() - fps_time)
            fps_counter  = 0
            fps_time     = time.time()

        # Correction distorsion
        if undistort and calib_loaded and new_mtx is not None:
            frame = cv2.undistort(frame, mtx, dist, None, new_mtx)

        # Détection ArUco
        dict_name, dict_id = DICTS[dict_idx]
        adict    = aruco.getPredefinedDictionary(dict_id)
        params   = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(adict, params)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = detector.detectMarkers(gray)

        markers = {}
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                markers[int(mid)] = corners_list[i]

        # Overlays sur la frame
        draw_marker_overlays(frame, markers, show_axes, show_info,
                             mtx if undistort else None,
                             dist if undistort else None)

        # Construction du canvas final avec panneau latéral
        canvas = np.zeros((frame_h, frame_w + PANEL_W, 3), dtype=np.uint8)
        canvas[:, :frame_w] = frame
        draw_panel(canvas, markers, fps_display, dict_name,
                   undistort, show_axes, show_info, calib_loaded)

        cv2.imshow("ArUco Verify", canvas)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('u') or key == ord('U'):
            if calib_loaded:
                undistort = not undistort
                print(f"Undistort : {'ON' if undistort else 'OFF'}")
            else:
                print("Calibration non chargée — undistort indisponible.")

        elif key == ord('a') or key == ord('A'):
            show_axes = not show_axes
            print(f"Axes 3D : {'ON' if show_axes else 'OFF'}")

        elif key == ord('i') or key == ord('I'):
            show_info = not show_info

        elif key == ord('d') or key == ord('D'):
            dict_idx = (dict_idx + 1) % len(DICTS)
            print(f"Dictionnaire → {DICTS[dict_idx][0]}")

        elif key == ord('q') or key == ord('Q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Interface fermée.")

if __name__ == "__main__":
    main()
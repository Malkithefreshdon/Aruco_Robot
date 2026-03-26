#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
#  controller_visual.py — Contrôleur ArUco avec interface visuelle complète
#
#  Fenêtre gauche : flux caméra + overlays ArUco (ce que la caméra perçoit)
#  Fenêtre droite : panneau de décision en temps réel
#
#  Usage : python3 controller_visual.py
#
#  Contrôles :
#   [ESPACE]  — démarrer / mettre en pause la navigation
#   [R]       — réinitialiser (stop robot + reset état)
#   [Q/ESC]   — quitter et stopper le robot
# ─────────────────────────────────────────────────────────────────────────────

import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import sys
import os
import time
import math
import threading
import collections

# ═════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION — à adapter
# ═════════════════════════════════════════════════════════════════════════════

ROBOT_IP   = "192.168.137.254"   # ← IP du Raspberry Pi
ROBOT_PORT = 9999

CALIB_FILE      = "camera_calibration.npz"
MARKER_SIZE_MM  = 50.0
ROBOT_HEIGHT_MM = 125.0       # hauteur du marker ID1 (12.5 cm)

# Paramètres navigation
ANGLE_THRESH_DEG  = 12.0   # seuil alignement (°) — en dessous : on avance
DIST_THRESH_CM    = 0.5    # seuil arrivée (cm sol)
TURN_SEC_PER_DEG  = 0.25 / 90   # secondes de pivot par degré d'erreur (à calibrer)
FWD_SEC_PER_CM    = 0.08        # secondes d'avance par cm (à calibrer)
LOOP_HZ           = 4.0         # fréquence de correction (Hz)

# ArUco
ARUCO_DICT_ID = aruco.DICT_4X4_50

# IDs marqueurs
ID_BASE    = 0
ID_ROBOT   = 1
ID_TARGET  = 2   # première destination
ID_TARGET2 = 3   # deuxième destination

# ═════════════════════════════════════════════════════════════════════════════
#  COULEURS & TYPOGRAPHIE (palette industrielle / HUD militaire)
# ═════════════════════════════════════════════════════════════════════════════

# BGR
C_BG        = (14,  18,  24 )
C_PANEL     = (18,  24,  32 )
C_GRID      = (28,  36,  48 )
C_WHITE     = (220, 225, 230)
C_DIM       = (90,  100, 110)
C_ROBOT     = (60,  220,  80)   # vert vif
C_TARGET    = (60,   80, 220)   # bleu
C_TARGET2   = (0,   180, 255)   # cyan-orange — deuxième destination
C_BASE      = (180, 180, 180)   # gris
C_HEADING   = (0,   200, 255)   # cyan — direction actuelle robot
C_PATH      = (0,   200, 140)   # cyan-vert — vecteur sol
C_HYPO      = (80,  110, 150)   # bleu atténué — distance directe
C_WARN      = (0,   165, 255)   # orange
C_OK        = (60,  220,  80)   # vert
C_ERROR     = (60,   60, 220)   # rouge
C_ACCENT    = (0,   210, 170)   # teal

# Commandes → couleur
CMD_COLORS = {
    "FORWARD":  (60,  220,  80),
    "LEFT":     (0,   200, 255),
    "RIGHT":    (0,   200, 255),
    "STOP":     (180, 180, 180),
    "ARRIVED":  (0,   210, 170),
    "WAITING":  (90,  100, 110),
    "LOST":     (60,   60, 220),
}

FONT   = cv2.FONT_HERSHEY_SIMPLEX
FONT_M = cv2.FONT_HERSHEY_DUPLEX

PANEL_W  = 340
LOG_SIZE = 12     # nombre de lignes dans l'historique

# ═════════════════════════════════════════════════════════════════════════════
#  ÉTAT GLOBAL (partagé thread caméra / thread navigation)
# ═════════════════════════════════════════════════════════════════════════════

class State:
    def __init__(self):
        self.lock       = threading.Lock()
        # Détections
        self.markers    = {}        # {id: corners}
        self.frame      = None      # dernière frame caméra
        # Navigation
        self.running    = False     # navigation active
        self.arrived    = False
        self.active_target   = ID_TARGET  # marqueur cible actif (2 ou 3)
        self.current_cmd     = "WAITING"
        self.cmd_duration    = 0.0
        self.angle_error_deg = 0.0
        self.ground_dist_cm  = 0.0
        self.direct_dist_cm  = 0.0
        self.robot_heading   = 0.0
        self.target_bearing  = 0.0
        self.scale_px_mm     = 1.0
        self.fps             = 0.0
        # Log
        self.log = collections.deque(maxlen=LOG_SIZE)
        # Connexion robot
        self.sock       = None
        self.connected  = False

state = State()

# ═════════════════════════════════════════════════════════════════════════════
#  CONNEXION TCP
# ═════════════════════════════════════════════════════════════════════════════

def connect_robot() -> bool:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(3.0)
        s.connect((ROBOT_IP, ROBOT_PORT))
        s.settimeout(None)
        state.sock      = s
        state.connected = True
        state.log.append((time.time(), "CONN", f"Robot connecté  {ROBOT_IP}:{ROBOT_PORT}"))
        print(f"  ✓ Robot connecté : {ROBOT_IP}:{ROBOT_PORT}")
        return True
    except Exception as e:
        state.connected = False
        state.log.append((time.time(), "WARN", f"Robot non connecté : {e}"))
        print(f"  ⚠ Impossible de joindre le robot : {e}")
        print("    Navigation en mode simulation (pas d'envoi TCP).")
        return False

def send_cmd(cmd: str, duration: float = 0.0):
    """Envoie une commande TCP et attend OK. Thread-safe."""
    msg = f"{cmd} {duration:.3f}\n" if duration > 0 else f"{cmd} 0\n"
    if state.connected and state.sock:
        try:
            state.sock.sendall(msg.encode())
            state.sock.recv(64)
        except Exception as e:
            state.log.append((time.time(), "ERR", f"TCP error: {e}"))

# ═════════════════════════════════════════════════════════════════════════════
#  GÉOMÉTRIE
# ═════════════════════════════════════════════════════════════════════════════

def marker_center(corners) -> tuple:
    c = corners[0]
    return (int(np.mean(c[:, 0])), int(np.mean(c[:, 1])))

def marker_angle_deg(corners) -> float:
    c = corners[0]
    return math.degrees(math.atan2(c[1][1]-c[0][1], c[1][0]-c[0][0]))

def estimate_scale(corners) -> float:
    c = corners[0]
    sides = [math.hypot(c[(i+1)%4][0]-c[i][0], c[(i+1)%4][1]-c[i][1]) for i in range(4)]
    return sum(sides)/4 / MARKER_SIZE_MM

def ground_distance_cm(direct_px: float, scale: float) -> tuple:
    """Retourne (direct_cm, ground_cm)."""
    if scale <= 0:
        return 0.0, 0.0
    d_mm = direct_px / scale
    h_mm = ROBOT_HEIGHT_MM
    g_mm = math.sqrt(max(0, d_mm**2 - h_mm**2))
    return d_mm/10, g_mm/10

def normalize_angle(a: float) -> float:
    return (a + 180) % 360 - 180

# ═════════════════════════════════════════════════════════════════════════════
#  THREAD NAVIGATION
# ═════════════════════════════════════════════════════════════════════════════

def navigation_loop():
    period = 1.0 / LOOP_HZ
    while True:
        time.sleep(period)

        with state.lock:
            if not state.running or state.arrived:
                continue
            markers    = dict(state.markers)
            active_tgt = state.active_target

        # Les deux marqueurs nécessaires
        if ID_ROBOT not in markers or active_tgt not in markers:
            with state.lock:
                state.current_cmd = "LOST"
                state.log.append((time.time(), "LOST", "Marqueur(s) manquant(s)"))
            continue

        rp    = marker_center(markers[ID_ROBOT])
        tp    = marker_center(markers[active_tgt])
        r_ang = marker_angle_deg(markers[ID_ROBOT])

        # Échelle depuis marqueur sol
        scale_ref = markers.get(ID_BASE) if ID_BASE in markers else markers.get(active_tgt)
        scale     = estimate_scale(scale_ref) if scale_ref is not None else estimate_scale(markers[ID_ROBOT])

        direct_px   = math.hypot(tp[0]-rp[0], tp[1]-rp[1])
        direct_cm, ground_cm = ground_distance_cm(direct_px, scale)

        dx = tp[0] - rp[0];  dy = tp[1] - rp[1]
        bearing     = math.degrees(math.atan2(dy, dx))
        angle_error = normalize_angle(bearing - r_ang)

        # Mise à jour état pour l'affichage
        with state.lock:
            state.scale_px_mm     = scale
            state.direct_dist_cm  = direct_cm
            state.ground_dist_cm  = ground_cm
            state.robot_heading   = r_ang
            state.target_bearing  = bearing
            state.angle_error_deg = angle_error

        # ── Décision ──────────────────────────────────────────────────────
        if ground_cm < DIST_THRESH_CM:
            send_cmd("STOP")
            with state.lock:
                tgt_num = 2 if state.active_target == ID_TARGET else 3
                state.current_cmd = "ARRIVED"
                state.running     = False
                state.arrived     = True
                state.log.append((time.time(), "ARRIVED",
                                  f"Dest {tgt_num} atteinte ({ground_cm:.1f} cm)"))
            continue

        if abs(angle_error) > ANGLE_THRESH_DEG:
            # Phase 1 : alignement angulaire
            duration = round(abs(angle_error) * TURN_SEC_PER_DEG, 3)
            direction = "LEFT" if angle_error < 0 else "RIGHT"
            with state.lock:
                state.current_cmd  = direction
                state.cmd_duration = duration
                state.log.append((time.time(), direction,
                                  f"err={angle_error:+.1f}°  t={duration:.2f}s"))
            send_cmd(direction, duration)
            time.sleep(duration + 0.05)   # attendre fin d'exécution
        else:
            # Phase 2 : avance
            duration = round(ground_cm * FWD_SEC_PER_CM, 3)
            duration = min(duration, 1.0)  # sécurité : max 1s d'avance d'un coup
            with state.lock:
                state.current_cmd  = "FORWARD"
                state.cmd_duration = duration
                state.log.append((time.time(), "FORWARD",
                                  f"dist={ground_cm:.1f}cm  t={duration:.2f}s"))
            send_cmd("FORWARD", duration)
            time.sleep(duration + 0.05)

# ═════════════════════════════════════════════════════════════════════════════
#  CAMÉRA
# ═════════════════════════════════════════════════════════════════════════════

def find_camera():
    for idx in range(6):
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, f = cap.read()
            if ret and f is not None:
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                print(f"  ✓ Caméra index {idx}  ({w}×{h})")
                return cap
            cap.release()
    print("  ✗ Aucune caméra."); sys.exit(1)

def load_calibration():
    if not os.path.exists(CALIB_FILE):
        return None, None
    d = np.load(CALIB_FILE)
    print(f"  ✓ Calibration chargée")
    return d["camera_matrix"], d["dist_coeffs"]

# ═════════════════════════════════════════════════════════════════════════════
#  DESSIN — VUE CAMÉRA
# ═════════════════════════════════════════════════════════════════════════════

def draw_camera_view(frame: np.ndarray, markers: dict) -> np.ndarray:
    out = frame.copy()

    # Grille légère
    h, w = out.shape[:2]
    for x in range(0, w, 80):
        cv2.line(out, (x,0), (x,h), C_GRID, 1)
    for y in range(0, h, 80):
        cv2.line(out, (0,y), (w,y), C_GRID, 1)

    role_meta = {
        ID_BASE:    ("BASE",   C_BASE,   (200,200,200)),
        ID_ROBOT:   ("ROBOT",  C_ROBOT,  (60,220,80)),
        ID_TARGET:  ("DEST 1", C_TARGET, (60,80,220)),
        ID_TARGET2: ("DEST 2", C_TARGET2,(0,180,255)),
    }

    for mid, corners in markers.items():
        role, col, _ = role_meta.get(mid, (f"ID{mid}", C_WHITE, C_WHITE))
        pts    = corners[0].astype(np.int32)
        center = marker_center(corners)
        angle  = marker_angle_deg(corners)

        # Halo
        cv2.circle(out, center, 22, col, 1)

        # Contour
        cv2.polylines(out, [pts], True, col, 2)

        # Centre
        cv2.circle(out, center, 5, col, -1)

        # Flèche orientation
        rad = math.radians(angle)
        tip = (int(center[0] + 55*math.cos(rad)), int(center[1] + 55*math.sin(rad)))
        cv2.arrowedLine(out, center, tip, col, 2, tipLength=0.3)

        # Label fond
        label = f"{role} #{mid}"
        (tw, th), _ = cv2.getTextSize(label, FONT_M, 0.5, 1)
        lx, ly = center[0]+14, center[1]-14
        cv2.rectangle(out, (lx-3, ly-th-3), (lx+tw+3, ly+4), (10,12,16), -1)
        cv2.putText(out, label, (lx, ly), FONT_M, 0.5, col, 1, cv2.LINE_AA)

    # ── Ligne sol robot→cible active ──────────────────────────────────────────
    with state.lock:
        active_tgt = state.active_target
    if ID_ROBOT in markers and active_tgt in markers:
        rp = marker_center(markers[ID_ROBOT])
        tp = marker_center(markers[active_tgt])
        tgt_col = C_TARGET if active_tgt == ID_TARGET else C_TARGET2
        mid_pt = ((rp[0]+tp[0])//2, (rp[1]+tp[1])//2)

        direct_px  = math.hypot(tp[0]-rp[0], tp[1]-rp[1])
        scale_ref  = markers.get(ID_BASE) if ID_BASE in markers else (markers.get(active_tgt) if active_tgt in markers else markers[ID_ROBOT])
        scale      = estimate_scale(scale_ref)
        direct_cm, ground_cm = ground_distance_cm(direct_px, scale)

        # Ligne directe (pointillée, atténuée)
        _dashed_line(out, rp, tp, C_HYPO, thickness=1)

        # Ligne sol (pleine, colorée)
        cv2.line(out, rp, tp, tgt_col, 2)
        _end_tick(out, rp, tp, tgt_col)
        _end_tick(out, tp, rp, tgt_col)

        # Label distance sol
        perp = _perp_offset(rp, tp, -18)
        lx = mid_pt[0] + perp[0]
        ly = mid_pt[1] + perp[1]
        label = f"{ground_cm:.1f} cm"
        (tw, th), _ = cv2.getTextSize(label, FONT_M, 0.58, 2)
        cv2.rectangle(out, (lx-5, ly-th-5), (lx+tw+5, ly+5), (8,10,14), -1)
        cv2.putText(out, label, (lx, ly), FONT_M, 0.58, tgt_col, 2, cv2.LINE_AA)

        # Label distance directe
        perp2 = _perp_offset(rp, tp, +14)
        cv2.putText(out, f"direct {direct_cm:.1f}cm",
                    (mid_pt[0]+perp2[0]-20, mid_pt[1]+perp2[1]),
                    FONT, 0.34, C_HYPO, 1, cv2.LINE_AA)

        # Arc angle error
        with state.lock:
            err = state.angle_error_deg
            hdg = state.robot_heading
        _draw_angle_arc(out, rp, hdg, err, radius=45)

    return out


def _draw_angle_arc(img, center, heading_deg, error_deg, radius=45):
    """Arc coloré montrant l'erreur angulaire à corriger."""
    if abs(error_deg) < 1:
        return
    col  = C_OK if abs(error_deg) < ANGLE_THRESH_DEG else C_WARN
    axes = (radius, radius)
    # openCV ellipse : angles en degrés, sens horaire depuis axe X
    start_a = heading_deg
    end_a   = heading_deg + error_deg
    if start_a > end_a:
        start_a, end_a = end_a, start_a
    cv2.ellipse(img, center, axes, 0,
                start_a, end_a, col, 2)
    # Flèche au bout de l'arc
    rad_end = math.radians(heading_deg + error_deg)
    tip = (int(center[0] + radius * math.cos(rad_end)),
           int(center[1] + radius * math.sin(rad_end)))
    cv2.circle(img, tip, 4, col, -1)

# ═════════════════════════════════════════════════════════════════════════════
#  DESSIN — PANNEAU DÉCISION
# ═════════════════════════════════════════════════════════════════════════════

def draw_decision_panel(height: int) -> np.ndarray:
    panel = np.full((height, PANEL_W, 3), C_PANEL, dtype=np.uint8)

    # Ligne séparatrice gauche
    cv2.line(panel, (0, 0), (0, height), C_GRID, 2)

    x0 = 16
    y  = 0

    def txt(text, cy, color=C_WHITE, scale=0.46, bold=False):
        cv2.putText(panel, text, (x0, cy), FONT, scale, color,
                    2 if bold else 1, cv2.LINE_AA)

    def sep(cy, alpha=55):
        cv2.line(panel, (x0-4, cy), (PANEL_W-8, cy), (alpha, alpha+8, alpha+16), 1)

    def section(label, cy):
        cv2.rectangle(panel, (x0-4, cy-14), (PANEL_W-8, cy+4),
                      (26,32,42), -1)
        cv2.putText(panel, label, (x0, cy), FONT, 0.4,
                    (100,115,130), 1, cv2.LINE_AA)
        return cy + 16

    # ── TITRE ─────────────────────────────────────────────────────────────
    y += 22
    cv2.putText(panel, "NAVIGATION", (x0, y), FONT_M, 0.7,
                C_ACCENT, 2, cv2.LINE_AA)
    y += 20
    cv2.putText(panel, "CONTROLLER", (x0+2, y), FONT_M, 0.45,
                C_DIM, 1, cv2.LINE_AA)
    y += 18
    sep(y); y += 12

    # ── CONNEXION ─────────────────────────────────────────────────────────
    with state.lock:
        connected = state.connected
        fps       = state.fps
        running   = state.running
        arrived   = state.arrived
        active_tgt = state.active_target

    conn_col = C_OK if connected else C_WARN
    conn_txt = f"Robot  {ROBOT_IP}" if connected else "Robot  NON CONNECTE"
    cv2.circle(panel, (x0+5, y-5), 5, conn_col, -1)
    txt(f"  {conn_txt}", y, conn_col, 0.43)
    y += 18
    txt(f"FPS : {fps:.1f}", y, C_DIM, 0.42)
    y += 22
    sep(y); y += 10

    # ── DESTINATION ACTIVE ────────────────────────────────────────────────
    y = section("DESTINATION", y); y += 6
    if active_tgt == ID_TARGET:
        dest_label = "DEST 1  (ArUco #2)"
        dest_col   = C_TARGET
    else:
        dest_label = "DEST 2  (ArUco #3)"
        dest_col   = C_TARGET2
    cv2.rectangle(panel, (x0-4, y), (PANEL_W-8, y+22), (22,28,38), -1)
    cv2.rectangle(panel, (x0-4, y), (PANEL_W-8, y+22), dest_col, 1)
    cv2.putText(panel, dest_label, (x0+6, y+15), FONT_M, 0.52, dest_col, 1, cv2.LINE_AA)
    y += 30
    sep(y); y += 10

    # ── COMMANDE COURANTE ─────────────────────────────────────────────────
    y = section("COMMANDE COURANTE", y); y += 4

    with state.lock:
        cmd  = state.current_cmd
        dur  = state.cmd_duration

    cmd_col = CMD_COLORS.get(cmd, C_WHITE)

    # Grand bloc commande
    bh = 52
    cv2.rectangle(panel, (x0-4, y), (PANEL_W-8, y+bh), (22,28,38), -1)
    cv2.rectangle(panel, (x0-4, y), (PANEL_W-8, y+bh), cmd_col, 1)

    # Icône
    icons = {"FORWARD":"▲", "LEFT":"◄", "RIGHT":"►",
             "STOP":"■", "ARRIVED":"✓", "WAITING":"…", "LOST":"✗"}
    icon = icons.get(cmd, "?")
    cv2.putText(panel, icon, (x0+6, y+36), FONT_M, 1.1, cmd_col, 2, cv2.LINE_AA)
    cv2.putText(panel, cmd, (x0+52, y+24), FONT_M, 0.7, cmd_col, 2, cv2.LINE_AA)
    if dur > 0:
        cv2.putText(panel, f"{dur:.2f} s", (x0+52, y+44),
                    FONT, 0.44, C_DIM, 1, cv2.LINE_AA)
    y += bh + 14
    sep(y); y += 10

    # ── MÉTRIQUES NAVIGATION ──────────────────────────────────────────────
    y = section("METRIQUES", y); y += 6

    with state.lock:
        gcm   = state.ground_dist_cm
        dcm   = state.direct_dist_cm
        err   = state.angle_error_deg
        hdg   = state.robot_heading
        bear  = state.target_bearing
        scale = state.scale_px_mm

    # Distance sol — barre de progression
    max_dist = 150.0   # cm max affiché
    bar_w    = PANEL_W - x0 - 64
    fill_w   = int(min(gcm / max_dist, 1.0) * bar_w)
    bar_col  = C_OK if gcm < 20 else (C_WARN if gcm < 60 else C_WHITE)
    txt("Distance sol :", y, C_DIM, 0.40); y += 16
    cv2.rectangle(panel, (x0, y), (x0+bar_w, y+12), (30,36,44), -1)
    if fill_w > 0:
        cv2.rectangle(panel, (x0, y), (x0+fill_w, y+12), bar_col, -1)
    cv2.putText(panel, f"{gcm:.1f} cm", (x0+bar_w+6, y+10),
                FONT, 0.43, bar_col, 1, cv2.LINE_AA)
    y += 20

    txt(f"Direct (hypo) : {dcm:.1f} cm", y, C_DIM, 0.41); y += 18
    txt(f"Echelle       : {scale:.2f} px/mm", y, C_DIM, 0.41); y += 20
    sep(y); y += 10

    # ── ANGLES ────────────────────────────────────────────────────────────
    y = section("ANGLES", y); y += 6

    err_col = C_OK if abs(err) < ANGLE_THRESH_DEG else C_WARN
    txt(f"Cap robot  : {hdg:+.1f} deg", y, C_WHITE, 0.43); y += 18
    txt(f"Cap cible  : {bear:+.1f} deg", y, C_WHITE, 0.43); y += 18
    txt(f"Erreur     : {err:+.1f} deg", y, err_col, 0.46, bold=True); y += 20

    # Jauge angulaire ± 180°
    gauge_w = PANEL_W - x0 - 20
    gauge_h = 14
    cv2.rectangle(panel, (x0, y), (x0+gauge_w, y+gauge_h), (30,36,44), -1)
    center_x = x0 + gauge_w//2
    cv2.line(panel, (center_x, y), (center_x, y+gauge_h), (60,70,80), 1)
    # Zone OK (±ANGLE_THRESH)
    ok_half = int(ANGLE_THRESH_DEG / 180 * gauge_w//2)
    cv2.rectangle(panel, (center_x-ok_half, y+2), (center_x+ok_half, y+gauge_h-2),
                  (30, 55, 35), -1)
    # Curseur erreur
    cursor_x = int(center_x + (err / 180) * gauge_w//2)
    cursor_x = max(x0, min(x0+gauge_w, cursor_x))
    cv2.rectangle(panel, (cursor_x-3, y), (cursor_x+3, y+gauge_h), err_col, -1)
    y += gauge_h + 14
    sep(y); y += 10

    # ── SEUILS ────────────────────────────────────────────────────────────
    y = section("SEUILS", y); y += 6
    txt(f"Alignement  : ±{ANGLE_THRESH_DEG:.0f} deg", y, C_DIM, 0.41); y += 16
    txt(f"Arrivee     : {DIST_THRESH_CM:.0f} cm", y, C_DIM, 0.41); y += 20
    sep(y); y += 10

    # ── HISTORIQUE COMMANDES ──────────────────────────────────────────────
    y = section(f"HISTORIQUE ({LOG_SIZE} derniers)", y); y += 6

    with state.lock:
        log_items = list(state.log)

    for ts, lvl, msg in reversed(log_items):
        if y > height - 22:
            break
        col_map = {"CONN": C_ACCENT, "ARRIVED": C_ACCENT,
                   "FORWARD": C_ROBOT, "LEFT": C_HEADING, "RIGHT": C_HEADING,
                   "STOP": C_DIM, "LOST": C_ERROR, "WARN": C_WARN,
                   "ERR": C_ERROR}
        col = col_map.get(lvl, C_DIM)
        t_str  = time.strftime("%H:%M:%S", time.localtime(ts))
        cv2.putText(panel, t_str, (x0, y), FONT, 0.35, (70,80,90), 1, cv2.LINE_AA)
        cv2.putText(panel, f"[{lvl}]", (x0+50, y), FONT, 0.37, col, 1, cv2.LINE_AA)
        cv2.putText(panel, msg[:28], (x0+100, y), FONT, 0.37, C_DIM, 1, cv2.LINE_AA)
        y += 16

    # ── CONTRÔLES ─────────────────────────────────────────────────────────
    ctrl_y = height - 74
    sep(ctrl_y); ctrl_y += 10
    run_col = C_OK if running else C_DIM
    cv2.putText(panel, "[ESPACE] Start/Pause", (x0, ctrl_y),
                FONT, 0.41, run_col, 1, cv2.LINE_AA); ctrl_y += 16
    cv2.putText(panel, "[N] → Destination 2", (x0, ctrl_y),
                FONT, 0.41, C_TARGET2, 1, cv2.LINE_AA); ctrl_y += 16
    cv2.putText(panel, "[R] Reset   [Q] Quitter", (x0, ctrl_y),
                FONT, 0.41, C_DIM, 1, cv2.LINE_AA)

    return panel

# ═════════════════════════════════════════════════════════════════════════════
#  HELPERS GÉOMÉTRIQUES (visuels)
# ═════════════════════════════════════════════════════════════════════════════

def _perp_offset(p1, p2, d):
    dx = p2[0]-p1[0]; dy = p2[1]-p1[1]
    n  = math.hypot(dx, dy)
    if n == 0: return (0, 0)
    return (int(-dy/n*d), int(dx/n*d))

def _end_tick(img, a, b, col, l=7):
    ox, oy = _perp_offset(a, b, l)
    cv2.line(img, (a[0]-ox, a[1]-oy), (a[0]+ox, a[1]+oy), col, 2)

def _dashed_line(img, p1, p2, col, thickness=1, dash=8, gap=6):
    dx = p2[0]-p1[0]; dy = p2[1]-p1[1]
    n  = math.hypot(dx, dy)
    if n == 0: return
    ux, uy = dx/n, dy/n
    pos, draw = 0.0, True
    while pos < n:
        seg = dash if draw else gap
        ep  = min(pos+seg, n)
        if draw:
            s = (int(p1[0]+ux*pos), int(p1[1]+uy*pos))
            e = (int(p1[0]+ux*ep),  int(p1[1]+uy*ep))
            cv2.line(img, s, e, col, thickness)
        pos += seg; draw = not draw

# ═════════════════════════════════════════════════════════════════════════════
#  BOUCLE PRINCIPALE
# ═════════════════════════════════════════════════════════════════════════════

def main():
    print("\n─── Aruco Based Robot Controller ───")
    cap       = find_camera()
    mtx, dist = load_calibration()
    calib_ok  = mtx is not None

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Calibration undistort map
    new_mtx = None
    if calib_ok:
        new_mtx, _ = cv2.getOptimalNewCameraMatrix(
            mtx, dist, (frame_w, frame_h), 1, (frame_w, frame_h))

    # Détecteur ArUco
    adict    = aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    params   = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(adict, params)

    # Connexion robot (non bloquant)
    connect_robot()

    # Thread navigation
    nav_thread = threading.Thread(target=navigation_loop, daemon=True)
    nav_thread.start()

    state.log.append((time.time(), "INFO", "Appuie ESPACE pour demarrer"))

    fps_t  = time.time()
    fps_n  = 0

    print("\n  [ESPACE] Démarrer   [N] Dest 2   [R] Reset   [Q] Quitter\n")

    while True:
        ret, raw = cap.read()
        if not ret:
            continue

        fps_n += 1
        if time.time() - fps_t >= 1.0:
            with state.lock:
                state.fps = fps_n / (time.time() - fps_t)
            fps_n = 0; fps_t = time.time()

        # Undistort
        frame = cv2.undistort(raw, mtx, dist, None, new_mtx) if calib_ok and new_mtx is not None else raw

        # Détection ArUco
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = detector.detectMarkers(gray)

        markers = {}
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                markers[int(mid)] = corners_list[i]

        with state.lock:
            state.markers = dict(markers)
            state.frame   = frame.copy()

        # ── Rendu ──────────────────────────────────────────────────────────
        cam_view = draw_camera_view(frame, markers)
        panel    = draw_decision_panel(frame_h)

        # En-tête statut sur la vue caméra
        with state.lock:
            running = state.running; arrived = state.arrived
        status_txt = "● EN MARCHE" if running else ("✓ ARRIVE" if arrived else "⏸ EN PAUSE")
        status_col = C_OK if running else (C_ACCENT if arrived else C_WARN)
        cv2.rectangle(cam_view, (0,0), (frame_w, 32), (8,10,14), -1)
        cv2.putText(cam_view, status_txt, (10, 22), FONT_M, 0.6,
                    status_col, 2, cv2.LINE_AA)
        m_txt = f"Marqueurs : {len(markers)}/4"
        cv2.putText(cam_view, m_txt, (frame_w-160, 22), FONT, 0.5,
                    C_OK if len(markers)==4 else C_WARN, 1, cv2.LINE_AA)

        # Assemblage final
        canvas = np.zeros((frame_h, frame_w + PANEL_W, 3), dtype=np.uint8)
        canvas[:, :frame_w] = cam_view
        canvas[:, frame_w:] = panel

        cv2.imshow("Robot Controller", canvas)
        key = cv2.waitKey(1) & 0xFF

        # Espace : start/pause
        if key == ord(' '):
            with state.lock:
                if state.arrived:
                    pass   # ne peut pas redémarrer après arrivée (→ [R])
                else:
                    state.running = not state.running
                    verb = "Démarrage" if state.running else "Pause"
                    state.log.append((time.time(), "INFO", verb))
                    print(f"  {verb}")

        # R : reset
        elif key == ord('r') or key == ord('R'):
            send_cmd("STOP")
            with state.lock:
                state.running      = False
                state.arrived      = False
                state.active_target = ID_TARGET
                state.current_cmd  = "WAITING"
                state.log.append((time.time(), "INFO", "Reset → Dest 1"))
            print("  Reset.")

        # N : basculer vers destination 2 (ArUco #3)
        elif key == ord('n') or key == ord('N'):
            with state.lock:
                if state.active_target != ID_TARGET2:
                    state.active_target = ID_TARGET2
                    state.arrived       = False
                    state.running       = True
                    state.log.append((time.time(), "INFO", "→ Dest 2 (ArUco #3)"))
                    print("  Navigation vers destination 2 (ArUco #3)")

        # Q / ESC : quitter
        elif key == ord('q') or key == ord('Q') or key == 27:
            break

    # Nettoyage
    send_cmd("STOP")
    if state.sock:
        state.sock.close()
    cap.release()
    cv2.destroyAllWindows()
    print("Terminé.")

if __name__ == "__main__":
    main()
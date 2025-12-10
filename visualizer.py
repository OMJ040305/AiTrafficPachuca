import cv2
import numpy as np
import time
import config as cfg


def draw_traffic_light(frame, state, position):
    """Dibuja el semáforo principal"""
    cv2.circle(frame, position, 20, cfg.TRAFFIC_LIGHT_COLORS[state], -1)
    cv2.circle(frame, position, 20, (255, 255, 255), 2)


def draw_arrow_light(frame, state, position):
    """Dibuja el semáforo de flecha"""
    cv2.circle(frame, position, 15, cfg.TRAFFIC_LIGHT_COLORS[state], -1)
    cv2.circle(frame, position, 15, (255, 255, 255), 2)


def draw_direction_arrow(frame, idx, arrow_state):
    """Dibuja la flecha indicadora de dirección"""
    if idx not in [cfg.ESTE_IDX, cfg.OESTE_IDX]: return

    height, width = frame.shape[:2]
    cx, cy = width - 80, 40

    pts = np.array([
        [cx - 10, cy - 10], [cx + 5, cy - 10], [cx + 5, cy - 20],
        [cx + 25, cy],
        [cx + 5, cy + 20], [cx + 5, cy + 10], [cx - 10, cy + 10]
    ], np.int32).reshape((-1, 1, 2))

    cv2.fillPoly(frame, [pts], cfg.TRAFFIC_LIGHT_COLORS[arrow_state])
    cv2.polylines(frame, [pts], True, (255, 255, 255), 1)


def add_overlay(frame, channel, name, idx, system_state):
    """Dibuja la información sobre cada cámara individual"""
    height, width = frame.shape[:2]

    # Info de Cámara
    cv2.putText(frame, name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    mode_txt = f"Modo: {cfg.SYSTEM_MODES[system_state['mode']]}"
    col = (0, 255, 0) if system_state['status'] == 'active' else (0, 0, 255)
    cv2.putText(frame, mode_txt, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

    # Semáforos
    draw_traffic_light(frame, system_state['traffic_color'], (width - 40, 40))

    if idx in [cfg.ESTE_IDX, cfg.OESTE_IDX]:
        draw_arrow_light(frame, system_state['arrow_color'], (width - 80, 40))
        draw_direction_arrow(frame, idx, system_state['arrow_color'])

    # Debug Visual (Zonas y Conteos)
    if system_state['mode'] == 'INTELLIGENT' and system_state['counts']:
        cv2.putText(frame, f"RECTO: {system_state['counts']['main']}", (10, height - 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if idx in [cfg.ESTE_IDX, cfg.OESTE_IDX]:
            cv2.putText(frame, f"FLECHA: {system_state['counts']['arrow']}", (10, height - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        main_z, arrow_z = system_state['zones']

        if len(main_z) > 0:
            pts_m = np.array(main_z, np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts_m], True, (255, 0, 0), 2)
        if len(arrow_z) > 0:
            pts_a = np.array(arrow_z, np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts_a], True, (0, 255, 0), 2)

    return frame


def draw_edit_mode(frame, points, camera_name, zone_type):
    """Dibuja la interfaz de EDICIÓN con menú lateral."""
    h, w = frame.shape[:2]
    MENU_W = 300
    canvas = np.zeros((h, w + MENU_W, 3), dtype=np.uint8)
    canvas[:, :w] = frame
    canvas[:, w:] = (40, 40, 40)

    if len(points) > 0:
        pts = np.array(points, np.int32).reshape((-1, 1, 2))
        cv2.polylines(canvas, [pts], True, (0, 255, 0), 2)
        for p in points:
            cv2.circle(canvas, (p[0], p[1]), 5, (0, 255, 255), -1)

    ui_x = w + 20
    cv2.putText(canvas, "MODO EDICION", (ui_x, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(canvas, camera_name, (ui_x, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    type_text = "ZONA: RECTA" if zone_type == 'main' else "ZONA: FLECHA"
    type_col = (255, 100, 100) if zone_type == 'main' else (100, 255, 100)
    cv2.putText(canvas, type_text, (ui_x, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, type_col, 2)

    instrucciones = [("Click: Poner punto", (200, 200, 200)), ("'T': Cambiar Tipo", (200, 200, 200)),
                     ("'Z': Deshacer", (200, 200, 200)), ("'S': GUARDAR", (0, 255, 0)),
                     ("'ESC': Cancelar", (0, 0, 255))]
    y_start = 180
    for text, color in instrucciones:
        cv2.putText(canvas, text, (ui_x, y_start), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        y_start += 30
    return canvas


def draw_dashboard(grid_frame, info_data, stats_data=None, sync_data=None):
    """
    Dibuja el menú lateral principal.
    sync_data: Diccionario con toda la info de red y sincronización
    """
    h, w = grid_frame.shape[:2]
    MENU_W = 350
    canvas = np.zeros((h, w + MENU_W, 3), dtype=np.uint8)
    canvas[:, :w] = grid_frame
    canvas[:, w:] = (30, 30, 30)
    ui_x = w + 20

    # --- TÍTULO ---
    cv2.putText(canvas, "CONTROL TRAFICO", (ui_x, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.line(canvas, (ui_x, 40), (w + MENU_W - 20, 40), (100, 100, 100), 1)

    # --- ESTADÍSTICAS ---
    if stats_data:
        _, total_cars, total_incidents = stats_data
        cv2.putText(canvas, f"AUTOS: {total_cars} | INCID: {total_incidents}", (ui_x, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    # --- FASE ACTUAL ---
    cv2.line(canvas, (ui_x, 80), (w + MENU_W - 20, 80), (100, 100, 100), 1)
    phases = ["1. Flechas E-O", "2. Rectos E-O", "3. Norte", "4. Sur"]
    current = info_data['phase_idx']
    y_ph = 105
    for i, ph_name in enumerate(phases):
        color = (0, 255, 0) if i == current else (80, 80, 80)
        thickness = 2 if i == current else 1
        prefix = "> " if i == current else "  "
        cv2.putText(canvas, prefix + ph_name, (ui_x, y_ph), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)
        y_ph += 25

    # --- SECCIÓN RED Y VPS ---
    y_net = 230
    cv2.line(canvas, (ui_x, y_net - 15), (w + MENU_W - 20, y_net - 15), (100, 100, 100), 1)
    cv2.putText(canvas, "RED & NODOS", (ui_x, y_net), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
    y_net += 25

    if sync_data:
        # Estado VPS
        status = sync_data['connection_status']
        col_vps = (0, 255, 0) if status == 'CONNECTED' else (0, 0, 255)
        icon_vps = "CONECTADO" if status == 'CONNECTED' else status
        cv2.putText(canvas, f"VPS: {icon_vps}", (ui_x, y_net), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col_vps, 1)
        y_net += 20

        # Mensajes
        msg_str = f"MSG: Tx {sync_data['msg_sent']} | Rx {sync_data['msg_recv']}"
        cv2.putText(canvas, msg_str, (ui_x, y_net), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        y_net += 25

        # Lista de Nodos Remotos
        remotes = sync_data.get('remote_data', {})
        if not remotes:
            cv2.putText(canvas, "Sin nodos remotos...", (ui_x, y_net), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 100),
                        1)
            y_net += 20

        for rid, rdata in remotes.items():
            age = time.time() - rdata.get('timestamp', 0)
            col_node = (0, 255, 0) if age < 5 else (0, 255, 255) if age < 15 else (0, 0, 255)
            fase = rdata.get('fase', '?')
            veh = rdata.get('stats', {}).get('total_vehicles', 0)
            txt = f"> {rid}: F{fase} ({veh} veh)"
            cv2.putText(canvas, txt, (ui_x, y_net), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col_node, 1)
            y_net += 20

        # --- SECCIÓN SINCRONIZACIÓN ---
        y_sync = 380
        cv2.line(canvas, (ui_x, y_sync - 15), (w + MENU_W - 20, y_sync - 15), (100, 100, 100), 1)

        if sync_data['enabled']:
            header = "SYNC: ACTIVADA"
            col_head = (0, 255, 0)
        else:
            header = "SYNC: DESACTIVADA"
            col_head = (100, 100, 100)

        cv2.putText(canvas, header, (ui_x, y_sync), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col_head, 1)
        y_sync += 25

        # Estrategia
        strat = sync_data['strategy']
        strat_cols = {'ADAPTIVE': (0, 255, 255), 'PRIORITY': (255, 100, 255),
                      'BALANCED': (100, 255, 100), 'GREEN_WAVE': (100, 200, 255)}
        cv2.putText(canvas, f"MODO: {strat}", (ui_x, y_sync), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    strat_cols.get(strat, (255, 255, 255)), 1)
        y_sync += 20

        # Score
        score = sync_data['score']
        col_sc = (0, 255, 0) if score > 70 else (0, 255, 255) if score > 40 else (0, 0, 255)
        cv2.putText(canvas, f"Coordinacion: {int(score)}%", (ui_x, y_sync), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col_sc, 1)
        y_sync += 20

        # Metricas
        cv2.putText(canvas, f"Optimizaciones: {sync_data['optimizations']}", (ui_x, y_sync), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (200, 200, 200), 1)
        y_sync += 20
        cv2.putText(canvas, f"Conflictos: {sync_data['conflicts']}", (ui_x, y_sync), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (100, 100, 255), 1)

    # Pie de página
    cv2.rectangle(canvas, (ui_x, 680), (ui_x + 120, 715), (50, 50, 50), -1)
    cv2.putText(canvas, "Q: SALIR", (ui_x + 10, 705), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return canvas
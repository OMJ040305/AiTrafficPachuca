import datetime
import math
import os
import threading
import time
import json
import zmq
import cv2
import numpy as np
import requests

# Importar módulos propios
import config as cfg
import visualizer as vis
from detector import VehicleDetector
from stats import StatsManager
from tracker import EuclideanDistTracker

# ==========================================
# CONFIGURACIÓN DE RED
# ==========================================
VPS_IP = "213.199.32.40"
NODE_ID = "INTERSECCION_A"


# ==========================================

class TrafficLightSystem:
    def __init__(self):
        self.cameras = {}
        # Estados del sistema
        self.traffic_states = {ch: 'red' for ch in cfg.CAMERA_CHANNELS}
        self.arrow_states = {ch: 'red' for ch in cfg.CAMERA_CHANNELS}
        self.detection_counts = {ch: {'main': 0, 'arrow': 0} for ch in cfg.CAMERA_CHANNELS}
        self.camera_status = {ch: 'unknown' for ch in cfg.CAMERA_CHANNELS}
        self.camera_failures = {ch: 0 for ch in cfg.CAMERA_CHANNELS}
        self.system_mode = {ch: 'INTELLIGENT' for ch in cfg.CAMERA_CHANNELS}
        self.last_frame_time = {ch: time.time() for ch in cfg.CAMERA_CHANNELS}

        # Rastreo
        self.trackers = {ch: EuclideanDistTracker() for ch in cfg.CAMERA_CHANNELS}
        self.vehicle_data = {ch: {} for ch in cfg.CAMERA_CHANNELS}
        self.stats_manager = StatsManager()

        # Configuración Incidentes
        self.STOP_THRESHOLD = 15
        self.ACCIDENT_TIME = 20.0
        self.COLLISION_DIST = 150

        # Caché visual
        self.last_detections = {ch: [] for ch in cfg.CAMERA_CHANNELS}
        self.frame_counter = 0
        self.detection_interval = 5

        # Zonas
        self.live_zones = {}
        for i, ch in enumerate(cfg.CAMERA_CHANNELS):
            mz, az = cfg.get_zones(i)
            self.live_zones[ch] = {
                'main': mz[0] if len(mz) > 0 else [],
                'arrow': az[0] if len(az) > 0 else []
            }

        # Edición
        self.is_editing = False
        self.edit_channel = None
        self.edit_zone_type = 'main'
        self.edit_points = []
        self.click_cooldown = 0

        # Detector y Control
        self.detector = VehicleDetector()
        self.current_phase = 0
        self.phase_start_time = time.time()
        self.sequence_lock = threading.Lock()
        self.running = True

        # RED y ZMQ
        self.remote_data = {}
        self.zmq_context = zmq.Context()
        self.connection_status = 'DISCONNECTED'
        self.last_heartbeat = time.time()
        self.heartbeat_interval = 2.0
        self.messages_sent = 0
        self.messages_received = 0
        self.connection_attempts = 0
        self.vps_last_seen = time.time()
        self.system_start_time = time.time()

        # SINCRONIZACIÓN AVANZADA
        self.sync_enabled = True
        self.sync_history = []
        self.sync_adjustments = {ch: 0 for ch in cfg.CAMERA_CHANNELS}
        self.traffic_pressure = {ch: 0.0 for ch in cfg.CAMERA_CHANNELS}
        self.sync_strategy = 'ADAPTIVE'
        self.green_wave_direction = None
        self.coordination_score = 100.0
        self.conflict_count = 0
        self.optimization_count = 0
        self.SYNC_MIN_ADJUSTMENT = 2
        self.SYNC_MAX_ADJUSTMENT = 10

        # Sockets
        self.publisher = self.zmq_context.socket(zmq.PUB)
        try:
            self.publisher.connect(f"tcp://{VPS_IP}:5555")
            print(f"[RED] 📡 Conectado al VPS ({VPS_IP}) para enviar datos.")
        except Exception as e:
            print(f"[RED] ❌ Error conectando publicador: {e}")

        # Hilos
        self.threads = []
        self.threads.append(threading.Thread(target=self.intelligent_control, daemon=True))
        self.threads.append(threading.Thread(target=self.standard_control, daemon=True))
        self.threads.append(threading.Thread(target=self.monitor_cameras, daemon=True))
        self.threads.append(threading.Thread(target=self.network_listener, daemon=True))
        self.threads.append(threading.Thread(target=self.heartbeat_sender, daemon=True))

        for t in self.threads: t.start()
        time.sleep(1)
        self.send_telemetry(msg_type='SYSTEM_START')

    # ==========================================
    # RED Y SINCRONIZACIÓN (LA LÓGICA QUE PEDISTE)
    # ==========================================

    def network_listener(self):
        subscriber = self.zmq_context.socket(zmq.SUB)
        while self.running:
            try:
                if self.connection_status != 'CONNECTED':
                    subscriber.connect(f"tcp://{VPS_IP}:5556")
                    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
                    self.connection_status = 'CONNECTED'
                try:
                    msg = subscriber.recv_json(flags=zmq.NOBLOCK)
                    self.vps_last_seen = time.time()
                    self.messages_received += 1
                    if msg.get('node_id') == NODE_ID: continue

                    msg_type = msg.get('type', 'STATE_UPDATE')
                    sender_id = msg.get('node_id', 'unknown')

                    if msg_type == 'STATE_UPDATE':
                        self.remote_data[sender_id] = msg
                        self.sync_with_remote(sender_id, msg)
                    elif msg_type == 'STRATEGY_CHANGE':
                        print(f"[SYNC] Nodo {sender_id} cambio estrategia.")
                except zmq.Again:
                    if time.time() - self.vps_last_seen > 15: self.connection_status = 'TIMEOUT'
                    time.sleep(0.1)
            except Exception:
                self.connection_status = 'DISCONNECTED'
                time.sleep(5)

    def send_telemetry(self, msg_type='STATE_UPDATE', extra_data=None):
        try:
            payload = {
                "type": msg_type, "node_id": NODE_ID, "timestamp": time.time(),
                "connection_status": self.connection_status, "fase": self.current_phase,
                "stats": {"vehicles": sum(t.id_count for t in self.trackers.values())},
                "cameras": {str(c): {"vehicles": self.detection_counts[c], "status": self.camera_status[c]} for c in
                            cfg.CAMERA_CHANNELS}
            }
            if extra_data: payload.update(extra_data)
            self.publisher.send_json(payload)
            self.messages_sent += 1
        except:
            pass

    def heartbeat_sender(self):
        while self.running:
            if time.time() - self.last_heartbeat >= 2.0:
                self.send_telemetry('HEARTBEAT')
                self.last_heartbeat = time.time()
            time.sleep(1)

    # --- Lógica de Sync Avanzada ---
    def sync_with_remote(self, remote_id, remote_data):
        if not self.sync_enabled: return
        remote_fase = remote_data.get('fase', 0)
        remote_stats = remote_data.get('stats', {})
        remote_cameras = remote_data.get('cameras', {})

        remote_pressure = self.calculate_traffic_pressure(remote_cameras)
        local_pressure = self.calculate_traffic_pressure(
            {str(c): {"vehicles": self.detection_counts[c], "status": self.camera_status[c]} for c in
             cfg.CAMERA_CHANNELS})

        if self.sync_strategy == 'ADAPTIVE':
            self.adaptive_sync(remote_id, remote_fase, remote_pressure, local_pressure)
        elif self.sync_strategy == 'PRIORITY':
            self.priority_sync(remote_id, remote_fase, remote_pressure, local_pressure)
        elif self.sync_strategy == 'BALANCED':
            self.balanced_sync(remote_id, remote_fase, remote_pressure, local_pressure)
        elif self.sync_strategy == 'GREEN_WAVE':
            self.green_wave_sync(remote_id, remote_fase, remote_pressure, local_pressure)

        self.update_coordination_score(remote_fase, remote_pressure, local_pressure)

    def calculate_traffic_pressure(self, cameras_data):
        total = 0
        active = 0
        for cd in cameras_data.values():
            if cd.get('status') == 'active':
                active += 1
                v = cd.get('vehicles', {})
                if isinstance(v, dict):
                    total += v.get('main', 0) + v.get('arrow', 0)
                else:
                    total += v
        if active == 0: return 0.0
        return min(1.0, (total / active) / 20.0)

    def adaptive_sync(self, rid, rfase, rpres, lpres):
        diff = lpres - rpres
        if abs(diff) < 0.2: return
        adj = 0
        if diff > 0.3:
            adj = self.SYNC_MAX_ADJUSTMENT
        elif diff > 0.2:
            adj = self.SYNC_MIN_ADJUSTMENT + 2
        elif diff < -0.3:
            adj = -self.SYNC_MAX_ADJUSTMENT
        elif diff < -0.2:
            adj = -(self.SYNC_MIN_ADJUSTMENT + 2)

        if adj != 0 and self.current_phase in [0, 1] and rfase in [0, 1]:
            self.apply_phase_adjustment(adj, "ADAPTIVE")

    def priority_sync(self, rid, rfase, rpres, lpres):
        we_priority = NODE_ID < rid
        if we_priority and lpres > 0.3:
            self.apply_phase_adjustment(5, "PRIORITY_MAIN")
        elif not we_priority and rpres > 0.4:
            self.apply_phase_adjustment(-3, "PRIORITY_YIELD")

    def balanced_sync(self, rid, rfase, rpres, lpres):
        total = lpres + rpres
        if total < 0.3: return
        share = lpres / total
        if share > 0.6:
            self.apply_phase_adjustment(int(self.SYNC_MAX_ADJUSTMENT * (share - 0.5) * 2), "BALANCE_LOCAL")
        elif share < 0.4:
            self.apply_phase_adjustment(-int(self.SYNC_MAX_ADJUSTMENT * (0.5 - share) * 2), "BALANCE_REMOTE")

    def green_wave_sync(self, rid, rfase, rpres, lpres):
        # Simplificado para brevedad: Detecta flujo E-O y anticipa
        if rfase in [0, 1] and self.current_phase not in [0, 1]:
            self.apply_phase_adjustment(-5, "GREEN_WAVE_ANTICIPATION")

    def apply_phase_adjustment(self, seconds, reason):
        current_t = cfg.PHASE_TIMES[self.current_phase]
        seconds = max(-self.SYNC_MAX_ADJUSTMENT, min(self.SYNC_MAX_ADJUSTMENT, seconds))
        self.phase_start_time -= seconds  # Alargar o acortar fase actual
        self.optimization_count += 1
        print(f"[SYNC] Ajuste {seconds}s ({reason})")

    def update_coordination_score(self, rfase, rpres, lpres):
        score = 100.0
        if self.current_phase in [0, 1] and rfase in [0, 1] and lpres > 0.5 and rpres > 0.5:
            score -= 30
            self.conflict_count += 1
        self.coordination_score = (self.coordination_score * 0.9 + score * 0.1)

    def set_sync_strategy(self, strat):
        self.sync_strategy = strat
        self.send_telemetry('STRATEGY_CHANGE', {'new_strategy': strat})

    # ==========================================
    # LÓGICA DE DETECCIÓN E INCIDENTES (ORIGINAL RESTAURADA)
    # ==========================================

    def handle_incident_log(self, channel, vehicle_id, duration, incident_type, frame_copy, position):
        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        timestamp_pretty = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            cam_idx = cfg.CAMERA_CHANNELS.index(channel)
            cam_name = cfg.CAMERA_NAMES[cam_idx]
        except:
            cam_name = f"Cam_{channel}"

        self.stats_manager.log_incident(cam_name)
        cx, cy = position
        cv2.circle(frame_copy, (cx, cy), 40, (0, 0, 255), 3)
        cv2.putText(frame_copy, f"INCIDENTE: {incident_type.upper()}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (0, 0, 255), 3)

        folder = "evidencias"
        os.makedirs(folder, exist_ok=True)
        filename = f"{folder}/{incident_type}_{cam_name}_ID{vehicle_id}_{timestamp_str}.jpg"
        cv2.imwrite(filename, frame_copy)

        # Webhook
        webhook_url = "https://n8n.trazo.xyz/webhook/sendMessageImg"
        payload = {"tipo_evento": "ALERTA_TRAFICO", "camara": cam_name, "mensaje": f"{incident_type} detectado"}
        try:
            with open(filename, 'rb') as f:
                requests.post(webhook_url, data=payload, files={'imagen_evidencia': f})
        except:
            pass

        self.send_telemetry('INCIDENT', {'incident_data': {'type': incident_type, 'cam': cam_name}})

    def trigger_alert(self, channel, vehicle_id, duration, incident_type, frame):
        frame_copy = frame.copy()
        try:
            pos = self.vehicle_data[channel][vehicle_id]['last_pos']
            t = threading.Thread(target=self.handle_incident_log,
                                 args=(channel, vehicle_id, duration, incident_type, frame_copy, pos))
            t.daemon = True
            t.start()
        except:
            pass

    def update_vehicle_status(self, channel, tracked_objects, main_light, arrow_light, main_zone, arrow_zone, frame):
        current_time = time.time()
        active_ids = []
        for obj in tracked_objects:
            x, y, x2, y2, vid = obj
            cx, cy = (x + x2) // 2, (y + y2) // 2
            active_ids.append(vid)

            if vid not in self.vehicle_data[channel]:
                self.vehicle_data[channel][vid] = {'last_pos': (cx, cy), 'accumulated_time': 0.0,
                                                   'last_update_time': current_time, 'incident_type': 'none',
                                                   'alert_sent': False}
            else:
                data = self.vehicle_data[channel][vid]
                dist = math.hypot(cx - data['last_pos'][0], cy - data['last_pos'][1])
                dt = current_time - data['last_update_time']

                is_arrow = self.detector.is_valid_detection(cx, cy, [arrow_zone])
                rel_light = arrow_light if is_arrow else main_light

                if dist > self.STOP_THRESHOLD:
                    data['accumulated_time'] = 0.0;
                    data['incident_type'] = 'none';
                    data['alert_sent'] = False
                else:
                    if rel_light == 'green':
                        data['accumulated_time'] += dt
                        if data['accumulated_time'] > self.ACCIDENT_TIME:
                            if data['incident_type'] == 'none': data['incident_type'] = 'breakdown'
                            if not data['alert_sent']:
                                self.trigger_alert(channel, vid, data['accumulated_time'], data['incident_type'], frame)
                                data['alert_sent'] = True
                data['last_pos'] = (cx, cy);
                data['last_update_time'] = current_time

        for vid in list(self.vehicle_data[channel].keys()):
            if vid not in active_ids: del self.vehicle_data[channel][vid]

    def check_collisions(self, channel):
        stopped = [(v, d['last_pos']) for v, d in self.vehicle_data[channel].items() if
                   d['accumulated_time'] > self.ACCIDENT_TIME]
        for i in range(len(stopped)):
            for j in range(i + 1, len(stopped)):
                if math.hypot(stopped[i][1][0] - stopped[j][1][0],
                              stopped[i][1][1] - stopped[j][1][1]) < self.COLLISION_DIST:
                    self.vehicle_data[channel][stopped[i][0]]['incident_type'] = 'collision'
                    self.vehicle_data[channel][stopped[j][0]]['incident_type'] = 'collision'

    def process_camera(self, channel, frame):
        self.last_frame_time[channel] = time.time()
        if self.system_mode[channel] != 'INTELLIGENT': return frame

        if self.frame_counter % self.detection_interval == 0:
            h, w = frame.shape[:2]
            small = cv2.resize(frame, (int(w * 0.4), int(h * 0.4)))
            bboxes = self.detector.detect(small)
            rects = (bboxes / 0.4).astype(int).tolist() if len(bboxes) > 0 else []
            tracked = self.trackers[channel].update(rects)
            self.last_detections[channel] = tracked

            try:
                self.stats_manager.update_flow(cfg.CAMERA_NAMES[cfg.CAMERA_CHANNELS.index(channel)],
                                               self.trackers[channel].id_count)
            except:
                pass

            ml = self.traffic_states[channel];
            al = self.arrow_states[channel]
            mz = self.live_zones[channel]['main'];
            az = self.live_zones[channel]['arrow']
            self.update_vehicle_status(channel, tracked, ml, al, mz, az, frame)
            self.check_collisions(channel)

            cm = sum(
                1 for o in tracked if self.detector.is_valid_detection((o[0] + o[2]) // 2, (o[1] + o[3]) // 2, [mz]))
            ca = sum(
                1 for o in tracked if self.detector.is_valid_detection((o[0] + o[2]) // 2, (o[1] + o[3]) // 2, [az]))
            self.detection_counts[channel] = {'main': cm, 'arrow': ca}

        # Dibujar rectangulos (feedback visual sobre frame)
        for obj in self.last_detections[channel]:
            x, y, x2, y2, vid = obj
            itype = self.vehicle_data[channel].get(vid, {}).get('incident_type', 'none')
            col = (0, 0, 255) if itype != 'none' else (255, 0, 0)
            cv2.rectangle(frame, (x, y), (x2, y2), col, 2)
            if itype != 'none': cv2.putText(frame, itype.upper(), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        return frame

    # ==========================================
    # CONTROL DE SEMÁFOROS (ORIGINAL RESTAURADO)
    # ==========================================
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if time.time() - self.click_cooldown < 0.3: return
            self.click_cooldown = time.time()
            if not self.is_editing:
                if x > 960: return
                col, row = (0 if x < 480 else 1), (0 if y < 360 else 1)
                idx = row * 2 + col
                if idx < len(cfg.CAMERA_CHANNELS):
                    self.edit_channel = cfg.CAMERA_CHANNELS[idx];
                    self.edit_points = [];
                    self.is_editing = True
            else:
                if x < 640: self.edit_points.append([x, y])

    def has_vehicles(self, channel, type='any'):
        if self.system_mode[channel] != 'INTELLIGENT': return True
        cnt = self.detection_counts[channel]
        return cnt['arrow'] > 0 if type == 'arrow' else (cnt['main'] > 0 or cnt['arrow'] > 0)

    def should_skip_phase(self, phase):
        if phase == 0:
            return not (self.has_vehicles(cfg.CAMERA_CHANNELS[cfg.ESTE_IDX], 'arrow') or self.has_vehicles(
                cfg.CAMERA_CHANNELS[cfg.OESTE_IDX], 'arrow'))
        elif phase == 2:
            return not self.has_vehicles(cfg.CAMERA_CHANNELS[cfg.NORTE_IDX])
        elif phase == 3:
            return not self.has_vehicles(cfg.CAMERA_CHANNELS[cfg.SUR_IDX])
        return False

    def set_lights(self, phase, color='green'):
        for ch in cfg.CAMERA_CHANNELS: self.traffic_states[ch] = 'red'; self.arrow_states[ch] = 'red'
        if color != 'red':
            if phase == 0:
                self.arrow_states[cfg.CAMERA_CHANNELS[cfg.ESTE_IDX]] = color
                self.arrow_states[cfg.CAMERA_CHANNELS[cfg.OESTE_IDX]] = color
            elif phase == 1:
                self.traffic_states[cfg.CAMERA_CHANNELS[cfg.ESTE_IDX]] = color
                self.traffic_states[cfg.CAMERA_CHANNELS[cfg.OESTE_IDX]] = color
            elif phase == 2:
                self.traffic_states[cfg.CAMERA_CHANNELS[cfg.NORTE_IDX]] = color
            elif phase == 3:
                self.traffic_states[cfg.CAMERA_CHANNELS[cfg.SUR_IDX]] = color

    def intelligent_control(self):
        while self.running:
            with self.sequence_lock:
                if not any(self.system_mode[ch] == 'INTELLIGENT' for ch in cfg.CAMERA_CHANNELS):
                    time.sleep(1);
                    continue
                elapsed = time.time() - self.phase_start_time
                if elapsed >= cfg.PHASE_TIMES[self.current_phase] - cfg.YELLOW_TIME:
                    if elapsed < cfg.PHASE_TIMES[self.current_phase]:
                        self.set_lights(self.current_phase, 'yellow')
                    else:
                        next_ph = (self.current_phase + 1) % 4
                        skipped = 0
                        while self.should_skip_phase(next_ph) and skipped < 4:
                            print(f"⏭️ Saltando fase {next_ph}");
                            next_ph = (next_ph + 1) % 4;
                            skipped += 1
                        if skipped == 4:
                            if self.current_phase != 1: self.current_phase = 1; self.set_lights(1, 'green')
                        else:
                            self.current_phase = next_ph; self.set_lights(next_ph, 'green')
                        self.phase_start_time = time.time()
            time.sleep(0.5)

    def standard_control(self):
        durations = [cfg.PHASE_TIMES[i] for i in range(4)]
        start_t = time.time();
        curr_ph = 0
        while self.running:
            std_cams = [ch for ch in cfg.CAMERA_CHANNELS if self.system_mode[ch] in ['STANDARD', 'FALLBACK']]
            if not std_cams: time.sleep(1); continue
            elapsed = time.time() - start_t
            if elapsed >= durations[curr_ph]:
                curr_ph = (curr_ph + 1) % 4;
                start_t = time.time()

            is_yellow = elapsed >= (durations[curr_ph] - cfg.YELLOW_TIME)
            for ch in std_cams:
                idx = cfg.CAMERA_CHANNELS.index(ch)
                tc, ac = 'red', 'red'
                if curr_ph == 0 and idx in [cfg.ESTE_IDX, cfg.OESTE_IDX]:
                    ac = 'yellow' if is_yellow else 'green'
                elif curr_ph == 1 and idx in [cfg.ESTE_IDX, cfg.OESTE_IDX]:
                    tc = 'yellow' if is_yellow else 'green'
                elif curr_ph == 2 and idx == cfg.NORTE_IDX:
                    tc = 'yellow' if is_yellow else 'green'
                elif curr_ph == 3 and idx == cfg.SUR_IDX:
                    tc = 'yellow' if is_yellow else 'green'
                self.traffic_states[ch] = tc;
                self.arrow_states[ch] = ac
            time.sleep(0.2)

    def monitor_cameras(self):
        while self.running:
            now = time.time()
            for ch in cfg.CAMERA_CHANNELS:
                if self.camera_status[ch] == 'active' and (now - self.last_frame_time[ch] > cfg.CAMERA_TIMEOUT):
                    print(f"⚠️ Timeout {ch}");
                    self.camera_status[ch] = 'failed';
                    self.system_mode[ch] = 'STANDARD';
                    self.attempt_reconnect(ch)
            time.sleep(2)

    def attempt_reconnect(self, channel):
        try:
            self.cameras[channel].release()
            cap = cv2.VideoCapture(channel)
            if cap.isOpened(): self.cameras[channel] = cap; self.camera_status[channel] = 'active'
        except:
            pass

    def initialize_cameras(self):
        print("\n=== CONECTANDO CAMARAS ===")
        for ch in cfg.CAMERA_CHANNELS:
            try:
                cap = cv2.VideoCapture(ch)
                if cap.isOpened():
                    self.cameras[ch] = cap; self.camera_status[ch] = 'active'
                else:
                    self.camera_status[ch] = 'failed'; self.system_mode[ch] = 'STANDARD'
            except:
                self.camera_status[ch] = 'failed'; self.system_mode[ch] = 'STANDARD'

    # ==========================================
    # LOOP PRINCIPAL LIMPIO
    # ==========================================
    def run(self):
        print(f"=== AITRAFFIC SYSTEM: {NODE_ID} ===")
        self.initialize_cameras()
        window_name = f'Sistema AITRAFFIC - {NODE_ID}'
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.mouse_callback)

        while True:
            self.frame_counter += 1
            self.stats_manager.check_periodic_save()
            if self.frame_counter % 30 == 0: self.send_telemetry()

            if self.is_editing and self.edit_channel in self.cameras:
                ret, raw = self.cameras[self.edit_channel].read()
                if ret:
                    edit_frame = vis.draw_edit_mode(raw.copy(), self.edit_points, f"EDIT: {self.edit_channel}",
                                                    self.edit_zone_type)
                    cv2.imshow(window_name, edit_frame)
                k = cv2.waitKey(1) & 0xFF
                if k == 27:
                    self.is_editing = False
                elif k == ord('s'):
                    self.live_zones[self.edit_channel][self.edit_zone_type] = np.array(self.edit_points)
                    self.is_editing = False
            else:
                frames_list = []
                for i, ch in enumerate(cfg.CAMERA_CHANNELS):
                    frame = np.zeros((360, 480, 3), dtype=np.uint8)
                    if ch in self.cameras and self.cameras[ch].isOpened():
                        ret, raw = self.cameras[ch].read()
                        if ret: frame = self.process_camera(ch, raw)

                    state = {
                        'mode': self.system_mode[ch], 'status': self.camera_status[ch],
                        'traffic_color': self.traffic_states[ch], 'arrow_color': self.arrow_states[ch],
                        'counts': self.detection_counts[ch],
                        'zones': (self.live_zones[ch]['main'], self.live_zones[ch]['arrow'])
                    }
                    frame = vis.add_overlay(frame, ch, cfg.CAMERA_NAMES[i], i, state)
                    frames_list.append(cv2.resize(frame, (480, 360)))

                top = np.hstack([frames_list[0], frames_list[1]])
                bot = np.hstack([frames_list[2], frames_list[3]])
                grid = np.vstack([top, bot])

                dash_info = {
                    'phase_idx': self.current_phase,
                    'active_cams': sum(1 for s in self.camera_status.values() if s == 'active'),
                    'intelligent_cams': sum(1 for m in self.system_mode.values() if m == 'INTELLIGENT')
                }

                sync_packet = {
                    'enabled': self.sync_enabled,
                    'connection_status': self.connection_status,
                    'msg_sent': self.messages_sent,
                    'msg_recv': self.messages_received,
                    'remote_data': self.remote_data,
                    'strategy': self.sync_strategy,
                    'score': self.coordination_score,
                    'optimizations': self.optimization_count,
                    'conflicts': self.conflict_count
                }

                final_view = vis.draw_dashboard(grid, dash_info, self.stats_manager.get_dashboard_data(), sync_packet)
                cv2.imshow(window_name, final_view)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('1'):
                    self.set_sync_strategy('ADAPTIVE')
                elif key == ord('2'):
                    self.set_sync_strategy('PRIORITY')
                elif key == ord('3'):
                    self.set_sync_strategy('BALANCED')
                elif key == ord('4'):
                    self.set_sync_strategy('GREEN_WAVE')
                elif key == ord('0'):
                    self.sync_enabled = not self.sync_enabled

        self.running = False
        cv2.destroyAllWindows()
        self.zmq_context.term()


if __name__ == '__main__':
    TrafficLightSystem().run()
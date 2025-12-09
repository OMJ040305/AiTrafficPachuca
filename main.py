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
# CONFIGURACIÓN DE RED (MODIFICAR AQUI)
# ==========================================
VPS_IP = "213.199.32.40"  # <--- PON AQUI LA IP PÚBLICA DE TU VPS
NODE_ID = "INTERSECCION_A"  # <--- CAMBIAR A "INTERSECCION_B" EN LA OTRA PC


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

        # --- RASTREO Y DETECCIÓN DE INCIDENTES ---
        self.trackers = {ch: EuclideanDistTracker() for ch in cfg.CAMERA_CHANNELS}
        self.vehicle_data = {ch: {} for ch in cfg.CAMERA_CHANNELS}

        # --- GESTOR DE ESTADÍSTICAS ---
        self.stats_manager = StatsManager()

        # --- CONFIGURACIÓN DE INCIDENTES ---
        self.STOP_THRESHOLD = 15
        self.ACCIDENT_TIME = 20.0
        self.COLLISION_DIST = 150

        # Caché visual
        self.last_detections = {ch: [] for ch in cfg.CAMERA_CHANNELS}
        self.frame_counter = 0
        self.detection_interval = 5

        # --- GESTIÓN DE ZONAS EN VIVO ---
        self.live_zones = {}
        for i, ch in enumerate(cfg.CAMERA_CHANNELS):
            mz, az = cfg.get_zones(i)
            self.live_zones[ch] = {
                'main': mz[0] if len(mz) > 0 else [],
                'arrow': az[0] if len(az) > 0 else []
            }

        # Variables de Edición
        self.is_editing = False
        self.edit_channel = None
        self.edit_zone_type = 'main'
        self.edit_points = []
        self.click_cooldown = 0

        # Detector
        self.detector = VehicleDetector()

        # Control de secuencia
        self.current_phase = 0
        self.phase_start_time = time.time()
        self.sequence_lock = threading.Lock()
        self.running = True

        # --- SISTEMA DE COMUNICACIÓN MEJORADO ---
        self.remote_data = {}
        self.zmq_context = zmq.Context()

        # Estadísticas de conexión
        self.connection_status = 'DISCONNECTED'
        self.last_heartbeat = time.time()
        self.heartbeat_interval = 2.0
        self.messages_sent = 0
        self.messages_received = 0
        self.connection_attempts = 0
        self.vps_last_seen = time.time()
        self.system_start_time = time.time()

        # Socket para ENVIAR
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

        # Iniciar threads
        for t in self.threads:
            t.start()

        # Esperar establecer conexión y notificar inicio
        time.sleep(1)
        self.send_telemetry(msg_type='SYSTEM_START', extra_data={
            'version': '2.0',
            'capabilities': ['INTELLIGENT_CONTROL', 'INCIDENT_DETECTION', 'SYNC'],
            'config': {
                'cameras': len(cfg.CAMERA_CHANNELS),
                'detection_interval': self.detection_interval,
                'accident_time': self.ACCIDENT_TIME
            }
        })

    # ==========================================
    # SISTEMA DE RED MEJORADO
    # ==========================================

    def network_listener(self):
        """Escucha datos del VPS con reconexión automática"""
        subscriber = self.zmq_context.socket(zmq.SUB)
        retry_delay = 5

        while self.running:
            try:
                if self.connection_status != 'CONNECTED':
                    print(f"[RED] 🔄 Intentando conectar al VPS ({VPS_IP})...")
                    subscriber.connect(f"tcp://{VPS_IP}:5556")
                    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
                    subscriber.setsockopt(zmq.RCVTIMEO, 5000)
                    self.connection_status = 'CONNECTED'
                    self.connection_attempts += 1
                    print(f"[RED] ✅ Conectado al VPS (intento #{self.connection_attempts})")

                    # Enviar notificación de conexión
                    self.send_telemetry(msg_type='CONNECTION', extra_data={
                        'status': 'CONNECTED',
                        'attempt': self.connection_attempts
                    })

                try:
                    msg = subscriber.recv_json(flags=zmq.NOBLOCK)
                    self.vps_last_seen = time.time()
                    self.messages_received += 1

                    # Ignorar mensajes propios
                    if msg.get('node_id') == NODE_ID:
                        continue

                    msg_type = msg.get('type', 'STATE_UPDATE')
                    sender_id = msg.get('node_id', 'unknown')

                    # Procesar según tipo
                    if msg_type == 'STATE_UPDATE':
                        self.remote_data[sender_id] = msg
                        self.sync_with_remote(sender_id, msg)

                    elif msg_type == 'INCIDENT':
                        print(f"[RED] 🚨 Incidente reportado en {sender_id}")
                        incident_data = msg.get('incident_data', {})
                        print(f"[RED]    └─ {incident_data.get('tipo_incidente')} en {incident_data.get('camara')}")
                        self.remote_data[sender_id] = msg

                    elif msg_type == 'CONNECTION_LOST':
                        lost_node = msg.get('node_id')
                        print(f"[RED] ⚠️ Nodo {lost_node} desconectado")
                        if lost_node in self.remote_data:
                            del self.remote_data[lost_node]

                    elif msg_type == 'SYSTEM_SUMMARY':
                        active = msg.get('active_nodes', [])
                        print(f"[RED] 📊 Nodos activos: {', '.join(active)}")

                    elif msg_type == 'HEARTBEAT':
                        # Actualizar última vez vista
                        self.remote_data[sender_id] = msg

                except zmq.Again:
                    # No hay mensajes, verificar timeout
                    if time.time() - self.vps_last_seen > 15:
                        print("[RED] ⚠️ Sin respuesta del VPS (15s)")
                        self.connection_status = 'TIMEOUT'
                        raise Exception("VPS timeout")
                    time.sleep(0.1)

            except Exception as e:
                self.connection_status = 'DISCONNECTED'
                print(f"[RED] ❌ Error de conexión: {e}")
                print(f"[RED] ⏳ Reintentando en {retry_delay}s...")
                time.sleep(retry_delay)
                subscriber.close()
                subscriber = self.zmq_context.socket(zmq.SUB)

    def send_telemetry(self, msg_type='STATE_UPDATE', extra_data=None):
        """Envía datos al VPS con información completa"""
        try:
            # Convertir claves a strings para JSON
            luces_str = {str(k): v for k, v in self.traffic_states.items()}
            flechas_str = {str(k): v for k, v in self.arrow_states.items()}

            # Calcular estadísticas
            total_vehicles = sum(self.trackers[ch].id_count for ch in cfg.CAMERA_CHANNELS)
            active_cams = sum(1 for s in self.camera_status.values() if s == 'active')

            payload = {
                "type": msg_type,
                "node_id": NODE_ID,
                "timestamp": time.time(),
                "connection_status": self.connection_status,

                # Estado del sistema
                "fase": self.current_phase,
                "luces": luces_str,
                "flechas": flechas_str,

                # Estadísticas
                "stats": {
                    "total_vehicles": total_vehicles,
                    "active_cameras": active_cams,
                    "messages_sent": self.messages_sent,
                    "messages_received": self.messages_received,
                    "uptime": int(time.time() - self.system_start_time)
                },

                # Estado de cámaras
                "cameras": {
                    str(ch): {
                        "status": self.camera_status[ch],
                        "mode": self.system_mode[ch],
                        "vehicles": self.detection_counts[ch]
                    } for ch in cfg.CAMERA_CHANNELS
                }
            }

            # Agregar datos extra si los hay
            if extra_data:
                payload.update(extra_data)

            self.publisher.send_json(payload)
            self.messages_sent += 1
            return True

        except Exception as e:
            print(f"[RED] ❌ Error enviando telemetría: {e}")
            return False

    def sync_with_remote(self, remote_id, remote_data):
        """Sincroniza fases con intersección remota"""
        try:
            remote_fase = remote_data.get('fase', 0)
            remote_luces = remote_data.get('luces', {})
            remote_stats = remote_data.get('stats', {})

            # Lógica de sincronización simple:
            # Si la otra intersección tiene mucho tráfico, podríamos ajustar tiempos
            remote_vehicles = remote_stats.get('total_vehicles', 0)

            # Ejemplo: Si ambos están en fase 1 (rectos E-O) y el remoto tiene
            # significativamente más tráfico, podríamos considerar ajustes
            if remote_fase == 1 and self.current_phase == 1:
                # Aquí puedes implementar lógica de coordinación
                pass

            # Detectar si hay conflicto potencial
            if remote_fase == self.current_phase:
                # Ambos en la misma fase - esto es normal para E-O
                pass

        except Exception as e:
            print(f"[SYNC] Error sincronizando con {remote_id}: {e}")

    def heartbeat_sender(self):
        """Envía señal de vida periódica al VPS"""
        while self.running:
            try:
                if time.time() - self.last_heartbeat >= self.heartbeat_interval:
                    self.send_telemetry(msg_type='HEARTBEAT')
                    self.last_heartbeat = time.time()
            except Exception as e:
                print(f"[HEARTBEAT] Error: {e}")

            time.sleep(1)

    # ==========================================
    # GESTIÓN DE EVIDENCIAS Y WEBHOOK
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
        label_top = f"INCIDENTE: {incident_type.upper()}"
        label_bot = f"ID: {vehicle_id} | {int(duration)}s DETENIDO"

        cv2.putText(frame_copy, label_top, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        cv2.putText(frame_copy, label_bot, (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame_copy, timestamp_pretty, (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        folder = "evidencias"
        os.makedirs(folder, exist_ok=True)
        filename = f"{folder}/{incident_type}_{cam_name}_ID{vehicle_id}_{timestamp_str}.jpg"
        cv2.imwrite(filename, frame_copy)
        print(f"[ALERTA] 📸 Evidencia guardada en disco: {filename}")

        webhook_url = "https://n8n.trazo.xyz/webhook/sendMessageImg"
        msj_intro = "⚠️ ALERTA DE TRAFICO"
        if incident_type == 'breakdown':
            msj_intro = "⚠️ POSIBLE VEHICULO AVERIADO"
        elif incident_type == 'collision':
            msj_intro = "💥 POSIBLE CHOQUE"

        payload = {
            "tipo_evento": "ALERTA_TRAFICO",
            "camara": cam_name,
            "tipo_incidente": incident_type,
            "id_vehiculo": str(vehicle_id),
            "duracion_detenido": f"{int(duration)} segundos",
            "fecha_hora": timestamp_pretty,
            "mensaje": f"{msj_intro} en {cam_name}. Vehiculo ID {vehicle_id} detenido {int(duration)}s."
        }

        try:
            with open(filename, 'rb') as img_file:
                files_data = {'imagen_evidencia': (os.path.basename(filename), img_file, 'image/jpeg')}
                response = requests.post(webhook_url, data=payload, files=files_data, timeout=15)
            if response.status_code == 200:
                print(f"[N8N] ✅ Alerta enviada.")
        except Exception as e:
            print(f"[N8N] ❌ Error subiendo imagen: {e}")

        # Notificar al VPS sobre el incidente
        self.send_telemetry(msg_type='INCIDENT', extra_data={
            'incident_data': {
                'camara': cam_name,
                'tipo_incidente': incident_type,
                'id_vehiculo': vehicle_id,
                'duracion': duration,
                'timestamp': timestamp_pretty,
                'position': position
            }
        })

    def trigger_alert(self, channel, vehicle_id, duration, incident_type, frame):
        frame_copy = frame.copy()
        try:
            pos = self.vehicle_data[channel][vehicle_id]['last_pos']
            t = threading.Thread(target=self.handle_incident_log,
                                 args=(channel, vehicle_id, duration, incident_type, frame_copy, pos))
            t.daemon = True
            t.start()
        except KeyError:
            pass

    # ==========================================
    # DETECCIÓN Y TRACKING
    # ==========================================

    def update_vehicle_status(self, channel, tracked_objects, main_light, arrow_light, main_zone, arrow_zone,
                              frame_for_evidence):
        current_time = time.time()
        active_ids = []

        for obj in tracked_objects:
            x, y, x2, y2, vid = obj
            cx, cy = (x + x2) // 2, (y + y2) // 2
            active_ids.append(vid)

            if vid not in self.vehicle_data[channel]:
                self.vehicle_data[channel][vid] = {
                    'last_pos': (cx, cy), 'accumulated_time': 0.0, 'last_update_time': current_time,
                    'incident_type': 'none', 'alert_sent': False, 'lane_type': 'unknown'
                }
            else:
                data = self.vehicle_data[channel][vid]
                dist = math.hypot(cx - data['last_pos'][0], cy - data['last_pos'][1])
                dt = current_time - data['last_update_time']

                is_in_arrow = self.detector.is_valid_detection(cx, cy, [arrow_zone])
                is_in_main = self.detector.is_valid_detection(cx, cy, [main_zone])

                relevant_light_color = arrow_light if is_in_arrow else main_light

                if dist > self.STOP_THRESHOLD:
                    data['accumulated_time'] = 0.0
                    data['incident_type'] = 'none'
                    data['alert_sent'] = False
                else:
                    if relevant_light_color == 'green':
                        data['accumulated_time'] += dt
                        if data['accumulated_time'] > self.ACCIDENT_TIME:
                            if data['incident_type'] == 'none':
                                data['incident_type'] = 'breakdown'
                            if not data['alert_sent']:
                                self.trigger_alert(channel, vid, data['accumulated_time'], data['incident_type'],
                                                   frame_for_evidence)
                                data['alert_sent'] = True

                data['last_pos'] = (cx, cy)
                data['last_update_time'] = current_time

        known = list(self.vehicle_data[channel].keys())
        for vid in known:
            if vid not in active_ids:
                del self.vehicle_data[channel][vid]

    def check_collisions(self, channel):
        stopped = []
        for vid, data in self.vehicle_data[channel].items():
            if data['accumulated_time'] > self.ACCIDENT_TIME:
                stopped.append((vid, data['last_pos']))

        n = len(stopped)
        if n >= 2:
            for i in range(n):
                for j in range(i + 1, n):
                    id1, p1 = stopped[i]
                    id2, p2 = stopped[j]
                    if math.hypot(p1[0] - p2[0], p1[1] - p2[1]) < self.COLLISION_DIST:
                        self.vehicle_data[channel][id1]['incident_type'] = 'collision'
                        self.vehicle_data[channel][id2]['incident_type'] = 'collision'

    def process_camera(self, channel, frame):
        self.last_frame_time[channel] = time.time()
        if self.system_mode[channel] != 'INTELLIGENT':
            return frame

        if self.frame_counter % self.detection_interval == 0:
            h, w = frame.shape[:2]
            scale_factor = 0.4
            small = cv2.resize(frame, (int(w * scale_factor), int(h * scale_factor)))
            bboxes = self.detector.detect(small)
            rects = (bboxes / scale_factor).astype(int).tolist() if len(bboxes) > 0 else []
            tracked_objects = self.trackers[channel].update(rects)
            self.last_detections[channel] = tracked_objects

            try:
                cam_idx = cfg.CAMERA_CHANNELS.index(channel)
                cam_name = cfg.CAMERA_NAMES[cam_idx]
                self.stats_manager.update_flow(cam_name, self.trackers[channel].id_count)
            except:
                pass

            curr_main_light = self.traffic_states[channel]
            curr_arrow_light = self.arrow_states[channel]
            main_zone = self.live_zones[channel]['main']
            arrow_zone = self.live_zones[channel]['arrow']

            self.update_vehicle_status(channel, tracked_objects, curr_main_light, curr_arrow_light, main_zone,
                                       arrow_zone, frame)
            self.check_collisions(channel)

            mz_list = [main_zone]
            az_list = [arrow_zone]
            cm, ca = 0, 0
            for obj in tracked_objects:
                cx, cy = (obj[0] + obj[2]) // 2, (obj[1] + obj[3]) // 2
                if self.detector.is_valid_detection(cx, cy, mz_list):
                    cm += 1
                if self.detector.is_valid_detection(cx, cy, az_list):
                    ca += 1
            self.detection_counts[channel] = {'main': cm, 'arrow': ca}

        for obj in self.last_detections[channel]:
            x, y, x2, y2, vid = obj
            v_data = self.vehicle_data[channel].get(vid, {})
            accum = v_data.get('accumulated_time', 0)
            itype = v_data.get('incident_type', 'none')

            if itype == 'breakdown' or accum > self.ACCIDENT_TIME:
                color = (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x2, y2), color, 4)
                cv2.putText(frame, "ALERTA", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            elif itype == 'collision':
                color = (255, 0, 255)
                cv2.rectangle(frame, (x, y), (x2, y2), color, 4)
                cv2.putText(frame, "CHOQUE", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            else:
                color = (255, 0, 0)
                cv2.rectangle(frame, (x, y), (x2, y2), color, 1)

        return frame

    # ==========================================
    # CONTROL DE SEMÁFOROS
    # ==========================================

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if time.time() - self.click_cooldown < 0.3:
                return
            self.click_cooldown = time.time()
            if not self.is_editing:
                if x > 960:
                    return
                col, row = (0 if x < 480 else 1), (0 if y < 360 else 1)
                idx = row * 2 + col
                if idx < len(cfg.CAMERA_CHANNELS):
                    self.edit_channel = cfg.CAMERA_CHANNELS[idx]
                    self.edit_points = []
                    self.is_editing = True
                    self.edit_zone_type = 'main'
            else:
                if x < 640:
                    self.edit_points.append([x, y])

    def has_vehicles(self, channel, type='any'):
        if self.system_mode[channel] != 'INTELLIGENT':
            return True
        cnt = self.detection_counts[channel]
        if type == 'arrow':
            return cnt['arrow'] > 0
        return cnt['main'] > 0 or cnt['arrow'] > 0

    def should_skip_phase(self, phase):
        if phase == 0:
            e, o = cfg.CAMERA_CHANNELS[cfg.ESTE_IDX], cfg.CAMERA_CHANNELS[cfg.OESTE_IDX]
            return not (self.has_vehicles(e, 'arrow') or self.has_vehicles(o, 'arrow'))
        elif phase == 2:
            return not self.has_vehicles(cfg.CAMERA_CHANNELS[cfg.NORTE_IDX])
        elif phase == 3:
            return not self.has_vehicles(cfg.CAMERA_CHANNELS[cfg.SUR_IDX])
        return False

    def set_lights(self, phase, color='green'):
        for ch in cfg.CAMERA_CHANNELS:
            self.traffic_states[ch] = 'red'
            self.arrow_states[ch] = 'red'
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
                    time.sleep(1)
                    continue
                elapsed = time.time() - self.phase_start_time
                if elapsed >= cfg.PHASE_TIMES[self.current_phase] - cfg.YELLOW_TIME:
                    if elapsed < cfg.PHASE_TIMES[self.current_phase]:
                        self.set_lights(self.current_phase, 'yellow')
                    else:
                        next_ph = (self.current_phase + 1) % 4
                        skipped = 0
                        while self.should_skip_phase(next_ph) and skipped < 4:
                            print(f"⏭️ Saltando fase {next_ph}")
                            next_ph = (next_ph + 1) % 4
                            skipped += 1
                        if skipped == 4:
                            if self.current_phase != 1:
                                self.current_phase = 1
                                self.set_lights(1, 'green')
                        else:
                            self.current_phase = next_ph
                            self.set_lights(next_ph, 'green')
                        self.phase_start_time = time.time()
            time.sleep(0.5)

    def standard_control(self):
        durations = [cfg.PHASE_TIMES[i] for i in range(4)]
        start_t = time.time()
        curr_ph = 0
        while self.running:
            std_cams = [ch for ch in cfg.CAMERA_CHANNELS if self.system_mode[ch] in ['STANDARD', 'FALLBACK']]
            if not std_cams:
                time.sleep(1)
                continue
            elapsed = time.time() - start_t
            total_ph_time = durations[curr_ph]
            is_yellow = elapsed >= (total_ph_time - cfg.YELLOW_TIME)
            if elapsed >= total_ph_time:
                curr_ph = (curr_ph + 1) % 4
                start_t = time.time()
                is_yellow = False
            for ch in std_cams:
                idx = cfg.CAMERA_CHANNELS.index(ch)
                t_color, a_color = 'red', 'red'
                if curr_ph == 0 and idx in [cfg.ESTE_IDX, cfg.OESTE_IDX]:
                    a_color = 'yellow' if is_yellow else 'green'
                elif curr_ph == 1 and idx in [cfg.ESTE_IDX, cfg.OESTE_IDX]:
                    t_color = 'yellow' if is_yellow else 'green'
                elif curr_ph == 2 and idx == cfg.NORTE_IDX:
                    t_color = 'yellow' if is_yellow else 'green'
                elif curr_ph == 3 and idx == cfg.SUR_IDX:
                    t_color = 'yellow' if is_yellow else 'green'
                self.traffic_states[ch] = t_color
                self.arrow_states[ch] = a_color
            time.sleep(0.2)

    # ==========================================
    # MONITOREO DE CÁMARAS
    # ==========================================

    def monitor_cameras(self):
        while self.running:
            now = time.time()
            for ch in cfg.CAMERA_CHANNELS:
                if self.camera_status[ch] == 'active' and (now - self.last_frame_time[ch] > cfg.CAMERA_TIMEOUT):
                    print(f"⚠️ Timeout en camara {ch}.")
                    self.camera_status[ch] = 'failed'
                    self.system_mode[ch] = 'STANDARD'
                    self.attempt_reconnect(ch)
            time.sleep(2)

    def attempt_reconnect(self, channel):
        try:
            if channel in self.cameras:
                self.cameras[channel].release()
            cap = cv2.VideoCapture(channel)
            if cap.isOpened():
                self.cameras[channel] = cap
                self.camera_status[channel] = 'active'
                print(f"[CÁMARA] ✅ Reconexión exitosa: {channel}")
        except Exception as e:
            print(f"[CÁMARA] ❌ Error reconectando {channel}: {e}")

    def initialize_cameras(self):
        print("\n=== CONECTANDO CÁMARAS ===")
        for i, ch in enumerate(cfg.CAMERA_CHANNELS):
            try:
                cap = cv2.VideoCapture(ch)
                if cap.isOpened():
                    self.cameras[ch] = cap
                    self.camera_status[ch] = 'active'
                    print(f"✅ Camara {ch} ({cfg.CAMERA_NAMES[i]}) OK.")
                else:
                    self.camera_status[ch] = 'failed'
                    self.system_mode[ch] = 'STANDARD'
                    print(f"❌ Camara {ch} ({cfg.CAMERA_NAMES[i]}) FALLO.")
            except Exception as e:
                self.camera_status[ch] = 'failed'
                self.system_mode[ch] = 'STANDARD'
                print(f"❌ Error en camara {ch}: {e}")

    # ==========================================
    # LOOP PRINCIPAL
    # ==========================================

    def run(self):
        print("╔" + "═" * 50 + "╗")
        print("║" + " 🚦 AITRAFFIC SYSTEM v2.0 ".center(50) + "║")
        print("╠" + "═" * 50 + "╣")
        print(f"║ 🆔 Nodo: {NODE_ID}".ljust(52) + "║")
        print(f"║ 🌐 VPS: {VPS_IP}".ljust(52) + "║")
        print(f"║ ⏰ Inicio: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}".ljust(52) + "║")
        print("╚" + "═" * 50 + "╝\n")

        self.initialize_cameras()
        window_name = f'Sistema AITRAFFIC - {NODE_ID}'
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.mouse_callback)

        print("\n🎬 Sistema iniciado. Presiona 'Q' para salir.\n")

        while True:
            self.frame_counter += 1
            self.stats_manager.check_periodic_save()

            # Enviar telemetría periódica
            if self.frame_counter % 30 == 0:  # Cada 30 frames (~1 segundo)
                self.send_telemetry(msg_type='STATE_UPDATE')

            if self.is_editing and self.edit_channel in self.cameras:
                ret, raw = self.cameras[self.edit_channel].read()
                if ret:
                    edit_frame = vis.draw_edit_mode(raw.copy(), self.edit_points,
                                                    f"EDITANDO: {self.edit_channel}",
                                                    self.edit_zone_type)
                    cv2.imshow(window_name, edit_frame)
                k = cv2.waitKey(1) & 0xFF
                if k == 27:  # ESC
                    self.is_editing = False
                elif k == ord('z'):
                    if self.edit_points:
                        self.edit_points.pop()
                elif k == ord('t'):
                    # Cambiar tipo de zona
                    self.edit_zone_type = 'arrow' if self.edit_zone_type == 'main' else 'main'
                    print(f"[EDICIÓN] Tipo de zona cambiado a: {self.edit_zone_type}")
                elif k == ord('s'):
                    self.live_zones[self.edit_channel][self.edit_zone_type] = np.array(self.edit_points)
                    print(f"[EDICIÓN] ✅ Zona guardada: {self.edit_zone_type}")
                    self.is_editing = False
            else:
                frames_list = []
                for i, ch in enumerate(cfg.CAMERA_CHANNELS):
                    frame = np.zeros((360, 480, 3), dtype=np.uint8)
                    camera_ok = False
                    if ch in self.cameras and self.cameras[ch].isOpened():
                        ret, raw = self.cameras[ch].read()
                        if ret:
                            frame = self.process_camera(ch, raw)
                            camera_ok = True

                    state = {
                        'mode': self.system_mode[ch],
                        'status': self.camera_status[ch],
                        'traffic_color': self.traffic_states[ch],
                        'arrow_color': self.arrow_states[ch],
                        'counts': self.detection_counts[ch],
                        'zones': (self.live_zones[ch]['main'], self.live_zones[ch]['arrow'])
                    }
                    frame = vis.add_overlay(frame, ch, cfg.CAMERA_NAMES[i], i, state)
                    frames_list.append(cv2.resize(frame, (480, 360)))

                top = np.hstack([frames_list[0], frames_list[1]])
                bot = np.hstack([frames_list[2], frames_list[3]])
                grid = np.vstack([top, bot])

                dashboard_info = {
                    'phase_idx': self.current_phase,
                    'active_cams': sum(1 for s in self.camera_status.values() if s == 'active'),
                    'intelligent_cams': sum(1 for m in self.system_mode.values() if m == 'INTELLIGENT')
                }
                stats_data = self.stats_manager.get_dashboard_data()
                final_view = vis.draw_dashboard(grid, dashboard_info, stats_data)

                # Visualizar datos de intersecciones remotas
                y_remote = 30
                for rid, rdata in sorted(self.remote_data.items()):
                    timestamp = rdata.get('timestamp', 0)
                    age = time.time() - timestamp

                    # Color según antigüedad del dato
                    if age < 5:
                        conn_status = "🟢"
                        color = (0, 255, 0)
                    elif age < 15:
                        conn_status = "🟡"
                        color = (0, 255, 255)
                    else:
                        conn_status = "🔴"
                        color = (0, 0, 255)

                    fase_remote = rdata.get('fase', '?')
                    stats_remote = rdata.get('stats', {})
                    vehicles = stats_remote.get('total_vehicles', 0)

                    text = f"{conn_status} {rid}: Fase {fase_remote} | {vehicles} vehiculos"
                    cv2.putText(final_view, text, (1000, y_remote),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    y_remote += 25

                # Mostrar estado de conexión VPS
                y_remote += 10
                vps_color = (0, 255, 0) if self.connection_status == 'CONNECTED' else (0, 0, 255)
                vps_status_icon = "🟢" if self.connection_status == 'CONNECTED' else "🔴"
                vps_text = f"{vps_status_icon} VPS: {self.connection_status}"
                cv2.putText(final_view, vps_text, (1000, y_remote),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, vps_color, 2)

                y_remote += 25
                msg_text = f"MSG: ↑{self.messages_sent} ↓{self.messages_received}"
                cv2.putText(final_view, msg_text, (1000, y_remote),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                cv2.imshow(window_name, final_view)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        # Limpieza al salir
        print("\n🛑 Deteniendo sistema...")
        self.running = False

        # Notificar al VPS que nos vamos
        self.send_telemetry(msg_type='SYSTEM_SHUTDOWN', extra_data={
            'reason': 'USER_STOP',
            'uptime': int(time.time() - self.system_start_time)
        })

        time.sleep(0.5)  # Dar tiempo a que se envíe el mensaje

        cv2.destroyAllWindows()
        for cap in self.cameras.values():
            cap.release()

        self.zmq_context.term()

        print("✅ Sistema detenido correctamente.\n")


if __name__ == '__main__':
    try:
        TrafficLightSystem().run()
    except KeyboardInterrupt:
        print("\n\n⚠️ Interrupción detectada. Cerrando...")
    except Exception as e:
        print(f"\n❌ Error fatal: {e}")
        import traceback

        traceback.print_exc()
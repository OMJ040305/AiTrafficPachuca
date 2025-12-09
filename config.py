import numpy as np

# --- CONFIGURACIÓN DE CÁMARAS ---
# Canales: Norte, Sur, Este, Oeste
CAMERA_CHANNELS = [4, 3, 2, 1]
CAMERA_NAMES = ["Camara Norte", "Camara Sur", "Camara Este", "Camara Oeste"]

# Índices
NORTE_IDX = 2
SUR_IDX = 3

ESTE_IDX = 0
OESTE_IDX = 1

# --- ZONAS DE DETECCIÓN ---
zonaRectoCamaraEste = np.array([[116, 115],
                                [2, 360],
                                [8, 469],
                                [360, 474],
                                [338, 124]])
zonaFlechaCamaraEste = np.array([[344, 207],
                                 [362, 472],
                                 [597, 473],
                                 [503, 283]])

zonaRectoCamaraOeste = np.array([[204, 207],
                                 [188, 350],
                                 [316, 344],
                                 [296, 203]])
zonaFlechaCamaraOeste = np.array([[381, 193],
                                  [433, 473],
                                  [630, 476],
                                  [608, 391],
                                  [546, 288]])

zonaCamaraNorte = np.array([[204, 207],
                            [188, 350],
                            [316, 344],
                            [296, 203]])
zonaCamaraSur = np.array([[240, 201],
                          [237, 343],
                          [345, 336],
                          [318, 199]])

# --- CONFIGURACIÓN DE SEMÁFORO ---
TRAFFIC_LIGHT_COLORS = {
    'red': (0, 0, 255),
    'yellow': (0, 255, 255),
    'green': (0, 255, 0)
}

SYSTEM_MODES = {
    'INTELLIGENT': 'Inteligente',
    'STANDARD': 'Estandar',
    'FALLBACK': 'Respaldo'
}

# Tiempos (segundos)
PHASE_TIMES = {
    0: 15,  # Flechas E-O
    1: 25,  # Rectos E-O
    2: 20,  # Norte
    3: 20  # Sur
}
YELLOW_TIME = 3
CAMERA_TIMEOUT = 20.0
MAX_FAILURES = 5


# Función helper para obtener zonas según el canal
def get_zones(channel_idx):
    if channel_idx == NORTE_IDX:
        return [zonaCamaraNorte], []
    elif channel_idx == SUR_IDX:
        return [zonaCamaraSur], []
    elif channel_idx == ESTE_IDX:
        return [zonaRectoCamaraEste], [zonaFlechaCamaraEste]
    elif channel_idx == OESTE_IDX:
        return [zonaRectoCamaraOeste], [zonaFlechaCamaraOeste]
    return [], []

"""Contrato comun entre la laptop (Python 3.12) y la Jetson (Python 3.6).

Define el formato de la observacion de la DQN, el espacio de acciones, la
inferencia de la red en numpy y la configuracion de percepcion. Solo depende de
numpy y debe seguir siendo valido en Python 3.6.

Convencion de angulos de ROS: radianes, 0 al frente, positivo a la izquierda.
"""
import json
import math
import os

import numpy as np

N_RAYOS = 24                    # sectores del LiDAR en la observacion
RANGO_MAX_LIDAR = 3.5           # metros; lecturas mayores o invalidas se recortan aqui
FOV_H_GRADOS = 58.4             # campo de vision horizontal de la Astra S
RANGO_PROFUNDIDAD = (0.4, 2.0)  # alcance util de la profundidad de la Astra S, metros
DIST_MAX_PERSONA = 5.0          # metros; normaliza la distancia a la persona
ANTIGUEDAD_MAX = 3.0            # segundos; normaliza la antiguedad de la estimacion

# Carpeta raiz del repositorio: las rutas relativas (models/, logs/) cuelgan de aqui
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Espacio de acciones: (v en m/s, w en rad/s). Indice = iv * 5 + iw
ACCIONES = [(v, w) for v in (0.0, 0.1, 0.2) for w in (-0.5, -0.25, 0.0, 0.25, 0.5)]

CONFIG_PERCEPCION = {
    "nombre": "yolov8n_320",
    "onnx": "models/yolov8n_320.onnx",
    "imgsz": 320,
    "confianza_min": 0.4,
    "cada_n_cuadros": 1,
}

PERFIL_IDEAL = {
    "sigma_distancia": 0.0,   # metros
    "sigma_angulo": 0.0,      # radianes
    "prob_perdida": 0.0,      # probabilidad de no detectar a una persona que si se ve
    "latencia": 0.0,          # segundos
    "confianza_media": 1.0,
}


def ruta_repo(ruta):
    """Ruta absoluta: las relativas se toman desde la raiz del repositorio."""
    return ruta if os.path.isabs(ruta) else os.path.join(RAIZ, ruta)


def reducir_scan(angle_min, angle_increment, ranges):
    """Reduce un LaserScan a N_RAYOS sectores normalizados a [0, 1].

    El sector 0 esta centrado al frente y los siguientes avanzan a la izquierda
    (sentido antihorario), cada uno de 360 / N_RAYOS grados. Cada sector toma la
    lectura valida mas cercana. inf, nan, None (asi llega inf por rosbridge) y 0
    cuentan como RANGO_MAX_LIDAR.
    """
    rangos = np.array(ranges, dtype=np.float64)
    rangos[~np.isfinite(rangos)] = RANGO_MAX_LIDAR
    rangos[rangos <= 0] = RANGO_MAX_LIDAR
    rangos = np.minimum(rangos, RANGO_MAX_LIDAR)
    angulos = angle_min + np.arange(len(rangos)) * angle_increment
    ancho = 2.0 * math.pi / N_RAYOS
    sector = np.floor(angulos / ancho + 0.5).astype(np.int64) % N_RAYOS
    rayos = np.full(N_RAYOS, RANGO_MAX_LIDAR)
    np.minimum.at(rayos, sector, rangos)
    return (rayos / RANGO_MAX_LIDAR).astype(np.float32)


def distancia_en_angulo(angle_min, angle_increment, ranges, angulo, cono_grados=3):
    """Mediana de las lecturas validas a +-cono_grados de angulo, en metros, o None.

    Respaldo del LiDAR cuando la profundidad de la camara no da distancia.
    """
    rangos = np.array(ranges, dtype=np.float64)
    rangos[~np.isfinite(rangos)] = 0.0
    angulos = angle_min + np.arange(len(rangos)) * angle_increment
    diferencia = np.arctan2(np.sin(angulos - angulo), np.cos(angulos - angulo))
    validos = (rangos > 0) & (np.abs(diferencia) <= math.radians(cono_grados))
    if not validos.any():
        return None
    return float(np.median(rangos[validos]))


def persona_vacia():
    """Estimacion de persona cuando nunca se ha detectado a nadie.

    Campos del dict de persona (igual en la Jetson y en la simulacion):
      visible     1 si se detecto en la ultima medicion, 0 si no
      distancia   metros desde el robot (ultima conocida si visible es 0)
      angulo      radianes, positivo a la izquierda (ultimo conocido si visible es 0)
      confianza   confianza del detector, 0 si visible es 0
      antiguedad  segundos desde que se capturo el ultimo cuadro donde se vio
    """
    return {"visible": 0, "distancia": DIST_MAX_PERSONA, "angulo": 0.0,
            "confianza": 0.0, "antiguedad": ANTIGUEDAD_MAX}


def observacion(scan_reducido, persona):
    """Vector de entrada de la DQN, np.float32 de largo N_RAYOS + 6.

    Orden:
      0 .. N_RAYOS-1  rayos del LiDAR de reducir_scan (sector 0 al frente)
      N_RAYOS + 0     visible (0 o 1)
      N_RAYOS + 1     min(distancia, DIST_MAX_PERSONA) / DIST_MAX_PERSONA
      N_RAYOS + 2     sin(angulo)
      N_RAYOS + 3     cos(angulo)
      N_RAYOS + 4     confianza
      N_RAYOS + 5     min(antiguedad, ANTIGUEDAD_MAX) / ANTIGUEDAD_MAX
    Distancia y angulo son la ultima estimacion conocida aunque visible sea 0.
    """
    angulo = persona["angulo"]
    extra = [
        float(persona["visible"]),
        min(persona["distancia"], DIST_MAX_PERSONA) / DIST_MAX_PERSONA,
        math.sin(angulo),
        math.cos(angulo),
        persona["confianza"],
        min(persona["antiguedad"], ANTIGUEDAD_MAX) / ANTIGUEDAD_MAX,
    ]
    return np.concatenate([np.asarray(scan_reducido, dtype=np.float32),
                           np.array(extra, dtype=np.float32)])


def accion_mas_cercana(v, w):
    """Indice de la accion de ACCIONES mas cercana a (v, w)."""
    errores = [(v - av) ** 2 + (w - aw) ** 2 for av, aw in ACCIONES]
    return int(np.argmin(errores))


def mlp_numpy(pesos, x):
    """Valores Q de un MLP guardado como W0, b0, W1, b1, ... (x @ W + b).

    ReLU entre capas y sin activacion en la salida. pesos puede ser el dict o el
    archivo abierto con np.load.
    """
    h = np.asarray(x, dtype=np.float32)
    n = 0
    while "W%d" % n in pesos:
        n += 1
    for i in range(n):
        h = np.dot(h, pesos["W%d" % i]) + pesos["b%d" % i]
        if i < n - 1:
            h = np.maximum(h, 0.0)
    return h


def cargar_config(ruta=None):
    """CONFIG_PERCEPCION con los valores de un JSON opcional encima."""
    config = dict(CONFIG_PERCEPCION)
    if ruta:
        with open(ruta) as f:
            config.update(json.load(f))
    return config


def cargar_perfil(nombre):
    """Perfil de errores de percepcion medido en la Jetson (models/perfiles/<nombre>.json)."""
    ruta = ruta_repo(os.path.join("models", "perfiles", nombre + ".json"))
    perfil = dict(PERFIL_IDEAL)
    if not os.path.exists(ruta):
        print("AVISO: no existe %s, se usa PERFIL_IDEAL (percepcion perfecta)" % ruta)
        return perfil
    with open(ruta) as f:
        datos = json.load(f)
    for clave in PERFIL_IDEAL:
        if datos.get(clave) is not None:
            perfil[clave] = float(datos[clave])
    return perfil

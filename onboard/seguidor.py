#!/usr/bin/env python3
"""Seguidor de personas a bordo de la Jetson. Ciclo principal a 10 Hz.

    python3 onboard/seguidor.py [--host localhost] [--config cfg.json] [--dqn models/dqn.npz]

Habla con ROS solo por rosbridge (roslibpy), como brain_src/evasor.py. Si existe el
.npz decide la DQN; si no, una politica reactiva provisional con las mismas acciones.
"""
import argparse
import csv
import math
import os
import time

import numpy as np
import roslibpy

from percepcion import RAIZ, RECORDATORIO, Percepcion
import contrato  # noqa: E402  (percepcion ya agrego brain_src al path)

PERIODO = 0.1           # segundos por ciclo (10 Hz)
SCAN_MAX_EDAD = 0.5     # segundos sin /scan antes de frenar
DIST_FRENO = 0.3        # metros al frente antes de frenar
# Sectores de reducir_scan que cubren +-37.5 grados: el ancho del Kobuki a 0.3 m
SECTORES_FRENTE = [-2, -1, 0, 1, 2]
DIST_OBJETIVO = 1.0     # metros a los que se quiere seguir a la persona
K_V, K_W = 0.2, 1.0     # ganancias de la politica reactiva
W_BUSQUEDA = 0.25       # rad/s al buscar a la persona perdida


def politica_reactiva(persona):
    """Provisional mientras no hay DQN. Devuelve un indice de ACCIONES."""
    if persona["visible"]:
        w = max(-0.5, min(0.5, K_W * persona["angulo"]))
        v = max(0.0, min(0.2, K_V * (persona["distancia"] - DIST_OBJETIVO)))
    elif persona["antiguedad"] >= contrato.ANTIGUEDAD_MAX:
        v, w = 0.0, 0.0
    else:
        v, w = 0.0, math.copysign(W_BUSQUEDA, persona["angulo"])
    return contrato.accion_mas_cercana(v, w)


def mover(cmd, v, w):
    cmd.publish(roslibpy.Message({
        "linear": {"x": v, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": w},
    }))


def main():
    parser = argparse.ArgumentParser(description="Seguidor de personas en la Jetson")
    parser.add_argument("--host", default="localhost", help="donde corre rosbridge")
    parser.add_argument("--config", help="JSON que cambia valores de CONFIG_PERCEPCION")
    parser.add_argument("--dqn", default="models/dqn.npz")
    args = parser.parse_args()

    print(RECORDATORIO)
    config = contrato.cargar_config(args.config)
    ruta_dqn = contrato.ruta_repo(args.dqn)
    pesos = None
    if os.path.exists(ruta_dqn):
        archivo_npz = np.load(ruta_dqn)
        pesos = dict((k, archivo_npz[k]) for k in archivo_npz.files)
        archivo_npz.close()
        print("Decision: DQN de " + ruta_dqn)
    else:
        print("Decision: politica reactiva provisional (no existe %s)" % ruta_dqn)

    percepcion = Percepcion(config)
    cliente = roslibpy.Ros(host=args.host, port=9090)
    cliente.run()
    print("Conectado a ROS:", cliente.is_connected)
    cmd = roslibpy.Topic(cliente, "/mobile_base/commands/velocity", "geometry_msgs/Twist")
    ultimo = {"scan": (0.0, None)}  # (hora de llegada, mensaje)

    def al_scan(msg):
        ultimo["scan"] = (time.time(), msg)

    roslibpy.Topic(cliente, "/scan", "sensor_msgs/LaserScan").subscribe(al_scan)

    carpeta = os.path.join(RAIZ, "logs")
    if not os.path.isdir(carpeta):
        os.makedirs(carpeta)
    ruta_log = os.path.join(carpeta, "seguidor_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
    archivo = open(ruta_log, "w", newline="")
    escritor = csv.writer(archivo)
    escritor.writerow(["t", "lat_percepcion", "lat_decision", "visible", "distancia",
                       "angulo", "confianza", "antiguedad", "fuente_distancia",
                       "accion", "v", "w"]
                      + ["obs%d" % i for i in range(contrato.N_RAYOS + 6)])
    print("Registro en " + ruta_log)

    ultima_distancia = contrato.persona_vacia()["distancia"]
    inicio = time.time()
    try:
        while True:
            t_ciclo = time.time()
            persona = percepcion.paso()
            t_decision = time.time()

            llegada, msg = ultimo["scan"]
            if msg is not None:
                rayos = contrato.reducir_scan(msg["angle_min"], msg["angle_increment"],
                                              msg["ranges"])
            else:
                rayos = np.ones(contrato.N_RAYOS, dtype=np.float32)

            # Distancia: profundidad, si no LiDAR en el angulo de la persona, si no la previa
            if not persona["visible"]:
                fuente, persona["distancia"] = "previa", ultima_distancia
            elif persona["distancia"] is not None:
                fuente = "camara"
            else:
                d = None
                if msg is not None:
                    d = contrato.distancia_en_angulo(msg["angle_min"], msg["angle_increment"],
                                                     msg["ranges"], persona["angulo"])
                fuente, persona["distancia"] = (("lidar", d) if d is not None
                                                else ("previa", ultima_distancia))
            ultima_distancia = persona["distancia"]

            obs = contrato.observacion(rayos, persona)
            if pesos is not None:
                accion = int(np.argmax(contrato.mlp_numpy(pesos, obs)))
            else:
                accion = politica_reactiva(persona)
            v, w = contrato.ACCIONES[accion]

            # Seguridad: sin /scan reciente o con algo muy cerca al frente, no moverse
            frente = float(np.min(rayos[SECTORES_FRENTE])) * contrato.RANGO_MAX_LIDAR
            if t_decision - llegada > SCAN_MAX_EDAD or frente < DIST_FRENO:
                v, w = 0.0, 0.0
            mover(cmd, v, w)
            lat_decision = time.time() - t_decision

            escritor.writerow(
                ["%.3f" % (t_ciclo - inicio), "%.4f" % (t_decision - t_ciclo),
                 "%.4f" % lat_decision, persona["visible"], "%.3f" % persona["distancia"],
                 "%.4f" % persona["angulo"], "%.3f" % persona["confianza"],
                 "%.3f" % persona["antiguedad"], fuente, accion, v, w]
                + ["%.4f" % x for x in obs])
            archivo.flush()
            print("visible %d  dist %.2f (%s)  ang %+6.1f  frente %.2f  ->  v %.2f  w %+.2f"
                  % (persona["visible"], persona["distancia"], fuente,
                     math.degrees(persona["angulo"]), frente, v, w))
            time.sleep(max(0.0, PERIODO - (time.time() - t_ciclo)))
    except KeyboardInterrupt:
        print("Detenido con Ctrl+C")
    finally:
        mover(cmd, 0.0, 0.0)
        time.sleep(0.2)  # dar tiempo a que salga el alto antes de cerrar la conexion
        cliente.terminate()
        percepcion.cerrar()
        archivo.close()


if __name__ == "__main__":
    main()

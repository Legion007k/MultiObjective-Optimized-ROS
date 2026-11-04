#!/usr/bin/env python
"""Prueba de solo lectura del LiDAR y la camara Astra S. No publica velocidades.

Verifica que ambos sensores entreguen datos y compara la distancia que cada uno
mide a un objeto colocado al frente del robot.

Se ejecuta dentro del contenedor robot, junto a los drivers, para que las
imagenes no viajen por WiFi:
    rosrun tb2_real prueba_sensores.py --captura /root/ros_ws/capturas
"""
from __future__ import division, print_function

import argparse
import math
import os
import sys
import threading

import numpy as np

CONO_GRADOS = 5.0        # mitad del cono frontal que comparan ambos sensores
FOV_H_GRADOS = 58.4      # campo de vision horizontal de profundidad de la Astra S
RANGO_CAMARA = (0.4, 2.0)  # alcance de profundidad de la Astra S, en metros
FILAS_CENTRALES = 15     # mitad de la franja de filas usada en la imagen de profundidad
MIN_HZ = {"scan": 5.0, "profundidad": 10.0, "color": 10.0}


def distancia_lidar(angle_min, angle_increment, ranges, cono=CONO_GRADOS):
    """Mediana de la distancia al frente, proyectada sobre el eje x del LiDAR."""
    rangos = np.asarray(ranges, dtype=float)
    angulos = angle_min + np.arange(len(rangos)) * angle_increment
    angulos = np.arctan2(np.sin(angulos), np.cos(angulos))
    validos = np.isfinite(rangos) & (rangos > 0) & (np.abs(angulos) < math.radians(cono))
    if not validos.any():
        return None
    return float(np.median(rangos[validos] * np.cos(angulos[validos])))


def profundidad_a_matriz(alto, ancho, paso, big_endian, datos):
    """Convierte un sensor_msgs/Image en 16UC1 a una matriz de milimetros."""
    tipo = ">u2" if big_endian else "<u2"
    return np.frombuffer(datos, dtype=tipo, count=alto * paso // 2).reshape(
        alto, paso // 2)[:, :ancho]


def distancia_camara(profundidad_mm, cono=CONO_GRADOS, fov_h=FOV_H_GRADOS):
    """Mediana en metros de la zona central que cubre el mismo cono que el LiDAR.

    Regresa (distancia, fraccion de pixeles validos). Un valor 0 significa sin dato.
    """
    alto, ancho = profundidad_mm.shape
    foco = (ancho / 2.0) / math.tan(math.radians(fov_h / 2.0))
    medio_ancho = int(round(foco * math.tan(math.radians(cono))))
    centro_x, centro_y = ancho // 2, alto // 2
    zona = profundidad_mm[centro_y - FILAS_CENTRALES:centro_y + FILAS_CENTRALES,
                          centro_x - medio_ancho:centro_x + medio_ancho]
    validos = zona[zona > 0]
    if validos.size == 0:
        return None, 0.0
    return float(np.median(validos)) / 1000.0, validos.size / float(zona.size)


def evaluar(hz, lidar, camara, fraccion, desfase, tolerancia):
    """Regresa una lista de (nombre, paso, detalle) con los criterios de la prueba."""
    res = []
    for nombre, minimo in sorted(MIN_HZ.items()):
        res.append(("Frecuencia " + nombre, hz.get(nombre, 0.0) >= minimo,
                    "%.1f Hz (minimo %.0f)" % (hz.get(nombre, 0.0), minimo)))
    res.append(("LiDAR ve el objeto", lidar is not None,
                "sin lecturas al frente" if lidar is None else "%.3f m" % lidar))
    if camara is None:
        res.append(("Camara ve el objeto", False, "sin pixeles validos al centro"))
    else:
        res.append(("Camara ve el objeto", fraccion >= 0.5,
                    "%.3f m, %.0f%% de pixeles validos" % (camara, fraccion * 100)))
        res.append(("Objeto dentro del alcance de la camara",
                    RANGO_CAMARA[0] <= camara <= RANGO_CAMARA[1],
                    "alcance %.1f a %.1f m" % RANGO_CAMARA))
    if lidar is not None and camara is not None:
        diferencia = (lidar + desfase) - camara
        res.append(("Coinciden las distancias", abs(diferencia) <= tolerancia,
                    "diferencia %+.3f m (tolerancia %.2f, desfase %+.3f)"
                    % (diferencia, tolerancia, desfase)))
    return res


def guardar_capturas(carpeta, profundidad_mm, color_msg):
    import cv2
    if not os.path.isdir(carpeta):
        os.makedirs(carpeta)
    minimo, maximo = RANGO_CAMARA[0] * 1000, RANGO_CAMARA[1] * 1000
    escala = (np.clip(profundidad_mm, minimo, maximo) - minimo) * 255.0 / (maximo - minimo)
    falso = cv2.applyColorMap(escala.astype(np.uint8), cv2.COLORMAP_JET)
    falso[profundidad_mm == 0] = 0
    cv2.imwrite(os.path.join(carpeta, "profundidad.png"), falso)
    if color_msg is not None:
        rgb = np.frombuffer(color_msg.data, dtype=np.uint8).reshape(
            color_msg.height, color_msg.step // 3, 3)[:, :color_msg.width]
        cv2.imwrite(os.path.join(carpeta, "color.png"), rgb[:, :, ::-1])


def desfase_sensores(tf2_ros, rospy):
    """Metros que el LiDAR esta por delante de la camara, segun el arbol de TF."""
    buf = tf2_ros.Buffer()
    escucha = tf2_ros.TransformListener(buf)  # debe seguir vivo durante la consulta
    try:
        lidar = buf.lookup_transform("base_footprint", "rplidar_link", rospy.Time(0),
                                     rospy.Duration(3.0))
        camara = buf.lookup_transform("base_footprint", "camera_link", rospy.Time(0),
                                      rospy.Duration(3.0))
    except Exception as error:
        print("AVISO: sin TF de los sensores, se usa desfase 0 (%s)" % error)
        return 0.0
    del escucha
    return lidar.transform.translation.x - camara.transform.translation.x


def main():
    import rospy
    import tf2_ros
    from sensor_msgs.msg import Image, LaserScan

    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--duracion", type=float, default=10.0, help="segundos de medicion")
    parser.add_argument("--tolerancia", type=float, default=0.10, help="metros")
    parser.add_argument("--captura", help="carpeta donde guardar las imagenes")
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])

    rospy.init_node("prueba_sensores", anonymous=True, disable_signals=True)
    candado = threading.Lock()
    cuentas = {"scan": 0, "profundidad": 0, "color": 0}
    ultimo = {"profundidad": None, "color": None}
    lidar, camara, fraccion = [], [], []
    midiendo = threading.Event()

    def al_scan(msg):
        if not midiendo.is_set():
            return
        d = distancia_lidar(msg.angle_min, msg.angle_increment, msg.ranges)
        with candado:
            cuentas["scan"] += 1
            if d is not None:
                lidar.append(d)

    def al_profundidad(msg):
        if not midiendo.is_set():
            return
        with candado:
            cuentas["profundidad"] += 1
            ultimo["profundidad"] = msg

    def al_color(msg):
        if not midiendo.is_set():
            return
        with candado:
            cuentas["color"] += 1
            ultimo["color"] = msg

    tam = 2 ** 24
    subs = [rospy.Subscriber("/scan", LaserScan, al_scan, queue_size=1),
            rospy.Subscriber("/camera/depth/image_raw", Image, al_profundidad,
                             queue_size=1, buff_size=tam),
            rospy.Subscriber("/camera/color/image_raw", Image, al_color,
                             queue_size=1, buff_size=tam)]

    desfase = desfase_sensores(tf2_ros, rospy)
    rospy.sleep(3.0)  # la camara arranca sus flujos al detectar el suscriptor
    print("Midiendo %.0f s. No muevas el objeto..." % args.duracion)
    midiendo.set()
    inicio = rospy.Time.now()
    fin = rospy.Time.now() + rospy.Duration(args.duracion)
    procesado = None
    matriz = None
    while rospy.Time.now() < fin and not rospy.is_shutdown():
        with candado:
            msg = ultimo["profundidad"]
        if msg is not None and msg is not procesado:
            procesado = msg
            if msg.encoding not in ("16UC1", "mono16"):
                print("ERROR: codificacion inesperada en profundidad: " + msg.encoding)
                return 1
            matriz = profundidad_a_matriz(msg.height, msg.width, msg.step,
                                          msg.is_bigendian, msg.data)
            d, f = distancia_camara(matriz)
            if d is not None:
                camara.append(d)
                fraccion.append(f)
        rospy.sleep(0.05)
    midiendo.clear()
    transcurrido = (rospy.Time.now() - inicio).to_sec()
    for sub in subs:
        sub.unregister()

    with candado:
        hz = dict((k, v / transcurrido) for k, v in cuentas.items())
        color = ultimo["color"]
    d_lidar = float(np.median(lidar)) if lidar else None
    d_camara = float(np.median(camara)) if camara else None
    f_media = float(np.mean(fraccion)) if fraccion else 0.0

    resultados = evaluar(hz, d_lidar, d_camara, f_media, desfase, args.tolerancia)
    print("")
    for nombre, paso, detalle in resultados:
        print("[%s] %s: %s" % ("OK" if paso else "FALLA", nombre, detalle))
    if args.captura and matriz is not None:
        guardar_capturas(args.captura, matriz, color)
        print("Imagenes guardadas en " + args.captura)
    exito = all(r[1] for r in resultados)
    print("\nRESULTADO: " + ("APROBADA" if exito else "NO APROBADA"))
    return 0 if exito else 1


if __name__ == "__main__":
    sys.exit(main())

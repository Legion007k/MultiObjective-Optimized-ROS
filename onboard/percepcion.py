#!/usr/bin/env python3
"""Percepcion a bordo: camara Astra S por OpenNI2 y YOLOv8 en TensorRT.

Corre nativo en la Jetson con el Python 3.6 de JetPack. El robot se lanza con
camara:=false porque la camara solo la puede abrir un programa a la vez.

    python3 onboard/percepcion.py                    # detecciones y FPS
    python3 onboard/percepcion.py --perfil --distancia-real 1.5 --angulo-real 0 --segundos 30
"""
import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(RAIZ, "brain_src"))
import contrato  # noqa: E402

RUTA_OPENNI = os.path.join(RAIZ, "onboard", "vendor", "openni2")
ANCHO, ALTO, FPS_CAMARA = 640, 480, 30
UMBRAL_NMS = 0.45
RECORDATORIO = ("RECORDATORIO: el robot debe estar lanzado con camara:=false; "
                "la camara solo la puede abrir un programa a la vez.")


# ---------- Camara ----------

def abrir_camara():
    """Abre profundidad y color a 640x480 con la profundidad registrada al color."""
    from primesense import openni2
    openni2.initialize(RUTA_OPENNI)
    dispositivo = openni2.Device.open_any()
    profundidad = dispositivo.create_depth_stream()
    profundidad.configure_mode(ANCHO, ALTO, FPS_CAMARA, openni2.PIXEL_FORMAT_DEPTH_1_MM)
    profundidad.set_mirroring_enabled(False)  # sin espejo: izquierda de la imagen = izquierda
    profundidad.start()
    try:
        color = dispositivo.create_color_stream()
        color.configure_mode(ANCHO, ALTO, FPS_CAMARA, openni2.PIXEL_FORMAT_RGB888)
        color.set_mirroring_enabled(False)
        color.start()
        if openni2.wait_for_any_stream([color], timeout=3.0) is None:
            raise RuntimeError("no llego ningun cuadro en 3 s")
    except Exception as error:
        raise RuntimeError("No se pudo abrir el color de la Astra S por OpenNI2 (%s). "
                           "No hay otro respaldo para el color." % error)
    dispositivo.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)
    print("Camara abierta. FOV horizontal segun OpenNI2: color %.1f, profundidad %.1f grados"
          % (math.degrees(color.get_horizontal_fov()),
             math.degrees(profundidad.get_horizontal_fov())))
    return {"openni2": openni2, "profundidad": profundidad, "color": color}


def leer_camara(camara):
    """(profundidad en mm uint16, color RGB uint8), ambos de ALTO x ANCHO."""
    p = camara["profundidad"].read_frame()
    c = camara["color"].read_frame()
    profundidad = np.frombuffer(p.get_buffer_as_uint16(), dtype=np.uint16)
    color = np.frombuffer(c.get_buffer_as_uint8(), dtype=np.uint8)
    return (profundidad.reshape(p.height, p.width).copy(),
            color.reshape(c.height, c.width, 3).copy())


# ---------- Detector ----------

def cargar_motor(ruta):
    """Carga un motor de TensorRT 8.2 y reserva sus buffers en la GPU."""
    import tensorrt as trt
    import pycuda.autoinit  # noqa: F401  crea el contexto de CUDA
    import pycuda.driver as cuda

    runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
    with open(ruta, "rb") as f:
        motor = runtime.deserialize_cuda_engine(f.read())
    if motor is None:
        raise RuntimeError("TensorRT no pudo cargar " + ruta)
    m = {"cuda": cuda, "runtime": runtime, "motor": motor, "enlaces": [],
         "contexto": motor.create_execution_context(), "flujo": cuda.Stream()}
    for i in range(motor.num_bindings):
        forma = tuple(motor.get_binding_shape(i))
        host = cuda.pagelocked_empty(trt.volume(forma), trt.nptype(motor.get_binding_dtype(i)))
        disp = cuda.mem_alloc(host.nbytes)
        m["enlaces"].append(int(disp))
        m["entrada" if motor.binding_is_input(i) else "salida"] = (host, disp, forma)
    return m


def inferir(m, tensor):
    cuda, flujo = m["cuda"], m["flujo"]
    host_in, disp_in, _ = m["entrada"]
    host_out, disp_out, forma_out = m["salida"]
    np.copyto(host_in, tensor.ravel())
    cuda.memcpy_htod_async(disp_in, host_in, flujo)
    m["contexto"].execute_async_v2(bindings=m["enlaces"], stream_handle=flujo.handle)
    cuda.memcpy_dtoh_async(host_out, disp_out, flujo)
    flujo.synchronize()
    return host_out.reshape(forma_out)


def letterbox(rgb, imgsz):
    """Escala sin deformar a imgsz x imgsz con relleno gris, como YOLOv8.

    Devuelve (tensor 1x3xHxW float32 en [0, 1], escala, relleno_x, relleno_y).
    """
    alto, ancho = rgb.shape[:2]
    escala = min(imgsz / alto, imgsz / ancho)
    nuevo_ancho, nuevo_alto = int(round(ancho * escala)), int(round(alto * escala))
    pad_x, pad_y = (imgsz - nuevo_ancho) // 2, (imgsz - nuevo_alto) // 2
    lienzo = np.full((imgsz, imgsz, 3), 114, dtype=np.uint8)
    lienzo[pad_y:pad_y + nuevo_alto, pad_x:pad_x + nuevo_ancho] = cv2.resize(
        rgb, (nuevo_ancho, nuevo_alto), interpolation=cv2.INTER_LINEAR)
    tensor = lienzo.transpose(2, 0, 1)[np.newaxis].astype(np.float32) / 255.0
    return np.ascontiguousarray(tensor), escala, pad_x, pad_y


def decodificar(salida, confianza_min, escala, pad_x, pad_y, ancho, alto):
    """Salida de YOLOv8 (1, 84, N) -> lista de (x1, y1, x2, y2, confianza) de personas.

    Filas 0-3: centro y tamano de la caja en pixeles del letterbox; fila 4: clase 0.
    """
    pred = salida.reshape(salida.shape[-2], salida.shape[-1])
    puntaje = pred[4]
    sel = np.flatnonzero(puntaje >= confianza_min)
    if sel.size == 0:
        return []
    cx, cy, w, h = pred[0, sel], pred[1, sel], pred[2, sel], pred[3, sel]
    x1 = np.clip((cx - w / 2 - pad_x) / escala, 0, ancho - 1)
    y1 = np.clip((cy - h / 2 - pad_y) / escala, 0, alto - 1)
    x2 = np.clip((cx + w / 2 - pad_x) / escala, 0, ancho - 1)
    y2 = np.clip((cy + h / 2 - pad_y) / escala, 0, alto - 1)
    p = puntaje[sel]
    cajas = [[int(a), int(b), int(c - a), int(d - b)] for a, b, c, d in zip(x1, y1, x2, y2)]
    indices = cv2.dnn.NMSBoxes(cajas, p.tolist(), confianza_min, UMBRAL_NMS)
    return [(float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]), float(p[i]))
            for i in np.array(indices, dtype=np.int64).flatten()]


# ---------- Estimacion ----------

def estimar(caja, profundidad_mm):
    """(angulo, distancia) de la persona en la caja.

    Angulo desde el centro horizontal de la caja con el foco de prueba_sensores.py,
    positivo a la izquierda. Distancia: mediana de la profundidad valida en la mitad
    central de la caja, None si cae fuera de RANGO_PROFUNDIDAD.
    """
    x1, y1, x2, y2 = caja[:4]
    alto, ancho = profundidad_mm.shape
    foco = (ancho / 2.0) / math.tan(math.radians(contrato.FOV_H_GRADOS / 2.0))
    angulo = math.atan(((ancho / 2.0) - (x1 + x2) / 2.0) / foco)
    margen_x, margen_y = (x2 - x1) / 4.0, (y2 - y1) / 4.0
    zona = profundidad_mm[int(y1 + margen_y):int(y2 - margen_y) + 1,
                          int(x1 + margen_x):int(x2 - margen_x) + 1]
    validos = zona[zona > 0]
    if validos.size == 0:
        return angulo, None
    z = float(np.median(validos)) / 1000.0
    if not contrato.RANGO_PROFUNDIDAD[0] <= z <= contrato.RANGO_PROFUNDIDAD[1]:
        return angulo, None
    # La profundidad se mide sobre el eje optico; se pasa a distancia en el plano del piso
    return angulo, z / math.cos(angulo)


class Percepcion(object):

    def __init__(self, config):
        self.config = config
        ruta_motor = os.path.splitext(contrato.ruta_repo(config["onnx"]))[0] + ".engine"
        if not os.path.exists(ruta_motor):
            raise RuntimeError("No existe %s: corre onboard/instalar_jetson.sh" % ruta_motor)
        self.motor = cargar_motor(ruta_motor)
        if self.motor["entrada"][2][-1] != config["imgsz"]:
            raise RuntimeError("El motor espera imgsz %d y la config dice %d"
                               % (self.motor["entrada"][2][-1], config["imgsz"]))
        self.camara = abrir_camara()
        self.persona = contrato.persona_vacia()
        self.t_visto = None     # captura del ultimo cuadro donde se vio a la persona
        self.centro = None      # centro en pixeles de la ultima persona elegida
        self.cuadro = 0
        self.nueva = False      # True si el ultimo paso() corrio el detector
        self.n_personas = 0
        self.latencia_captura = 0.0
        self.latencia_inferencia = 0.0

    def elegir(self, cajas):
        """La persona mas cercana a la ultima elegida, o la de mayor confianza."""
        if not cajas:
            return None
        if self.centro is None:
            return max(cajas, key=lambda c: c[4])
        cx, cy = self.centro
        return min(cajas, key=lambda c: ((c[0] + c[2]) / 2 - cx) ** 2
                   + ((c[1] + c[3]) / 2 - cy) ** 2)

    def paso(self):
        """Dict de persona del contrato. distancia es None si la persona se ve pero
        la profundidad no sirve (el seguidor la completa con el LiDAR)."""
        self.nueva = self.cuadro % int(self.config["cada_n_cuadros"]) == 0
        self.cuadro += 1
        if self.nueva:
            t0 = time.time()
            profundidad, rgb = leer_camara(self.camara)
            t1 = time.time()
            tensor, escala, pad_x, pad_y = letterbox(rgb, self.config["imgsz"])
            cajas = decodificar(inferir(self.motor, tensor), self.config["confianza_min"],
                                escala, pad_x, pad_y, rgb.shape[1], rgb.shape[0])
            caja = self.elegir(cajas)
            if caja is None:
                self.persona.update(visible=0, confianza=0.0)
            else:
                angulo, distancia = estimar(caja, profundidad)
                self.persona.update(visible=1, distancia=distancia, angulo=angulo,
                                    confianza=caja[4])
                self.centro = ((caja[0] + caja[2]) / 2, (caja[1] + caja[3]) / 2)
                self.t_visto = t0
            self.n_personas = len(cajas)
            self.latencia_captura = t1 - t0
            self.latencia_inferencia = time.time() - t1
        persona = dict(self.persona)
        persona["antiguedad"] = (contrato.ANTIGUEDAD_MAX if self.t_visto is None
                                 else time.time() - self.t_visto)
        return persona

    def cerrar(self):
        self.camara["profundidad"].stop()
        self.camara["color"].stop()
        self.camara["openni2"].unload()


# ---------- Linea de comandos ----------

def mostrar(percepcion):
    anterior, fps = time.time(), 0.0
    while True:
        persona = percepcion.paso()
        ahora = time.time()
        fps = 0.9 * fps + 0.1 / max(ahora - anterior, 1e-6)
        anterior = ahora
        distancia = persona["distancia"]
        print("personas %d  visible %d  dist %s  ang %+6.1f  conf %.2f  |  captura %3.0f ms"
              "  inferencia %3.0f ms  |  %4.1f FPS"
              % (percepcion.n_personas, persona["visible"],
                 "  -  " if distancia is None else "%.2f" % distancia,
                 math.degrees(persona["angulo"]), persona["confianza"],
                 percepcion.latencia_captura * 1000, percepcion.latencia_inferencia * 1000, fps))


def media(valores):
    return sum(valores) / len(valores) if valores else None


def rms(valores):
    return math.sqrt(media([v * v for v in valores])) if valores else None


def combinar(mediciones):
    """Perfil a partir de todas las mediciones, ponderando cada una por sus muestras."""
    def promedio(clave, peso, cuadratico=False):
        pares = [(m[clave], m[peso]) for m in mediciones
                 if m.get(clave) is not None and m[peso] > 0]
        total = sum(n for _, n in pares)
        if total == 0:
            return None
        if cuadratico:
            return math.sqrt(sum(v * v * n for v, n in pares) / total)
        return sum(v * n for v, n in pares) / total

    return {"sigma_distancia": promedio("sigma_distancia", "con_distancia", True),
            "sigma_angulo": promedio("sigma_angulo", "vistos", True),
            "prob_perdida": promedio("prob_perdida", "cuadros"),
            "latencia": promedio("latencia", "cuadros"),
            "confianza_media": promedio("confianza_media", "vistos")}


def medir_perfil(percepcion, config, distancia_real, angulo_real_grados, segundos):
    """Persona quieta en una posicion medida: errores, perdidas, latencia y confianza."""
    angulo_real = math.radians(angulo_real_grados)
    cuadros, vistos = 0, 0
    err_d, err_a, confianzas, latencias = [], [], [], []
    print("Midiendo %.0f s. La persona se queda quieta y sola frente a la camara..." % segundos)
    fin = time.time() + segundos
    while time.time() < fin:
        persona = percepcion.paso()
        if not percepcion.nueva:
            continue
        cuadros += 1
        latencias.append(percepcion.latencia_captura + percepcion.latencia_inferencia)
        if persona["visible"]:
            vistos += 1
            confianzas.append(persona["confianza"])
            dif = persona["angulo"] - angulo_real
            err_a.append(math.atan2(math.sin(dif), math.cos(dif)))
            if persona["distancia"] is not None:
                err_d.append(persona["distancia"] - distancia_real)

    medicion = {
        "fecha": time.strftime("%Y-%m-%d %H:%M:%S"),
        "distancia_real": distancia_real, "angulo_real_grados": angulo_real_grados,
        "segundos": segundos, "cuadros": cuadros, "vistos": vistos,
        "con_distancia": len(err_d),
        "sigma_distancia": rms(err_d), "sesgo_distancia": media(err_d),
        "sigma_angulo": rms(err_a), "sesgo_angulo": media(err_a),
        "prob_perdida": 1.0 - vistos / cuadros if cuadros else None,
        "latencia": media(latencias), "confianza_media": media(confianzas),
    }
    for clave in sorted(medicion):
        print("  %-18s %s" % (clave, medicion[clave]))

    ruta = contrato.ruta_repo(os.path.join("models", "perfiles", config["nombre"] + ".json"))
    datos = {}
    if os.path.exists(ruta):
        with open(ruta) as f:
            datos = json.load(f)
    datos["config"] = config
    datos.setdefault("mediciones", []).append(medicion)
    datos.update(combinar(datos["mediciones"]))
    if not os.path.isdir(os.path.dirname(ruta)):
        os.makedirs(os.path.dirname(ruta))
    with open(ruta, "w") as f:
        json.dump(datos, f, indent=2, sort_keys=True)
    print("Perfil con %d mediciones guardado en %s" % (len(datos["mediciones"]), ruta))


def main():
    parser = argparse.ArgumentParser(description="Percepcion a bordo con la Astra S y YOLOv8")
    parser.add_argument("--config", help="JSON que cambia valores de CONFIG_PERCEPCION")
    parser.add_argument("--perfil", action="store_true",
                        help="medir el perfil de errores y guardarlo en models/perfiles/")
    parser.add_argument("--distancia-real", type=float, help="metros, medida con flexometro")
    parser.add_argument("--angulo-real", type=float, default=0.0,
                        help="grados, positivo a la izquierda")
    parser.add_argument("--segundos", type=float, default=30.0)
    args = parser.parse_args()
    if args.perfil and args.distancia_real is None:
        parser.error("--perfil necesita --distancia-real")

    print(RECORDATORIO)
    config = contrato.cargar_config(args.config)
    percepcion = Percepcion(config)
    try:
        if args.perfil:
            medir_perfil(percepcion, config, args.distancia_real, args.angulo_real,
                         args.segundos)
        else:
            mostrar(percepcion)
    except KeyboardInterrupt:
        pass
    finally:
        percepcion.cerrar()


if __name__ == "__main__":
    main()

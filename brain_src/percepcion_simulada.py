"""Percepcion simulada para Gazebo: en lugar de correr YOLO, toma la posicion real
de la persona y le aplica un perfil de errores medido en la Jetson.

Entrega el mismo dict de persona que onboard/percepcion.py (ver contrato.persona_vacia).
Valido en Python 3.6.
"""
import collections
import math

import numpy as np

from contrato import (ANTIGUEDAD_MAX, DIST_MAX_PERSONA, FOV_H_GRADOS, PERFIL_IDEAL,
                      persona_vacia)


class PercepcionSimulada(object):

    def __init__(self, perfil, semilla=None):
        self.perfil = dict(PERFIL_IDEAL)
        self.perfil.update(perfil)
        self.rng = np.random.RandomState(semilla)
        self.cola = collections.deque()   # mediciones (t, visible, distancia, angulo)
        self.persona = persona_vacia()
        self.t_visto = None               # tiempo de la ultima medicion con persona visible

    def paso(self, t, x_rel, y_rel, ocluido):
        """Estimacion de la persona en el tiempo t (segundos).

        x_rel, y_rel: posicion real de la persona en el marco del robot (x al frente,
        y a la izquierda), en metros. ocluido: True si algo tapa la linea de vista.
        Devuelve la medicion tomada en t - latencia, como pasa en la Jetson.
        """
        p = self.perfil
        distancia = math.hypot(x_rel, y_rel)
        angulo = math.atan2(y_rel, x_rel)
        visible = (abs(angulo) <= math.radians(FOV_H_GRADOS / 2.0)
                   and distancia <= DIST_MAX_PERSONA
                   and not ocluido
                   and self.rng.uniform() >= p["prob_perdida"])
        distancia = max(0.0, distancia + self.rng.normal(0.0, p["sigma_distancia"]))
        angulo += self.rng.normal(0.0, p["sigma_angulo"])
        angulo = math.atan2(math.sin(angulo), math.cos(angulo))
        self.cola.append((t, visible, distancia, angulo))

        # Latencia: usar la medicion mas reciente que ya tenga al menos esa edad
        limite = t - p["latencia"]
        while len(self.cola) > 1 and self.cola[1][0] <= limite:
            self.cola.popleft()
        t_med, visible, distancia, angulo = self.cola[0]
        if t_med <= limite:
            if visible:
                self.persona.update(visible=1, distancia=distancia, angulo=angulo,
                                    confianza=min(1.0, max(0.0, p["confianza_media"])))
                self.t_visto = t_med
            else:
                self.persona.update(visible=0, confianza=0.0)

        persona = dict(self.persona)
        persona["antiguedad"] = ANTIGUEDAD_MAX if self.t_visto is None else t - self.t_visto
        return persona


def aleatorizar(perfil, factor, rng):
    """Copia del perfil con cada parametro escalado al azar en [1-factor, 1+factor].

    Aleatorizacion de dominio para entrenar. rng: np.random.RandomState o Generator.
    """
    nuevo = dict(perfil)
    for clave in PERFIL_IDEAL:
        nuevo[clave] = perfil[clave] * rng.uniform(1.0 - factor, 1.0 + factor)
    nuevo["prob_perdida"] = min(1.0, nuevo["prob_perdida"])
    nuevo["confianza_media"] = min(1.0, nuevo["confianza_media"])
    return nuevo

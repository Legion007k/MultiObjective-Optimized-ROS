# Cerebro nativo en la Jetson

## Arquitectura

| Pieza | Donde corre | Que hace |
|---|---|---|
| Contenedor `robot` | Jetson (Docker) | Kobuki, LiDAR y rosbridge en el puerto 9090 |
| `onboard/seguidor.py` | Jetson, Python 3.6 de JetPack, sin contenedor | Ciclo de 10 Hz: percepcion, observacion, decision, velocidad |
| `onboard/percepcion.py` | Dentro del seguidor | Abre la Astra S con OpenNI2 y corre YOLOv8n en TensorRT |
| `brain_src/contrato.py` | Laptop y Jetson | Observacion, acciones, MLP en numpy, config y perfiles |

El seguidor habla con ROS solo por roslibpy a `localhost:9090`, como `evasor.py`.
Decide con la DQN si existe `models/dqn.npz`; si no, con una politica reactiva
provisional que usa las mismas 15 acciones.

**Regla de la camara: la abre un solo programa a la vez.**

- Produccion: robot con `camara:=false` y la camara la abre `percepcion.py`.
- Diagnostico: robot con `camara:=true` y `prueba_sensores.py` (ver `prueba_astra_s.md`),
  sin el seguidor ni `percepcion.py` corriendo.

## Flujo laptop -> git -> Jetson

En la laptop, desde la raiz del repositorio (el contenedor `brain` monta el repo en `/repo`):

```bash
docker compose build brain        # solo si cambio brain/
docker compose run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD":/repo -w /repo \
  brain python brain_src/exportar_modelos.py yolo --imgsz 320
docker compose run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD":/repo -w /repo \
  brain python brain_src/exportar_modelos.py dqn ruta/al/state_dict.pt   # cuando exista la DQN
git add models/ && git commit -m "Modelos" && git push
```

En la Jetson:

```bash
git pull
bash onboard/instalar_jetson.sh
```

`instalar_jetson.sh` se puede correr siempre: instala los paquetes de Python, copia
OpenNI2 a `onboard/vendor/`, instala las reglas de udev (pide sudo solo ahi) y construye
un `.engine` por cada `.onnx` nuevo con `trtexec` (tarda varios minutos). Los `.engine`
no se versionan porque dependen de la Jetson y de TensorRT.

## Correr el seguidor

Terminal 1, dentro del contenedor `robot`:

```bash
roslaunch tb2_real tb2_real.launch camara:=false
```

Terminal 2, en la Jetson fuera del contenedor:

```bash
python3 onboard/percepcion.py     # primero: ver detecciones y FPS, Ctrl+C para salir
python3 onboard/seguidor.py       # opcional: --config cfg.json --dqn models/dqn.npz
```

Antes de dejarlo moverse, comprobar con `percepcion.py` que una persona a la izquierda
del robot da angulo positivo.

El seguidor frena si deja de llegar `/scan` por 0.5 s, si hay algo a menos de 0.3 m al
frente, ante cualquier error y con Ctrl+C. Cada corrida queda en
`logs/seguidor_<fecha>.csv`.

## Medir un perfil de percepcion

El perfil dice cuanto se equivoca la percepcion real y se usa en Gazebo para simular
la camara. No necesita el robot encendido, solo la camara.

1. Una sola persona, quieta y de frente a la camara.
2. Medir con flexometro desde la camara: `X` al frente y `L` de lado (positivo a la
   izquierda). Distancia `D = raiz(X^2 + L^2)` en metros, angulo `A = atan(L / X)` en grados.
3. Correr:

```bash
python3 onboard/percepcion.py --perfil --distancia-real D --angulo-real A --segundos 30
```

Cada corrida se agrega a `models/perfiles/<nombre>.json` (el `nombre` de la config,
`yolov8n_320` por defecto) y el perfil se recalcula con todas las corridas. Conviene
medir varias posiciones, por ejemplo 0.8, 1.2 y 1.8 m al centro y a unos 20 grados de
cada lado. Despues se sube el JSON con git y en la laptop se carga con
`contrato.cargar_perfil("yolov8n_320")`.

Para otra configuracion (modelo, resolucion, umbral, cada cuantos cuadros) se pasa un
JSON con `--config`, con otro `nombre` para que tenga su propio perfil, por ejemplo
`{"nombre": "yolov8n_416", "onnx": "models/yolov8n_416.onnx", "imgsz": 416}`.

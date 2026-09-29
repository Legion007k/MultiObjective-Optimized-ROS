import math
import time
import roslibpy
import os

HOST = os.environ.get("ROBOT_HOST", "localhost")# buscar variable de entorno ROBOT_HOST que es el robot real, si no existe usar localhost que es el robot virtual

cliente = roslibpy.Ros(host=HOST, port=9090)
cliente.run()
print("Conectado a ROS:", cliente.is_connected)

cmd = roslibpy.Topic(cliente, "/mobile_base/commands/velocity", "geometry_msgs/Twist")
scan = roslibpy.Topic(cliente, "/scan", "sensor_msgs/LaserScan")
estado = {"frente": float("inf")}

def al_recibir_scan(msg):
    a0 = msg["angle_min"]
    da = msg["angle_increment"]
    frente = []
    for i, d in enumerate(msg["ranges"]):
        ang = a0 + i * da
        ang = math.atan2(math.sin(ang), math.cos(ang))  # normaliza a [-pi, pi]
        if abs(ang) < math.radians(15) and d is not None and d > 0 \
                and not math.isinf(d) and not math.isnan(d):
            frente.append(d)
    estado["frente"] = min(frente) if frente else float("inf")

scan.subscribe(al_recibir_scan)

def mover(v, w):
    cmd.publish(roslibpy.Message({
        "linear": {"x": v, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": w},
    }))

try:
    while True:
        if estado["frente"] > 0.6:
            v, w = 0.2, 0.0   # camino libre: avanza
        else:
            v, w = 0.0, 0.8   # obstáculo: gira
        mover(v, w)
        print(f"frente = {estado['frente']:.2f} m   v = {v}   w = {w}")
        time.sleep(0.1)
except KeyboardInterrupt:
    mover(0.0, 0.0)
    cliente.terminate()
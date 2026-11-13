# Prueba basica de la camara Astra S junto al LiDAR

Objetivo: confirmar que la Jetson recibe profundidad y color de la Astra S mientras
el LiDAR funciona, y que ambos sensores miden lo mismo. La prueba es de solo lectura:
el robot no se mueve.

## Antes de ir al laboratorio

Necesitan: hub USB con alimentacion propia (la TX2 tiene un solo puerto USB tipo A y
ahora son tres dispositivos), flexometro y una caja o pared plana de al menos 40 cm de ancho.

## 1. Conectar y reconstruir (solo la primera vez)

Con la camara conectada a la Jetson:

```bash
lsusb | grep 2bc5        # la Astra S debe aparecer como 2bc5:0402
ls -l /dev/ttyUSB*       # el hub puede cambiar el orden: revisar KOBUKI_PORT y LIDAR_PORT en .env
docker-compose -f docker-compose.jetson.yml build robot
docker-compose -f docker-compose.jetson.yml up -d --force-recreate robot
```

El Kobuki ya compilado se reutiliza; solo se compilan libuvc y el driver de la camara.

Si la camara se desconecta despues, hay que repetir el `up -d --force-recreate`.

## 2. Medir los sensores

Medir en metros, desde el centro de la base y hacia el frente, la posicion `x` del
LiDAR y de la camara, y la altura `z` de cada uno sobre el piso. La camara debe
quedar mirando al frente, sin inclinacion.

## 3. Arrancar

Terminal 1, dentro del contenedor (`docker exec -it <contenedor> bash`):

```bash
roslaunch tb2_real tb2_real.launch camara:=true \
  lidar_x:=X lidar_z:=Z camara_x:=X camara_z:=Z
```

Sin `camara:=true` el robot arranca como siempre y `evasor.py` no cambia.

## 4. Probar

Colocar la caja al frente, centrada, a 1.0 m. Debe cruzar la altura del LiDAR y la
de la camara. Terminal 2, dentro del contenedor:

```bash
rosrun tb2_real prueba_sensores.py --captura /root/ros_ws/capturas
```

Repetir a 0.6 m y 1.5 m. Las imagenes quedan en `ros_ws/capturas` del repositorio
en la Jetson; se pueden copiar con `scp` para el reporte.

La prueba se aprueba cuando las tres distancias terminan en `RESULTADO: APROBADA`:
scan a 5 Hz o mas, profundidad y color a 10 Hz o mas, y diferencia entre sensores
de 10 cm o menos.

## Si algo falla

| Sintoma | Causa probable | Que hacer |
|---|---|---|
| `lsusb` no muestra `2bc5` | Falta de energia o cable | Usar el hub alimentado; probar otro cable |
| El driver no encuentra dispositivos | Contenedor creado sin la camara | `up -d --force-recreate` |
| El driver se queda detenido al arrancar | Semaforo viejo en `/dev/shm` | `rosrun astra_camera cleanup_shm_node` |
| El LiDAR o el Kobuki dejan de abrir su puerto | El hub cambio el orden | Corregir `.env` y recrear el contenedor |
| 0 Hz en profundidad o color | El driver no inicio el flujo | Revisar la terminal 1 |
| Diferencia constante en las tres distancias | Medidas de `x` incorrectas | Volver a medir el paso 2 |
| Diferencia que crece con la distancia | Camara inclinada u objeto no perpendicular | Enderezar y repetir |
| Pocos pixeles validos | Superficie negra o brillante, luz solar | Usar una caja mate, lejos de la ventana |

## Limites que hay que recordar

- La Astra S mide de 0.4 a 2 m. No ve nada mas cerca de 0.4 m, asi que no reemplaza al LiDAR.
- No suscribir el cerebro a las imagenes desde la laptop: viajarian crudas por rosbridge y
  el hotspot. La camara solo envia datos cuando alguien se suscribe.

# Bitácora técnica: de las primeras pruebas con ROS al primer movimiento del robot

**Proyecto:** robot seguidor autónomo sobre TurtleBot 2 (base Kobuki), con DQN como módulo de decisión y YOLO para percepción visual. Supervisor: Dr. Ulises Orozco-Rosas.
**Equipo:** Karim Ortiz, Alonso López, Paris.
**Periodo:** desde el primer contacto con ROS en Docker hasta el 28 de octubre de 2026.
**Estado al cierre:** el robot real se mueve de forma autónoma con un evasor reactivo, controlado desde la laptop a través de la Jetson TX2, con el LiDAR correctamente alineado.

Las sesiones se trabajaron los miércoles. Las fechas marcadas como aproximadas se reconstruyeron a partir de los commits y de las conversaciones.

| Sesión | Fecha | Hito |
|---|---|---|
| 1 | Mediados de septiembre (aprox.) | Primer contacto con ROS Melodic en Docker |
| 2 | 23 de septiembre (aprox.) | Entorno de simulación funcionando: TurtleBot 2 con RPLIDAR en Gazebo, controlado desde el cerebro |
| 3 | 30 de septiembre (aprox.) | Paquete del robot real, host configurable, README y repositorio compartido |
| 4 | 7 de octubre (aprox.) | Primer laboratorio: teleoperación del Kobuki desde la laptop |
| 5 | Principios de octubre (aprox.) | Plan de despliegue en la Jetson y decisión de reflashear |
| 6 | 14 de octubre | Jetson reflasheada y contenedor del robot construido |
| 7 | 21 de octubre | Kobuki, LiDAR y rosbridge funcionando dentro de la Jetson |
| 8 | 28 de octubre | Primer movimiento autónomo del robot real |

---

## Arquitectura vigente

El sistema tiene dos lados conectados por WiFi.

En la **laptop** (partición con Ubuntu moderno) viven dos contenedores Docker. El contenedor `sim` tiene Ubuntu 18.04, ROS Melodic y Gazebo 9, y se usa para simular. El contenedor `brain` tiene Python 3.12, PyTorch y roslibpy, y es donde corre el código que decide. La separación existe porque ROS Melodic depende de Python 2.7 y las herramientas modernas de aprendizaje profundo necesitan Python 3.

En el **robot**, la Jetson TX2 (JetPack 4.6.6, Ubuntu 18.04) corre un contenedor `robot` con ROS Melodic, el driver del Kobuki, el driver del RPLIDAR A2M8 y rosbridge, todo levantado por `tb2_real.launch`. El contenedor usa `network_mode: host`, así que rosbridge queda expuesto en el puerto 9090 de la IP de la Jetson.

El **cerebro se conecta a rosbridge por websocket**. Su variable de entorno `ROBOT_HOST` decide a quién habla: si no se define, apunta a `localhost` (Gazebo); si se define con la IP de la Jetson, controla el robot real. El código del cerebro no sabe si los datos vienen de la simulación o del hardware, porque ambos publican los mismos tópicos (`/scan`, `/odom`, `/mobile_base/commands/velocity`).

| Pieza | En simulación | En el robot real |
|---|---|---|
| Quién publica `/scan` | Plugin de Gazebo | Driver `rplidar_ros` |
| Quién mueve las ruedas | Plugin de Gazebo | Driver del Kobuki |
| Dónde corre rosbridge | Contenedor `sim` | Contenedor `robot` en la Jetson |
| Cerebro | Contenedor `brain`, `ROBOT_HOST` vacío | Contenedor `brain`, `ROBOT_HOST` con la IP de la Jetson |

---

## Bitácora por sesiones

### Sesión 1. Primer contacto con ROS en Docker (mediados de septiembre, aprox.)

**Objetivo.** Entender ROS, Gazebo y su relación practicando con las manos, y decidir en qué entorno trabajar.

**Qué se hizo.** Se corrió la imagen oficial `osrf/ros:melodic-desktop-full` y se practicó con turtlesim, `rostopic` y `rqt_graph` para ver en vivo los conceptos base: nodos, tópicos, mensajes y el `roscore` como la operadora que conecta a todos. Se estableció que Gazebo y ROS son programas separados, unidos por los plugins de `gazebo_ros`, y que RViz no es Gazebo sino una ventana a lo que ROS cree del mundo.

**Problemas y decisiones.** Apareció el conflicto central: Melodic usa Python 2.7 y PyTorch y YOLO exigen Python 3. WSL quedó descartado por dos razones, la red NAT de WSL2 choca con las conexiones bidireccionales de ROS 1 y Gazebo no tendría aceleración por GPU con Ubuntu 18.04. La máquina virtual quedó descartada por la falta de aceleración 3D.

**Resultado.** Se adoptó la arquitectura de partición con Ubuntu moderno más contenedores Docker.

### Sesión 2. Entorno de simulación (23 de septiembre, aprox.)

**Objetivo.** Tener el TurtleBot 2 simulado con su sensor y un primer programa que lo controle.

**Qué se hizo.** Se escribieron los Dockerfiles de `sim` (Melodic, Gazebo 9 y los paquetes del TurtleBot 2, compilados en parte desde forks comunitarios porque los oficiales se quedaron en Kinetic) y de `brain`, más el `docker-compose.yml` con red del host, GPU por NVIDIA Container Toolkit y servidor gráfico compartido. Se creó el paquete `tb2_sim` con el URDF del robot incluyendo el RPLIDAR y su launch. Se escribió `evasor.py`, un evasor reactivo que lee `/scan` y publica velocidades por rosbridge. La ejecución quedó en tres terminales: mundo (`roscore`, `gzserver`, `gzclient`), traductor (rosbridge) y cerebro.

**Problemas y soluciones.** Acentos en archivos que Melodic no acepta, Gazebo cerrándose al seleccionar el robot, un aviso de libcurl sin consecuencias y la llave vencida del repositorio de ROS. Hallazgo importante para el futuro: las personas animadas de Gazebo 9 (actors) no tienen cuerpo físico y el LiDAR simulado por física no las detecta; para simular a la persona a seguir habrá que cambiar el sensor a `gpu_ray`.

**Resultado.** El robot simulado se mueve controlado desde el contenedor `brain`. Primer commit: *TurtleBot 2 con RPLIDAR en Gazebo controlado desde el cerebro*.

### Sesión 3. Preparación para el robot real (30 de septiembre, aprox.)

**Objetivo.** Que el mismo cerebro pueda controlar el robot físico sin reescribir código.

**Qué se hizo.** Se creó el paquete `tb2_real`, cuyo launch es el equivalente real de la simulación: arranca el driver del Kobuki, el del RPLIDAR, la transformación del LiDAR sobre la base y rosbridge. El cerebro pasó a leer la dirección del robot desde `ROBOT_HOST`. Se preparó un script de prueba de solo lectura y se escribió el README con la arquitectura, el setup desde cero y los problemas conocidos. El repositorio se compartió con el equipo.

**Resultado.** Commit *Paquete tb2_real y host configurable en el cerebro*. Se acordó que con el robot real lo primero siempre sería una prueba de solo lectura, antes de mandar cualquier comando de movimiento.

### Sesión 4. Primer laboratorio (7 de octubre, aprox.)

**Objetivo.** Conocer el robot físico asignado, el TurtleBot 2 unidad 1405.

**Qué se hizo.** Se conectó el Kobuki por USB A-B directamente a la laptop y, desde el contenedor `sim`, se siguió la guía oficial hasta teleoperar el robot con `turtlebot_bringup minimal.launch`.

**Resultado.** Primera comunicación real con la base Kobuki desde el entorno del equipo.

### Sesión 5. Plan de despliegue en la Jetson (principios de octubre, aprox.)

**Objetivo.** Definir cómo correría ROS a bordo del robot.

**Qué se hizo.** Se estableció que las imágenes de la laptop no corren en la Jetson porque fueron construidas para x86 y la Jetson es ARM, y que la Jetson no necesita nada de simulación, solo drivers y rosbridge. El plan inicial fue instalar ROS Melodic directamente en la Jetson.

**Problemas y decisiones.** No se conocían el usuario ni la contraseña de la Jetson. Se decidió reflashearla con JetPack 4.6.x, que es la forma de "instalar Ubuntu 18.04" en esa tarjeta: se conecta por micro USB a una computadora anfitriona, se pone en modo recovery y se le copia el sistema, lo que borra todo y permite crear un usuario nuevo.

**Resultado.** Plan de reflasheo definido.

### Sesión 6. Reflasheo y contenedor del robot (14 de octubre)

**Objetivo.** Dejar la Jetson operativa y con el entorno del robot.

**Qué se hizo.** Se reflasheó la Jetson con JetPack 4.6.6 ejecutando las herramientas de NVIDIA desde un contenedor en la laptop. En lugar de instalar ROS nativo, se mantuvo el mismo patrón reproducible de la laptop: se creó `robot/Dockerfile` y `docker-compose.jetson.yml`, y la imagen se construyó en la Jetson compilando los paquetes del Kobuki desde fuente, porque no existen como binarios para Melodic. El repositorio se clonó con una deploy key de solo lectura.

**Problemas y soluciones.** Errores de `flash.sh` (permisos de root, `cpio`, modo recovery y `ssh-keygen`), incompatibilidad de la versión del archivo compose, confusión entre `172.17.0.1` (la red interna de Docker) y la IP real de la Jetson, y configuración de la red WiFi.

**Resultado.** Jetson operativa con el contenedor del robot. Commits *Nuevo dockerfile para instalar dependencias a la Jetson TX2*, *Actualizacion de dockerfile para Jetson* y *Versionamiento de docker*. Se cerró la tarea KAN-11.

### Sesión 7. Diagnóstico de sensores en la Jetson (21 de octubre)

**Objetivo.** Que el Kobuki, el LiDAR y rosbridge funcionen dentro del contenedor del robot.

**Qué se hizo.** Se probó el RPLIDAR, que no podía abrir su puerto serial ("cannot bind"). Se fijaron nombres estables para los puertos (`/dev/kobuki` y `/dev/rplidar`). Se estableció que el workspace `/root/ros_ws` es un volumen montado que hay que compilar con `catkin_make` dentro del contenedor después de cada clon limpio, porque `build/` y `devel/` no se versionan. Finalmente se levantó `tb2_real.launch`.

**Problemas y soluciones.** El fallo del LiDAR parecía de hardware y resultó ser de permisos: el usuario no pertenecía al grupo `dialout`. Al arrancar aparecieron mensajes "malformed sub-payload" del Kobuki; se analizaron y se concluyó que son paquetes corruptos esporádicos que el driver descarta, sin efecto mientras no se vuelvan continuos. Quedaron dos avisos sin consecuencia: falta de descripción URDF (solo afecta la visualización en RViz) y firmware 1.1.3 del Kobuki frente a 1.1.4 o 1.2.0 recomendados; se decidió no actualizar el firmware para no arriesgar la base a mitad del semestre.

**Resultado.** El Kobuki responde con su melodía de conexión, el LiDAR arranca sano (modo Express, 12 m de alcance) y rosbridge escucha en el puerto 9090.

### Sesión 8. Primer movimiento del robot real (28 de octubre)

**Objetivo.** Conectar el cerebro de la laptop con la Jetson y mover el robot.

**Qué se hizo.** Se conectaron laptop y Jetson a la misma red (un hotspot de celular) y se ejecutó `evasor.py` desde el contenedor `brain`. El evasor avanza a 0.05 m/s mientras el obstáculo más cercano dentro de un cono de ±15° al frente esté a más de 0.6 m, y gira en su lugar a 0.3 rad/s cuando no.

**Problemas y soluciones.** El primer intento falló con `RosTimeoutError`. Una prueba directa del puerto devolvió "Connection refused", que indicaba que sí se llegaba a una máquina pero nadie escuchaba en el 9090. Con `hostname -I` en ambos equipos se encontró la causa: `ROBOT_HOST` apuntaba a `172.20.10.7`, que era la propia laptop, mientras la Jetson era `172.20.10.8`. Ya en movimiento, el robot se comportaba "al revés": avanzaba hacia obstáculos y giraba cuando alguien pasaba por detrás. La causa fue que el 0° del LiDAR apuntaba hacia la parte trasera del robot, así que el cono de ±15° vigilaba lo que tenía detrás. Se corrigió girando físicamente el LiDAR 180° para alinear su 0° con el frente del Kobuki.

**Resultado.** Primer movimiento autónomo del robot real, controlado de punta a punta: LiDAR, Jetson, rosbridge, WiFi, cerebro en la laptop y de regreso a las ruedas. Dos notas para las siguientes sesiones: las IPs del hotspot cambian al reconectarse, así que hay que revisar `hostname -I` en la Jetson al inicio de cada sesión, y el evasor todavía no tiene el freno de seguridad que detiene el robot si deja de llegar `/scan`.

---

## Siguientes pasos: la arquitectura actual es temporal

La arquitectura vigente sirve para desarrollar y probar, pero no es la del sistema final. La prueba real del proyecto será con la red del DQN ya entrenada y YOLO ya entrenado corriendo directamente en la Jetson, para que el robot sea autónomo y transportable. En esa configuración, la laptop deja de ser el cerebro y se usa solo para entrenar en simulación, donde sí se conservan los contenedores `sim` y `brain` con rosbridge local.

**Dirección elegida: todo en la Jetson.** Se agregaría a la Jetson un segundo contenedor de cerebro, construido para ARM, junto al contenedor `robot` que ya existe. El reto no es de potencia sino de versiones: JetPack 4.6 trae Python 3.6, PyTorch para esa versión solo existe en Python 3.6, Ultralytics requiere Python 3.8 y ROS Melodic piensa en Python 2.7. La ruta prevista para cada pieza es la siguiente.

Para **YOLO**, entrenar en la laptop, exportar a ONNX y convertir en la Jetson a un motor de TensorRT que corre con el Python 3.6 nativo, sin necesidad de Ultralytics ni PyTorch en el robot. Es además la forma más eficiente de usar la GPU de la TX2. Hay que verificar que la versión de YOLO elegida exporte bien a TensorRT 8.2; si no, el plan B son contenedores comunitarios con Ultralytics para JetPack 4.6.

Para el **DQN**, la red es pequeña y su inferencia cuesta microsegundos, así que no representa carga relevante.

Para la **comunicación con ROS**, el primer paso es que el cerebro en la Jetson use el rosbridge de la misma Jetson en `localhost`, con el mismo código actual cambiando solo `ROBOT_HOST`. Dentro de la misma máquina rosbridge ya no viaja por WiFi, y su latencia se medirá en las pruebas de latencia y sensores (KAN-17). Si resulta aceptable se conserva; si no, se reemplaza por un nodo nativo con `rospy` en Python 3, que en Melodic es viable para mensajes estándar como `LaserScan` y `Twist`.

Concentrar todo en la Jetson también fortalece la hipótesis del proyecto: percepción y planificación compitiendo por el mismo procesador y el mismo presupuesto de tiempo es justo el compromiso que el proyecto busca optimizar, y en una sola tarjeta ese conflicto existe y se puede medir.

**Alternativa considerada: Raspberry Pi.** Se evaluó agregar una Raspberry Pi como cerebro junto a la Jetson, o pivotear todo a la Pi. La primera opción no resuelve nada que un contenedor no resuelva, y agrega peso, consumo, una segunda computadora y un salto de red con más latencia. La segunda pierde la GPU, lo que dejaría a YOLO en CPU a pocos cuadros por segundo. La Pi solo se retomaría si las mediciones muestran que la Jetson no alcanza a correr YOLO y el DQN dentro del presupuesto de tiempo, e incluso entonces la primera respuesta sería un modelo de YOLO más pequeño o una imagen de menor resolución antes que agregar hardware.

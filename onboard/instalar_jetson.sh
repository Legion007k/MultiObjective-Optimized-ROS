#!/usr/bin/env bash
# Prepara la Jetson para el cerebro nativo (Python 3.6 de JetPack 4.6).
# Se puede correr varias veces: salta lo que ya esta hecho. Solo usa sudo para udev.
#   bash onboard/instalar_jetson.sh
set -euo pipefail

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$RAIZ"
ASTRA_COMMIT=210f530144314da7c70072215bacba12130d9b20  # el mismo de robot/Dockerfile
VENDOR=onboard/vendor
REGLAS=56-orbbec-usb.rules
TMP=""
trap 'rm -rf "$TMP"' EXIT

# 1. Paquetes de Python. Se usa "python3 -m pip" porque el pip3 de Ubuntu 18.04
#    deja de funcionar despues de actualizar pip.
if ! python3 -m pip --version > /dev/null 2>&1 || ! python3 -c \
    'import os, sysconfig; assert os.path.exists(os.path.join(sysconfig.get_paths()["include"], "Python.h"))' 2> /dev/null; then
    echo "Falta pip o los encabezados de Python: sudo apt-get install python3-pip python3-dev" >&2
    exit 1
fi
# pycuda se compila: necesita nvcc y las bibliotecas de CUDA
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH:-}
python3 -m pip install --user --upgrade "pip<22"
python3 -m pip install --user -r onboard/requirements.txt

# 2. OpenNI2 de Orbbec para arm64 y reglas de udev, del commit del contenedor robot
if [ ! -f "$VENDOR/openni2/libOpenNI2.so" ] || [ ! -f "$VENDOR/$REGLAS" ]; then
    TMP="$(mktemp -d)"
    git -C "$TMP" init -q
    git -C "$TMP" remote add origin https://github.com/orbbec/ros_astra_camera.git
    git -C "$TMP" fetch -q --depth 1 origin "$ASTRA_COMMIT"
    git -C "$TMP" checkout -q FETCH_HEAD
    mkdir -p "$VENDOR/openni2"
    cp -r "$TMP/include/openni2_redist/arm64/." "$VENDOR/openni2/"
    cp "$TMP/$REGLAS" "$VENDOR/"
fi
if ! cmp -s "$VENDOR/$REGLAS" "/etc/udev/rules.d/$REGLAS"; then
    sudo cp "$VENDOR/$REGLAS" /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "Reglas de udev instaladas: desconecta y conecta la camara."
fi

# 3. Motor de TensorRT para cada ONNX sin motor (o con un ONNX mas nuevo que su motor)
for onnx in models/*.onnx; do
    [ -e "$onnx" ] || continue
    engine="${onnx%.onnx}.engine"
    if [ "$onnx" -nt "$engine" ]; then
        echo "Construyendo $engine, tarda varios minutos..."
        /usr/src/tensorrt/bin/trtexec --onnx="$onnx" --saveEngine="$engine" --fp16
    fi
done

echo "Listo."

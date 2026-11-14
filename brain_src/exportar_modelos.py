"""Exporta los modelos que viajan a la Jetson. Corre en el contenedor brain con el
repositorio montado, para que la salida quede en models/ (ver docs/cerebro_jetson.md):

    docker compose run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -v "$PWD":/repo -w /repo brain python brain_src/exportar_modelos.py yolo --imgsz 320
    ... brain python brain_src/exportar_modelos.py dqn ruta/al/state_dict.pt
"""
import argparse
import os
import shutil
import tempfile

import numpy as np

from contrato import ACCIONES, N_RAYOS, mlp_numpy, ruta_repo


def exportar_yolo(imgsz):
    from ultralytics import YOLO

    salida = ruta_repo("models/yolov8n_%d.onnx" % imgsz)
    temporal = tempfile.mkdtemp()  # aqui se descargan los pesos .pt, fuera del repo
    try:
        modelo = YOLO(os.path.join(temporal, "yolov8n.pt"))
        onnx = modelo.export(format="onnx", opset=12, simplify=True, imgsz=imgsz,
                             batch=1, dynamic=False)
        os.makedirs(os.path.dirname(salida), exist_ok=True)
        shutil.move(str(onnx), salida)
    finally:
        shutil.rmtree(temporal, ignore_errors=True)
    print("ONNX guardado en " + salida)


def exportar_dqn(ruta_pt):
    import torch

    estado = torch.load(ruta_pt, map_location="cpu")
    # Capas Linear en orden: cada peso 2D va seguido de su sesgo
    capas = [(k, k[:-len("weight")] + "bias") for k, t in estado.items()
             if k.endswith("weight") and t.dim() == 2]
    pesos = {}
    for i, (k_w, k_b) in enumerate(capas):
        pesos["W%d" % i] = estado[k_w].detach().float().numpy().T.copy()
        pesos["b%d" % i] = estado[k_b].detach().float().numpy().copy()

    entradas = pesos["W0"].shape[0]
    salidas = pesos["W%d" % (len(capas) - 1)].shape[1]
    if entradas != N_RAYOS + 6 or salidas != len(ACCIONES):
        print("AVISO: la red es %d -> %d y el contrato pide %d -> %d"
              % (entradas, salidas, N_RAYOS + 6, len(ACCIONES)))

    # Verificacion: la misma entrada aleatoria en PyTorch y en numpy
    x = torch.randn(1, entradas)
    h = x
    for i, (k_w, k_b) in enumerate(capas):
        h = torch.nn.functional.linear(h, estado[k_w].float(), estado[k_b].float())
        if i < len(capas) - 1:
            h = torch.relu(h)
    q_torch = h.detach().numpy()
    q_numpy = mlp_numpy(pesos, x.numpy())
    diferencia = float(np.max(np.abs(q_torch - q_numpy)))
    if diferencia > 1e-4:
        raise SystemExit("ERROR: numpy y PyTorch difieren en %g" % diferencia)

    salida = ruta_repo("models/dqn.npz")
    os.makedirs(os.path.dirname(salida), exist_ok=True)
    np.savez(salida, **pesos)
    print("%d capas guardadas en %s (diferencia maxima con PyTorch %.2g)"
          % (len(capas), salida, diferencia))


def main():
    parser = argparse.ArgumentParser(description="Exporta modelos para la Jetson")
    sub = parser.add_subparsers(dest="modelo", required=True)
    p_yolo = sub.add_parser("yolo", help="YOLOv8n a ONNX (opset 12, batch 1)")
    p_yolo.add_argument("--imgsz", type=int, default=320)
    p_dqn = sub.add_parser("dqn", help="state_dict de un MLP a models/dqn.npz")
    p_dqn.add_argument("ruta_pt")
    args = parser.parse_args()
    if args.modelo == "yolo":
        exportar_yolo(args.imgsz)
    else:
        exportar_dqn(args.ruta_pt)


if __name__ == "__main__":
    main()

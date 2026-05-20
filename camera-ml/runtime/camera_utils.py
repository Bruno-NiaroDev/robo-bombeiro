import cv2
import re
from pathlib import Path

from config import ZOOM_DIGITAL


def aplicar_zoom_digital(frame, zoom=None):
    if zoom is None:
        zoom = ZOOM_DIGITAL

    if zoom <= 1.0:
        return frame

    altura, largura = frame.shape[:2]

    nova_largura = int(largura / zoom)
    nova_altura = int(altura / zoom)

    x1 = (largura - nova_largura) // 2
    y1 = (altura - nova_altura) // 2
    x2 = x1 + nova_largura
    y2 = y1 + nova_altura

    crop = frame[y1:y2, x1:x2]
    frame_zoom = cv2.resize(crop, (largura, altura))

    return frame_zoom


def salvar_zoom_no_config(zoom):
    caminho_config = Path(__file__).parent / "config.py"

    texto = caminho_config.read_text(encoding="utf-8")

    if re.search(r"ZOOM_DIGITAL\s*=\s*[0-9.]+", texto):
        novo_texto = re.sub(
            r"ZOOM_DIGITAL\s*=\s*[0-9.]+",
            f"ZOOM_DIGITAL = {zoom:.1f}",
            texto
        )
    else:
        novo_texto = texto + f"\nZOOM_DIGITAL = {zoom:.1f}\n"

    caminho_config.write_text(novo_texto, encoding="utf-8")

    print(f"Zoom salvo no config.py: {zoom:.1f}")
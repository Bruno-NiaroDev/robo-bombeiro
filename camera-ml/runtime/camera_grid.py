import cv2
from config import GRID_SIZE


def dividir_em_blocos(frame):
    h, w, _ = frame.shape

    bloco_h = h // GRID_SIZE
    bloco_w = w // GRID_SIZE

    blocos = {}

    numero = 1

    for linha in range(GRID_SIZE):
        for coluna in range(GRID_SIZE):
            x1 = coluna * bloco_w
            y1 = linha * bloco_h
            x2 = (coluna + 1) * bloco_w
            y2 = (linha + 1) * bloco_h

            nome_bloco = f"B{numero:02d}"

            crop = frame[y1:y2, x1:x2]

            blocos[nome_bloco] = {
                "crop": crop,
                "coords_img": (x1, y1, x2, y2),
                "linha": linha,
                "coluna": coluna
            }

            numero += 1

    return blocos


def desenhar_grid(frame, resultados=None, bloco_destacado=None):
    blocos = dividir_em_blocos(frame)

    for nome_bloco, dados in blocos.items():
        x1, y1, x2, y2 = dados["coords_img"]

        cor = (0, 255, 0)

        if bloco_destacado == nome_bloco:
            cor = (0, 0, 255)

        cv2.rectangle(frame, (x1, y1), (x2, y2), cor, 2)

        texto = nome_bloco

        if resultados and nome_bloco in resultados:
            texto += f" {resultados[nome_bloco]:.2f}"

        cv2.putText(
            frame,
            texto,
            (x1 + 8, y1 + 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            cor,
            2
        )

    return frame
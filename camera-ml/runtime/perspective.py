import cv2
import json
import numpy as np

ARQUIVO_PONTOS = "perspective_points.json"
TAMANHO_SAIDA = 800


def carregar_pontos():
    with open(ARQUIVO_PONTOS, "r", encoding="utf-8") as arquivo:
        dados = json.load(arquivo)

    pontos = np.float32(dados["pontos_origem"])
    return pontos


def corrigir_perspectiva(frame):
    pontos_origem = carregar_pontos()

    pontos_destino = np.float32([
        [0, 0],
        [TAMANHO_SAIDA, 0],
        [TAMANHO_SAIDA, TAMANHO_SAIDA],
        [0, TAMANHO_SAIDA],
    ])

    matriz = cv2.getPerspectiveTransform(
        pontos_origem,
        pontos_destino
    )

    imagem_corrigida = cv2.warpPerspective(
        frame,
        matriz,
        (TAMANHO_SAIDA, TAMANHO_SAIDA)
    )

    return imagem_corrigida
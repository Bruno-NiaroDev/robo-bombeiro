import cv2
import os
from pathlib import Path
from datetime import datetime

from config import CAMERA_INDEX
from perspective import corrigir_perspectiva
from camera_grid import desenhar_grid
from camera_utils import aplicar_zoom_digital


DATASET_BASE = Path.home() / "Desktop" / "robo_incendio" / "dataset_original"

EXTENSOES_VALIDAS = (".jpg", ".jpeg", ".png")


def criar_pastas_se_necessario():
    os.makedirs(DATASET_BASE / "vazio", exist_ok=True)

    for estado in ["vela_acesa", "vela_apagada"]:
        for i in range(1, 17):
            bloco = f"B{i:02d}"
            os.makedirs(DATASET_BASE / estado / bloco, exist_ok=True)


def validar_bloco(bloco):
    blocos_validos = [f"B{i:02d}" for i in range(1, 17)]
    return bloco in blocos_validos


def obter_destino(estado, bloco=None):
    if estado == "vazio":
        return DATASET_BASE / "vazio"

    return DATASET_BASE / estado / bloco


def salvar_foto(frame_corrigido, estado, bloco=None):
    destino = obter_destino(estado, bloco)
    os.makedirs(destino, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    if estado == "vazio":
        nome = f"vazio_{timestamp}.jpg"
    else:
        nome = f"{estado}_{bloco}_{timestamp}.jpg"

    caminho = destino / nome

    cv2.imwrite(str(caminho), frame_corrigido)

    print(f"Foto salva: {caminho}")


def contar_imagens(pasta):
    if not os.path.exists(pasta):
        return 0

    return len([
        arquivo for arquivo in os.listdir(pasta)
        if arquivo.lower().endswith(EXTENSOES_VALIDAS)
    ])


def escolher_modo_captura():
    print("\n=== CAPTURA DE DATASET - ROBÔ INCÊNDIO ===")
    print("1 - Ambiente vazio")
    print("2 - Vela acesa")
    print("3 - Vela apagada")

    opcao = input("Escolha o tipo de foto: ").strip()

    if opcao == "1":
        return "vazio", None

    if opcao == "2":
        bloco = input("Informe o bloco da vela acesa. Ex: B01 até B16: ").strip().upper()

        if not validar_bloco(bloco):
            print("Bloco inválido.")
            return None, None

        return "vela_acesa", bloco

    if opcao == "3":
        bloco = input("Informe o bloco da vela apagada. Ex: B01 até B16: ").strip().upper()

        if not validar_bloco(bloco):
            print("Bloco inválido.")
            return None, None

        return "vela_apagada", bloco

    print("Opção inválida.")
    return None, None


def main():
    criar_pastas_se_necessario()

    estado, bloco = escolher_modo_captura()

    if estado is None:
        return

    destino = obter_destino(estado, bloco)
    fotos_atuais = contar_imagens(destino)

    print("\nDestino das fotos:")
    print(destino)
    print(f"Fotos já existentes nesta pasta: {fotos_atuais}")

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print("Erro: não foi possível abrir a webcam.")
        print("Tente alterar CAMERA_INDEX no config.py para 0, 1, 2 ou 3.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("\nWebcam iniciada.")
    print("Comandos:")
    print("ESPAÇO = salvar foto")
    print("Q = sair")
    print("\nAs imagens serão salvas corrigidas e sem o grid verde.\n")

    contador_sessao = 0

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Erro ao capturar imagem.")
            break

        frame = aplicar_zoom_digital(frame)
        frame_corrigido = corrigir_perspectiva(frame)

        frame_preview = desenhar_grid(frame_corrigido.copy())

        texto_info = f"Modo: {estado}"

        if bloco:
            texto_info += f" | Bloco: {bloco}"

        texto_info += f" | Fotos nesta sessao: {contador_sessao}"

        cv2.putText(
            frame_preview,
            texto_info,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

        cv2.imshow("Captura Dataset Corrigida - Robo Incendio", frame_preview)

        tecla = cv2.waitKey(1) & 0xFF

        if tecla == ord(" "):
            salvar_foto(frame_corrigido, estado, bloco)
            contador_sessao += 1

        elif tecla in [ord("q"), ord("Q")]:
            break

    cap.release()
    cv2.destroyAllWindows()

    print("\nCaptura encerrada.")
    print(f"Fotos salvas nesta sessão: {contador_sessao}")


if __name__ == "__main__":
    main()
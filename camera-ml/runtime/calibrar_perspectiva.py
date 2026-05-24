import cv2
import json

from config import CAMERA_INDEX, ZOOM_DIGITAL
from camera_utils import aplicar_zoom_digital

ARQUIVO_PONTOS = "perspective_points.json"

pontos = []


def abrir_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print("Erro: não foi possível abrir a webcam com CAP_DSHOW.")
        print("Tentando abrir sem CAP_DSHOW...")
        cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print("Erro: não foi possível abrir a câmera.")
        print("Verifique CAMERA_INDEX no config.py.")
        return None

    return cap


def clique_mouse(event, x, y, flags, param):
    global pontos

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(pontos) < 4:
            pontos.append([x, y])
            print(f"Ponto {len(pontos)} capturado: x={x}, y={y}")

        if len(pontos) == 4:
            print("\n4 pontos capturados.")
            print("Pressione S para salvar ou R para refazer.")


def desenhar_pontos(frame):
    nomes = ["P1", "P2", "P3", "P4"]

    for i, ponto in enumerate(pontos):
        x, y = ponto

        cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)
        cv2.putText(
            frame,
            nomes[i],
            (x + 12, y - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2
        )

    if len(pontos) >= 2:
        for i in range(len(pontos) - 1):
            x1, y1 = pontos[i]
            x2, y2 = pontos[i + 1]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)

    if len(pontos) == 4:
        x1, y1 = pontos[3]
        x2, y2 = pontos[0]
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)

    cv2.rectangle(frame, (0, 0), (frame.shape[1], 45), (0, 0, 0), -1)

    cv2.putText(
        frame,
        "Clique: B01 -> B04 -> B16 -> B13 | S salva | R refaz | Q/ESC sai",
        (20, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2
    )

    cv2.putText(
        frame,
        f"Zoom: {float(ZOOM_DIGITAL):.1f}x",
        (20, frame.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2
    )

    return frame


def salvar_pontos():
    dados = {
        "pontos_origem": pontos
    }

    with open(ARQUIVO_PONTOS, "w", encoding="utf-8") as arquivo:
        json.dump(dados, arquivo, indent=4)

    print(f"\nPontos salvos em: {ARQUIVO_PONTOS}")
    print("Agora rode: python main_camera_preview_corrigido.py")


def main():
    global pontos

    cap = abrir_camera()

    if cap is None:
        return

    janela = "Calibracao de Perspectiva"
    cv2.namedWindow(janela)
    cv2.setMouseCallback(janela, clique_mouse)

    print("\n=== CALIBRAÇÃO DE PERSPECTIVA ===")
    print("Clique nos 4 cantos EXTERNOS do tapete nesta ordem:")
    print("P1 = canto externo do B01")
    print("P2 = canto externo do B04")
    print("P3 = canto externo do B16")
    print("P4 = canto externo do B13")
    print("\nS = salvar | R = refazer | Q ou ESC = sair")

    while True:
        ret, frame = cap.read()

        if not ret or frame is None:
            print("Erro ao capturar imagem da câmera.")
            break

        frame = aplicar_zoom_digital(frame)
        preview = desenhar_pontos(frame.copy())

        cv2.imshow(janela, preview)

        tecla = cv2.waitKey(1) & 0xFF

        if tecla in [ord("s"), ord("S")]:
            if len(pontos) == 4:
                salvar_pontos()
            else:
                print("Você precisa clicar nos 4 pontos antes de salvar.")

        elif tecla in [ord("r"), ord("R")]:
            pontos = []
            print("\nPontos apagados. Clique novamente nos 4 cantos.")

        elif tecla in [ord("q"), ord("Q"), 27]:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
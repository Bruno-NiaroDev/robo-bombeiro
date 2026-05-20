import cv2

from config import CAMERA_INDEX, ZOOM_DIGITAL
from camera_grid import desenhar_grid
from camera_utils import aplicar_zoom_digital, salvar_zoom_no_config


zoom_atual = float(ZOOM_DIGITAL)

BOTAO_MENOS = None
BOTAO_MAIS = None


def limitar_zoom(valor):
    if valor < 1.0:
        return 1.0

    if valor > 2.5:
        return 2.5

    return round(valor, 1)


def ponto_dentro_botao(x, y, botao):
    if botao is None:
        return False

    x1, y1, x2, y2 = botao

    return x1 <= x <= x2 and y1 <= y <= y2


def callback_mouse(event, x, y, flags, param):
    global zoom_atual

    if event != cv2.EVENT_LBUTTONDOWN:
        return

    if ponto_dentro_botao(x, y, BOTAO_MENOS):
        zoom_atual = limitar_zoom(zoom_atual - 0.1)
        print(f"Zoom atual: {zoom_atual:.1f}")

    elif ponto_dentro_botao(x, y, BOTAO_MAIS):
        zoom_atual = limitar_zoom(zoom_atual + 0.1)
        print(f"Zoom atual: {zoom_atual:.1f}")


def desenhar_controles_zoom(frame):
    global BOTAO_MENOS, BOTAO_MAIS

    h, w = frame.shape[:2]

    largura_botao = 70
    altura_botao = 45
    margem = 15

    y1 = margem
    y2 = y1 + altura_botao

    x_menos_1 = w - (2 * largura_botao) - (3 * margem)
    x_menos_2 = x_menos_1 + largura_botao

    x_mais_1 = w - largura_botao - margem
    x_mais_2 = x_mais_1 + largura_botao

    BOTAO_MENOS = (x_menos_1, y1, x_menos_2, y2)
    BOTAO_MAIS = (x_mais_1, y1, x_mais_2, y2)

    cv2.rectangle(frame, (x_menos_1, y1), (x_menos_2, y2), (40, 40, 40), -1)
    cv2.rectangle(frame, (x_mais_1, y1), (x_mais_2, y2), (40, 40, 40), -1)

    cv2.rectangle(frame, (x_menos_1, y1), (x_menos_2, y2), (0, 255, 255), 2)
    cv2.rectangle(frame, (x_mais_1, y1), (x_mais_2, y2), (0, 255, 255), 2)

    cv2.putText(
        frame,
        "-",
        (x_menos_1 + 25, y1 + 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (0, 255, 255),
        3
    )

    cv2.putText(
        frame,
        "+",
        (x_mais_1 + 22, y1 + 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.2,
        (0, 255, 255),
        3
    )

    texto_zoom = f"Zoom: {zoom_atual:.1f}x | S salva | Q sai"

    cv2.putText(
        frame,
        texto_zoom,
        (20, h - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    return frame


def main():
    global zoom_atual

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print("Erro: não foi possível abrir a webcam.")
        print("Tente alterar CAMERA_INDEX no config.py para 0, 1, 2 ou 3.")
        return

    janela = "Preview Webcam - Grid 4x4"

    cv2.namedWindow(janela)
    cv2.setMouseCallback(janela, callback_mouse)

    print("Webcam iniciada com sucesso.")
    print("Controles:")
    print("Clique em + para aproximar")
    print("Clique em - para diminuir o zoom")
    print("Tecla + também aproxima")
    print("Tecla - também diminui")
    print("Tecla S salva o zoom no config.py")
    print("Tecla Q sai")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Erro ao capturar imagem da webcam.")
            break

        frame = aplicar_zoom_digital(frame, zoom_atual)

        frame_com_grid = desenhar_grid(frame.copy())
        frame_com_grid = desenhar_controles_zoom(frame_com_grid)

        cv2.imshow(janela, frame_com_grid)

        tecla = cv2.waitKey(1) & 0xFF

        if tecla in [ord("+"), ord("=")]:
            zoom_atual = limitar_zoom(zoom_atual + 0.1)
            print(f"Zoom atual: {zoom_atual:.1f}")

        elif tecla in [ord("-"), ord("_")]:
            zoom_atual = limitar_zoom(zoom_atual - 0.1)
            print(f"Zoom atual: {zoom_atual:.1f}")

        elif tecla == ord("s"):
            salvar_zoom_no_config(zoom_atual)

        elif tecla == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
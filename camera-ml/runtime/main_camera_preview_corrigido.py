import cv2

from config import CAMERA_INDEX
from perspective import corrigir_perspectiva
from camera_grid import desenhar_grid
from camera_utils import aplicar_zoom_digital


def abrir_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print("Erro com CAP_DSHOW. Tentando abrir sem CAP_DSHOW...")
        cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print("Erro: não foi possível abrir a câmera.")
        return None

    return cap


def main():
    cap = abrir_camera()

    if cap is None:
        return

    print("Preview corrigido iniciado.")
    print("Pressione Q ou ESC para sair.")

    while True:
        ret, frame = cap.read()

        if not ret or frame is None:
            print("Erro ao capturar imagem.")
            break

        frame = aplicar_zoom_digital(frame)
        frame_corrigido = corrigir_perspectiva(frame)
        frame_com_grid = desenhar_grid(frame_corrigido.copy())

        cv2.imshow("Imagem Corrigida - Grid 4x4", frame_com_grid)

        tecla = cv2.waitKey(1) & 0xFF

        if tecla in [ord("q"), ord("Q"), 27]:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
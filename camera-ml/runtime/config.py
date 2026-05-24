GRID_SIZE = 4

AMBIENTE_LARGURA_M = 1.5
AMBIENTE_ALTURA_M = 1.5

BLOCO_LARGURA_M = AMBIENTE_LARGURA_M / GRID_SIZE
BLOCO_ALTURA_M = AMBIENTE_ALTURA_M / GRID_SIZE

# 0 geralmente é a câmera do notebook
# 1 geralmente é a webcam USB externa
# se não funcionar, teste 2 ou 3
CAMERA_INDEX = 1

CAPTURA_INTERVALO_SEGUNDOS = 3

THRESHOLD_FOGO = 0.70

# Zoom digital:
# 1.0 = sem zoom
# 1.2 = zoom leve
# 1.5 = zoom médio
# Use aqui o valor que você escolheu no preview
ZOOM_DIGITAL = 1.0
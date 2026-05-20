from config import GRID_SIZE, BLOCO_LARGURA_M, BLOCO_ALTURA_M, AMBIENTE_ALTURA_M


def gerar_coordenadas_blocos():
    coordenadas = {}

    numero = 1

    for linha in range(GRID_SIZE):
        for coluna in range(GRID_SIZE):
            bloco = f"B{numero:02d}"

            x = (coluna * BLOCO_LARGURA_M) + (BLOCO_LARGURA_M / 2)

            # B01 está no topo da imagem.
            # O eixo Y físico começa embaixo, então invertemos o Y.
            y = AMBIENTE_ALTURA_M - ((linha * BLOCO_ALTURA_M) + (BLOCO_ALTURA_M / 2))

            coordenadas[bloco] = {
                "x": round(x, 4),
                "y": round(y, 4)
            }

            numero += 1

    return coordenadas


COORDENADAS_BLOCOS = gerar_coordenadas_blocos()


def obter_coordenada_bloco(bloco):
    return COORDENADAS_BLOCOS.get(bloco)


if __name__ == "__main__":
    print("Coordenadas dos blocos do ambiente 4x4:\n")

    for bloco, coord in COORDENADAS_BLOCOS.items():
        print(f"{bloco} -> x={coord['x']}m | y={coord['y']}m")
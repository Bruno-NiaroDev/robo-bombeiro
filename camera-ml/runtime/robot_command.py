from coordinates import obter_coordenada_bloco


def gerar_comando_robo(resultado_ml):
    """
    Converte a saída do Machine Learning em comando lógico para o robô.
    """

    if not resultado_ml.get("fogo_detectado", False):
        return {
            "acao": "ambiente_seguro",
            "fogo_detectado": False,
            "bloco": None,
            "x": None,
            "y": None,
            "probabilidade": resultado_ml.get("probabilidade", 0.0)
        }

    bloco = resultado_ml.get("bloco")
    coordenada = obter_coordenada_bloco(bloco)

    if coordenada is None:
        return {
            "acao": "erro_bloco_invalido",
            "fogo_detectado": False,
            "bloco": bloco,
            "x": None,
            "y": None,
            "probabilidade": resultado_ml.get("probabilidade", 0.0)
        }

    return {
        "acao": "ir_para_fogo",
        "fogo_detectado": True,
        "bloco": bloco,
        "x": coordenada["x"],
        "y": coordenada["y"],
        "probabilidade": resultado_ml.get("probabilidade", 0.0)
    }


def imprimir_comando(comando):
    print("\n=== COMANDO GERADO PARA O ROBÔ ===")
    print("Ação:", comando["acao"])
    print("Fogo detectado:", comando["fogo_detectado"])
    print("Bloco:", comando["bloco"])
    print("X:", comando["x"])
    print("Y:", comando["y"])
    print("Probabilidade:", comando["probabilidade"])

    if "latencia_ms" in comando:
        print("Latência:", comando["latencia_ms"], "ms")

    print("==================================\n")

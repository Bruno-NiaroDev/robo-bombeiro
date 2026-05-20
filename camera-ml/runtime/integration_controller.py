import json
import time
from pathlib import Path

from robot_command import gerar_comando_robo, imprimir_comando


LOG_DIR = Path("logs")
LOG_DIR.mkdir(exist_ok=True)

LOG_COMANDOS = LOG_DIR / "comandos_robo.jsonl"


def detectar_fogo_simulado(bloco="B07", probabilidade=0.91):
    """
    Simula a saída do modelo de Machine Learning.
    Futuramente será substituída pela inferência real do modelo treinado.
    """

    if bloco is None:
        return {
            "fogo_detectado": False,
            "bloco": None,
            "probabilidade": 0.05
        }

    return {
        "fogo_detectado": True,
        "bloco": bloco,
        "probabilidade": probabilidade
    }


def enviar_comando_para_robo(comando):
    """
    Camada de envio do comando ao robô.

    Nesta versão, o envio é simulado e registrado em arquivo JSONL.
    Futuramente pode usar Serial, WebSocket, HTTP ou MQTT.
    """

    with open(LOG_COMANDOS, "a", encoding="utf-8") as arquivo:
        arquivo.write(json.dumps(comando, ensure_ascii=False) + "\n")

    print("Comando registrado em:", LOG_COMANDOS)

    return True


def executar_ciclo_integracao(bloco_simulado="B07", probabilidade=0.91):
    inicio = time.perf_counter()

    resultado_ml = detectar_fogo_simulado(
        bloco=bloco_simulado,
        probabilidade=probabilidade
    )

    comando = gerar_comando_robo(resultado_ml)

    fim = time.perf_counter()

    latencia_ms = (fim - inicio) * 1000
    comando["latencia_ms"] = round(latencia_ms, 4)

    imprimir_comando(comando)

    enviar_comando_para_robo(comando)

    return comando


def main():
    print("Sistema de integração ML -> Robô iniciado.")
    print("Modo atual: inferência simulada.")

    print("\nCenário de teste: fogo detectado no bloco B07.")
    executar_ciclo_integracao("B07", 0.91)

    print("\nCenário de teste: ambiente seguro.")
    executar_ciclo_integracao(None, 0.05)


if __name__ == "__main__":
    main()

import csv
import time
from pathlib import Path

from robot_command import gerar_comando_robo


RESULTS_DIR = Path("resultados")
RESULTS_DIR.mkdir(exist_ok=True)

CSV_PATH = RESULTS_DIR / "resultado_latencia.csv"


CASOS_TESTE = [
    {
        "nome": "Ambiente seguro",
        "resultado_ml": {
            "fogo_detectado": False,
            "bloco": None,
            "probabilidade": 0.05
        }
    },
    {
        "nome": "Fogo no B01",
        "resultado_ml": {
            "fogo_detectado": True,
            "bloco": "B01",
            "probabilidade": 0.88
        }
    },
    {
        "nome": "Fogo no B06",
        "resultado_ml": {
            "fogo_detectado": True,
            "bloco": "B06",
            "probabilidade": 0.93
        }
    },
    {
        "nome": "Fogo no B07",
        "resultado_ml": {
            "fogo_detectado": True,
            "bloco": "B07",
            "probabilidade": 0.91
        }
    },
    {
        "nome": "Fogo no B16",
        "resultado_ml": {
            "fogo_detectado": True,
            "bloco": "B16",
            "probabilidade": 0.95
        }
    }
]


def medir_latencia(caso):
    inicio = time.perf_counter()

    comando = gerar_comando_robo(caso["resultado_ml"])

    fim = time.perf_counter()

    latencia_ms = (fim - inicio) * 1000

    return {
        "cenario": caso["nome"],
        "fogo_detectado": caso["resultado_ml"]["fogo_detectado"],
        "entrada_bloco": caso["resultado_ml"]["bloco"],
        "acao": comando["acao"],
        "saida_bloco": comando["bloco"],
        "x": comando["x"],
        "y": comando["y"],
        "probabilidade": comando["probabilidade"],
        "latencia_ms": round(latencia_ms, 4)
    }


def main():
    resultados = []

    print("Executando testes de latência da integração ML -> Robô...\n")

    for caso in CASOS_TESTE:
        resultado = medir_latencia(caso)
        resultados.append(resultado)
        print(resultado)

    with open(CSV_PATH, "w", newline="", encoding="utf-8") as arquivo:
        campos = [
            "cenario",
            "fogo_detectado",
            "entrada_bloco",
            "acao",
            "saida_bloco",
            "x",
            "y",
            "probabilidade",
            "latencia_ms"
        ]

        writer = csv.DictWriter(arquivo, fieldnames=campos)
        writer.writeheader()
        writer.writerows(resultados)

    print("\nArquivo gerado:", CSV_PATH)


if __name__ == "__main__":
    main()

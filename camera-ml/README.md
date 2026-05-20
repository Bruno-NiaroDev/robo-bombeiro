# Camera ML - Robô Bombeiro

Módulo de visão computacional, Machine Learning e integração com o robô do projeto **Robô Bombeiro**.

Este módulo é responsável por capturar imagens do ambiente, corrigir perspectiva da câmera, dividir o chão em blocos, identificar a região com fogo e transformar essa informação em comando para o robô.

---

## Objetivo do módulo

O objetivo do `camera-ml` é integrar a visão computacional com o controle do robô.

Fluxo geral:

```text
Webcam
↓
Correção de perspectiva
↓
Divisão do ambiente em grid 4x4
↓
Detecção do bloco com fogo
↓
Conversão do bloco em coordenada física
↓
Geração de comando para o robô
↓
Robô se desloca até a região indicada
```

---

## Ambiente físico

O ambiente simula o chão de uma fábrica.

- Tamanho total: **1,5m x 1,5m**
- Divisão: **4x4**
- Total: **16 blocos**
- Tamanho de cada bloco: **37,5cm x 37,5cm**

Mapa dos blocos:

```text
B01 | B02 | B03 | B04
B05 | B06 | B07 | B08
B09 | B10 | B11 | B12
B13 | B14 | B15 | B16
```

A câmera é posicionada em ângulo diagonal. Por isso, o sistema utiliza correção de perspectiva para transformar a imagem capturada em uma visão superior do tapete.

---

## Estrutura do módulo

```text
camera-ml/
├── runtime/
│   ├── calibrar_perspectiva.py
│   ├── camera_grid.py
│   ├── camera_utils.py
│   ├── capture_dataset.py
│   ├── config.py
│   ├── coordinates.py
│   ├── integration_controller.py
│   ├── main_camera_preview.py
│   ├── main_camera_preview_corrigido.py
│   ├── main_detection_stub.py
│   ├── perspective.py
│   ├── requirements.txt
│   ├── robot_command.py
│   └── test_latency.py
│
├── docs/
│   └── entrega_12_05_2026.md
│
├── training/
└── README.md
```

---

## Principais arquivos

### `config.py`

Guarda as configurações principais do sistema:

- tamanho do grid;
- tamanho físico do ambiente;
- índice da câmera;
- intervalo de captura;
- threshold de fogo;
- zoom digital.

---

### `main_camera_preview.py`

Abre a webcam e mostra o grid 4x4 sobre a imagem original.

Também permite ajustar o zoom digital.

Comandos:

```text
+ = aumenta zoom
- = diminui zoom
S = salva zoom
Q = sair
```

---

### `calibrar_perspectiva.py`

Permite clicar nos 4 cantos do tapete para gerar a calibração de perspectiva.

Ordem dos cliques:

```text
B01 -> B04 -> B16 -> B13
```

Ou seja:

```text
1º ponto: canto externo do B01
2º ponto: canto externo do B04
3º ponto: canto externo do B16
4º ponto: canto externo do B13
```

Esse script gera o arquivo local:

```text
perspective_points.json
```

Esse arquivo **não deve ser enviado ao GitHub**, pois depende da posição da câmera de cada máquina.

---

### `main_camera_preview_corrigido.py`

Mostra a imagem já corrigida pela perspectiva, com o grid 4x4 desenhado.

Esse script deve ser usado para validar se a calibração ficou correta.

---

### `capture_dataset.py`

Captura imagens reais do ambiente para criação do dataset.

As imagens são salvas em:

```text
C:\Users\yurid\Desktop\robo_incendio\dataset_original
```

Estrutura esperada:

```text
dataset_original/
├── vazio/
├── vela_acesa/
│   ├── B01/
│   ├── B02/
│   └── ...
│   └── B16/
└── vela_apagada/
    ├── B01/
    ├── B02/
    └── ...
    └── B16/
```

As imagens são salvas **sem o grid verde**. O grid aparece apenas na interface de visualização.

---

### `coordinates.py`

Converte cada bloco do grid em uma coordenada física do ambiente.

Exemplos:

| Bloco | X | Y |
|---|---:|---:|
| B01 | 0.1875 | 1.3125 |
| B06 | 0.5625 | 0.9375 |
| B07 | 0.9375 | 0.9375 |
| B16 | 1.3125 | 0.1875 |

---

### `robot_command.py`

Converte a saída do algoritmo de Machine Learning em comando para o robô.

Exemplo de entrada:

```json
{
  "fogo_detectado": true,
  "bloco": "B07",
  "probabilidade": 0.91
}
```

Exemplo de saída:

```json
{
  "acao": "ir_para_fogo",
  "fogo_detectado": true,
  "bloco": "B07",
  "x": 0.9375,
  "y": 0.9375,
  "probabilidade": 0.91
}
```

---

### `integration_controller.py`

Simula a integração entre o Machine Learning e o robô.

Nesta versão, a inferência do modelo é simulada. Isso permite validar a camada de integração antes do modelo final estar treinado.

Fluxo executado:

```text
Inferência simulada
↓
Geração do comando
↓
Medição de latência
↓
Registro do comando
```

---

### `test_latency.py`

Executa testes de latência da integração.

Cenários testados:

- ambiente seguro;
- fogo no B01;
- fogo no B06;
- fogo no B07;
- fogo no B16.

Gera o arquivo local:

```text
camera-ml/runtime/resultados/resultado_latencia.csv
```

Esse arquivo também não deve ser versionado.

---

## Como executar

Entre na pasta do runtime:

```powershell
cd camera-ml/runtime
```

Instale as dependências:

```powershell
python -m pip install -r requirements.txt
```

Teste a webcam:

```powershell
python main_camera_preview.py
```

Calibre a perspectiva:

```powershell
python calibrar_perspectiva.py
```

Teste a imagem corrigida:

```powershell
python main_camera_preview_corrigido.py
```

Capture imagens do dataset:

```powershell
python capture_dataset.py
```

Teste a integração ML -> Robô:

```powershell
python integration_controller.py
```

Teste a latência:

```powershell
python test_latency.py
```

---

## Integração Machine Learning -> Robô

A entrega de integração consiste em transformar a saída do modelo em comportamento autônomo do robô.

Exemplo:

```json
{
  "acao": "ir_para_fogo",
  "fogo_detectado": true,
  "bloco": "B07",
  "x": 0.9375,
  "y": 0.9375,
  "probabilidade": 0.91,
  "latencia_ms": 0.008
}
```

Interpretação:

```text
A IA identificou fogo no bloco B07.
O sistema converteu B07 para coordenada física.
O comando gerado orienta o robô a se deslocar até essa região.
```

Quando o robô chegar ao bloco indicado, o sensor infravermelho poderá ser usado para fazer a busca fina da chama.

---

## Treinamento do modelo

O modelo planejado para treinamento é o **MobileNetV2**, usando classificação binária:

```text
0 = non_fire
1 = fire
```

A IA não aprende diretamente `B01`, `B02`, `B03`, etc.

Ela aprende somente:

```text
fire
non_fire
```

A localização do fogo é feita pelo código, que divide a imagem corrigida em 16 blocos e aplica o modelo em cada região.

---

## Observações importantes

- Não mover a câmera depois da calibração.
- Não alterar o zoom depois da calibração.
- Se mover a câmera, rode novamente `calibrar_perspectiva.py`.
- O arquivo `perspective_points.json` é local e não deve ir para o GitHub.
- As imagens do dataset não devem ser versionadas.
- Modelos treinados `.h5`, `.keras` ou `.tflite` não devem ser enviados diretamente ao GitHub.
- O grid verde aparece apenas no preview, não nas imagens salvas.

---

## Status atual

Implementado:

- Captura da webcam.
- Ajuste de zoom digital.
- Correção de perspectiva.
- Grid 4x4.
- Captura de dataset.
- Mapeamento bloco -> coordenada.
- Camada de integração ML -> robô.
- Teste de latência.
- Documentação da entrega de 12/05/2026.

Próximos passos:

- Finalizar captura das imagens reais.
- Treinar o modelo MobileNetV2.
- Substituir a inferência simulada pela inferência real.
- Integrar envio de comando com o robô físico.

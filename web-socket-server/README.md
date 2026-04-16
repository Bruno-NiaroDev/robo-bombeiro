# 🔌 WebSocket Relay Server

Servidor WebSocket simples para comunicação em tempo real entre sistemas

## 🎯 Objetivo

Este servidor atua como um **relay de mensagens**, permitindo que diferentes clientes (ex: Python e C#) troquem dados via WebSocket usando um canal compartilhado (`channelCode`).

Uso típico no projeto:

- Python (Machine Learning) → detecta fogo e envia coordenadas  
- C# (Robô) → recebe coordenadas e executa ações  

---

## 🚀 Como funciona

- Clientes conectam via WebSocket
- Cada cliente entra em um `channelCode`
- Mensagens enviadas por um cliente são repassadas para os outros no mesmo canal
- Suporte a flag `hook` para ignorar envio (webhooks)

---

## 📦 Instalação

```bash
npm install
````

---

## ▶️ Executar servidor

```bash
node server.js
```

Saída esperada:

```bash
Server listening on port 55619
```

---

## 🔗 Conexão

URL padrão:

```text
ws://localhost:55619?channelCode=fire1
```

Parâmetros:

| Parâmetro   | Descrição                         |
| ----------- | --------------------------------- |
| channelCode | Canal de comunicação              |
| hook        | (opcional) Ignora envio se `true` |

---

## 📨 Formato de mensagem

```json
{
  "type": "TARGET_POSITION",
  "payload": {
    "x": 120,
    "y": 80,
    "side": "left"
  }
}
```

---

## 🧠 Exemplo de uso - Python (ML)

### Instalar dependência

```bash
pip install websocket-client
```

### Código

```python
import websocket
import json
import time

ws = websocket.WebSocket()
ws.connect("ws://localhost:55619?channelCode=fire1")

while True:
    data = {
        "type": "TARGET_POSITION",
        "payload": {
            "x": 120,
            "y": 80,
            "side": "left"
        }
    }

    ws.send(json.dumps(data))
    print("Sent:", data)

    time.sleep(2)
```

---

## 🤖 Exemplo de uso - C# (Robô)

### Dependência

```bash
dotnet add package Newtonsoft.Json
```

### Código

```csharp
using System;
using System.Net.WebSockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

class Program
{
    static async Task Main()
    {
        using (ClientWebSocket ws = new ClientWebSocket())
        {
            Uri serverUri = new Uri("ws://localhost:55619?channelCode=fire1");

            await ws.ConnectAsync(serverUri, CancellationToken.None);
            Console.WriteLine("Connected");

            var buffer = new byte[1024];

            while (ws.State == WebSocketState.Open)
            {
                var result = await ws.ReceiveAsync(new ArraySegment<byte>(buffer), CancellationToken.None);
                string message = Encoding.UTF8.GetString(buffer, 0, result.Count);

                Console.WriteLine("Received: " + message);

                ProcessMessage(message);
            }
        }
    }

    static void ProcessMessage(string json)
    {
        dynamic data = Newtonsoft.Json.JsonConvert.DeserializeObject(json);

        if (data.type == "TARGET_POSITION")
        {
            int x = data.payload.x;
            int y = data.payload.y;
            string side = data.payload.side;

            Console.WriteLine($"X: {x}, Y: {y}, Side: {side}");

            if (side == "left")
                Console.WriteLine("Turn left");
            else if (side == "right")
                Console.WriteLine("Turn right");
            else
                Console.WriteLine("Move forward");
        }
    }
}
```

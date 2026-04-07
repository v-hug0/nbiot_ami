# AMI Server — Servidor de Testes DLMS/COSEM

Servidor Python para receber, decodificar e visualizar pacotes DLMS/COSEM
enviados via UDP pelo simulador do medidor (PC Python + ESP32 mock Quectel).

## Arquitetura

```
PC (meter_simulator.py)
  -- AT cmds (serial) -->  ESP32 (mock Quectel)
                              -- UDP 4059 -->  Oracle Cloud (ami_server.py)
                              <-- UDP 4059 --  (downstream: valve, FOTA, GET)
```

## Arquivos

| Arquivo            | Funcao                                              |
|--------------------|-----------------------------------------------------|
| `ami_server.py`    | Servidor principal (UDP listener + Flask dashboard) |
| `dlms_decoder.py`  | Decoder WRAPPER + APDU + OBIS (sem dependencias)   |
| `test_decoder.py`  | Suite de testes unitarios do decoder                |
| `requirements.txt` | Dependencias Python                                 |

## Deploy no Oracle Cloud (Ubuntu 22.04)

```bash
# 1. Instalar dependencias
sudo apt update && sudo apt install -y python3-pip
pip3 install -r requirements.txt

# 2. Abrir portas no Oracle Security List
#    UDP 4059  (DLMS/COSEM — pacotes do medidor)
#    TCP 8080  (dashboard HTTP)
#    Equivalente no iptables:
sudo iptables -I INPUT -p udp --dport 4059 -j ACCEPT
sudo iptables -I INPUT -p tcp --dport 8080 -j ACCEPT

# 3. Rodar o servidor
python3 ami_server.py --host 0.0.0.0 --udp-port 4059 --http-port 8080

# 4. Rodar em background com nohup
nohup python3 ami_server.py > ami_server.log 2>&1 &
```

## Endpoints da API HTTP

| Metodo | Path                | Descricao                              |
|--------|---------------------|----------------------------------------|
| GET    | `/`                 | Dashboard HTML                         |
| GET    | `/api/meters`       | Lista de medidores conhecidos          |
| GET    | `/api/meter/<id>`   | Leituras de um medidor especifico      |
| GET    | `/api/events`       | Log de alarmes N1 recebidos            |
| POST   | `/api/cmd/valve`    | Envia comando de valvula               |
| POST   | `/api/cmd/fota`     | Inicia sessao FOTA                     |
| POST   | `/api/cmd/get`      | Envia GET-Request (leitura sob demanda)|

### Exemplo: comando de valvula via curl

```bash
curl -X POST http://<IP_ORACLE>:8080/api/cmd/valve \
  -H "Content-Type: application/json" \
  -d '{"serial": "FAE00001234", "position_pct": 50}'
```

### Exemplo: GET-Request para volume acumulado

```bash
curl -X POST http://<IP_ORACLE>:8080/api/cmd/get \
  -H "Content-Type: application/json" \
  -d '{"serial": "FAE00001234", "obis": [7,0,3,0,0,255], "class_id": 1, "attr_id": 2}'
```

## Validacao sem hardware

```bash
# Roda suite de testes (sem ESP32 nem Oracle)
python3 test_decoder.py

# Testa contra servidor local (inicie o servidor primeiro)
python3 ami_server.py &
python3 test_decoder.py 127.0.0.1
```

## Proximos passos

1. **ESP32 (mock Quectel)** — firmware Arduino que parseia AT+QIOPEN/QISEND
   e repassa UDP via WiFiUDP para o Oracle.

2. **PC simulator (meter_simulator.py)** — script Python que simula o MSP430:
   envia comandos AT via serial para o ESP32 e constroi APDUs N1/N2/N3.

## OBIS codes implementados

| OBIS              | Nome             | Escala |
|-------------------|------------------|--------|
| 0.0.1.0.0.255     | Clock            | -      |
| 0.0.96.1.0.255    | Serial           | -      |
| 7.0.3.0.0.255     | Vol Forward      | x0.001 m3 |
| 7.0.4.0.0.255     | Vol Reverse      | x0.001 m3 |
| 7.0.11.0.0.255    | Flow Rate        | x0.001 m3/h |
| 7.0.41.0.0.255    | Temperature      | x0.1 C |
| 0.0.96.5.4.255    | Status bitmask   | -      |
| 0.0.96.3.10.255   | Valve Status     | -      |
| 0.0.96.12.0..2.255| RSRP/RSRQ/SINR   | -      |
| 0.0.96.12.3.255   | IMEI             | -      |
| 0.0.96.10.1.255   | Valve Action     | downstream |
| 0.0.96.10.2.255   | FOTA Start       | downstream |

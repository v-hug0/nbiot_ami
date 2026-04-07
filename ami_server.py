"""
ami_server.py — Servidor AMI para testes do hidrômetro NB-IoT.

Funcoes:
  - UDP socket na porta 4059 (DLMS/COSEM padrao IEC 62056-47)
  - Decode de WRAPPER + APDU de todos os pacotes recebidos
  - Armazenamento em memoria dos ultimos pacotes por medidor
  - API HTTP (Flask) para:
      GET  /api/meters          — lista medidores conhecidos
      GET  /api/meter/<serial>  — ultimas leituras de um medidor
      GET  /api/events          — log de eventos/alarmes (N1)
      POST /api/cmd/valve       — envia comando de valvula
      POST /api/cmd/fota        — inicia sessao FOTA
      POST /api/cmd/get         — leitura sob demanda (GET-Request)
  - Dashboard HTML simples em /

Uso:
  python ami_server.py [--host 0.0.0.0] [--udp-port 4059] [--http-port 8080]

  No Oracle Cloud: abrir porta UDP 4059 e TCP 8080 no Security List.
"""

import argparse
import json
import logging
import socket
import struct
import threading
import time
from collections import deque
from datetime import datetime

from flask import Flask, jsonify, request, render_template_string

from dlms_decoder import (
    decode_packet, format_decoded,
    build_valve_command, build_fota_start, build_get_request,
    OBIS_MAP, STATUS_BITS,
)

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("ami_server")

# ---------------------------------------------------------------------------
# Estado global compartilhado entre UDP e HTTP
# ---------------------------------------------------------------------------
_lock    = threading.Lock()
_meters  = {}      # serial -> {last_seen, addr, readings: {obis_str: ObisValue}, invoke_id}
_events  = deque(maxlen=500)   # lista de alarmes N1
_invoke_counter = 1


def _next_invoke_id() -> int:
    global _invoke_counter
    with _lock:
        v = _invoke_counter
        _invoke_counter = (v + 1) & 0xFFFFFF
        return v


# Mapa de IP → serial do medidor (preenchido quando N1/N2 chega com serial)
_ip_to_serial = {}   # type: dict — IP → serial do medidor


def _identify_meter(addr: tuple, decoded) -> str:
    """
    Retorna o identificador canônico do medidor.

    Prioridade:
    1. Serial number do OBIS 0.0.96.1.0.255 (presente em N1/N2).
    2. Serial já conhecido para este IP (permite vincular GET-Response
       ao medidor correto mesmo quando vem de porta efêmera diferente).
    3. Fallback: só o IP, sem porta (evita medidores duplicados por
       porta UDP efêmera que muda a cada sessão).
    """
    ip = addr[0]

    # 1. Serial explícito no pacote (N1 / N2)
    for obj in decoded.objects:
        if obj.obis == (0, 0, 96, 1, 0, 255) and obj.raw:
            serial = str(obj.raw)
            _ip_to_serial[ip] = serial   # registra associação IP → serial
            return serial

    # 2. IP já associado a um serial conhecido (GET-Response, N1 sem serial)
    if ip in _ip_to_serial:
        return _ip_to_serial[ip]

    # 3. Fallback: só IP (sem porta — evita entradas duplicadas)
    return ip


def _store_reading(meter_id: str, addr: tuple, decoded):
    """Atualiza _meters com os valores decodificados."""
    with _lock:
        if meter_id not in _meters:
            _meters[meter_id] = {
                "serial": meter_id,
                "addr":   addr,
                "first_seen": datetime.utcnow().isoformat(),
                "last_seen":  None,
                "packet_count": 0,
                "readings": {},
                "last_apdu_type": None,
            }
        m = _meters[meter_id]
        m["last_seen"]     = datetime.utcnow().isoformat()
        m["addr"]          = addr
        m["packet_count"] += 1
        m["last_apdu_type"] = decoded.tag_name
        if decoded.timestamp:
            m["meter_timestamp"] = decoded.timestamp

        for obj in decoded.objects:
            obis_str = ".".join(str(b) for b in obj.obis)
            m["readings"][obis_str] = {
                "name":    obj.name,
                "raw":     obj.raw,
                "scaled":  obj.scaled,
                "unit":    obj.unit,
                "extra":   obj.extra,
                "updated": datetime.utcnow().isoformat(),
            }

        # Verifica alarmes N1 (Status bitmask)
        status_obj = next(
            (o for o in decoded.objects if o.obis == (0, 0, 96, 5, 4, 255)), None
        )
        if status_obj and isinstance(status_obj.raw, int) and status_obj.raw != 0:
            active = [STATUS_BITS[b] for b in range(13)
                      if status_obj.raw & (1 << b)]
            ev = {
                "time":    datetime.utcnow().isoformat(),
                "meter":   meter_id,
                "type":    decoded.tag_name,
                "alarms":  active,
                "bitmask": status_obj.raw,
            }
            _events.appendleft(ev)
            log.warning("ALARM N1 | meter=%s | %s", meter_id, " | ".join(active))


# ---------------------------------------------------------------------------
# Thread UDP
# ---------------------------------------------------------------------------
_udp_sock = None  # type: socket.socket


def _udp_thread(host: str, port: int):
    global _udp_sock
    _udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    _udp_sock.bind((host, port))
    _udp_sock.settimeout(1.0)
    log.info("UDP listener em %s:%d", host, port)

    while True:
        try:
            data, addr = _udp_sock.recvfrom(1024)
        except socket.timeout:
            continue
        except Exception as e:
            log.error("UDP recv error: %s", e)
            continue

        try:
            hdr, dec = decode_packet(data)
            if hdr is None or dec is None:
                log.warning("Pacote invalido de %s (len=%d)", addr, len(data))
                continue

            meter_id = _identify_meter(addr, dec)
            _store_reading(meter_id, addr, dec)

            log.info("PKT from %-22s | %-20s | invoke=%d | objs=%d",
                     f"{addr[0]}:{addr[1]}", dec.tag_name,
                     dec.invoke_id, len(dec.objects))

            if log.isEnabledFor(logging.DEBUG):
                log.debug("\n%s", format_decoded(hdr, dec))

        except Exception as e:
            log.exception("Erro ao processar pacote de %s: %s", addr, e)


def _send_downstream(meter_id: str, payload: bytes) -> bool:
    """Envia payload UDP para o endereco do medidor."""
    with _lock:
        m = _meters.get(meter_id)
        if not m:
            return False
        addr = m["addr"]

    try:
        _udp_sock.sendto(payload, addr)
        log.info("Downstream -> %s (%d bytes)", addr, len(payload))
        return True
    except Exception as e:
        log.error("Erro ao enviar downstream: %s", e)
        return False


# ---------------------------------------------------------------------------
# Flask app
# ---------------------------------------------------------------------------
app = Flask(__name__)
app.config["JSON_SORT_KEYS"] = False


DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>AMI Server — Dashboard de Testes</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: 'Segoe UI', system-ui, sans-serif; background: #f0f4f8; color: #1a2332; }
  header { background: #1a2332; color: #fff; padding: 14px 24px;
           display: flex; align-items: center; gap: 12px; }
  header h1 { font-size: 1.1rem; font-weight: 600; }
  header span { font-size: .8rem; opacity: .6; margin-left: auto; }
  .container { max-width: 1100px; margin: 0 auto; padding: 20px; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
  .card { background: #fff; border-radius: 10px; padding: 18px;
          box-shadow: 0 1px 4px rgba(0,0,0,.08); }
  .card h2 { font-size: .85rem; text-transform: uppercase; letter-spacing: .06em;
             color: #64748b; margin-bottom: 14px; }
  table { width: 100%; border-collapse: collapse; font-size: .88rem; }
  th, td { text-align: left; padding: 7px 10px; border-bottom: 1px solid #f0f4f8; }
  th { font-weight: 600; color: #475569; font-size: .78rem; text-transform: uppercase; }
  tr:last-child td { border-bottom: none; }
  .badge { display: inline-block; padding: 2px 8px; border-radius: 20px;
           font-size: .75rem; font-weight: 600; }
  .badge-ok  { background: #dcfce7; color: #166534; }
  .badge-alarm { background: #fee2e2; color: #991b1b; }
  .badge-info  { background: #dbeafe; color: #1e40af; }
  .controls { margin-top: 12px; display: flex; gap: 8px; flex-wrap: wrap; }
  select, input { border: 1px solid #cbd5e1; border-radius: 6px; padding: 6px 10px;
                  font-size: .88rem; }
  button { background: #3b82f6; color: #fff; border: none; border-radius: 6px;
           padding: 7px 16px; font-size: .88rem; cursor: pointer; }
  button:hover { background: #2563eb; }
  button.danger { background: #ef4444; }
  #log { background: #0f172a; color: #94a3b8; border-radius: 8px; padding: 14px;
         font-family: monospace; font-size: .78rem; height: 220px; overflow-y: auto;
         white-space: pre-wrap; margin-top: 16px; }
  .alarm-row td:first-child { color: #dc2626; font-weight: 600; }
  @media (max-width: 700px) { .grid { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<header>
  <h1>&#x1F4E1; AMI Server — Dashboard de Testes</h1>
  <span id="clock"></span>
</header>
<div class="container">

<div class="grid" style="margin-bottom:16px">
  <div class="card">
    <h2>Medidores Conhecidos</h2>
    <table id="tbl-meters">
      <thead><tr><th>ID/Serial</th><th>Ultimo pacote</th><th>Tipo</th><th>Pkts</th></tr></thead>
      <tbody></tbody>
    </table>
  </div>
  <div class="card">
    <h2>Alarmes N1</h2>
    <table id="tbl-alarms">
      <thead><tr><th>Hora</th><th>Medidor</th><th>Alarmes</th></tr></thead>
      <tbody></tbody>
    </table>
  </div>
</div>

<div class="card" style="margin-bottom:16px">
  <h2>Leituras do Medidor</h2>
  <div class="controls">
    <select id="sel-meter"><option value="">-- selecione --</option></select>
    <button onclick="loadReadings()">Atualizar</button>
  </div>
  <table id="tbl-readings" style="margin-top:12px">
    <thead><tr><th>OBIS</th><th>Nome</th><th>Valor</th><th>Atualizado</th></tr></thead>
    <tbody></tbody>
  </table>
</div>

<div class="card">
  <h2>Comandos Downstream</h2>
  <div class="controls">
    <select id="cmd-meter"><option value="">-- medidor --</option></select>
    <select id="cmd-valve">
      <option value="100">Valvula 100% (aberta)</option>
      <option value="75">Valvula 75%</option>
      <option value="50">Valvula 50%</option>
      <option value="25">Valvula 25%</option>
      <option value="0">Valvula 0% (fechada)</option>
    </select>
    <button onclick="sendValve()">Enviar Valvula</button>
    <button onclick="sendGet()">GET-Request</button>
    <button class="danger" onclick="clearLog()">Limpar Log</button>
  </div>
  <div id="log">-- log de comandos --\n</div>
</div>

</div>

<script>
const logEl = document.getElementById('log');
function addLog(msg) {
  const t = new Date().toLocaleTimeString('pt-BR');
  logEl.textContent += `[${t}] ${msg}\n`;
  logEl.scrollTop = logEl.scrollHeight;
}
function clearLog() { logEl.textContent = ''; }

async function api(path, opts) {
  try {
    const r = await fetch('/api' + path, opts);
    return await r.json();
  } catch(e) { addLog('Erro: ' + e); return null; }
}

function refreshSelects(meters) {
  ['sel-meter','cmd-meter'].forEach(id => {
    const sel = document.getElementById(id);
    const cur = sel.value;
    sel.innerHTML = '<option value="">-- selecione --</option>';
    meters.forEach(m => {
      const o = document.createElement('option');
      o.value = o.textContent = m.serial;
      if (m.serial === cur) o.selected = true;
      sel.appendChild(o);
    });
  });
}

async function refresh() {
  document.getElementById('clock').textContent = new Date().toLocaleTimeString('pt-BR');

  // Meters
  const meters = await api('/meters');
  if (!meters) return;
  refreshSelects(meters);
  const tbody = document.querySelector('#tbl-meters tbody');
  tbody.innerHTML = meters.map(m =>
    `<tr><td>${m.serial}</td><td>${(m.last_seen||'').slice(11,19)}</td>
     <td><span class="badge badge-info">${m.last_apdu_type||''}</span></td>
     <td>${m.packet_count}</td></tr>`
  ).join('');

  // Events
  const evs = await api('/events');
  if (!evs) return;
  const etbody = document.querySelector('#tbl-alarms tbody');
  etbody.innerHTML = evs.slice(0,10).map(e =>
    `<tr class="alarm-row"><td>${(e.time||'').slice(11,19)}</td>
     <td>${e.meter}</td>
     <td><span class="badge badge-alarm">${(e.alarms||[]).join(', ')}</span></td></tr>`
  ).join('') || '<tr><td colspan="3" style="color:#94a3b8">Sem alarmes</td></tr>';
}

async function loadReadings() {
  const serial = document.getElementById('sel-meter').value;
  if (!serial) return;
  const data = await api(`/meter/${encodeURIComponent(serial)}`);
  if (!data) return;
  const tbody = document.querySelector('#tbl-readings tbody');
  const readings = data.readings || {};
  tbody.innerHTML = Object.entries(readings).map(([obis, r]) =>
    `<tr><td style="font-family:monospace;font-size:.8rem">${obis}</td>
     <td>${r.name}</td>
     <td>${r.scaled != null ? r.scaled.toFixed(4)+' '+r.unit : r.raw}
         ${r.extra ? '<br><small style="color:#64748b">'+r.extra+'</small>' : ''}</td>
     <td style="color:#94a3b8;font-size:.8rem">${(r.updated||'').slice(11,19)}</td></tr>`
  ).join('') || '<tr><td colspan="4">Sem leituras</td></tr>';
}

async function sendValve() {
  const serial = document.getElementById('cmd-meter').value;
  const pos = parseInt(document.getElementById('cmd-valve').value);
  if (!serial) { addLog('Selecione um medidor'); return; }
  const r = await api('/cmd/valve', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({serial, position_pct: pos})
  });
  addLog(r ? `Valvula ${pos}% -> ${serial}: ${r.status}` : 'Falha');
}

async function sendGet() {
  const serial = document.getElementById('cmd-meter').value;
  if (!serial) { addLog('Selecione um medidor'); return; }
  const obis = [7,0,3,0,0,255];  // Vol Forward
  const r = await api('/cmd/get', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({serial, obis, class_id: 1, attr_id: 2})
  });
  addLog(r ? `GET-Request Vol Forward -> ${serial}: ${r.status}` : 'Falha');
}

setInterval(refresh, 3000);
refresh();
</script>
</body>
</html>"""


@app.route("/")
def dashboard():
    return render_template_string(DASHBOARD_HTML)


@app.route("/api/meters")
def api_meters():
    with _lock:
        out = []
        for m in _meters.values():
            out.append({
                "serial":         m["serial"],
                "addr":           f"{m['addr'][0]}:{m['addr'][1]}",
                "first_seen":     m["first_seen"],
                "last_seen":      m["last_seen"],
                "packet_count":   m["packet_count"],
                "last_apdu_type": m["last_apdu_type"],
            })
    return jsonify(out)


@app.route("/api/meter/<serial>")
def api_meter(serial: str):
    with _lock:
        m = _meters.get(serial)
        if not m:
            return jsonify({"error": "not found"}), 404
        return jsonify(dict(m, addr=f"{m['addr'][0]}:{m['addr'][1]}"))


@app.route("/api/events")
def api_events():
    with _lock:
        return jsonify(list(_events))


@app.route("/api/cmd/valve", methods=["POST"])
def api_cmd_valve():
    body = request.get_json(force=True)
    serial   = body.get("serial", "")
    position = int(body.get("position_pct", 100))
    if position not in (0, 25, 50, 75, 100):
        return jsonify({"status": "error", "msg": "posicao invalida"}), 400

    iid     = _next_invoke_id()
    payload = build_valve_command(iid, position)
    ok      = _send_downstream(serial, payload)
    log.info("CMD valve -> %s pos=%d%% invoke=%d ok=%s", serial, position, iid, ok)
    return jsonify({"status": "sent" if ok else "error",
                    "invoke_id": iid, "position_pct": position})


@app.route("/api/cmd/fota", methods=["POST"])
def api_cmd_fota():
    body      = request.get_json(force=True)
    serial    = body.get("serial", "")
    file_size = int(body.get("file_size", 0))
    crc32     = int(body.get("crc32", 0))
    if not file_size:
        return jsonify({"status": "error", "msg": "file_size obrigatorio"}), 400

    iid     = _next_invoke_id()
    payload = build_fota_start(iid, file_size, crc32)
    ok      = _send_downstream(serial, payload)
    log.info("CMD fota -> %s size=%d crc=0x%08X invoke=%d ok=%s",
             serial, file_size, crc32, iid, ok)
    return jsonify({"status": "sent" if ok else "error",
                    "invoke_id": iid})


@app.route("/api/cmd/get", methods=["POST"])
def api_cmd_get():
    body     = request.get_json(force=True)
    serial   = body.get("serial", "")
    obis     = tuple(body.get("obis", [0,0,96,1,0,255]))
    class_id = int(body.get("class_id", 1))
    attr_id  = int(body.get("attr_id", 2))

    iid     = _next_invoke_id()
    payload = build_get_request(iid, obis, class_id, attr_id)
    ok      = _send_downstream(serial, payload)
    obis_str = ".".join(str(b) for b in obis)
    log.info("CMD GET-Request -> %s obis=%s invoke=%d ok=%s",
             serial, obis_str, iid, ok)
    return jsonify({"status": "sent" if ok else "error",
                    "invoke_id": iid, "obis": obis_str})


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Servidor AMI de testes")
    parser.add_argument("--host",      default="0.0.0.0")
    parser.add_argument("--udp-port",  type=int, default=4059)
    parser.add_argument("--http-port", type=int, default=8080)
    parser.add_argument("--debug",     action="store_true")
    args = parser.parse_args()

    if args.debug:
        log.setLevel(logging.DEBUG)

    # Inicia thread UDP
    t = threading.Thread(
        target=_udp_thread, args=(args.host, args.udp_port), daemon=True
    )
    t.start()

    log.info("HTTP dashboard em http://%s:%d", args.host, args.http_port)
    app.run(host=args.host, port=args.http_port,
            debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
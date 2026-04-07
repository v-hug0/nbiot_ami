"""
meter_simulator.py — Simulador do medidor AMI (MSP430 + NB-IoT).

Replica a FSM de nbiot_mng.c:
  POWERING_ON → CONFIGURING → REGISTERING → UDP_OPENING →
  IDLE ↔ TRANSMITTING ↔ RECV_WAIT → PSM

Interface interativa (CLI) para:
  - Injetar alarmes N1 (last_gasp, tamper, magnetic, valve)
  - Disparar transmissão N2 / N3 manualmente
  - Alterar estado do medidor (volume, temperatura, válvula)
  - Ver estado atual da FSM e leituras

Uso:
  # Com ESP32 conectado na USB:
  python meter_simulator.py --port /dev/ttyUSB0

  # Sem hardware (dry-run, testa apenas a lógica):
  python meter_simulator.py --dry-run

  # Com servidor AMI no Oracle (para envio UDP real):
  python meter_simulator.py --port COM3 --server 150.x.x.x --serial FAE00099
"""

import argparse
import logging
import os
import re
import sys
import threading
import time
from enum import Enum, auto
from datetime import datetime

# Garante que os módulos locais (at_serial, dlms_builder) sejam encontrados
# independentemente do cwd ou de como o script foi chamado (Windows/Linux).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from at_serial  import ATSerial, ATSerialDryRun, ATSerialDirectUDP, ATResult
from dlms_builder import (
    MeterState, Alarm,
    build_n1, build_n2, build_n3, build_get_response,
    STAT_LAST_GASP, STAT_TAMPER, STAT_MAGNETIC_ATTACK,
    STAT_BATTERY_LOW, STAT_COMM_FAIL,
)

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)-8s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("meter_sim")

# ---------------------------------------------------------------------------
# Estados da FSM (E_NBIOT_STATE_t de nbiot_mng.h)
# ---------------------------------------------------------------------------
class State(Enum):
    POWERING_ON  = auto()
    CONFIGURING  = auto()
    REGISTERING  = auto()
    UDP_OPENING  = auto()
    IDLE         = auto()
    TRANSMITTING = auto()
    RECV_WAIT    = auto()
    PSM          = auto()
    ERROR        = auto()


# Parâmetros NB-IoT do firmware (nbiot_mng.h)
NBIOT_APN          = "nbiot.claro.com.br"
NBIOT_PSM_TAU      = "00101001"   # TAU = 24h
NBIOT_PSM_ACT      = "00001111"   # Active = 30s
NBIOT_BAND_MASK    = "0x8000000"  # Band 28
RECV_WAIT_S        = 20.0         # janela pós-transmissão
RECONNECT_DELAY_S  = 30.0
UDP_SERVER_PORT    = 4059
UDP_SOCKET_ID      = 0


def _cereg_registered(line: str) -> bool:
    """
    True se a linha indica registro na rede NB-IoT.
    Cobre os dois formatos que o ESP32 mock (e o Quectel real) produzem:
      +CEREG: 1     — URC quando AT+CEREG=1 está ativo (stat=1, rede local)
      +CEREG: 5     — URC roaming
      +CEREG: 0,1   — resposta de AT+CEREG? (n=0, stat=1)
      +CEREG: 0,5   — resposta de AT+CEREG? (n=0, stat=5)
    """
    return bool(re.search(r'\+CEREG:.*\b[15]\b', line))


# ---------------------------------------------------------------------------
# FSM principal
# ---------------------------------------------------------------------------
class MeterSimulator:
    def __init__(self, at: ATSerial, state: MeterState,
                 server_host: str, server_port: int = UDP_SERVER_PORT):
        self._at      = at
        self._state   = state
        self._server  = server_host
        self._port    = server_port
        self._fsm     = State.POWERING_ON
        self._t_enter = time.monotonic()
        self._running = False
        self._lock    = threading.Lock()

        # Filas de transmissão
        self._n1_pending: bytes | None = None
        self._n2_queue:   list[bytes]  = []
        self._n3_pending: bytes | None = None

        # Registro de URC recv para dados downstream
        self._recv_data: bytes | None = None
        self._urc_recv_pending: bool  = False   # sinaliza +QIURC: recv recebida
        self._at.register_urc('+QIURC: "recv"', self._on_recv_urc)
        self._at.register_urc("+CEREG:",         self._on_cereg_urc)

    # ------------------------------------------------------------------
    # Callbacks de URC
    # ------------------------------------------------------------------
    def _on_recv_urc(self, line: str):
        """Chamado quando +QIURC: "recv",0 chega — marca dado pendente."""
        log.info("URC recv: dado downstream disponível")
        self._urc_recv_pending = True   # garante QIRD mesmo se URC chegou dentro de expect()

    def _on_cereg_urc(self, line: str):
        """Chamado quando +CEREG: URC chega fora do expect() de registro."""
        log.debug("URC CEREG (fora do registering): %s", line)

    # ------------------------------------------------------------------
    # Controle externo (thread da CLI)
    # ------------------------------------------------------------------
    def inject_alarm(self, alarm: Alarm):
        """Enfileira alarme N1 imediato."""
        with self._lock:
            pkt = build_n1(alarm, self._state)
            self._n1_pending = pkt
        log.info("Alarme N1 enfileirado: %s", alarm.name)

    def trigger_n2(self):
        """Enfileira transmissão N2 imediata."""
        with self._lock:
            pkt = build_n2(self._state)
            self._n2_queue.append(pkt)
        log.info("N2 enfileirado (%d bytes)", len(pkt))

    def trigger_n3(self):
        """Enfileira transmissão N3 mensal."""
        with self._lock:
            pkt = build_n3(self._state)
            self._n3_pending = pkt
        log.info("N3 enfileirado (%d bytes)", len(pkt))

    def get_fsm_state(self) -> str:
        return self._fsm.name

    def get_meter_state(self) -> MeterState:
        return self._state

    # ------------------------------------------------------------------
    # Loop principal
    # ------------------------------------------------------------------
    def run(self):
        self._running = True
        log.info("FSM iniciada. Estado inicial: %s", self._fsm.name)
        while self._running:
            try:
                self._step()
            except Exception as e:
                log.exception("Erro na FSM: %s", e)
                self._enter(State.ERROR)
                time.sleep(5.0)

    def stop(self):
        self._running = False

    def _enter(self, new_state: State):
        if new_state != self._fsm:
            log.info("FSM: %s → %s", self._fsm.name, new_state.name)
        self._fsm     = new_state
        self._t_enter = time.monotonic()

    def _elapsed(self) -> float:
        return time.monotonic() - self._t_enter

    def _step(self):
        fsm = self._fsm
        if   fsm == State.POWERING_ON:  self._run_powering_on()
        elif fsm == State.CONFIGURING:  self._run_configuring()
        elif fsm == State.REGISTERING:  self._run_registering()
        elif fsm == State.UDP_OPENING:  self._run_udp_opening()
        elif fsm == State.IDLE:         self._run_idle()
        elif fsm == State.TRANSMITTING: self._run_transmitting()
        elif fsm == State.RECV_WAIT:    self._run_recv_wait()
        elif fsm == State.PSM:          self._run_psm()
        elif fsm == State.ERROR:        self._run_error()

    # ------------------------------------------------------------------
    # POWERING_ON — aguarda "+QIND: ready"
    # ------------------------------------------------------------------
    def _run_powering_on(self):
        log.info("Aguardando módulo inicializar (+QIND: ready)...")
        result, line = self._at.read_line(timeout=20.0)

        if result == ATResult.OK and ("RDY" in line or "ready" in line.lower()):
            log.info("Módulo pronto: '%s'", line)
            # Tenta ler IMEI
            r, imei_line = self._at.expect("AT+CGSN", "+CGSN:", timeout=3.0)
            if r == ATResult.OK and imei_line:
                imei = imei_line.split(":")[-1].strip().strip('"')
                if imei:
                    self._state.imei = imei
                    log.info("IMEI: %s", self._state.imei)
            self._enter(State.CONFIGURING)
            return

        if self._elapsed() > 20.0:
            log.warning("Timeout aguardando módulo. Tentando AT básico...")
            r, _ = self._at.expect("AT", None, timeout=2.0)
            if r == ATResult.OK:
                self._enter(State.CONFIGURING)
            else:
                time.sleep(2.0)

    # ------------------------------------------------------------------
    # CONFIGURING — envia parâmetros AT (runConfiguring)
    # ------------------------------------------------------------------
    def _run_configuring(self):
        log.info("Configurando módulo...")

        # Configura o IP do servidor no ESP32 via AT+AMICFG (comando extra,
        # não existe no Quectel real — ignorado se o módulo for real).
        r, _ = self._at.expect(
            f'AT+AMICFG="server","{self._server}"', None, timeout=2.0)
        if r == ATResult.OK:
            log.info("  IP do servidor configurado no módulo: %s", self._server)
        r, _ = self._at.expect(
            f'AT+AMICFG="port",{self._port}', None, timeout=2.0)

        cmds = [
            ("ATE0",                              None, 2.0),
            (f'AT+QCFG="nwscanseq",03,1',        None, 3.0),
            (f'AT+QCFG="band",0,0,{NBIOT_BAND_MASK},1', None, 3.0),
            (f'AT+CPSMS=1,,,"{NBIOT_PSM_TAU}","{NBIOT_PSM_ACT}"', None, 3.0),
            ('AT+CEDRXS=2,5,"0101"',              None, 3.0),
            ("AT+QIACT=1",                        None, 15.0),
            ("AT+CEREG=1",                        None, 3.0),
        ]

        for cmd, exp, tmo in cmds:
            r, line = self._at.expect(cmd, exp, timeout=tmo)
            log.debug("  %s → %s '%s'", cmd, r.name, line)
            if r == ATResult.ERROR:
                log.warning("  Comando falhou: %s — continuando", cmd)

        self._enter(State.REGISTERING)

    # ------------------------------------------------------------------
    # REGISTERING — aguarda +CEREG: 1 ou 5
    # ------------------------------------------------------------------
    def _run_registering(self):
        log.info("Aguardando registro NB-IoT...")

        # Verifica status atual — o ESP32 retorna +CEREG: 1 (sem vírgula)
        # quando AT+CEREG=1 está ativo; Quectel real retorna +CEREG: 0,1.
        # _cereg_registered() cobre os dois formatos.
        r, line = self._at.expect("AT+CEREG?", "+CEREG:", timeout=10.0)
        if r == ATResult.OK and _cereg_registered(line):
            log.info("Registrado na rede: %s", line)
            self._read_kpis()
            self._enter(State.UDP_OPENING)
            return

        # Aguarda URC +CEREG: 1 / +CEREG: 0,1 por até 60s
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            r2, ln2 = self._at.read_line(timeout=5.0)
            if r2 == ATResult.OK and "+CEREG" in ln2:
                log.info("CEREG recebido: %s", ln2)
                if _cereg_registered(ln2):
                    log.info("Registrado na rede!")
                    self._read_kpis()
                    self._enter(State.UDP_OPENING)
                    return
            with self._lock:
                if self._n1_pending:
                    log.info("N1 pendente durante REGISTERING — aguardando rede...")

        log.error("Timeout de registro. Indo para ERROR.")
        self._state.set_alarm(STAT_COMM_FAIL)
        self._enter(State.ERROR)

    def _read_kpis(self):
        """Lê RSRP/RSRQ/SINR via AT+QENG (readKPIs do firmware)."""
        r, line = self._at.expect('AT+QENG="servingcell"', "+QENG:", timeout=5.0)
        if r != ATResult.OK:
            return
        # Parseia valores: ...,NB-IoT,<mcc>,<mnc>,<cell>,<pcid>,<earfcn>,<band>,<rsrp>,<rsrq>,<sinr>...
        try:
            nb_idx = line.find("NB-IoT")
            if nb_idx < 0:
                return
            parts = line[nb_idx:].split(",")
            # offsets: [0]=NB-IoT [1]=mcc [2]=mnc [3]=cell [4]=pcid [5]=earfcn
            #          [6]=band [7]=rsrp [8]=rsrq [9]=sinr [10]=rssi
            if len(parts) > 9:
                self._state.rsrp = int(parts[7])
                self._state.rsrq = int(parts[8])
                self._state.sinr = int(parts[9])
                log.info("KPIs: RSRP=%d RSRQ=%d SINR=%d",
                         self._state.rsrp, self._state.rsrq, self._state.sinr)
        except Exception as e:
            log.debug("Parse QENG falhou: %s", e)

    # ------------------------------------------------------------------
    # UDP_OPENING — AT+QIOPEN
    # ------------------------------------------------------------------
    def _run_udp_opening(self):
        log.info("Abrindo socket UDP...")

        # Fecha socket anterior se existir
        self._at.expect(f"AT+QICLOSE={UDP_SOCKET_ID}", None, timeout=3.0)

        cmd = (f'AT+QIOPEN=1,{UDP_SOCKET_ID},"UDP",'
               f'"{self._server}",{self._port},{self._port},0')
        r, _ = self._at.expect(cmd, None, timeout=5.0)  # espera OK imediato

        if r == ATResult.ERROR:
            log.error("AT+QIOPEN retornou ERROR")
            self._enter(State.ERROR)
            return

        # Aguarda +QIOPEN: 0,0 assíncrono
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            r2, ln2 = self._at.read_line(timeout=2.0)
            if r2 == ATResult.OK and "+QIOPEN:" in ln2:
                # Verifica código de erro: +QIOPEN: 0,0 → OK
                parts = ln2.split(",")
                err = parts[-1].strip() if parts else "1"
                if err == "0":
                    log.info("Socket UDP aberto com sucesso")
                    self._enter(State.IDLE)
                else:
                    log.error("+QIOPEN erro: %s", ln2)
                    self._enter(State.ERROR)
                return

        log.error("Timeout aguardando +QIOPEN")
        self._enter(State.ERROR)

    # ------------------------------------------------------------------
    # IDLE — decide o que transmitir ou entra em PSM
    # ------------------------------------------------------------------
    def _run_idle(self):
        # Leitura de dado downstream — prioritária se URC foi recebida
        if self._urc_recv_pending:
            self._urc_recv_pending = False
            data = self._at.qird(UDP_SOCKET_ID, max_len=512, timeout=3.0)
        else:
            data = self._at.qird(UDP_SOCKET_ID, max_len=512, timeout=0.5)
        if data:
            self._handle_downstream(data)

        # Verifica filas
        with self._lock:
            has_n1 = self._n1_pending is not None
            has_n2 = len(self._n2_queue) > 0
            has_n3 = self._n3_pending is not None

        if has_n1 or has_n2 or has_n3:
            self._enter(State.TRANSMITTING)
            return

        # Sem mensagens → PSM simulado (espera curta para manter CLI responsiva)
        time.sleep(1.0)

    # ------------------------------------------------------------------
    # TRANSMITTING — envia APDUs pelas filas N1 > N3 > N2
    # ------------------------------------------------------------------
    def _run_transmitting(self):
        ok = True

        with self._lock:
            n1 = self._n1_pending
            self._n1_pending = None

        if n1:
            log.info("Transmitindo N1 (%d bytes)...", len(n1))
            ok = self._at.qisend(UDP_SOCKET_ID, n1, self._server, self._port)
            if not ok:
                log.error("QISEND N1 falhou")

        with self._lock:
            n3 = self._n3_pending
            self._n3_pending = None

        if n3 and ok:
            log.info("Transmitindo N3 (%d bytes)...", len(n3))
            ok = self._at.qisend(UDP_SOCKET_ID, n3, self._server, self._port)

        with self._lock:
            queue = list(self._n2_queue)
            self._n2_queue.clear()

        for pkt in queue:
            if not ok:
                break
            log.info("Transmitindo N2 (%d bytes)...", len(pkt))
            ok = self._at.qisend(UDP_SOCKET_ID, pkt, self._server, self._port)

        if not ok:
            self._state.set_alarm(STAT_COMM_FAIL)
            self._enter(State.ERROR)
            return

        self._enter(State.RECV_WAIT)

    # ------------------------------------------------------------------
    # RECV_WAIT — aguarda 5s por comandos downstream
    # ------------------------------------------------------------------
    def _run_recv_wait(self):
        # Se a URC +QIURC: recv chegou (dentro ou fora de expect()),
        # faz QIRD com timeout maior para garantir a leitura.
        if self._urc_recv_pending:
            self._urc_recv_pending = False
            log.info("URC pendente — executando AT+QIRD...")
            data = self._at.qird(UDP_SOCKET_ID, max_len=512, timeout=3.0)
            if data:
                self._handle_downstream(data)
                return   # processa imediatamente; elapsed continua contando
            else:
                # Dado sumiu do buffer do ESP32 (raro) — aguarda mais 1s
                log.warning("QIRD após URC retornou vazio; aguardando...")
                time.sleep(1.0)
                data = self._at.qird(UDP_SOCKET_ID, max_len=512, timeout=3.0)
                if data:
                    self._handle_downstream(data)
                return

        # Polling passivo enquanto a janela não expirou.
        # A URC pode ter chegado DENTRO deste qird() — se sim, os dados
        # já foram lidos, então limpamos o flag para evitar um QIRD duplo.
        data = self._at.qird(UDP_SOCKET_ID, max_len=512, timeout=1.0)
        if data:
            self._urc_recv_pending = False   # dados já consumidos neste poll
            self._handle_downstream(data)

        # Não encerra enquanto houver URC pendente não processada
        if self._urc_recv_pending:
            return

        if self._elapsed() > RECV_WAIT_S:
            log.info("Janela RECV_WAIT encerrada → IDLE")
            self._enter(State.IDLE)

    # ------------------------------------------------------------------
    # PSM — simulado: dorme por TAU ou acorda por N1
    # ------------------------------------------------------------------
    def _run_psm(self):
        log.info("Módulo em PSM (simulado). Aguardando N1 ou TAU...")
        # Aguarda até N1 ser injetado ou 5s (TAU real seria 24h)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with self._lock:
                if self._n1_pending:
                    log.info("N1 recebido em PSM — acordando")
                    self._enter(State.REGISTERING)
                    return
            time.sleep(0.2)
        self._enter(State.REGISTERING)

    # ------------------------------------------------------------------
    # ERROR — aguarda e tenta reconectar
    # ------------------------------------------------------------------
    def _run_error(self):
        if self._elapsed() < RECONNECT_DELAY_S:
            time.sleep(1.0)
            return
        log.info("Tentando reconectar após erro...")
        self._at.expect(f"AT+QICLOSE={UDP_SOCKET_ID}", None, timeout=3.0)
        self._state.clear_alarm(STAT_COMM_FAIL)
        self._enter(State.REGISTERING)

    # ------------------------------------------------------------------
    # Processamento de comandos downstream (handleIncomingCmd)
    # ------------------------------------------------------------------
    def _handle_downstream(self, data: bytes):
        """Parseia WRAPPER + APDU e executa o comando."""
        # Importa o decoder do servidor para não duplicar código
        try:
            import sys, os
            # Adiciona o diretório do servidor ao path (para dlms_decoder)
            server_dir = os.path.join(os.path.dirname(__file__), "..", "ami_server")
            if server_dir not in sys.path:
                sys.path.insert(0, server_dir)
            from dlms_decoder import decode_packet, TAG_ACTION_REQUEST, TAG_GET_REQUEST
        except ImportError:
            log.warning("dlms_decoder não encontrado — ignorando downstream")
            return

        try:
            hdr, dec = decode_packet(data)
        except Exception as e:
            log.warning("Downstream parse error: %s", e)
            return

        if not dec:
            return

        log.info("Downstream recebido: %s invoke=%d", dec.tag_name, dec.invoke_id)

        if dec.tag == TAG_ACTION_REQUEST:
            for obj in dec.objects:
                # Valve command: OBIS 0.0.96.10.1.255
                if obj.obis == (0, 0, 96, 10, 1, 255):
                    enum_val = obj.raw if isinstance(obj.raw, int) else 0
                    pos_map  = {0: 0, 1: 25, 2: 50, 3: 75, 4: 100}
                    pos_pct  = pos_map.get(enum_val, 100)
                    log.info("  CMD Válvula: %d%% (enum=%d)", pos_pct, enum_val)
                    self._state.valve_position = pos_pct
                    self._state.valve_status   = (0 if pos_pct == 100 else
                                                   1 if pos_pct == 0 else 2)
                    # Gera alarme N1 de status alterado
                    with self._lock:
                        self._n1_pending = build_n1(Alarm.VALVE_CHANGED, self._state)

                # FOTA start: OBIS 0.0.96.10.2.255
                elif obj.obis == (0, 0, 96, 10, 2, 255):
                    log.info("  CMD FOTA start: param=%s", obj.raw)
                    log.warning("  [FOTA] Simulação: FOTA não implementado no PC simulator")

        elif dec.tag == TAG_GET_REQUEST:
            log.info("  CMD GET-Request invoke=%d — respondendo com GET-Response", dec.invoke_id)
            resp = build_get_response(dec.invoke_id, self._state)
            ok   = self._at.qisend(UDP_SOCKET_ID, resp, self._server, self._port)
            if ok:
                log.info("  GET-Response enviado (%d bytes)", len(resp))
            else:
                log.error("  GET-Response: falha no envio")


# ---------------------------------------------------------------------------
# CLI interativa (thread separada)
# ---------------------------------------------------------------------------
HELP = """
Comandos disponíveis:
  n1 last_gasp | tamper | magnetic | valve  — injeta alarme N1
  n2                                        — transmite pacote N2 diário
  n3                                        — transmite pacote N3 mensal
  set vol <litros>                          — define volume acumulado
  set flow <m3h>                            — define vazão atual (m³/h)
  set temp <celsius>                        — define temperatura da água
  set valve <0|25|50|75|100>               — simula posição da válvula
  set serial <serial>                       — define número de série
  alarm <bit_hex>                           — seta bit de alarme (ex: alarm 0x10)
  clear                                     — limpa todos os alarmes
  status                                    — mostra estado atual do medidor
  state                                     — mostra estado da FSM
  help                                      — este menu
  quit                                      — encerra o simulador
"""

def run_cli(sim: MeterSimulator):
    print("\n=== Simulador AMI — CLI interativa ===")
    print('Digite "help" para ver os comandos disponíveis.\n')

    while True:
        try:
            raw = input("AMI> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nEncerrando...")
            sim.stop()
            break

        if not raw:
            continue

        parts = raw.split()
        cmd   = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            sim.stop()
            break

        elif cmd == "help":
            print(HELP)

        elif cmd == "n1":
            alarm_map = {
                "last_gasp": Alarm.LAST_GASP,
                "tamper":    Alarm.TAMPER,
                "magnetic":  Alarm.MAGNETIC,
                "valve":     Alarm.VALVE_CHANGED,
            }
            alarm_name = parts[1].lower() if len(parts) > 1 else ""
            alarm = alarm_map.get(alarm_name)
            if alarm is None:
                print(f"Alarme desconhecido: {alarm_name}. Use: {list(alarm_map)}")
            else:
                sim.inject_alarm(alarm)
                print(f"Alarme N1 injetado: {alarm.name}")

        elif cmd == "n2":
            sim.trigger_n2()
            print("N2 enfileirado para transmissão")

        elif cmd == "n3":
            sim.trigger_n3()
            print("N3 enfileirado para transmissão")

        elif cmd == "set" and len(parts) >= 3:
            field = parts[1].lower()
            val   = parts[2]
            st    = sim.get_meter_state()
            try:
                if field == "vol":
                    st.vol_forward_L = float(val)
                    print(f"Volume: {st.vol_forward_L:.3f} L")
                elif field == "flow":
                    st.flow_rate_mh3 = float(val)
                    print(f"Vazão: {st.flow_rate_mh3:.4f} m³/h")
                elif field == "temp":
                    st.temp_cx10 = int(float(val) * 10)
                    print(f"Temperatura: {st.temp_cx10 / 10:.1f} °C")
                elif field == "valve":
                    pos = int(val)
                    if pos not in (0, 25, 50, 75, 100):
                        print("Posição inválida. Use: 0, 25, 50, 75 ou 100")
                    else:
                        st.valve_position = pos
                        st.valve_status   = (0 if pos == 100 else 1 if pos == 0 else 2)
                        print(f"Válvula: {pos}%")
                elif field == "serial":
                    st.serial = val
                    print(f"Serial: {st.serial}")
                else:
                    print(f"Campo desconhecido: {field}")
            except ValueError as e:
                print(f"Valor inválido: {e}")

        elif cmd == "alarm" and len(parts) >= 2:
            try:
                bit = int(parts[1], 0)
                sim.get_meter_state().set_alarm(bit)
                print(f"Alarme setado: 0x{bit:08X}")
            except ValueError:
                print("Use: alarm 0x<hex>")

        elif cmd == "clear":
            sim.get_meter_state().clear_all_alarms()
            print("Alarmes limpos")

        elif cmd == "status":
            st = sim.get_meter_state()
            print(f"\n  Serial:        {st.serial}")
            print(f"  IMEI:          {st.imei}")
            print(f"  FW:            {st.fw_ver}")
            print(f"  Volume:        {st.vol_forward_L:.3f} L ({st.vol_forward_L/1000:.3f} m³)")
            print(f"  Vazão:         {st.flow_rate_mh3:.4f} m³/h")
            print(f"  Temperatura:   {st.temp_cx10/10:.1f} °C")
            print(f"  Válvula:       {st.valve_position}%  (status={st.valve_status})")
            print(f"  Status mask:   0x{st.status_mask:08X}")
            print(f"  RSRP/RSRQ/SINR:{st.rsrp}/{st.rsrq}/{st.sinr} dBm/dB/dB")
            print(f"  Lat/Lon:       {st.latitude:.6f} / {st.longitude:.6f}")
            print()

        elif cmd == "state":
            print(f"FSM: {sim.get_fsm_state()}")

        else:
            print(f"Comando desconhecido: '{raw}'. Digite 'help'.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Simulador do medidor AMI")
    ap.add_argument("--port",    default=None,
                    help="Porta serial do ESP32 (ex: /dev/ttyUSB0 ou COM3)")
    ap.add_argument("--baud",    type=int, default=115200)
    ap.add_argument("--server",  default="127.0.0.1",
                    help="IP do servidor AMI Oracle Cloud")
    ap.add_argument("--udp-port",type=int, default=UDP_SERVER_PORT)
    ap.add_argument("--serial",  default="FAE00001234",
                    help="Número de série do medidor simulado")
    ap.add_argument("--no-esp32", action="store_true",
                    help="UDP direto para o servidor, sem ESP32 nem porta serial")
    ap.add_argument("--dry-run", action="store_true",
                    help="Sem rede nem hardware (só testa lógica da FSM)")
    ap.add_argument("--local-port", type=int, default=4060,
                    help="Porta UDP local do simulador em modo --no-esp32 (padrão: 4060)")
    ap.add_argument("--debug",   action="store_true")
    ap.add_argument("--n2-interval", type=int, default=0,
                    help="Envia N2 automaticamente a cada N segundos (0=desabilitado)")
    args = ap.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    # Cria interface AT
    if args.no_esp32:
        log.info("Modo NO-ESP32: UDP direto para %s:%d (local :%d)",
                 args.server, args.udp_port, args.local_port)
        at = ATSerialDirectUDP(
            server=args.server,
            port=args.udp_port,
            local_port=args.local_port,
        )
    elif args.dry_run or args.port is None:
        log.info("Modo DRY-RUN: sem hardware real")
        at = ATSerialDryRun()
    else:
        at = ATSerial(port=args.port, baudrate=args.baud)
        if not at.is_open():
            log.error("Não foi possível abrir a porta serial %s", args.port)
            sys.exit(1)

    # Estado inicial do medidor
    meter = MeterState(serial=args.serial)

    # Simulador
    sim = MeterSimulator(at, meter, args.server, args.udp_port)

    # Thread FSM
    fsm_thread = threading.Thread(target=sim.run, daemon=True, name="FSM")
    fsm_thread.start()

    # Thread de N2 automático
    if args.n2_interval > 0:
        def auto_n2():
            while sim._running:
                time.sleep(args.n2_interval)
                if sim._running:
                    log.info("Auto N2 (intervalo=%ds)", args.n2_interval)
                    sim.trigger_n2()
        threading.Thread(target=auto_n2, daemon=True, name="AutoN2").start()
        log.info("N2 automático a cada %d segundos", args.n2_interval)

    # CLI (thread principal)
    run_cli(sim)

    # Aguarda FSM encerrar
    fsm_thread.join(timeout=3.0)
    if at:
        at.close()
    log.info("Simulador encerrado.")


if __name__ == "__main__":
    main()
"""
at_serial.py — Interface AT serial para o ESP32 mock.

Replica o comportamento de uart_at.c do firmware MSP430:
  - sendCmd()   → uart_at_sendCmd()
  - readLine()  → uart_at_readLine()
  - expect()    → uart_at_expect()
  - sendRaw()   → uart_at_sendRaw()
  - flushRx()   → uart_at_flushRx()

Três modos de operação:
  ATSerial(port)      — ESP32 real via USB serial
  ATSerialDryRun()    — sem hardware, sem rede (só lógica)
  ATSerialLocalUDP()  — sem ESP32, envia UDP direto para localhost
                        (modo de desenvolvimento local completo)
"""

import time
import socket
import threading
import logging
from enum import IntEnum
from typing import Optional

import time
import threading
import logging
from enum import IntEnum
from typing import Optional

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

log = logging.getLogger("at_serial")


class ATResult(IntEnum):
    OK       = 0
    TIMEOUT  = 1
    ERROR    = 2
    OVERFLOW = 3


class ATSerial:
    """
    Wrapper de pyserial que expõe a mesma interface que uart_at.c.

    Parâmetros:
      port      — porta serial (ex: "COM3", "/dev/ttyUSB0") ou None para dry-run
      baudrate  — velocidade (padrão 115200, igual ao ESP32)
      timeout   — timeout padrão de leitura de linha, em segundos
    """

    DEFAULT_TIMEOUT = 5.0
    LONG_TIMEOUT    = 90.0
    LINE_MAX        = 192

    def __init__(self, port: Optional[str] = None, baudrate: int = 115200):
        self._port     = port
        self._baudrate = baudrate
        self._ser      = None
        self._lock     = threading.Lock()
        self._dry_run  = (port is None) or (not SERIAL_AVAILABLE)

        # Buffer de URCs recebidas enquanto aguarda uma resposta específica
        self._urc_queue: list[str] = []
        self._urc_callbacks: dict[str, callable] = {}

        if not self._dry_run:
            self._open()
        else:
            log.info("AT serial: modo DRY-RUN (sem porta serial)")

    # ------------------------------------------------------------------
    # Abertura / fechamento
    # ------------------------------------------------------------------
    def _open(self):
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,            # leitura não bloqueante
            )
            log.info("Serial aberta: %s @ %d bps", self._port, self._baudrate)
        except Exception as e:
            log.error("Erro ao abrir serial %s: %s — usando dry-run", self._port, e)
            self._dry_run = True

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()

    def is_open(self) -> bool:
        return self._dry_run or (self._ser is not None and self._ser.is_open)

    # ------------------------------------------------------------------
    # Registro de callbacks URC
    # ------------------------------------------------------------------
    def register_urc(self, prefix: str, callback: callable):
        """Registra callback para URCs (ex: "+QIURC:", "+CEREG:")."""
        self._urc_callbacks[prefix] = callback

    # ------------------------------------------------------------------
    # Envio
    # ------------------------------------------------------------------
    def send_cmd(self, cmd: str):
        """Envia comando AT + CRLF  (uart_at_sendCmd)."""
        line = cmd + "\r\n"
        log.debug("AT TX >> %s", cmd)
        if not self._dry_run and self._ser:
            with self._lock:
                self._ser.write(line.encode("ascii"))

    def send_raw(self, data: bytes):
        """Envia bytes binários sem terminador  (uart_at_sendRaw)."""
        log.debug("AT TX raw %d bytes", len(data))
        if not self._dry_run and self._ser:
            with self._lock:
                self._ser.write(data)

    def flush_rx(self):
        """Descarta buffer RX  (uart_at_flushRx)."""
        if not self._dry_run and self._ser:
            with self._lock:
                self._ser.reset_input_buffer()
        self._urc_queue.clear()

    # ------------------------------------------------------------------
    # Recepção de linha
    # ------------------------------------------------------------------
    def read_line(self, timeout: float = DEFAULT_TIMEOUT) -> tuple[ATResult, str]:
        """
        Lê uma linha completa da serial  (uart_at_readLine).
        Retorna (ATResult, linha_sem_crlf).
        """
        if self._dry_run:
            time.sleep(0.05)
            return ATResult.TIMEOUT, ""

        deadline = time.monotonic() + timeout
        buf = bytearray()

        while time.monotonic() < deadline:
            with self._lock:
                b = self._ser.read(1)
            if not b:
                continue
            c = b[0]
            if c == ord('\r'):
                continue
            if c == ord('\n'):
                line = buf.decode("ascii", errors="replace").strip()
                if not line:
                    buf.clear()
                    continue
                log.debug("AT RX << %s", line)
                return ATResult.OK, line
            if len(buf) < self.LINE_MAX:
                buf.append(c)

        return ATResult.TIMEOUT, buf.decode("ascii", errors="replace")

    # ------------------------------------------------------------------
    # expect — equivalente a uart_at_expect()
    # ------------------------------------------------------------------
    def expect(self, cmd: Optional[str], expect: Optional[str],
               timeout: float = DEFAULT_TIMEOUT) -> tuple[ATResult, str]:
        """
        Envia um comando AT e aguarda uma linha contendo `expect`.

        Replica uart_at_expect():
          - Envia `cmd` (se não None)
          - Lê linhas até encontrar `expect`, "OK" ou "ERROR"
          - URCs sem relação com `expect` são despachadas para callbacks
          - Retorna (ATResult, linha_que_casou)
        """
        self.flush_rx()
        if cmd:
            self.send_cmd(cmd)

        found_line   = ""
        found        = False
        deadline     = time.monotonic() + timeout

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break

            result, line = self.read_line(min(remaining, 0.5))
            if result == ATResult.TIMEOUT and not line:
                continue

            # Verifica resposta esperada ANTES dos callbacks URC.
            # Se a linha casa com expect_str, não despacha para URC —
            # evita que +CEREG: 1 seja consumido pelo callback quando
            # é justamente a resposta que estamos esperando.
            if expect and expect in line:
                found_line = line
                found      = True
                # Continua lendo até OK/ERROR para limpar buffer
                continue

            # Despacha URCs (só linhas que não casaram com expect_str)
            dispatched = False
            for prefix, cb in self._urc_callbacks.items():
                if line.startswith(prefix):
                    log.debug("URC dispatchada: %s", line)
                    cb(line)
                    dispatched = True
                    break

            if dispatched:
                continue

            if line == "OK":
                if found or expect is None:
                    return ATResult.OK, found_line
                continue

            if (line.startswith("ERROR") or
                line.startswith("+CME ERROR") or
                line.startswith("+CMS ERROR")):
                return ATResult.ERROR, line

        return (ATResult.OK if found else ATResult.TIMEOUT), found_line

    # ------------------------------------------------------------------
    # Envio binário com protocolo QISEND
    # ------------------------------------------------------------------
    def qisend(self, socket_id: int, data: bytes,
               host: str, port: int,
               timeout: float = 10.0) -> bool:
        """
        Executa o fluxo completo AT+QISEND:
          AT+QISEND=<id>,<len>,"<host>",<port>
          → aguarda ">"
          → envia bytes binários
          → aguarda "SEND OK"

        Replica udp_transport_send() do firmware.
        """
        if self._dry_run:
            log.info("DRY-RUN: QISEND %d bytes simulado", len(data))
            return True

        cmd = f'AT+QISEND={socket_id},{len(data)},"{host}",{port}'
        result, line = self.expect(cmd, ">", timeout=5.0)

        if result != ATResult.OK:
            log.error("QISEND: módulo não retornou '>' (result=%s line='%s')",
                      result.name, line)
            return False

        # Envia payload binário
        self.send_raw(data)

        # Aguarda "SEND OK"
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            r, ln = self.read_line(timeout=1.0)
            if ln == "SEND OK":
                log.info("QISEND: OK (%d bytes enviados)", len(data))
                return True
            if "SEND FAIL" in ln or "ERROR" in ln:
                log.error("QISEND: falha '%s'", ln)
                return False

        log.error("QISEND: timeout aguardando SEND OK")
        return False

    # ------------------------------------------------------------------
    # Recepção QIRD
    # ------------------------------------------------------------------
    def qird(self, socket_id: int, max_len: int = 512,
             timeout: float = 3.0) -> Optional[bytes]:
        """
        Executa AT+QIRD e retorna bytes recebidos (ou None se vazio).
        Replica udp_transport_recv() do firmware.

        NÃO usa expect() porque o protocolo QIRD mistura linhas de texto
        com payload binário no mesmo stream serial.
        expect() continuaria lendo linhas até o OK e consumiria os bytes
        binários antes que pudéssemos lê-los como dados brutos.

        Protocolo de 3 fases:
          Fase 1 — envia AT+QIRD, lê linhas até "+QIRD:" aparecer
          Fase 2 — lê exatamente recv_len bytes binários diretamente do serial
          Fase 3 — drena \r\nOK\r\n para limpar o buffer
        """
        if self._dry_run:
            return None

        # Fase 1 — envia comando e lê header "+QIRD: N,\"ip\",port"
        self.flush_rx()
        self.send_cmd(f"AT+QIRD={socket_id},{max_len}")

        recv_len = 0
        found    = False
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            r, line = self.read_line(timeout=min(deadline - time.monotonic(), 1.0))
            if r == ATResult.TIMEOUT and not line:
                continue

            # Despacha URCs que possam chegar enquanto aguarda o header
            urc_dispatched = False
            for prefix, cb in self._urc_callbacks.items():
                if line.startswith(prefix):
                    log.debug("QIRD fase1 URC: %s", line)
                    cb(line)
                    urc_dispatched = True
                    break
            if urc_dispatched:
                continue

            if "+QIRD:" in line:
                try:
                    recv_len = int(line.split(":")[1].split(",")[0].strip())
                except (ValueError, IndexError):
                    recv_len = 0
                found = True
                break   # ← para imediatamente: próximos bytes são binários

            if line in ("OK", "ERROR"):
                break

        if not found or recv_len == 0:
            # Drena até OK para deixar buffer limpo antes de retornar
            drain = time.monotonic() + 1.0
            while time.monotonic() < drain:
                r2, ln2 = self.read_line(timeout=0.3)
                if ln2 in ("OK", "ERROR") or r2 == ATResult.TIMEOUT:
                    break
            return None

        # Fase 2 — lê exatamente recv_len bytes binários do stream.
        # Usa _ser.read() diretamente para não interpretar \n como fim de linha.
        raw = bytearray()
        with self._lock:
            while len(raw) < recv_len and time.monotonic() < deadline:
                chunk = self._ser.read(recv_len - len(raw))
                if chunk:
                    raw.extend(chunk)

        # Fase 3 — drena \r\n + OK que o ESP32 envia após o payload
        drain = time.monotonic() + 1.5
        while time.monotonic() < drain:
            r3, ln3 = self.read_line(timeout=0.3)
            if ln3 == "OK" or r3 == ATResult.TIMEOUT:
                break

        log.info("QIRD: recebidos %d bytes", len(raw))
        return bytes(raw) if raw else None


# ---------------------------------------------------------------------------
# DirectUDP — modo sem ESP32: envia UDP direto para o servidor AMI
# Bypassa totalmente a interface AT serial.
# Usado com --no-esp32 no meter_simulator.py.
# ---------------------------------------------------------------------------
import socket as _socket

class ATSerialDirectUDP(ATSerial):
    """
    Replica a interface ATSerial mas envia/recebe UDP diretamente,
    sem ESP32 nem porta serial.

    Útil para desenvolvimento local: PC roda ami_server.py e
    meter_simulator.py sem nenhum hardware adicional.

    Fluxo:
      qisend(data) → socket.sendto(data, (server, port))
      qird()       → socket.recv() com timeout curto
    """

    def __init__(self, server: str, port: int = 4059, local_port: int = 4060):
        super().__init__(port=None)   # dry_run = True na classe pai
        self._server      = server
        self._server_port = port
        self._local_port  = local_port
        self._sock        = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", local_port))
        self._sock.settimeout(0.2)
        self._registered  = False
        self._step        = 0
        log.info("DirectUDP: enviando para %s:%d  (local :%d)",
                 server, port, local_port)

    def close(self):
        self._sock.close()

    def is_open(self) -> bool:
        return True

    def read_line(self, timeout=ATSerial.DEFAULT_TIMEOUT):
        self._step += 1
        if self._step == 1:
            # Simula "+QIND: ready" para triggar POWERING_ON
            time.sleep(0.05)
            return ATResult.OK, "+QIND: ready"
        time.sleep(0.05)
        return ATResult.TIMEOUT, ""

    def expect(self, cmd, expect_str, timeout=ATSerial.DEFAULT_TIMEOUT):
        time.sleep(0.02)
        if cmd:
            log.debug("DirectUDP AT (ignorado): %s", cmd)

        # Simula respostas relevantes para a FSM
        if cmd and "CEREG?" in cmd:
            if not self._registered:
                time.sleep(0.3)
                self._registered = True
            return ATResult.OK, "+CEREG: 0,1"

        if cmd and "QIOPEN" in cmd:
            return ATResult.OK, "+QIOPEN: 0,0"

        if cmd and "QENG" in cmd:
            return ATResult.OK, ('+QENG: "servingcell","NOCONN","NB-IoT",'
                                 '724,5,3A2B,12AB,0,28,-95,-12,8,-90,5')

        return ATResult.OK, ""

    def qisend(self, socket_id, data, host, port, timeout=10.0):
        """Envia UDP direto para o servidor AMI."""
        try:
            self._sock.sendto(data, (self._server, self._server_port))
            log.info("DirectUDP TX: %d bytes → %s:%d",
                     len(data), self._server, self._server_port)
            return True
        except Exception as e:
            log.error("DirectUDP TX error: %s", e)
            return False

    def qird(self, socket_id, max_len=512, timeout=3.0):
        """Recebe UDP do servidor AMI."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                data, addr = self._sock.recvfrom(max_len)
                log.info("DirectUDP RX: %d bytes de %s", len(data), addr)
                return data
            except _socket.timeout:
                pass
            except Exception as e:
                log.debug("DirectUDP RX: %s", e)
                break
        return None

    def send_raw(self, data):
        pass   # não usado no modo direto

    def flush_rx(self):
        # Drena socket
        self._sock.settimeout(0)
        try:
            while True:
                self._sock.recv(1024)
        except Exception:
            pass
        self._sock.settimeout(0.2)


# ---------------------------------------------------------------------------
# Dry-run mock — respostas automáticas para testes sem hardware
# ---------------------------------------------------------------------------
class ATSerialDryRun(ATSerial):
    """
    Versão dry-run com respostas pré-programadas.
    Útil para testar a FSM do simulador sem ESP32 físico.
    """
    def __init__(self):
        super().__init__(port=None)
        self._dry_run = True
        self._sim_registered = False
        self._sim_socket_open = False
        self._step = 0

    def expect(self, cmd, expect_str, timeout=ATSerial.DEFAULT_TIMEOUT):
        time.sleep(0.05)  # simula latência serial
        log.debug("DRY-RUN expect cmd='%s' expect='%s'", cmd, expect_str)

        # Simulações específicas
        if cmd and "+QIND" in (cmd or ""):
            return ATResult.OK, "+QIND: ready"

        if cmd == "AT+CEREG?":
            if not self._sim_registered:
                self._sim_registered = True
                time.sleep(0.3)
            return ATResult.OK, "+CEREG: 0,1"

        if cmd and "QIOPEN" in (cmd or ""):
            self._sim_socket_open = True
            return ATResult.OK, "+QIOPEN: 0,0"

        if cmd and "QENG" in (cmd or ""):
            return ATResult.OK, ('+QENG: "servingcell","NOCONN","NB-IoT",'
                                 '724,5,3A2B,12AB,0,28,-95,-12,8,-90,5')

        return ATResult.OK, ""

    def qisend(self, socket_id, data, host, port, timeout=10.0):
        log.info("DRY-RUN: QISEND %d bytes para %s:%d", len(data), host, port)
        time.sleep(0.1)
        return True

    def qird(self, socket_id, max_len=512, timeout=3.0):
        time.sleep(0.05)
        return None

    def read_line(self, timeout=ATSerial.DEFAULT_TIMEOUT):
        time.sleep(0.05)
        self._step += 1
        # Emite "+QIND: ready" na primeira leitura para triggar POWERING_ON
        if self._step == 1:
            return ATResult.OK, "+QIND: ready"
        return ATResult.TIMEOUT, ""


# ---------------------------------------------------------------------------
# ATSerialLocalUDP — modo local completo sem ESP32
# ---------------------------------------------------------------------------
class ATSerialLocalUDP(ATSerial):
    """
    Substitui o ESP32 por um socket UDP direto.

    O medidor simulado envia/recebe datagramas UDP direto para o servidor
    AMI rodando em localhost (ou qualquer host) — sem serial, sem ESP32.

    Fluxo:
      meter_simulator.py → ATSerialLocalUDP.qisend() → UDP → ami_server.py
      ami_server.py      → UDP → ATSerialLocalUDP (fila interna) → qird()

    Todos os comandos AT (CEREG, QIOPEN, QENG...) são simulados localmente,
    igual ao ATSerialDryRun, mas QISEND e QIRD usam UDP real.
    """

    def __init__(self, server_host: str = "127.0.0.1",
                 server_port: int = 4059,
                 local_port:  int = 4060):
        super().__init__(port=None)
        self._dry_run    = True          # desabilita pyserial
        self._server     = server_host
        self._srv_port   = server_port
        self._local_port = local_port

        # Socket UDP
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", local_port))
        self._sock.settimeout(0.05)

        # Fila de datagramas recebidos (downstream do servidor)
        self._rx_queue: list[bytes] = []
        self._rx_lock   = threading.Lock()

        # Thread de recepção UDP contínua
        self._rx_running = True
        self._rx_thread  = threading.Thread(
            target=self._rx_loop, daemon=True, name="UDP-RX"
        )
        self._rx_thread.start()

        # Estado simulado (igual ao ATSerialDryRun)
        self._sim_registered  = False
        self._sim_socket_open = False
        self._step = 0

        log.info("LocalUDP: %s:%d → servidor %s:%d",
                 "0.0.0.0", local_port, server_host, server_port)

    # ------------------------------------------------------------------
    # Thread de recepção UDP
    # ------------------------------------------------------------------
    def _rx_loop(self):
        while self._rx_running:
            try:
                data, addr = self._sock.recvfrom(1024)
                with self._rx_lock:
                    self._rx_queue.append(data)
                log.debug("LocalUDP RX: %d bytes de %s", len(data), addr)

                # Dispara URC recv para callbacks registrados
                urc_line = '+QIURC: "recv",0'
                for prefix, cb in self._urc_callbacks.items():
                    if urc_line.startswith(prefix):
                        cb(urc_line)
                        break
            except socket.timeout:
                continue
            except Exception as e:
                if self._rx_running:
                    log.debug("LocalUDP RX error: %s", e)

    # ------------------------------------------------------------------
    # Comandos AT simulados (mesmos do DryRun)
    # ------------------------------------------------------------------
    def read_line(self, timeout=ATSerial.DEFAULT_TIMEOUT):
        time.sleep(0.05)
        self._step += 1
        if self._step == 1:
            return ATResult.OK, "+QIND: ready"
        return ATResult.TIMEOUT, ""

    def expect(self, cmd, expect_str, timeout=ATSerial.DEFAULT_TIMEOUT):
        time.sleep(0.05)
        log.debug("LocalUDP expect cmd='%s'", cmd)

        if cmd == "AT+CEREG?":
            if not self._sim_registered:
                self._sim_registered = True
                time.sleep(0.3)
            return ATResult.OK, "+CEREG: 0,1"

        if cmd and "QIOPEN" in (cmd or ""):
            self._sim_socket_open = True
            log.info("LocalUDP: socket 'aberto' (UDP bind em :%d)", self._local_port)
            return ATResult.OK, "+QIOPEN: 0,0"

        if cmd and "QENG" in (cmd or ""):
            return ATResult.OK, (
                '+QENG: "servingcell","NOCONN","NB-IoT",'
                '724,5,3A2B,12AB,0,28,-95,-12,8,-90,5'
            )

        if cmd and "QICLOSE" in (cmd or ""):
            return ATResult.OK, ""

        return ATResult.OK, ""

    # ------------------------------------------------------------------
    # QISEND — envia UDP real
    # ------------------------------------------------------------------
    def qisend(self, socket_id, data: bytes, host: str, port: int,
               timeout: float = 10.0) -> bool:
        try:
            sent = self._sock.sendto(data, (self._server, self._srv_port))
            log.info("LocalUDP SEND: %d bytes → %s:%d",
                     sent, self._server, self._srv_port)
            return sent == len(data)
        except Exception as e:
            log.error("LocalUDP SEND error: %s", e)
            return False

    # ------------------------------------------------------------------
    # QIRD — lê da fila de recepção UDP
    # ------------------------------------------------------------------
    def qird(self, socket_id, max_len: int = 512,
             timeout: float = 3.0) -> Optional[bytes]:
        with self._rx_lock:
            if not self._rx_queue:
                return None
            data = self._rx_queue.pop(0)
        log.info("LocalUDP RECV: %d bytes", len(data))
        return data[:max_len]

    # ------------------------------------------------------------------
    # Fechamento
    # ------------------------------------------------------------------
    def close(self):
        self._rx_running = False
        try:
            self._sock.close()
        except Exception:
            pass
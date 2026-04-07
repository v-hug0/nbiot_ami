"""
dlms_builder.py — Construtor de APDUs DLMS/COSEM para o simulador do medidor.

Replica data_packager.c do firmware:
  build_n1(alarm, state) → bytes WRAPPER + Data-Notification N1
  build_n2(state)        → bytes WRAPPER + Data-Notification N2
  build_n3(state)        → bytes WRAPPER + Data-Notification N3

Formato alinhado com dlms_cosem.c e dlms_cosem.h do firmware MSP430.
"""

import struct
import time
from datetime import datetime, timezone
from dataclasses import dataclass, field
from typing import Optional
from enum import IntEnum


# ---------------------------------------------------------------------------
# Tipos de alarme N1 (E_PKG_ALARM_t de data_packager.h)
# ---------------------------------------------------------------------------
class Alarm(IntEnum):
    LAST_GASP     = 0
    TAMPER        = 1
    MAGNETIC      = 2
    VALVE_CHANGED = 3


# ---------------------------------------------------------------------------
# OBIS codes (dlms_cosem.h)
# ---------------------------------------------------------------------------
OBIS_CLOCK          = (0, 0,  1,  0,  0, 255)
OBIS_SERIAL         = (0, 0, 96,  1,  0, 255)
OBIS_FW_VER         = (0, 0, 96,  2,  0, 255)
OBIS_VOL_FORWARD    = (7, 0,  3,  0,  0, 255)
OBIS_VOL_REVERSE    = (7, 0,  4,  0,  0, 255)
OBIS_FLOW_RATE      = (7, 0, 11,  0,  0, 255)
OBIS_FLOW_MAX       = (7, 0, 22,  0,  0, 255)
OBIS_FLOW_MIN       = (7, 0, 23,  0,  0, 255)
OBIS_TEMPERATURE    = (7, 0, 41,  0,  0, 255)
OBIS_TEMP_MAX       = (7, 0, 55,  0,  0, 255)
OBIS_TEMP_MIN       = (7, 0, 56,  0,  0, 255)
OBIS_STATUS         = (0, 0, 96,  5,  4, 255)
OBIS_VALVE_STATUS   = (0, 0, 96,  3, 10, 255)
OBIS_VALVE_POSITION = (0, 0, 96,  3, 11, 255)
OBIS_NBIOT_RSRP     = (0, 0, 96, 12,  0, 255)
OBIS_NBIOT_RSRQ     = (0, 0, 96, 12,  1, 255)
OBIS_NBIOT_SINR     = (0, 0, 96, 12,  2, 255)
OBIS_NBIOT_IMEI     = (0, 0, 96, 12,  3, 255)
OBIS_LATITUDE       = (0, 0, 96,250,  0, 255)
OBIS_LONGITUDE      = (0, 0, 96,250,  1, 255)

# Bitmask de status (DLMS_STAT_* de dlms_cosem.h)
STAT_LEAKAGE        = 1 << 0
STAT_REVERSE_FLOW   = 1 << 1
STAT_AIR_IN_PIPE    = 1 << 2
STAT_BATTERY_LOW    = 1 << 3
STAT_TAMPER         = 1 << 4
STAT_MAX_OVERFLOW   = 1 << 5
STAT_NO_USAGE       = 1 << 6
STAT_OVER_TEMP      = 1 << 7
STAT_REV_INSTALL    = 1 << 8
STAT_LAST_GASP      = 1 << 9
STAT_MAGNETIC_ATTACK= 1 << 10
STAT_VALVE_FAULT    = 1 << 11
STAT_COMM_FAIL      = 1 << 12

WRAPPER_VERSION = 0x0001


# ---------------------------------------------------------------------------
# Estado do medidor (compartilhado com meter_simulator.py)
# ---------------------------------------------------------------------------
@dataclass
class MeterState:
    # Identificação
    serial:    str = "FAE00001234"
    imei:      str = "354668110000001"
    fw_ver:    str = "1.0.0"

    # Volumes (litros internamente, convertidos para m³ × 1000 = L no OBIS)
    vol_forward_L:  float = 0.0      # volume acumulado positivo (L)
    vol_reverse_L:  float = 0.0      # volume reverso (L)

    # Vazão (m³/h × 1000 internamente)
    flow_rate_mh3:  float = 0.0      # vazão atual (m³/h)
    flow_max_mh3:   float = 0.0
    flow_min_mh3:   float = 0.0

    # Temperatura (°C × 10 internamente → divide por 10 para exibir)
    temp_cx10:      int   = 215      # 21.5 °C
    temp_max_cx10:  int   = 215
    temp_min_cx10:  int   = 215

    # Alarmes (bitmask DLMS_STAT_*)
    status_mask:    int   = 0

    # Válvula (0=fechada, 1=aberta, 2=parcial, 3=movendo, 4=desconhecida)
    valve_status:   int   = 0        # 0 = open (100%)
    valve_position: int   = 100      # %

    # NB-IoT KPIs
    rsrp: int = -95
    rsrq: int = -12
    sinr: int = 8

    # Localização geográfica (N3)
    latitude:  float = -23.5505    # São Paulo
    longitude: float = -46.6333

    # Invoke ID incremental
    invoke_id: int = 1

    def next_invoke(self) -> int:
        v = self.invoke_id
        self.invoke_id = (self.invoke_id + 1) & 0xFFFFFF
        return v

    def set_alarm(self, bit: int):
        self.status_mask |= bit

    def clear_alarm(self, bit: int):
        self.status_mask &= ~bit

    def clear_all_alarms(self):
        self.status_mask = 0

    @property
    def vol_forward_L_int(self) -> int:
        """Volume em litros inteiros (uint32 para DLMS)."""
        return int(self.vol_forward_L)

    @property
    def vol_reverse_L_int(self) -> int:
        return int(self.vol_reverse_L)

    @property
    def flow_rate_int(self) -> int:
        """Vazão em m³/h × 1000 (uint32)."""
        return int(self.flow_rate_mh3 * 1000)

    @property
    def flow_max_int(self) -> int:
        return int(self.flow_max_mh3 * 1000)

    @property
    def flow_min_int(self) -> int:
        return int(self.flow_min_mh3 * 1000)


# ---------------------------------------------------------------------------
# Primitivas de codificação BER-TLV (dlms_cosem.c)
# ---------------------------------------------------------------------------
def _enc_len(n: int) -> bytes:
    if n < 0x80:
        return bytes([n])
    elif n < 0x100:
        return bytes([0x81, n])
    else:
        return bytes([0x82, (n >> 8) & 0xFF, n & 0xFF])


def _enc_obis(obis: tuple) -> bytes:
    """octet-string de 6 bytes representando o OBIS."""
    return bytes([0x09, 6]) + bytes(list(obis)[:6])


def _enc_null() -> bytes:
    return bytes([0x00])

def _enc_uint8(v: int) -> bytes:
    return bytes([0x11, v & 0xFF])

def _enc_uint16(v: int) -> bytes:
    return bytes([0x12]) + struct.pack(">H", v & 0xFFFF)

def _enc_uint32(v: int) -> bytes:
    return bytes([0x06]) + struct.pack(">I", v & 0xFFFFFFFF)

def _enc_int8(v: int) -> bytes:
    return bytes([0x0F]) + struct.pack("b", v)

def _enc_int16(v: int) -> bytes:
    return bytes([0x10]) + struct.pack(">h", v)

def _enc_int32(v: int) -> bytes:
    return bytes([0x05]) + struct.pack(">i", v)

def _enc_float32(v: float) -> bytes:
    return bytes([0x17]) + struct.pack(">f", v)

def _enc_visible_string(s: str) -> bytes:
    b = s.encode("ascii", errors="replace")
    return bytes([0x0A]) + _enc_len(len(b)) + b

def _enc_datetime(dt: Optional[datetime] = None) -> bytes:
    """
    Codifica date-time DLMS (12 octets, octet-string).
    Formato: year(2B) month day dow hour min sec cs tz(2B) status
    """
    if dt is None:
        dt = datetime.now(timezone.utc)
    year  = dt.year
    month = dt.month
    day   = dt.day
    dow   = dt.isoweekday() % 7  # 1=Mon..7=Sun → 0=Sun..6=Sat
    hour  = dt.hour
    minute= dt.minute
    second= dt.second
    cs    = dt.microsecond // 10000
    tz    = 0  # UTC

    raw = struct.pack(">H", year) + bytes([month, day, dow, hour, minute, second, cs])
    raw += struct.pack(">h", tz) + bytes([0xFF])  # clock_status = 0xFF (not specified)
    return bytes([0x09, 12]) + raw


def _capture_obj(obis: tuple, value_bytes: bytes) -> bytes:
    """structure { octet_string(6 obis), value }"""
    body = _enc_obis(obis) + value_bytes
    return bytes([0x02]) + _enc_len(2) + body


def _build_notification(invoke_id: int, objects: list,
                        with_ts: bool = True) -> bytes:
    """
    Constrói Data-Notification APDU (tag 0x0F) + WRAPPER.
    Replica dlms_buildDataNotification() do firmware.

    objects: lista de (obis_tuple, value_bytes)
    """
    # Array de capture objects
    items = b"".join(_capture_obj(obis, val) for obis, val in objects)
    array = bytes([0x01]) + _enc_len(len(objects)) + items

    # Long invoke-id (3 bytes)
    iid = bytes([
        (invoke_id >> 16) & 0xFF,
        (invoke_id >>  8) & 0xFF,
         invoke_id & 0xFF,
    ])

    # APDU: tag + invoke_id + datetime opcional + corpo
    apdu = bytes([0x0F]) + iid
    if with_ts:
        apdu += _enc_datetime()
    apdu += array

    # WRAPPER IEC 62056-47
    hdr = struct.pack(">HHHH", WRAPPER_VERSION, 0x0001, 0x0001, len(apdu))
    return hdr + apdu


# ---------------------------------------------------------------------------
# Builders públicos — alinhados com data_packager.c
# ---------------------------------------------------------------------------

def build_n1(alarm: Alarm, state: MeterState) -> bytes:
    """
    data_pkgr_buildN1() — Alarme imediato.

    OBIS transmitidos (data_packager.h):
      OBIS_CLOCK, OBIS_STATUS, OBIS_VALVE_STATUS, OBIS_SERIAL
    """
    # Define bitmask de acordo com o tipo de alarme
    mask = state.status_mask
    if alarm == Alarm.LAST_GASP:
        mask |= STAT_LAST_GASP
    elif alarm == Alarm.TAMPER:
        mask |= STAT_TAMPER
    elif alarm == Alarm.MAGNETIC:
        mask |= STAT_MAGNETIC_ATTACK
    elif alarm == Alarm.VALVE_CHANGED:
        pass  # status já reflete o estado da válvula

    iid = state.next_invoke()
    objects = [
        (OBIS_CLOCK,        _enc_datetime()),
        (OBIS_STATUS,       _enc_uint32(mask)),
        (OBIS_VALVE_STATUS, _enc_uint8(state.valve_status)),
        (OBIS_SERIAL,       _enc_visible_string(state.serial)),
    ]
    return _build_notification(iid, objects)


def build_n2(state: MeterState) -> bytes:
    """
    data_pkgr_buildN2() — Pacote diário completo.

    OBIS transmitidos (AMI-001 §5.38):
      Serial, FW, Vol Forward, Vol Reverse, Flow Rate, Flow Max, Flow Min,
      Temperatura, Status, Valve Status, Valve Position,
      RSRP, RSRQ, SINR, IMEI, Clock
    """
    iid = state.next_invoke()
    objects = [
        (OBIS_CLOCK,         _enc_datetime()),
        (OBIS_SERIAL,        _enc_visible_string(state.serial)),
        (OBIS_FW_VER,        _enc_visible_string(state.fw_ver)),
        (OBIS_VOL_FORWARD,   _enc_uint32(state.vol_forward_L_int)),
        (OBIS_VOL_REVERSE,   _enc_uint32(state.vol_reverse_L_int)),
        (OBIS_FLOW_RATE,     _enc_uint32(state.flow_rate_int)),
        (OBIS_FLOW_MAX,      _enc_uint32(state.flow_max_int)),
        (OBIS_FLOW_MIN,      _enc_uint32(state.flow_min_int)),
        (OBIS_TEMPERATURE,   _enc_int16(state.temp_cx10)),
        (OBIS_TEMP_MAX,      _enc_int16(state.temp_max_cx10)),
        (OBIS_TEMP_MIN,      _enc_int16(state.temp_min_cx10)),
        (OBIS_STATUS,        _enc_uint32(state.status_mask)),
        (OBIS_VALVE_STATUS,  _enc_uint8(state.valve_status)),
        (OBIS_VALVE_POSITION,_enc_uint8(state.valve_position)),
        (OBIS_NBIOT_RSRP,    _enc_int16(state.rsrp)),
        (OBIS_NBIOT_RSRQ,    _enc_int16(state.rsrq)),
        (OBIS_NBIOT_SINR,    _enc_int16(state.sinr)),
        (OBIS_NBIOT_IMEI,    _enc_visible_string(state.imei)),
    ]
    return _build_notification(iid, objects)


def build_n3(state: MeterState) -> bytes:
    """
    data_pkgr_buildN3() — Pacote mensal com localização geográfica.
    """
    iid = state.next_invoke()
    objects = [
        (OBIS_CLOCK,     _enc_datetime()),
        (OBIS_SERIAL,    _enc_visible_string(state.serial)),
        (OBIS_LATITUDE,  _enc_float32(state.latitude)),
        (OBIS_LONGITUDE, _enc_float32(state.longitude)),
    ]
    return _build_notification(iid, objects)


def build_get_response(invoke_id: int, state: MeterState) -> bytes:
    """
    data_pkgr_buildGetResponse() — Resposta a GET-Request.
    Retorna um subconjunto dos dados N2 (leitura sob demanda).
    """
    # GET-Response-Normal (tag 0xC4)
    objects = [
        (OBIS_VOL_FORWARD,  _enc_uint32(state.vol_forward_L_int)),
        (OBIS_FLOW_RATE,    _enc_uint32(state.flow_rate_int)),
        (OBIS_TEMPERATURE,  _enc_int16(state.temp_cx10)),
        (OBIS_STATUS,       _enc_uint32(state.status_mask)),
        (OBIS_VALVE_STATUS, _enc_uint8(state.valve_status)),
    ]
    items = b"".join(_capture_obj(o, v) for o, v in objects)
    array = bytes([0x01]) + _enc_len(len(objects)) + items

    apdu = bytes([0xC4, 0x01])                  # GET-Response-Normal
    apdu += struct.pack(">I", invoke_id)         # invoke_id
    apdu += bytes([0x00])                         # result: data (OK)
    apdu += array

    hdr = struct.pack(">HHHH", WRAPPER_VERSION, 0x0001, 0x0001, len(apdu))
    return hdr + apdu

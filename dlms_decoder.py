"""
dlms_decoder.py — Decodificador DLMS/COSEM para o servidor AMI.

Suporta:
  - WRAPPER IEC 62056-47 (8 bytes de cabeçalho)
  - Data-Notification (tag 0x0F) — push N1/N2/N3 do medidor
  - GET-Response (tag 0xC4)     — resposta a leitura sob demanda
  - GET-Request  (tag 0xC0)     — leitura recebida (eco/debug)
  - ACTION-Request (tag 0xC3)   — comando de válvula / FOTA

Mapeamento de OBIS codes alinhado com dlms_cosem.h do firmware.
"""

import struct
from dataclasses import dataclass, field
from typing import Optional, Any

# ---------------------------------------------------------------------------
# WRAPPER IEC 62056-47
# ---------------------------------------------------------------------------
WRAPPER_VERSION  = 0x0001
WRAPPER_HDR_LEN  = 8   # version(2) + wport_src(2) + wport_dst(2) + length(2)

# ---------------------------------------------------------------------------
# APDU tags xDLMS
# ---------------------------------------------------------------------------
TAG_DATA_NOTIFICATION = 0x0F
TAG_GET_REQUEST       = 0xC0
TAG_GET_RESPONSE      = 0xC4
TAG_SET_REQUEST       = 0xC1
TAG_SET_RESPONSE      = 0xC5
TAG_ACTION_REQUEST    = 0xC3
TAG_ACTION_RESPONSE   = 0xC7

# ---------------------------------------------------------------------------
# Tipos de dado DLMS (IEC 62056-62 Table 2) — alinhado com dlms_cosem.h
# ---------------------------------------------------------------------------
DLMS_TYPE = {
    0x00: "null",
    0x01: "array",
    0x02: "structure",
    0x03: "boolean",
    0x05: "int32",
    0x06: "uint32",
    0x09: "octet_string",
    0x0A: "visible_string",
    0x0F: "int8",
    0x10: "int16",
    0x11: "uint8",
    0x12: "uint16",
    0x14: "int64",
    0x15: "uint64",
    0x17: "float32",
    0x18: "float64",
    0x19: "datetime",
    0x1A: "date",
    0x1B: "time",
}

# ---------------------------------------------------------------------------
# Mapa OBIS -> (nome, descricao, fator_escala)  [alinhado com dlms_cosem.h]
# ---------------------------------------------------------------------------
OBIS_MAP = {
    (0, 0,  1,  0,  0, 255): ("Clock",          "Timestamp do medidor",         None),
    (0, 0, 96,  1,  0, 255): ("Serial",          "Numero de serie",              None),
    (0, 0, 96,  2,  0, 255): ("FW Version",      "Versao de firmware",           None),
    (7, 0,  3,  0,  0, 255): ("Vol Forward",     "Volume acumulado (L)",         0.001),
    (7, 0,  4,  0,  0, 255): ("Vol Reverse",     "Volume reverso (L)",           0.001),
    (7, 0, 11,  0,  0, 255): ("Flow Rate",       "Vazao atual (m3/h)",           0.001),
    (7, 0, 22,  0,  0, 255): ("Flow Max",        "Vazao maxima no intervalo",    0.001),
    (7, 0, 23,  0,  0, 255): ("Flow Min",        "Vazao minima no intervalo",    0.001),
    (7, 0, 41,  0,  0, 255): ("Temperature",     "Temperatura da agua (C)",      0.1),
    (7, 0, 55,  0,  0, 255): ("Temp Max",        "Temperatura maxima (C)",       0.1),
    (7, 0, 56,  0,  0, 255): ("Temp Min",        "Temperatura minima (C)",       0.1),
    (0, 0, 96,  5,  4, 255): ("Status",          "Bitmask de alarmes/status",    None),
    (0, 0, 96,  3, 10, 255): ("Valve Status",    "Status da valvula",            None),
    (0, 0, 96,  3, 11, 255): ("Valve Position",  "Posicao da valvula (%)",       None),
    (0, 0, 96, 12,  0, 255): ("RSRP",            "NB-IoT RSRP (dBm)",           None),
    (0, 0, 96, 12,  1, 255): ("RSRQ",            "NB-IoT RSRQ (dB)",            None),
    (0, 0, 96, 12,  2, 255): ("SINR",            "NB-IoT SINR (dB)",            None),
    (0, 0, 96, 12,  3, 255): ("IMEI",            "IMEI do modulo NB-IoT",        None),
    (0, 0, 96,  1,  1, 255): ("IMEI alt",        "IMEI do modulo",               None),
    (0, 0, 96,  1,  2, 255): ("ICCID",           "ICCID do eSIM",                None),
    (1, 0, 96,  6,  0, 255): ("Battery Level",   "Nivel de bateria (%)",         None),
    (1, 0, 96,  6,  1, 255): ("Battery Days",    "Dias restantes de bateria",    None),
    (0, 0, 96,240,  9, 255): ("Alarm Bitmask",   "Bitmask de alarmes ativos",    None),
    (0, 0, 96,250,  0, 255): ("Latitude",        "Latitude GPS",                 1e-6),
    (0, 0, 96,250,  1, 255): ("Longitude",       "Longitude GPS",                1e-6),
    (7, 1, 99,  2,  0, 255): ("Profile Generic", "Perfil de consumo horario",    None),
    # Script Table (OBIS de acoes downstream)
    (0, 0, 96, 10,  1, 255): ("Valve Action",    "Acao: controle de valvula",    None),
    (0, 0, 96, 10,  2, 255): ("FOTA Start",      "Acao: iniciar OTA",            None),
    (0, 0, 96, 10,  3, 255): ("Vol Limit",       "Acao: volume autorizado",      0.001),
}

# Bitmask de status (OBIS_STATUS — dlms_cosem.h)
STATUS_BITS = {
    0: "Leakage",
    1: "Reverse flow",
    2: "Air in pipe",
    3: "Battery low",
    4: "Tamper",
    5: "Max overflow",
    6: "No usage",
    7: "Over temperature",
    8: "Reverse installation",
    9: "Last Gasp",
    10: "Magnetic attack",
    11: "Valve fault",
    12: "Comm fail",
}

VALVE_STATUS_MAP = {0: "Open (100%)", 1: "Closed (0%)", 2: "Intermediate",
                    3: "Moving", 4: "Unknown"}


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------
@dataclass
class WrapperHeader:
    version:   int
    wport_src: int
    wport_dst: int
    apdu_len:  int

@dataclass
class ObisValue:
    obis:        tuple
    name:        str
    description: str
    raw:         Any
    scaled:      Optional[float]
    unit:        str = ""
    extra:       str = ""    # informacao extra (ex: alarmes ativos)

@dataclass
class DecodedApdu:
    tag:       int
    tag_name:  str
    invoke_id: int
    timestamp: Optional[str]
    objects:   list = field(default_factory=list)
    raw_apdu:  bytes = b""
    errors:    list = field(default_factory=list)


# ---------------------------------------------------------------------------
# Buffer de leitura
# ---------------------------------------------------------------------------
class Buffer:
    def __init__(self, data: bytes):
        self._d   = data
        self._pos = 0

    @property
    def remaining(self) -> int:
        return len(self._d) - self._pos

    def peek(self, n: int = 1) -> bytes:
        return self._d[self._pos: self._pos + n]

    def read(self, n: int) -> bytes:
        if n > self.remaining:
            raise ValueError(f"Buffer underflow: need {n}, have {self.remaining}")
        c = self._d[self._pos: self._pos + n]
        self._pos += n
        return c

    def read_u8(self)  -> int: return struct.unpack("B",  self.read(1))[0]
    def read_u16(self) -> int: return struct.unpack(">H", self.read(2))[0]
    def read_u32(self) -> int: return struct.unpack(">I", self.read(4))[0]
    def read_i8(self)  -> int: return struct.unpack("b",  self.read(1))[0]
    def read_i16(self) -> int: return struct.unpack(">h", self.read(2))[0]
    def read_i32(self) -> int: return struct.unpack(">i", self.read(4))[0]
    def read_i64(self) -> int: return struct.unpack(">q", self.read(8))[0]
    def read_u64(self) -> int: return struct.unpack(">Q", self.read(8))[0]
    def read_f32(self) -> float: return struct.unpack(">f", self.read(4))[0]
    def read_f64(self) -> float: return struct.unpack(">d", self.read(8))[0]

    def read_length(self) -> int:
        first = self.read_u8()
        if first & 0x80:
            nb = first & 0x7F
            v = 0
            for _ in range(nb):
                v = (v << 8) | self.read_u8()
            return v
        return first


# ---------------------------------------------------------------------------
# Decodificacao de tipos DLMS
# ---------------------------------------------------------------------------
def _decode_datetime(buf: Buffer) -> str:
    try:
        d = buf.read(12)
        year = struct.unpack(">H", d[0:2])[0]
        mo, day, dow = d[2], d[3], d[4]
        hh, mm, ss, cs = d[5], d[6], d[7], d[8]
        tz = struct.unpack(">h", d[9:11])[0]
        return f"{year:04d}-{mo:02d}-{day:02d} {hh:02d}:{mm:02d}:{ss:02d} (tz={tz}min)"
    except Exception as e:
        return f"<datetime error: {e}>"


def _decode_value(buf: Buffer) -> Any:
    if buf.remaining == 0:
        return None
    tag = buf.read_u8()
    if tag == 0x00: return None
    if tag == 0x03: return bool(buf.read_u8())
    if tag == 0x05: return buf.read_i32()
    if tag == 0x06: return buf.read_u32()
    if tag == 0x0F: return buf.read_i8()
    if tag == 0x10: return buf.read_i16()
    if tag == 0x11: return buf.read_u8()
    if tag == 0x12: return buf.read_u16()
    if tag == 0x14: return buf.read_i64()
    if tag == 0x15: return buf.read_u64()
    if tag == 0x17: return buf.read_f32()
    if tag == 0x18: return buf.read_f64()
    if tag == 0x19: return _decode_datetime(buf)
    if tag in (0x09, 0x0A):
        ln = buf.read_length()
        raw = buf.read(ln)
        if tag == 0x09 and ln == 12:
            return _decode_datetime(Buffer(raw))
        try:
            return raw.decode("ascii", errors="replace")
        except Exception:
            return raw.hex()
    if tag in (0x01, 0x02):   # array / structure
        count = buf.read_length()
        return [_decode_value(buf) for _ in range(count)]
    # Desconhecido
    rest = buf.read(buf.remaining)
    return f"<0x{tag:02X}: {rest.hex()}>"


# ---------------------------------------------------------------------------
# Parser do corpo Data-Notification
# ---------------------------------------------------------------------------
def _parse_capture_list(buf: Buffer) -> list:
    """
    Decode the notification body emitted by dlms_buildDataNotification().
    Format: array [ structure { octet_string(6 obis), <typed value> }, ... ]
    """
    results = []
    if buf.remaining < 2:
        return results

    top_tag = buf.read_u8()
    if top_tag not in (0x01, 0x02):
        return results
    count = buf.read_length()

    for _ in range(count):
        if buf.remaining < 3:
            break
        struct_tag = buf.read_u8()
        if struct_tag != 0x02:
            break
        n_fields = buf.read_length()
        if n_fields < 2:
            break

        # Field 0: OBIS octet-string (6 bytes)
        obis_tag = buf.read_u8()
        if obis_tag != 0x09:
            break
        obis_len = buf.read_length()
        obis_raw = buf.read(obis_len)
        obis = tuple(obis_raw) if len(obis_raw) == 6 else (0,)*6

        # Field 1..n-1: value(s)
        values = [_decode_value(buf) for _ in range(n_fields - 1)]
        raw_val = values[0] if len(values) == 1 else values

        meta  = OBIS_MAP.get(obis)
        if meta:
            name, desc, scale = meta
        else:
            obis_str = ".".join(str(b) for b in obis)
            name, desc, scale = f"OBIS {obis_str}", "Desconhecido", None

        scaled = None
        unit   = ""
        extra  = ""
        if scale is not None and isinstance(raw_val, (int, float)):
            scaled = raw_val * scale

        # Formatacao especial
        if obis == (0, 0, 96, 5, 4, 255) and isinstance(raw_val, int):
            active = [STATUS_BITS[b] for b in range(13) if raw_val & (1 << b)]
            extra  = " | ".join(active) if active else "OK"
        if obis in ((0, 0, 96, 3, 10, 255),):
            extra = VALVE_STATUS_MAP.get(raw_val, "")
        if scale == 0.001:
            unit = "m3"
        elif scale == 0.1:
            unit = "C"
        elif scale == 1e-6:
            unit = "deg"

        results.append(ObisValue(
            obis=obis, name=name, description=desc,
            raw=raw_val, scaled=scaled, unit=unit, extra=extra,
        ))

    return results


# ---------------------------------------------------------------------------
# Parser principal
# ---------------------------------------------------------------------------
def parse_wrapper(data: bytes):
    if len(data) < WRAPPER_HDR_LEN:
        return None, data
    ver, src, dst, ln = struct.unpack(">HHHH", data[:8])
    if ver != WRAPPER_VERSION:
        return None, data
    apdu = data[8: 8 + ln]
    return WrapperHeader(ver, src, dst, ln), apdu


def decode_packet(data: bytes):
    """
    Ponto de entrada principal.
    Retorna (WrapperHeader, DecodedApdu).
    """
    hdr, apdu = parse_wrapper(data)
    if hdr is None or len(apdu) < 1:
        return hdr, None

    buf = Buffer(apdu)
    tag = buf.read_u8()

    NAMES = {
        0x0F: "Data-Notification",
        0xC0: "GET-Request",
        0xC4: "GET-Response",
        0xC1: "SET-Request",
        0xC5: "SET-Response",
        0xC3: "ACTION-Request",
        0xC7: "ACTION-Response",
    }
    dec = DecodedApdu(tag=tag, tag_name=NAMES.get(tag, f"0x{tag:02X}"),
                      invoke_id=0, timestamp=None, raw_apdu=apdu)
    try:
        if tag == TAG_DATA_NOTIFICATION:
            _parse_data_notification(buf, dec)
        elif tag == TAG_GET_RESPONSE:
            _parse_get_response(buf, dec)
        elif tag == TAG_GET_REQUEST:
            _parse_get_request(buf, dec)
        elif tag == TAG_ACTION_REQUEST:
            _parse_action_request(buf, dec)
        else:
            dec.errors.append(f"Tag 0x{tag:02X} sem parser dedicado")
    except Exception as e:
        dec.errors.append(f"Parse error: {e}")

    return hdr, dec


def _parse_data_notification(buf: Buffer, dec: DecodedApdu):
    if buf.remaining < 3:
        return
    b0, b1, b2 = buf.read_u8(), buf.read_u8(), buf.read_u8()
    dec.invoke_id = (b0 << 16) | (b1 << 8) | b2

    # datetime opcional
    if buf.remaining > 0 and buf.peek(1)[0] == 0x09:
        buf.read_u8()
        dt_len = buf.read_length()
        dt_raw = buf.read(dt_len)
        dec.timestamp = _decode_datetime(Buffer(dt_raw))

    dec.objects = _parse_capture_list(buf)


def _parse_get_response(buf: Buffer, dec: DecodedApdu):
    if buf.remaining < 5:
        return
    buf.read_u8()   # sub-type
    dec.invoke_id = buf.read_u32()
    if buf.remaining < 1:
        return
    result_tag = buf.read_u8()
    if result_tag != 0x00:
        code = buf.read_u8() if buf.remaining else 0
        dec.errors.append(f"GET-Response error 0x{code:02X}")
        return
    if buf.remaining > 0:
        val = _decode_value(buf)
        dec.objects.append(ObisValue(
            obis=(0,)*6, name="GET-Response", description="Dado retornado",
            raw=val, scaled=None,
        ))


def _parse_get_request(buf: Buffer, dec: DecodedApdu):
    if buf.remaining < 5:
        return
    buf.read_u8()
    dec.invoke_id = buf.read_u32()
    if buf.remaining < 8:
        return
    buf.read_u16()          # class_id
    obis = tuple(buf.read(6))
    meta = OBIS_MAP.get(obis)
    name = meta[0] if meta else "OBIS " + ".".join(str(b) for b in obis)
    dec.objects.append(ObisValue(
        obis=obis, name=name, description="GET-Request target",
        raw=None, scaled=None,
    ))


def _parse_action_request(buf: Buffer, dec: DecodedApdu):
    if buf.remaining < 5:
        return
    buf.read_u8()
    dec.invoke_id = buf.read_u32()
    if buf.remaining < 9:
        return
    buf.read_u8()            # service-class
    class_id = buf.read_u16()
    obis = tuple(buf.read(6))
    meta = OBIS_MAP.get(obis)
    name = meta[0] if meta else "OBIS " + ".".join(str(b) for b in obis)
    param = None
    if buf.remaining >= 2:
        buf.read_u8()        # method_id
        has_param = buf.read_u8()
        if has_param and buf.remaining:
            param = _decode_value(buf)
    dec.objects.append(ObisValue(
        obis=obis, name=name,
        description=f"ACTION class={class_id} param={param}",
        raw=param, scaled=None,
    ))


# ---------------------------------------------------------------------------
# Builders de comandos downstream
# ---------------------------------------------------------------------------
def _wrap(apdu: bytes) -> bytes:
    return struct.pack(">HHHH", WRAPPER_VERSION, 0x0001, 0x0001, len(apdu)) + apdu


def build_valve_command(invoke_id: int, position_pct: int) -> bytes:
    """
    ACTION-Request para controlar valvula.
    OBIS 0.0.96.10.1.255  — Script Table.
    Posicoes: 0, 25, 50, 75, 100 -> enum 0..4
    """
    enum_val = {0: 0, 25: 1, 50: 2, 75: 3, 100: 4}.get(position_pct, 0)
    apdu = bytes([TAG_ACTION_REQUEST, 0x01])
    apdu += struct.pack(">I", invoke_id)
    apdu += bytes([0x00, 0x00, 0x09,
                   0x00, 0x00, 0x60, 0x0A, 0x01, 0xFF,
                   0x01, 0x01,
                   0x16, enum_val])
    return _wrap(apdu)


def build_fota_start(invoke_id: int, file_size: int, crc32: int) -> bytes:
    """ACTION-Request para iniciar OTA. OBIS 0.0.96.10.2.255."""
    param = bytes([0x02, 0x02, 0x06]) + struct.pack(">I", file_size)
    param += bytes([0x06]) + struct.pack(">I", crc32)
    apdu = bytes([TAG_ACTION_REQUEST, 0x01])
    apdu += struct.pack(">I", invoke_id)
    apdu += bytes([0x00, 0x00, 0x09,
                   0x00, 0x00, 0x60, 0x0A, 0x02, 0xFF,
                   0x01, 0x01]) + param
    return _wrap(apdu)


def build_get_request(invoke_id: int, obis: tuple,
                      class_id: int = 1, attr_id: int = 2) -> bytes:
    """GET-Request-Normal para leitura sob demanda."""
    apdu = bytes([TAG_GET_REQUEST, 0x01])
    apdu += struct.pack(">I", invoke_id)
    apdu += struct.pack(">H", class_id)
    apdu += bytes(list(obis)[:6])
    apdu += bytes([attr_id, 0x00])
    return _wrap(apdu)


# ---------------------------------------------------------------------------
# Pretty-print
# ---------------------------------------------------------------------------
def format_decoded(hdr: WrapperHeader, dec: DecodedApdu) -> str:
    lines = ["=" * 58]
    lines.append(f"WRAPPER  ver=0x{hdr.version:04X}  src={hdr.wport_src}  "
                 f"dst={hdr.wport_dst}  len={hdr.apdu_len}B")
    lines.append(f"APDU     {dec.tag_name}  invoke_id={dec.invoke_id}")
    if dec.timestamp:
        lines.append(f"Timestamp  {dec.timestamp}")
    for e in dec.errors:
        lines.append(f"  [ERRO] {e}")
    for obj in dec.objects:
        obis_str = ".".join(str(b) for b in obj.obis)
        val_str = (f"{obj.scaled:.4f} {obj.unit}"
                   if obj.scaled is not None else str(obj.raw))
        extra = f"  [{obj.extra}]" if obj.extra else ""
        lines.append(f"  {obis_str:<22} {obj.name:<18} {val_str}{extra}")
    return "\n".join(lines)

"""
test_decoder.py — Valida o dlms_decoder.py com pacotes gerados localmente.

Executa sem hardware: gera APDUs N1 e N2 manualmente (byte a byte,
seguindo o mesmo formato que dlms_buildDataNotification() do firmware)
e confirma que o decoder extrai os OBIS corretos.
"""

import struct
import socket
import time
import sys
from dlms_decoder import (
    decode_packet, format_decoded,
    build_valve_command, build_fota_start, build_get_request,
    WRAPPER_VERSION,
)

PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"
_results = []

def check(name: str, cond: bool, detail: str = ""):
    status = PASS if cond else FAIL
    print(f"  {status}  {name}" + (f"  [{detail}]" if detail else ""))
    _results.append(cond)
    return cond


# ---------------------------------------------------------------------------
# Builder de pacotes de teste (replica dlms_cosem.c do firmware)
# ---------------------------------------------------------------------------
def encode_length(n: int) -> bytes:
    if n < 0x80:
        return bytes([n])
    elif n < 0x100:
        return bytes([0x81, n])
    else:
        return bytes([0x82, (n >> 8) & 0xFF, n & 0xFF])


def encode_obis(obis: tuple) -> bytes:
    """OBIS ref como octet-string de 6 bytes."""
    return bytes([0x09, 6]) + bytes(list(obis)[:6])


def encode_uint32(v: int) -> bytes:
    return bytes([0x06]) + struct.pack(">I", v)

def encode_uint16(v: int) -> bytes:
    return bytes([0x12]) + struct.pack(">H", v)

def encode_uint8(v: int) -> bytes:
    return bytes([0x11, v & 0xFF])

def encode_int16(v: int) -> bytes:
    return bytes([0x10]) + struct.pack(">h", v)

def encode_visible_string(s: str) -> bytes:
    b = s.encode("ascii")
    return bytes([0x0A]) + encode_length(len(b)) + b

def encode_datetime() -> bytes:
    """2026-04-06 12:00:00 UTC como octet-string de 12 bytes."""
    dt = struct.pack(">H", 2026) + bytes([4, 6, 1, 12, 0, 0, 0, 0xFF, 0, 0])
    return bytes([0x09, 12]) + dt

def encode_capture_object(obis: tuple, value_bytes: bytes) -> bytes:
    """structure { obis_ref, value }"""
    body = encode_obis(obis) + value_bytes
    return bytes([0x02]) + encode_length(2) + body


def build_data_notification(invoke_id: int, objects: list) -> bytes:
    """
    Constroi Data-Notification (tag 0x0F) identico ao firmware:
      long-invoke-id (3B) | datetime | array de capture objects
    """
    # Corpo: array de structures
    items = b"".join(encode_capture_object(obis, val) for obis, val in objects)
    array_body = bytes([0x01]) + encode_length(len(objects)) + items

    # APDU
    iid_bytes = bytes([(invoke_id >> 16) & 0xFF,
                       (invoke_id >> 8) & 0xFF,
                        invoke_id & 0xFF])
    apdu = bytes([0x0F]) + iid_bytes + encode_datetime() + array_body

    # WRAPPER
    hdr = struct.pack(">HHHH", WRAPPER_VERSION, 0x0001, 0x0001, len(apdu))
    return hdr + apdu


# ---------------------------------------------------------------------------
# Teste 1: Pacote N1 (alarme)
# ---------------------------------------------------------------------------
def test_n1():
    print("\n--- Teste 1: Data-Notification N1 (alarme Last Gasp + Tamper) ---")
    OBIS_CLOCK   = (0, 0,  1,  0,  0, 255)
    OBIS_STATUS  = (0, 0, 96,  5,  4, 255)
    OBIS_VALVE   = (0, 0, 96,  3, 10, 255)
    OBIS_SERIAL  = (0, 0, 96,  1,  0, 255)

    # bitmask: bit9=LastGasp, bit4=Tamper -> 0x0210
    status_mask = (1 << 9) | (1 << 4)

    pkt = build_data_notification(
        invoke_id=1,
        objects=[
            (OBIS_CLOCK,   encode_datetime()),
            (OBIS_STATUS,  encode_uint32(status_mask)),
            (OBIS_VALVE,   encode_uint8(1)),     # 1=Closed
            (OBIS_SERIAL,  encode_visible_string("FAE00001234")),
        ]
    )

    hdr, dec = decode_packet(pkt)
    check("WRAPPER valido",          hdr is not None)
    check("Tag Data-Notification",   dec is not None and dec.tag == 0x0F)
    check("Invoke ID correto",       dec.invoke_id == 1, str(dec.invoke_id))
    check("Timestamp decodificado",  dec.timestamp is not None, str(dec.timestamp))
    check("4 objetos decodificados", len(dec.objects) == 4, str(len(dec.objects)))

    # Status
    status_obj = next((o for o in dec.objects if o.obis == (0,0,96,5,4,255)), None)
    check("OBIS Status presente",    status_obj is not None)
    check("Bitmask correto",         status_obj and status_obj.raw == status_mask,
          hex(status_obj.raw) if status_obj else "?")
    check("Last Gasp no extra",      status_obj and "Last Gasp" in status_obj.extra,
          status_obj.extra if status_obj else "")
    check("Tamper no extra",         status_obj and "Tamper" in status_obj.extra)

    # Valve
    valve_obj = next((o for o in dec.objects if o.obis == (0,0,96,3,10,255)), None)
    check("OBIS Valve presente",     valve_obj is not None)
    check("Valve = Closed",          valve_obj and "Closed" in valve_obj.extra,
          valve_obj.extra if valve_obj else "")

    # Serial
    serial_obj = next((o for o in dec.objects if o.obis == (0,0,96,1,0,255)), None)
    check("Serial decodificado",     serial_obj and "FAE00001234" in str(serial_obj.raw),
          str(serial_obj.raw) if serial_obj else "")

    print(format_decoded(hdr, dec))


# ---------------------------------------------------------------------------
# Teste 2: Pacote N2 (dados diarios)
# ---------------------------------------------------------------------------
def test_n2():
    print("\n--- Teste 2: Data-Notification N2 (dados diarios completos) ---")
    OBIS_VOL     = (7, 0,  3,  0,  0, 255)
    OBIS_FLOW    = (7, 0, 11,  0,  0, 255)
    OBIS_TEMP    = (7, 0, 41,  0,  0, 255)
    OBIS_STATUS  = (0, 0, 96,  5,  4, 255)
    OBIS_VALVE   = (0, 0, 96,  3, 10, 255)
    OBIS_RSRP    = (0, 0, 96, 12,  0, 255)
    OBIS_RSRQ    = (0, 0, 96, 12,  1, 255)
    OBIS_SINR    = (0, 0, 96, 12,  2, 255)
    OBIS_IMEI    = (0, 0, 96, 12,  3, 255)
    OBIS_SERIAL  = (0, 0, 96,  1,  0, 255)
    OBIS_FW      = (0, 0, 96,  2,  0, 255)

    pkt = build_data_notification(
        invoke_id=42,
        objects=[
            (OBIS_SERIAL, encode_visible_string("FAE00001234")),
            (OBIS_FW,     encode_visible_string("1.0.0")),
            (OBIS_VOL,    encode_uint32(120543)),   # 120.543 m3
            (OBIS_FLOW,   encode_uint32(3500)),     # 3.500 m3/h
            (OBIS_TEMP,   encode_int16(215)),       # 21.5 C
            (OBIS_STATUS, encode_uint32(0)),        # sem alarmes
            (OBIS_VALVE,  encode_uint8(0)),         # open (100%)
            (OBIS_RSRP,   encode_int16(-95)),       # -95 dBm
            (OBIS_RSRQ,   encode_int16(-12)),       # -12 dB
            (OBIS_SINR,   encode_int16(8)),         # 8 dB
            (OBIS_IMEI,   encode_visible_string("354668110000001")),
        ]
    )

    hdr, dec = decode_packet(pkt)
    check("WRAPPER valido",          hdr is not None)
    check("Tag Data-Notification",   dec and dec.tag == 0x0F)
    check("Invoke ID 42",            dec and dec.invoke_id == 42)
    check("11 objetos",              dec and len(dec.objects) == 11, str(len(dec.objects) if dec else 0))

    # Volume
    vol = next((o for o in dec.objects if o.obis == (7,0,3,0,0,255)), None)
    check("Vol raw = 120543",        vol and vol.raw == 120543)
    check("Vol scaled ~120.543 m3",  vol and abs(vol.scaled - 120.543) < 0.001,
          str(vol.scaled) if vol else "")

    # Temperatura
    temp = next((o for o in dec.objects if o.obis == (7,0,41,0,0,255)), None)
    check("Temp scaled = 21.5 C",    temp and abs(temp.scaled - 21.5) < 0.01,
          str(temp.scaled) if temp else "")

    # RSRP
    rsrp = next((o for o in dec.objects if o.obis == (0,0,96,12,0,255)), None)
    check("RSRP = -95",              rsrp and rsrp.raw == -95, str(rsrp.raw) if rsrp else "")

    # Status sem alarmes
    st = next((o for o in dec.objects if o.obis == (0,0,96,5,4,255)), None)
    check("Status = OK",             st and st.extra == "OK", st.extra if st else "")

    print(format_decoded(hdr, dec))


# ---------------------------------------------------------------------------
# Teste 3: Builders de comandos downstream
# ---------------------------------------------------------------------------
def test_downstream_builders():
    print("\n--- Teste 3: Builders de comandos downstream ---")

    # Valve command
    pkt = build_valve_command(invoke_id=10, position_pct=25)
    hdr, dec = decode_packet(pkt)
    check("Valve cmd: WRAPPER valido",   hdr is not None)
    check("Valve cmd: ACTION-Request",   dec and dec.tag == 0xC3, dec.tag_name if dec else "")
    check("Valve cmd: invoke_id=10",     dec and dec.invoke_id == 10)
    check("Valve cmd: OBIS 0.0.96.10.1.255",
          dec and any(o.obis == (0,0,96,10,1,255) for o in dec.objects))

    # FOTA start
    pkt = build_fota_start(invoke_id=20, file_size=22528, crc32=0xDEADBEEF)
    hdr, dec = decode_packet(pkt)
    check("FOTA cmd: ACTION-Request",    dec and dec.tag == 0xC3)
    check("FOTA cmd: invoke_id=20",      dec and dec.invoke_id == 20)
    check("FOTA cmd: OBIS 0.0.96.10.2.255",
          dec and any(o.obis == (0,0,96,10,2,255) for o in dec.objects))

    # GET-Request
    OBIS_VOL = (7, 0, 3, 0, 0, 255)
    pkt = build_get_request(invoke_id=30, obis=OBIS_VOL, class_id=1, attr_id=2)
    hdr, dec = decode_packet(pkt)
    check("GET-Request: valido",         dec and dec.tag == 0xC0)
    check("GET-Request: invoke_id=30",   dec and dec.invoke_id == 30)
    check("GET-Request: OBIS Vol",
          dec and any(o.obis == OBIS_VOL for o in dec.objects))


# ---------------------------------------------------------------------------
# Teste 4: Envio UDP contra o servidor (opcional, requer servidor rodando)
# ---------------------------------------------------------------------------
def test_udp_send(host: str = "127.0.0.1", port: int = 4059):
    print(f"\n--- Teste 4: Envio UDP para {host}:{port} ---")
    OBIS_SERIAL = (0, 0, 96,  1,  0, 255)
    OBIS_VOL    = (7, 0,  3,  0,  0, 255)
    OBIS_STATUS = (0, 0, 96,  5,  4, 255)

    pkt = build_data_notification(
        invoke_id=99,
        objects=[
            (OBIS_SERIAL, encode_visible_string("FAE_TEST_001")),
            (OBIS_VOL,    encode_uint32(55000)),   # 55.000 m3
            (OBIS_STATUS, encode_uint32(0)),
        ]
    )
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(2.0)
        s.sendto(pkt, (host, port))
        s.close()
        print(f"  Pacote N2 enviado ({len(pkt)} bytes) para {host}:{port}")
        print(f"  Verifique o dashboard em http://{host}:8080")
    except Exception as e:
        print(f"  Servidor nao acessivel ({e}) — pule este teste se o servidor nao estiver rodando")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    test_n1()
    test_n2()
    test_downstream_builders()

    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    test_udp_send(host)

    total  = len(_results)
    passed = sum(_results)
    print(f"\n{'='*40}")
    print(f"Resultado: {passed}/{total} testes passaram")
    if passed == total:
        print("Todos os testes passaram!")
    else:
        print("Alguns testes falharam. Veja os detalhes acima.")
    sys.exit(0 if passed == total else 1)

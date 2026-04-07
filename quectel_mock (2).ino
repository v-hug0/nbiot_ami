/*******************************************************************************
 * quectel_mock.ino — Mock do módulo Quectel BG77 / BC660K-GL para o ESP32.
 * v2: IP do servidor configurável via AT+AMICFG — sem recompilar ao
 *     trocar de ambiente local → Oracle Cloud.
 *
 * Comandos AT implementados:
 *   AT / ATE0 / ATI
 *   AT+CEREG=1          → OK + emite +CEREG: 1 após 300ms
 *   AT+CPSMS=...        → OK  (sem efeito)
 *   AT+QCFG=...         → OK  (sem efeito)
 *   AT+QIACT=...        → OK  (sem efeito)
 *   AT+QENG=...         → linha fake de KPIs NB-IoT
 *   AT+QICLOSE=<id>     → OK
 *   AT+QIOPEN=...       → OK + +QIOPEN: 0,0 (200ms depois)
 *   AT+QISEND=<id>,<N>  → ">" → recebe N bytes → UDP → "SEND OK"
 *   AT+QIRD=<id>,<max>  → +QIRD: <len>,"ip",port + bytes + OK
 *
 * Comando extra (não existe no Quectel real):
 *   AT+AMICFG="server","<ip>"  → grava IP do servidor (NVS, persiste)
 *   AT+AMICFG="port",<port>    → grava porta UDP
 *   AT+AMICFG?                 → mostra config atual
 *
 * URC automática quando dado UDP chega:
 *   +QIURC: "recv",0
 *
 * CONFIGURAÇÃO: edite só WIFI_SSID e WIFI_PASSWORD.
 * O IP do servidor é enviado pelo meter_simulator.py via AT+AMICFG.
 ******************************************************************************/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>

/* ── USER CONFIG ──────────────────────────────────────────── */
static const char*    WIFI_SSID         = "SUA_REDE_WIFI";
static const char*    WIFI_PASSWORD     = "SUA_SENHA_WIFI";
static const uint16_t DLMS_PORT         = 4059;
static const uint32_t AT_BAUD           = 115200;
static const uint32_t SEND_TIMEOUT_MS   = 10000;
/* ─────────────────────────────────────────────────────────── */

#define AT_LINE_BUF   256
#define UDP_RX_BUF    600
#define AT_TX_BUF     600
#define SERVER_IP_LEN  48

/* ── Configuração persistente (NVS) ──────────────────────── */
Preferences prefs;
static char     gs_serverIp[SERVER_IP_LEN] = "127.0.0.1";
static uint16_t gui_serverPort             = DLMS_PORT;

static void loadConfig() {
    prefs.begin("amicfg", true);
    String ip = prefs.getString("server_ip",  "127.0.0.1");
    gui_serverPort = prefs.getUShort("server_port", DLMS_PORT);
    prefs.end();
    strncpy(gs_serverIp, ip.c_str(), SERVER_IP_LEN - 1);
}

static void saveConfig() {
    prefs.begin("amicfg", false);
    prefs.putString("server_ip",   gs_serverIp);
    prefs.putUShort("server_port", gui_serverPort);
    prefs.end();
}

/* ── Estado global ───────────────────────────────────────── */
WiFiUDP udp;

static char     gs_lineBuf[AT_LINE_BUF];
static uint16_t gs_lineLen = 0;

typedef enum { STATE_AT, STATE_SEND_WAIT } E_STATE;
static E_STATE   ge_state       = STATE_AT;
static uint8_t   gau_txBuf[AT_TX_BUF];
static uint16_t  gui_txExpected = 0;
static uint16_t  gui_txReceived = 0;
static uint32_t  gul_txDeadline = 0;

static uint8_t   gau_rxBuf[UDP_RX_BUF];
static uint16_t  gui_rxLen  = 0;
static char      gs_rxIp[24]= {0};
static uint16_t  gui_rxPort = 0;

static bool     gb_urcPending    = false;
static bool     gb_socketOpen    = false;
static bool     gb_ceregPending  = false;
static uint32_t gul_ceregTime    = 0;
static bool     gb_qiopenPending = false;
static uint32_t gul_qiopenTime   = 0;

/* ── Helpers ─────────────────────────────────────────────── */
static void atSend(const char* s) {
    Serial.print("\r\n"); Serial.print(s); Serial.print("\r\n");
}
static void atOk()    { atSend("OK"); }
static void atError() { atSend("ERROR"); }

/* ── Handlers ───────────────────────────────────────────── */

static void handleAMICFG(const char* args) {
    /* AT+AMICFG?  ou  AT+AMICFG="field","value" */
    if (args[0] == '?') {
        char resp[80];
        snprintf(resp, sizeof(resp),
                 "+AMICFG: \"%s\",%u", gs_serverIp, gui_serverPort);
        atSend(resp);
        atOk();
        return;
    }
    if (args[0] != '=') { atError(); return; }
    const char* p = args + 1;

    /* Extrai campo entre aspas */
    char field[16] = {0};
    if (*p == '"') p++;
    int fi = 0;
    while (*p && *p != '"' && fi < 15) field[fi++] = *p++;
    if (*p == '"') p++;
    if (*p == ',') p++;

    if (strcasecmp(field, "server") == 0) {
        char ip[SERVER_IP_LEN] = {0};
        if (*p == '"') p++;
        int vi = 0;
        while (*p && *p != '"' && vi < SERVER_IP_LEN - 1) ip[vi++] = *p++;
        strncpy(gs_serverIp, ip, SERVER_IP_LEN - 1);
        saveConfig();
        Serial.print("[DBG] Servidor: "); Serial.println(gs_serverIp);
        atOk();
    } else if (strcasecmp(field, "port") == 0) {
        if (*p == '"') p++;
        gui_serverPort = (uint16_t)atoi(p);
        saveConfig();
        Serial.print("[DBG] Porta: "); Serial.println(gui_serverPort);
        atOk();
    } else {
        atError();
    }
}

static void handleQICLOSE(const char*) {
    if (gb_socketOpen) { udp.stop(); gb_socketOpen = false; }
    gb_urcPending = false;
    gui_rxLen     = 0;
    atOk();
}

static void handleQIOPEN(const char* args) {
    /*
     * AT+QIOPEN=1,<id>,"UDP","<host>",<rport>,<lport>,0
     * Lê o host dos args se válido; usa gs_serverIp como fallback.
     * Isso permite que udp_transport.c passe o IP sem AT+AMICFG.
     */
    char host[SERVER_IP_LEN] = {0};
    int  ctx = 0, sock_id = 0, rport = 0, lport = 0, mode = 0;
    sscanf(args, "%d,%d", &ctx, &sock_id);

    /* Extrai host: vai ao 4o campo (índice 3) */
    const char* p = args;
    for (int comma = 0; *p && comma < 3; p++) if (*p == ',') comma++;
    if (*p == '"') {
        p++;
        int hi = 0;
        while (*p && *p != '"' && hi < SERVER_IP_LEN - 1) host[hi++] = *p++;
        if (*p == '"') p++;
    }
    sscanf(p, ",%d,%d,%d", &rport, &lport, &mode);

    if (host[0] && strcmp(host, "0.0.0.0") != 0)
        strncpy(gs_serverIp, host, SERVER_IP_LEN - 1);
    if (rport > 0) gui_serverPort = (uint16_t)rport;

    if (gb_socketOpen) { udp.stop(); gb_socketOpen = false; }

    uint16_t local = (lport > 0) ? (uint16_t)lport : DLMS_PORT;
    if (!udp.begin(local)) {
        Serial.print("\r\n+QIOPEN: 0,1\r\n");
        return;
    }

    gb_socketOpen    = true;
    gb_urcPending    = false;
    gui_rxLen        = 0;
    atOk();
    gb_qiopenPending = true;
    gul_qiopenTime   = millis() + 200;

    Serial.print("[DBG] Socket → "); Serial.print(gs_serverIp);
    Serial.print(":"); Serial.println(gui_serverPort);
}

static void handleQISEND(const char* args) {
    if (!gb_socketOpen) { atError(); return; }
    int sock_id = 0, pay_len = 0;
    if (sscanf(args, "%d,%d", &sock_id, &pay_len) < 2
            || pay_len <= 0 || pay_len > AT_TX_BUF) {
        atError(); return;
    }
    gui_txExpected = (uint16_t)pay_len;
    gui_txReceived = 0;
    gul_txDeadline = millis() + SEND_TIMEOUT_MS;
    ge_state       = STATE_SEND_WAIT;
    Serial.print("\r\n>");
}

static void handleQIRD(const char* args) {
    if (!gb_socketOpen) { atError(); return; }
    int sock_id = 0, max_len = 0;
    sscanf(args, "%d,%d", &sock_id, &max_len);
    gb_urcPending = false;

    if (gui_rxLen == 0) {
        char hdr[72];
        snprintf(hdr, sizeof(hdr),
                 "+QIRD: 0,\"%s\",%u", gs_serverIp, gui_serverPort);
        atSend(hdr);
        atOk();
        return;
    }

    uint16_t n = (gui_rxLen < (uint16_t)max_len) ? gui_rxLen : (uint16_t)max_len;
    char hdr[72];
    snprintf(hdr, sizeof(hdr),
             "+QIRD: %u,\"%s\",%u", n, gs_rxIp, gui_rxPort);
    atSend(hdr);
    Serial.write(gau_rxBuf, n);
    Serial.print("\r\n");

    if (n < gui_rxLen) {
        memmove(gau_rxBuf, gau_rxBuf + n, gui_rxLen - n);
        gui_rxLen -= n;
    } else {
        gui_rxLen = 0;
    }
    atOk();
}

/* ── Parser de linha AT ──────────────────────────────────── */
static void processLine(const char* line) {
    if (!line[0]) return;
    Serial.print("[DBG] AT: "); Serial.println(line);

    if (strcmp(line, "AT")  == 0)              { atOk(); return; }
    if (strcmp(line, "ATE0") == 0)             { atOk(); return; }
    if (strcmp(line, "ATI")  == 0) {
        atSend("Quectel BG77 (ESP32 Mock) v2"); atOk(); return;
    }
    if (strncmp(line, "AT+AMICFG",  9) == 0)  { handleAMICFG(line+9);   return; }
    if (strncmp(line, "AT+QICLOSE=",11) == 0)  { handleQICLOSE(line+11); return; }
    if (strncmp(line, "AT+QIOPEN=", 10) == 0)  { handleQIOPEN(line+10);  return; }
    if (strncmp(line, "AT+QISEND=", 10) == 0)  { handleQISEND(line+10);  return; }
    if (strncmp(line, "AT+QIRD=",    8) == 0)  { handleQIRD(line+8);     return; }
    if (strncmp(line, "AT+QENG=",    8) == 0)  {
        atSend("+QENG: \"servingcell\",\"NOCONN\",\"NB-IoT\","
               "724,5,3A2B,12AB,0,28,-95,-12,8,-90,5,5,5,0,0,0,-");
        atOk(); return;
    }
    if (strncmp(line, "AT+CEREG=",  9) == 0) {
        atOk();
        gb_ceregPending = true;
        gul_ceregTime   = millis() + 300;
        return;
    }
    if (strncmp(line, "AT+", 3) == 0) { atOk(); return; }
    atError();
}

/* ── Leitura serial ─────────────────────────────────────── */
static void readAtSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            gs_lineBuf[gs_lineLen] = '\0';
            processLine(gs_lineBuf);
            gs_lineLen = 0;
            return;
        }
        if (gs_lineLen < AT_LINE_BUF - 1) gs_lineBuf[gs_lineLen++] = c;
    }
}

/* ── Payload binário QISEND ─────────────────────────────── */
static void collectPayload() {
    if (millis() > gul_txDeadline) {
        Serial.print("\r\nSEND FAIL\r\n");
        ge_state = STATE_AT; return;
    }
    while (Serial.available() && gui_txReceived < gui_txExpected)
        gau_txBuf[gui_txReceived++] = (uint8_t)Serial.read();
    if (gui_txReceived < gui_txExpected) return;

    udp.beginPacket(gs_serverIp, gui_serverPort);
    udp.write(gau_txBuf, gui_txExpected);
    bool ok = udp.endPacket();

    Serial.print(ok ? "\r\nSEND OK\r\n" : "\r\nSEND FAIL\r\n");
    Serial.print("[DBG] UDP "); Serial.print(ok ? "OK " : "FAIL ");
    Serial.print(gui_txExpected); Serial.print("B → ");
    Serial.print(gs_serverIp); Serial.print(":");
    Serial.println(gui_serverPort);

    ge_state = STATE_AT;
}

/* ── Poll UDP recepção ──────────────────────────────────── */
static void pollUdpRx() {
    if (!gb_socketOpen) return;
    int pkt = udp.parsePacket();
    if (pkt <= 0) return;
    uint16_t space = UDP_RX_BUF - gui_rxLen;
    if (pkt > (int)space) pkt = (int)space;
    int rd = udp.read(gau_rxBuf + gui_rxLen, pkt);
    if (rd <= 0) return;
    gui_rxLen += (uint16_t)rd;
    strncpy(gs_rxIp, udp.remoteIP().toString().c_str(), sizeof(gs_rxIp)-1);
    gui_rxPort = udp.remotePort();
    if (!gb_urcPending) gb_urcPending = true;
    Serial.print("[DBG] UDP recv "); Serial.print(rd);
    Serial.print("B from "); Serial.println(gs_rxIp);
}

/* ── URCs assíncronas ───────────────────────────────────── */
static void emitPendingUrcs() {
    if (ge_state != STATE_AT) return;
    if (gb_ceregPending && millis() >= gul_ceregTime) {
        gb_ceregPending = false;
        Serial.print("\r\n+CEREG: 1\r\n");
    }
    if (gb_qiopenPending && millis() >= gul_qiopenTime) {
        gb_qiopenPending = false;
        Serial.print("\r\n+QIOPEN: 0,0\r\n");
    }
    if (gb_urcPending && gui_rxLen > 0) {
        gb_urcPending = false;
        Serial.print("\r\n+QIURC: \"recv\",0\r\n");
    }
}

/* ── WiFi keepalive ─────────────────────────────────────── */
static void maintainWifi() {
    static uint32_t s_t = 0;
    if (millis() - s_t < 5000) return;
    s_t = millis();
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[DBG] WiFi perdido, reconectando...");
        WiFi.reconnect();
    }
}

/* ── setup / loop ───────────────────────────────────────── */
void setup() {
    Serial.begin(AT_BAUD);
    delay(100);
    loadConfig();

    Serial.println("\n=== Quectel Mock ESP32 v2 ===");
    Serial.print("[DBG] Servidor atual: ");
    Serial.print(gs_serverIp); Serial.print(":");
    Serial.println(gui_serverPort);
    Serial.println("[DBG] Para mudar o IP: AT+AMICFG=\"server\",\"<ip>\"");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[DBG] WiFi: "); Serial.println(WIFI_SSID);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(250); Serial.print(".");
        if (millis() - t0 > 20000) {
            Serial.println("\n[AVISO] WiFi timeout");
            break;
        }
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("\n[DBG] IP local: "); Serial.println(WiFi.localIP());
    }

    Serial.println("[DBG] Pronto.\n");
    delay(300);
    Serial.print("\r\n+QIND: ready\r\n");
}

void loop() {
    maintainWifi();
    if (ge_state == STATE_AT) {
        pollUdpRx();
        emitPendingUrcs();
        readAtSerial();
    } else {
        collectPayload();
    }
}

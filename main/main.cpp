/*versión multi-gateway para pruebas de ACK_DATA y RESULT. Se generan órdenes de prueba alternando ON y OFF, con COMMAND_ID persistente en NVS. 
detecta duplicados de DATA y reenvía la misma respuesta. Se publica en MQTT el payload recibido y se confirma RESULT por LoRa.
fiable 
*/

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Preferences.h>

#include <map>
#include <deque>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --------------------------------------------------
// IDENTIDAD DEL GATEWAY - MULTI-GATEWAY
// --------------------------------------------------

Preferences preferences;

const char* DEFAULT_GATEWAY_UID = "GW0001";

char gatewayUID[16];
char mqttClientID[32];

char topic_data[64];
char topic_actuadores[64];
char topic_respuesta[64];

// --------------------------------------------------
// CONFIGURACIÓN DE PRUEBA FASE 2
// --------------------------------------------------

// Alternancia: 0, 1, 2 y 3 órdenes.
uint8_t secuenciaRespuestas = 0;

// COMMAND_ID persistente en NVS.
uint32_t siguienteCommandId = 1000;


// --------------------------------------------------
// LOG
// --------------------------------------------------

static const char* TAG = "SCIGateway";

// --------------------------------------------------
// WIFI
// --------------------------------------------------

// Sustituye estos valores por los de tu instalación.
const char* ssid = "DIGIFIBRA-fAUx";
const char* password = "yGxhH7heZdQF";

// --------------------------------------------------
// MQTT
// --------------------------------------------------

const char* mqtt_server = "13.37.77.172";
const int mqtt_port = 1883;

// Sustituye por tus credenciales actuales.
const char* mqtt_user = "usuario";
const char* mqtt_password = "clave";

WiFiClient espClient;
PubSubClient client(espClient);

// --------------------------------------------------
// PROTOCOLO LORA GLOBALIOT - VERSION 1
// --------------------------------------------------

#define PROTOCOL_VERSION       0x01

#define MSG_DATA               0x01
#define MSG_ACK_DATA           0x02
#define MSG_ACK_DEF            0x03
#define MSG_COMMAND_STARTED    0x04
#define MSG_RESULT             0x05
#define MSG_ACK_RESULT         0x06
#define MSG_ACK_STARTED        0x07

#define LORA_HEADER_SIZE       8
#define LORA_MAX_PACKET        255
#define LORA_MAX_PAYLOAD       (LORA_MAX_PACKET - LORA_HEADER_SIZE)

const byte localAddress = 0x01;

// Acciones temporales de prueba.
#define ACTION_ACTUATOR_ON     0x01
#define ACTION_ACTUATOR_OFF    0x02

// Estados RESULT.
#define RESULT_EXECUTED        0x01
#define RESULT_FAILED          0x02

struct LoRaHeader {
    uint8_t version;
    uint8_t type;
    uint8_t src;
    uint8_t dst;
    uint16_t msgId;
    uint8_t flags;
    uint8_t length;
};

// --------------------------------------------------
// CACHE DE RESPUESTAS ACK_DATA
// --------------------------------------------------

struct CachedAck {
    uint16_t msgId;
    uint8_t length;
    uint8_t payload[LORA_MAX_PAYLOAD];
};

// Últimos ocho intercambios de cada nodo.
std::map<uint8_t, std::deque<CachedAck>> cacheACK;

const size_t MAX_INTERCAMBIOS_CACHE = 8;

// --------------------------------------------------
// PINES Y SENSORES
// --------------------------------------------------

const int csPin = 5;
const int resetPin = 14;
const int irqPin = 4;

#define LED       2
#define DHTpin    15
#define DHTTYPE   DHT22

DHT dht(DHTpin, DHTTYPE);

float temp;
float hum;

// --------------------------------------------------
// DEPURACIÓN
// --------------------------------------------------

unsigned long loopCounter = 0;

// --------------------------------------------------
// DECLARACIÓN DE FUNCIONES
// --------------------------------------------------

bool validarGatewayUID(const String& uid);
void cargarConfiguracionGateway();
void construirTopicsMQTT();
void procesarConfiguracionSerie();

void cargarContadorCommandId();
uint32_t nuevoCommandId();

bool buscarACKCache(uint8_t nodo, uint16_t msgId, CachedAck& resultado);
void registrarACKCache(uint8_t nodo, const CachedAck& respuesta);

void generarACKPrueba(uint8_t numeroOrdenes, CachedAck& respuesta);
bool enviarACKData(uint8_t destino, uint16_t msgId,
                   const uint8_t* payload, uint8_t payloadLength);

bool enviarACKResult(uint8_t destino, uint16_t msgId);

bool enviarDatosMQTT(const String& datos);
void procesarPaqueteLoRa();
void procesarResultado(const LoRaHeader& header, const uint8_t* payload);

void conectarWiFi();
void conectarMQTT();
void callbackMQTT(char* topic, byte* payload, unsigned int length);
void checkConexiones();

// --------------------------------------------------
// VALIDAR IDENTIFICADOR DEL GATEWAY
// --------------------------------------------------

bool validarGatewayUID(const String& uid) {

    if (uid.length() < 1 || uid.length() > 12) {
        return false;
    }

    for (size_t i = 0; i < uid.length(); i++) {

        char c = uid.charAt(i);

        bool valido =
            (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') ||
            c == '-' ||
            c == '_';

        if (!valido) {
            return false;
        }
    }

    return true;
}

// --------------------------------------------------
// CARGAR IDENTIFICADOR DESDE NVS
// --------------------------------------------------

void cargarConfiguracionGateway() {

    if (!preferences.begin("globaliot", false)) {

        ESP_LOGE(TAG, "No se pudo abrir la memoria NVS.");

        while (true) {
            delay(1000);
        }
    }

    String uidGuardado =
        preferences.getString("gateway_uid", "");

    if (uidGuardado.length() == 0) {

        uidGuardado = DEFAULT_GATEWAY_UID;

        preferences.putString("gateway_uid", uidGuardado);

        ESP_LOGW(TAG,
                 "No existia gateway_uid. Se asigna %s",
                 uidGuardado.c_str());
    }

    if (!validarGatewayUID(uidGuardado)) {

        ESP_LOGE(TAG,
                 "gateway_uid almacenado no valido: %s",
                 uidGuardado.c_str());

        while (true) {
            delay(1000);
        }
    }

    snprintf(
        gatewayUID,
        sizeof(gatewayUID),
        "%s",
        uidGuardado.c_str()
    );

    construirTopicsMQTT();

    ESP_LOGI(TAG, "Identidad del gateway: %s", gatewayUID);
}

// --------------------------------------------------
// CONSTRUIR TOPICS MQTT
// --------------------------------------------------

void construirTopicsMQTT() {

    snprintf(
        mqttClientID,
        sizeof(mqttClientID),
        "GlobalIoT_%s",
        gatewayUID
    );

    snprintf(
        topic_data,
        sizeof(topic_data),
        "gateway/%s/data",
        gatewayUID
    );

    snprintf(
        topic_actuadores,
        sizeof(topic_actuadores),
        "gateway/%s/actuadores",
        gatewayUID
    );

    snprintf(
        topic_respuesta,
        sizeof(topic_respuesta),
        "gateway/%s/respuesta",
        gatewayUID
    );

    ESP_LOGI(TAG, "MQTT Client ID: %s", mqttClientID);
    ESP_LOGI(TAG, "Topic DATA: %s", topic_data);
    ESP_LOGI(TAG, "Topic ACTUADORES: %s", topic_actuadores);
    ESP_LOGI(TAG, "Topic RESPUESTA: %s", topic_respuesta);
}

// --------------------------------------------------
// CONFIGURAR IDENTIDAD POR PUERTO SERIE
// --------------------------------------------------

void procesarConfiguracionSerie() {

    if (!Serial.available()) {
        return;
    }

    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (!comando.startsWith("GWUID=")) {
        return;
    }

    String nuevoUID = comando.substring(6);
    nuevoUID.trim();

    if (!validarGatewayUID(nuevoUID)) {

        ESP_LOGW(TAG,
                 "Identificador no valido. Usa 1-12 caracteres: A-Z, 0-9, - o _.");

        return;
    }

    if (nuevoUID == gatewayUID) {

        ESP_LOGI(TAG,
                 "El gateway ya tiene el identificador %s",
                 gatewayUID);

        return;
    }

    ESP_LOGI(TAG,
             "Cambiando gateway_uid de %s a %s",
             gatewayUID,
             nuevoUID.c_str());

    size_t escritos =
        preferences.putString("gateway_uid", nuevoUID);

    if (escritos == 0) {

        ESP_LOGE(TAG, "Error guardando gateway_uid en NVS.");

        return;
    }

    ESP_LOGI(TAG,
             "Identificador guardado. Reiniciando gateway...");

    Serial.flush();
    delay(500);

    ESP.restart();
}

// --------------------------------------------------
// CONTADOR COMMAND_ID
// --------------------------------------------------

void cargarContadorCommandId() {

    siguienteCommandId =
        preferences.getUInt("test_cmd_id", 1000);

    ESP_LOGI(TAG,
             "Siguiente COMMAND_ID de prueba: %lu",
             (unsigned long)siguienteCommandId);
}

uint32_t nuevoCommandId() {

    uint32_t id = siguienteCommandId;

    siguienteCommandId++;

    // Guardamos el siguiente valor para que no se reutilice
    // un COMMAND_ID después de reiniciar el gateway.
    preferences.putUInt("test_cmd_id", siguienteCommandId);

    return id;
}

// --------------------------------------------------
// CACHE DE RESPUESTAS ACK_DATA
// --------------------------------------------------

bool buscarACKCache(
    uint8_t nodo,
    uint16_t msgId,
    CachedAck& resultado
) {

    auto it = cacheACK.find(nodo);

    if (it == cacheACK.end()) {
        return false;
    }

    for (const CachedAck& entrada : it->second) {

        if (entrada.msgId == msgId) {

            resultado = entrada;

            return true;
        }
    }

    return false;
}

void registrarACKCache(
    uint8_t nodo,
    const CachedAck& respuesta
) {

    auto& cache = cacheACK[nodo];

    cache.push_back(respuesta);

    while (cache.size() > MAX_INTERCAMBIOS_CACHE) {
        cache.pop_front();
    }
}

// --------------------------------------------------
// GENERAR ACK_DATA DE PRUEBA
// --------------------------------------------------
//
// Formato:
//
// [NUM_ORDENES]
//
// Por cada orden:
//
// [COMMAND_ID: 4 bytes]
// [ACTION: 1 byte]
// [PARAM_LENGTH: 1 byte]
// [PARAMETERS: N bytes]
//
// COMMAND_ID y campos multibyte: Big Endian.
//
// --------------------------------------------------

void generarACKPrueba(
    uint8_t numeroOrdenes,
    CachedAck& respuesta
) {

    respuesta.length = 0;

    if (numeroOrdenes == 0) {
        // ACK_DATA vacío: payload de longitud cero.
        return;
    }

    uint8_t posicion = 0;

    respuesta.payload[posicion++] = numeroOrdenes;

    for (uint8_t i = 0; i < numeroOrdenes; i++) {

        uint32_t commandId = nuevoCommandId();

        // COMMAND_ID: 4 bytes Big Endian.
        respuesta.payload[posicion++] =
            (commandId >> 24) & 0xFF;

        respuesta.payload[posicion++] =
            (commandId >> 16) & 0xFF;

        respuesta.payload[posicion++] =
            (commandId >> 8) & 0xFF;

        respuesta.payload[posicion++] =
            commandId & 0xFF;

        // Alternamos ON y OFF.
        uint8_t action =
            (i % 2 == 0)
                ? ACTION_ACTUATOR_ON
                : ACTION_ACTUATOR_OFF;

        respuesta.payload[posicion++] = action;

        // Un byte de parámetros: número de actuador.
        respuesta.payload[posicion++] = 0x01;

        // Actuadores 1, 2 y 3.
        respuesta.payload[posicion++] = i + 1;

        ESP_LOGI(TAG,
                 "Orden de prueba: COMMAND_ID=%lu ACTION=0x%02X ACTUADOR=%u",
                 (unsigned long)commandId,
                 action,
                 i + 1);
    }

    respuesta.length = posicion;
}

// --------------------------------------------------
// ENVIAR ACK_DATA POR LORA
// --------------------------------------------------

bool enviarACKData(
    uint8_t destino,
    uint16_t msgId,
    const uint8_t* payload,
    uint8_t payloadLength
) {

    if (payloadLength > LORA_MAX_PAYLOAD) {
        return false;
    }

    uint8_t frame[LORA_MAX_PACKET];

    frame[0] = PROTOCOL_VERSION;
    frame[1] = MSG_ACK_DATA;
    frame[2] = localAddress;
    frame[3] = destino;

    frame[4] = (msgId >> 8) & 0xFF;
    frame[5] = msgId & 0xFF;

    frame[6] = 0x00;
    frame[7] = payloadLength;

    if (payloadLength > 0) {
        memcpy(
            frame + LORA_HEADER_SIZE,
            payload,
            payloadLength
        );
    }

    LoRa.beginPacket();

    LoRa.write(
        frame,
        LORA_HEADER_SIZE + payloadLength
    );

    int resultado = LoRa.endPacket();

    LoRa.receive();

    if (resultado == 0) {

        ESP_LOGE(TAG, "Error transmitiendo ACK_DATA.");

        return false;
    }

    ESP_LOGI(TAG,
             "ACK_DATA enviado a nodo %u, MSG_ID=%u, payload=%u bytes",
             destino,
             msgId,
             payloadLength);

    return true;
}

// --------------------------------------------------
// ENVIAR ACK_RESULT
// --------------------------------------------------

bool enviarACKResult(uint8_t destino, uint16_t msgId) {

    uint8_t frame[LORA_HEADER_SIZE];

    frame[0] = PROTOCOL_VERSION;
    frame[1] = MSG_ACK_RESULT;
    frame[2] = localAddress;
    frame[3] = destino;

    // ACK_RESULT identifica el RESULT mediante su MSG_ID.
    frame[4] = (msgId >> 8) & 0xFF;
    frame[5] = msgId & 0xFF;

    frame[6] = 0x00;
    frame[7] = 0x00;

    LoRa.beginPacket();

    LoRa.write(frame, LORA_HEADER_SIZE);

    int resultado = LoRa.endPacket();

    LoRa.receive();

    if (resultado == 0) {

        ESP_LOGE(TAG, "Error transmitiendo ACK_RESULT.");

        return false;
    }

    ESP_LOGI(TAG,
             "ACK_RESULT enviado a nodo %u, MSG_ID=%u",
             destino,
             msgId);

    return true;
}

// --------------------------------------------------
// PUBLICAR DATOS EN MQTT
// --------------------------------------------------

bool enviarDatosMQTT(const String& datos) {

    if (!client.connected()) {

        ESP_LOGW(TAG,
                 "MQTT desconectado. No se confirma DATA.");

        return false;
    }

    bool publicado = client.publish(
        topic_data,
        datos.c_str()
    );

    if (publicado) {

        ESP_LOGI(TAG,
                 "MQTT publicado: '%s' en '%s'",
                 datos.c_str(),
                 topic_data);

        return true;
    }

    ESP_LOGW(TAG, "Fallo al publicar en MQTT.");

    return false;
}

// --------------------------------------------------
// PROCESAR RESULT
// --------------------------------------------------

void procesarResultado(
    const LoRaHeader& header,
    const uint8_t* payload
) {

    if (header.src < 0x02 || header.src > 0xFE) {

        ESP_LOGW(TAG,
                 "RESULT de nodo no valido: %u",
                 header.src);

        return;
    }

    if (header.flags != 0x00 || header.length != 6) {

        ESP_LOGW(TAG,
                 "RESULT con estructura incorrecta.");

        return;
    }

    uint32_t commandId =
        ((uint32_t)payload[0] << 24) |
        ((uint32_t)payload[1] << 16) |
        ((uint32_t)payload[2] << 8) |
        payload[3];

    uint8_t status = payload[4];
    uint8_t errorCode = payload[5];

    ESP_LOGI(TAG,
             "RESULT recibido: nodo=%u MSG_ID=%u COMMAND_ID=%lu",
             header.src,
             header.msgId,
             (unsigned long)commandId);

    ESP_LOGI(TAG,
             "STATUS=%u ERROR_CODE=%u",
             status,
             errorCode);

    if (status == RESULT_EXECUTED) {

        ESP_LOGI(TAG,
                 "La orden %lu ha finalizado correctamente.",
                 (unsigned long)commandId);

    } else if (status == RESULT_FAILED) {

        ESP_LOGW(TAG,
                 "La orden %lu ha fallado.",
                 (unsigned long)commandId);

    } else {

        ESP_LOGW(TAG,
                 "STATUS desconocido en RESULT.");
    }

    // En esta fase de pruebas confirmamos el resultado
    // por LoRa. La publicación MQTT de estados se hará
    // durante la integración con el backend.
    enviarACKResult(header.src, header.msgId);
}

// --------------------------------------------------
// PROCESAR PAQUETE LORA
// --------------------------------------------------

void procesarPaqueteLoRa() {

    int packetSize = LoRa.parsePacket();

    if (packetSize == 0) {
        return;
    }

    if (packetSize < LORA_HEADER_SIZE ||
        packetSize > LORA_MAX_PACKET) {

        ESP_LOGW(TAG,
                 "Tamaño de paquete no valido: %d",
                 packetSize);

        while (LoRa.available()) {
            LoRa.read();
        }

        return;
    }

    uint8_t frame[LORA_MAX_PACKET];

    int bytesLeidos = 0;

    while (LoRa.available() &&
           bytesLeidos < LORA_MAX_PACKET) {

        frame[bytesLeidos++] = LoRa.read();
    }

    if (bytesLeidos != packetSize) {

        ESP_LOGW(TAG, "Longitud de paquete incorrecta");

        return;
    }

    // ---------------------------------------------
    // EXTRAER CABECERA
    // ---------------------------------------------

    LoRaHeader header;

    header.version = frame[0];
    header.type    = frame[1];
    header.src     = frame[2];
    header.dst     = frame[3];

    header.msgId =
        ((uint16_t)frame[4] << 8) | frame[5];

    header.flags  = frame[6];
    header.length = frame[7];

    // ---------------------------------------------
    // VALIDAR CABECERA
    // ---------------------------------------------

    if (header.version != PROTOCOL_VERSION) {

        ESP_LOGW(TAG,
                 "Version no soportada: %u",
                 header.version);

        return;
    }

    if (header.dst != localAddress) {

        ESP_LOGW(TAG,
                 "Paquete destinado a otro dispositivo");

        return;
    }

    if (header.length != packetSize - LORA_HEADER_SIZE) {

        ESP_LOGW(TAG,
                 "Longitud de payload incorrecta");

        return;
    }

    if (header.length > LORA_MAX_PAYLOAD) {

        ESP_LOGW(TAG, "Payload demasiado grande");

        return;
    }

    const uint8_t* payload = frame + LORA_HEADER_SIZE;

    // ---------------------------------------------
    // PROCESAR DATA
    // ---------------------------------------------

    if (header.type == MSG_DATA) {

        if (header.src < 0x02 ||
            header.src > 0xFE) {

            ESP_LOGW(TAG,
                     "Direccion de nodo no valida: %u",
                     header.src);

            return;
        }

        if (header.flags != 0x00) {

            ESP_LOGW(TAG,
                     "FLAGS no soportados en DATA: %u",
                     header.flags);

            return;
        }

        if (header.length == 0) {

            ESP_LOGW(TAG, "DATA sin payload.");

            return;
        }

        // -----------------------------------------
        // COMPROBAR SI YA TENEMOS SU ACK_DATA
        // -----------------------------------------

        CachedAck respuestaGuardada;

        if (buscarACKCache(
                header.src,
                header.msgId,
                respuestaGuardada
            )) {

            ESP_LOGW(TAG,
                     "DATA duplicado: nodo=%u MSG_ID=%u",
                     header.src,
                     header.msgId);

            // Reenviamos exactamente la misma respuesta.
            enviarACKData(
                header.src,
                header.msgId,
                respuestaGuardada.payload,
                respuestaGuardada.length
            );

            return;
        }

        // -----------------------------------------
        // EXTRAER PAYLOAD
        // -----------------------------------------

        String payloadString = "";

        for (int i = 0; i < header.length; i++) {
            payloadString += (char)payload[i];
        }

        int rssi = LoRa.packetRssi();
        float snr = LoRa.packetSnr();

        ESP_LOGI(TAG,
                 "DATA recibido: nodo=%u MSG_ID=%u",
                 header.src,
                 header.msgId);

        ESP_LOGI(TAG,
                 "Payload: %s",
                 payloadString.c_str());

        ESP_LOGI(TAG,
                 "RSSI: %d dBm, SNR: %.2f dB",
                 rssi,
                 snr);

        // -----------------------------------------
        // CONSTRUIR MENSAJE MQTT
        // -----------------------------------------

        String datosMQTT =
            String(header.src) + "," + payloadString;

        // -----------------------------------------
        // PUBLICAR MQTT
        // -----------------------------------------

        if (!enviarDatosMQTT(datosMQTT)) {

            ESP_LOGW(TAG,
                     "No se confirma DATA. El nodo podra reintentarlo.");

            return;
        }

        // -----------------------------------------
        // GENERAR RESPUESTA DE PRUEBA
        // -----------------------------------------

        CachedAck respuesta;

        respuesta.msgId = header.msgId;

        uint8_t numeroOrdenes =
            secuenciaRespuestas % 4;

        secuenciaRespuestas++;

        generarACKPrueba(
            numeroOrdenes,
            respuesta
        );

        // Guardamos antes de transmitir.
        // Si DATA se retransmite, no generaremos
        // nuevos COMMAND_ID.
        registrarACKCache(
            header.src,
            respuesta
        );

        // -----------------------------------------
        // ENVIAR ACK_DATA
        // -----------------------------------------

        enviarACKData(
            header.src,
            header.msgId,
            respuesta.payload,
            respuesta.length
        );

        ESP_LOGI(TAG,
                 "DATA procesado y confirmado: nodo=%u MSG_ID=%u",
                 header.src,
                 header.msgId);
    }

    // ---------------------------------------------
    // PROCESAR RESULT
    // ---------------------------------------------

    else if (header.type == MSG_RESULT) {

        procesarResultado(header, payload);
    }

    else {

        ESP_LOGW(TAG,
                 "Tipo de mensaje no implementado en esta fase: 0x%02X",
                 header.type);
    }
}

// --------------------------------------------------
// CONECTAR WIFI
// --------------------------------------------------

void conectarWiFi() {

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    WiFi.begin(ssid, password);

    ESP_LOGI(TAG, "Conectando a WiFi...");

    uint32_t start = millis();

    while (WiFi.status() != WL_CONNECTED) {

        delay(250);

        if (millis() - start > 20000) {

            ESP_LOGW(TAG,
                     "WiFi timeout, reintentando...");

            start = millis();

            WiFi.disconnect();
            WiFi.begin(ssid, password);
        }
    }

    ESP_LOGI(TAG,
             "WiFi conectado con IP: %s",
             WiFi.localIP().toString().c_str());
}

// --------------------------------------------------
// CALLBACK MQTT
// --------------------------------------------------

void callbackMQTT(
    char* topic,
    byte* payload,
    unsigned int length
) {

    ESP_LOGI(TAG,
             "Mensaje recibido en topic: %s",
             topic);

    String mensaje = "";

    for (unsigned int i = 0; i < length; i++) {
        mensaje += (char)payload[i];
    }

    ESP_LOGI(TAG,
             "Payload MQTT recibido: '%s'",
             mensaje.c_str());

    // Se mantiene temporalmente el formato JSON anterior.
    // La recepción y conversión de órdenes MQTT a LoRa
    // se implementará al integrar el backend.

    int nodo_id = -1;
    int estado = -1;

    int resultado = sscanf(
        mensaje.c_str(),
        "{\"nodo\":%d,\"estado\":%d}",
        &nodo_id,
        &estado
    );

    if (resultado == 2) {

        ESP_LOGI(TAG,
                 "Orden recibida: nodo=%d, estado=%d",
                 nodo_id,
                 estado);

    } else {

        ESP_LOGW(TAG,
                 "Formato JSON de orden no reconocido.");
    }
}

// --------------------------------------------------
// CONECTAR MQTT
// --------------------------------------------------

void conectarMQTT() {

    client.setKeepAlive(30);
    client.setSocketTimeout(10);

    while (!client.connected()) {

        if (WiFi.status() != WL_CONNECTED) {

            ESP_LOGW(TAG,
                     "No se puede conectar a MQTT: WiFi desconectada.");

            conectarWiFi();
        }

        ESP_LOGI(TAG,
                 "Conectando a MQTT como %s...",
                 mqttClientID);

        if (client.connect(
                mqttClientID,
                mqtt_user,
                mqtt_password
            )) {

            ESP_LOGI(TAG, "Conectado a MQTT.");

            if (client.subscribe(topic_actuadores)) {

                ESP_LOGI(TAG,
                         "Suscrito a: %s",
                         topic_actuadores);

            } else {

                ESP_LOGE(TAG,
                         "Error suscribiendo a %s",
                         topic_actuadores);
            }

        } else {

            ESP_LOGE(TAG,
                     "Error conectando a MQTT, rc=%d",
                     client.state());

            delay(3000);
        }
    }
}

// --------------------------------------------------
// COMPROBAR CONEXIONES
// --------------------------------------------------

void checkConexiones() {

    if (WiFi.status() != WL_CONNECTED) {

        ESP_LOGW(TAG,
                 "WiFi desconectada, reconectando...");

        conectarWiFi();
    }

    if (!client.connected()) {

        ESP_LOGW(TAG,
                 "MQTT desconectado, reconectando...");

        conectarMQTT();
    }
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------

void setup() {

    Serial.begin(115200);
    Serial.setTimeout(50);

    delay(1000);

    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG,
             "GLOBALIoT - Gateway LoRa v1.0 Multi-Gateway Fase 2");

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    // ---------------------------------------------
    // IDENTIDAD DEL GATEWAY
    // ---------------------------------------------

    cargarConfiguracionGateway();

    // ---------------------------------------------
    // CONTADOR DE COMMAND_ID
    // ---------------------------------------------

    cargarContadorCommandId();

    // ---------------------------------------------
    // SENSOR DHT
    // ---------------------------------------------

    dht.begin();

    // ---------------------------------------------
    // INICIALIZAR LORA
    // ---------------------------------------------

    LoRa.setPins(csPin, resetPin, irqPin);

    if (!LoRa.begin(868E6)) {

        ESP_LOGE(TAG, "Fallo al iniciar LoRa");

        while (true) {
            delay(1000);
        }
    }

    LoRa.setSyncWord(0x33);
    LoRa.enableCrc();
    LoRa.receive();

    ESP_LOGI(TAG,
             "LoRa iniciado. SyncWord=0x33");

    // ---------------------------------------------
    // WIFI Y MQTT
    // ---------------------------------------------

    conectarWiFi();

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callbackMQTT);

    conectarMQTT();

    ESP_LOGI(TAG,
             "Gateway %s preparado.",
             gatewayUID);
}

// --------------------------------------------------
// LOOP
// --------------------------------------------------

void loop() {

    procesarConfiguracionSerie();

    procesarPaqueteLoRa();

    client.loop();

    loopCounter++;

    if (loopCounter % 1000 == 0) {

        ESP_LOGI(TAG,
                 "[CICLO %lu] Heap libre: %u, WiFi: %s, MQTT: %s",
                 loopCounter,
                 esp_get_free_heap_size(),
                 WiFi.status() == WL_CONNECTED
                    ? "OK"
                    : "DESCONECTADO",
                 client.connected()
                    ? "CONECTADO"
                    : "DESCONECTADO");

        checkConexiones();
    }

    delay(10);
}

// --------------------------------------------------
// APP MAIN - ESP-IDF + ARDUINO COMO COMPONENTE
// --------------------------------------------------

extern "C" void app_main() {

    initArduino();

    setup();

    while (true) {

        loop();

        delay(1);
    }
}


/*versión muy verbose para ver por qué hay a veces reintentos. sin embargo no detecta duplicados

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Preferences.h>

#include <map>
#include <deque>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --------------------------------------------------
// IDENTIDAD DEL GATEWAY - MULTI-GATEWAY
// --------------------------------------------------

Preferences preferences;

const char* DEFAULT_GATEWAY_UID = "GW0001";

char gatewayUID[16];
char mqttClientID[32];

char topic_data[64];
char topic_actuadores[64];
char topic_respuesta[64];

// --------------------------------------------------
// CONFIGURACIÓN DE PRUEBA FASE 2
// --------------------------------------------------

// Alternancia: 0, 1, 2 y 3 órdenes.
uint8_t secuenciaRespuestas = 0;

// COMMAND_ID persistente en NVS.
uint32_t siguienteCommandId = 1000;


// --------------------------------------------------
// LOG
// --------------------------------------------------

static const char* TAG = "SCIGateway";

// --------------------------------------------------
// WIFI
// --------------------------------------------------

// Sustituye estos valores por los de tu instalación.
const char* ssid = "DIGIFIBRA-fAUx";
const char* password = "yGxhH7heZdQF";

// --------------------------------------------------
// MQTT
// --------------------------------------------------

const char* mqtt_server = "13.37.77.172";
const int mqtt_port = 1883;

// Sustituye por tus credenciales actuales.
const char* mqtt_user = "usuario";
const char* mqtt_password = "clave";

WiFiClient espClient;
PubSubClient client(espClient);

// --------------------------------------------------
// PROTOCOLO LORA GLOBALIOT - VERSION 1
// --------------------------------------------------

#define PROTOCOL_VERSION       0x01

#define MSG_DATA               0x01
#define MSG_ACK_DATA           0x02
#define MSG_ACK_DEF            0x03
#define MSG_COMMAND_STARTED    0x04
#define MSG_RESULT             0x05
#define MSG_ACK_RESULT         0x06
#define MSG_ACK_STARTED        0x07

#define LORA_HEADER_SIZE       8
#define LORA_MAX_PACKET        255
#define LORA_MAX_PAYLOAD       (LORA_MAX_PACKET - LORA_HEADER_SIZE)

// --------------------------------------------------
// CACHE DE RESPUESTAS ACK_DATA
// --------------------------------------------------

struct CachedAck {
    uint16_t msgId;
    uint8_t length;
    uint8_t payload[LORA_MAX_PAYLOAD];
};

// Últimos ocho intercambios de cada nodo.
std::map<uint8_t, std::deque<CachedAck>> cacheACK;

const size_t MAX_INTERCAMBIOS_CACHE = 8;

const byte localAddress = 0x01;

// Acciones temporales de prueba.
#define ACTION_ACTUATOR_ON     0x01
#define ACTION_ACTUATOR_OFF    0x02

// Estados RESULT.
#define RESULT_EXECUTED        0x01
#define RESULT_FAILED          0x02

struct LoRaHeader {
    uint8_t version;
    uint8_t type;
    uint8_t src;
    uint8_t dst;
    uint16_t msgId;
    uint8_t flags;
    uint8_t length;
};

// --------------------------------------------------
// PINES Y SENSORES
// --------------------------------------------------

const int csPin = 5;
const int resetPin = 14;
const int irqPin = 4;

#define LED       2
#define DHTpin    15
#define DHTTYPE   DHT22

DHT dht(DHTpin, DHTTYPE);

float temp;
float hum;

// --------------------------------------------------
// DEPURACIÓN
// --------------------------------------------------

unsigned long loopCounter = 0;

// --------------------------------------------------
// DIAGNÓSTICO DE RECEPCIÓN LORA
// --------------------------------------------------

uint32_t ultimoPaqueteLoRaMs = 0;
uint32_t contadorPaquetesLoRa = 0;

// --------------------------------------------------
// DECLARACIÓN DE FUNCIONES
// --------------------------------------------------

bool validarGatewayUID(const String& uid);
void cargarConfiguracionGateway();
void construirTopicsMQTT();
void procesarConfiguracionSerie();

void cargarContadorCommandId();
uint32_t nuevoCommandId();

bool buscarACKCache(uint8_t nodo, uint16_t msgId, CachedAck& resultado);
void registrarACKCache(uint8_t nodo, const CachedAck& respuesta);

void generarACKPrueba(uint8_t numeroOrdenes, CachedAck& respuesta);
bool enviarACKData(uint8_t destino, uint16_t msgId,
                   const uint8_t* payload, uint8_t payloadLength);

bool enviarACKResult(uint8_t destino, uint16_t msgId);

bool enviarDatosMQTT(const String& datos);
void procesarPaqueteLoRa();
void procesarResultado(const LoRaHeader& header, const uint8_t* payload);

void conectarWiFi();
void conectarMQTT();
void callbackMQTT(char* topic, byte* payload, unsigned int length);
void checkConexiones();

// --------------------------------------------------
// VALIDAR IDENTIFICADOR DEL GATEWAY
// --------------------------------------------------

bool validarGatewayUID(const String& uid) {

    if (uid.length() < 1 || uid.length() > 12) {
        return false;
    }

    for (size_t i = 0; i < uid.length(); i++) {

        char c = uid.charAt(i);

        bool valido =
            (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') ||
            c == '-' ||
            c == '_';

        if (!valido) {
            return false;
        }
    }

    return true;
}

// --------------------------------------------------
// CARGAR IDENTIFICADOR DESDE NVS
// --------------------------------------------------

void cargarConfiguracionGateway() {

    if (!preferences.begin("globaliot", false)) {

        ESP_LOGE(TAG, "No se pudo abrir la memoria NVS.");

        while (true) {
            delay(1000);
        }
    }

    String uidGuardado =
        preferences.getString("gateway_uid", "");

    if (uidGuardado.length() == 0) {

        uidGuardado = DEFAULT_GATEWAY_UID;

        preferences.putString("gateway_uid", uidGuardado);

        ESP_LOGW(TAG,
                 "No existia gateway_uid. Se asigna %s",
                 uidGuardado.c_str());
    }

    if (!validarGatewayUID(uidGuardado)) {

        ESP_LOGE(TAG,
                 "gateway_uid almacenado no valido: %s",
                 uidGuardado.c_str());

        while (true) {
            delay(1000);
        }
    }

    snprintf(
        gatewayUID,
        sizeof(gatewayUID),
        "%s",
        uidGuardado.c_str()
    );

    construirTopicsMQTT();

    ESP_LOGI(TAG, "Identidad del gateway: %s", gatewayUID);
}

// --------------------------------------------------
// CONSTRUIR TOPICS MQTT
// --------------------------------------------------

void construirTopicsMQTT() {

    snprintf(
        mqttClientID,
        sizeof(mqttClientID),
        "GlobalIoT_%s",
        gatewayUID
    );

    snprintf(
        topic_data,
        sizeof(topic_data),
        "gateway/%s/data",
        gatewayUID
    );

    snprintf(
        topic_actuadores,
        sizeof(topic_actuadores),
        "gateway/%s/actuadores",
        gatewayUID
    );

    snprintf(
        topic_respuesta,
        sizeof(topic_respuesta),
        "gateway/%s/respuesta",
        gatewayUID
    );

    ESP_LOGI(TAG, "MQTT Client ID: %s", mqttClientID);
    ESP_LOGI(TAG, "Topic DATA: %s", topic_data);
    ESP_LOGI(TAG, "Topic ACTUADORES: %s", topic_actuadores);
    ESP_LOGI(TAG, "Topic RESPUESTA: %s", topic_respuesta);
}

// --------------------------------------------------
// CONFIGURAR IDENTIDAD POR PUERTO SERIE
// --------------------------------------------------

void procesarConfiguracionSerie() {

    if (!Serial.available()) {
        return;
    }

    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (!comando.startsWith("GWUID=")) {
        return;
    }

    String nuevoUID = comando.substring(6);
    nuevoUID.trim();

    if (!validarGatewayUID(nuevoUID)) {

        ESP_LOGW(TAG,
                 "Identificador no valido. Usa 1-12 caracteres: A-Z, 0-9, - o _.");

        return;
    }

    if (nuevoUID == gatewayUID) {

        ESP_LOGI(TAG,
                 "El gateway ya tiene el identificador %s",
                 gatewayUID);

        return;
    }

    ESP_LOGI(TAG,
             "Cambiando gateway_uid de %s a %s",
             gatewayUID,
             nuevoUID.c_str());

    size_t escritos =
        preferences.putString("gateway_uid", nuevoUID);

    if (escritos == 0) {

        ESP_LOGE(TAG, "Error guardando gateway_uid en NVS.");

        return;
    }

    ESP_LOGI(TAG,
             "Identificador guardado. Reiniciando gateway...");

    Serial.flush();
    delay(500);

    ESP.restart();
}

// --------------------------------------------------
// CONTADOR COMMAND_ID
// --------------------------------------------------

void cargarContadorCommandId() {

    siguienteCommandId =
        preferences.getUInt("test_cmd_id", 1000);

    ESP_LOGI(TAG,
             "Siguiente COMMAND_ID de prueba: %lu",
             (unsigned long)siguienteCommandId);
}

uint32_t nuevoCommandId() {

    uint32_t id = siguienteCommandId;

    siguienteCommandId++;

    // Guardamos el siguiente valor para que no se reutilice
    // un COMMAND_ID después de reiniciar el gateway.
    preferences.putUInt("test_cmd_id", siguienteCommandId);

    return id;
}

// --------------------------------------------------
// CACHE DE RESPUESTAS ACK_DATA
// --------------------------------------------------

bool buscarACKCache(
    uint8_t nodo,
    uint16_t msgId,
    CachedAck& resultado
) {

    auto it = cacheACK.find(nodo);

    if (it == cacheACK.end()) {
        return false;
    }

    for (const CachedAck& entrada : it->second) {

        if (entrada.msgId == msgId) {

            resultado = entrada;

            return true;
        }
    }

    return false;
}

void registrarACKCache(
    uint8_t nodo,
    const CachedAck& respuesta
) {

    auto& cache = cacheACK[nodo];

    cache.push_back(respuesta);

    while (cache.size() > MAX_INTERCAMBIOS_CACHE) {
        cache.pop_front();
    }
}

// --------------------------------------------------
// GENERAR ACK_DATA DE PRUEBA
// --------------------------------------------------
//
// Formato:
//
// [NUM_ORDENES]
//
// Por cada orden:
//
// [COMMAND_ID: 4 bytes]
// [ACTION: 1 byte]
// [PARAM_LENGTH: 1 byte]
// [PARAMETERS: N bytes]
//
// COMMAND_ID y campos multibyte: Big Endian.
//
// --------------------------------------------------

void generarACKPrueba(
    uint8_t numeroOrdenes,
    CachedAck& respuesta
) {

    respuesta.length = 0;

    if (numeroOrdenes == 0) {
        // ACK_DATA vacío: payload de longitud cero.
        return;
    }

    uint8_t posicion = 0;

    respuesta.payload[posicion++] = numeroOrdenes;

    for (uint8_t i = 0; i < numeroOrdenes; i++) {

        uint32_t commandId = nuevoCommandId();

        // COMMAND_ID: 4 bytes Big Endian.
        respuesta.payload[posicion++] =
            (commandId >> 24) & 0xFF;

        respuesta.payload[posicion++] =
            (commandId >> 16) & 0xFF;

        respuesta.payload[posicion++] =
            (commandId >> 8) & 0xFF;

        respuesta.payload[posicion++] =
            commandId & 0xFF;

        // Alternamos ON y OFF.
        uint8_t action =
            (i % 2 == 0)
                ? ACTION_ACTUATOR_ON
                : ACTION_ACTUATOR_OFF;

        respuesta.payload[posicion++] = action;

        // Un byte de parámetros: número de actuador.
        respuesta.payload[posicion++] = 0x01;

        // Actuadores 1, 2 y 3.
        respuesta.payload[posicion++] = i + 1;

        ESP_LOGI(TAG,
                 "Orden de prueba: COMMAND_ID=%lu ACTION=0x%02X ACTUADOR=%u",
                 (unsigned long)commandId,
                 action,
                 i + 1);
    }

    respuesta.length = posicion;
}

// --------------------------------------------------
// ENVIAR ACK_DATA POR LORA
// --------------------------------------------------

bool enviarACKData(
    uint8_t destino,
    uint16_t msgId,
    const uint8_t* payload,
    uint8_t payloadLength
) {

    if (payloadLength > LORA_MAX_PAYLOAD) {
        return false;
    }

    uint8_t frame[LORA_MAX_PACKET];

    frame[0] = PROTOCOL_VERSION;
    frame[1] = MSG_ACK_DATA;
    frame[2] = localAddress;
    frame[3] = destino;

    frame[4] = (msgId >> 8) & 0xFF;
    frame[5] = msgId & 0xFF;

    frame[6] = 0x00;
    frame[7] = payloadLength;

    if (payloadLength > 0) {
        memcpy(
            frame + LORA_HEADER_SIZE,
            payload,
            payloadLength
        );
    }

    LoRa.beginPacket();

    LoRa.write(
        frame,
        LORA_HEADER_SIZE + payloadLength
    );

    uint32_t inicioTX = millis();

    int resultado = LoRa.endPacket();

    uint32_t finTX = millis();

    LoRa.receive();

    ESP_LOGI(TAG,
            "[TX ACK_DATA] Nodo=%u MSG_ID=%u | Duracion=%lu ms | Resultado=%d",
            destino,
            msgId,
            (unsigned long)(finTX - inicioTX),
            resultado);

    if (resultado == 0) {

        ESP_LOGE(TAG, "Error transmitiendo ACK_DATA.");

        return false;
    }

    ESP_LOGI(TAG,
             "ACK_DATA enviado a nodo %u, MSG_ID=%u, payload=%u bytes",
             destino,
             msgId,
             payloadLength);

    return true;
}

// --------------------------------------------------
// ENVIAR ACK_RESULT
// --------------------------------------------------

bool enviarACKResult(uint8_t destino, uint16_t msgId) {

    uint8_t frame[LORA_HEADER_SIZE];

    frame[0] = PROTOCOL_VERSION;
    frame[1] = MSG_ACK_RESULT;
    frame[2] = localAddress;
    frame[3] = destino;

    // ACK_RESULT identifica el RESULT mediante su MSG_ID.
    frame[4] = (msgId >> 8) & 0xFF;
    frame[5] = msgId & 0xFF;

    frame[6] = 0x00;
    frame[7] = 0x00;

    LoRa.beginPacket();

    LoRa.write(frame, LORA_HEADER_SIZE);

    uint32_t inicioTX = millis();

    int resultado = LoRa.endPacket();

    uint32_t finTX = millis();

    LoRa.receive();

    ESP_LOGI(TAG,
            "[TX ACK_RESULT] Nodo=%u MSG_ID=%u | Duracion=%lu ms | Resultado=%d",
            destino,
            msgId,
            (unsigned long)(finTX - inicioTX),
            resultado);

    if (resultado == 0) {

        ESP_LOGE(TAG, "Error transmitiendo ACK_RESULT.");

        return false;
    }

    ESP_LOGI(TAG,
             "ACK_RESULT enviado a nodo %u, MSG_ID=%u",
             destino,
             msgId);

    return true;
}

// --------------------------------------------------
// PUBLICAR DATOS EN MQTT
// --------------------------------------------------

bool enviarDatosMQTT(const String& datos) {

    if (!client.connected()) {

        ESP_LOGW(TAG,
                 "MQTT desconectado. No se confirma DATA.");

        return false;
    }

    bool publicado = client.publish(
        topic_data,
        datos.c_str()
    );

    if (publicado) {

        ESP_LOGI(TAG,
                 "MQTT publicado: '%s' en '%s'",
                 datos.c_str(),
                 topic_data);

        return true;
    }

    ESP_LOGW(TAG, "Fallo al publicar en MQTT.");

    return false;
}

// --------------------------------------------------
// PROCESAR RESULT
// --------------------------------------------------

void procesarResultado(
    const LoRaHeader& header,
    const uint8_t* payload
) {

    if (header.src < 0x02 || header.src > 0xFE) {

        ESP_LOGW(TAG,
                 "RESULT de nodo no valido: %u",
                 header.src);

        return;
    }

    if (header.flags != 0x00 || header.length != 6) {

        ESP_LOGW(TAG,
                 "RESULT con estructura incorrecta.");

        return;
    }

    uint32_t commandId =
        ((uint32_t)payload[0] << 24) |
        ((uint32_t)payload[1] << 16) |
        ((uint32_t)payload[2] << 8) |
        payload[3];

    uint8_t status = payload[4];
    uint8_t errorCode = payload[5];

    ESP_LOGI(TAG,
             "RESULT recibido: nodo=%u MSG_ID=%u COMMAND_ID=%lu",
             header.src,
             header.msgId,
             (unsigned long)commandId);

    ESP_LOGI(TAG,
             "STATUS=%u ERROR_CODE=%u",
             status,
             errorCode);

    if (status == RESULT_EXECUTED) {

        ESP_LOGI(TAG,
                 "La orden %lu ha finalizado correctamente.",
                 (unsigned long)commandId);

    } else if (status == RESULT_FAILED) {

        ESP_LOGW(TAG,
                 "La orden %lu ha fallado.",
                 (unsigned long)commandId);

    } else {

        ESP_LOGW(TAG,
                 "STATUS desconocido en RESULT.");
    }

    // En esta fase de pruebas confirmamos el resultado
    // por LoRa. La publicación MQTT de estados se hará
    // durante la integración con el backend.
    enviarACKResult(header.src, header.msgId);
}

// --------------------------------------------------
// PROCESAR PAQUETE LORA
// --------------------------------------------------

void procesarPaqueteLoRa() {

    int packetSize = LoRa.parsePacket();

    if (packetSize == 0) {
        return;
    }

    // ---------------------------------------------
    // DIAGNÓSTICO DE RECEPCIÓN
    // ---------------------------------------------

    uint32_t instanteRecepcion = millis();

    int rssiRecibido = LoRa.packetRssi();
    float snrRecibido = LoRa.packetSnr();

    uint32_t tiempoDesdeUltimoPaquete = 0;

    if (ultimoPaqueteLoRaMs != 0) {
        tiempoDesdeUltimoPaquete =
            instanteRecepcion - ultimoPaqueteLoRaMs;
    }

    ultimoPaqueteLoRaMs = instanteRecepcion;
    contadorPaquetesLoRa++;

    ESP_LOGI(TAG,
             "[RX #%lu] Paquete detectado: %d bytes | RSSI=%d dBm | SNR=%.2f dB",
             (unsigned long)contadorPaquetesLoRa,
             packetSize,
             rssiRecibido,
             snrRecibido);

    if (contadorPaquetesLoRa > 1) {
        ESP_LOGI(TAG,
                 "[RX] Tiempo desde el paquete anterior: %lu ms",
                 (unsigned long)tiempoDesdeUltimoPaquete);
    }

    if (packetSize < LORA_HEADER_SIZE ||
        packetSize > LORA_MAX_PACKET) {

        ESP_LOGW(TAG,
                 "Tamaño de paquete no valido: %d",
                 packetSize);

        while (LoRa.available()) {
            LoRa.read();
        }

        return;
    }

    uint8_t frame[LORA_MAX_PACKET];

    int bytesLeidos = 0;

    while (LoRa.available() &&
           bytesLeidos < LORA_MAX_PACKET) {

        frame[bytesLeidos++] = LoRa.read();
    }

    if (bytesLeidos != packetSize) {

        ESP_LOGW(TAG, "Longitud de paquete incorrecta");

        return;
    }

    // ---------------------------------------------
    // EXTRAER CABECERA
    // ---------------------------------------------

    LoRaHeader header;

    header.version = frame[0];
    header.type    = frame[1];
    header.src     = frame[2];
    header.dst     = frame[3];

    header.msgId =
        ((uint16_t)frame[4] << 8) | frame[5];

    header.flags  = frame[6];
    header.length = frame[7];

    ESP_LOGI(TAG,
         "[RX] CABECERA: VERSION=0x%02X TYPE=0x%02X SRC=0x%02X DST=0x%02X MSG_ID=%u FLAGS=0x%02X LENGTH=%u",
         header.version,
         header.type,
         header.src,
         header.dst,
         header.msgId,
         header.flags,
         header.length);

    // ---------------------------------------------
    // VALIDAR CABECERA
    // ---------------------------------------------

    if (header.version != PROTOCOL_VERSION) {

        ESP_LOGW(TAG,
                 "Version no soportada: %u",
                 header.version);

        return;
    }

    if (header.dst != localAddress) {

        ESP_LOGW(TAG,
                 "Paquete destinado a otro dispositivo");

        return;
    }

    if (header.length != packetSize - LORA_HEADER_SIZE) {

        ESP_LOGW(TAG,
                 "Longitud de payload incorrecta");

        return;
    }

    if (header.length > LORA_MAX_PAYLOAD) {

        ESP_LOGW(TAG, "Payload demasiado grande");

        return;
    }

    const uint8_t* payload = frame + LORA_HEADER_SIZE;

    // ---------------------------------------------
    // PROCESAR DATA
    // ---------------------------------------------

    if (header.type == MSG_DATA) {

        if (header.src < 0x02 ||
            header.src > 0xFE) {

            ESP_LOGW(TAG,
                     "Direccion de nodo no valida: %u",
                     header.src);

            return;
        }

        if (header.flags != 0x00) {

            ESP_LOGW(TAG,
                     "FLAGS no soportados en DATA: %u",
                     header.flags);

            return;
        }

        if (header.length == 0) {

            ESP_LOGW(TAG, "DATA sin payload.");

            return;
        }

        // -----------------------------------------
        // COMPROBAR SI YA TENEMOS SU ACK_DATA
        // -----------------------------------------

        CachedAck respuestaGuardada;

        if (buscarACKCache(
                header.src,
                header.msgId,
                respuestaGuardada
            )) {

            ESP_LOGW(TAG,
                     "DATA duplicado: nodo=%u MSG_ID=%u",
                     header.src,
                     header.msgId);

            // Reenviamos exactamente la misma respuesta.
            enviarACKData(
                header.src,
                header.msgId,
                respuestaGuardada.payload,
                respuestaGuardada.length
            );

            return;
        }

        // -----------------------------------------
        // EXTRAER PAYLOAD
        // -----------------------------------------

        String payloadString = "";

        for (int i = 0; i < header.length; i++) {
            payloadString += (char)payload[i];
        }

        int rssi = LoRa.packetRssi();
        float snr = LoRa.packetSnr();

        ESP_LOGI(TAG,
                 "DATA recibido: nodo=%u MSG_ID=%u",
                 header.src,
                 header.msgId);

        ESP_LOGI(TAG,
                 "Payload: %s",
                 payloadString.c_str());

        ESP_LOGI(TAG,
                 "RSSI: %d dBm, SNR: %.2f dB",
                 rssi,
                 snr);

        // -----------------------------------------
        // CONSTRUIR MENSAJE MQTT
        // -----------------------------------------

        String datosMQTT =
            String(header.src) + "," + payloadString;

        // -----------------------------------------
        // PUBLICAR MQTT
        // -----------------------------------------

        uint32_t tInicio = millis();

        if (!enviarDatosMQTT(datosMQTT)) {

            ESP_LOGW(TAG,
                    "No se confirma DATA. El nodo podra reintentarlo.");

            return;
        }

        uint32_t tMQTT = millis();

        // -----------------------------------------
        // GENERAR RESPUESTA DE PRUEBA
        // -----------------------------------------

        CachedAck respuesta;

        respuesta.msgId = header.msgId;

        uint8_t numeroOrdenes =
            secuenciaRespuestas % 4;

        secuenciaRespuestas++;

        generarACKPrueba(
            numeroOrdenes,
            respuesta
        );

        uint32_t tGeneracion = millis();

        // -----------------------------------------
        // REGISTRAR EN CACHE
        // -----------------------------------------

        registrarACKCache(
            header.src,
            respuesta
        );

        uint32_t tCache = millis();

        // -----------------------------------------
        // ENVIAR ACK_DATA
        // -----------------------------------------

        bool ackEnviado = enviarACKData(
            header.src,
            header.msgId,
            respuesta.payload,
            respuesta.length
        );

        uint32_t tACK = millis();

        // -----------------------------------------
        // MEDIR TIEMPOS
        // -----------------------------------------

        ESP_LOGI(TAG,
                "TIEMPOS DATA: MQTT=%lu ms | Generacion=%lu ms | Cache=%lu ms | ACK=%lu ms | Total=%lu ms",
                (unsigned long)(tMQTT - tInicio),
                (unsigned long)(tGeneracion - tMQTT),
                (unsigned long)(tCache - tGeneracion),
                (unsigned long)(tACK - tCache),
                (unsigned long)(tACK - tInicio));

        ESP_LOGI(TAG,
                "DATA procesado: nodo=%u MSG_ID=%u ACK=%s",
                header.src,
                header.msgId,
                ackEnviado ? "OK" : "ERROR");
    }

    // ---------------------------------------------
    // PROCESAR RESULT
    // ---------------------------------------------

    else if (header.type == MSG_RESULT) {

        procesarResultado(header, payload);
    }

    else {

        ESP_LOGW(TAG,
                 "Tipo de mensaje no implementado en esta fase: 0x%02X",
                 header.type);
    }
}

// --------------------------------------------------
// CONECTAR WIFI
// --------------------------------------------------

void conectarWiFi() {

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    WiFi.begin(ssid, password);

    ESP_LOGI(TAG, "Conectando a WiFi...");

    uint32_t start = millis();

    while (WiFi.status() != WL_CONNECTED) {

        delay(250);

        if (millis() - start > 20000) {

            ESP_LOGW(TAG,
                     "WiFi timeout, reintentando...");

            start = millis();

            WiFi.disconnect();
            WiFi.begin(ssid, password);
        }
    }

    ESP_LOGI(TAG,
             "WiFi conectado con IP: %s",
             WiFi.localIP().toString().c_str());
}

// --------------------------------------------------
// CALLBACK MQTT
// --------------------------------------------------

void callbackMQTT(
    char* topic,
    byte* payload,
    unsigned int length
) {

    ESP_LOGI(TAG,
             "Mensaje recibido en topic: %s",
             topic);

    String mensaje = "";

    for (unsigned int i = 0; i < length; i++) {
        mensaje += (char)payload[i];
    }

    ESP_LOGI(TAG,
             "Payload MQTT recibido: '%s'",
             mensaje.c_str());

    // Se mantiene temporalmente el formato JSON anterior.
    // La recepción y conversión de órdenes MQTT a LoRa
    // se implementará al integrar el backend.

    int nodo_id = -1;
    int estado = -1;

    int resultado = sscanf(
        mensaje.c_str(),
        "{\"nodo\":%d,\"estado\":%d}",
        &nodo_id,
        &estado
    );

    if (resultado == 2) {

        ESP_LOGI(TAG,
                 "Orden recibida: nodo=%d, estado=%d",
                 nodo_id,
                 estado);

    } else {

        ESP_LOGW(TAG,
                 "Formato JSON de orden no reconocido.");
    }
}

// --------------------------------------------------
// CONECTAR MQTT
// --------------------------------------------------

void conectarMQTT() {

    client.setKeepAlive(30);
    client.setSocketTimeout(10);

    while (!client.connected()) {

        if (WiFi.status() != WL_CONNECTED) {

            ESP_LOGW(TAG,
                     "No se puede conectar a MQTT: WiFi desconectada.");

            conectarWiFi();
        }

        ESP_LOGI(TAG,
                 "Conectando a MQTT como %s...",
                 mqttClientID);

        if (client.connect(
                mqttClientID,
                mqtt_user,
                mqtt_password
            )) {

            ESP_LOGI(TAG, "Conectado a MQTT.");

            if (client.subscribe(topic_actuadores)) {

                ESP_LOGI(TAG,
                         "Suscrito a: %s",
                         topic_actuadores);

            } else {

                ESP_LOGE(TAG,
                         "Error suscribiendo a %s",
                         topic_actuadores);
            }

        } else {

            ESP_LOGE(TAG,
                     "Error conectando a MQTT, rc=%d",
                     client.state());

            delay(3000);
        }
    }
}

// --------------------------------------------------
// COMPROBAR CONEXIONES
// --------------------------------------------------

void checkConexiones() {

    if (WiFi.status() != WL_CONNECTED) {

        ESP_LOGW(TAG,
                 "WiFi desconectada, reconectando...");

        conectarWiFi();
    }

    if (!client.connected()) {

        ESP_LOGW(TAG,
                 "MQTT desconectado, reconectando...");

        conectarMQTT();
    }
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------

void setup() {

    Serial.begin(115200);
    Serial.setTimeout(50);

    delay(1000);

    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG,
             "GLOBALIoT - Gateway LoRa v1.0 Multi-Gateway Fase 2");

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    // ---------------------------------------------
    // IDENTIDAD DEL GATEWAY
    // ---------------------------------------------

    cargarConfiguracionGateway();

    // ---------------------------------------------
    // CONTADOR DE COMMAND_ID
    // ---------------------------------------------

    cargarContadorCommandId();

    // ---------------------------------------------
    // SENSOR DHT
    // ---------------------------------------------

    dht.begin();

    // ---------------------------------------------
    // INICIALIZAR LORA
    // ---------------------------------------------

    LoRa.setPins(csPin, resetPin, irqPin);

    if (!LoRa.begin(868E6)) {

        ESP_LOGE(TAG, "Fallo al iniciar LoRa");

        while (true) {
            delay(1000);
        }
    }

    LoRa.setSyncWord(0x33);
    LoRa.enableCrc();
    LoRa.receive();

    ESP_LOGI(TAG,
             "LoRa iniciado. SyncWord=0x33");

    // ---------------------------------------------
    // WIFI Y MQTT
    // ---------------------------------------------

    conectarWiFi();

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callbackMQTT);

    conectarMQTT();

    ESP_LOGI(TAG,
             "Gateway %s preparado.",
             gatewayUID);
}

// --------------------------------------------------
// LOOP
// --------------------------------------------------

void loop() {

    procesarConfiguracionSerie();

    procesarPaqueteLoRa();

    client.loop();

    loopCounter++;

    if (loopCounter % 1000 == 0) {

        ESP_LOGI(TAG,
                 "[CICLO %lu] Heap libre: %u, WiFi: %s, MQTT: %s",
                 loopCounter,
                 esp_get_free_heap_size(),
                 WiFi.status() == WL_CONNECTED
                    ? "OK"
                    : "DESCONECTADO",
                 client.connected()
                    ? "CONECTADO"
                    : "DESCONECTADO");

        checkConexiones();
    }

    delay(10);
}

// --------------------------------------------------
// APP MAIN - ESP-IDF + ARDUINO COMO COMPONENTE
// --------------------------------------------------

extern "C" void app_main() {

    initArduino();

    setup();

    while (true) {

        loop();

        delay(1);
    }
}
*/


/* versión anterior a la de arriba 

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Preferences.h>

#include <map>
#include <deque>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// --------------------------------------------------
// IDENTIDAD DEL GATEWAY - MULTI-GATEWAY
// --------------------------------------------------

Preferences preferences;

// Identificador utilizado solamente si todavía no hay
// uno guardado en NVS.
const char* DEFAULT_GATEWAY_UID = "GW0001";

char gatewayUID[16];
char mqttClientID[32];

char topic_data[64];
char topic_actuadores[64];
char topic_respuesta[64];

// --------------------------------------------------
// CACHE DE MENSAJES PROCESADOS
// --------------------------------------------------

std::map<uint8_t, std::deque<uint16_t>> mensajesProcesados;

const size_t MAX_MENSAJES_CACHE = 8;

// --------------------------------------------------
// LOG
// --------------------------------------------------

static const char* TAG = "SCIGateway";

// --------------------------------------------------
// WIFI
// --------------------------------------------------

const char* ssid = "DIGIFIBRA-fAUx";
const char* password = "yGxhH7heZdQF";

// --------------------------------------------------
// MQTT
// --------------------------------------------------

const char* mqtt_server = "13.37.77.172";
const int mqtt_port = 1883;

const char* mqtt_user = "usuario";
const char* mqtt_password = "clave";

WiFiClient espClient;
PubSubClient client(espClient);

// --------------------------------------------------
// PROTOCOLO LORA GLOBALIOT - VERSION 1
// --------------------------------------------------

#define PROTOCOL_VERSION       0x01

#define MSG_DATA               0x01
#define MSG_ACK_DATA           0x02
#define MSG_ACK_DEF            0x03
#define MSG_COMMAND_STARTED    0x04
#define MSG_RESULT              0x05
#define MSG_ACK_RESULT          0x06
#define MSG_ACK_STARTED         0x07

#define LORA_HEADER_SIZE        8
#define LORA_MAX_PACKET         255
#define LORA_MAX_PAYLOAD        (LORA_MAX_PACKET - LORA_HEADER_SIZE)

const byte localAddress = 0x01;

struct LoRaHeader {
    uint8_t version;
    uint8_t type;
    uint8_t src;
    uint8_t dst;
    uint16_t msgId;
    uint8_t flags;
    uint8_t length;
};

// --------------------------------------------------
// PINES Y SENSORES
// --------------------------------------------------

const int csPin = 5;
const int resetPin = 14;
const int irqPin = 4;

#define LED       2
#define DHTpin    15
#define DHTTYPE   DHT22

DHT dht(DHTpin, DHTTYPE);

float temp;
float hum;

// --------------------------------------------------
// DEPURACIÓN
// --------------------------------------------------

unsigned long loopCounter = 0;

// --------------------------------------------------
// DECLARACIÓN DE FUNCIONES
// --------------------------------------------------

bool validarGatewayUID(const String& uid);
void cargarConfiguracionGateway();
void construirTopicsMQTT();
void procesarConfiguracionSerie();

bool mensajeYaProcesado(uint8_t nodo, uint16_t msgId);
void registrarMensajeProcesado(uint8_t nodo, uint16_t msgId);

void enviarACKData(uint8_t destino, uint16_t msgId);
bool enviarDatosMQTT(const String& datos);
void procesarPaqueteLoRa();

void conectarWiFi();
void conectarMQTT();
void callbackMQTT(char* topic, byte* payload, unsigned int length);
void checkConexiones();

// --------------------------------------------------
// VALIDAR IDENTIFICADOR DEL GATEWAY
// --------------------------------------------------

bool validarGatewayUID(const String& uid) {

    // Longitud permitida: 1 a 12 caracteres.
    if (uid.length() < 1 || uid.length() > 12) {
        return false;
    }

    // Solo permitimos letras mayúsculas,
    // números, guion y guion bajo.
    for (size_t i = 0; i < uid.length(); i++) {

        char c = uid.charAt(i);

        bool valido =
            (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') ||
            c == '-' ||
            c == '_';

        if (!valido) {
            return false;
        }
    }

    return true;
}

// --------------------------------------------------
// CARGAR IDENTIFICADOR DESDE NVS
// --------------------------------------------------

void cargarConfiguracionGateway() {

    if (!preferences.begin("globaliot", false)) {

        ESP_LOGE(TAG, "No se pudo abrir la memoria NVS.");

        while (true) {
            delay(1000);
        }
    }

    String uidGuardado =
        preferences.getString("gateway_uid", "");

    // Si todavía no existe un identificador,
    // asignamos el identificador inicial.
    if (uidGuardado.length() == 0) {

        uidGuardado = DEFAULT_GATEWAY_UID;

        preferences.putString("gateway_uid", uidGuardado);

        ESP_LOGW(TAG,
                 "No existia gateway_uid. Se asigna %s",
                 uidGuardado.c_str());
    }

    if (!validarGatewayUID(uidGuardado)) {

        ESP_LOGE(TAG,
                 "gateway_uid almacenado no valido: %s",
                 uidGuardado.c_str());

        while (true) {
            delay(1000);
        }
    }

    snprintf(
        gatewayUID,
        sizeof(gatewayUID),
        "%s",
        uidGuardado.c_str()
    );

    construirTopicsMQTT();

    ESP_LOGI(TAG, "Identidad del gateway: %s", gatewayUID);
}

// --------------------------------------------------
// CONSTRUIR TOPICS MQTT
// --------------------------------------------------

void construirTopicsMQTT() {

    snprintf(
        mqttClientID,
        sizeof(mqttClientID),
        "GlobalIoT_%s",
        gatewayUID
    );

    snprintf(
        topic_data,
        sizeof(topic_data),
        "gateway/%s/data",
        gatewayUID
    );

    snprintf(
        topic_actuadores,
        sizeof(topic_actuadores),
        "gateway/%s/actuadores",
        gatewayUID
    );

    snprintf(
        topic_respuesta,
        sizeof(topic_respuesta),
        "gateway/%s/respuesta",
        gatewayUID
    );

    ESP_LOGI(TAG, "MQTT Client ID: %s", mqttClientID);
    ESP_LOGI(TAG, "Topic DATA: %s", topic_data);
    ESP_LOGI(TAG, "Topic ACTUADORES: %s", topic_actuadores);
    ESP_LOGI(TAG, "Topic RESPUESTA: %s", topic_respuesta);
}

// --------------------------------------------------
// CONFIGURAR IDENTIDAD POR PUERTO SERIE
// --------------------------------------------------
//
// Comando:
//
// GWUID=GW0002
//
// El nuevo identificador se guarda en NVS
// y el ESP32 se reinicia automáticamente.
// --------------------------------------------------

void procesarConfiguracionSerie() {

    if (!Serial.available()) {
        return;
    }

    String comando = Serial.readStringUntil('\n');

    comando.trim();

    if (!comando.startsWith("GWUID=")) {
        return;
    }

    String nuevoUID = comando.substring(6);

    nuevoUID.trim();

    if (!validarGatewayUID(nuevoUID)) {

        ESP_LOGW(TAG,
                 "Identificador no valido. Usa 1-12 caracteres: A-Z, 0-9, - o _.");

        return;
    }

    if (nuevoUID == gatewayUID) {

        ESP_LOGI(TAG,
                 "El gateway ya tiene el identificador %s",
                 gatewayUID);

        return;
    }

    ESP_LOGI(TAG,
             "Cambiando gateway_uid de %s a %s",
             gatewayUID,
             nuevoUID.c_str());

    size_t escritos =
        preferences.putString("gateway_uid", nuevoUID);

    if (escritos == 0) {

        ESP_LOGE(TAG, "Error guardando gateway_uid en NVS.");

        return;
    }

    ESP_LOGI(TAG,
             "Identificador guardado. Reiniciando gateway...");

    Serial.flush();

    delay(500);

    ESP.restart();
}

// --------------------------------------------------
// CONTROL DE MENSAJES DUPLICADOS
// --------------------------------------------------

bool mensajeYaProcesado(uint8_t nodo, uint16_t msgId) {

    auto &mensajes = mensajesProcesados[nodo];

    for (uint16_t id : mensajes) {

        if (id == msgId) {
            return true;
        }
    }

    return false;
}

void registrarMensajeProcesado(uint8_t nodo, uint16_t msgId) {

    auto &mensajes = mensajesProcesados[nodo];

    mensajes.push_back(msgId);

    while (mensajes.size() > MAX_MENSAJES_CACHE) {
        mensajes.pop_front();
    }
}

// --------------------------------------------------
// ENVIAR ACK_DATA POR LORA
// --------------------------------------------------

void enviarACKData(uint8_t destino, uint16_t msgId) {

    uint8_t frame[LORA_HEADER_SIZE];

    frame[0] = PROTOCOL_VERSION;
    frame[1] = MSG_ACK_DATA;
    frame[2] = localAddress;
    frame[3] = destino;

    frame[4] = (msgId >> 8) & 0xFF;
    frame[5] = msgId & 0xFF;

    frame[6] = 0x00;
    frame[7] = 0x00;

    LoRa.beginPacket();

    LoRa.write(frame, LORA_HEADER_SIZE);

    LoRa.endPacket();

    ESP_LOGI(TAG,
             "ACK_DATA enviado a nodo %u, MSG_ID=%u",
             destino,
             msgId);

    LoRa.receive();
}

// --------------------------------------------------
// PUBLICAR DATOS EN MQTT
// --------------------------------------------------

bool enviarDatosMQTT(const String& datos) {

    if (!client.connected()) {

        ESP_LOGW(TAG,
                 "MQTT desconectado. No se confirma DATA.");

        return false;
    }

    bool publicado = client.publish(
        topic_data,
        datos.c_str()
    );

    if (publicado) {

        ESP_LOGI(TAG,
                 "MQTT publicado: '%s' en '%s'",
                 datos.c_str(),
                 topic_data);

        return true;
    }

    ESP_LOGW(TAG, "Fallo al publicar en MQTT.");

    return false;
}

// --------------------------------------------------
// PROCESAR PAQUETE LORA
// --------------------------------------------------

void procesarPaqueteLoRa() {

    int packetSize = LoRa.parsePacket();

    if (packetSize == 0) {
        return;
    }

    if (packetSize < LORA_HEADER_SIZE ||
        packetSize > LORA_MAX_PACKET) {

        ESP_LOGW(TAG,
                 "Tamaño de paquete no valido: %d",
                 packetSize);

        while (LoRa.available()) {
            LoRa.read();
        }

        return;
    }

    uint8_t frame[LORA_MAX_PACKET];

    int bytesLeidos = 0;

    while (LoRa.available() &&
           bytesLeidos < LORA_MAX_PACKET) {

        frame[bytesLeidos++] = LoRa.read();
    }

    if (bytesLeidos != packetSize) {

        ESP_LOGW(TAG, "Longitud de paquete incorrecta");

        return;
    }

    // ---------------------------------------------
    // EXTRAER CABECERA
    // ---------------------------------------------

    LoRaHeader header;

    header.version = frame[0];
    header.type    = frame[1];
    header.src     = frame[2];
    header.dst     = frame[3];

    header.msgId =
        ((uint16_t)frame[4] << 8) | frame[5];

    header.flags  = frame[6];
    header.length = frame[7];

    // ---------------------------------------------
    // VALIDAR CABECERA
    // ---------------------------------------------

    if (header.version != PROTOCOL_VERSION) {

        ESP_LOGW(TAG,
                 "Version no soportada: %u",
                 header.version);

        return;
    }

    if (header.dst != localAddress &&
        header.dst != 0xFF) {

        ESP_LOGW(TAG,
                 "Paquete destinado a otro dispositivo");

        return;
    }

    if (header.length != packetSize - LORA_HEADER_SIZE) {

        ESP_LOGW(TAG,
                 "Longitud de payload incorrecta");

        return;
    }

    if (header.length > LORA_MAX_PAYLOAD) {

        ESP_LOGW(TAG, "Payload demasiado grande");

        return;
    }

    // ---------------------------------------------
    // PROCESAR DATA
    // ---------------------------------------------

    if (header.type == MSG_DATA) {

        if (header.src < 0x02 ||
            header.src > 0xFE) {

            ESP_LOGW(TAG,
                     "Direccion de nodo no valida: %u",
                     header.src);

            return;
        }

        if (header.flags != 0x00) {

            ESP_LOGW(TAG,
                     "FLAGS no soportados en DATA: %u",
                     header.flags);

            return;
        }

        if (header.length == 0) {

            ESP_LOGW(TAG, "DATA sin payload.");

            return;
        }

        // -----------------------------------------
        // COMPROBAR DUPLICADOS
        // -----------------------------------------

        if (mensajeYaProcesado(header.src, header.msgId)) {

            ESP_LOGW(TAG,
                     "DATA duplicado: nodo=%u MSG_ID=%u",
                     header.src,
                     header.msgId);

            // Se vuelve a confirmar el mensaje.
            enviarACKData(header.src, header.msgId);

            return;
        }

        // -----------------------------------------
        // EXTRAER PAYLOAD
        // -----------------------------------------

        String payload = "";

        for (int i = 0; i < header.length; i++) {

            payload += (char)frame[LORA_HEADER_SIZE + i];
        }

        int rssi = LoRa.packetRssi();
        float snr = LoRa.packetSnr();

        ESP_LOGI(TAG,
                 "DATA recibido: nodo=%u MSG_ID=%u",
                 header.src,
                 header.msgId);

        ESP_LOGI(TAG, "Payload: %s", payload.c_str());

        ESP_LOGI(TAG,
                 "RSSI: %d dBm, SNR: %.2f dB",
                 rssi,
                 snr);

        // -----------------------------------------
        // CONSTRUIR MENSAJE MQTT
        // -----------------------------------------

        // La direccion LoRa del nodo es el primer
        // campo del payload MQTT.

        String datosMQTT =
            String(header.src) + "," + payload;

        // -----------------------------------------
        // PUBLICAR MQTT
        // -----------------------------------------

        if (!enviarDatosMQTT(datosMQTT)) {

            ESP_LOGW(TAG,
                     "No se confirma DATA. El nodo podra reintentarlo.");

            return;
        }

        // -----------------------------------------
        // REGISTRAR Y CONFIRMAR
        // -----------------------------------------

        registrarMensajeProcesado(
            header.src,
            header.msgId
        );

        enviarACKData(
            header.src,
            header.msgId
        );

        ESP_LOGI(TAG,
                 "DATA procesado y confirmado: nodo=%u MSG_ID=%u",
                 header.src,
                 header.msgId);
    }
}

// --------------------------------------------------
// CONECTAR WIFI
// --------------------------------------------------

void conectarWiFi() {

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    WiFi.begin(ssid, password);

    ESP_LOGI(TAG, "Conectando a WiFi...");

    uint32_t start = millis();

    while (WiFi.status() != WL_CONNECTED) {

        delay(250);

        if (millis() - start > 20000) {

            ESP_LOGW(TAG,
                     "WiFi timeout, reintentando...");

            start = millis();

            WiFi.disconnect();

            WiFi.begin(ssid, password);
        }
    }

    ESP_LOGI(TAG,
             "WiFi conectado con IP: %s",
             WiFi.localIP().toString().c_str());
}

// --------------------------------------------------
// CALLBACK MQTT
// --------------------------------------------------

void callbackMQTT(
    char* topic,
    byte* payload,
    unsigned int length
) {

    ESP_LOGI(TAG,
             "Mensaje recibido en topic: %s",
             topic);

    String mensaje = "";

    for (unsigned int i = 0; i < length; i++) {

        mensaje += (char)payload[i];
    }

    ESP_LOGI(TAG,
             "Payload MQTT recibido: '%s'",
             mensaje.c_str());

    // Por ahora conservamos el formato JSON
    // que tenía el firmware original.
    //
    // La implementación definitiva de órdenes
    // se abordará al integrar el protocolo
    // de comandos LoRa.

    int nodo_id = -1;
    int estado = -1;

    int resultado = sscanf(
        mensaje.c_str(),
        "{\"nodo\":%d,\"estado\":%d}",
        &nodo_id,
        &estado
    );

    if (resultado == 2) {

        ESP_LOGI(TAG,
                 "Orden recibida: nodo=%d, estado=%d",
                 nodo_id,
                 estado);

    } else {

        ESP_LOGW(TAG,
                 "Formato JSON de orden no reconocido.");
    }
}

// --------------------------------------------------
// CONECTAR MQTT
// --------------------------------------------------

void conectarMQTT() {

    client.setKeepAlive(30);
    client.setSocketTimeout(10);

    while (!client.connected()) {

        if (WiFi.status() != WL_CONNECTED) {

            ESP_LOGW(TAG,
                     "No se puede conectar a MQTT: WiFi desconectada.");

            conectarWiFi();
        }

        ESP_LOGI(TAG,
                 "Conectando a MQTT como %s...",
                 mqttClientID);

        if (client.connect(
                mqttClientID,
                mqtt_user,
                mqtt_password
            )) {

            ESP_LOGI(TAG, "Conectado a MQTT.");

            // -------------------------------------
            // SUSCRIPCION DEL GATEWAY
            // -------------------------------------

            // El gateway solamente recibe órdenes
            // por su topic de actuadores.
            //
            // NO se suscribe a su topic respuesta:
            // ese topic es de publicación.

            if (client.subscribe(topic_actuadores)) {

                ESP_LOGI(TAG,
                         "Suscrito a: %s",
                         topic_actuadores);

            } else {

                ESP_LOGE(TAG,
                         "Error suscribiendo a %s",
                         topic_actuadores);
            }

        } else {

            ESP_LOGE(TAG,
                     "Error conectando a MQTT, rc=%d",
                     client.state());

            delay(3000);
        }
    }
}

// --------------------------------------------------
// COMPROBAR CONEXIONES
// --------------------------------------------------

void checkConexiones() {

    if (WiFi.status() != WL_CONNECTED) {

        ESP_LOGW(TAG,
                 "WiFi desconectada, reconectando...");

        conectarWiFi();
    }

    if (!client.connected()) {

        ESP_LOGW(TAG,
                 "MQTT desconectado, reconectando...");

        conectarMQTT();
    }
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------

void setup() {

    Serial.begin(115200);
    Serial.setTimeout(50);

    delay(1000);

    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG,
             "GLOBALIoT - Gateway LoRa v1.0 Multi-Gateway");

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);

    // ---------------------------------------------
    // IDENTIDAD DEL GATEWAY
    // ---------------------------------------------

    cargarConfiguracionGateway();

    // ---------------------------------------------
    // SENSOR DHT
    // ---------------------------------------------

    dht.begin();

    // ---------------------------------------------
    // INICIALIZAR LORA
    // ---------------------------------------------

    LoRa.setPins(csPin, resetPin, irqPin);

    if (!LoRa.begin(868E6)) {

        ESP_LOGE(TAG, "Fallo al iniciar LoRa");

        while (true) {
            delay(1000);
        }
    }

    // SyncWord se mantiene manualmente configurado.
    LoRa.setSyncWord(0x33);

    // CRC por hardware del SX127x.
    LoRa.enableCrc();

    LoRa.receive();

    ESP_LOGI(TAG,
             "LoRa iniciado. SyncWord=0x33");

    // ---------------------------------------------
    // WIFI Y MQTT
    // ---------------------------------------------

    conectarWiFi();

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callbackMQTT);

    conectarMQTT();

    ESP_LOGI(TAG,
             "Gateway %s preparado.",
             gatewayUID);
}

// --------------------------------------------------
// LOOP
// --------------------------------------------------

void loop() {

    // ---------------------------------------------
    // 1. CONFIGURACION POR PUERTO SERIE
    // ---------------------------------------------

    procesarConfiguracionSerie();

    // ---------------------------------------------
    // 2. RECEPCION LORA
    // ---------------------------------------------

    procesarPaqueteLoRa();

    // ---------------------------------------------
    // 3. ATENCION MQTT
    // ---------------------------------------------

    client.loop();

    // ---------------------------------------------
    // 4. COMPROBAR CONEXIONES
    // ---------------------------------------------

    loopCounter++;

    if (loopCounter % 1000 == 0) {

        ESP_LOGI(TAG,
                 "[CICLO %lu] Heap libre: %u, WiFi: %s, MQTT: %s",
                 loopCounter,
                 esp_get_free_heap_size(),
                 WiFi.status() == WL_CONNECTED
                    ? "OK"
                    : "DESCONECTADO",
                 client.connected()
                    ? "CONECTADO"
                    : "DESCONECTADO");

        checkConexiones();
    }

    delay(10);
}

// --------------------------------------------------
// APP MAIN - ESP-IDF + ARDUINO COMO COMPONENTE
// --------------------------------------------------

extern "C" void app_main() {

    initArduino();

    setup();

    while (true) {

        loop();

        delay(1);
    }
}
*/

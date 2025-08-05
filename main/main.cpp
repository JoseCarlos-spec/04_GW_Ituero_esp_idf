#include <Arduino.h>
#include <SPI.h>              // SPI : miso mosi communication
#include <LoRa.h>             // wireless lora
#include <WiFi.h>           
#include <WiFiClient.h>       //http protocol
#include <HTTPClient.h>
#include <PubSubClient.h>     //MQTT protocol

#include "esp_log.h"
static const char* TAG = "SCIGateway";

//wifi credentials
const char* ssid = "DIGIFIBRA-fAUx";
const char* password = "yGxhH7heZdQF";

const char* mqtt_server = "13.37.77.172";    //AWS lightsail server
const int mqtt_port = 1883;
const char* mqtt_user = "usuario";
const char* mqtt_password = "clave";
const char* topic_data = "nodo/datos";
const char* topic_actuadores = "nodo/actuadores";
const char* topic_respuesta = "nodo/respuesta";

WiFiClient espClient;
PubSubClient client(espClient);

const int csPin = 5;          // LoRa radio chip select
const int resetPin = 14;      // LoRa radio reset
const int irqPin = 4;         // IRQ pin

String outgoing;              // outgoing message
String receivedMsg;           // incoming message
byte msgID = 0;               // ID of the received message
byte localAddress = 0x01;     // address of this device
byte destination = 0x02;      // destination node
const byte paytype = 0xAA;    // normal message
const byte acktype = 0xBB;    // ack message
boolean correcto = false;     // message validation flag
byte msgtype = paytype;       // default type
String RSSI;                  // signal strength
String Snr;                   // signal-to-noise ratio
unsigned long loopCounter = 0;// para depuración
volatile int errorCode = 0;   // si el mensaje viene con errores, identificar el error

int i = 0;                    // task index

#define LED 2

uint8_t checksum(String msg) {
  uint8_t value = 0;
  for (size_t i = 0; i < msg.length(); i++) {
    value += msg[i];
  }
  return value;
}

void sendMessage() {
  LoRa.beginPacket();
  LoRa.write(destination);
  LoRa.write(localAddress);
  LoRa.write(msgID);
  LoRa.write(msgtype);
  LoRa.write(checksum(outgoing));
  LoRa.write(outgoing.length());
  LoRa.print(outgoing);
  ESP_LOGD(TAG, "Enviando paquete LoRa a 0x%02X, tipo=0x%02X, ID=%d, payload='%s'", destination, msgtype, msgID, outgoing.c_str());
  LoRa.endPacket();
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  byte recipient = LoRa.read();
  byte sender = LoRa.read();
  byte incomingMsgId = LoRa.read();
  byte incomingtype = LoRa.read();
  byte check = LoRa.read();
  byte incomingLength = LoRa.read();

  String incoming = "";
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {
    errorCode = 1;
    return;
  }

  if (recipient != localAddress && recipient != 0xFF) {
    errorCode = 2;
    return;
  }

  if (checksum(incoming) != check) {
    errorCode = 3;
    return;
  }

  int tareas[5] = {0,1,2,3,4};
  if (i == 5) i = 0;
  outgoing = String(tareas[i]);
  i++;

  if (incomingtype == paytype) {
    correcto = true;
    destination = sender;
    msgID = incomingMsgId;
    receivedMsg = incoming;
    msgtype = acktype;
  } else {
    ESP_LOGI(TAG, "Tarea confirmada por nodo");
  }

  RSSI = String(LoRa.packetRssi());
  Snr = String(LoRa.packetSnr());
}

void conectarWiFi() {
  WiFi.begin(ssid, password);
  ESP_LOGI(TAG, "Conectando a WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  ESP_LOGI(TAG, "WiFi conectado con IP: %s", WiFi.localIP().toString().c_str());
}

void conectarMQTT() {
  while (!client.connected()) {
    ESP_LOGI(TAG, "Conectando a MQTT...");
    if (client.connect("ESP32_Gateway", mqtt_user, mqtt_password)) {
      ESP_LOGI(TAG, "Conectado a MQTT");
    } else {
      ESP_LOGE(TAG, "Error conectando a MQTT, rc=%d", client.state());
      delay(5000);
    }
  }
}

void enviarDatosMQTT(String datos) {
  if (client.connected()) {
    client.publish(topic_data, datos.c_str());
    ESP_LOGI(TAG, "MQTT publicado: '%s' en topic '%s'", datos.c_str(), topic_data);
  }
}

void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  ESP_LOGI(TAG, "Mensaje recibido en %s", topic);
  String mensaje = "";
  for (int i = 0; i < length; i++) {
    mensaje += (char)payload[i];
  }
  ESP_LOGI(TAG, "MQTT recibido en topic '%s': '%s'", topic, mensaje.c_str());

}

void checkConexiones() {
  if (WiFi.status() != WL_CONNECTED) {
    ESP_LOGW(TAG, "WiFi desconectada, reconectando...");
    conectarWiFi();
  }

  if (!client.connected()) {
    ESP_LOGW(TAG, "MQTT desconectado, reconectando...");
    conectarMQTT();
  }
}

void setup() {
  Serial.begin(115200);
  esp_log_level_set("*", ESP_LOG_WARN);         // El resto del sistema solo advertencias y errores
  esp_log_level_set(TAG, ESP_LOG_DEBUG);        // Tus logs en modo DEBUG
  while (!Serial);
  ESP_LOGI(TAG, "Ituero LoRa Gateway v0.1");

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(868E6)) {
    ESP_LOGE(TAG, "Fallo al iniciar LoRa");
    while (true);
  }
  LoRa.setSyncWord(0x33);
  LoRa.onReceive(onReceive);
  LoRa.receive();
  ESP_LOGI(TAG, "LoRa iniciado correctamente");

  conectarWiFi();
  client.setServer(mqtt_server, mqtt_port);
  conectarMQTT();
  client.setCallback(callbackMQTT);
  client.subscribe(topic_respuesta);
  client.subscribe(topic_actuadores);
}

void loop() {
  LoRa.receive();
  client.loop();
  delay(100);


  loopCounter++;
  if (loopCounter % 1000 == 0) {
    ESP_LOGI(TAG, "[CICLO %lu] Heap libre: %u, Min heap: %u, WiFi: %s, MQTT: %s",
             loopCounter,
             esp_get_free_heap_size(),
             esp_get_minimum_free_heap_size(),
             WiFi.status() == WL_CONNECTED ? "OK" : "DESCONECTADO",
             client.connected() ? "CONECTADO" : "DESCONECTADO");
    checkConexiones();
  }

  if (correcto) {
    sendMessage();
    correcto = false;
    ESP_LOGI(TAG, "Enviando ACK con tarea: %s", outgoing.c_str());

    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);

    receivedMsg += "," + String(RSSI) + "," + String(Snr);
    ESP_LOGI(TAG, "Mensaje recibido: %s", receivedMsg.c_str());
    enviarDatosMQTT(receivedMsg);
    ESP_LOGI(TAG, "Heap libre: %u bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Min heap histórico: %d bytes", esp_get_minimum_free_heap_size());

    receivedMsg = ""; // Limpiar buffer
  }

  if (errorCode != 0) {
    switch (errorCode) {
      case 1: ESP_LOGW(TAG, "Mensaje descartado: longitudes no coinciden"); break;
      case 2: ESP_LOGW(TAG, "Mensaje no dirigido a este nodo"); break;
      case 3: ESP_LOGW(TAG, "Checksum incorrecto"); break;
    }
    errorCode = 0;
  }
}

extern "C" void app_main() {
  initArduino();
  setup();
  while (true) {
    loop();
    delay(1);
  }
}

/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <Arduino.h>
#include <SPI.h>              // SPI : miso mosi communication
#include <LoRa.h>             // wireless lora
#include <WiFi.h>           
#include <WiFiClient.h>       //http protocol
#include <HTTPClient.h>
#include <PubSubClient.h>     //MQTT protocol

#include "esp_log.h"
static const char* TAG = "SCIGateway";

//#include <rtc_wdt.h>        //to disable watchdog timeout blocking problem

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

const int csPin = 5;          // LoRa radio chip select (const = var constante, no se puede alterar)
const int resetPin = 14;      // LoRa radio reset
const int irqPin = 4;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message
String receivedMsg;           // incoming message
byte msgID = 0;               // ID of the received message
byte localAddress = 0x01;     // address of this device
byte destination = 0x02;      // destination to send to (the node address of the sender node)
const byte paytype = 0xAA;    // message is a normal one
const byte acktype = 0xBB;    // message is an ack confirmation
boolean correcto = false;     // mensaje recibido correcto o no
byte msgtype = paytype;       // it can be normal or ack message
String RSSI;                  // lora received signal level in dBm
String Snr;                   // lora signal to noise ratio

int i = 0;                    //indice provisional para un for provisional

#define LED 2


uint8_t checksum (String msg) {               //Checksum 8 bits complemento a 2

uint8_t value = 0;                            //uint8-t equivalente a un byte, de 0 a 255
for (size_t i = 0; i < msg.length(); i++) {   //size-t equivalente a unsigned long
  value += msg[i];
}                        
 return value;
}

void sendMessage() {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address (the sender node)
  LoRa.write(localAddress);             // add gateway address 
  LoRa.write(msgID);                 // add message ID from the received msg
  LoRa.write(msgtype);                     // add message type (data or ack)
  LoRa.write(checksum(outgoing));       // add outgoing checksum               
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it

}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  byte recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingtype = LoRa.read();      // message type (normal or ack)
  byte check = LoRa.read();             // incoming msg checksum
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";                 // payload of packet

  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }

  // check message length. It must match for error, if no, discard message

  if (incomingLength != incoming.length()) {   
    Serial.println("error: message length does not match");
    return;                             
  }

  // if the recipient isn't this device or broadcast, discard message

  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             
  }
  
  //checksum must match. 

  if (checksum(incoming) != check) {            
    Serial.println("Checksum does not match");
    return;                                      
  }

  //at this point, the message is OK.
  //Get ready to send an ack

  int tareas[5] = {0,1,2,3,4};
  
  if (i == 5) i = 0;
  outgoing = tareas[i];
  i++;

  //if the incoming message is a payload, send an ack

  if (incomingtype == paytype) {

    correcto = true;              //the incoming message is correct
    destination = sender;         //send the ack to the current sender
    msgID = incomingMsgId;        //the ID is the same
    receivedMsg = incoming;       //get the message
    msgtype = acktype;            //set the outgoing message as an ack

  } else {                        //the msg is an ack. no need to send anything

    //Serial.println("la instruccion se ha recibido correctamente");
    //Serial.println();
    //////////////////////////////////////////////////////////
    // WHAT TO DO WHEN THE TASK IS CONFIRMED                //
    //////////////////////////////////////////////////////////
    
  }
  RSSI = String(LoRa.packetRssi());
  Snr = String(LoRa.packetSnr());

}

void conectarWiFi() {

  WiFi.begin(ssid,password);
  Serial.println("Connecting wifi..");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado con IP: " + WiFi.localIP().toString());
}

void conectarMQTT() {
  while (!client.connected()) {
    Serial.print("Conectando a MQTT...");
    if (client.connect("ESP32_Gateway", mqtt_user, mqtt_password)) {
      Serial.println("Conectado a MQTT");
    } else {
      Serial.print("Error conectando a MQTT, rc= ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void enviarDatosMQTT(String datos) {
  if (client.connected()) {
    client.publish(topic_data, datos.c_str());
    //Serial.print("Enviando por MQTT, esperar confirmación: ");
    //Serial.println(datos.c_str());
  }
}

void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  //Serial.print("Mensaje recibido en ");
  //Serial.print(topic);
  //Serial.print(": ");
  String mensaje = "";
  for (int i=0; i<length; i++) {
    mensaje += (char)payload[i];
  }
  //Serial.println("Orden recibida: " + mensaje);
}

void setup() {                            //void = no se espera respuesta
  Serial.begin(115200);                   // initialize serial
  while (!Serial);

  Serial.println("Ituero LoRa Duplex and MQTT with callback gateway v.01");

  //rtc_wdt_protect_off();    //disable WDT lock problem, only if necessary. 
  //rtc_wdt_disable();        //check program in advance.

  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(868E6)) {             // initialize ratio at 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  LoRa.setSyncWord(0x33); 
  LoRa.onReceive(onReceive);
  LoRa.receive();
  Serial.println("LoRa init succeeded.");

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
  if (correcto) {
    //outgoing = "te envio el ack";
    sendMessage();
    correcto = false;
    //Serial.println("enviando el ack y la tarea " + String(outgoing));
    LoRa.receive();
    digitalWrite(LED,HIGH);
    delay (100);
    digitalWrite(LED,LOW);
    //parseMsg(receivedMsg);              //only for checking
    receivedMsg += "," + String(RSSI) + "," + String(Snr);
    Serial.println(receivedMsg);
    enviarDatosMQTT(receivedMsg);
  }

//////////////////////////////////////////
//  MANAGE TASKS TO SEND                //
//////////////////////////////////////////


}

extern "C" void app_main() {
  initArduino();
  setup();
  while (true) {
    loop();
    delay(1);  // muy importante para evitar el watchdog
  }
}

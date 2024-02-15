#include <RFM69.h>
#include <RFM69_ATC.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <espMqttClient.h>
#include <mbedtls/base64.h>
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>

#define FREQUENCY     RF69_868MHZ
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define SERIAL_BAUD   115200
#define WDT_TIMEOUT 700

uint8_t reconnectAttempts = 0;
uint8_t nodeID = 100;
uint8_t networkID = 200;
char encryptKey[] = "1234567812345678";
char mqttServer[40] = {"192.168.178.3\0"};
uint16_t mqttPort = 8883;
char mqttServerCA[2048] = {0};
char mqttClientCert[2048] = {0};
char mqttClientKey[2500] = {0};

Preferences prefs;
bool shouldSaveConfig = false;

void saveConfigCallback() {
  shouldSaveConfig = true;
}

RFM69 radio;
bool spy = true; //set to 'true' to sniff all packets on the same network
char json_string[256];

WiFiManager wm;
typedef struct __attribute__((packed)) {
    uint8_t id;    // byte from 0-255
    uint8_t sensortype;    // byte from 0-255
    int16_t temperature;      // temperature * 100 to avoid float
    uint8_t humidity;    // byte relative humidity from 0 to 100
    uint16_t batttery_voltage; // battery * 100
    uint32_t checksum; // checksum byte
} SensorData;

SensorData sensorData = {};

espMqttClientSecure mqttClient(espMqttClientTypes::UseInternalTask::NO);
static TaskHandle_t taskHandle;
bool reconnectMqtt = false;
uint32_t lastReconnect = 0;

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  if (!mqttClient.connect()) {
    reconnectMqtt = true;
    lastReconnect = millis();
    Serial.println("Connecting failed.");
  } else {
    reconnectMqtt = false;
  }
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      break;
    default:
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason) {
  Serial.printf("Disconnected from MQTT: %u.\n", static_cast<uint8_t>(reason));

  if (WiFi.isConnected()) {
    reconnectMqtt = true;
    lastReconnect = millis();
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void networkingTask() {
  for (;;) {
    mqttClient.loop();
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  // Move all dynamic allocations >512byte to psram (if available)
  heap_caps_malloc_extmem_enable(512);
  WiFi.onEvent(WiFiEvent);
  wm.setWiFiAutoReconnect(true);
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setConfigPortalTimeout(600);
  prefs.begin("mqtt_config", false);

  WiFiManagerParameter custom_nodeID("nodeID", "nodeID", String(nodeID).c_str(), 4);
  WiFiManagerParameter custom_networkID("networkID", "networkID", String(networkID).c_str(), 4);
  WiFiManagerParameter custom_encryptKey("encryptKey", "encryptKey", encryptKey, 17);
  WiFiManagerParameter custom_mqttServer("mqttServer", "mqttServer", mqttServer, 40);
  WiFiManagerParameter custom_mqttPort("mqttPort", "mqttPort", String(mqttPort).c_str(), 6);
  WiFiManagerParameter custom_mqttServerCA("mqttServerCA", "mqttServerCA", mqttServerCA, 2048);  
  WiFiManagerParameter custom_mqttClientCert("mqttClientCert", "mqttClientCert", mqttClientCert, 2048);
  WiFiManagerParameter custom_mqttClientKey("mqttClientKey", "mqttClientKey", mqttClientKey, 2500);  
  
  wm.addParameter(&custom_nodeID);
  wm.addParameter(&custom_networkID);
  wm.addParameter(&custom_encryptKey);
  wm.addParameter(&custom_mqttServer);
  wm.addParameter(&custom_mqttPort);
  wm.addParameter(&custom_mqttServerCA);
  wm.addParameter(&custom_mqttClientCert);
  wm.addParameter(&custom_mqttClientKey);
  
  //wm.resetSettings();
  bool res = wm.autoConnect("AutoConnectAP","password");
  if(!res) {
    Serial.println("Failed to connect");
    delay(3000);
    ESP.restart();
  } 
  else {
    //if you get here you have connected to the WiFi    
    Serial.println("connected...yeey :)");
    if (shouldSaveConfig) {  
      nodeID = atoi(custom_nodeID.getValue());
      networkID = atoi(custom_networkID.getValue());
      strcpy(encryptKey, custom_encryptKey.getValue());
      strcpy(mqttServer, custom_mqttServer.getValue());
      mqttPort = atoi(custom_mqttPort.getValue());
    
      size_t outlen;
      mbedtls_base64_decode((unsigned char*)mqttServerCA, 2048, &outlen, (const unsigned char*) custom_mqttServerCA.getValue(), strlen(custom_mqttServerCA.getValue()));
      mbedtls_base64_decode((unsigned char*)mqttClientCert, 2048, &outlen, (const unsigned char*) custom_mqttClientCert.getValue(), strlen(custom_mqttClientCert.getValue()));
      mbedtls_base64_decode((unsigned char*)mqttClientKey, 2500, &outlen, (const unsigned char*) custom_mqttClientKey.getValue(), strlen(custom_mqttClientKey.getValue()));
    
      prefs.putInt("nodeID", nodeID);
      prefs.putInt("networkID", networkID);
      prefs.putString("encryptKey", encryptKey);
      prefs.putString("mqttServer", mqttServer);
      prefs.putInt("mqttPort", mqttPort);
      prefs.putString("mqttServerCA", mqttServerCA);
      prefs.putString("mqttClientCert", mqttClientCert);
      prefs.putString("mqttClientKey", mqttClientKey);
    }
    
    radio.initialize(FREQUENCY, prefs.getInt("nodeID"),prefs.getInt("networkID"));
    #ifdef IS_RFM69HW_HCW
      radio.setHighPower(); //must include this only for RFM69HW/HCW!
    #endif
    radio.encrypt(prefs.getString("encryptKey").c_str());
    radio.spyMode(spy);
    
    prefs.getString("mqttServerCA").toCharArray(mqttServerCA,2048);
    prefs.getString("mqttClientCert").toCharArray(mqttClientCert,2048);
    prefs.getString("mqttClientKey").toCharArray(mqttClientKey,2048);
    //client.setServer(prefs.getString("mqttServer").c_str(), prefs.getUInt("mqttPort"));
    //client.setServer("192.168.178.3",8883);
    mqttClient.setCACert(mqttServerCA);
    mqttClient.setCertificate(mqttClientCert);
    mqttClient.setPrivateKey(mqttClientKey);
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    // FIX!!!
    mqttClient.setServer("192.168.178.3",8883);
    mqttClient.setCleanSession(true);
    xTaskCreatePinnedToCore((TaskFunction_t)networkingTask, "mqttclienttask", 5120, nullptr, 1, &taskHandle, 0);
    mqttClient.connect();
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable WDT
    esp_task_wdt_add(NULL); //add current thread to WDT watch
  }
}

uint32_t counter = 0;
void loop() {
  if (radio.receiveDone()){
    if(radio.DATALEN == sizeof(sensorData)){
      memcpy(&sensorData, radio.DATA, sizeof(SensorData));  
      Serial.print("ID: ");
      Serial.println(sensorData.id);
      Serial.print("Temperature: ");
      Serial.println(float(sensorData.temperature) / 100.0);
      Serial.print("Humidity: ");
      Serial.println(sensorData.humidity);
      Serial.print("Battery: ");
      Serial.println(float(sensorData.batttery_voltage) / 100.0);
      Serial.print("Counter: ");
      Serial.println(counter);
      snprintf(json_string, sizeof(json_string), 
        "{\"id\":%d,\"sensor_type\":%d,\"temperature\":%.2f,\"humidity\":%d,\"battery_voltage\":%.2f}",
        sensorData.id, sensorData.sensortype, float(sensorData.temperature) / 100.0, sensorData.humidity, float(sensorData.batttery_voltage) / 100.0);
      mqttClient.publish("outTopic",0,false,json_string);
      counter++;
      esp_task_wdt_reset();
    }
  }
}   

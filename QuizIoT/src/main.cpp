#include <Arduino.h>
#include <DHTesp.h>
#include <BH1750.h>
#include <Wire.h>
#include <Wifi.h>
#include <PubSubClient.h>
#include <Ticker.h>

#define DHT_PIN 23
#define PIN_SCL 22
#define PIN_SDA 21
#define LED_RED 15
#define LED_GREEN 2
#define LED_ORANGE 4

#define WIFI_SSID "fAnus"
#define WIFI_PASSWORD "makancuy"

#define MQTT_BROKER  "broker.emqx.io"
#define MQTT_TOPIC_PUBLISH_TEMPERATURE "esp32_Stefanus/temperature"
#define MQTT_TOPIC_PUBLISH_HUMIDITY "esp32_Stefanus/humidity"
#define MQTT_TOPIC_PUBLISH_LIGHT "esp32_Stefanus/light"  

Ticker timerPublishHumidity;
Ticker timerPublishTemperature;
DHTesp dht;
BH1750 lightmeter;
WiFiClient espClient;
PubSubClient mqtt(espClient);
boolean mqttConnect();
void WifiConnect();

char g_szDeviceId[30];
float humidity;
float temperature;
float lux;

void WifiConnect()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) 
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }  
  Serial.print("System connected with IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("RSSI: %d\n", WiFi.RSSI());
}
void taskReadTemperatureAndHumidity(void *pvParameters) {
  while (true) 
  {
    float temperature_temp = temperature;
    float humidity_temp = humidity;
    humidity = dht.getHumidity();
    temperature = dht.getTemperature();
    if (isnan(humidity) || isnan(temperature)) {
      humidity = humidity_temp;
      temperature = temperature_temp;
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void publishTemperature() {
    char szMsg[50];
    sprintf(szMsg, "Temperature: %.2f C", temperature);
    mqtt.publish(MQTT_TOPIC_PUBLISH_TEMPERATURE, szMsg);
}

void PublishHumidity() {
    char szMsg[50];
    sprintf(szMsg, "Humidity: %.2f %%", humidity);
    mqtt.publish(MQTT_TOPIC_PUBLISH_HUMIDITY, szMsg);
}

void taskPublishLight(void *pvParameters) {
  while (true) 
  {
    lux = lightmeter.readLightLevel();
    char szMsg[50];
    sprintf(szMsg, "Light: %.2f lux", lux);
    mqtt.publish(MQTT_TOPIC_PUBLISH_LIGHT, szMsg);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
  }
}

void taskTemperatureAndHumidityCheck(void *pvParameters) {
  while (true) 
  {
    if(temperature > 28 && humidity > 80) {
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_ORANGE, LOW);
      digitalWrite(LED_GREEN, LOW);
    } else 
    if(temperature > 28 && (humidity < 80 && humidity > 60)) {
      digitalWrite(LED_ORANGE, HIGH);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_ORANGE, LOW);
    } else 
    {
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_ORANGE, LOW);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void taskLightCheck(void *pvParameters) {
  while (true) {
    if(lux > 400) 
    {
      Serial.printf("WARNING: Door Opened; Light: %.2f lux\n", lux);
    } else 
    {
      Serial.printf("Door Closed; Light: %.2f lux\n", lux);
    }
    vTaskDelay(4000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_ORANGE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  dht.setup(DHT_PIN, DHTesp::DHT11);
  Wire.begin(PIN_SDA, PIN_SCL);
  lightmeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);

  WifiConnect();
  mqttConnect();

  xTaskCreatePinnedToCore(taskReadTemperatureAndHumidity, "taskReadTemperatureAndHumidity", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskPublishLight, "taskPublishLight", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskTemperatureAndHumidityCheck, "taskTemperatureAndHumidityCheck", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskLightCheck, "taskLightCheck", 2048, NULL, 1, NULL, 1);

  timerPublishTemperature.attach_ms(5000, publishTemperature);
  timerPublishHumidity.attach_ms(6000, PublishHumidity);
}

void loop() {
  mqtt.loop();
}

boolean mqttConnect() {
  sprintf(g_szDeviceId, "esp32_%08X",(uint32_t)ESP.getEfuseMac());
  mqtt.setServer(MQTT_BROKER, 1883);
  Serial.printf("Connecting to %s clientId: %s\n", MQTT_BROKER, g_szDeviceId);

  boolean fMqttConnected = false;
  for (int i = 0; i < 3 && !fMqttConnected; i++) 
  {
    Serial.print("Connecting to MQTT Broker...");
    fMqttConnected = mqtt.connect(g_szDeviceId);
    if (fMqttConnected == false) {
      Serial.print(" failed, Reconnect=");
      Serial.println(mqtt.state());
      delay(1000);
    }
  }

  if (fMqttConnected)
  {
    Serial.println("SUCCESS");
  }
  return mqtt.connected();
}
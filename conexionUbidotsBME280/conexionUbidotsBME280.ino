#include <WiFi.h>
#include "UbidotsEsp32Mqtt.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

/* Configuración del sensor BME280 */
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

/* Credenciales de WiFi y Ubidots */
const char *WIFI_SSID = "DAYSI";
const char *WIFI_PASS = "1106042474001";
const char *UBIDOTS_TOKEN = "BBUS-PJLFSJ9RRDy1Znh1uvr6jWM6l28svo";

/* Etiquetas para el dispositivo y las variables en Ubidots */
const char *DEVICE_LABEL = "esp32-bme280";
const char *VARIABLE_LABEL_TEMP = "temperatura";
const char *VARIABLE_LABEL_HUMIDITY = "humedad";
const char *VARIABLE_LABEL_PRESSURE = "presion";
const char *VARIABLE_LABEL_ALTITUDE = "altitud";
const char *VARIABLE_LABEL_LED = "led";

/* Configuración del LED */
const int LED_PIN = 4;

/* Frecuencia de publicación */
const int PUBLISH_FREQUENCY = 10000;  // Cada 10 segundos
unsigned long lastPublishTime = 0;

/* Inicializa Ubidots */
Ubidots ubidots(UBIDOTS_TOKEN);

/* Función callback para procesar mensajes entrantes */
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("]: ");
  
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);

  // Controla el LED según el valor recibido
  if (message == "1.0") {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("LED ENCENDIDO");
  } else if (message == "0.0") {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LED APAGADO");
  }
}

void setup() {
  // Inicializa el monitor serial
  Serial.begin(115200);
  Serial.println("Iniciando sistema...");

  // Configura el pin del LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Inicializa el sensor BME280
  if (!bme.begin(0x76)) {
    Serial.println("No se encontró un sensor válido, revisa el cableado.");
    while (1);
  }

  // Conexión WiFi
  Serial.println("Conectando a WiFi...");
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);

  // Configuración de Ubidots
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL_LED); // Suscribirse a la variable LED
}

void loop() {
  // Reconexión si es necesario
  if (!ubidots.connected()) {
    ubidots.reconnect();
    ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL_LED);
  }

  // Publicar datos del sensor
  if ((millis() - lastPublishTime) > PUBLISH_FREQUENCY) {
    ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL_LED);
    lastPublishTime = millis();

    // Lee los valores del sensor
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure();
    float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    if (isnan(temperature) || isnan(humidity) || isnan(pressure) || isnan(altitude)) {
      Serial.println("Error al leer los datos del sensor BME280.");
      return;
    }

    // Imprime los valores en el monitor serial
    Serial.println("----- Datos del sensor -----");
    Serial.print("Temperatura: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Humedad: ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Presión: ");
    Serial.print(pressure / 100.0F);
    Serial.println(" hPa");

    Serial.print("Altitud: ");
    Serial.print(altitude);
    Serial.println(" m");
    Serial.println("---------------------------");

    // Publica los datos en Ubidots
    ubidots.add(VARIABLE_LABEL_TEMP, temperature);
    ubidots.add(VARIABLE_LABEL_HUMIDITY, humidity);
    ubidots.add(VARIABLE_LABEL_PRESSURE, pressure / 100.0F);
    ubidots.add(VARIABLE_LABEL_ALTITUDE, altitude);
    bool published = ubidots.publish(DEVICE_LABEL);

    if (published) {
      Serial.println("Datos publicados en Ubidots.");
    } else {
      Serial.println("Error al publicar en Ubidots.");
    }
  }

  // Procesar mensajes entrantes
  ubidots.loop();
}

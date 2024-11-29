#include "UbidotsEsp32Mqtt.h"
#include "DHT.h"

/* Definimos el pin de datos del sensor y el tipo de sensor */
#define DHTPIN 23
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);

/* Credenciales para Ubidots y WiFi */
const char *UBIDOTS_TOKEN = "BBUS-PJLFSJ9RRDy1Znh1uvr6jWM6l28svo";
const char *WIFI_SSID = "DAYSI";
const char *WIFI_PASS = "1106042474001";

/* Etiquetas para el dispositivo y las variables en Ubidots */
const char *DEVICE_LABEL = "esp32-dht11";
const char *VARIABLE_LABEL_1 = "temperatura";
const char *VARIABLE_LABEL_2 = "humedad";

/* Configuración de tiempo */
const int PUBLISH_FREQUENCY = 5000; // 5 segundos
unsigned long lastPublishTime = 0;
Ubidots ubidots(UBIDOTS_TOKEN);

/* Función callback para mensajes de retorno desde Ubidots */
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Mensaje recibido [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup() {
  // Inicializa el monitor serial
  Serial.begin(115200);
  Serial.println("Iniciando sistema...");
  dht.begin();
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);

  // Configura Ubidots
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
}

void loop() {
  // Reintenta conexión a Ubidots si es necesario
  if (!ubidots.connected()) {
    ubidots.reconnect();
  }

  if ((millis() - lastPublishTime) > PUBLISH_FREQUENCY) {
    lastPublishTime = millis();

    // Lee los valores del sensor
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Error al leer los datos del sensor DHT11");
      return;
    }

    Serial.println("----- Lectura del sensor DHT11 -----");
    Serial.print("Temperatura: ");
    Serial.print(temperature);
    Serial.println(" °C");
    Serial.print("Humedad: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.println("------------------------------------");

    // Publica los datos en Ubidots
    ubidots.add(VARIABLE_LABEL_1, temperature);
    ubidots.add(VARIABLE_LABEL_2, humidity);
    bool published = ubidots.publish(DEVICE_LABEL);

    if (published) {
      Serial.println("Datos publicados en Ubidots.");
    } else {
      Serial.println("Error al publicar en Ubidots.");
    }
  }

  ubidots.loop();
}
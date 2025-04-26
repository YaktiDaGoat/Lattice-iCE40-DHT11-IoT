#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

// UART pins from your FPGA
#define RX_PIN 16
#define TX_PIN 17

// Your Wi-Fi credentials
constexpr char WIFI_SSID[]     = "Imah Janu";
constexpr char WIFI_PASSWORD[] = "JimiDebbie";

// ThingsBoard Cloud (EU) settings
constexpr char TOKEN[]               = "razYl28Ayv9fhs0YBOob";
constexpr char THINGSBOARD_SERVER[]  = "eu.thingsboard.cloud";
constexpr uint16_t THINGSBOARD_PORT  = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE  = 1024U;

// Number of DHT11 sensors feeding this ESP32
constexpr uint8_t NUM_SENSORS = 3;

// Underlying network & MQTT client
WiFiClient           wifiClient;
Arduino_MQTT_Client  mqttClient(wifiClient);

// ThingsBoard client
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

// 4-byte buffer: [0]=H_int, [1]=H_dec, [2]=T_int, [3]=T_dec
uint8_t buf[4];
uint8_t bufIndex = 0;

// Which sensor we’re on (1..3)
uint8_t sensorIndex = 1;

// Bring up Wi-Fi (blocks until connected)
void InitWiFi() {
  Serial.printf("Connecting to WiFi \"%s\"...", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());
}

// Reconnect Wi-Fi if needed
bool reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  InitWiFi();
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // UART1 @9600 to receive from FPGA
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  // Start Wi-Fi
  InitWiFi();
}

void loop() {
  // 1) Ensure Wi-Fi
  if (!reconnectWiFi()) return;

  // 2) Ensure ThingsBoard MQTT connection
  if (!tb.connected()) {
    Serial.printf("Connecting to TB [%s] with token [%s]...\n",
                  THINGSBOARD_SERVER, TOKEN);
    if (tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("→ Connected to ThingsBoard Cloud (EU)");
    } else {
      Serial.println("!!! TB connect failed, retry in 5 s");
      delay(5000);
      return;
    }
  }
  tb.loop();

  // 3) Read & buffer incoming bytes
  while (Serial1.available()) {
    buf[bufIndex++] = Serial1.read();

    // Once we have 4 bytes, parse & send
    if (bufIndex == 4) {
      uint8_t h_int = buf[0], h_dec = buf[1];
      uint8_t t_int = buf[2], t_dec = buf[3];

      // Optionally convert to float
      float humidity    = h_int + (h_dec / 100.0f);
      float temperature = t_int + (t_dec / 100.0f);

      Serial.printf("Sensor %u → Humidity: %u.%02u  Temperature: %u.%02u\n",
                    sensorIndex, h_int, h_dec, t_int, t_dec);

      // Format telemetry keys
      char keyHum[16], keyTemp[16];
      snprintf(keyHum,   sizeof(keyHum),   "humidity%u",    sensorIndex);
      snprintf(keyTemp,  sizeof(keyTemp),  "temperature%u", sensorIndex);

      // Send telemetry
      tb.sendTelemetryData(keyHum,   humidity);
      tb.sendTelemetryData(keyTemp,  temperature);

      // advance to next sensor, wrap around after 3
      if (++sensorIndex > NUM_SENSORS) {
        sensorIndex = 1;
      }

      // reset buffer for next quartet
      bufIndex = 0;
    }
  }

  // throttle loop
  delay(50);
}

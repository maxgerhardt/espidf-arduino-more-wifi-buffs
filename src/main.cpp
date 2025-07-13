#include <Arduino.h>
#include <WiFi.h>
#include <esp_idf_version.h>
#include <esp_wifi.h>
#include <lwip/sockets.h>
#include <WebSocketsClient.h>
#include <AsyncUDP.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include <ESP32Servo.h>

// ==== WiFi ====
const char *ssid = "my wifi";
const char *password = "my wifi password";

// ==== WebSocket ====
WebSocketsClient webSocket;
const char *websocket_host = "192.168.0.112";
const uint16_t websocket_port = 8000;
const char *websocket_path = "/esp32";

// ===== UDP ====
#define THROTTLE_PERIOD 30 // Throttles the sent FPS. 80ms approx. => 12 fps
#define UDP_PORT 4210
IPAddress serverIP(192, 168, 0, 125);
WiFiUDP udp;
uint32_t user_id = 0xDEADBEEF; // Hardcoded user ID for testing
uint8_t frame_id = 0;
const size_t CHUNK_SIZE = 128 * 8;
uint8_t buffer[CHUNK_SIZE + 12];

// ==== Camera ====
camera_fb_t *fb;
uint16_t frameAcqAndTransTime = 0;
volatile int frameCount = 0;
#define FRAME_SIZE FRAMESIZE_VGA // VGA = 640x320px
#define JPEG_QUALITY 8           // Max of 63. Lower number = less compression = bigger size
#define MAX_ATTEMPTS 4           // Abandon frame if a single packet fails MAX_ATTEMPTS times.
uint8_t attempts = 0;
bool sent = false;

// ==== Servos ====
Servo srvA;
bool srvAState;
uint8_t srvACount = 0;
unsigned long srvALastAction = 0;

Servo srvB;
bool srvBState;
uint8_t srvBCount = 0;
unsigned long srvBLastAction = 0;

Servo srvC;
bool srvCState;
uint8_t srvCCount = 0;
unsigned long srvCLastAction = 0;

#define SERVO_DOWN 120
#define SERVO_UP 160       // 180 is true home, 165 seems like a good approximate home
#define SERVO_UP_MINOR 145 // 180 is true home, 165 seems like a good approximate home
#define SERVO_WAIT_TIME 140

// ==== Misc ====
#define LEDs 48

uint16_t avgTransmissionTime = 0;
unsigned long lastLogTime = 0;
uint8_t numDropped = 0;
uint8_t numRetry = 0;
unsigned long debugTime = 0;

// ==== WiFi Setup ====
void startWiFi()
{
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(250);
    Serial.print(".");
  }
  Serial.println("[WiFi Connected]");

  // esp_wifi_set_max_tx_power(78);  // Max power
  // esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  esp_log_level_set("wifi", ESP_LOG_VERBOSE);
  esp_log_level_set("wifi", ESP_LOG_DEBUG);
}

// ==== Servos Setup ====
void startServos()
{
  srvA.setPeriodHertz(50);
  srvA.attach(41, 500, 2400);
  srvA.write(SERVO_UP);
  srvAState = true;

  srvB.setPeriodHertz(50);
  srvB.attach(42, 500, 2400);
  srvB.write(SERVO_UP);
  srvBState = true;

  srvC.setPeriodHertz(50);
  srvC.attach(1, 500, 2400);
  srvC.write(SERVO_UP);
  srvCState = true;
}

// ==== Camera Setup ====
void startCamera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20 * 1000000;
  config.frame_size = FRAME_SIZE;
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = JPEG_QUALITY;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 2;
  esp_camera_init(&config);

  pinMode(LEDs, OUTPUT);
  digitalWrite(LEDs, HIGH);
}

// ==== WebSocket Client Setup ====
void startWebSocketClient()
{
  webSocket.begin(websocket_host, websocket_port, websocket_path, "arduino");

  webSocket.onEvent([](WStype_t type, uint8_t *payload, size_t length)
                    {
    switch (type) {
      case WStype_CONNECTED:
        Serial.println("WebSocket connected");
        break;
      case WStype_DISCONNECTED:
        Serial.println("WebSocket disconnected");
        break;
      case WStype_ERROR:
        Serial.println("WebSocket error");
        break;
      case WStype_BIN:
        if (payload[0] == 'A') {
          if (payload[1] == '0')
            srvACount = 0;
          else
            srvACount++;
        }

        if (payload[0] == 'B') {
          if (payload[1] == '0')
            srvBCount = 0;
          else
            srvBCount++;
        }

        if (payload[0] == 'C') {
          if (payload[1] == '0')
            srvCCount = 0;
          else
            srvCCount++;
        }
        break;
      default:
        break;
    } });

  webSocket.setReconnectInterval(2000);
}

void handleServoActions()
{
  if ((srvACount || !srvAState) && (millis() >= (srvALastAction + SERVO_WAIT_TIME)))
  {
    if (srvAState)
    {
      srvA.write(SERVO_DOWN);
    }
    else
    {
      srvA.write(srvACount > 3 ? SERVO_UP_MINOR : SERVO_UP);
      srvACount--;
    }
    srvAState = !srvAState;
    srvALastAction = millis();
  }

  if ((srvBCount || !srvBState) && (millis() >= (srvBLastAction + SERVO_WAIT_TIME)))
  {
    if (srvBState)
    {
      srvB.write(SERVO_DOWN);
    }
    else
    {
      srvB.write(SERVO_UP);
      srvBCount--;
    }
    srvBState = !srvBState;
    srvBLastAction = millis();
  }

  if ((srvCCount || !srvCState) && (millis() >= (srvCLastAction + SERVO_WAIT_TIME)))
  {
    if (srvCState)
    {
      srvC.write(SERVO_DOWN);
    }
    else
    {
      srvC.write(SERVO_UP);
      srvCCount--;
    }
    srvCState = !srvCState;
    srvCLastAction = millis();
  }
}

void instructionsTask(void *pvParameters)
{
  for (;;)
  {
    handleServoActions();
    vTaskDelay(3); // This task generally takes less than 0.1ms, so safe to do frequently.
  }
}

void webSocketsClientTask(void *pvParameters)
{
  for (;;)
  {
    webSocket.loop();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void sendFrameUDP()
{
  size_t total_size = fb->len;
  size_t packet_count = (total_size + CHUNK_SIZE - 1) / CHUNK_SIZE;

  for (uint8_t packet_id = 0; packet_id < packet_count; packet_id++)
  {
    size_t offset = packet_id * CHUNK_SIZE;
    size_t this_size = min(CHUNK_SIZE, total_size - offset);

    // Fill header
    buffer[0] = (user_id >> 24) & 0xFF;
    buffer[1] = (user_id >> 16) & 0xFF;
    buffer[2] = (user_id >> 8) & 0xFF;
    buffer[3] = user_id & 0xFF;

    buffer[4] = (frame_id >> 8) & 0xFF;
    buffer[5] = frame_id & 0xFF;

    buffer[6] = packet_id;
    buffer[7] = packet_count;

    buffer[8] = (this_size >> 8) & 0xFF;
    buffer[9] = this_size & 0xFF;

    buffer[10] = 0; // CRC placeholder (optional)
    buffer[11] = 0;

    memcpy(buffer + 12, fb->buf + offset, this_size);

    sent = false;
    attempts = 0;
    while (!sent && (attempts <= MAX_ATTEMPTS))
    {
      udp.beginPacket(serverIP, UDP_PORT);
      udp.write(buffer, this_size + 12);
      if (udp.endPacket() == 1)
      {
        sent = true;
      }
      else
      {
        attempts++;
        vTaskDelay(2);
      }
    }

    if (!sent)
    {
      numDropped++;
      vTaskDelay(pdMS_TO_TICKS(THROTTLE_PERIOD * 4)); // Back-off for a frames worth of time.
      return;                                         // Cancel this frame if one packet won't.
    }
  }

  frame_id++; // Increment for next frame
}

void cameraTask(void *pvParameters)
{
  for (;;)
  {
    frameAcqAndTransTime = millis();
    fb = esp_camera_fb_get();
    if (fb)
    {
      sendFrameUDP();
      esp_camera_fb_return(fb);
      frameCount++;

      frameAcqAndTransTime = millis() - frameAcqAndTransTime;
      avgTransmissionTime += frameAcqAndTransTime;
    }

    if (millis() - lastLogTime >= 1000)
    {
      debugTime = ((float)ESP.getMinFreeHeap() / (float)ESP.getHeapSize()) * 100;
      Serial.printf("[FPS: %d] [TT: %d] [ND: %d] [NR: %d] | [MF: %lu]\n", frameCount, avgTransmissionTime / frameCount, numDropped, numRetry, debugTime);
      frameCount = 0;
      numDropped = 0;
      numRetry = 0;
      avgTransmissionTime = 0;

      // printHeapInfo("Debug");

      lastLogTime = millis();
    }

    // Back off for at least 10ms, up to the minmum trans period.
    uint16_t t = max((THROTTLE_PERIOD - frameAcqAndTransTime), 10);
    vTaskDelay(pdMS_TO_TICKS(t));
  }
}

void printHeapInfo(const char *tag)
{
  Serial.printf("[%s] Current free heap:   %d bytes\n", tag, ESP.getFreeHeap());
  Serial.printf("[%s] Min free since boot: %d bytes\n", tag, ESP.getMinFreeHeap());
  // Serial.printf("[%s] Max alloc'able block: %d bytes\n", tag, ESP.getMaxAllocHeap());
  Serial.printf("[%s] Heap fragmentation:  %d%%\n", tag, 100 - (ESP.getMaxAllocHeap() * 100) / ESP.getFreeHeap());
}

void setup()
{
  Serial.begin(115200);

  uint32_t heapTotal = ESP.getHeapSize();
  uint32_t heapFree = ESP.getFreeHeap();
  float heapUsedPercent = 100.0 * (heapTotal - heapFree) / heapTotal;

  Serial.printf("Internal Heap: %u / %u bytes used (%.2f%%)\n",
                heapTotal - heapFree, heapTotal, heapUsedPercent);

  startWiFi();
  vTaskDelay(pdMS_TO_TICKS(1000));
  startCamera();
  startServos();
  vTaskDelay(pdMS_TO_TICKS(500));
  // startWebSocketClient();
  // vTaskDelay(pdMS_TO_TICKS(100));

  // "Prime" the camera by pulling the first few frames that usually suck.
  for (uint8_t i = 0; i < 10; i++)
  {
    fb = esp_camera_fb_get();
    esp_camera_fb_return(fb);
    delay(20);
  }

  xTaskCreatePinnedToCore(cameraTask, "CameraTask", 1024 * 24, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(instructionsTask, "ServoTask", 1024, NULL, 1, NULL, 1);

  // Turned off for debugging
  // xTaskCreatePinnedToCore(webSocketsClientTask, "WebSocketsClientTask", 1024 * 4, NULL, 2, NULL, 0);

  heapTotal = ESP.getHeapSize();
  heapFree = ESP.getFreeHeap();
  heapUsedPercent = 100.0 * (heapTotal - heapFree) / heapTotal;

  Serial.printf("Internal Heap: %u / %u bytes used (%.2f%%)\n",
                heapTotal - heapFree, heapTotal, heapUsedPercent);

  vTaskDelete(NULL); // Effectively removes the void loop.
}

void loop() {}
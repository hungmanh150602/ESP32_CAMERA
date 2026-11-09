#include <WebServer.h>
#include <WiFi.h>
#include "esp_camera.h"

/* ==== WIFI ==== */
// const char *ssid = "Manh Hung";
// const char *password = "a12345678";
const char *ssid = "THEGIOIBAOLA.";
const char *password = "thegioibaola";

/* ==== CAMERA PINS (AI Thinker) ==== */
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

/* ==== WEB SERVER ==== */
WebServer server(80);

/* ==== STREAM SERVER ==== */
// WiFiServer streamServer(81);

void handleCapture()
{
  /* Capture one JPEG frame from camera. */
  camera_fb_t *fb = esp_camera_fb_get();

  if (fb == NULL)
  {
    server.send(503, "text/plain", "Camera capture failed");
    return;
  }

  /* Get the TCP client associated with the HTTP request. */
  WiFiClient client = server.client();

  /* Send HTTP response headers. */
  client.printf("HTTP/1.1 200 OK\r\n"
                "Content-Type: image/jpeg\r\n"
                "Content-Length: %u\r\n"
                "Connection: close\r\n"
                "\r\n",
                (unsigned int)fb->len);

  /* Send JPEG binary data. */
  size_t sent = client.write(fb->buf, fb->len);

  /* Release camera frame buffer. */
  esp_camera_fb_return(fb);

  /* Check transmission result. */
  if (sent != fb->len)
  {
    Serial.println("JPEG transmission incomplete");
  }
}

/* ===== STREAM MJPEG ===== */
// void streamTask()
// {
//   WiFiClient client = streamServer.available();
//   if (!client)
//     return;
//
//   Serial.println("MJPEG Stream connected!");
//
//   client.println("HTTP/1.1 200 OK");
//   client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
//   client.println();
//
//   while (client.connected())
//   {
//     camera_fb_t *fb = esp_camera_fb_get();
//     if (!fb)
//       continue;
//
//     client.printf("--frame\r\n");
//     client.printf("Content-Type: image/jpeg\r\n");
//     client.printf("Content-Length: %d\r\n\r\n", fb->len);
//     client.write(fb->buf, fb->len);
//     client.printf("\r\n");
//
//     esp_camera_fb_return(fb);
//
//     delay(30); // FPS ~ 30
//   }
//
//   Serial.println("MJPEG Stream disconnected!");
// }

/* ===== SETUP ===== */
void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  /* ==== Camera Config ==== */
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QVGA; // 320x240
  config.jpeg_quality = 12;
  config.fb_count = 2;

  /* ==== Start Camera ==== */
  if (esp_camera_init(&config) != ESP_OK)
  {
    Serial.println("Camera init failed");
    return;
  }

  /* ==== WiFi ==== */
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.on("/capture", HTTP_GET, handleCapture);
  server.begin();

  Serial.print("Capture: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":80/capture");

  // streamServer.begin();

  // Serial.print("MJPEG Stream: http://");
  // Serial.print(WiFi.localIP());
  // Serial.println(":81/stream");
}

/* ===== LOOP ===== */
void loop()
{
  server.handleClient();
  // streamTask();
}

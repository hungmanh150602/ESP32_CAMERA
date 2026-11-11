#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdint.h>
#include <string.h>

/*
 * This ESP32-CAM joins the configured Wi-Fi network in station mode. Replace
 * both placeholders with the credentials for the network shared with the
 * Raspberry Pi before uploading the sketch.
 */
const char *WIFI_SSID = "Manh Hung";
const char *WIFI_PASSWORD = "a12345678";

/*
 * The ESP32 binds this UDP port and waits for control datagrams from the Pi.
 * The Pi does not need a fixed IP configured in this sketch: when a valid
 * START_STREAM command arrives, the ESP32 learns the sender's IP address and
 * source port from that datagram and sends video back to that endpoint.
 */
#define ESP32_CONTROL_PORT 5432

/*
 * START_STREAM also acts as a heartbeat. If no valid command is received for
 * this many milliseconds, the ESP32 stops sending frames until another
 * command arrives.
 */
#define CONTROL_TIMEOUT_MS 5000

/*
 * GPIO assignments for the AI Thinker ESP32-CAM board and its OV2640 camera.
 * The camera driver uses these values to connect its data and control
 * signals. They are specific to this board's wiring.
 */
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

/*
 * Each UDP datagram contains a fixed-size protocol header followed by one
 * portion of JPEG data. Keeping the total datagram below 1200 bytes reduces
 * the chance of IP fragmentation on typical Wi-Fi networks.
 */
#define UDP_DATAGRAM_SIZE 1200
#define UDP_HEADER_SIZE 24
#define UDP_PAYLOAD_SIZE (UDP_DATAGRAM_SIZE - UDP_HEADER_SIZE)

/*
 * Reject unusually large frames before dividing them into packets. The
 * packet count is stored in a 16-bit field in the protocol header.
 */
#define MAX_FRAME_SIZE (2U * 1024U * 1024U)

#define PROTOCOL_VERSION 1

/*
 * The 24-byte packet header uses network byte order for all multi-byte
 * numbers. Its fields are: bytes 0-3, "MJPG" magic; byte 4, protocol version;
 * byte 5, packet flags; bytes 6-7, header length; bytes 8-11, frame ID;
 * bytes 12-13, packet index; bytes 14-15, packet count; bytes 16-19, full
 * JPEG frame size; bytes 20-21, payload size; and bytes 22-23, reserved.
 */
#define FLAG_FIRST 0x01
#define FLAG_LAST 0x02

/*
 * udp is used both to receive Pi control commands and to send JPEG packets.
 * streaming records whether a recent START_STREAM heartbeat was received.
 * piAddress and piPort hold the most recently observed sender endpoint.
 * frameId identifies each captured JPEG frame; lastControlMs tracks when the
 * most recent valid heartbeat arrived.
 */
WiFiUDP udp;

bool streaming = false;

IPAddress piAddress;
uint16_t piPort = 0;

uint32_t frameId = 0;
uint32_t lastControlMs = 0;

/*
 * Store a 16-bit value in network byte order (most significant byte first),
 * so packet headers are interpreted consistently by the Raspberry Pi.
 */
void write_u16_be(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = value & 0xFF;
}

/*
 * Store a 32-bit value in network byte order (most significant byte first).
 * This is used for frame identifiers and complete JPEG frame lengths.
 */
void write_u32_be(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (value >> 24) & 0xFF;
    buffer[1] = (value >> 16) & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = value & 0xFF;
}

/*
 * Join the configured wireless network as a station. The function waits up
 * to 20 seconds for association, prints the assigned local IP on success,
 * and returns false on timeout so the caller can decide how to recover.
 */
bool connect_wifi()
{
    Serial.println();
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long start = millis();

    /* Poll the connection status while periodically yielding to the Wi-Fi
     * stack. The elapsed-time check prevents startup from waiting forever.
     */
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");

        if (millis() - start > 10000)
        {
            Serial.println("\nWiFi connection timeout");
            return false;
        }
    }

    Serial.println();
    Serial.println("WiFi connected");

    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());

    return true;
}

/*
 * Configure the camera driver for the AI Thinker board and JPEG output.
 * QVGA resolution and two PSRAM frame buffers are selected for continuous
 * capture. Return false if the camera driver cannot be initialized.
 */
bool init_camera()
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

    /* Configure a 20 MHz camera clock and JPEG capture. A quality value of
     * 12 is passed to the camera driver; lower values generally mean higher
     * JPEG quality and larger frames.
     */
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    /* Use QVGA frames and two frame buffers in PSRAM. CAMERA_GRAB_LATEST
     * favors recent images rather than preserving an older capture backlog.
     */
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;

    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK)
    {
        Serial.printf(
            "Camera initialization failed: 0x%x\n",
            err);

        return false;
    }

    Serial.println("Camera initialized");

    return true;
}

/*
 * Drain all currently queued UDP control packets. Only the exact
 * START_STREAM command is accepted. For each accepted command, refresh the
 * heartbeat timestamp and learn the sender's IP address and UDP source port;
 * those values become the destination for outgoing video.
 */
void process_control_packets()
{
    int packetSize;

    while ((packetSize = udp.parsePacket()) > 0)
    {
        /* Reserve one byte for the null terminator so the datagram can be
         * compared safely as a C string. Longer commands are truncated and
         * therefore will not match the supported command.
         */
        char command[64] = {0};

        int bytesRead = udp.read(
            (uint8_t *)command,
            sizeof(command) - 1);

        if (bytesRead <= 0)
        {
            continue;
        }

        command[bytesRead] = '\0';

        /* Strip line endings commonly added by text-based UDP clients before
         * comparing the command with its expected spelling.
         */
        while (bytesRead > 0 &&
               (command[bytesRead - 1] == '\r' ||
                command[bytesRead - 1] == '\n'))
        {
            command[--bytesRead] = '\0';
        }

        if (strcmp(command, "START_STREAM") == 0)
        {
            /* remoteIP() and remotePort() refer to the current datagram's
             * sender. This discovery lets the Pi's address change without
             * hard-coding it in the ESP32 firmware.
             */
            piAddress = udp.remoteIP();
            piPort = udp.remotePort();

            lastControlMs = millis();

            if (!streaming)
            {
                /* Print the endpoint only when entering streaming mode,
                 * rather than on every periodic heartbeat.
                 */
                Serial.println("Received START_STREAM");
                Serial.print("Raspberry Pi IP: ");
                Serial.println(piAddress);

                Serial.print("Raspberry Pi UDP port: ");
                Serial.println(piPort);

                streaming = true;
            }
        }
    }
}

/*
 * Stop streaming when the Pi's heartbeat becomes stale. millis() subtraction
 * is used for elapsed-time comparison and remains valid across its rollover
 * for this short timeout interval.
 */
void check_stream_timeout()
{
    if (!streaming)
    {
        return;
    }

    if (millis() - lastControlMs > CONTROL_TIMEOUT_MS)
    {
        /* Clear the state so the main loop stops capturing and transmitting
         * until another valid START_STREAM command is received.
         */
        streaming = false;

        Serial.println("Control timeout");
        Serial.println("Stopping video stream");
    }
}

/*
 * Split one JPEG frame into bounded UDP datagrams and send them to the last
 * Pi endpoint learned from a START_STREAM command. Every packet carries the
 * same frame ID and frame size, plus its own index and payload length, so the
 * Pi can reassemble the frame and identify missing or out-of-order packets.
 */
void send_jpeg_frame(const uint8_t *jpegData, size_t jpegSize)
{
    /* Reject invalid input, oversized frames, or an endpoint that has not
     * been learned from a control command.
     */
    if (jpegData == nullptr || jpegSize == 0 ||
        jpegSize > MAX_FRAME_SIZE || piPort == 0)
    {
        return;
    }

    /* Round up the division so a partially filled final packet is counted.
     * The protocol allows no more than UINT16_MAX packets per frame.
     */
    uint16_t packetCount = (jpegSize + UDP_PAYLOAD_SIZE - 1) / UDP_PAYLOAD_SIZE;

    if (packetCount == 0 || packetCount > UINT16_MAX)
    {
        return;
    }

    uint32_t currentFrameId = frameId++;

    uint8_t packet[UDP_DATAGRAM_SIZE];

    /* Send packets sequentially. The Wi-Fi stack gets periodic opportunities
     * to process new commands and heartbeats while large frames are sent.
     */
    for (uint16_t packetIndex = 0; packetIndex < packetCount; packetIndex++)
    {
        /* Check control traffic periodically so streaming can be stopped
         * promptly, even while a frame spans many datagrams.
         */
        if ((packetIndex % 16) == 0)
        {
            process_control_packets();
            check_stream_timeout();

            if (!streaming)
            {
                return;
            }
        }

        size_t offset = (size_t)packetIndex * UDP_PAYLOAD_SIZE;

        size_t remaining = jpegSize - offset;

        uint16_t payloadSize = (remaining > UDP_PAYLOAD_SIZE)
                                   ? UDP_PAYLOAD_SIZE
                                   : (uint16_t)remaining;

        /* Construct the fixed header. FIRST and LAST mark frame boundaries;
         * all other metadata lets the receiver determine where this payload
         * belongs in the complete JPEG.
         */
        memcpy(packet, "MJPG", 4);

        packet[4] = PROTOCOL_VERSION;

        uint8_t flags = 0;

        if (packetIndex == 0)
        {
            flags |= FLAG_FIRST;
        }

        if (packetIndex == packetCount - 1)
        {
            flags |= FLAG_LAST;
        }

        packet[5] = flags;

        write_u16_be(packet + 6, UDP_HEADER_SIZE);
        write_u32_be(packet + 8, currentFrameId);

        write_u16_be(packet + 12, packetIndex);
        write_u16_be(packet + 14, packetCount);

        write_u32_be(packet + 16, jpegSize);

        write_u16_be(packet + 20, payloadSize);

        packet[22] = 0;
        packet[23] = 0;

        /* Copy only this packet's JPEG slice after the header. The payload
         * length is bounded by UDP_PAYLOAD_SIZE, including for the last
         * partially filled packet.
         */
        memcpy(packet + UDP_HEADER_SIZE, jpegData + offset, payloadSize);

        /* Send the complete header-plus-payload datagram to the learned Pi
         * endpoint. Stop the frame if packet creation or transmission fails.
         */
        if (!udp.beginPacket(piAddress, piPort))
        {
            Serial.println("UDP beginPacket failed");
            return;
        }

        udp.write(packet, UDP_HEADER_SIZE + payloadSize);

        if (!udp.endPacket())
        {
            Serial.println("UDP packet send failed");
            return;
        }

        if ((packetIndex % 16) == 0)
        {
            delay(0);
        }
    }
}

/*
 * Arduino startup routine: initialize serial diagnostics, connect to Wi-Fi,
 * initialize the camera, and bind the UDP control port. A failed Wi-Fi or
 * camera initialization restarts the ESP32; a failed UDP bind also triggers
 * a restart after reporting the error.
 */
void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("ESP32-CAM UDP MJPEG STREAMER");
    Serial.println("================================");

    /* Join the wireless network before starting the UDP listener. Restart
     * the device if the bounded connection attempt fails.
     */
    if (!connect_wifi())
    {
        ESP.restart();
    }

    /* The camera is required for streaming, so do not continue if its driver
     * cannot be initialized.
     */
    if (!init_camera())
    {
        ESP.restart();
    }

    /* Bind the local UDP port that receives START_STREAM heartbeats. The Pi
     * must send its control datagrams to this port.
     */
    if (!udp.begin(ESP32_CONTROL_PORT))
    {
        Serial.println("UDP initialization failed");
        ESP.restart();
    }

    Serial.printf("Listening for control commands on UDP port %u\n",
                  ESP32_CONTROL_PORT);

    Serial.println("Waiting for Raspberry Pi...");
}

/*
 * Main application loop. Maintain Wi-Fi connectivity, process Pi control
 * commands and their timeout, then capture and transmit frames only while a
 * recent START_STREAM heartbeat keeps streaming enabled.
 */
void loop()
{
    /* Re-establish the station connection if Wi-Fi drops. Stop streaming and
     * rebind the UDP socket after a successful reconnection; return now so
     * packet processing resumes on the next loop iteration.
     */
    if (WiFi.status() != WL_CONNECTED)
    {
        streaming = false;

        Serial.println("WiFi disconnected");

        udp.stop();

        if (connect_wifi())
        {
            if (!udp.begin(ESP32_CONTROL_PORT))
            {
                Serial.println("UDP restart failed");
            }
        }
        else
        {
            delay(1000);
        }

        return;
    }

    /* Handle pending START_STREAM commands before deciding whether to
     * capture another image.
     */
    process_control_packets();

    /* A missing heartbeat disables streaming before any new frame is
     * captured.
     */
    check_stream_timeout();

    /* Stay responsive to control traffic without consuming the processor
     * continuously while idle.
     */
    if (!streaming)
    {
        delay(2);
        return;
    }

    /* Capture one JPEG frame from the camera. On failure, report it and
     * briefly yield before retrying on the next loop iteration.
     */
    camera_fb_t *fb = esp_camera_fb_get();

    if (fb == nullptr)
    {
        Serial.println("Camera capture failed");
        delay(10);
        return;
    }

    /* Transmit the JPEG buffer in protocol-sized UDP packets. The frame
     * buffer remains owned by the camera driver until explicitly returned.
     */
    send_jpeg_frame(fb->buf, fb->len);

    /* Always return a successfully acquired frame buffer so the camera
     * driver can reuse it for a later capture.
     */
    esp_camera_fb_return(fb);
}

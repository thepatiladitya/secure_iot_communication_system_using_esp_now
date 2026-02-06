/************************************************************
 * Module      : Base Station Controller
 * Role        : USB ↔ ESP-NOW bridge
 * Function    :
 *   - Receives trigger (0x01) from PC via Serial
 *   - Requests data from SD ESP32 via ESP-NOW
 *   - Forwards all received sensor data to Serial
 *   - Sends "END" when transfer is complete
 * Protocol    : ESP-NOW (Peer-to-Peer)
 * Interface   : USB Serial (PC / Python)
 ************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* -------------------- Configuration -------------------- */
// 🔴 Replace with SD ESP32 MAC address
const uint8_t SD_ESP_MAC[6] = {0xFC, 0xE8, 0xC0, 0xE1, 0xBD, 0xA4};

#define ESPNOW_CHANNEL   6
#define TRANSFER_TIMEOUT_MS 8000

/* -------------------- State Variables ------------------ */
bool transferActive = false;
unsigned long lastDataTime = 0;

/* -------------------- ESP-NOW Receive Callback ---------- */
void onDataReceived(const esp_now_recv_info_t *info,
                    const uint8_t *data,
                    int len)
{
    // Ignore data if transfer not active
    if (!transferActive) return;

    // Verify sender MAC address (SD ESP32)
    if (memcmp(info->src_addr, SD_ESP_MAC, 6) != 0) return;

    if (len > 0)
    {
        char message[256];
        int copyLen = (len < 255) ? len : 255;
        memcpy(message, data, copyLen);
        message[copyLen] = '\0';

        String line(message);
        line.trim();

        if (line.length() > 0)
        {
            Serial.println(line);     // Forward to PC / Python
            lastDataTime = millis();  // Reset timeout
        }
    }
}

/* -------------------- ESP-NOW Setup -------------------- */
void setupEspNow()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("❌ ESP-NOW initialization failed");
        return;
    }

    esp_now_register_recv_cb(onDataReceived);

    // Register SD ESP32 as peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, SD_ESP_MAC, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("❌ Failed to add ESP-NOW peer");
        return;
    }

    // Lock Wi-Fi channel
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    Serial.println("✅ Base Station ready (ESP-NOW active)");
}

/* --------------------------- Setup ---------------------- */
void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println("\n🟢 Base Station ESP32 Starting...");
    setupEspNow();
}

/* ---------------------------- Loop ---------------------- */
void loop()
{
    /* ---------- Listen for trigger from PC ---------- */
    if (Serial.available())
    {
        uint8_t cmd = Serial.read();

        if (cmd == 0x01)
        {
            Serial.println("📥 Transfer trigger received from PC");

            // Request data from SD ESP32
            const char *request = "GET_DATA";
            esp_now_send(SD_ESP_MAC,
                         (uint8_t *)request,
                         strlen(request));

            transferActive = true;
            lastDataTime = millis();

            Serial.println("📤 Data request sent to SD ESP32");
        }
    }

    /* ---------- Timeout Handling ---------- */
    if (transferActive &&
        (millis() - lastDataTime > TRANSFER_TIMEOUT_MS))
    {
        Serial.println("END");   // Signal end of transfer to PC
        transferActive = false;

        Serial.println("✅ Transfer completed (timeout)");
    }

    delay(10);
}

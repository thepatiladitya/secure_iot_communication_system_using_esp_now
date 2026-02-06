/************************************************************
 * Module      : MQ-6 Gas Sensor Node
 * Node ID     : 2
 * Function    : Measures gas concentration using MQ-6 sensor
 * Trigger     : Receives command 0x01 via ESP-NOW
 * Response    : Sends gas value as string
 * Protocol    : ESP-NOW (Peer-to-Peer)
 * WiFi Mode   : Station (STA)
 ************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* -------------------- Hardware Configuration -------------------- */
#define MQ6_PIN  34        // ADC1 pin (safe for ESP-NOW)

/* -------------------- ESP-NOW Configuration --------------------- */
// 🔴 Replace with Data Postman / SD ESP32 MAC address
const uint8_t RECEIVER_MAC[6] = {0xFC, 0xE8, 0xC0, 0xE1, 0xBD, 0xA4};

#define ESPNOW_CHANNEL  1

/* -------------------- ESP-NOW Receive Callback ------------------ */
void onDataReceived(const esp_now_recv_info_t *info,
                    const uint8_t *data,
                    int len)
{
    // Respond only to single-byte trigger 0x01
    if (len == 1 && data[0] == 0x01)
    {
        Serial.println("📥 Trigger received → Reading MQ-6 sensor");

        // Small delay to stabilize ADC after radio activity
        delay(5);

        // Read sensor twice and average to reduce noise
        int reading1 = analogRead(MQ6_PIN);
        delay(2);
        int reading2 = analogRead(MQ6_PIN);
        int gasValue = (reading1 + reading2) / 2;

        // Prepare payload
        char payload[16];
        snprintf(payload, sizeof(payload), "%d", gasValue);

        Serial.printf("📤 Sending gas value → %s\n", payload);

        // Send data via ESP-NOW
        esp_now_send(RECEIVER_MAC,
                     (uint8_t *)payload,
                     strlen(payload));
    }
}

/* --------------------------- Setup ------------------------------- */
void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println("\n🟢 MQ-6 Gas Sensor Node Initializing");

    // Explicitly configure sensor pin
    pinMode(MQ6_PIN, INPUT);

    // Configure WiFi for ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Lock WiFi channel (must match receiver)
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("❌ ESP-NOW initialization failed");
        return;
    }

    // Register receive callback
    esp_now_register_recv_cb(onDataReceived);

    // Register peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("❌ Failed to add ESP-NOW peer");
        return;
    }

    Serial.println("✅ MQ-6 Gas Sensor Node ready (ESP-NOW only)");
}

/* ---------------------------- Loop ------------------------------- */
void loop()
{
    // Event-driven design – handled via ESP-NOW callback
    delay(10);
}

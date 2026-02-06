/************************************************************
 * Module      : DHT11 Sensor Node
 * Node ID     : 1
 * Function    : Reads temperature & humidity from DHT11
 * Trigger     : Receives command 0x01 via ESP-NOW
 * Response    : Sends "temperature,humidity" string
 * Protocol    : ESP-NOW (Peer-to-Peer)
 * WiFi Mode   : Station (STA)
 ************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "DHT.h"

/* -------------------- DHT11 Configuration -------------------- */
#define DHT_PIN   14
#define DHT_TYPE  DHT11

DHT dht(DHT_PIN, DHT_TYPE);

/* -------------------- ESP-NOW Configuration ------------------ */
// 🔴 Replace with Data Postman / Receiver MAC address
const uint8_t RECEIVER_MAC[6] = {0xFC, 0xE8, 0xC0, 0xE1, 0xBD, 0xA4};

#define ESPNOW_CHANNEL  1

/* -------------------- ESP-NOW Receive Callback ---------------- */
void onDataReceived(const esp_now_recv_info_t *info,
                    const uint8_t *data,
                    int len)
{
    // Expecting single-byte command: 0x01
    if (len == 1 && data[0] == 0x01)
    {
        Serial.println("📥 Request received → Reading DHT11");

        float temperature = dht.readTemperature();
        float humidity    = dht.readHumidity();

        // Retry once if sensor read fails
        if (isnan(temperature) || isnan(humidity))
        {
            delay(500);
            temperature = dht.readTemperature();
            humidity    = dht.readHumidity();
        }

        if (isnan(temperature) || isnan(humidity))
        {
            Serial.println("❌ DHT11 sensor read failed");
            return;
        }

        // Format data as "temp,hum"
        char payload[32];
        snprintf(payload, sizeof(payload), "%.1f,%.1f",
                 temperature, humidity);

        Serial.printf("📤 Sending data → %s\n", payload);

        esp_now_send(RECEIVER_MAC,
                      (uint8_t *)payload,
                      strlen(payload));
    }
}

/* --------------------------- Setup ---------------------------- */
void setup()
{
    Serial.begin(115200);
    delay(1000);

    dht.begin();

    // Configure WiFi in Station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Set fixed ESP-NOW channel
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

    Serial.println("✅ DHT11 Sensor Node initialized successfully");
}

/* ---------------------------- Loop ---------------------------- */
void loop()
{
    // All operations handled via ESP-NOW callback
    delay(10);
}

/************************************************************
 * Module      : Data Postman Controller (SD ESP32)
 * Role        : Central coordinator on mobile unit
 * Function    :
 *   - Receives TCP commands from PC / Controller
 *   - Triggers sensor nodes (DHT11 / MQ-6) via ESP-NOW
 *   - Collects fixed number of sensor readings
 *   - Stores readings with timestamps
 *   - Responds to Base Station "GET_DATA" requests
 * Communication:
 *   - TCP (Wi-Fi)  → Control & trigger
 *   - ESP-NOW      → Sensor & Base Station data exchange
 ************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* -------------------- Wi-Fi Configuration -------------------- */
const char* WIFI_SSID     = "LXX518";
const char* WIFI_PASSWORD = "wajid1234";

#define TCP_SERVER_PORT 5000

/* -------------------- ESP-NOW Configuration ------------------ */
#define ESPNOW_CHANNEL 1
#define MAX_READINGS   5

/* -------------------- Peer MAC Addresses --------------------- */
// 🔴 Replace all MAC addresses with your actual devices
uint8_t DHT_NODE_MAC[]  = {0x6C, 0xC8, 0x40, 0x35, 0x46, 0xCC};
uint8_t MQ6_NODE_MAC[]  = {0xFC, 0xE8, 0xC0, 0xE0, 0xF0, 0x3C};
uint8_t BASE_ESP_MAC[]  = {0xFC, 0xE8, 0xC0, 0xE0, 0xC6, 0x34};

/* -------------------- Global State --------------------------- */
String baseTimestamp = "";
int currentTargetNode = 0;          // 1 = DHT11, 2 = MQ-6
String sensorReadings[MAX_READINGS];
int receivedCount = 0;

/* -------------------- TCP Server ----------------------------- */
WiFiServer tcpServer(TCP_SERVER_PORT);

/* -------------------- Timestamp Helper ----------------------- */
String addSecondsToTimestamp(const String& ts, int seconds)
{
    if (ts.length() < 19) return ts + ",ERR";

    int pos = ts.lastIndexOf(':');
    if (pos == -1) return ts + ",ERR";

    int sec = ts.substring(pos + 1).toInt();
    int newSec = (sec + seconds) % 60;

    char buffer[24];
    snprintf(buffer, sizeof(buffer), "%s:%02d",
             ts.substring(0, pos).c_str(), newSec);

    return String(buffer);
}

/* -------------------- ESP-NOW Receive Callback --------------- */
void onDataReceived(const esp_now_recv_info_t *info,
                    const uint8_t *data,
                    int len)
{
    if (len <= 0) return;

    /* ---------- Handle Base Station Request ---------- */
    if (memcmp(info->src_addr, BASE_ESP_MAC, 6) == 0)
    {
        char msg[256];
        int copyLen = (len < 255) ? len : 255;
        memcpy(msg, data, copyLen);
        msg[copyLen] = '\0';

        String command(msg);
        command.trim();

        if (command == "GET_DATA")
        {
            Serial.println("📞 Base Station requested data");

            for (int i = 0; i < receivedCount; i++)
            {
                if (sensorReadings[i].length() > 0)
                {
                    esp_now_send(BASE_ESP_MAC,
                                 (uint8_t*)sensorReadings[i].c_str(),
                                 sensorReadings[i].length());
                    delay(5);
                }
            }

            const char* endMarker = "END";
            esp_now_send(BASE_ESP_MAC,
                         (uint8_t*)endMarker,
                         strlen(endMarker));

            Serial.println("📤 Data sent to Base Station");
            return;
        }
    }

    /* ---------- Handle Sensor Data ---------- */
    if (receivedCount >= MAX_READINGS) return;

    bool fromDHT = (memcmp(info->src_addr, DHT_NODE_MAC, 6) == 0);
    bool fromMQ6 = (memcmp(info->src_addr, MQ6_NODE_MAC, 6) == 0);

    char msg[256];
    int copyLen = (len < 255) ? len : 255;
    memcpy(msg, data, copyLen);
    msg[copyLen] = '\0';

    String rawData(msg);
    rawData.trim();

    if (fromDHT && currentTargetNode == 1)
    {
        int comma = rawData.indexOf(',');
        if (comma != -1)
        {
            String temp = rawData.substring(0, comma);
            String hum  = rawData.substring(comma + 1);

            String ts = addSecondsToTimestamp(baseTimestamp, receivedCount);
            sensorReadings[receivedCount] = temp + "," + hum + "," + ts;

            Serial.printf("📥 DHT [%d]: %s\n",
                          receivedCount + 1,
                          sensorReadings[receivedCount].c_str());
            receivedCount++;
        }
    }
    else if (fromMQ6 && currentTargetNode == 2)
    {
        String ts = addSecondsToTimestamp(baseTimestamp, receivedCount);
        sensorReadings[receivedCount] = rawData + "," + ts;

        Serial.printf("📥 MQ-6 [%d]: %s\n",
                      receivedCount + 1,
                      sensorReadings[receivedCount].c_str());
        receivedCount++;
    }
}

/* -------------------- ESP-NOW Setup -------------------------- */
void setupEspNow()
{
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("❌ ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(onDataReceived);

    esp_now_peer_info_t peer = {};

    memcpy(peer.peer_addr, DHT_NODE_MAC, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    memcpy(peer.peer_addr, MQ6_NODE_MAC, 6);
    esp_now_add_peer(&peer);

    memcpy(peer.peer_addr, BASE_ESP_MAC, 6);
    esp_now_add_peer(&peer);

    Serial.println("✅ ESP-NOW peers registered");
}

/* --------------------------- Setup ---------------------------- */
void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println("\n🚗 SD ESP32 (Data Postman) Starting");

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\n✅ Wi-Fi connected");
    Serial.println("IP: " + WiFi.localIP().toString());

    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    setupEspNow();

    tcpServer.begin();
    Serial.println("📡 TCP server ready (port 5000)");
}

/* ---------------------------- Loop ---------------------------- */
void loop()
{
    WiFiClient client = tcpServer.available();

    if (client)
    {
        String cmd = client.readStringUntil('\n');
        cmd.trim();

        if (cmd.startsWith("r,"))
        {
            int c1 = cmd.indexOf(',');
            int c2 = cmd.indexOf(',', c1 + 1);

            if (c1 != -1 && c2 != -1)
            {
                currentTargetNode = cmd.substring(c1 + 1, c2).toInt();
                baseTimestamp = cmd.substring(c2 + 1);

                if (currentTargetNode == 1 || currentTargetNode == 2)
                {
                    receivedCount = 0;
                    for (int i = 0; i < MAX_READINGS; i++)
                        sensorReadings[i] = "";

                    client.println("OK");
                    client.println("END");
                    client.stop();

                    uint8_t trigger = 0x01;
                    uint8_t* targetMac =
                        (currentTargetNode == 1) ? DHT_NODE_MAC : MQ6_NODE_MAC;

                    Serial.printf("🟢 Triggering Node %d\n", currentTargetNode);

                    for (int i = 0; i < MAX_READINGS; i++)
                    {
                        esp_now_send(targetMac, &trigger, 1);
                        Serial.printf("📤 Trigger %d sent\n", i + 1);
                        if (i < MAX_READINGS - 1) delay(1000);
                    }
                }
                else
                {
                    client.println("ERROR: Invalid node");
                    client.stop();
                }
            }
        }
        else
        {
            client.println("ERROR: Unknown command");
            client.stop();
        }
    }

    delay(10);
}

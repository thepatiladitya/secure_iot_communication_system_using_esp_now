/************************************************************
 * Module      : Car Controller
 * Function    : Controls mobile robot movement
 * Interface   : TCP commands over Wi-Fi
 * Features    :
 *   - Forward / backward motion
 *   - In-place rotation (angle based)
 *   - Ultrasonic-guided approach & return
 *   - Simple state machine
 ************************************************************/

#include <WiFi.h>

/* -------------------- Wi-Fi Configuration -------------------- */
const char* WIFI_SSID     = "LXX518";
const char* WIFI_PASSWORD = "wajid1234";

#define SERVER_PORT 12345
WiFiServer server(SERVER_PORT);
WiFiClient activeClient;

/* -------------------- Motor Pin Configuration ---------------- */
#define ENA  5     // PWM
#define ENB  23    // PWM
#define IN1  22
#define IN2  21
#define IN3  19
#define IN4  18

/* -------------------- Ultrasonic Sensor ---------------------- */
#define TRIG_PIN 4
#define ECHO_PIN 2

/* -------------------- PWM Settings --------------------------- */
#define PWM_FREQ 1000
#define PWM_RES  8

/* -------------------- Motion Parameters ---------------------- */
int MOTOR_SPEED_APPROACH = 80;
int MOTOR_SPEED_ROTATE   = 130;

/* Rotation calibration */
int ROTATE_DEG        = 7;   // degrees
#define DEG_TO_MS     6
int ROTATE_TIME_MS   = ROTATE_DEG * DEG_TO_MS;

/* Ultrasonic parameters */
#define MIN_STOP_DISTANCE_CM   15.0f
#define US_MEASURE_INTERVAL_MS 30

/* -------------------- State Machine -------------------------- */
enum RobotState {
    IDLE,
    ROTATING,
    APPROACHING_NODE,
    WAITING_AT_NODE,
    RETURNING_TO_BASE
};

RobotState currentState = IDLE;

/* -------------------- Timing Variables ----------------------- */
unsigned long stateStartTime   = 0;
unsigned long approachStartTime = 0;
unsigned long lastUSReadTime   = 0;
unsigned long travelTimeMs    = 0;

/* -------------------- Motor Control Functions ---------------- */
void stopMotors()
{
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
}

void brakeMotors()
{
    digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);
    delay(150);
    stopMotors();
}

void forward(int speed)
{
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    ledcWrite(ENA, speed);
    ledcWrite(ENB, speed);
}

void backward(int speed)
{
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    ledcWrite(ENA, speed);
    ledcWrite(ENB, speed);
}

void rotateInPlace(int speed)
{
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    ledcWrite(ENA, speed);
    ledcWrite(ENB, speed);
}

/* -------------------- Ultrasonic Measurement ----------------- */
float getDistanceCM()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0) return 999.0f;

    return (duration * 0.0343f) / 2.0f;
}

/* -------------------- Motor Setup ---------------------------- */
void motorSetup()
{
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

    ledcAttach(ENA, PWM_FREQ, PWM_RES);
    ledcAttach(ENB, PWM_FREQ, PWM_RES);

    stopMotors();
}

/* --------------------------- Setup ---------------------------- */
void setup()
{
    Serial.begin(115200);

    motorSetup();
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("\n📡 Connecting to Wi-Fi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\n✅ Wi-Fi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    server.begin();
    Serial.println("🚗 Car Controller ready (TCP server running)");
}

/* ---------------------------- Loop ---------------------------- */
void loop()
{
    unsigned long now = millis();

    /* ---------- Command Handling (IDLE state) ---------- */
    if (currentState == IDLE) {
        WiFiClient client = server.available();
        if (client) {
            String cmd = client.readStringUntil('\n');
            cmd.trim();

            Serial.printf("📥 Command received: %s\n", cmd.c_str());

            if (cmd == "ROTATE") {
                rotateInPlace(MOTOR_SPEED_ROTATE);
                stateStartTime = now;
                currentState = ROTATING;
                client.println("OK");
                client.stop();
            }
            else if (cmd == "APPROACH_NODE") {
                forward(MOTOR_SPEED_APPROACH);
                approachStartTime = now;
                lastUSReadTime = now;
                activeClient = client;
                currentState = APPROACHING_NODE;
                client.println("OK");
                Serial.println("🚶 Approaching node...");
            }
            else if (cmd.startsWith("SPEED_FWD")) {
                MOTOR_SPEED_APPROACH = constrain(cmd.substring(9).toInt(), 0, 255);
                client.println("OK");
                client.stop();
            }
            else if (cmd.startsWith("SPEED_ROT")) {
                MOTOR_SPEED_ROTATE = constrain(cmd.substring(9).toInt(), 0, 255);
                client.println("OK");
                client.stop();
            }
            else {
                client.println("ERROR");
                client.stop();
            }
        }
    }

    /* ---------- State Machine ---------- */
    switch (currentState)
    {
        case ROTATING:
            if (now - stateStartTime >= ROTATE_TIME_MS) {
                stopMotors();
                Serial.println("🔄 Rotation complete");
                currentState = IDLE;
            }
            break;

        case APPROACHING_NODE: {
            bool shouldStop = false;

            if (now - lastUSReadTime >= US_MEASURE_INTERVAL_MS) {
                float dist = getDistanceCM();
                lastUSReadTime = now;
                Serial.printf("📡 Distance: %.1f cm\n", dist);

                if (dist <= MIN_STOP_DISTANCE_CM) {
                    shouldStop = true;
                }
            }

            if (now - approachStartTime > 10000) {
                shouldStop = true;
                travelTimeMs = now - approachStartTime;
                Serial.println("⚠️ Approach timeout");
            }

            if (shouldStop) {
                brakeMotors();
                if (travelTimeMs == 0)
                    travelTimeMs = now - approachStartTime;

                if (activeClient && activeClient.connected()) {
                    activeClient.println("STATUS:REACHED_NODE");
                    activeClient.flush();
                }

                stateStartTime = now;
                currentState = WAITING_AT_NODE;
            }
            break;
        }

        case WAITING_AT_NODE:
            if (now - stateStartTime >= 7000) {
                backward(MOTOR_SPEED_APPROACH);
                stateStartTime = now;
                currentState = RETURNING_TO_BASE;
            }
            break;

        case RETURNING_TO_BASE:
            if (now - stateStartTime >= travelTimeMs) {
                brakeMotors();
                if (activeClient && activeClient.connected()) {
                    activeClient.println("STATUS:BACK_AT_BASE");
                    activeClient.stop();
                }
                travelTimeMs = 0;
                currentState = IDLE;
            }
            break;

        default:
            break;
    }

    delay(2);
}

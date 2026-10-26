// Device-ul IoT din tema 01, de data aceasta pe un ESP32-C3 Super Mini.
//
// Aceeași logică și același protocol ca device-simulator din tema 01, cu trei
// taskuri FreeRTOS în loc de trei thread-uri Java:
//
//   task periodic --(coada de alarme)--> task sporadic
//        |                                     |
//        +---- telemetry ---> MQTT <--- alarm -+
//                              |
//                          commands --> command_received() --(coada de comenzi)--> task aperiodic ---> status
//
// Gata implementate (nu trebuie modificate):
//   config.h         setările: Wi-Fi, broker, STUDENT, valorile inițiale; DEVICE_ID
//                    se citește din adresa MAC a plăcii
//   connection.h/cpp Wi-Fi, ora exactă (SNTP), clientul MQTT, mqtt_publish(), epoch_ms()
//
// Așa cum este, schița se conectează la Wi-Fi și la broker și afișează pe
// Serial comenzile primite. Taskurile le scrieți voi: TODO 0 ... TODO 3.
// Exemplele de pornire sunt în laboratorul 0 din rt-labs-esp32c3; vezi
// exemple/README.md din folderul temei.
//
// Placa: ESP32C3 Dev Module, USB CDC On Boot: Enabled. Serial Monitor: 115200 baud.

#include "config.h"
#include "connection.h"

// Temperatura cipului, în °C, de la senzorul intern al ESP32-C3.
// Este temperatura siliciului, nu a camerei: imediat după pornirea Wi-Fi-ului
// poate fi în jur de 50 °C, apoi scade în câteva minute spre 35-40 °C.
// Rezoluția este de aproximativ 1 °C. Ca să provocați alarme, puneți pragul
// cu SET_THRESHOLD cu un grad sub valoarea citită.
float read_temperature(void) {
    return temperatureRead();
}

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

struct AlarmEvent {
    float temperature;
    float threshold;
};

struct CommandEvent {
    char text[COMMAND_MAX_LEN];
};

static QueueHandle_t alarmQueue = nullptr;
static QueueHandle_t commandQueue = nullptr;
static volatile uint32_t currentPeriodMs = PERIOD_MS_INITIAL;
static volatile float currentThreshold = THRESHOLD_INITIAL;

// Task periodic: xTaskDelayUntil păstrează grila de eliberări.
static void periodic_task(void* parameter) {
    (void)parameter;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        const float temperature = read_temperature();
        const int64_t timestamp = epoch_ms();

        char message[COMMAND_MAX_LEN + 32];
        snprintf(message, sizeof(message), "%s;%lld;%.2f",
                 DEVICE_ID, (long long)timestamp, temperature);
        mqtt_publish("telemetry", message);
        Serial.printf("[periodic] telemetry %.2f C, T=%lu ms\n",
                      temperature, (unsigned long)currentPeriodMs);

        const float threshold = currentThreshold;
        if (temperature > threshold) {
            AlarmEvent event{temperature, threshold};
            if (xQueueSend(alarmQueue, &event, 0) != pdTRUE) {
                Serial.println("[periodic] alarma respinsa: coada plina");
            }
        }

        uint32_t period = currentPeriodMs;
        if (period < 100) period = 100;
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(period));
    }
}

// TODO 2: task sporadic: consumă alarmele și respectă intervalul minim.
static void sporadic_task(void* parameter) {
    (void)parameter;
    uint32_t lastPublished = 0;
    bool hasPublished = false;
    AlarmEvent event;
    for (;;) {
        if (xQueueReceive(alarmQueue, &event, portMAX_DELAY) == pdTRUE) {
            const uint32_t now = millis();
            if (hasPublished && (uint32_t)(now - lastPublished) < MIN_INTERARRIVAL_MS) {
                Serial.printf("[sporadic] alarma respinsa: interval %lu ms (< %lu ms)\n",
                    (unsigned long)(uint32_t)(now - lastPublished),
                    (unsigned long)MIN_INTERARRIVAL_MS);
                continue;
            }
            char message[COMMAND_MAX_LEN + 48];
            snprintf(message, sizeof(message), "%s;%lld;%.2f;%.2f",
                     DEVICE_ID, (long long)epoch_ms(), event.temperature, event.threshold);
            if (mqtt_publish("alarm", message)) {
                lastPublished = now;
                hasPublished = true;
                Serial.printf("[sporadic] alarma publicata: %.2f C > %.2f C\n",
                              event.temperature, event.threshold);
            }
        }
    }
}

static void publish_status(const char* text) {
    char message[COMMAND_MAX_LEN + 48];
    snprintf(message, sizeof(message), "%s;%lld;%s",
             DEVICE_ID, (long long)epoch_ms(), text);
    mqtt_publish("status", message);
}

static void trim_command(char* text) {
    size_t n = strlen(text);
    while (n > 0 && isspace((unsigned char)text[n - 1])) text[--n] = '\0';
    size_t i = 0;
    while (text[i] && isspace((unsigned char)text[i])) ++i;
    if (i) memmove(text, text + i, strlen(text + i) + 1);
}

// TODO 3: task aperiodic: validează și execută comenzile din coadă.
static void aperiodic_task(void* parameter) {
    (void)parameter;
    CommandEvent event;
    for (;;) {
        if (xQueueReceive(commandQueue, &event, portMAX_DELAY) != pdTRUE) continue;
        trim_command(event.text);
        char* separator = strchr(event.text, ' ');
        char* argument = nullptr;
        if (separator) {
            *separator = '\0';
            argument = separator + 1;
            while (*argument == ' ') ++argument;
        }

        char response[COMMAND_MAX_LEN];
        if (strcmp(event.text, "SET_INTERVAL") == 0) {
            char* end = nullptr;
            long value = argument ? strtol(argument, &end, 10) : 0;
            while (end && *end && isspace((unsigned char)*end)) ++end;
            if (!argument || end == argument || (end && *end) || value < 100 || value > 3600000) {
                snprintf(response, sizeof(response), "ERROR invalid SET_INTERVAL (100..3600000 ms)");
            } else {
                currentPeriodMs = (uint32_t)value;
                snprintf(response, sizeof(response), "OK SET_INTERVAL %lu", (unsigned long)currentPeriodMs);
            }
        } else if (strcmp(event.text, "SET_THRESHOLD") == 0) {
            char* end = nullptr;
            float value = argument ? strtof(argument, &end) : 0.0f;
            while (end && *end && isspace((unsigned char)*end)) ++end;
            if (!argument || end == argument || (end && *end) || !isfinite(value) || value < -40.0f || value > 125.0f) {
                snprintf(response, sizeof(response), "ERROR invalid SET_THRESHOLD");
            } else {
                currentThreshold = value;
                snprintf(response, sizeof(response), "OK SET_THRESHOLD %.2f", currentThreshold);
            }
        } else if (strcmp(event.text, "RESET") == 0 && (!argument || !*argument)) {
            currentPeriodMs = PERIOD_MS_INITIAL;
            currentThreshold = THRESHOLD_INITIAL;
            snprintf(response, sizeof(response), "OK RESET");
        } else if (strcmp(event.text, "STATUS") == 0 && (!argument || !*argument)) {
            snprintf(response, sizeof(response), "OK STATUS interval=%lu threshold=%.2f",
                     (unsigned long)currentPeriodMs, currentThreshold);
        } else {
            snprintf(response, sizeof(response), "ERROR unknown command or invalid arguments");
        }

        Serial.printf("[aperiodic] %s -> %s\n", event.text, response);
        publish_status(response);
    }
}

// Apelată de clientul MQTT pentru fiecare comandă primită (vezi connection.h).
void command_received(const char* text) {
    Serial.printf("[mqtt] comandă primită: %s\n", text);
    if (commandQueue == nullptr) {
        Serial.println("[mqtt] comanda ignorata: coada nu este creata");
        return;
    }
    CommandEvent event{};
    strncpy(event.text, text, sizeof(event.text) - 1);
    event.text[sizeof(event.text) - 1] = '\0';
    if (xQueueSend(commandQueue, &event, 0) != pdTRUE) {
        Serial.println("[mqtt] comanda respinsa: coada plina");
    }
}

void setup() {
    Serial.begin(115200);
    // Fără așteptare la scriere: cu placa la USB, dar fără Serial Monitor deschis,
    // fiecare Serial.printf() ar aștepta până la 2 s să fie citit și ar întârzia
    // taskul care scrie. Așa, textul pe care nu-l citește nimeni se pierde.
    Serial.setTxTimeoutMs(0);
    uint32_t t = millis();
    while (!Serial && millis() - t < 3000) delay(10);  // așteptăm Serial Monitor, cel mult 3 s
    delay(200);
    Serial.println();
    Serial.printf("=== tema 02: STUDENT %s, DEVICE_ID %s ===\n", STUDENT, DEVICE_ID);

    wifi_connect();
    time_sync();

    // Cozile trebuie create înainte de pornirea clientului MQTT.
    alarmQueue = xQueueCreate(10, sizeof(AlarmEvent));
    commandQueue = xQueueCreate(10, sizeof(CommandEvent));
    if (alarmQueue == nullptr || commandQueue == nullptr) {
        Serial.println("EROARE: nu s-au putut crea cozile FreeRTOS");
        while (true) delay(1000);
    }

    mqtt_start();

    // Pornim taskurile după MQTT; toate au stivă de minimum 4096 octeți.
    BaseType_t ok1 = xTaskCreate(periodic_task, "periodic", 4096, nullptr, 4, nullptr);
    BaseType_t ok2 = xTaskCreate(sporadic_task, "sporadic", 4096, nullptr, 3, nullptr);
    BaseType_t ok3 = xTaskCreate(aperiodic_task, "aperiodic", 4096, nullptr, 5, nullptr);
    if (ok1 != pdPASS || ok2 != pdPASS || ok3 != pdPASS) {
        Serial.println("EROARE: nu s-au putut porni toate taskurile");
    }
}

void loop() {
    // Totul se face în taskuri: loop() nu are nimic de făcut.
    delay(1000);
}

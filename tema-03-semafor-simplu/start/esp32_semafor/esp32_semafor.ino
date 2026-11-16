// Semafor pentru pietoni, cu buton, pe ESP32-C3 Super Mini.
//
// Placa este controlerul semaforului; lămpile sunt desenate de dashboard, care
// le aprinde după mesajele "lights" publicate de placă. Patru activități:
//
//   ISR buton BOOT ----+
//                      +--(coada butonului)--> task sporadic
//   command_received --+   (BUTTON)                 |
//                                                   v
//                                         [ starea semaforului ] <-- mutex
//                                            ^          |
//                                            |          v
//                         task periodic de control   task periodic de ieșire
//                         (fazele, T = 100 ms)       (lămpile, T = 500 ms) --> lights
//
// Gata implementate (nu trebuie modificate):
//   config.h         setările: Wi-Fi, broker, STUDENT, pinii, perioadele, duratele fazelor
//   connection.h/cpp Wi-Fi, ora exactă (SNTP), clientul MQTT, mqtt_publish(), epoch_ms()
//
// Așa cum este, schița se conectează la Wi-Fi și la broker și afișează pe
// Serial comenzile primite. Restul îl scrieți voi: TODO 0 ... TODO 3.
//
// Placa: ESP32C3 Dev Module, USB CDC On Boot: Enabled. Serial Monitor: 115200 baud.

#include "config.h"
#include "connection.h"

// Fazele semaforului (vezi tabelul și diagrama din README.md). Numele din
// PHASE_NAMES sunt cele din mesajul "lights", în aceeași ordine ca în enum.
enum Phase { CAR_GREEN, CAR_YELLOW, ALL_RED, PED_GREEN };
const char* const PHASE_NAMES[] = {"CAR_GREEN", "CAR_YELLOW", "ALL_RED", "PED_GREEN"};

// Culorile unei lămpi, cu numele din mesajul "lights".
enum Light { RED, YELLOW, GREEN };
const char* const LIGHT_NAMES[] = {"RED", "YELLOW", "GREEN"};

// TODO 0 (c): STAREA SEMAFORULUI, partajată de taskuri și protejată de un
//   mutex (xSemaphoreCreateMutex). De exemplu un struct cu:
struct SemaphoreState {
    Phase phase;
    uint32_t phase_started_ms;  // millis(): timp monoton, independent de SNTP
    bool pedestrian_request;
};

// Momentul sosirii este salvat in coada, nu calculat cand taskul preia elementul.
struct ButtonPress {
    TickType_t arrival_tick;
};

static SemaphoreState semaphore_state;
static SemaphoreHandle_t state_mutex = nullptr;
static QueueHandle_t button_queue = nullptr;

static void control_task(void* parameter);
static void output_task(void* parameter);
static void button_task(void* parameter);
void IRAM_ATTR button_isr(void);

static const char* phase_name(Phase phase) {
    if (phase >= CAR_GREEN && phase <= PED_GREEN) return PHASE_NAMES[phase];
    return "UNKNOWN";
}

// TODO 1: TASK PERIODIC DE CONTROL, cu perioada CONTROL_PERIOD_MS.
static void control_task(void* parameter) {
    (void)parameter;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    for (;;) {
        xTaskDelayUntil(&last_wake, period);

        const uint32_t now = millis();
        bool changed = false;
        Phase new_phase = CAR_GREEN;

        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
            const uint32_t elapsed = now - semaphore_state.phase_started_ms;

            switch (semaphore_state.phase) {
                case CAR_GREEN:
                    if (semaphore_state.pedestrian_request && elapsed >= CAR_GREEN_MIN_MS) {
                        semaphore_state.phase = CAR_YELLOW;
                        changed = true;
                    }
                    break;

                case CAR_YELLOW:
                    if (elapsed >= CAR_YELLOW_MS) {
                        semaphore_state.phase = ALL_RED;
                        changed = true;
                    }
                    break;

                case ALL_RED:
                    if (elapsed >= ALL_RED_MS) {
                        semaphore_state.phase = PED_GREEN;
                        semaphore_state.pedestrian_request = false;
                        changed = true;
                    }
                    break;

                case PED_GREEN:
                    if (elapsed >= PED_GREEN_MS) {
                        semaphore_state.phase = CAR_GREEN;
                        changed = true;
                    }
                    break;
            }

            if (changed) {
                semaphore_state.phase_started_ms = now;
                new_phase = semaphore_state.phase;
            }
            xSemaphoreGive(state_mutex);
        }

        // Nu scriem pe Serial cat timp mutexul este tinut.
        if (changed) {
            Serial.printf("[control] faza noua: %s\n", phase_name(new_phase));
        }
    }
}
// TODO 2: TASK PERIODIC DE IEȘIRE, cu perioada OUTPUT_PERIOD_MS.
static void output_task(void* parameter) {
    (void)parameter;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(OUTPUT_PERIOD_MS);

    for (;;) {
        xTaskDelayUntil(&last_wake, period);

        SemaphoreState snapshot;
        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
            snapshot = semaphore_state;
            xSemaphoreGive(state_mutex);
        } else {
            continue;
        }

        Light car_light = RED;
        Light pedestrian_light = RED;
        switch (snapshot.phase) {
            case CAR_GREEN:
                car_light = GREEN;
                pedestrian_light = RED;
                break;
            case CAR_YELLOW:
                car_light = YELLOW;
                pedestrian_light = RED;
                break;
            case ALL_RED:
                car_light = RED;
                pedestrian_light = RED;
                break;
            case PED_GREEN:
                car_light = RED;
                pedestrian_light = GREEN;
                break;
        }

        uint32_t seconds_remaining = 0;
        if (snapshot.phase == PED_GREEN) {
            const uint32_t elapsed = millis() - snapshot.phase_started_ms;
            const uint32_t remaining = (elapsed >= PED_GREEN_MS) ? 0 : (PED_GREEN_MS - elapsed);
            // Rotunjire in sus: 7200 ms -> 8, 300 ms -> 1.
            seconds_remaining = (remaining + 999U) / 1000U;
        }

        // LED-ul placii este aprins pe LOW cat timp exista cerere pietonala.
        digitalWrite(LED_PIN, snapshot.pedestrian_request ? LOW : HIGH);

        char message[160];
        snprintf(message, sizeof(message), "%s;%lld;%s;%s;%s;%d;%lu",
                 DEVICE_ID,
                 (long long)epoch_ms(),
                 phase_name(snapshot.phase),
                 LIGHT_NAMES[car_light],
                 LIGHT_NAMES[pedestrian_light],
                 snapshot.pedestrian_request ? 1 : 0,
                 (unsigned long)seconds_remaining);

        if (!mqtt_publish("lights", message)) {
            Serial.println("[output] publicarea MQTT lights a esuat");
        }
    }
}
// TODO 3: TASK SPORADIC, tratează apăsările butonului pentru pietoni.
static void button_task(void* parameter) {
    (void)parameter;
    ButtonPress press;
    TickType_t last_accepted_tick = 0;
    bool has_accepted_press = false;
    const TickType_t min_interval = pdMS_TO_TICKS(MIN_INTERARRIVAL_MS);

    for (;;) {
        if (xQueueReceive(button_queue, &press, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (has_accepted_press && (TickType_t)(press.arrival_tick - last_accepted_tick) < min_interval) {
            Serial.println("[button] apasare respinsa: interval sub 1000 ms");
            continue;
        }

        last_accepted_tick = press.arrival_tick;
        has_accepted_press = true;

        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
            // Cererile multiple se consolideaza intr-o singura cerere.
            // O apasare in PED_GREEN ramane setata pentru ciclul urmator.
            semaphore_state.pedestrian_request = true;
            xSemaphoreGive(state_mutex);
            Serial.println("[button] cerere pieton acceptata");
        }
    }
}

// ISR BOOT: doar timestamp + trimitere in coada, fara Serial sau mutex.
void IRAM_ATTR button_isr(void) {
    if (button_queue == nullptr) return;

    ButtonPress press;
    press.arrival_tick = xTaskGetTickCountFromISR();
    BaseType_t higher_priority_task_woken = pdFALSE;
    xQueueSendFromISR(button_queue, &press, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}


// Apelată de clientul MQTT pentru fiecare comandă primită (vezi connection.h).
void command_received(const char* text) {
    if (text == nullptr || strcmp(text, "BUTTON") != 0 || button_queue == nullptr) {
            return;
        }

        ButtonPress press;
        press.arrival_tick = xTaskGetTickCount();
        // Timeout 0: callback-ul MQTT nu trebuie sa se blocheze.
        if (xQueueSend(button_queue, &press, 0) != pdTRUE) {
            Serial.println("[mqtt] cerere BUTTON ignorata: coada este plina");
        }
}

void setup() {
   Serial.begin(115200);
       Serial.setTxTimeoutMs(0);
       const uint32_t serial_start = millis();
       while (!Serial && millis() - serial_start < 3000) delay(10);
       delay(200);
       Serial.println();
       Serial.printf("=== tema 03: STUDENT %s, DEVICE_ID %s ===\n", STUDENT, DEVICE_ID);

       pinMode(LED_PIN, OUTPUT);
       digitalWrite(LED_PIN, HIGH);  // LED stins (activ pe LOW)
       pinMode(BUTTON_PIN, INPUT_PULLUP);

       wifi_connect();
       time_sync();

       // TODO 0(a): mutexul si coada trebuie create inainte de mqtt_start(),
       // deoarece callback-ul MQTT poate introduce comenzi BUTTON in coada.
       state_mutex = xSemaphoreCreateMutex();
       button_queue = xQueueCreate(10, sizeof(ButtonPress));
       if (state_mutex == nullptr || button_queue == nullptr) {
           Serial.println("[setup] EROARE: nu s-au putut crea mutexul/coada");
           while (true) delay(1000);
       }

       semaphore_state.phase = CAR_GREEN;
       semaphore_state.phase_started_ms = millis();
       semaphore_state.pedestrian_request = false;

       mqtt_start();

       // TODO 0(b): Rate Monotonic: control (100 ms) are prioritate mai mare
       // decat output (500 ms). Prioritatile sunt in intervalul 2..19.
       // Stiva este exprimata in octeti pe portul ESP32 Arduino.
       BaseType_t control_created = xTaskCreate(control_task, "semafor_control", 4096, nullptr, 5, nullptr);
       BaseType_t output_created = xTaskCreate(output_task, "semafor_output", 4096, nullptr, 4, nullptr);
       BaseType_t button_created = xTaskCreate(button_task, "semafor_button", 4096, nullptr, 6, nullptr);

       if (control_created != pdPASS || output_created != pdPASS || button_created != pdPASS) {
           Serial.println("[setup] EROARE: nu s-au putut crea toate taskurile");
           while (true) delay(1000);
       }

       attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_isr, FALLING);
       Serial.println("[setup] semafor pornit in CAR_GREEN");
}

void loop() {
    // Totul se face în taskuri: loop() nu are nimic de făcut.
    delay(1000);
}

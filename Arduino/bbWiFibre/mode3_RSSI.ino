// --- LED Configuration ---
#define NUM_RSSI_SLOTS 13 // We have 13 "columns" or slots to display RSSI

// --- RSSI Mapping & Visualization Tuning ---
const int RSSI_MIN_THRESHOLD = -90; // RSSI value considered the weakest signal for our scale
const int RSSI_MAX_THRESHOLD = -35; // RSSI value considered the strongest signal for our scale
const float RSSI_SMOOTHING_FACTOR = 0.4f; // For EMA filter (0.0 to 1.0, higher = more smoothing but slower reaction)

// Color Mapping (Hue values for Adafruit_NeoPixel::ColorHSV which uses 0-65535 for hue)
const uint16_t HUE_WEAK_SIGNAL = 0;       // Red
const uint16_t HUE_STRONG_SIGNAL = 21845; // Green (approx 120 degrees on a 360 scale)

// --- Animation Tuning ---
const unsigned long WIFI_SCAN_INTERVAL_MS = 2500; // How often to initiate a new WiFi scan

// Breathing effect for the lit RSSI bar
const bool ENABLE_RSSI_BAR_BREATHING = true;
const float RSSI_BAR_BREATH_SPEED = 0.8f;     // Speed of brightness oscillation (cycles per second)
const uint8_t RSSI_BAR_BREATH_MIN_V = 180;    // Min HSV Value (brightness) for breathing (0-255)
const uint8_t RSSI_BAR_BREATH_MAX_V = 255;    // Max HSV Value (brightness) for breathing

// Effect for when the target router is NOT found
const bool ENABLE_NOT_FOUND_EFFECT = true;
const char* NOT_FOUND_EFFECT_TYPE = "PULSE_ALL_RED"; // "PULSE_ALL_RED" or "CHASE_LIGHT" (Chase not implemented here for brevity)
const uint8_t NOT_FOUND_PULSE_MIN_V = 10;
const uint8_t NOT_FOUND_PULSE_MAX_V = 70;
const float NOT_FOUND_PULSE_SPEED = 0.6f;

// Color for unlit slots within the RSSI bar range (when router IS found but signal isn't max)
const uint8_t UNLIT_RSSI_SLOT_R = 0;
const uint8_t UNLIT_RSSI_SLOT_G = 5;
const uint8_t UNLIT_RSSI_SLOT_B = 10; // Very dim deep blue/teal

// --- Global Variables ---
volatile int g_latestRSSI = RSSI_MIN_THRESHOLD - 1; // Initial "not found" state
volatile bool g_routerIsVisible = false;
float g_smoothedRSSI = RSSI_MIN_THRESHOLD - 1;
SemaphoreHandle_t g_rssiDataMutex;

unsigned long g_lastScanInitiationTime = 0;
bool g_scanIsOngoing = false;

// Process WiFi Scan Results
void process_wifi_scan_results(int num_networks_found) {
    bool found_this_scan = false;
    int rssi_this_scan = RSSI_MIN_THRESHOLD - 1;

    for (int i = 0; i < num_networks_found; ++i) {
        if (memcmp(WiFi.BSSID(i), ROUTER_MAC, 6) == 0) {
            rssi_this_scan = WiFi.RSSI(i);
            found_this_scan = true;
            Serial.printf("Target router '%s' FOUND with RSSI: %d dBm\n", WiFi.SSID(i).c_str(), rssi_this_scan);
            break;
        }
    }
    if (!found_this_scan) {
        Serial.println("Target router NOT found in this scan.");
    }
    update_router_visibility_status(found_this_scan, rssi_this_scan);
}

// Update Global Router Status (Thread-Safe)
void update_router_visibility_status(bool found, int rssi_value) {
    if (xSemaphoreTake(g_rssiDataMutex, portMAX_DELAY) == pdTRUE) {
        g_routerIsVisible = found;
        g_latestRSSI = found ? rssi_value : (RSSI_MIN_THRESHOLD - 1); // Use out-of-range if not found
        xSemaphoreGive(g_rssiDataMutex);
    }
}

// RSSI Update Task (RTOS Task)
void RSSIUpdateTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(LED_UPDATE_INTERVAL_MS);
    float time_val_rad; // For animations

    // Initialize WiFi scanning
    WiFi.scanNetworks(true, false, false, 250, WIFI_CHANNEL);
    g_scanIsOngoing = true;
    g_lastScanInitiationTime = millis();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Handle WiFi scanning
        if (!g_scanIsOngoing && (millis() - g_lastScanInitiationTime > WIFI_SCAN_INTERVAL_MS)) {
            Serial.println("Initiating WiFi scan...");
            WiFi.scanNetworks(true, false, false, 250, WIFI_CHANNEL);
            g_scanIsOngoing = true;
            g_lastScanInitiationTime = millis();
        }

        if (g_scanIsOngoing) {
            int16_t scan_result = WiFi.scanComplete();
            if (scan_result >= 0) {
                Serial.printf("Scan complete. Networks found: %d\n", scan_result);
                process_wifi_scan_results(scan_result);
                WiFi.scanDelete();
                g_scanIsOngoing = false;
            } else if (scan_result == WIFI_SCAN_FAILED) {
                Serial.println("WiFi scan failed.");
                update_router_visibility_status(false, RSSI_MIN_THRESHOLD - 1);
                g_scanIsOngoing = false;
            }
        }

        int localLatestRSSI_val;
        bool localRouterIsVisible_val;

        // Safely get the latest RSSI data
        if (xSemaphoreTake(g_rssiDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            localLatestRSSI_val = g_latestRSSI;
            localRouterIsVisible_val = g_routerIsVisible;
            xSemaphoreGive(g_rssiDataMutex);
        } else {
            localRouterIsVisible_val = false;
            localLatestRSSI_val = RSSI_MIN_THRESHOLD - 1;
        }

        // Apply Exponential Moving Average (EMA) smoothing to RSSI
        if (localRouterIsVisible_val) {
            if (g_smoothedRSSI < RSSI_MIN_THRESHOLD) {
                g_smoothedRSSI = (float)localLatestRSSI_val;
            } else {
                g_smoothedRSSI = RSSI_SMOOTHING_FACTOR * (float)localLatestRSSI_val + (1.0f - RSSI_SMOOTHING_FACTOR) * g_smoothedRSSI;
            }
        } else {
            g_smoothedRSSI = RSSI_SMOOTHING_FACTOR * (RSSI_MIN_THRESHOLD - 5) + (1.0f - RSSI_SMOOTHING_FACTOR) * g_smoothedRSSI;
            if (g_smoothedRSSI > RSSI_MIN_THRESHOLD) g_smoothedRSSI = RSSI_MIN_THRESHOLD - 1;
        }
        g_smoothedRSSI = fmax(RSSI_MIN_THRESHOLD - 5, fmin(RSSI_MAX_THRESHOLD, g_smoothedRSSI));

        // Render LEDs based on router visibility and smoothed RSSI
        time_val_rad = (float)millis() / 1000.0f * TWO_PI;

        if (localRouterIsVisible_val && g_smoothedRSSI >= RSSI_MIN_THRESHOLD) {
            // Router is found, display RSSI bar
            int numSlotsToLight = map(round(g_smoothedRSSI), RSSI_MIN_THRESHOLD, RSSI_MAX_THRESHOLD, 1, NUM_RSSI_SLOTS);
            numSlotsToLight = constrain(numSlotsToLight, 0, NUM_RSSI_SLOTS);

            uint16_t currentHue = map(round(g_smoothedRSSI), RSSI_MIN_THRESHOLD, RSSI_MAX_THRESHOLD, HUE_WEAK_SIGNAL, HUE_STRONG_SIGNAL);
            currentHue = constrain(currentHue, min(HUE_WEAK_SIGNAL, HUE_STRONG_SIGNAL), max(HUE_WEAK_SIGNAL, HUE_STRONG_SIGNAL));

            uint8_t breathValue = RSSI_BAR_BREATH_MAX_V;
            if (ENABLE_RSSI_BAR_BREATHING) {
                float breathFactor = (sinf(time_val_rad * RSSI_BAR_BREATH_SPEED) + 1.0f) / 2.0f;
                breathValue = RSSI_BAR_BREATH_MIN_V + (uint8_t)(breathFactor * (RSSI_BAR_BREATH_MAX_V - RSSI_BAR_BREATH_MIN_V));
            }

            for (int i = 0; i < NUM_RSSI_SLOTS; ++i) {
                if (i < numSlotsToLight) {
                    uint32_t color = strip.ColorHSV(currentHue, 255, breathValue);
                    strip.setPixelColor(i, color);
                    strip.setPixelColor(i + NUM_RSSI_SLOTS, color);
                } else {
                    strip.setPixelColor(i, strip.Color(UNLIT_RSSI_SLOT_R, UNLIT_RSSI_SLOT_G, UNLIT_RSSI_SLOT_B));
                    strip.setPixelColor(i + NUM_RSSI_SLOTS, strip.Color(UNLIT_RSSI_SLOT_R, UNLIT_RSSI_SLOT_G, UNLIT_RSSI_SLOT_B));
                }
            }
        } else {
            // Router not found, display "searching" or "not found" animation
            if (ENABLE_NOT_FOUND_EFFECT) {
                if (strcmp(NOT_FOUND_EFFECT_TYPE, "PULSE_ALL_RED") == 0) {
                    float pulseFactor = (sinf(time_val_rad * NOT_FOUND_PULSE_SPEED) + 1.0f) / 2.0f;
                    uint8_t pulseValue = NOT_FOUND_PULSE_MIN_V + (uint8_t)(pulseFactor * (NOT_FOUND_PULSE_MAX_V - NOT_FOUND_PULSE_MIN_V));
                    uint32_t pulseColor = strip.ColorHSV(HUE_WEAK_SIGNAL, 255, pulseValue);
                    for (int i = 0; i < NUM_LEDS; ++i) {
                        strip.setPixelColor(i, pulseColor);
                    }
                }
            } else {
                for (int i = 0; i < NUM_LEDS; ++i) {
                    strip.setPixelColor(i, strip.Color(0, 0, 0));
                }
            }
        }
        strip.show();
    }
}
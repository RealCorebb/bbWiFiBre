#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>
#include <string.h> // For memcpy and memcmp
#include <math.h>   // For fmin, fmax, sinf

// --- Network Configuration ---
#define LED_PIN 10
#define MAX_CLIENT_DEVICES 13
#define NUM_LEDS (MAX_CLIENT_DEVICES * 2)

const uint8_t WIFI_CHANNEL = 13; // << SET YOUR ROUTER'S 2.4GHz WIFI CHANNEL HERE >>
// << REPLACE WITH YOUR ROUTER'S ACTUAL MAC ADDRESS >>
const uint8_t ROUTER_MAC[6] = {0xDC, 0xD8, 0x7C, 0x5C, 0x32, 0x7D};

// --- General LED & Task Config ---
const int LED_BRIGHTNESS = 200;
const TickType_t LED_UPDATE_INTERVAL_MS = 20;
const unsigned long DEVICE_TIMEOUT_MS = 90000;

//==================================================================================
// >>> TUNING PARAMETERS for Activity Sensitivity & Visualization <<<
//==================================================================================
const float MAX_ACTIVITY_SCORE = 150.0f;
const float ACTIVITY_DECAY_FACTOR = 0.92f;
const float PACKET_SIZE_DIVISOR_FOR_BOOST = 2.0f;
const float BASE_BOOST_EXISTING_DEVICE = 15.0f;
const float BASE_BOOST_NEW_DEVICE = 30.0f;

const uint8_t BRIGHTNESS_BASE_ACTIVE = 40;
const uint8_t BRIGHTNESS_SCALING_FACTOR = 215;

const float THRESHOLD_INTENSITY_VERY_LOW = 0.05f;
const float THRESHOLD_INTENSITY_LOW_TO_MID = 0.15f;
const float THRESHOLD_INTENSITY_MID_TO_HIGH = 0.50f;
const float THRESHOLD_INTENSITY_FORCE_RED = 0.85f;

const bool  ENABLE_VERY_LOW_BRIGHTNESS_OVERRIDE = true;
const uint8_t BRIGHTNESS_VERY_LOW_BASE_OVERRIDE = 50;
const float BRIGHTNESS_VERY_LOW_SCALE_OVERRIDE = 50.0f;

const uint16_t HUE_VERY_LOW      = 43690; // Dim Blue/Purple
const uint16_t HUE_LOW_START     = 43690; // Blue
const uint16_t HUE_LOW_END       = 32768; // Cyan
const uint16_t HUE_MID_START     = 32768; // Cyan
const uint16_t HUE_MID_END       = 21845; // Green
const uint16_t HUE_HIGH_START    = 21845; // Green
const uint16_t HUE_HIGH_END      = 0;     // Red
const uint16_t HUE_FORCE_RED     = 0;

// --- Idle LED Effect Tuning (for unused device slots) ---
const bool ENABLE_IDLE_EFFECT = true;
// Speed of the breathing effect (e.g., 0.5f for slow, 2.0f for faster)
const float IDLE_BREATH_SPEED = 1.0f;
// Phase offset per LED column for the breathing effect, creates a gentle wave. Radians.
const float IDLE_COLUMN_PHASE_OFFSET = 0.5f;
// Min and Max brightness for the idle breathing effect (0-255)
const uint8_t IDLE_MIN_BRIGHTNESS = 10;
const uint8_t IDLE_MAX_BRIGHTNESS = 35; // Keep it dim to not overpower active LEDs
// Base colors for idle LEDs (RGB). These will be brightness modulated.
const uint8_t IDLE_R1 = 5, IDLE_G1 = 0, IDLE_B1 = 20;   // For first row of idle slot (e.g., dim purple)
const uint8_t IDLE_R2 = 0, IDLE_G2 = 5, IDLE_B2 = 20;   // For second row of idle slot (e.g., dim blue/teal)
//==================================================================================

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

struct ieee80211_hdr_minimal_t {
    uint16_t frame_control;
    uint16_t duration_id;
    uint8_t addr1[6]; 
    uint8_t addr2[6]; 
    uint8_t addr3[6]; 
} __attribute__((packed));

struct DeviceInfo {
    uint8_t mac[6];
    volatile float activityScoreToAP;
    volatile float activityScoreFromAP;
    unsigned long lastSeenTime;
    bool isActive;
};

DeviceInfo trackedClientDevices[MAX_CLIENT_DEVICES];
SemaphoreHandle_t deviceDataMutex;

// --- Helper function to scale a base RGB color by a brightness value (0-255) ---
uint32_t scaleColorBrightness(uint8_t r_base, uint8_t g_base, uint8_t b_base, uint8_t brightness_scaler) {
    float factor = (float)brightness_scaler / 255.0f;
    uint8_t r = (uint8_t)(r_base * factor);
    uint8_t g = (uint8_t)(g_base * factor);
    uint8_t b = (uint8_t)(b_base * factor);
    return strip.Color(r, g, b);
}

// --- WiFi Packet Handler (Mostly Unchanged) ---
void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_DATA) {
        return; 
    }

    const wifi_promiscuous_pkt_t *promiscuous_packet = (wifi_promiscuous_pkt_t *)buff;
    const struct ieee80211_hdr_minimal_t *header = (const struct ieee80211_hdr_minimal_t *)promiscuous_packet->payload;

    bool toDS   = (header->frame_control & (1 << 8)) != 0; 
    bool fromDS = (header->frame_control & (1 << 9)) != 0;

    const uint8_t *client_mac_ptr = nullptr;
    bool isToAPTraffic = false;
    bool isFromAPTraffic = false;
    uint16_t packet_length = promiscuous_packet->rx_ctrl.sig_len;

    if (toDS && !fromDS) {
        if (memcmp(header->addr1, ROUTER_MAC, 6) == 0) {
            client_mac_ptr = header->addr2;
            isToAPTraffic = true;
        }
    } else if (!toDS && fromDS) {
        if (memcmp(header->addr2, ROUTER_MAC, 6) == 0) {
            client_mac_ptr = header->addr1;
            isFromAPTraffic = true;
        }
    } else { return; }

    if (client_mac_ptr == nullptr || memcmp(client_mac_ptr, ROUTER_MAC, 6) == 0) { return; }
    if (client_mac_ptr[0] == 0xFF || client_mac_ptr[0] == 0x01 || client_mac_ptr[0] == 0x33) { return; }

    if (xSemaphoreTake(deviceDataMutex, (TickType_t)0) == pdTRUE) {
        int found_device_index = -1;
        int empty_slot_index = -1;
        unsigned long oldest_device_time = millis(); 
        int oldest_device_index = 0;                 

        for (int i = 0; i < MAX_CLIENT_DEVICES; i++) {
            if (trackedClientDevices[i].isActive) {
                if (memcmp(trackedClientDevices[i].mac, client_mac_ptr, 6) == 0) {
                    found_device_index = i;
                    break; 
                }
                if (trackedClientDevices[i].lastSeenTime < oldest_device_time) {
                    oldest_device_time = trackedClientDevices[i].lastSeenTime;
                    oldest_device_index = i;
                }
            } else if (empty_slot_index == -1) {
                empty_slot_index = i; 
            }
        }

        int target_idx;
        bool is_newly_assigned_device_in_slot = false;
        if (found_device_index != -1) { 
            target_idx = found_device_index;
        } else { 
            target_idx = (empty_slot_index != -1) ? empty_slot_index : oldest_device_index;
            if (!trackedClientDevices[target_idx].isActive || memcmp(trackedClientDevices[target_idx].mac, client_mac_ptr, 6) != 0) {
                memcpy(trackedClientDevices[target_idx].mac, client_mac_ptr, 6);
                trackedClientDevices[target_idx].activityScoreToAP = 0;
                trackedClientDevices[target_idx].activityScoreFromAP = 0;
                is_newly_assigned_device_in_slot = true;
            }
        }
        
        float activity_packet_contribution = (float)packet_length / PACKET_SIZE_DIVISOR_FOR_BOOST;
        float activity_base_boost = (found_device_index != -1 && !is_newly_assigned_device_in_slot) ? BASE_BOOST_EXISTING_DEVICE : BASE_BOOST_NEW_DEVICE;
        float activity_boost = activity_packet_contribution + activity_base_boost;

        if (isToAPTraffic) {
            trackedClientDevices[target_idx].activityScoreToAP += activity_boost;
            if (trackedClientDevices[target_idx].activityScoreToAP > MAX_ACTIVITY_SCORE) {
                trackedClientDevices[target_idx].activityScoreToAP = MAX_ACTIVITY_SCORE;
            }
        } else if (isFromAPTraffic) {
            trackedClientDevices[target_idx].activityScoreFromAP += activity_boost;
            if (trackedClientDevices[target_idx].activityScoreFromAP > MAX_ACTIVITY_SCORE) {
                trackedClientDevices[target_idx].activityScoreFromAP = MAX_ACTIVITY_SCORE;
            }
        }
        trackedClientDevices[target_idx].lastSeenTime = millis();
        trackedClientDevices[target_idx].isActive = true;

        xSemaphoreGive(deviceDataMutex);
    }
}

// --- Helper function to get LED color based on activity (Unchanged) ---
uint32_t getActivityColor(float activityScore, float maxScoreToNormalizeAgainst) {
    if (activityScore < 1.0f) return strip.Color(0,0,0); 

    float normalized_intensity = fmin(1.0f, activityScore / maxScoreToNormalizeAgainst); 
    
    uint8_t brightness_val = BRIGHTNESS_BASE_ACTIVE + (uint8_t)(normalized_intensity * BRIGHTNESS_SCALING_FACTOR); 
    brightness_val = fmax(BRIGHTNESS_BASE_ACTIVE, fmin((uint8_t)255, brightness_val));

    uint16_t hue;

    if (normalized_intensity < THRESHOLD_INTENSITY_VERY_LOW) {
        hue = HUE_VERY_LOW;
        if (ENABLE_VERY_LOW_BRIGHTNESS_OVERRIDE) {
             brightness_val = BRIGHTNESS_VERY_LOW_BASE_OVERRIDE + 
                             (uint8_t)((normalized_intensity / THRESHOLD_INTENSITY_VERY_LOW) * BRIGHTNESS_VERY_LOW_SCALE_OVERRIDE);
             brightness_val = fmax(BRIGHTNESS_VERY_LOW_BASE_OVERRIDE, fmin((uint8_t)255, brightness_val));
        }
    } else if (normalized_intensity < THRESHOLD_INTENSITY_LOW_TO_MID) {
        float factor = (normalized_intensity - THRESHOLD_INTENSITY_VERY_LOW) / (THRESHOLD_INTENSITY_LOW_TO_MID - THRESHOLD_INTENSITY_VERY_LOW);
        hue = HUE_LOW_START - (uint16_t)(factor * (HUE_LOW_START - HUE_LOW_END));
    } else if (normalized_intensity < THRESHOLD_INTENSITY_MID_TO_HIGH) {
        float factor = (normalized_intensity - THRESHOLD_INTENSITY_LOW_TO_MID) / (THRESHOLD_INTENSITY_MID_TO_HIGH - THRESHOLD_INTENSITY_LOW_TO_MID);
        hue = HUE_MID_START - (uint16_t)(factor * (HUE_MID_START - HUE_MID_END));
    } else { 
        float factor = (normalized_intensity - THRESHOLD_INTENSITY_MID_TO_HIGH) / (1.0f - THRESHOLD_INTENSITY_MID_TO_HIGH);
        factor = fmin(1.0f, factor); 
        hue = HUE_HIGH_START - (uint16_t)(factor * (HUE_HIGH_START - HUE_HIGH_END));
        if (normalized_intensity > THRESHOLD_INTENSITY_FORCE_RED) {
            hue = HUE_FORCE_RED; 
        }
    }
    return strip.ColorHSV(hue, 255, brightness_val);
}


// --- LED Update Task (RTOS Task) ---
void led_update_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(LED_UPDATE_INTERVAL_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 

        if (xSemaphoreTake(deviceDataMutex, portMAX_DELAY) == pdTRUE) {
            unsigned long currentTime = millis();
            for (int i = 0; i < MAX_CLIENT_DEVICES; i++) {
                if (trackedClientDevices[i].isActive) {
                    // Decay and timeout logic (mostly unchanged)
                    trackedClientDevices[i].activityScoreToAP *= ACTIVITY_DECAY_FACTOR;
                    if (trackedClientDevices[i].activityScoreToAP < 0.5f) {
                        trackedClientDevices[i].activityScoreToAP = 0;
                    }
                    trackedClientDevices[i].activityScoreFromAP *= ACTIVITY_DECAY_FACTOR;
                    if (trackedClientDevices[i].activityScoreFromAP < 0.5f) {
                        trackedClientDevices[i].activityScoreFromAP = 0;
                    }

                    if (currentTime - trackedClientDevices[i].lastSeenTime > DEVICE_TIMEOUT_MS) {
                        if (trackedClientDevices[i].activityScoreToAP == 0 && trackedClientDevices[i].activityScoreFromAP == 0) {
                             trackedClientDevices[i].isActive = false;
                        } else if (currentTime - trackedClientDevices[i].lastSeenTime > DEVICE_TIMEOUT_MS + 20000) { 
                            trackedClientDevices[i].isActive = false;
                        }
                    }
                    
                    if (!trackedClientDevices[i].isActive) { 
                        trackedClientDevices[i].activityScoreToAP = 0;
                        trackedClientDevices[i].activityScoreFromAP = 0;
                    }

                    // Set colors for active device based on its scores
                    uint32_t colorToAP = getActivityColor(trackedClientDevices[i].activityScoreToAP, MAX_ACTIVITY_SCORE);
                    strip.setPixelColor(i, colorToAP);

                    uint32_t colorFromAP = getActivityColor(trackedClientDevices[i].activityScoreFromAP, MAX_ACTIVITY_SCORE);
                    strip.setPixelColor(i + MAX_CLIENT_DEVICES, colorFromAP);

                } else { 
                    // **** THIS IS THE NEW IDLE EFFECT LOGIC ****
                    if (ENABLE_IDLE_EFFECT) {
                        
                        float time_val_rad = (float)millis() / 1000.0f * IDLE_BREATH_SPEED; // Time in radians for sine wave
                        // Calculate a smoothly varying brightness (0.0 to 1.0) using sine
                        float breath_normalized_brightness = (sinf(time_val_rad + (float)i * IDLE_COLUMN_PHASE_OFFSET) + 1.0f) / 2.0f;
                        uint8_t current_breath_brightness = IDLE_MIN_BRIGHTNESS + (uint8_t)(breath_normalized_brightness * (IDLE_MAX_BRIGHTNESS - IDLE_MIN_BRIGHTNESS));
                        
                        // Apply to first row idle LED for this slot
                        uint32_t idle_color_row1 = scaleColorBrightness(IDLE_R1, IDLE_G1, IDLE_B1, current_breath_brightness);
                        strip.setPixelColor(i, idle_color_row1); 

                        // Apply to second row idle LED for this slot (can use same brightness or a variation)
                        // For a slightly different feel, let's make the second row's phase slightly offset from the column's main phase
                        float breath_normalized_brightness_r2 = (sinf(time_val_rad + (float)i * IDLE_COLUMN_PHASE_OFFSET + 0.3f) + 1.0f) / 2.0f; // Small extra phase for row2
                        uint8_t current_breath_brightness_r2 = IDLE_MIN_BRIGHTNESS + (uint8_t)(breath_normalized_brightness_r2 * (IDLE_MAX_BRIGHTNESS - IDLE_MIN_BRIGHTNESS));
                        uint32_t idle_color_row2 = scaleColorBrightness(IDLE_R2, IDLE_G2, IDLE_B2, current_breath_brightness_r2);
                        strip.setPixelColor(i + MAX_CLIENT_DEVICES, idle_color_row2);
                    } else {
                        // If idle effect is disabled, turn them off
                        strip.setPixelColor(i, strip.Color(0, 0, 0)); 
                        strip.setPixelColor(i + MAX_CLIENT_DEVICES, strip.Color(0, 0, 0));
                    }
                }
            }
            xSemaphoreGive(deviceDataMutex);
        }
        strip.show();
    }
}

// --- Arduino Setup Function (Mostly Unchanged) ---
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); } 
    Serial.println("Starting Configurable WiFi Home Device Activity Monitor (Dual Row with Idle Effect)...");
    Serial.println("--- Current Tuning Parameters ---");
    Serial.printf("MAX_ACTIVITY_SCORE: %.2f\n", MAX_ACTIVITY_SCORE);
    Serial.printf("IDLE_BREATH_SPEED: %.2f\n", IDLE_BREATH_SPEED);
    Serial.println("-------------------------------");

    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.show(); 

    for (int i = 0; i < MAX_CLIENT_DEVICES; i++) {
        trackedClientDevices[i].isActive = false;
        trackedClientDevices[i].activityScoreToAP = 0;
        trackedClientDevices[i].activityScoreFromAP = 0;
        memset(trackedClientDevices[i].mac, 0, 6); 
    }

    deviceDataMutex = xSemaphoreCreateMutex();
    if (deviceDataMutex == NULL) {
        Serial.println("Error: Mutex creation failed!");
        while (1) { delay(1000); } 
    }

    WiFi.mode(WIFI_STA); 
    WiFi.disconnect();   

    esp_err_t promiscuous_err = esp_wifi_set_promiscuous(true);
    if (promiscuous_err != ESP_OK) { Serial.printf("Error promiscuous: %s\n", esp_err_to_name(promiscuous_err)); return; }
    esp_err_t channel_err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (channel_err != ESP_OK) { Serial.printf("Error channel: %s\n", esp_err_to_name(channel_err)); return; }
    esp_err_t cb_err = esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_packet_handler);
    if (cb_err != ESP_OK) { Serial.printf("Error RX CB: %s\n", esp_err_to_name(cb_err)); return; }
    
    Serial.printf("Sniffing on WiFi Channel %d.\n", WIFI_CHANNEL);
    Serial.print("Monitoring traffic with Router MAC: ");
    for(int i=0; i<6; ++i) { Serial.printf("%02X", ROUTER_MAC[i]); if(i<5) Serial.print(":"); }
    Serial.println();

    xTaskCreatePinnedToCore(
        led_update_task, "LEDUpdateTask", 4096, NULL, 2, NULL, tskNO_AFFINITY);    
}

// --- Arduino Loop Function ---
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000)); 
}
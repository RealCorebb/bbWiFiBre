// --- General LED & Task Config ---
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

const bool ENABLE_VERY_LOW_BRIGHTNESS_OVERRIDE = true;
const uint8_t BRIGHTNESS_VERY_LOW_BASE_OVERRIDE = 50;
const float BRIGHTNESS_VERY_LOW_SCALE_OVERRIDE = 50.0f;

const uint16_t HUE_VERY_LOW     = 43690; // Dim Blue/Purple
const uint16_t HUE_LOW_START    = 43690; // Blue
const uint16_t HUE_LOW_END      = 32768; // Cyan
const uint16_t HUE_MID_START    = 32768; // Cyan
const uint16_t HUE_MID_END      = 21845; // Green
const uint16_t HUE_HIGH_START   = 21845; // Green
const uint16_t HUE_HIGH_END     = 0;     // Red
const uint16_t HUE_FORCE_RED    = 0;

// --- Idle LED Effect Tuning (for unused device slots) ---
const bool ENABLE_IDLE_EFFECT = true;
const float IDLE_BREATH_SPEED = 1.5f;
const float IDLE_COLUMN_PHASE_OFFSET = 0.5f;
const uint8_t IDLE_MIN_BRIGHTNESS = 10;
const uint8_t IDLE_MAX_BRIGHTNESS = 35;
const uint8_t IDLE_R1 = 10, IDLE_G1 = 0, IDLE_B1 = 15;
const uint8_t IDLE_R2 = 0, IDLE_G2 = 10, IDLE_B2 = 15;
//==================================================================================

// --- Intruder Mode Configuration ---
const bool ENABLE_INTRUDER_MODE = true; // Set to true to enable intruder detection
const int INTRUDER_MODE_ARM_DELAY_S = 20; // Seconds after boot to arm intruder detection
const int INTRUDER_ALERT_DURATION_S = 4; // Seconds to show red alert

// --- Intruder Mode State Variables ---
volatile bool g_intruder_detection_armed = false;
volatile bool g_intruder_alert_active = false;
volatile unsigned long g_intruder_alert_start_time_ms = 0;
unsigned long g_esp_boot_time_ms = 0;
//==================================================================================

struct ieee80211_hdr_minimal_t {
    uint16_t frame_control;
    uint16_t duration_id;
    uint8_t addr1[6];
    uint8_t addr2[6];
    uint8_t addr3[6];
} __attribute__((packed));


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

// --- WiFi Packet Handler ---
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
                is_newly_assigned_device_in_slot = true; // Key flag

                // --- Intruder Detection Logic ---
                if (ENABLE_INTRUDER_MODE && g_intruder_detection_armed) {
                    if (!g_intruder_alert_active) { // Only trigger if not already alerting
                        Serial.print(">>> INTRUDER ALERT! New device detected: MAC: ");
                        for(int k=0; k<6; ++k) { Serial.printf("%02X", trackedClientDevices[target_idx].mac[k]); if(k<5) Serial.print(":"); }
                        Serial.println(" <<<");
                        g_intruder_alert_active = true;
                        g_intruder_alert_start_time_ms = millis();
                    }
                }
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

// --- Helper function to get LED color based on activity ---
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
    const unsigned long arm_delay_ms = (unsigned long)INTRUDER_MODE_ARM_DELAY_S * 1000;
    const unsigned long alert_duration_ms = (unsigned long)INTRUDER_ALERT_DURATION_S * 1000;
    bool current_cycle_is_alert = false;

    // --- Parameters for Pulsing Intruder Alert ---
    const float ALERT_PULSE_FREQUENCY_HZ = 1.5f;     // How many pulses per second (e.g., 0.5f for slow, 1.0f for moderate, 2.0f for fast)
    const uint8_t ALERT_MIN_PULSE_BRIGHTNESS = 30;   // Minimum brightness during a pulse (0-255)
    const uint8_t ALERT_MAX_PULSE_BRIGHTNESS = 255;  // Maximum brightness during a pulse (0-255)
    // PI constant, ensure math.h provides it or define it: #ifndef PI #define PI 3.1415926535f #endif

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        unsigned long currentTime = millis(); // Cache current time for this cycle

        // --- Intruder Mode Arming Logic ---
        if (ENABLE_INTRUDER_MODE && !g_intruder_detection_armed) {
            if (currentTime - g_esp_boot_time_ms > arm_delay_ms) {
                g_intruder_detection_armed = true;
                Serial.println(">>> Intruder Detection Armed <<<");
            }
        }

        // --- Intruder Alert Active Check ---
        current_cycle_is_alert = false;
        if (ENABLE_INTRUDER_MODE && g_intruder_alert_active) {
            if (currentTime - g_intruder_alert_start_time_ms < alert_duration_ms) {
                current_cycle_is_alert = true;
            } else {
                g_intruder_alert_active = false; // Alert period ended
                Serial.println(">>> Intruder Alert Ended <<<");
                // Optional: You could add a short fade-out effect here for a few cycles
                // before fully reverting to normal display. For now, it will switch back directly.
            }
        }

        if (xSemaphoreTake(deviceDataMutex, portMAX_DELAY) == pdTRUE) {
            if (current_cycle_is_alert) {
                // --- Show Pulsing Intruder Alert Effect ---
                unsigned long time_since_alert_start = currentTime - g_intruder_alert_start_time_ms;
                
                // Calculate angular frequency for the sine wave based on desired Hz
                float pulse_angular_frequency = 2.0f * PI * ALERT_PULSE_FREQUENCY_HZ;
                
                // Calculate normalized brightness (0.0 to 1.0) using a sine wave.
                // The "- (PI / 2.0f)" phase shift makes the sine wave start at -1 (its minimum),
                // so the pulse starts from its dimmest and brightens up.
                float normalized_brightness = (sinf( (float)time_since_alert_start / 1000.0f * pulse_angular_frequency - (PI / 2.0f) ) + 1.0f) / 2.0f;
                
                // Scale normalized brightness to the desired min/max range
                uint8_t current_pulse_brightness = ALERT_MIN_PULSE_BRIGHTNESS + 
                                                   (uint8_t)(normalized_brightness * (ALERT_MAX_PULSE_BRIGHTNESS - ALERT_MIN_PULSE_BRIGHTNESS));

                for (int i = 0; i < NUM_LEDS; i++) {
                    strip.setPixelColor(i, strip.Color(current_pulse_brightness, 0, 0)); // Pulsing Red
                }
            } else {
                // --- Normal LED Update Logic (remains the same as before) ---
                for (int i = 0; i < MAX_CLIENT_DEVICES; i++) {
                    if (trackedClientDevices[i].isActive) {
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

                        uint32_t colorToAP = getActivityColor(trackedClientDevices[i].activityScoreToAP, MAX_ACTIVITY_SCORE);
                        strip.setPixelColor(i, colorToAP);

                        uint32_t colorFromAP = getActivityColor(trackedClientDevices[i].activityScoreFromAP, MAX_ACTIVITY_SCORE);
                        strip.setPixelColor(i + MAX_CLIENT_DEVICES, colorFromAP);

                    } else {
                        if (ENABLE_IDLE_EFFECT) {
                            float time_val_rad = (float)millis() / 1000.0f * IDLE_BREATH_SPEED; // millis() is okay here, it's for animation phase
                            float breath_normalized_brightness = (sinf(time_val_rad + (float)i * IDLE_COLUMN_PHASE_OFFSET) + 1.0f) / 2.0f;
                            uint8_t current_breath_brightness = IDLE_MIN_BRIGHTNESS + (uint8_t)(breath_normalized_brightness * (IDLE_MAX_BRIGHTNESS - IDLE_MIN_BRIGHTNESS));
                            uint32_t idle_color_row1 = scaleColorBrightness(IDLE_R1, IDLE_G1, IDLE_B1, current_breath_brightness);
                            strip.setPixelColor(i, idle_color_row1);

                            float breath_normalized_brightness_r2 = (sinf(time_val_rad + (float)i * IDLE_COLUMN_PHASE_OFFSET + 0.3f) + 1.0f) / 2.0f;
                            uint8_t current_breath_brightness_r2 = IDLE_MIN_BRIGHTNESS + (uint8_t)(breath_normalized_brightness_r2 * (IDLE_MAX_BRIGHTNESS - IDLE_MIN_BRIGHTNESS));
                            uint32_t idle_color_row2 = scaleColorBrightness(IDLE_R2, IDLE_G2, IDLE_B2, current_breath_brightness_r2);
                            strip.setPixelColor(i + MAX_CLIENT_DEVICES, idle_color_row2);
                        } else {
                            strip.setPixelColor(i, strip.Color(0, 0, 0));
                            strip.setPixelColor(i + MAX_CLIENT_DEVICES, strip.Color(0, 0, 0));
                        }
                    }
                }
            }
            xSemaphoreGive(deviceDataMutex);
        }
        strip.show();
    }
}

#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>
#include <string.h> // For memcpy and memcmp

// --- Configuration ---
#define LED_PIN 10             // GPIO pin connected to the NeoPixel data input
#define NUM_LEDS 26            // Total LEDs, each representing a device
const uint8_t WIFI_CHANNEL = 6; // << SET YOUR ROUTER'S 2.4GHz WIFI CHANNEL HERE >>

// << REPLACE WITH YOUR ROUTER'S ACTUAL MAC ADDRESS >>
const uint8_t ROUTER_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

#define MAX_DEVICES NUM_LEDS // Maximum number of unique devices we can track and display

// --- LED & Task Config ---
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
const int LED_BRIGHTNESS = 70;                // Global brightness for the NeoPixel strip (0-255)
const TickType_t LED_UPDATE_INTERVAL_MS = 50; // Update LEDs at 20Hz (50ms interval)
const float ACTIVITY_DECAY_FACTOR = 0.92;     // How quickly activity score decays (e.g., 0.90 = 10% decay per interval)
const unsigned long DEVICE_TIMEOUT_MS = 90000;// Forget device after 1.5 minutes of inactivity
const float MAX_ACTIVITY_SCORE = 1000.0f;     // Arbitrary cap for activity score to normalize visualization

// --- Structures ---
// Simplified 802.11 MAC header structure
struct ieee80211_hdr_minimal_t {
    uint16_t frame_control;
    uint16_t duration_id;
    uint8_t addr1[6]; // Receiver MAC Address (or Destination MAC)
    uint8_t addr2[6]; // Transmitter MAC Address (or Source MAC)
    uint8_t addr3[6]; // Filtering Address (often BSSID, or DA/SA depending on To/From DS)
    // Other fields follow but are not needed for this basic MAC extraction
} __attribute__((packed)); // Ensures compiler doesn't add padding

// Structure to hold information about each tracked device
struct DeviceInfo {
    uint8_t mac[6];              // MAC address of the client device
    volatile float activityScore;// Smoothed activity metric for visualization
    unsigned long lastSeenTime;  // Timestamp of the last packet seen from/to this device
    bool isActive;               // Flag indicating if this device slot is currently in use
                                 // The LED index is implicitly the array index of this struct
};

DeviceInfo trackedDevices[MAX_DEVICES]; // Array to store information for all trackable devices
SemaphoreHandle_t deviceDataMutex;      // Mutex to protect shared access to trackedDevices array

// --- WiFi Packet Handler ---
// This function is called by the ESP32 WiFi driver for each captured packet
void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type) {
    // We are primarily interested in Data frames for user activity
    // WIFI_PKT_MGMT can also be interesting (e.g. associations) but adds complexity
    if (type != WIFI_PKT_DATA) {
        return; // Ignore non-Data packets for this visualizer
    }

    const wifi_promiscuous_pkt_t *promiscuous_packet = (wifi_promiscuous_pkt_t *)buff;
    const struct ieee80211_hdr_minimal_t *header = (const struct ieee80211_hdr_minimal_t *)promiscuous_packet->payload;

    // Extract ToDS and FromDS bits from the Frame Control field
    // These bits tell us the direction of the packet relative to the Distribution System (DS), i.e., your Access Point/Router
    // Frame Control field: Bit 8 = ToDS, Bit 9 = FromDS (0-indexed)
    bool toDS   = (header->frame_control & (1 << 8)) != 0; // Packet is going TO the AP/router
    bool fromDS = (header->frame_control & (1 << 9)) != 0; // Packet is coming FROM the AP/router

    const uint8_t *client_mac_ptr = nullptr;
    uint16_t packet_length = promiscuous_packet->rx_ctrl.sig_len; // Length of the received packet (signal part)

    // Identify client MAC based on packet direction
    if (toDS && !fromDS) {
        // Packet from a client STA to the AP (Router)
        // Addr1 = BSSID (Router's MAC), Addr2 = Source Address (Client's MAC)
        if (memcmp(header->addr1, ROUTER_MAC, 6) == 0) {
            client_mac_ptr = header->addr2;
        }
    } else if (!toDS && fromDS) {
        // Packet from the AP (Router) to a client STA
        // Addr1 = Destination Address (Client's MAC), Addr2 = BSSID (Router's MAC)
        if (memcmp(header->addr2, ROUTER_MAC, 6) == 0) {
            client_mac_ptr = header->addr1;
        }
    } else {
        // Other cases (e.g. Ad-hoc mode ToDS=0,FromDS=0 or WDS bridge ToDS=1,FromDS=1)
        // We ignore these for typical home router setups.
        return;
    }

    // Validate the identified client MAC
    if (client_mac_ptr == nullptr || memcmp(client_mac_ptr, ROUTER_MAC, 6) == 0) {
        // Not a valid client MAC (or it's the router itself)
        return;
    }
    // Filter out broadcast (FF:FF:FF:FF:FF:FF) and common multicast MAC prefixes
    if (client_mac_ptr[0] == 0xFF || client_mac_ptr[0] == 0x01 || client_mac_ptr[0] == 0x33) {
        return;
    }

    // --- Update device activity ---
    // Attempt to take the mutex (non-blocking: 0 timeout).
    // If busy, we might skip this packet to keep the callback fast.
    if (xSemaphoreTake(deviceDataMutex, (TickType_t)0) == pdTRUE) {
        int found_device_index = -1;
        int empty_slot_index = -1;
        unsigned long oldest_device_time = millis(); // Initialize with current time for comparison
        int oldest_device_index = 0;                 // Default to first slot if all are active

        // Try to find the device or an empty/old slot
        for (int i = 0; i < MAX_DEVICES; i++) {
            if (trackedDevices[i].isActive) {
                if (memcmp(trackedDevices[i].mac, client_mac_ptr, 6) == 0) {
                    found_device_index = i;
                    break; // Device found
                }
                // Keep track of the least recently seen active device
                if (trackedDevices[i].lastSeenTime < oldest_device_time) {
                    oldest_device_time = trackedDevices[i].lastSeenTime;
                    oldest_device_index = i;
                }
            } else if (empty_slot_index == -1) {
                empty_slot_index = i; // Found an inactive slot
            }
        }

        if (found_device_index != -1) { // Device is already tracked
            // Increase activity score based on packet length (heuristic)
            trackedDevices[found_device_index].activityScore += (packet_length / 20.0f) + 5.0f;
            if (trackedDevices[found_device_index].activityScore > MAX_ACTIVITY_SCORE) {
                trackedDevices[found_device_index].activityScore = MAX_ACTIVITY_SCORE;
            }
            trackedDevices[found_device_index].lastSeenTime = millis();
        } else { // New device or device to replace
            int target_idx_for_new_device = (empty_slot_index != -1) ? empty_slot_index : oldest_device_index;
            
            memcpy(trackedDevices[target_idx_for_new_device].mac, client_mac_ptr, 6);
            trackedDevices[target_idx_for_new_device].activityScore = (packet_length / 20.0f) + 20.0f; // Initial higher score
             if (trackedDevices[target_idx_for_new_device].activityScore > MAX_ACTIVITY_SCORE) {
                trackedDevices[target_idx_for_new_device].activityScore = MAX_ACTIVITY_SCORE;
            }
            trackedDevices[target_idx_for_new_device].lastSeenTime = millis();
            trackedDevices[target_idx_for_new_device].isActive = true;
        }
        xSemaphoreGive(deviceDataMutex);
    }
}

// --- LED Update Task (RTOS Task) ---
void led_update_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(LED_UPDATE_INTERVAL_MS);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // precise timing

        if (xSemaphoreTake(deviceDataMutex, portMAX_DELAY) == pdTRUE) {
            unsigned long currentTime = millis();
            for (int i = 0; i < MAX_DEVICES; i++) {
                if (trackedDevices[i].isActive) {
                    // Decay activity score
                    trackedDevices[i].activityScore *= ACTIVITY_DECAY_FACTOR;
                    if (trackedDevices[i].activityScore < 0.5f) { // Threshold to consider it zero
                        trackedDevices[i].activityScore = 0;
                    }

                    // Check for device timeout
                    if (currentTime - trackedDevices[i].lastSeenTime > DEVICE_TIMEOUT_MS) {
                        trackedDevices[i].isActive = false;
                        trackedDevices[i].activityScore = 0;
                        // Optionally clear MAC: memset(trackedDevices[i].mac, 0, 6);
                    }

                    // Determine LED color based on activity score
                    uint32_t ledColor;
                    if (trackedDevices[i].activityScore == 0 && !trackedDevices[i].isActive) { // Ensure fully inactive is off
                        ledColor = strip.Color(0, 0, 0); // Off
                    } else {
                        // Normalize intensity from 0.0 to 1.0 for color calculation
                        float normalized_intensity = fmin(1.0f, trackedDevices[i].activityScore / (MAX_ACTIVITY_SCORE * 0.75f)); // Adjust scaling factor
                        uint8_t brightness_val = 10 + (uint8_t)(normalized_intensity * 245); // Base brightness + scaled

                        if (normalized_intensity < 0.01f && trackedDevices[i].activityScore > 0) { // Barely active
                             ledColor = strip.ColorHSV(43690, 200, 50); // Dim purple/blue
                        } else if (normalized_intensity < 0.2f) { // Low activity: Blueish
                            // Hue: Blue (~43690) to Cyan (~32768)
                            uint16_t hue = 43690 - (uint16_t)(normalized_intensity * 5.0f * (43690 - 32768));
                            ledColor = strip.ColorHSV(hue, 255, brightness_val);
                        } else if (normalized_intensity < 0.6f) { // Medium activity: Cyan -> Green
                            // Hue: Cyan (~32768) to Green (~21845)
                            uint16_t hue = 32768 - (uint16_t)(( (normalized_intensity - 0.2f) / 0.4f ) * (32768 - 21845));
                            ledColor = strip.ColorHSV(hue, 255, brightness_val);
                        } else { // High activity: Green -> Yellow -> Red
                            // Hue: Green (~21845) towards Red (0/65535)
                             uint16_t hue = 21845 - (uint16_t)(( (normalized_intensity - 0.6f) / 0.4f ) * (21845 - 5000)); // Stop at orange/yellow for visibility
                            if (normalized_intensity > 0.9) hue = 0; // Full red for max
                            ledColor = strip.ColorHSV(hue, 255, brightness_val);
                        }
                    }
                    strip.setPixelColor(i, ledColor);

                } else { // Slot is not active
                    strip.setPixelColor(i, strip.Color(0, 0, 0)); // Off
                }
            }
            xSemaphoreGive(deviceDataMutex);
        }
        strip.show();
    }
}

// --- Arduino Setup Function ---
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); } // Wait for Serial Monitor
    Serial.println("Starting WiFi Home Device Activity Monitor...");

    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS);
    strip.show(); // Initialize all LEDs to off

    // Initialize trackedDevices array
    for (int i = 0; i < MAX_DEVICES; i++) {
        trackedDevices[i].isActive = false;
        trackedDevices[i].activityScore = 0;
        memset(trackedDevices[i].mac, 0, 6); // Clear MAC addresses
    }

    deviceDataMutex = xSemaphoreCreateMutex();
    if (deviceDataMutex == NULL) {
        Serial.println("Error: Mutex creation failed!");
        while (1) { delay(1000); } // Halt
    }

    // Configure WiFi for promiscuous mode
    WiFi.mode(WIFI_STA); // Station mode is a prerequisite for promiscuous mode
    WiFi.disconnect();   // We don't want to connect to any AP

    esp_err_t promiscuous_err = esp_wifi_set_promiscuous(true);
    if (promiscuous_err != ESP_OK) {
        Serial.printf("Error setting promiscuous mode: %s\n", esp_err_to_name(promiscuous_err));
        return;
    }
    esp_err_t channel_err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
     if (channel_err != ESP_OK) {
        Serial.printf("Error setting WiFi channel: %s\n", esp_err_to_name(channel_err));
        return;
    }
    esp_err_t cb_err = esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_packet_handler);
    if (cb_err != ESP_OK) {
        Serial.printf("Error setting promiscuous RX callback: %s\n", esp_err_to_name(cb_err));
        return;
    }
    
    Serial.printf("Sniffing on WiFi Channel %d.\n", WIFI_CHANNEL);
    Serial.print("Monitoring traffic with Router MAC: ");
    for(int i=0; i<6; ++i) { Serial.printf("%02X", ROUTER_MAC[i]); if(i<5) Serial.print(":"); }
    Serial.println();

    // Create the LED update task
    xTaskCreatePinnedToCore(
        led_update_task,    // Function to implement the task
        "LEDUpdateTask",    // Name of the task
        4096,               // Stack size in words (increased for float math and NeoPixel)
        NULL,               // Task input parameter
        2,                  // Priority of the task (0 is lowest)
        NULL,               // Task handle (optional)
        tskNO_AFFINITY);    // Run on any core (or specify 0 or 1)
    
    // If you don't use the main loop() for anything, you can delete its task
    // vTaskDelete(NULL); 
}

// --- Arduino Loop Function ---
void loop() {
    // The main work is done in the RTOS task and WiFi callback.
    // This loop can be kept minimal or used for other non-critical tasks.
    vTaskDelay(pdMS_TO_TICKS(1000)); // Yield CPU
}

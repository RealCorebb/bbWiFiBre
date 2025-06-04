#include "bbWiFiBre.h"
#include <Preferences.h>

// Mode definitions
#define MODE_MONITOR_AP 1
#define MODE_SCAN_CHANNEL 2
#define MODE_RSSI 3

// Global variables
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
volatile int currentMode = MODE_MONITOR_AP;
TaskHandle_t modeTaskHandle = NULL;
TaskHandle_t channelTaskHandle = NULL;  // Added for mode2's second task

Preferences preferences;

void saveCurrentMode() {
    preferences.begin("appState", false);
    preferences.putInt("lastMode", currentMode);
    preferences.end();
    Serial.printf("Saved mode %d to NVS.\n", currentMode);
}

void stopCurrentMode() {
    Serial.println("Stopping current mode...");
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(NULL);
    Serial.println("WiFi promiscuous mode stopped.");

    int modeToClean = currentMode; // Capture the mode that is being stopped

    if (modeTaskHandle != NULL) {
        Serial.printf("Deleting modeTaskHandle (was for mode %d)\n", modeToClean);
        vTaskDelete(modeTaskHandle);
        modeTaskHandle = NULL;
    }
    if (channelTaskHandle != NULL) { // Specific to MODE_SCAN_CHANNEL
        Serial.printf("Deleting channelTaskHandle (was for mode %d)\n", modeToClean);
        vTaskDelete(channelTaskHandle);
        channelTaskHandle = NULL;
    }

    Serial.println("Delaying for task cleanup (500ms)...");
    vTaskDelay(pdMS_TO_TICKS(250));

    strip.clear();
    strip.show();
    Serial.println("LEDs cleared.");

    if (modeToClean == MODE_MONITOR_AP && deviceDataMutex != NULL) {
        Serial.println("Deleting deviceDataMutex");
        vSemaphoreDelete(deviceDataMutex);
        deviceDataMutex = NULL;
    } else if (modeToClean == MODE_SCAN_CHANNEL && xSemaphore != NULL) {
        Serial.println("Deleting xSemaphore for channel scan");
        vSemaphoreDelete(xSemaphore);
        xSemaphore = NULL;
    } else if (modeToClean == MODE_RSSI && g_rssiDataMutex != NULL) {
        Serial.println("Deleting g_rssiDataMutex");
        vSemaphoreDelete(g_rssiDataMutex);
        g_rssiDataMutex = NULL;
    }
    Serial.println("Semaphores deleted (if any). Mode stopping sequence complete.");
}

void startMonitorAPMode() {
    Serial.println("Initializing Monitor AP Mode...");
    stopCurrentMode(); // Ensure any previous mode is fully stopped
    currentMode = MODE_MONITOR_AP;

    deviceDataMutex = xSemaphoreCreateMutex();
    if (deviceDataMutex == NULL) {
        Serial.println("Failed to create deviceDataMutex");
        return;
    }

    g_esp_boot_time_ms = millis();
    g_intruder_detection_armed = false;
    g_intruder_alert_active = false;
    for (int i = 0; i < MAX_CLIENT_DEVICES; i++) {
        trackedClientDevices[i].isActive = false;
        trackedClientDevices[i].activityScoreToAP = 0;
        trackedClientDevices[i].activityScoreFromAP = 0;
        memset(trackedClientDevices[i].mac, 0, 6);
    }

    esp_err_t err = esp_wifi_set_promiscuous(true);
    if (err != ESP_OK) { Serial.printf("Failed to set promiscuous mode: %s\n", esp_err_to_name(err)); vSemaphoreDelete(deviceDataMutex); deviceDataMutex = NULL; return; }
    err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) { Serial.printf("Failed to set channel: %s\n", esp_err_to_name(err)); vSemaphoreDelete(deviceDataMutex); deviceDataMutex = NULL; return; }
    err = esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_packet_handler);
    if (err != ESP_OK) { Serial.printf("Failed to set callback: %s\n", esp_err_to_name(err)); vSemaphoreDelete(deviceDataMutex); deviceDataMutex = NULL; return; }

    BaseType_t xReturned = xTaskCreatePinnedToCore(led_update_task, "LEDUpdateTask", 4096, NULL, 2, &modeTaskHandle, tskNO_AFFINITY);
    if (xReturned != pdPASS) {
        Serial.println("Failed to create LED update task for Monitor AP");
        vSemaphoreDelete(deviceDataMutex); deviceDataMutex = NULL;
        return;
    }

    Serial.println("Monitor AP Mode Started Successfully");
    Serial.printf("Monitoring on Channel: %d\n", WIFI_CHANNEL);
    saveCurrentMode();
}

void startScanChannelMode() {
    Serial.println("Initializing Channel Scan Mode...");
    stopCurrentMode();
    currentMode = MODE_SCAN_CHANNEL;

    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore == NULL) {
        Serial.println("Failed to create xSemaphore for Channel Scan");
        return;
    }

    currentScanningChannel = 1;
    for (int i = 0; i < CHANNELS; i++) {
        pendingActivityIncrements[i] = 0;
        channelActivity[i] = 0;
        channelBrightness[i] = 1.0f;
        pendingColorForDelayedLed[i] = OFF_COLOR;
        framesToDelayForLed[i] = 0;
    }

    esp_err_t err = esp_wifi_set_promiscuous(true);
    if (err != ESP_OK) { Serial.printf("Failed to set promiscuous mode: %s\n", esp_err_to_name(err)); vSemaphoreDelete(xSemaphore); xSemaphore = NULL; return; }
    err = esp_wifi_set_promiscuous_rx_cb(onWiFiPacket);
    if (err != ESP_OK) { Serial.printf("Failed to set callback: %s\n", esp_err_to_name(err)); vSemaphoreDelete(xSemaphore); xSemaphore = NULL; return; }
    err = esp_wifi_set_channel(currentScanningChannel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) { Serial.printf("Failed to set channel: %s\n", esp_err_to_name(err)); vSemaphoreDelete(xSemaphore); xSemaphore = NULL; return; }

    BaseType_t xReturnedLed = xTaskCreatePinnedToCore(LEDUpdateTask, "LEDTask", 4096, NULL, 2, &modeTaskHandle, tskNO_AFFINITY);
    if (xReturnedLed != pdPASS) {
        Serial.println("Failed to create LED update task for Channel Scan");
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL;
        return;
    }

    BaseType_t xReturnedChannel = xTaskCreatePinnedToCore(ChannelTask, "ChannelTask", 4096, NULL, 1, &channelTaskHandle, tskNO_AFFINITY);
    if (xReturnedChannel != pdPASS) {
        Serial.println("Failed to create channel task for Channel Scan");
        if(modeTaskHandle != NULL) vTaskDelete(modeTaskHandle); modeTaskHandle = NULL;
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL;
        return;
    }

    Serial.println("Channel Scan Mode Started Successfully");
    saveCurrentMode();
}

void startRSSIMode() {
    Serial.println("Initializing RSSI Mode...");
    stopCurrentMode();
    currentMode = MODE_RSSI;

    g_rssiDataMutex = xSemaphoreCreateMutex();
    if (g_rssiDataMutex == NULL) {
        Serial.println("Failed to create RSSI mutex");
        return;
    }

    g_latestRSSI = -100;
    g_routerIsVisible = false;
    g_smoothedRSSI = -100.0f; // Assuming float

    BaseType_t xReturned = xTaskCreatePinnedToCore(RSSIUpdateTask, "RSSITask", 4096, NULL, 2, &modeTaskHandle, tskNO_AFFINITY);
    if (xReturned != pdPASS) {
        Serial.println("Failed to create RSSI update task");
        vSemaphoreDelete(g_rssiDataMutex); g_rssiDataMutex = NULL;
        return;
    }

    Serial.println("RSSI Mode Started Successfully");
    saveCurrentMode();
}

void serialCommandTask(void* parameter) {
    while (true) {
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            command.trim();

            if (command == "mode1") {
                if (currentMode != MODE_MONITOR_AP) {
                    startMonitorAPMode();
                } else {
                    Serial.println("Already in Monitor AP Mode");
                }
            } else if (command == "mode2") {
                if (currentMode != MODE_SCAN_CHANNEL) {
                    startScanChannelMode();
                } else {
                    Serial.println("Already in Channel Scan Mode");
                }
            } else if (command == "mode3") {
                if (currentMode != MODE_RSSI) {
                    startRSSIMode();
                } else {
                    Serial.println("Already in RSSI Mode");
                }
            } else {
                Serial.println("Invalid command. Use 'mode1', 'mode2', or 'mode3'");
            }
             vTaskDelay(pdMS_TO_TICKS(50)); // Small delay to allow mode to settle if changed
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("bbWiFiBre - Multi-mode WiFi Monitor");
    Serial.println("Commands:");
    Serial.println("  mode1 - Monitor AP Mode");
    Serial.println("  mode2 - Channel Scan Mode");
    Serial.println("  mode3 - RSSI Mode");

    strip.begin();
    strip.setBrightness(220); // Adjust brightness as needed
    strip.show();

    WiFi.mode(WIFI_STA); // Initialize WiFi stack
    WiFi.disconnect();   // Disconnect from any previous network

    preferences.begin("appState", true); // Read-only to check
    int savedMode = preferences.getInt("lastMode", MODE_MONITOR_AP);
    preferences.end();
    Serial.printf("Loaded mode %d from NVS.\n", savedMode);

    // Create a task for Serial command monitoring
    xTaskCreate(serialCommandTask, "SerialCommand", 4096, NULL, 1, NULL);

    delay(1000); // Allow serial task to initialize, etc.

    if (savedMode == MODE_MONITOR_AP) {
        startMonitorAPMode();
    } else if (savedMode == MODE_SCAN_CHANNEL) {
        startScanChannelMode();
    } else if (savedMode == MODE_RSSI) {
        startRSSIMode();
    } else {
        Serial.println("Invalid mode from NVS, defaulting to Monitor AP Mode.");
        startMonitorAPMode();
    }
}


void loop() {
    // Main loop is not used as we're using FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}
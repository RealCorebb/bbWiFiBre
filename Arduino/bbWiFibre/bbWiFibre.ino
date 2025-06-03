#include "bbWiFiBre.h"

// Mode definitions
#define MODE_MONITOR_AP 1
#define MODE_SCAN_CHANNEL 2

// Global variables
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
volatile int currentMode = MODE_MONITOR_AP;
TaskHandle_t modeTaskHandle = NULL;
TaskHandle_t channelTaskHandle = NULL;  // Added for mode2's second task

// Function declarations
void startMonitorAPMode();
void startScanChannelMode();
void stopCurrentMode();
void serialCommandTask(void* parameter);

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    
    Serial.println("bbWiFiBre - Multi-mode WiFi Monitor");
    Serial.println("Commands:");
    Serial.println("  mode1 - Monitor AP Mode");
    Serial.println("  mode2 - Channel Scan Mode");
    
    strip.begin();
    strip.setBrightness(80);
    strip.show();

    // Initialize WiFi in station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Create a task for Serial command monitoring
    xTaskCreate(
        serialCommandTask,
        "SerialCommand",
        4096,
        NULL,
        1,
        NULL
    );

    // Wait a moment for setup
    delay(1000);

    // Initial mode setup
    startMonitorAPMode();
}

void serialCommandTask(void* parameter) {
    while (true) {
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            command.trim();
            
            if (command == "mode1") {
                if (currentMode != MODE_MONITOR_AP) {
                    Serial.println("Switching to Monitor AP Mode...");
                    stopCurrentMode();
                    startMonitorAPMode();
                } else {
                    Serial.println("Already in Monitor AP Mode");
                }
            }
            else if (command == "mode2") {
                if (currentMode != MODE_SCAN_CHANNEL) {
                    Serial.println("Switching to Channel Scan Mode...");
                    stopCurrentMode();
                    startScanChannelMode();
                } else {
                    Serial.println("Already in Channel Scan Mode");
                }
            }
            else {
                Serial.println("Invalid command. Use 'mode1' or 'mode2'");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void stopCurrentMode() {
    // Stop WiFi promiscuous mode
    esp_wifi_set_promiscuous(false);
    esp_wifi_set_promiscuous_rx_cb(NULL);
    
    // Delete current mode tasks
    if (modeTaskHandle != NULL) {
        vTaskDelete(modeTaskHandle);
        modeTaskHandle = NULL;
    }
    if (channelTaskHandle != NULL) {
        vTaskDelete(channelTaskHandle);
        channelTaskHandle = NULL;
    }
    
    // Clear all LEDs
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, 0);
    }
    strip.show();
    
    // Allow time for cleanup
    vTaskDelay(pdMS_TO_TICKS(100));

    // Delete mode-specific semaphores
    if (currentMode == MODE_MONITOR_AP && deviceDataMutex != NULL) {
        vSemaphoreDelete(deviceDataMutex);
        deviceDataMutex = NULL;
    }
    else if (currentMode == MODE_SCAN_CHANNEL && xSemaphore != NULL) {
        vSemaphoreDelete(xSemaphore);
        xSemaphore = NULL;
    }
}

void startMonitorAPMode() {
    currentMode = MODE_MONITOR_AP;
    Serial.println("Initializing Monitor AP Mode");
    
    // Create mutex and initialize variables
    deviceDataMutex = xSemaphoreCreateMutex();
    if (deviceDataMutex == NULL) {
        Serial.println("Failed to create deviceDataMutex");
        return;
    }

    g_esp_boot_time_ms = millis();
    g_intruder_detection_armed = false;
    g_intruder_alert_active = false;
    
    // Initialize device tracking array
    for (int i = 0; i < MAX_CLIENT_DEVICES; i++) {
        trackedClientDevices[i].isActive = false;
        trackedClientDevices[i].activityScoreToAP = 0;
        trackedClientDevices[i].activityScoreFromAP = 0;
        memset(trackedClientDevices[i].mac, 0, 6);
    }
    
    // Setup WiFi for Monitor AP mode
    esp_err_t err;
    err = esp_wifi_set_promiscuous(true);
    if (err != ESP_OK) {
        Serial.printf("Failed to set promiscuous mode: %s\n", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        Serial.printf("Failed to set channel: %s\n", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_packet_handler);
    if (err != ESP_OK) {
        Serial.printf("Failed to set callback: %s\n", esp_err_to_name(err));
        return;
    }

    // Create the LED update task
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        led_update_task,
        "LEDUpdateTask",
        4096,
        NULL,
        2,
        &modeTaskHandle,
        tskNO_AFFINITY
    );

    if (xReturned != pdPASS) {
        Serial.println("Failed to create LED update task");
        return;
    }

    Serial.println("Monitor AP Mode Started Successfully");
    Serial.printf("Monitoring on Channel: %d\n", WIFI_CHANNEL);
}

void startScanChannelMode() {
    currentMode = MODE_SCAN_CHANNEL;
    Serial.println("Initializing Channel Scan Mode");
    
    // Create semaphore
    xSemaphore = xSemaphoreCreateMutex();
    if (xSemaphore == NULL) {
        Serial.println("Failed to create xSemaphore");
        return;
    }

    // Initialize scan variables
    currentScanningChannel = 1;
    
    for (int i = 0; i < CHANNELS; i++) {
        pendingActivityIncrements[i] = 0;
        channelActivity[i] = 0;
        channelBrightness[i] = 1.0f;
        pendingColorForDelayedLed[i] = OFF_COLOR;
        framesToDelayForLed[i] = 0;
    }
    
    // Setup WiFi for Channel Scan mode
    esp_err_t err;
    err = esp_wifi_set_promiscuous(true);
    if (err != ESP_OK) {
        Serial.printf("Failed to set promiscuous mode: %s\n", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_set_promiscuous_rx_cb(onWiFiPacket);
    if (err != ESP_OK) {
        Serial.printf("Failed to set callback: %s\n", esp_err_to_name(err));
        return;
    }

    err = esp_wifi_set_channel(currentScanningChannel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        Serial.printf("Failed to set channel: %s\n", esp_err_to_name(err));
        return;
    }

    // Create LED update task
    BaseType_t xReturned = xTaskCreatePinnedToCore(
        LEDUpdateTask,
        "LEDTask",
        4096,
        NULL,
        2,
        &modeTaskHandle,
        tskNO_AFFINITY
    );

    if (xReturned != pdPASS) {
        Serial.println("Failed to create LED update task");
        return;
    }

    // Create channel switching task
    xReturned = xTaskCreatePinnedToCore(
        ChannelTask,
        "ChannelTask",
        4096,
        NULL,
        1,
        &channelTaskHandle,
        tskNO_AFFINITY
    );

    if (xReturned != pdPASS) {
        Serial.println("Failed to create channel task");
        vTaskDelete(modeTaskHandle);
        modeTaskHandle = NULL;
        return;
    }

    Serial.println("Channel Scan Mode Started Successfully");
}

void loop() {
    // Main loop is not used as we're using FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}
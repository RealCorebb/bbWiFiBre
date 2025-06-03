#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define common constants
#define LED_PIN 10
#define NUM_LEDS 26

// Mode definitions
#define MODE_MONITOR_AP 1
#define MODE_SCAN_CHANNEL 2

// Global variables
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
volatile int currentMode = MODE_MONITOR_AP;
TaskHandle_t modeTaskHandle = NULL;

// Function declarations
void startMonitorAPMode();
void startScanChannelMode();
void stopCurrentMode();

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    
    strip.begin();
    strip.setBrightness(80);
    strip.show();

    // Initialize WiFi in station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Initial mode setup
    startMonitorAPMode();

    // Create a task for Serial command monitoring
    xTaskCreate(
        serialCommandTask,
        "SerialCommand",
        4096,
        NULL,
        1,
        NULL
    );
}

void serialCommandTask(void* parameter) {
    while (true) {
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            command.trim();
            
            if (command == "mode1") {
                if (currentMode != MODE_MONITOR_AP) {
                    stopCurrentMode();
                    startMonitorAPMode();
                }
            }
            else if (command == "mode2") {
                if (currentMode != MODE_SCAN_CHANNEL) {
                    stopCurrentMode();
                    startScanChannelMode();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void stopCurrentMode() {
    // Stop WiFi promiscuous mode
    esp_wifi_set_promiscuous(false);
    
    // Delete current mode task if exists
    if (modeTaskHandle != NULL) {
        vTaskDelete(modeTaskHandle);
        modeTaskHandle = NULL;
    }
    
    // Clear all LEDs
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, 0);
    }
    strip.show();
    
    // Delete all existing tasks except the serial command task
    // Note: FreeRTOS will not delete the current task or system tasks
    vTaskDelay(pdMS_TO_TICKS(100)); // Allow time for cleanup
}

void startMonitorAPMode() {
    currentMode = MODE_MONITOR_AP;
    Serial.println("Starting Monitor AP Mode...");
    
    // Create mutex and initialize variables specific to Monitor AP mode
    deviceDataMutex = xSemaphoreCreateMutex();
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
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_packet_handler);
    
    // Create the LED update task for Monitor AP mode
    xTaskCreatePinnedToCore(
        led_update_task,
        "LEDUpdateTask",
        4096,
        NULL,
        2,
        &modeTaskHandle,
        tskNO_AFFINITY
    );
}

void startScanChannelMode() {
    currentMode = MODE_SCAN_CHANNEL;
    Serial.println("Starting Channel Scan Mode...");
    
    // Create semaphore and initialize variables specific to Scan Channel mode
    xSemaphore = xSemaphoreCreateMutex();
    currentScanningChannel = 1;
    
    // Initialize arrays
    memset(pendingActivityIncrements, 0, sizeof(pendingActivityIncrements));
    memset(channelActivity, 0, sizeof(channelActivity));
    for (int i = 0; i < CHANNELS; i++) {
        channelBrightness[i] = 1.0f;
        pendingColorForDelayedLed[i] = OFF_COLOR;
        framesToDelayForLed[i] = 0;
    }
    
    // Setup WiFi for Channel Scan mode
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(onWiFiPacket);
    esp_wifi_set_channel(currentScanningChannel, WIFI_SECOND_CHAN_NONE);
    
    // Create tasks for Channel Scan mode
    xTaskCreatePinnedToCore(
        LEDUpdateTask,
        "LEDTask",
        4096,
        NULL,
        2,
        &modeTaskHandle,
        tskNO_AFFINITY
    );
    
    xTaskCreatePinnedToCore(
        ChannelTask,
        "ChannelTask",
        4096,
        NULL,
        1,
        NULL,
        tskNO_AFFINITY
    );
}

void loop() {
    // Main loop is not used as we're using FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}
